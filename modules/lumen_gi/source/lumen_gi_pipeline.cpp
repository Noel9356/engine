// lumen_gi frame pipeline: plans and runs one amortized GI frame.
//
//   plan capture -> capture -> finalize captured cards ->
//   plan direct/indirect/probes -> direct -> (indirect | probes) -> commit
//
// Planning is single threaded; each stage fans out over the scheduler with
// dynamic chunking (parallelRange). The indirect stage writes into staging
// buffers and the commit step publishes them afterwards, so the indirect and
// probe tasks of one frame only ever read the committed cache.

#include "lumen_gi_internal.hpp"

#include <algorithm>
#include <cmath>

namespace engine
{
namespace lumen_gi
{

namespace
{

constexpr std::uint32_t captureUnitTexels  = 1024;
constexpr std::uint32_t lightingUnitTexels = 2048;
// Indirect refreshes needed for the cache to settle (history ramp + bounces).
constexpr std::uint32_t indirectSettlePasses = 12;

// Emits work units covering `budget` texels of the cards in priority order,
// continuing each card at its rotating cursor (wrapping), so a card larger
// than the per-frame budget still gets every texel refreshed in turn.
void planCursorWork(GiPipelineState &state, std::vector<GiCardOrderKey> &keys, std::uint64_t budget,
    std::uint32_t GiCard::*cursorMember, std::vector<GiTexelWork> &out)
{
    std::sort(keys.begin(), keys.end(),
        [](const GiCardOrderKey &left, const GiCardOrderKey &right)
        {
            if (left.ageFrames != right.ageFrames)
            {
                return left.ageFrames > right.ageFrames;
            }
            return left.cameraDistance < right.cameraDistance;
        });
    for (const GiCardOrderKey &key : keys)
    {
        if (budget == 0)
        {
            break;
        }
        GiCard &            card  = state.card(key.index);
        const std::uint32_t total = static_cast<std::uint32_t>(card.totalTexels());
        const std::uint32_t take  = static_cast<std::uint32_t>(std::min<std::uint64_t>(total, budget));
        std::uint32_t       begin = card.*cursorMember % total;
        std::uint32_t       left  = take;
        while (left > 0)
        {
            const std::uint32_t run = std::min(left, total - begin);
            for (std::uint32_t unitBegin = begin; unitBegin < begin + run; unitBegin += lightingUnitTexels)
            {
                out.push_back(GiTexelWork{key.index, unitBegin, std::min(unitBegin + lightingUnitTexels, begin + run)});
            }
            left -= run;
            begin = (begin + run) % total;
        }
        card.*cursorMember = begin;
        budget -= take;
    }
}

} // namespace

bool runGiFrame(GiPipelineState &state, TaskScheduler *scheduler)
{
    try
    {
        ensureSceneTlas(state);
        const GiSettings &settings = state.settings;

        // Camera distance priorities (per surface center).
        for (GiSurface &surface : state.surfaces)
        {
            const float distance = glm::distance(0.5f * (surface.worldMin + surface.worldMax), state.cameraPosition);
            for (GiCard &card : surface.cards)
            {
                card.cameraDistance = distance;
            }
        }
        snapProbeLevels(state, state.cameraPosition);

        // ---- capture (incremental: only cards that are not captured yet) ----
        std::vector<std::uint32_t> &pendingCards = state.scratchPendingCards;
        pendingCards.clear();
        std::uint64_t              pendingTexels = 0;
        for (std::uint32_t index = 0; index < state.cardRefs.size(); ++index)
        {
            const GiCard &card = state.card(index);
            if (!card.captureComplete)
            {
                pendingCards.push_back(index);
                pendingTexels += card.planeTexels() - card.captureCursor;
            }
        }
        std::sort(pendingCards.begin(), pendingCards.end(),
            [&](std::uint32_t left, std::uint32_t right)
            { return state.card(left).cameraDistance < state.card(right).cameraDistance; });
        std::vector<GiTexelWork>   &captureWork = state.scratchCaptureWork;
        std::vector<std::uint32_t> &finishing    = state.scratchFinishingCards;
        captureWork.clear();
        finishing.clear();
        std::uint64_t              captureBudget = settings.captureTexelsPerFrame;
        for (const std::uint32_t index : pendingCards)
        {
            if (captureBudget == 0)
            {
                break;
            }
            GiCard &            card  = state.card(index);
            const std::uint32_t plane = static_cast<std::uint32_t>(card.planeTexels());
            const std::uint32_t take  = static_cast<std::uint32_t>(
                std::min<std::uint64_t>(plane - card.captureCursor, captureBudget));
            for (std::uint32_t begin = card.captureCursor; begin < card.captureCursor + take; begin += captureUnitTexels)
            {
                captureWork.push_back(GiTexelWork{index, begin, std::min(begin + captureUnitTexels, card.captureCursor + take)});
            }
            card.captureCursor += take;
            captureBudget -= take;
            card.lastCaptureFrame = state.frameIndex;
            if (card.captureCursor >= plane)
            {
                finishing.push_back(index);
            }
        }
        parallelRange(scheduler, captureWork.size(), 1,
            [&](std::size_t begin, std::size_t end)
            {
                for (std::size_t item = begin; item < end; ++item)
                {
                    const GiTexelWork &unit = captureWork[item];
                    captureCardTexels(state.cardSurface(unit.cardIndex), state.card(unit.cardIndex), unit.texelBegin,
                        unit.texelEnd);
                    state.captureTexelsThisFrame.fetch_add(unit.texelEnd - unit.texelBegin, std::memory_order_relaxed);
                }
            });
        for (const std::uint32_t index : finishing)
        {
            GiCard &card = state.card(index);
            std::uint32_t valid = 0;
            for (const float depth : card.depthTexels)
            {
                valid += depth < invalidCardDepth ? 1u : 0u;
            }
            card.validTexels       = valid;
            card.captureComplete   = true;
            card.directCursor      = 0;
            card.indirectCursor    = 0;
            card.lastDirectFrame   = 0;
            card.lastIndirectFrame = 0;
            std::fill(card.indirectSamples.begin(), card.indirectSamples.end(), static_cast<std::uint8_t>(0));
        }

        // ---- plan lighting + probes --------------------------------------
        std::vector<GiCardOrderKey> &directKeys   = state.scratchDirectKeys;
        std::vector<GiCardOrderKey> &indirectKeys = state.scratchIndirectKeys;
        directKeys.clear();
        indirectKeys.clear();
        std::uint64_t             eligibleTexels = 0;
        for (std::uint32_t index = 0; index < state.cardRefs.size(); ++index)
        {
            const GiCard &card = state.card(index);
            if (!card.captureComplete || card.validTexels == 0)
            {
                continue;
            }
            eligibleTexels += card.totalTexels();
            directKeys.push_back(GiCardOrderKey{state.frameIndex - card.lastDirectFrame, card.cameraDistance, index});
            indirectKeys.push_back(GiCardOrderKey{state.frameIndex - card.lastIndirectFrame, card.cameraDistance, index});
        }
        std::vector<GiTexelWork> &directWork   = state.scratchDirectWork;
        std::vector<GiTexelWork> &indirectWork = state.scratchIndirectWork;
        directWork.clear();
        indirectWork.clear();
        planCursorWork(state, directKeys, settings.directTexelsPerFrame, &GiCard::directCursor, directWork);
        planCursorWork(state, indirectKeys, settings.indirectTexelsPerFrame, &GiCard::indirectCursor, indirectWork);
        for (const GiTexelWork &unit : directWork)
        {
            state.card(unit.cardIndex).lastDirectFrame = state.frameIndex;
        }
        std::vector<std::size_t> &scratchOffsets = state.scratchOffsets;
        scratchOffsets.resize(indirectWork.size());
        std::size_t              scratchTotal = 0;
        for (std::size_t index = 0; index < indirectWork.size(); ++index)
        {
            scratchOffsets[index] = scratchTotal;
            scratchTotal += indirectWork[index].texelEnd - indirectWork[index].texelBegin;
            state.card(indirectWork[index].cardIndex).lastIndirectFrame = state.frameIndex;
        }
        // Gather output staging: every element is written before the commit
        // step reads it, so the reused buffer needs no re-initialization.
        state.scratchRadiance.resize(scratchTotal);
        glm::vec3 *const scratch = state.scratchRadiance.data();

        std::vector<GiProbeItem> &probeItems = state.scratchProbeItems;
        probeItems.clear();
        const std::uint32_t    probeBudget = std::max(settings.probeUpdatesPerFrame, 1u);
        std::uint32_t          largestLevel = 0;
        for (std::uint32_t levelIndex = 0; levelIndex < 2; ++levelIndex)
        {
            GiProbeLevel       &level = levelIndex == 0 ? state.level0 : state.level1;
            const std::uint32_t total = static_cast<std::uint32_t>(level.probes.size());
            largestLevel              = std::max(largestLevel, total);
            if (total == 0)
            {
                continue;
            }
            std::vector<std::uint8_t> &taken = state.scratchTaken;
            taken.assign(total, 0u);
            std::uint32_t             count = 0;
            // Probes that scrolled into the box (no data yet) come first.
            for (std::uint32_t slot = 0; slot < total && count < probeBudget; ++slot)
            {
                if (level.probes[slot].fresh)
                {
                    taken[slot] = 1u;
                    probeItems.push_back(GiProbeItem{levelIndex, slot});
                    ++count;
                }
            }
            // Then continue the round-robin refresh with the remaining budget.
            for (std::uint32_t step = 0; step < total && count < probeBudget; ++step)
            {
                const std::uint32_t slot = (level.updateCursor + step) % total;
                if (taken[slot] == 0u)
                {
                    taken[slot] = 1u;
                    probeItems.push_back(GiProbeItem{levelIndex, slot});
                    ++count;
                }
                level.updateCursor = (slot + 1) % total;
            }
        }

        // ---- run ------------------------------------------------------------
        parallelRange(scheduler, directWork.size(), 1,
            [&](std::size_t begin, std::size_t end)
            {
                for (std::size_t item = begin; item < end; ++item)
                {
                    gatherDirectLightingUnit(state, directWork[item]);
                }
            });
        const std::size_t indirectCount = indirectWork.size();
        parallelRange(scheduler, indirectCount + probeItems.size(), 1,
            [&](std::size_t begin, std::size_t end)
            {
                for (std::size_t item = begin; item < end; ++item)
                {
                    if (item < indirectCount)
                    {
                        gatherIndirectLightingUnit(state, indirectWork[item], scratch + scratchOffsets[item]);
                    }
                    else
                    {
                        const GiProbeItem &probe = probeItems[item - indirectCount];
                        updateProbe(state, probe.level, probe.slot);
                    }
                }
            });
        for (std::size_t index = 0; index < indirectWork.size(); ++index)
        {
            commitIndirectUnit(state, indirectWork[index], scratch + scratchOffsets[index]);
        }

        // ---- convergence estimate for the static-frame skip --------------
        const auto cycles = [](std::uint64_t amount, std::uint64_t budget)
        { return static_cast<std::uint64_t>((amount + std::max<std::uint64_t>(budget, 1) - 1) / std::max<std::uint64_t>(budget, 1)); };
        const std::uint64_t raysBudget = std::max(settings.probeRaysPerUpdate, 1u);
        const std::uint64_t stride     = (state.probeDirections.size() + raysBudget - 1) / raysBudget;
        std::uint64_t needed = (cycles(largestLevel, probeBudget) + 1) * (stride + 1);
        needed = std::max(needed, cycles(eligibleTexels, settings.directTexelsPerFrame) + 1);
        needed = std::max(needed, cycles(eligibleTexels, settings.indirectTexelsPerFrame) * indirectSettlePasses);
        needed = std::max(needed, cycles(pendingTexels, settings.captureTexelsPerFrame) + 1);
        state.convergenceFrames = static_cast<std::uint32_t>(std::min<std::uint64_t>(needed + 2, 4096));

        state.totalRaysTraced.fetch_add(
            static_cast<std::uint64_t>(state.probeTracesThisFrame.load(std::memory_order_relaxed))
                + state.indirectRaysThisFrame.load(std::memory_order_relaxed)
                + state.gatherRaysThisFrame.load(std::memory_order_relaxed),
            std::memory_order_relaxed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace lumen_gi
} // namespace engine
