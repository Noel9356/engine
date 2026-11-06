// render_graph module implementation.
//
// Binds the "render_graph" capability: compiles graphs (validation,
// topological order, budgeted transient aliasing), executes them on the
// kernel task scheduler, and caches compiled plans keyed by graph shape.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{
namespace render_graph
{
namespace
{

// One physical allocation that transient resources alias. size and alignment
// grow to fit every resource assigned to the slot.
struct TransientSlot
{
    std::uint32_t compatibility = 0;
    std::uint32_t lastUse       = 0;
    std::uint64_t size          = 0;
    std::uint64_t alignment     = 1;
};

// A live slot together with the schedule position where its resource is
// last used.
struct PendingSlot
{
    std::uint32_t lastUse = 0;
    std::uint32_t slot    = 0;
};

// Heap comparator keeping the earliest-expiring slot on top.
struct PendingSlotLater
{
    bool operator()(const PendingSlot &left, const PendingSlot &right) const noexcept
    {
        return left.lastUse != right.lastUse ? left.lastUse > right.lastUse : left.slot > right.slot;
    }
};

// Ranges accepted through setSettings; construct() clamps config keys to
// the same limits.
bool validSettings(const RenderGraphSettings &value)
{
    return value.quality <= render::QualityPreset::Ultra && value.maxPasses >= 1 && value.maxPasses <= 4096
        && value.maxResources >= 1 && value.maxResources <= 8192
        && value.transientMemoryBudgetBytes <= (16ull * 1024ull * 1024ull * 1024ull);
}

// Small LRU cache of compiled plans keyed by structural equality of the
// graph (values, not pointers). Mutex-protected; allocation failures
// degrade to "no caching" instead of throwing.
class RenderGraphPlanCache final
{
public:
    // Entry capacity is hard-capped at 16; zero disables caching.
    void configure(std::size_t entryCapacity, std::size_t maximumGraphElements) noexcept
    {
        std::lock_guard lock(mutex_);
        entries_.clear();
        capacity_             = std::min<std::size_t>(entryCapacity, 16u);
        maximumGraphElements_ = maximumGraphElements;
        try
        {
            entries_.reserve(capacity_);
        }
        catch (...)
        {
            capacity_ = 0;
        }
    }

    void clear() noexcept
    {
        std::lock_guard lock(mutex_);
        entries_.clear();
    }

    [[nodiscard]] bool enabled() const noexcept
    {
        return capacity_ != 0 && maximumGraphElements_ != 0;
    }

    std::shared_ptr<const RenderGraphPlan> find(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        bool includeResourceLifetimes) const noexcept
    {
        if (capacity_ == 0 || (passCount != 0 && passes == nullptr) || (resourceCount != 0 && resources == nullptr))
            return {};
        std::lock_guard lock(mutex_);
        for (Entry &entry : entries_)
        {
            if (!matches(entry, passes, passCount, resources, resourceCount, includeResourceLifetimes))
                continue;
            entry.lastUse = ++clock_;
            return entry.plan;
        }
        return {};
    }

    // Copies the graph shape into the entry so later lookups can compare
    // without the caller's arrays. Returns the stored (or matching) plan.
    std::shared_ptr<const RenderGraphPlan> store(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, bool includeResourceLifetimes,
        std::size_t graphElementCount, RenderGraphPlan &plan) const noexcept
    {
        if (capacity_ == 0 || graphElementCount > maximumGraphElements_ || (passCount != 0 && passes == nullptr)
            || (resourceCount != 0 && resources == nullptr))
            return {};

        Entry entry;
        try
        {
            entry.includeResourceLifetimes = includeResourceLifetimes;
            entry.passShapes.reserve(passCount);
            if (resourceCount != 0)
                entry.resources.assign(resources, resources + resourceCount);
            for (std::uint32_t pass = 0; pass < passCount; ++pass)
            {
                const RenderGraphPassDescription &description = passes[pass];
                if ((description.useCount != 0 && description.uses == nullptr)
                    || (description.dependencyCount != 0 && description.dependencies == nullptr))
                    return {};
                PassShape shape;
                shape.useOffset        = entry.uses.size();
                shape.useCount         = description.useCount;
                shape.dependencyOffset = entry.dependencies.size();
                shape.dependencyCount  = description.dependencyCount;
                entry.passShapes.push_back(shape);
                if (description.useCount != 0)
                    entry.uses.insert(entry.uses.end(), description.uses, description.uses + description.useCount);
                if (description.dependencyCount != 0)
                    entry.dependencies.insert(entry.dependencies.end(),
                        description.dependencies,
                        description.dependencies + description.dependencyCount);
            }
            entry.plan = std::make_shared<RenderGraphPlan>(std::move(plan));
        }
        catch (...)
        {
            return {};
        }

        std::lock_guard lock(mutex_);
        for (Entry &existing : entries_)
        {
            if (matches(existing, passes, passCount, resources, resourceCount, includeResourceLifetimes))
            {
                existing.lastUse = ++clock_;
                return existing.plan;
            }
        }
        entry.lastUse = ++clock_;
        if (entries_.size() >= capacity_)
        {
            const auto oldest = std::min_element(entries_.begin(),
                entries_.end(),
                [](const Entry &left, const Entry &right) { return left.lastUse < right.lastUse; });
            if (oldest != entries_.end())
                entries_.erase(oldest);
        }
        const std::shared_ptr<const RenderGraphPlan> result = entry.plan;
        try
        {
            entries_.push_back(std::move(entry));
        }
        catch (...)
        {
            return result;
        }
        return result;
    }

private:
    struct PassShape
    {
        std::size_t   useOffset        = 0;
        std::size_t   dependencyOffset = 0;
        std::uint32_t useCount         = 0;
        std::uint32_t dependencyCount  = 0;
    };

    struct Entry
    {
        bool                                        includeResourceLifetimes = true;
        std::uint64_t                               lastUse                  = 0;
        std::vector<PassShape>                      passShapes;
        std::vector<RenderGraphResourceDescription> resources;
        std::vector<RenderGraphResourceUse>         uses;
        std::vector<std::uint32_t>                  dependencies;
        std::shared_ptr<const RenderGraphPlan>      plan;
    };

    static bool sameResource(
        const RenderGraphResourceDescription &left, const RenderGraphResourceDescription &right) noexcept
    {
        return left.sizeBytes == right.sizeBytes && left.alignmentBytes == right.alignmentBytes
            && left.compatibilityClass == right.compatibilityClass && left.transient == right.transient
            && left.initiallyDefined == right.initiallyDefined;
    }

    static bool sameUse(const RenderGraphResourceUse &left, const RenderGraphResourceUse &right) noexcept
    {
        return left.resourceIndex == right.resourceIndex && left.access == right.access;
    }

    // Full structural comparison: a hit must reproduce the same plan.
    static bool matches(const Entry &entry, const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        bool includeResourceLifetimes) noexcept
    {
        if (entry.includeResourceLifetimes != includeResourceLifetimes || entry.passShapes.size() != passCount
            || entry.resources.size() != resourceCount || (passCount != 0 && passes == nullptr)
            || (resourceCount != 0 && resources == nullptr))
            return false;
        for (std::uint32_t index = 0; index < resourceCount; ++index)
            if (!sameResource(entry.resources[index], resources[index]))
                return false;
        for (std::uint32_t index = 0; index < passCount; ++index)
        {
            const RenderGraphPassDescription &description = passes[index];
            const PassShape                  &shape       = entry.passShapes[index];
            if (description.useCount != shape.useCount || description.dependencyCount != shape.dependencyCount
                || (description.useCount != 0 && description.uses == nullptr)
                || (description.dependencyCount != 0 && description.dependencies == nullptr))
                return false;
            for (std::uint32_t use = 0; use < shape.useCount; ++use)
                if (!sameUse(entry.uses[shape.useOffset + use], description.uses[use]))
                    return false;
            for (std::uint32_t dependency = 0; dependency < shape.dependencyCount; ++dependency)
                if (entry.dependencies[shape.dependencyOffset + dependency] != description.dependencies[dependency])
                    return false;
        }
        return true;
    }

    mutable std::mutex         mutex_;
    mutable std::vector<Entry> entries_;
    mutable std::uint64_t      clock_                = 0;
    std::size_t                capacity_             = 0;
    std::size_t                maximumGraphElements_ = 0;
};

// Module singleton backing RenderGraphCapability. Stateless beyond the
// settings, the plan cache, and kernel-provided scheduler/profiler pointers.
class RenderGraphModule final : public ModuleInterface
{
public:
    RenderGraphModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads [render_graph] with per-key clamping to the valid ranges; absent
    // or invalid keys keep the defaults (unknown quality strings keep Auto).
    // Also configures the plan cache.
    void construct(ParticipantContext &context) override
    {
        scheduler_ = &context.taskScheduler();
        settings_  = RenderGraphSettings{};
        if (const auto value = context.configurationStringInSection("render_graph", "quality"); value.has_value())
        {
            if (*value == "low")
                settings_.quality = render::QualityPreset::Low;
            else if (*value == "medium")
                settings_.quality = render::QualityPreset::Medium;
            else if (*value == "high")
                settings_.quality = render::QualityPreset::High;
            else if (*value == "ultra")
                settings_.quality = render::QualityPreset::Ultra;
        }
        if (const auto value = context.configurationIntegerInSection("render_graph", "max_passes");
            value.has_value() && *value > 0)
            settings_.maxPasses = static_cast<std::uint32_t>(std::min<std::int64_t>(*value, 4096));
        if (const auto value = context.configurationIntegerInSection("render_graph", "max_resources");
            value.has_value() && *value > 0)
            settings_.maxResources = static_cast<std::uint32_t>(std::min<std::int64_t>(*value, 8192));
        if (const auto value = context.configurationIntegerInSection("render_graph", "transient_budget_mb");
            value.has_value() && *value >= 0)
            settings_.transientMemoryBudgetBytes
                = static_cast<std::uint64_t>(std::min<std::int64_t>(*value, 16384)) * 1024ull * 1024ull;
        settings_.transientAliasing
            = context.configurationBooleanInSection("render_graph", "transient_aliasing").value_or(true);
        // Compiling a plan is pure CPU work with the same result for a stable
        // graph shape; caching it removes that per-frame cost. Default 8
        // entries (set plan_cache_entries = 0 to restore the old behaviour).
        std::size_t planCacheEntries = 8;
        if (const auto value = context.configurationIntegerInSection("render_graph", "plan_cache_entries");
            value && *value >= 0 && *value <= 16)
            planCacheEntries = static_cast<std::size_t>(*value);
        std::size_t planCacheMaxElements = 16384;
        if (const auto value = context.configurationIntegerInSection("render_graph", "plan_cache_max_elements");
            value && *value >= 0 && *value <= 1'000'000)
            planCacheMaxElements = static_cast<std::size_t>(*value);
        planCache_.configure(planCacheEntries, planCacheMaxElements);
        profiler_ = &context.profiler();
    }

    void start() override
    {}
    void stop() override
    {}

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == RenderGraphCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "render_graph",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"render_graph"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self     = this;
        capability_.settings = [](void *self) -> const RenderGraphSettings *
        { return &static_cast<RenderGraphModule *>(self)->settings_; };
        capability_.setSettings = [](void *self, const RenderGraphSettings *value)
        { return static_cast<RenderGraphModule *>(self)->setSettingsImpl(value); };
        capability_.resolveProfile = [](void *self, const render::DeviceProfile *device, RenderGraphProfile *out)
        { return static_cast<RenderGraphModule *>(self)->resolveProfileImpl(device, out); };
        capability_.compile = [](void                                  *self,
                                  const RenderGraphPassDescription     *passes,
                                  std::uint32_t                         passCount,
                                  const RenderGraphResourceDescription *resources,
                                  std::uint32_t                         resourceCount,
                                  std::uint32_t                        *order,
                                  std::uint32_t                         orderCapacity,
                                  std::uint32_t                        *slots,
                                  std::uint32_t                         slotCapacity,
                                  RenderGraphCompileResult             *result,
                                  RenderGraphError                     *error)
        {
            return static_cast<RenderGraphModule *>(self)->compileImpl(
                passes, passCount, resources, resourceCount, order, orderCapacity, slots, slotCapacity, result, error);
        };
        capability_.execute = [](void                                  *self,
                                  const RenderGraphPassDescription     *passes,
                                  std::uint32_t                         passCount,
                                  const RenderGraphResourceDescription *resources,
                                  std::uint32_t                         resourceCount,
                                  RenderGraphExecuteCallback            callback,
                                  void                                 *userData,
                                  RenderGraphError                     *error)
        {
            return static_cast<RenderGraphModule *>(self)->executeImpl(
                passes, passCount, resources, resourceCount, callback, userData, error);
        };
    }

    bool setSettingsImpl(const RenderGraphSettings *value)
    {
        if (value == nullptr || !validSettings(*value))
            return false;
        settings_ = *value;
        planCache_.clear();
        return true;
    }

    // Auto quality derives a memory scale from the device class (non-Vulkan
    // gets 0.125); explicit presets scale by 0.25x/0.5x/1x/2x. Scaled limits
    // are clamped back to the configured settings; asyncCompute stays off.
    bool resolveProfileImpl(const render::DeviceProfile *device, RenderGraphProfile *out) const
    {
        if (device == nullptr || out == nullptr)
            return false;
        float memoryScale = 0.125f;
        if (device->isVulkan)
        {
            if (device->deviceClass == render::DeviceClass::DiscreteGpu)
                memoryScale = 1.0f;
            else if (device->deviceClass == render::DeviceClass::IntegratedGpu)
                memoryScale = 0.5f;
            else
                memoryScale = 0.25f;
            if (device->deviceLocalMemoryBytes != 0)
                memoryScale = std::min(memoryScale,
                    static_cast<float>(device->deviceLocalMemoryBytes) / (4.0f * 1024.0f * 1024.0f * 1024.0f));
        }
        float qualityScale = 1.0f;
        switch (settings_.quality)
        {
        case render::QualityPreset::Low:
            qualityScale = 0.25f;
            break;
        case render::QualityPreset::Medium:
            qualityScale = 0.5f;
            break;
        case render::QualityPreset::High:
            qualityScale = 1.0f;
            break;
        case render::QualityPreset::Ultra:
            qualityScale = 2.0f;
            break;
        case render::QualityPreset::Auto:
            break;
        }
        const float scale = settings_.quality == render::QualityPreset::Auto ? memoryScale : qualityScale;
        out->maxPasses
            = std::min(settings_.maxPasses, std::max(1u, static_cast<std::uint32_t>(settings_.maxPasses * scale)));
        out->maxResources = std::min(
            settings_.maxResources, std::max(1u, static_cast<std::uint32_t>(settings_.maxResources * scale)));
        const std::uint64_t budget      = static_cast<std::uint64_t>(settings_.transientMemoryBudgetBytes * scale);
        out->transientMemoryBudgetBytes = std::min(budget, settings_.transientMemoryBudgetBytes);
        out->transientAliasing          = settings_.transientAliasing;
        out->asyncCompute               = false;
        return true;
    }

    // Cache-aware plan lookup: a hit returns the cached plan (kept alive by
    // cachedPlan), a miss builds into localPlan and stores it. outPlan
    // points at whichever is used.
    bool preparePlan(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, bool includeResourceLifetimes,
        RenderGraphPlan &localPlan, std::shared_ptr<const RenderGraphPlan> &cachedPlan, const RenderGraphPlan *&outPlan,
        RenderGraphError &outError) const
    {
        if (!planCache_.enabled())
        {
            if (!buildRenderGraphPlan(
                    passes, passCount, resources, resourceCount, localPlan, outError, includeResourceLifetimes))
                return false;
            outPlan = &localPlan;
            return true;
        }

        cachedPlan = planCache_.find(passes, passCount, resources, resourceCount, includeResourceLifetimes);
        if (cachedPlan)
        {
            outPlan  = cachedPlan.get();
            outError = RenderGraphError::None;
            return true;
        }
        if (!buildRenderGraphPlan(
                passes, passCount, resources, resourceCount, localPlan, outError, includeResourceLifetimes))
            return false;

        std::size_t graphElementCount = static_cast<std::size_t>(passCount) + resourceCount;
        for (std::uint32_t pass = 0; pass < passCount; ++pass)
        {
            const std::size_t passElements
                = static_cast<std::size_t>(passes[pass].useCount) + passes[pass].dependencyCount;
            if (passElements > std::numeric_limits<std::size_t>::max() - graphElementCount)
            {
                graphElementCount = std::numeric_limits<std::size_t>::max();
                break;
            }
            graphElementCount += passElements;
        }
        cachedPlan = planCache_.store(
            passes, passCount, resources, resourceCount, includeResourceLifetimes, graphElementCount, localPlan);
        outPlan = cachedPlan ? cachedPlan.get() : &localPlan;
        return true;
    }

    // Validates capacities and pointers, obtains a plan with lifetimes, then
    // assigns transient resources to physical slots (loop below). Outputs:
    // pass order, per-resource slot index, and the compile statistics.
    bool compileImpl(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, std::uint32_t *outOrder,
        std::uint32_t orderCapacity, std::uint32_t *outSlots, std::uint32_t slotCapacity,
        RenderGraphCompileResult *outResult, RenderGraphError *outError) const
    {
        ScopedProfileTimer profileTimer(*profiler_, PerformanceMetric::RenderGraphCompile);
        const auto         fail = [outError](RenderGraphError error)
        {
            if (outError != nullptr)
                *outError = error;
            return false;
        };
        if (passCount > settings_.maxPasses || resourceCount > settings_.maxResources || orderCapacity < passCount
            || slotCapacity < resourceCount)
            return fail(RenderGraphError::CapacityExceeded);
        if ((passCount != 0 && (passes == nullptr || outOrder == nullptr))
            || (resourceCount != 0 && (resources == nullptr || outSlots == nullptr)) || outResult == nullptr)
            return fail(RenderGraphError::InvalidInput);

        RenderGraphPlan                        localPlan;
        std::shared_ptr<const RenderGraphPlan> cachedPlan;
        const RenderGraphPlan                 *analysis      = nullptr;
        RenderGraphError                       analysisError = RenderGraphError::None;
        if (!preparePlan(
                passes, passCount, resources, resourceCount, true, localPlan, cachedPlan, analysis, analysisError))
            return fail(analysisError);

        std::vector<TransientSlot>                                                     slots;
        std::vector<PendingSlot>                                                       pendingSlots;
        std::unordered_map<std::uint32_t, std::multimap<std::uint64_t, std::uint32_t>> availableSlots;
        slots.reserve(analysis->resourceOrder.size());
        pendingSlots.reserve(analysis->resourceOrder.size());
        if (resourceCount != 0)
            std::fill(outSlots, outSlots + resourceCount, invalidResourceSlot);

        // Aliasing allocator: walk transient resources in first-use order,
        // retire slots whose last use is past, then reuse a compatible free
        // slot (best fit by size) or allocate a new one, budget-checked.
        std::uint64_t allocatedBytes = 0;
        for (const std::uint32_t resource : analysis->resourceOrder)
        {
            const auto   &description = resources[resource];
            std::uint32_t selected    = invalidResourceSlot;
            if (settings_.transientAliasing)
            {
                while (!pendingSlots.empty() && pendingSlots.front().lastUse < analysis->firstUse[resource])
                {
                    std::pop_heap(pendingSlots.begin(), pendingSlots.end(), PendingSlotLater{});
                    const PendingSlot released = pendingSlots.back();
                    pendingSlots.pop_back();
                    const auto &slot = slots[released.slot];
                    availableSlots[slot.compatibility].emplace(slot.size, released.slot);
                }
                const auto found = availableSlots.find(description.compatibilityClass);
                if (found != availableSlots.end() && !found->second.empty())
                {
                    auto candidate = found->second.lower_bound(description.sizeBytes);
                    if (candidate == found->second.end())
                        candidate = std::prev(found->second.end());
                    selected = candidate->second;
                    found->second.erase(candidate);
                }
            }

            if (selected == invalidResourceSlot)
            {
                selected = static_cast<std::uint32_t>(slots.size());
                slots.push_back(TransientSlot{
                    description.compatibilityClass,
                    analysis->lastUse[resource],
                    description.sizeBytes,
                    description.alignmentBytes});
                if (description.sizeBytes > settings_.transientMemoryBudgetBytes
                                                - std::min(allocatedBytes, settings_.transientMemoryBudgetBytes))
                    return fail(RenderGraphError::BudgetExceeded);
                allocatedBytes += description.sizeBytes;
            }
            else
            {
                TransientSlot &slot = slots[selected];
                if (description.sizeBytes > slot.size)
                {
                    const std::uint64_t growth = description.sizeBytes - slot.size;
                    if (growth > settings_.transientMemoryBudgetBytes
                                     - std::min(allocatedBytes, settings_.transientMemoryBudgetBytes))
                        return fail(RenderGraphError::BudgetExceeded);
                    allocatedBytes += growth;
                    slot.size = description.sizeBytes;
                }
                slot.lastUse   = analysis->lastUse[resource];
                slot.alignment = std::max(slot.alignment, description.alignmentBytes);
            }
            outSlots[resource] = selected;
            if (settings_.transientAliasing)
            {
                pendingSlots.push_back(PendingSlot{analysis->lastUse[resource], selected});
                std::push_heap(pendingSlots.begin(), pendingSlots.end(), PendingSlotLater{});
            }
        }

        std::copy(analysis->order.begin(), analysis->order.end(), outOrder);
        *outResult = RenderGraphCompileResult{
            passCount, resourceCount, static_cast<std::uint32_t>(slots.size()), allocatedBytes};
        if (outError != nullptr)
            *outError = RenderGraphError::None;
        return true;
    }

    // Builds a TaskGraph with one node per pass, wires the plan's edges, and
    // blocks on the scheduler until all callbacks ran; failures surface as
    // ExecutionFailed. Callbacks receive the original pass index.
    bool executeImpl(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        RenderGraphExecuteCallback callback, void *userData, RenderGraphError *outError) const
    {
        ScopedProfileTimer profileTimer(*profiler_, PerformanceMetric::RenderGraphExecute);
        const auto         fail = [outError](RenderGraphError error)
        {
            if (outError != nullptr)
                *outError = error;
            return false;
        };
        if (scheduler_ == nullptr || callback == nullptr)
            return fail(RenderGraphError::InvalidInput);
        if (passCount > settings_.maxPasses || resourceCount > settings_.maxResources)
            return fail(RenderGraphError::CapacityExceeded);

        RenderGraphPlan                        localPlan;
        std::shared_ptr<const RenderGraphPlan> cachedPlan;
        const RenderGraphPlan                 *analysis      = nullptr;
        RenderGraphError                       analysisError = RenderGraphError::None;
        if (!preparePlan(
                passes, passCount, resources, resourceCount, false, localPlan, cachedPlan, analysis, analysisError))
            return fail(analysisError);

        // Cached plans reuse a prebuilt TaskGraph (executionCache_): node
        // tasks call the *current* callback through a bindings record, so a
        // graph compiled once executes every frame without per-frame node,
        // std::function or adjacency allocations. A single slot covers the
        // standard one-shape-per-frame case; concurrent or alternating use
        // falls back to the per-call graph (the historical behaviour).
        try
        {
            const bool   cacheable = planCache_.enabled() && cachedPlan != nullptr;
            TaskGraph   *graph     = nullptr;
            std::unique_lock lock(executionMutex_, std::defer_lock);
            bool claimed = false;
            if (cacheable)
            {
                lock.lock();
                if (!executionCache_.executing.exchange(true, std::memory_order_acq_rel))
                {
                    claimed = true;
                    if (executionCache_.plan.get() != analysis)
                    {
                        executionCache_.graph.reset();
                        executionCache_.nodeForPass.assign(passCount, invalidResourceSlot);
                        executionCache_.bindings         = std::make_shared<ExecutionBindings>();
                        std::vector<std::uint32_t> &nodes = executionCache_.nodeForPass;
                        for (std::uint32_t pass = 0; pass < passCount; ++pass)
                            nodes[pass] = executionCache_.graph.addTask(
                                [bindings = executionCache_.bindings, pass]
                                {
                                    const RenderGraphExecuteCallback currentCallback = bindings->callback;
                                    void                            *currentUserData = bindings->userData;
                                    if (currentCallback != nullptr)
                                        currentCallback(currentUserData, pass);
                                });
                        for (const auto &[prerequisite, dependent] : analysis->edges)
                            if (!executionCache_.graph.addDependency(nodes[prerequisite], nodes[dependent]))
                            {
                                executionCache_.release();
                                executionCache_.executing.store(false, std::memory_order_release);
                                return fail(RenderGraphError::InvalidInput);
                            }
                        executionCache_.plan = cachedPlan;
                    }
                    executionCache_.bindings->callback = callback;
                    executionCache_.bindings->userData = userData;
                    graph                              = &executionCache_.graph;
                }
            }
            if (graph == nullptr)
            {
                TaskGraph                 localGraph;
                std::vector<std::uint32_t> nodes(passCount, invalidResourceSlot);
                for (std::uint32_t pass = 0; pass < passCount; ++pass)
                    nodes[pass] = localGraph.addTask([callback, userData, pass] { callback(userData, pass); });
                for (const auto &[prerequisite, dependent] : analysis->edges)
                    if (!localGraph.addDependency(nodes[prerequisite], nodes[dependent]))
                    {
                        if (claimed)
                        {
                            lock.unlock();
                            executionCache_.release();
                            executionCache_.executing.store(false, std::memory_order_release);
                        }
                        return fail(RenderGraphError::InvalidInput);
                    }
                graph = &localGraph;
                try
                {
                    graph->execute(*scheduler_);
                }
                catch (...)
                {
                    if (claimed)
                    {
                        lock.unlock();
                        executionCache_.release();
                        executionCache_.executing.store(false, std::memory_order_release);
                    }
                    throw;
                }
            }
            else
            {
                try
                {
                    graph->execute(*scheduler_);
                }
                catch (...)
                {
                    lock.unlock();
                    executionCache_.release();
                    executionCache_.executing.store(false, std::memory_order_release);
                    throw;
                }
                // Clear the bindings so a stale callback can never run if the
                // cached graph were re-executed outside this call.
                executionCache_.bindings->callback = nullptr;
                executionCache_.bindings->userData = nullptr;
                lock.unlock();
                executionCache_.executing.store(false, std::memory_order_release);
            }
        }
        catch (...)
        {
            return fail(RenderGraphError::ExecutionFailed);
        }
        if (outError != nullptr)
            *outError = RenderGraphError::None;
        return true;
    }

    // Rebindable execution slot: one prebuilt graph per cached plan shape.
    // plan keeps the analysed plan alive so the pointer key can never be
    // recycled while the cached graph exists; bindings forward each execute
    // to the current callback. executing rejects overlapping use (falls back
    // to the per-call graph path, which is what concurrent callers did before).
    struct ExecutionBindings
    {
        RenderGraphExecuteCallback callback = nullptr;
        void                      *userData = nullptr;
    };
    struct CachedExecution
    {
        // Cannot assign the whole struct (atomic member); release() drops the
        // reusable state instead.
        void release() noexcept
        {
            plan.reset();
            bindings.reset();
            graph.reset();
            nodeForPass.clear();
        }

        std::shared_ptr<const RenderGraphPlan>   plan;
        std::shared_ptr<ExecutionBindings>       bindings;
        TaskGraph                                graph;
        std::vector<std::uint32_t>               nodeForPass;
        std::atomic<bool>                        executing{false};
    };

    // scheduler_/profiler_ are borrowed from the ParticipantContext in
    // construct(); the profiler feeds the RenderGraphCompile/Execute metrics.
    ParticipantManifest     manifest_;
    RenderGraphCapability   capability_{};
    RenderGraphSettings     settings_{};
    TaskScheduler          *scheduler_ = nullptr;
    PerformanceProfiler    *profiler_  = nullptr;
    RenderGraphPlanCache    planCache_;
    mutable std::mutex      executionMutex_;
    mutable CachedExecution executionCache_;
};

} // namespace
} // namespace render_graph
} // namespace engine

// Module entry table the kernel loads from the .so: factory plus the
// provided/required capability lists (kept in sync with module.toml).
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"render_graph"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "render_graph",
            .moduleVersion = "0.1.0",
            .createModule = []() -> engine::ModuleInterface * { return new engine::render_graph::RenderGraphModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
