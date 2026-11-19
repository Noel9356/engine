// Implementation of the version-gated frame input feeds (frame_inputs.hpp).

#include "frame_inputs.hpp"

#include "render_backend.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <new>

namespace engine
{
namespace renderer
{
namespace
{

// One-time notices for inputs a provider offered beyond what this module can
// consume; repeating the notice every frame would drown the log.
bool gNotifiedProbeOverflow   = false;
bool gNotifiedFieldOversize   = false;

// The live environment from the pbr capability, or the built-in default
// when the module has none published.
pbr::PbrEnvironment currentEnvironment(const pbr::PbrCapability &pbr)
{
    if (pbr.environment != nullptr)
    {
        const pbr::PbrEnvironment *environment = pbr.environment(pbr.self);
        if (environment != nullptr)
        {
            return *environment;
        }
    }
    return defaultEnvironment();
}

} // namespace

void FrameInputs::pushEnvironment(RenderBackend &backend, const pbr::PbrCapability *pbr)
{
    if (pbr == nullptr)
    {
        if (!environmentInitialized)
        {
            backend.setEnvironment(defaultEnvironment());
            environmentInitialized = true;
        }
        return;
    }
    if (pbr->environmentVersion != nullptr)
    {
        const std::uint64_t version = pbr->environmentVersion(pbr->self);
        if (!environmentInitialized || environmentVersion != version)
        {
            backend.setEnvironment(currentEnvironment(*pbr));
            environmentVersion     = version;
            environmentInitialized = true;
        }
        return;
    }
    backend.setEnvironment(currentEnvironment(*pbr));
}

void FrameInputs::pushDirectLights(RenderBackend &backend, const direct_lighting::DirectLightingCapability *directLighting,
    std::uint32_t lightLimit)
{
    if (directLighting != nullptr && directLighting->copyLights != nullptr)
    {
        std::uint64_t version = 0;
        directLighting->copyLights(directLighting->self, nullptr, 0, &version);
        if (!directLightsInitialized || directLightVersion != version)
        {
            const std::uint32_t count = directLighting->copyLights(
                directLighting->self, directLightScratch.data(), lightLimit, &version);
            backend.setDirectLights(directLightScratch.data(), count);
            directLightVersion      = version;
            directLightsInitialized = true;
        }
        return;
    }
    if (!directLightsInitialized)
    {
        backend.setDirectLights(nullptr, 0);
        directLightsInitialized = true;
    }
}

void FrameInputs::pushProbeGrid(RenderBackend &backend, const lumen_gi::LumenGiCapability *lumenGi, bool enabled)
{
    if (enabled && lumenGi != nullptr && lumenGi->copyProbes != nullptr)
    {
        std::uint64_t version = 0;
        lumenGi->copyProbes(lumenGi->self, nullptr, 0, &probeHeader, &version);
        if (!probeGridInitialized || probeGridVersion != version)
        {
            // The probe storage the Vulkan backend sizes is bounded by
            // maxGiProbes; a grid that grew past the bound keeps its leading
            // probes so the frame stays alive (the lattice walk treats
            // out-of-range probes as invalid and falls back to the ambient).
            if (probeHeader.probeCount > maxGiProbes)
            {
                if (!gNotifiedProbeOverflow)
                {
                    std::fprintf(stderr,
                        "renderer module: lumen_gi probe grid holds %u probes; consuming the first %u (the Vulkan "
                        "probe storage bound)\n",
                        probeHeader.probeCount,
                        maxGiProbes);
                    gNotifiedProbeOverflow = true;
                }
                probeHeader.probeCount = maxGiProbes;
            }
            try
            {
                probeScratch.resize(probeHeader.probeCount);
            }
            catch (const std::bad_alloc &)
            {
                // Keep the previous snapshot; the version stays stale so the
                // next frame retries the copy.
                return;
            }
            const std::uint32_t copied = probeHeader.probeCount != 0
                ? lumenGi->copyProbes(lumenGi->self,
                      probeScratch.data(),
                      static_cast<std::uint32_t>(probeScratch.size()),
                      &probeHeader,
                      &version)
                : 0;
            GiProbeGrid grid;
            grid.header     = probeHeader;
            grid.probes     = probeScratch.data();
            grid.probeCount = std::min(copied, probeHeader.probeCount);
            backend.setProbeGrid(grid);
            probeGridVersion     = version;
            probeGridInitialized = true;
        }
        return;
    }
    if (!probeGridInitialized)
    {
        backend.setProbeGrid(GiProbeGrid{});
        probeGridInitialized = true;
    }
}

void FrameInputs::pushReflectionField(
    RenderBackend &backend, const lumen_reflection::LumenReflectionCapability *lumenReflection, bool enabled)
{
    if (enabled && lumenReflection != nullptr && lumenReflection->copyField != nullptr)
    {
        std::uint64_t version = 0;
        lumen_reflection::ReflectionFieldHeader header{};
        lumenReflection->copyField(lumenReflection->self, nullptr, 0, &header, &version);
        if (!reflectionFieldInitialized || reflectionFieldVersion != version || reflectionHeader.width != header.width
            || reflectionHeader.height != header.height)
        {
            // copyField writes reflectionFieldTexelFloats floats per texel
            // (the radiance triplet interleaved with the texel roughness;
            // see the lumen_reflection ABI header) - the scratch sizing and
            // the copy capacity must both carry that stride.
            const std::size_t texelCount = static_cast<std::size_t>(header.width) * header.height;
            if (texelCount > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
            {
                if (!gNotifiedFieldOversize)
                {
                    std::fprintf(stderr,
                        "renderer module: lumen_reflection field of %ux%u texels exceeds the addressable bound; "
                        "keeping the previous snapshot\n",
                        header.width,
                        header.height);
                    gNotifiedFieldOversize = true;
                }
                return;
            }
            try
            {
                reflectionScratch.resize(texelCount * lumen_reflection::reflectionFieldTexelFloats);
            }
            catch (const std::bad_alloc &)
            {
                // Keep the previous snapshot; the version stays stale so the
                // next frame retries the copy.
                return;
            }
            std::uint32_t copiedTexels = 0;
            if (texelCount != 0)
            {
                copiedTexels = lumenReflection->copyField(lumenReflection->self,
                    reflectionScratch.data(),
                    static_cast<std::uint32_t>(texelCount),
                    &reflectionHeader,
                    &version);
            }
            else
            {
                reflectionHeader = header;
            }
            if (copiedTexels != texelCount || reflectionHeader.width != header.width
                || reflectionHeader.height != header.height)
            {
                // The module resized the field between the version probe and
                // this copy (copyField truncated to the probed capacity and
                // the header it wrote describes the new dimensions). Pushing
                // the mismatched pair would read past the scratch, so this
                // frame keeps the backend's previous snapshot; the recorded
                // state stays stale, which re-triggers the copy next frame at
                // the new size.
                return;
            }
            ReflectionField field;
            field.header   = reflectionHeader;
            field.radiance = reflectionScratch.data();
            backend.setReflectionField(field);
            reflectionFieldVersion     = version;
            reflectionFieldInitialized = true;
        }
        return;
    }
    if (!reflectionFieldInitialized)
    {
        backend.setReflectionField(ReflectionField{});
        reflectionFieldInitialized = true;
    }
}

} // namespace renderer
} // namespace engine
