// Version-gated frame input feeds: the per-renderer state that pulls the pbr
// environment, the direct_lighting light set, the radiance_cache probe grid and
// gather field, and the reflection_field field into a backend when their
// providers report new versions.
//
// Each provider is polled with a zero-capacity copy (a cheap version probe);
// only a changed version pays for the snapshot copy, which the backend
// borrows for the duration of one set* call. Snapshots may lag the latest
// evaluate() by one frame: module update order does not guarantee the
// providers run before the renderer, and the lighting caches are temporal by
// design. A renderer keeps one FrameInputs for its lifetime; the scratch
// buffers persist between frames so steady-state pushes allocate nothing.

#pragma once

#include "render_shared.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/reflection_field/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace engine
{
namespace renderer
{

class RenderBackend;

class FrameInputs
{
public:
    // The pbr environment: pushed when the capability's version counter
    // changes (a null environmentVersion hook degrades to pushing every
    // frame; a missing capability pushes the built-in default once).
    void pushEnvironment(RenderBackend &backend, const pbr::PbrCapability *pbr);

    // The direct_lighting light set: pushed when its version changes, capped
    // at lightLimit (the per-renderer budget negotiated from the module's
    // device profile at renderer creation). A missing capability clears the
    // lights once.
    void pushDirectLights(RenderBackend &backend, const direct_lighting::DirectLightingCapability *directLighting,
        std::uint32_t lightLimit);

    // The radiance_cache probe grid: pushed when its version changes and clamped
    // to maxGiProbes (the bound the Vulkan storage buffer is sized for). A
    // disabled hook or missing capability pushes an empty grid once, which
    // keeps the legacy hemispheric ambient.
    void pushProbeGrid(RenderBackend &backend, const radiance_cache::RadianceCacheCapability *radianceCache, bool enabled);

    // The reflection_field screen-space field: pushed when its version or
    // dimensions change. A disabled hook or missing capability pushes an
    // empty field once, which keeps the pre-hook specular.
    void pushReflectionField(RenderBackend &backend, const reflection_field::ReflectionFieldCapability *reflectionField,
        bool enabled);

    // The radiance_cache screen-space gather field: pushed when its version or
    // dimensions change. A disabled hook or missing capability pushes an
    // empty field once, which keeps the probe-walk ambient.
    void pushGatherField(RenderBackend &backend, const radiance_cache::RadianceGatherCapability *gather, bool enabled);

private:
    // Environment.
    std::uint64_t environmentVersion    = 0;
    bool          environmentInitialized = false;

    // Direct lights.
    std::uint64_t directLightVersion      = 0;
    bool          directLightsInitialized = false;
    std::array<direct_lighting::DirectLight, direct_lighting::maxDirectLights> directLightScratch{};

    // radiance_cache probe grid. probeGridLastEnabled tracks the disable
    // transition: a feed that went quiet after it was live must clear the
    // backend's last snapshot once (a stale grid would otherwise keep
    // lighting every later frame as if the provider still published).
    std::uint64_t                  probeGridVersion      = 0;
    bool                           probeGridInitialized  = false;
    bool                           probeGridLastEnabled = false;
    std::vector<radiance_cache::GiProbe> probeScratch{};
    radiance_cache::GiProbeHeader        probeHeader{};

    // reflection_field field. reflectionFieldLastEnabled tracks the disable
    // transition for the same reason as the probe grid's flag.
    std::uint64_t                             reflectionFieldVersion      = 0;
    bool                                      reflectionFieldInitialized  = false;
    bool                                      reflectionFieldLastEnabled = false;
    std::vector<float>                        reflectionScratch{};
    reflection_field::ReflectionFieldHeader   reflectionHeader{};

    // radiance_cache gather field. gatherFieldLastEnabled tracks the disable
    // transition for the same reason as the probe grid's flag.
    std::uint64_t                          gatherFieldVersion      = 0;
    bool                                   gatherFieldInitialized  = false;
    bool                                   gatherFieldLastEnabled = false;
    std::vector<float>                     gatherScratch{};
    radiance_cache::RadianceGatherHeader   gatherHeader{};
};

} // namespace renderer
} // namespace engine
