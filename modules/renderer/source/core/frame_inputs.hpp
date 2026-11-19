// Version-gated frame input feeds: the per-renderer state that pulls the pbr
// environment, the direct_lighting light set, the lumen_gi probe grid and
// the lumen_reflection field into a backend when their providers report new
// versions.
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
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
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

    // The lumen_gi probe grid: pushed when its version changes and clamped
    // to maxGiProbes (the bound the Vulkan storage buffer is sized for). A
    // disabled hook or missing capability pushes an empty grid once, which
    // keeps the legacy hemispheric ambient.
    void pushProbeGrid(RenderBackend &backend, const lumen_gi::LumenGiCapability *lumenGi, bool enabled);

    // The lumen_reflection screen-space field: pushed when its version or
    // dimensions change. A disabled hook or missing capability pushes an
    // empty field once, which keeps the pre-hook specular.
    void pushReflectionField(RenderBackend &backend, const lumen_reflection::LumenReflectionCapability *lumenReflection,
        bool enabled);

private:
    // Environment.
    std::uint64_t environmentVersion    = 0;
    bool          environmentInitialized = false;

    // Direct lights.
    std::uint64_t directLightVersion      = 0;
    bool          directLightsInitialized = false;
    std::array<direct_lighting::DirectLight, direct_lighting::maxDirectLights> directLightScratch{};

    // lumen_gi probe grid.
    std::uint64_t                  probeGridVersion     = 0;
    bool                           probeGridInitialized = false;
    std::vector<lumen_gi::GiProbe> probeScratch{};
    lumen_gi::GiProbeHeader        probeHeader{};

    // lumen_reflection field.
    std::uint64_t                             reflectionFieldVersion     = 0;
    bool                                      reflectionFieldInitialized = false;
    std::vector<float>                        reflectionScratch{};
    lumen_reflection::ReflectionFieldHeader   reflectionHeader{};
};

} // namespace renderer
} // namespace engine
