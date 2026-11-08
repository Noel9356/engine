// Abstract backend interface behind the "renderer" capability.
//
// renderer.cpp owns the capability plumbing and resource validation; a
// RenderBackend implementation owns actual pixel output. The headless
// backend (headless_backend.hpp) and the Vulkan backend (vulkan_backend.hpp)
// are the two implementations. Calls arrive on the main thread only.

#ifndef ENGINE_RENDERER_BACKEND_HPP
#define ENGINE_RENDERER_BACKEND_HPP

#include "engine/renderer/device_profile.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <cstdint>

namespace engine
{
namespace renderer
{

// Lifecycle contract: createMesh/createTexture are safe any time after
// construction; beginFrame/draw*/endFrame must nest exactly one frame at a
// time, and draw* calls between beginFrame/endFrame. Destruction must happen
// outside a frame.
class RenderBackend
{
public:
    virtual ~RenderBackend() = default;

    virtual const char *name() const = 0;

    virtual window::WindowHandle surface() const = 0;

    virtual DeviceClass   deviceClass() const   = 0;
    virtual const char   *deviceName() const    = 0;
    virtual DeviceProfile deviceProfile() const = 0;

    virtual void setViewport(std::uint32_t width, std::uint32_t height) = 0;

    virtual void setCamera(const CameraMatrices &camera) = 0;

    // Environment/lights are pushed by renderer.cpp when the pbr and
    // direct_lighting capabilities report new versions.
    virtual void setEnvironment(const pbr::PbrEnvironment &environment)                           = 0;
    virtual void setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count) = 0;

    virtual void beginFrame(const FrameParameters &frame)                                          = 0;
    virtual void drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material) = 0;
    virtual void drawMeshInstances(
        MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount, const MeshMaterial &material) = 0;
    virtual void drawOverlay(const OverlayDrawData &data)                                                       = 0;
    virtual void endFrame()                                                                                     = 0;

    // Resource creation takes the module-assigned id and copies/queues the
    // upload; the description's memory is not referenced afterwards.
    virtual bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error) = 0;
    virtual void destroyMesh(MeshHandle mesh)                                                      = 0;

    virtual bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error) = 0;
    virtual void destroyTexture(TextureHandle texture)                                                      = 0;

    virtual FrameStatistics statistics() const = 0;

    // Optional diagnostics hook: attach the shared profiler so backends can
    // record their explicit wait points (frame-slot retirement, swapchain
    // acquire/present, upload-fence waits) as percentile-tracked metrics.
    // Backends must work correctly without one; never null-checked by the
    // engine loop itself.
    virtual void setProfiler(engine::PerformanceProfiler *profiler)
    {
        profiler_ = profiler;
    }

protected:
    engine::PerformanceProfiler *profiler_ = nullptr;
};

} // namespace renderer
} // namespace engine

#endif
