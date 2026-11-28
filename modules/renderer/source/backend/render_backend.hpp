// Abstract backend interface behind the "renderer" capability.
//
// renderer.cpp owns the capability plumbing and resource validation; a
// RenderBackend implementation owns actual pixel output. The headless
// backend (headless_backend.hpp) and the Vulkan backend (vulkan_backend.hpp)
// are the two implementations. Calls arrive on the main thread only.

#pragma once

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/device_profile.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>
#include "render_errors.hpp"
#include "render_shared.hpp"

#include <cstdint>

namespace engine
{
namespace renderer
{

// Defined in gi_probe_sampling.hpp / gather_field_sampling.hpp /
// reflection_field_sampling.hpp (the borrowed radiance_cache probe-grid
// snapshot, radiance_cache gather-field snapshot, and reflection_field field
// snapshot passed to RenderBackend::setProbeGrid / setGatherField /
// setReflectionField).
struct GiProbeGrid;
struct GatherField;
struct ReflectionField;

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

    // Environment/lights/probe grid are pushed by renderer.cpp when the pbr,
    // direct_lighting and radiance_cache capabilities report new versions.
    virtual void setEnvironment(const pbr::PbrEnvironment &environment)                           = 0;
    virtual void setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count) = 0;

    // The radiance_cache probe-grid snapshot for the indirect-lighting hook. The
    // view is borrowed only for this call; backends copy what they need
    // (headless: the CPU sampling path in gi_probe_sampling.hpp; Vulkan: the
    // packed std430 probe storage buffer). An invalid/empty grid disables the
    // hook and keeps the legacy hemispheric ambient.
    virtual void setProbeGrid(const GiProbeGrid &grid) = 0;

    // The reflection_field screen-space field snapshot for the indirect
    // specular hook. The view is borrowed only for this call. An invalid/
    // empty field disables the hook and keeps the pre-hook shading. The
    // headless backend samples the field per pixel (reflection_field_sampling.hpp);
    // the Vulkan backend uploads it into per-frame images that mesh.frag
    // composites (see docs/backends.md).
    virtual void setReflectionField(const ReflectionField &field) = 0;

    // The radiance_cache screen-space gather field snapshot for the final
    // gather hook: the per-pixel indirect irradiance the CPU pixel path
    // composites over (and in place of) its probe-walk ambient, carrying the
    // contact-scale structure (short-range occlusion, caustic irradiance)
    // the coarse probe lattice cannot resolve. The view is borrowed only for
    // this call; backends that cannot consume it keep the probe-grid ambient
    // (see consumesGatherField). An invalid/empty field disables the hook.
    virtual void setGatherField(const GatherField &field) = 0;

    // Whether this backend composites the gather field: the module core
    // pushes the field's viewport to the radiance_cache.gather capability
    // only for a consuming backend, so non-consuming backends never make the
    // module pay the per-evaluate gather pass. Default false; the headless
    // rasterizer overrides it.
    virtual bool consumesGatherField() const { return false; }

    virtual void beginFrame(const FrameParameters &frame)                                          = 0;
    virtual void drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material) = 0;
    virtual void drawMeshInstances(
        MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount, const MeshMaterial &material) = 0;

    // One mesh draw inside a submitted batch; the fields mirror
    // RenderDrawCommand (borrowed pointers, default material when null).
    struct MeshBatchItem
    {
        MeshHandle           mesh        = invalidMesh;
        const float         *modelMatrix = nullptr;
        const MeshMaterial  *material    = nullptr;
    };

    // A frame's mesh draws in submission order. The default implementation
    // replays the items through drawMesh one by one, so a backend that does
    // not override this is behaviorally identical to the per-command loop it
    // replaces. The headless backend overrides it with one batched
    // tile-parallel pass: every item's triangles are binned into screen
    // tiles first and each tile is then rasterized by exactly one worker,
    // with per-tile record lists preserving submission order - so the
    // per-pixel triangle order (and therefore the image) equals the
    // sequential replay exactly, while the pixel work spreads across every
    // worker. Many-small-draw scenes (streamed chunk meshes, instanced
    // props) spend most of their frame in draws too small for the per-draw
    // parallel thresholds; the batch is what lets those scenes use the
    // whole machine.
    virtual void drawMeshBatch(const MeshBatchItem *items, std::uint32_t itemCount)
    {
        const MeshMaterial defaultMaterial;
        for (std::uint32_t index = 0; index < itemCount; ++index)
        {
            const MeshBatchItem &item = items[index];
            if (item.modelMatrix == nullptr || item.mesh == invalidMesh)
            {
                continue;
            }
            drawMesh(item.mesh, item.modelMatrix, item.material != nullptr ? *item.material : defaultMaterial);
        }
    }

    // Shell-fur draws (see FurLayerParams in capabilities.hpp): layerCount
    // alpha-blended shell layers over the opaque base surface. Both backends
    // implement them; the headless rasterizer mirrors the fur pipeline's
    // shading exactly (fur_shading.hpp furShade <-> shaders/fur.frag).
    virtual void drawFur(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material,
        const FurLayerParams &params) = 0;
    virtual void drawFurInstances(MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount,
        const MeshMaterial &material, const FurLayerParams &params) = 0;

    virtual void drawOverlay(const OverlayDrawData &data)                                                       = 0;
    virtual void endFrame()                                                                                     = 0;

    // Resource creation takes the module-assigned id and copies/queues the
    // upload; the description's memory is not referenced afterwards.
    virtual bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error) = 0;
    virtual void destroyMesh(MeshHandle mesh)                                                      = 0;

    // Dynamic-geometry refresh (the cloth path): replaces a mesh's whole
    // vertex stream with data laid out exactly like its creation description
    // (same count, stride, and attribute table; the index buffer is
    // untouched). Backends copy the source before returning; the new
    // positions are visible to the draws of the frame that follows the
    // update (same frame when called between beginFrame/endFrame). Default
    // implementation reports Unsupported so a backend without dynamic
    // updates stays interface-complete.
    virtual bool updateMeshVertices(
        MeshHandle mesh, const void *vertices, std::uint32_t vertexCount, RenderError *error)
    {
        (void)mesh;
        (void)vertices;
        (void)vertexCount;
        reportError(error, RenderError::Unsupported);
        return false;
    }

    virtual bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error) = 0;

    // Which block formats createBlockTexture accepts. The headless backend
    // software-decodes every format (bcdec), the Vulkan backend uploads
    // whichever formats the device samples natively; the module core asks
    // before transcoding a KTX file so the payload lands on the native
    // path whenever the machine has one.
    [[nodiscard]] virtual BlockTextureFormatSupport blockFormatSupport() const
    {
        return {};
    }

    // Block-compressed texture creation (the KTX path): every mip level's
    // block bytes concatenated, with per-level sizes; the mip chain comes
    // from the file, so no GPU blit pass is involved. Default reports
    // Unsupported so a backend without block support stays interface-
    // complete; the module core falls back to the RGBA path when
    // blockFormatSupport() reports nothing.
    virtual bool createBlockTexture(
        TextureHandle id, const BlockTextureDescription &description, RenderError *error)
    {
        (void)id;
        (void)description;
        reportError(error, RenderError::Unsupported);
        return false;
    }

    virtual void destroyTexture(TextureHandle texture)                                                      = 0;

    virtual FrameStatistics statistics() const = 0;

    // True when endFrame blocks until the display's vblank, i.e. the backend
    // already paces the loop to the display refresh. A FIFO swapchain does
    // this; an immediate/mailbox swapchain and the headless backend do not.
    // The application uses it to avoid throttling a second time on top of
    // presentation: two independent throttles (a steady_clock deadline chain
    // and the presentation engine) drift apart and slip whole refresh
    // periods, which shows up as a P99 of two frames rather than one.
    [[nodiscard]] virtual bool waitsForPresentation() const
    {
        return false;
    }

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

