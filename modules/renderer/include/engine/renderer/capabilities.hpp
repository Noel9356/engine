// Public capability surface of the renderer module.
//
// Declares the backend-agnostic "renderer" capability plus the "renderer.vulkan"
// and "renderer.headless" secondary capabilities: handle types, error codes, and
// the per-draw/per-frame structs consumers pass through the function-pointer
// structs at the bottom of this file. Everything here is consumed via
// ParticipantContext::acquireCapability<T>().

#ifndef ENGINE_RENDERER_CAPABILITIES_HPP
#define ENGINE_RENDERER_CAPABILITIES_HPP

#include <engine/renderer/device_profile.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include <cstdint>

namespace engine
{
namespace renderer
{

// ABI version shared by all three capability structs below. Bump whenever any
// struct here changes layout. History: 10 -> 11 added the device-local memory
// budget/pressure fields to FrameStatistics (the GPU-memory-budget feedback
// surface consumed by the CPU-side asset policy). 11 -> 12 marks the optional
// lumen_gi integration: the renderer now consumes the "lumen_gi" capability
// (probe-grid indirect lighting replacing the hemispheric ambient when the
// module publishes a valid snapshot) and reads the new [renderer]
// gi_indirect_lighting key; ABI-checked consumers recompile against this
// header so both sides agree on the integration contract.
inline constexpr std::uint32_t renderCapabilityAbi = 12u;

// Renderer, mesh, and texture handles are module-assigned ids starting at 1;
// handles are never reused and 0 always means "none".
using RendererHandle = std::uint32_t;

inline constexpr RendererHandle invalidRenderer = 0;

using MeshHandle = std::uint32_t;

inline constexpr MeshHandle invalidMesh = 0;

// Error codes reported through the optional RenderError* out-parameters.
// Out-parameters are left untouched on success paths that do not set them
// explicitly; always treat the return value as authoritative.
enum class RenderError : std::uint32_t
{
    None = 0,

    BackendUnavailable, // requested backend cannot run here (no Vulkan, wrong surface API)

    CreationFailed, // allocation or driver failure while creating renderer/mesh/texture

    InvalidHandle, // renderer handle is not a live renderer

    InvalidMesh, // mesh description malformed or indices out of range

    InvalidTexture, // texture description malformed or decode failed

    Unsupported // valid request, but this build/backend cannot do it (e.g. PPM export)
};

// Texture sampling controls honoured by both backends.
enum class TextureFilter : std::uint32_t
{
    Nearest = 0, // point sampling
    Linear       // bilinear (plus trilinear mip filter in the Vulkan backend)
};

enum class TextureAddress : std::uint32_t
{
    Repeat = 0, // wrap UVs modulo 1
    ClampEdge   // clamp UVs to the outer texel edge
};

// View/projection in OpenGL convention: column-major 4x4, right-handed clip
// space. Backends apply the Vulkan clip-space fixup themselves, so callers
// keep supplying the same matrices to every renderer.
struct CameraMatrices
{
    float view[16] = {};

    float projection[16] = {};

    float position[3] = {0.0f, 0.0f, 0.0f}; // world-space eye position, used for PBR shading
};

// One overlay (UI) vertex. position is in display pixels; color is packed
// RGBA with red in the lowest byte (0xAABBGGRR).
struct OverlayVertex
{
    float         position[2] = {};
    float         uv[2]       = {};
    std::uint32_t color       = 0xFFFFFFFFu;
};

// One indexed overlay draw. clipRectangle is (x0, y0, x1, y1) in display
// pixels; the backend turns it into a scissor rect. elementCount must be a
// multiple of 3. resetRenderState makes the backend rebind its overlay
// pipeline/vertex buffers before this command (for interleaved UI sources).
struct OverlayDrawCommand
{
    std::uint32_t indexOffset      = 0;
    std::uint32_t vertexOffset     = 0;
    std::uint32_t elementCount     = 0;
    float         clipRectangle[4] = {};
    TextureHandle texture          = invalidTexture;
    bool          resetRenderState = false;
};

// A whole overlay submission for one frame. vertices/indices/commands are
// borrowed and only need to stay valid until drawOverlay() returns; the
// backend copies them into per-frame GPU buffers. displayPosition/displaySize
// are in display pixels; framebufferScale converts display pixels to
// framebuffer pixels (set both to 1 when the scales match).
struct OverlayDrawData
{
    const OverlayVertex      *vertices     = nullptr;
    std::uint32_t             vertexCount  = 0;
    const std::uint32_t      *indices      = nullptr;
    std::uint32_t             indexCount   = 0;
    const OverlayDrawCommand *commands     = nullptr;
    std::uint32_t             commandCount = 0;

    float displayPosition[2]  = {};
    float displaySize[2]      = {};
    float framebufferScale[2] = {1.0f, 1.0f};
};

// Source pixel layouts accepted by createTexture. R8/RG8/RGB8 have no alpha
// (expanded to opaque); BGRA8 stores red in the last byte of each pixel.
// Uploads are converted to RGBA8 internally.
enum class TexturePixelFormat : std::uint32_t
{
    R8 = 0,
    RG8,
    RGB8,
    RGBA8,
    BGRA8
};

// Which end of the pixel buffer holds the first row of the image.
enum class TextureOrigin : std::uint32_t
{
    TopLeft = 0,
    BottomLeft
};

// Texture upload description. pixels is borrowed and fully consumed before
// createTexture returns. All sizes are in pixels; rowStrideBytes is the byte
// distance between source rows (0 = tightly packed width * bytesPerPixel).
struct TextureDescription
{
    std::uint32_t width  = 0;
    std::uint32_t height = 0;

    const std::uint8_t *pixels = nullptr;

    TextureFilter filter = TextureFilter::Linear;

    TextureAddress address = TextureAddress::Repeat;

    TexturePixelFormat pixelFormat    = TexturePixelFormat::RGBA8;
    TextureOrigin      origin         = TextureOrigin::TopLeft;
    std::uint32_t      rowStrideBytes = 0;
};

// One queued mesh draw for drawMeshes(). modelMatrix points at 16
// column-major floats; material may be null, meaning the default material.
// Both pointers are borrowed for the duration of the call only.
struct RenderDrawCommand
{
    MeshHandle          mesh        = invalidMesh;
    const float        *modelMatrix = nullptr;
    const MeshMaterial *material    = nullptr;
};

// Per-frame state passed to beginFrame(); nullptr falls back to the
// configured [renderer] clear_color.
struct FrameParameters
{
    float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f}; // RGBA, 0..1
};

// Cumulative counters since renderer creation. gpuFrameMilliseconds is the
// last completed frame's GPU duration and is only meaningful when
// gpuTimingAvailable is true (requires GPU timestamps and device support).
// The device-local memory fields are the GPU-budget feedback surface
// (potential_optimisations_hybrid.md candidate 12): the Vulkan backend
// publishes the last periodic budget sample and its hysteresis pressure
// state so a CPU-side asset policy can evict unreferenced data before the
// driver starts paging. Zeros/false on the headless backend and when the
// budget extension is unavailable.
struct FrameStatistics
{
    std::uint64_t framesRendered      = 0;
    std::uint64_t trianglesSubmitted  = 0;
    std::uint64_t trianglesRasterized = 0;

    double gpuFrameMilliseconds = 0.0;
    bool   gpuTimingAvailable   = false;

    std::uint64_t deviceLocalBudgetBytes = 0;
    std::uint64_t deviceLocalUsageBytes  = 0;
    bool          deviceLocalMemoryPressure = false;
};

// Device description strings returned by the Vulkan capability. The pointers
// are owned by the module and stay valid until the renderer is destroyed.
struct RendererInfoStrings
{
    const char *vendor          = nullptr;
    const char *renderer        = nullptr;
    const char *version         = nullptr;
    const char *shadingLanguage = nullptr;
};

// Raw Vulkan objects exposed for interop, typed as void* so this header
// stays Vulkan-free. Borrowed, not owned: never destroy them.
struct VulkanDeviceHandles
{
    void         *vkInstance          = nullptr;
    void         *vkPhysicalDevice    = nullptr;
    void         *vkDevice            = nullptr;
    void         *vkGraphicsQueue     = nullptr;
    std::uint32_t graphicsQueueFamily = 0;
};

// The "renderer" capability: backend-agnostic rendering into a window surface.
// All functions take the renderer handle first; invalid handles are ignored
// (or reported through the error out-parameter where present). Draw calls are
// main-thread only and must sit between beginFrame and endFrame.
struct RenderCapability
{
    static constexpr const char *capabilityName = "renderer";

    std::uint32_t abi = renderCapabilityAbi;

    void *self = nullptr;

    const char *(*activeBackendName)(void *self, RendererHandle renderer);

    DeviceClass (*deviceClass)(void *self, RendererHandle renderer);
    const char *(*deviceName)(void *self, RendererHandle renderer);

    bool (*deviceProfile)(void *self, RendererHandle renderer, DeviceProfile *outProfile);

    RendererHandle (*createRenderer)(void *self, window::WindowHandle surface, RenderError *error);
    void (*destroyRenderer)(void *self, RendererHandle renderer);

    void (*setViewport)(void *self, RendererHandle renderer, std::uint32_t width, std::uint32_t height);

    void (*setCamera)(void *self, RendererHandle renderer, const CameraMatrices *camera);

    MeshHandle (*createMesh)(
        void *self, RendererHandle renderer, const MeshDescription *description, RenderError *error);
    void (*destroyMesh)(void *self, RendererHandle renderer, MeshHandle mesh);
    void (*drawMesh)(
        void *self, RendererHandle renderer, MeshHandle mesh, const float *modelMatrix, const MeshMaterial *material);

    void (*drawMeshes)(
        void *self, RendererHandle renderer, const RenderDrawCommand *commands, std::uint32_t commandCount);

    TextureHandle (*createTexture)(
        void *self, RendererHandle renderer, const TextureDescription *description, RenderError *error);

    TextureHandle (*createTextureFromFile)(
        void *self, RendererHandle renderer, const char *filePath, RenderError *error);
    void (*destroyTexture)(void *self, RendererHandle renderer, TextureHandle texture);

    void (*beginFrame)(void *self, RendererHandle renderer, const FrameParameters *frame);
    void (*endFrame)(void *self, RendererHandle renderer);

    void (*frameStatistics)(void *self, RendererHandle renderer, FrameStatistics *outStatistics);

    void (*drawOverlay)(void *self, RendererHandle renderer, const OverlayDrawData *data);

    void (*drawMeshInstances)(void *self, RendererHandle renderer, MeshHandle mesh, const float *modelMatrices,
        std::uint32_t instanceCount, const MeshMaterial *material);

    // True when this renderer's endFrame already blocks until the display's
    // vblank (a FIFO swapchain does), so the host loop is paced by
    // presentation. An application that also throttles to a fixed frame rate
    // should skip its own throttle when this returns true: two independent
    // throttles - a wall-clock deadline chain and the presentation engine -
    // are never in lock-step, and the frames that slip land a whole refresh
    // period late, turning the frame-time P99 into two frames instead of one.
    // Returns false for backends that present without waiting (headless,
    // immediate, mailbox). Appended at the end of the struct: field offsets
    // of the existing entries and the ABI version are unchanged. Older
    // loaders ignore it; a loader must still null-check it before calling,
    // since a module built before this field leaves it null.
    bool (*waitsForPresentation)(void *self, RendererHandle renderer);
};

// The "renderer.vulkan" secondary capability: Vulkan-specific introspection and
// helpers on top of a renderer that happens to use the Vulkan backend. Every
// entry returns false (or RenderError::Unsupported) when the renderer is not
// a Vulkan renderer or the module was built without ENGINE_RENDERER_BACKEND_VULKAN.
struct VulkanRenderCapability
{
    static constexpr const char *capabilityName = "renderer.vulkan";

    std::uint32_t abi = renderCapabilityAbi;

    void *self = nullptr;

    bool (*infoStrings)(void *self, RendererHandle renderer, RendererInfoStrings *outInfo);

    bool (*nativeHandles)(void *self, RendererHandle renderer, VulkanDeviceHandles *outHandles);

    bool (*deviceProfile)(void *self, RendererHandle renderer, DeviceProfile *outProfile);

    // Schedules a PPM screenshot of the frame being rendered; the copy is
    // recorded by the matching endFrame, and the file is written once that
    // submission provably completed on the GPU (next rotation of its frame
    // slot, or at renderer destruction if the engine stops first).
    bool (*exportFramePpm)(void *self, RendererHandle renderer, const char *filePath, RenderError *error);
};

// CPU rasterizer counters (no GPU timing on the headless backend).
struct HeadlessRenderStatistics
{
    std::uint64_t framesRendered      = 0;
    std::uint64_t trianglesSubmitted  = 0;
    std::uint64_t trianglesRasterized = 0;
};

// The "renderer.headless" secondary capability: CPU rasterizer statistics for
// a headless renderer.
struct HeadlessRenderCapability
{
    static constexpr const char *capabilityName = "renderer.headless";

    std::uint32_t abi = renderCapabilityAbi;

    void *self = nullptr;

    void (*statistics)(void *self, RendererHandle renderer, HeadlessRenderStatistics *outStatistics);
};

} // namespace renderer
} // namespace engine

#endif
