#ifndef ENGINE_MODULES_RENDERER_RENDERER_OVERLAY_CAPABILITY_HPP
#define ENGINE_MODULES_RENDERER_RENDERER_OVERLAY_CAPABILITY_HPP

// The published capability surface of the renderer's overlay seam. Every
// consumer - other modules, native mods, the app - includes THIS header, from
// the module that provides the capability. Like every published capability
// header this stays implementation-, platform- and vendor-agnostic: no Vulkan,
// GLFW or platform header may appear here (the capability header guard,
// cmake/capability_header_guard.cmake, enforces that mechanically).
//
// What the seam is for: immediate mode debug UIs that must draw INSIDE the
// frame, after the composite pass wrote the swapchain image and before the
// present transition (so captures see the overlay too). The seam is one
// registration and one per-frame virtual call; the renderer stays UI agnostic
// and the UI provider stays renderer agnostic at the type level - the only
// contract between them is the opaque backend handle bundle documented below.
//
// The capability is the implementation-bound member "renderer.vulkan.overlay"
// of the module "renderer_vulkan" - deliberately a MEMBER, not a family: the seam's
// contract IS the Vulkan handle bundle documented below, so demanding it by
// family would let a different backend satisfy the demand with a bundle whose
// fields mean something else. A run with a debug UI names the member, the
// resolver hands it the renderer's host, and the UI module initializes its own
// backend against the bundle. Without a UI module nothing changes: the
// recorder pointer stays null and the recording path costs one branch per
// frame.

#include <cstdint>

namespace engine
{

// One frame's overlay recording context. The command buffer is the frame's
// primary command buffer as an opaque handle: the provider behind
// "renderer.overlay" documents what the pointer means (the Vulkan provider:
// a VkCommandBuffer in the recording state, with the swapchain image in
// eColorAttachmentOptimal, sized width x height). The recorder records render
// commands and returns; it owns no frame state between calls.
struct OverlayFrameContext
{
    void         *commandBuffer = nullptr; // backend command buffer handle
    std::uint32_t width         = 0;       // swapchain extent this frame renders into
    std::uint32_t height        = 0;
    std::uint32_t imageIndex    = 0;       // swapchain image slot of this frame
    std::uint64_t frameIndex    = 0;       // renderer's monotonic frame counter
};

// The per-frame recorder a UI module registers with the overlay host. The
// renderer calls recordOverlay once per rendered frame, after the composite
// wrote the swapchain image and before the capture/present transitions. The
// call happens on the main thread inside command recording: the recorder must
// not publish bus events, block, or touch the window system from here.
class OverlayRecorder
{
public:
    virtual ~OverlayRecorder() = default;

    virtual void recordOverlay(const OverlayFrameContext &context) = 0;
};

// The opaque backend handle bundle for UI backends that need device-level
// initialization (a Vulkan backend needs instance, physical device, logical
// device, queue and color format). The renderer.vulkan provider fills this
// structure; its field semantics are the Vulkan provider's documented
// contract (renderer_state.hpp, RendererVulkanBackendHandles - the layout
// matches field for field so a UI module that speaks Vulkan casts here and
// reads there). Every pointer field is an opaque object handle; the integers
// are the queue family index, the swapchain image counts and the color
// attachment VkFormat of the swapchain as a plain integer.
struct OverlayBackendHandles
{
    void         *instance         = nullptr;
    void         *physicalDevice   = nullptr;
    void         *device           = nullptr;
    void         *graphicsQueue    = nullptr;
    std::uint32_t queueFamilyIndex = 0;
    std::uint32_t framesInFlight   = 0;
    std::uint32_t minImageCount    = 0; // surface's minimum swapchain length
    std::uint32_t imageCount       = 0; // live swapchain image slots
    std::int32_t  colorFormat      = 0;
};

// Capability "renderer.vulkan.overlay": the in-frame UI recording seam. One recorder
// at a time; registering a second replaces the first (a debug UI is a
// singleton per run by nature). Pass nullptr to unregister.
class RendererOverlayHostCapability
{
public:
    // The implementation-bound member name: consumers acquire through THIS
    // name (the seam is intentionally not a family - see the file comment).
    static constexpr const char *capabilityName = "renderer.vulkan.overlay";

    virtual ~RendererOverlayHostCapability() = default;

    // Registers (or replaces, or clears with nullptr) the per-frame overlay
    // recorder. Call during construct/start, never from a frame callback:
    // the renderer reads the pointer without synchronization.
    virtual void setOverlayRecorder(OverlayRecorder *recorder) = 0;

    // The backend handle bundle, or nullptr before the renderer initialized
    // its device and swapchain. The bundle stays owned by the renderer and
    // remains valid for the lifetime of the capability.
    virtual const OverlayBackendHandles *overlayBackendHandles() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_RENDERER_OVERLAY_CAPABILITY_HPP
