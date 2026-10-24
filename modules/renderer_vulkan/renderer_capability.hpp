#ifndef ENGINE_MODULES_RENDERER_RENDERER_CAPABILITY_HPP
#define ENGINE_MODULES_RENDERER_RENDERER_CAPABILITY_HPP

// The published capability surface of the renderer module. Every consumer -
// other modules, native mods, the app - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift.
//
// Mod-facing and implementation-, platform- and vendor-agnostic: the renderer
// behind it may be Vulkan today and another graphics API tomorrow; frame
// events carry only primitive types. See mods/README.md, section
// "Portability".

#include <array>
#include <cstdint>

namespace engine
{

// Published by the frame rendering provider at the beginning of every
// rendered frame, before command recording. Components that must act inside
// the frame (capture requests queue here) do so from this event.
struct FrameBeginEvent
{
    std::uint64_t frameIndex = 0;
};

// Published after the GPU finished a requested capture. pixelData points into
// renderer owned staging memory and stays valid until the handler returns;
// rows are tightly packed RGBA8, top row first.
struct FrameCapturedEvent
{
    const std::uint8_t *pixelData = nullptr;
    std::uint32_t       width     = 0;
    std::uint32_t       height    = 0;
};

// Published after the frame was submitted and presented.
struct FrameRenderedEvent
{
    std::uint64_t frameIndex = 0;
};

// Capability "renderer": frame rendering. The surface is intentionally small
// and lighting agnostic: rendering features grow through configurable pipeline
// state inside the provider, never through algorithm specific forks of this
// interface.
class RendererCapability
{
public:
    // The implementation-agnostic family name of this capability. Consumers
    // acquire through it; the module's plugin descriptor carries the
    // implementation-bound member name ("renderer.vulkan").
    static constexpr const char *capabilityName = "renderer";

    virtual ~RendererCapability() = default;

    // Renders and presents one frame. Handles swapchain recreation when the
    // framebuffer size changed.
    virtual void renderFrame() = 0;

    // Blocks until every submitted GPU command finished. Used before shutdown.
    virtual void waitUntilIdle() = 0;

    // Requests that the next rendered frame is copied to host memory and
    // published through FrameCapturedEvent.
    virtual void requestFrameCapture() = 0;

    // Reports whether the graphics backend observed internal errors during
    // this session (driver diagnostics or validation-layer reports). The
    // provider keeps rendering after such errors, so automated runs and
    // embedders poll this to turn silent backend failures into exit codes.
    virtual bool validationErrorsObserved() const = 0;

    // Camera pose queries and moves. The pose is the free-fly controller's
    // state: a script that sets it teleports the camera, and the controller
    // keeps integrating input around the new pose afterwards (the look
    // direction is re-derived from position -> lookAt; a set position keeps
    // the current look direction). Values are plain world-space floats - the
    // implementation behind the capability may move the camera any way it
    // likes, the contract is the pose, not the controller.
    virtual std::array<float, 3> cameraPosition() const = 0;
    virtual std::array<float, 3> cameraLookAt() const    = 0;
    virtual void                 setCameraPosition(const std::array<float, 3> &position) = 0;
    virtual void                 setCameraLookAt(const std::array<float, 3> &lookAt)     = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_RENDERER_CAPABILITY_HPP
