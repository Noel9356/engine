#ifndef ENGINE_CAPABILITIES_RENDERER_CONTRACT_HPP
#define ENGINE_CAPABILITIES_RENDERER_CONTRACT_HPP

// Mod-facing contract: implementation-, platform- and vendor-agnostic. The
// renderer behind it may be Vulkan today and another graphics API tomorrow;
// frame events carry only primitive types. See mods/README.md, section
// "Portability".

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

// Capability "renderer": frame rendering. The contract is intentionally small
// and lighting agnostic: rendering features grow through configurable pipeline
// state inside the provider, never through algorithm specific forks of this
// interface.
class RendererCapability
{
public:
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
};

} // namespace engine

#endif // ENGINE_CAPABILITIES_RENDERER_CONTRACT_HPP
