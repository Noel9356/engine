#ifndef ENGINE_CAPABILITIES_WINDOW_CONTRACT_HPP
#define ENGINE_CAPABILITIES_WINDOW_CONTRACT_HPP

// Mod-facing contract. Like every header under engine/capabilities this must
// stay implementation-, platform- and vendor-agnostic: no Vulkan, OpenGL,
// GLFW, Win32 or X11 headers, and no types from them. The configure step
// enforces this mechanically (see kernel/CMakeLists.txt); the mod facing rules
// are documented in mods/README.md, section "Portability".

#include <cstdint>
#include <string>
#include <vector>

namespace engine
{

using WindowIdentifier = std::uint32_t;

inline constexpr WindowIdentifier invalidWindowIdentifier = 0;

struct WindowDescription
{
    std::string   title;
    std::uint32_t width     = 1280;
    std::uint32_t height    = 720;
    bool          resizable = true;
};

// Published when the user closes a window.
struct WindowClosedEvent
{
    WindowIdentifier windowIdentifier = invalidWindowIdentifier;
};

// Published when the framebuffer pixel size of a window changes (resize or
// device pixel ratio change). The renderer uses this to recreate the swapchain.
struct WindowResizedEvent
{
    WindowIdentifier windowIdentifier = invalidWindowIdentifier;
    std::uint32_t    width            = 0;
    std::uint32_t    height           = 0;
};

// Capability "window": surface and event pump. The contract is deliberately
// windowing library agnostic: nothing here names the underlying library. Two
// members deal in opaque handles (nativeWindowHandle, createPresentationSurface)
// so rendering and input implementations can attach to a window without this
// contract knowing their APIs. A rendering backend for another graphics API is
// a different module implementation behind the same renderer contract; mods
// cannot tell the difference (mods/README.md, section "Portability").
class WindowCapability
{
public:
    static constexpr const char *capabilityName = "window";

    virtual ~WindowCapability() = default;

    virtual WindowIdentifier createMainWindow(const WindowDescription &description) = 0;

    virtual bool windowCloseRequested(WindowIdentifier windowIdentifier) = 0;

    // Pumps the platform event loop once and publishes WindowClosedEvent and
    // WindowResizedEvent through the event bus.
    virtual void pollEvents() = 0;

    virtual std::uint32_t framebufferWidth(WindowIdentifier windowIdentifier)  = 0;
    virtual std::uint32_t framebufferHeight(WindowIdentifier windowIdentifier) = 0;

    virtual std::vector<WindowIdentifier> windowIdentifiers() const = 0;

    // Presentation support for rendering implementations, kept opaque on
    // purpose: graphicsApi names the graphics API family the caller implements
    // (this build ships "vulkan"), graphicsInstance is that family's instance
    // handle (a VkInstance, passed as void*), and the returned handle has the
    // same family's meaning (cast to VkSurfaceKHR). Only the matching
    // rendering implementation consumes these; script mods never see them, and
    // an implementation for another family ("opengl") uses the same members
    // with its own names and handles.
    virtual std::vector<std::string> requiredInstanceExtensions(const std::string &graphicsApi) = 0;
    virtual void                    *createPresentationSurface(
        const std::string &graphicsApi, void *graphicsInstance, WindowIdentifier windowIdentifier
    ) = 0;

    // Native window handle for implementations that attach backend specific
    // callbacks (input, platform integration). The pointed-to type is a
    // windowing library detail of the provider implementation; contract
    // consumers treat it as opaque.
    virtual void *nativeWindowHandle(WindowIdentifier windowIdentifier) = 0;

    // Reverse lookup used by input implementations inside platform callbacks.
    virtual WindowIdentifier windowIdentifierFromNativeHandle(void *nativeWindowHandle) = 0;
};

} // namespace engine

#endif // ENGINE_CAPABILITIES_WINDOW_CONTRACT_HPP
