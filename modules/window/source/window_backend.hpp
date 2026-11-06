// Internal backend interface shared between the window module's source
// files: the per-window record, the event queueing rules, and the abstract
// WindowBackend that the GLFW and headless implementations fulfill.

#ifndef ENGINE_WINDOW_MODULE_BACKEND_HPP
#define ENGINE_WINDOW_MODULE_BACKEND_HPP

#include <engine/window/capabilities.hpp>

#include <algorithm>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace engine
{
namespace window
{

// Module-side state for one window, shared with the backend that created it.
// width/height track the current client-area size and framebuffer* the
// swapchain-size pixels (both pixels); keyStateMask mirrors pressed keys;
// events is the queue drained by the module's popEvent(s). headlessRgba backs
// the software surface; platformWindow is the backend-native handle
// (GLFWwindow* for the GLFW backend).
struct WindowRecord
{
    WindowHandle id = invalidWindow;

    std::string title;

    std::uint32_t width  = 0;
    std::uint32_t height = 0;

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;

    std::uint32_t minimumWidth  = 0;
    std::uint32_t minimumHeight = 0;
    std::uint32_t maximumWidth  = 0;
    std::uint32_t maximumHeight = 0;

    bool closeRequested = false;

    // Bit (1 << Key value) set for each pressed key.
    std::uint64_t keyStateMask = 0;

    std::deque<WindowEvent> events;
    std::size_t             eventQueueCapacity = 4096;

    void *platformWindow = nullptr;

    std::vector<std::uint8_t> headlessRgba;
};

// Appends to the window's queue: a trailing same-kind resize event is
// replaced in place (resize storms collapse to the latest size); when the
// queue is full the first queued resize is evicted, else the oldest event.
// Allocation failure silently drops the new event.
inline void queueWindowEvent(WindowRecord &record, const WindowEvent &event)
{
    const bool coalescible
        = event.kind == WindowEventKind::Resized || event.kind == WindowEventKind::FramebufferResized;
    if (coalescible && !record.events.empty() && record.events.back().kind == event.kind)
    {
        record.events.back() = event;
        return;
    }
    if (record.eventQueueCapacity != 0 && record.events.size() >= record.eventQueueCapacity)
    {
        const auto disposable = std::find_if(record.events.begin(),
            record.events.end(),
            [](const WindowEvent &queued)
            { return queued.kind == WindowEventKind::Resized || queued.kind == WindowEventKind::FramebufferResized; });
        if (disposable != record.events.end())
        {
            record.events.erase(disposable);
        }
        else
        {
            record.events.pop_front();
        }
    }
    try
    {
        record.events.push_back(event);
    }
    catch (...)
    {}
}

// Interface every window backend implements (GLFW, headless). Calls come
// from the module on the main thread; OS callbacks fire during pollEvents()
// and land in the WindowRecord queues.
class WindowBackend
{
public:
    virtual ~WindowBackend() = default;

    virtual const char *name() const = 0;

    virtual SurfaceApi surfaceApi(const WindowRecord &record) const = 0;

    virtual bool initialize() = 0;
    virtual void shutdown()   = 0;

    virtual bool createWindow(WindowRecord &record, const WindowDescription &description) = 0;
    virtual void destroyWindow(WindowRecord &record)                                      = 0;

    virtual bool shouldClose(const WindowRecord &record) const     = 0;
    virtual void requestClose(WindowRecord &record)                = 0;
    virtual void setTitle(WindowRecord &record, const char *title) = 0;

    virtual bool setSize(WindowRecord &record, std::uint32_t width, std::uint32_t height) = 0;
    virtual bool setPosition(WindowRecord &record, std::int32_t x, std::int32_t y)        = 0;

    virtual void pollEvents() = 0;

    virtual void setCursorMode(const WindowRecord &record, CursorMode mode)                   = 0;
    virtual bool cursorPosition(const WindowRecord &record, double *outX, double *outY) const = 0;

    virtual const char *const *requiredInstanceExtensions(std::uint32_t *outCount) const = 0;

    virtual bool nativeSurfaceHandles(const WindowRecord &record, NativeWindowHandles *outHandles) const = 0;
};

} // namespace window
} // namespace engine

#endif
