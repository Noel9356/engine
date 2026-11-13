// Headless window backend: "windows" are plain CPU-side RGBA buffers with no
// OS window and no real input. Used as the fallback when GLFW is unavailable
// and for tests/CI (surface pixel access and PPM export).

#ifndef ENGINE_WINDOW_MODULE_HEADLESS_BACKEND_HPP
#define ENGINE_WINDOW_MODULE_HEADLESS_BACKEND_HPP

#include "window_backend.hpp"

namespace engine
{
namespace window
{

// Window backend without a windowing system: the framebuffer is a zeroed CPU
// RGBA buffer that can be locked for inspection or exported as a PPM.
// pollEvents does nothing, so only setSize, requestClose, and the iconify and
// restore calls produce events. Window state requested through the
// runtime-control calls is tracked in the WindowRecord and reported by
// getWindowAttributes; focus and hover are always false (no window system).
class HeadlessWindowBackend final : public WindowBackend
{
public:
    HeadlessWindowBackend() = default;

    const char *name() const override
    {
        return "headless";
    }

    // Vulkan only when the build defines ENGINE_WINDOW_HEADLESS_VULKAN.
    SurfaceApi surfaceApi(const WindowRecord &) const override
    {
#if defined(ENGINE_WINDOW_HEADLESS_VULKAN)
        return SurfaceApi::Vulkan;
#else
        return SurfaceApi::None;
#endif
    }

    bool initialize() override
    {
        return true;
    }

    void shutdown() override
    {}

    bool createWindow(WindowRecord &record, const WindowDescription &description) override;

    void destroyWindow(WindowRecord &record) override;

    bool shouldClose(const WindowRecord &record) const override
    {
        return record.closeRequested;
    }

    void requestClose(WindowRecord &record) override
    {
        record.closeRequested = true;
    }

    void setTitle(WindowRecord &record, const char *title) override;

    bool setSize(WindowRecord &record, std::uint32_t width, std::uint32_t height) override;
    bool setPosition(WindowRecord &record, std::int32_t x, std::int32_t y) override;
    bool getPosition(const WindowRecord &record, std::int32_t *outX, std::int32_t *outY) const override;

    void setOpacity(WindowRecord &record, float opacity) override;
    void setVisible(WindowRecord &record, bool visible) override;
    void setResizable(WindowRecord &record, bool resizable) override;
    void setDecorated(WindowRecord &record, bool decorated) override;
    void setFloating(WindowRecord &record, bool floating) override;
    void iconify(WindowRecord &record) override;
    void restore(WindowRecord &record) override;
    void maximize(WindowRecord &record) override;
    bool getWindowAttributes(const WindowRecord &record, WindowAttributes *outAttributes) const override;
    bool getContentScale(const WindowRecord &record, float *outX, float *outY) const override;

    void pollEvents() override
    {}

    void setCursorMode(const WindowRecord &, CursorMode) override
    {}

    bool cursorPosition(const WindowRecord &, double *, double *) const override
    {
        return false;
    }

    bool setClipboardText(const WindowRecord &, const char *text) override;
    bool copyClipboardText(const WindowRecord &, const char **outText) const override;

    const char *const *requiredInstanceExtensions(std::uint32_t *outCount) const override;

#if defined(ENGINE_WINDOW_HEADLESS_VULKAN)

    bool nativeSurfaceHandles(const WindowRecord &, NativeWindowHandles *outHandles) const override
    {
        if (outHandles != nullptr)
        {
            *outHandles = NativeWindowHandles{};
        }
        return true;
    }
#else
    bool nativeSurfaceHandles(const WindowRecord &, NativeWindowHandles *) const override
    {
        return false;
    }
#endif

private:
    std::string clipboardText_;
};

} // namespace window
} // namespace engine

#endif
