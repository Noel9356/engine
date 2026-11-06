// GLFW-backed window backend, compiled only when ENGINE_WINDOW_BACKEND_GLFW
// is defined (otherwise the header compiles empty).

#ifndef ENGINE_WINDOW_MODULE_GLFW_BACKEND_HPP
#define ENGINE_WINDOW_MODULE_GLFW_BACKEND_HPP

#include "window_backend.hpp"

#if defined(ENGINE_WINDOW_BACKEND_GLFW)

namespace engine
{
namespace window
{

// GLFW version string for the window.glfw capability.
const char *glfwBackendVersionString();

// Creates real OS windows via GLFW, translates GLFW callbacks into the
// per-window event queues, and answers Vulkan/native-handle queries.
class GlfwWindowBackend final : public WindowBackend
{
public:
    GlfwWindowBackend() = default;

    const char *name() const override
    {
        return "glfw";
    }

    SurfaceApi surfaceApi(const WindowRecord &record) const override;

    bool initialize() override;
    void shutdown() override;

    bool createWindow(WindowRecord &record, const WindowDescription &description) override;
    void destroyWindow(WindowRecord &record) override;

    bool shouldClose(const WindowRecord &record) const override;
    void requestClose(WindowRecord &record) override;
    void setTitle(WindowRecord &record, const char *title) override;

    bool setSize(WindowRecord &record, std::uint32_t width, std::uint32_t height) override;
    bool setPosition(WindowRecord &record, std::int32_t x, std::int32_t y) override;

    void pollEvents() override;

    void setCursorMode(const WindowRecord &record, CursorMode mode) override;

    bool cursorPosition(const WindowRecord &record, double *outX, double *outY) const override;

    const char *const *requiredInstanceExtensions(std::uint32_t *outCount) const override;

    bool nativeSurfaceHandles(const WindowRecord &record, NativeWindowHandles *outHandles) const override;
};

} // namespace window
} // namespace engine

#endif

#endif
