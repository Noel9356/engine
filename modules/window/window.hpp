#ifndef ENGINE_MODULES_WINDOW_HPP
#define ENGINE_MODULES_WINDOW_HPP

#include "engine/capabilities/logging_contract.hpp"
#include "engine/capabilities/window_contract.hpp"
#include "engine/kernel/component_interface.hpp"

#include <map>
#include <stdexcept>
#include <string>

struct GLFWwindow;

namespace engine
{

// Module "window": provides the WindowCapability on top of GLFW. Owns the GLFW
// lifecycle, the main window, the platform event pump and the Vulkan surface
// creation entry point. The renderer and the input module reach GLFW through
// this capability instead of initializing anything themselves.
class WindowModule final : public ModuleInterface, public WindowCapability
{
public:
    WindowModule();
    ~WindowModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    WindowIdentifier              createMainWindow(const WindowDescription &description) override;
    bool                          windowCloseRequested(WindowIdentifier windowIdentifier) override;
    void                          pollEvents() override;
    std::uint32_t                 framebufferWidth(WindowIdentifier windowIdentifier) override;
    std::uint32_t                 framebufferHeight(WindowIdentifier windowIdentifier) override;
    std::vector<WindowIdentifier> windowIdentifiers() const override;
    std::vector<std::string>      requiredInstanceExtensions(const std::string &graphicsApi) override;
    void                         *createPresentationSurface(
        const std::string &graphicsApi, void *graphicsInstance, WindowIdentifier windowIdentifier
    ) override;
    void            *nativeWindowHandle(WindowIdentifier windowIdentifier) override;
    WindowIdentifier windowIdentifierFromNativeHandle(void *nativeWindowHandle) override;

private:
    struct WindowRecord
    {
        WindowIdentifier identifier    = invalidWindowIdentifier;
        GLFWwindow      *window        = nullptr;
        bool             closeReported = false;
    };

    WindowRecord *findRecord(WindowIdentifier windowIdentifier);
    void          installCallbacks(GLFWwindow *window);

    static void framebufferResizeCallback(GLFWwindow *window, int width, int height);
    static void errorCallback(int errorCode, const char *description);

    ComponentManifest                        componentManifest;
    ComponentContext                        *boundContext = nullptr;
    LoggerCapability                        *logger       = nullptr;
    WindowDescription                        pendingDescription;
    bool                                     glfwInitialized = false;
    std::map<WindowIdentifier, WindowRecord> windows;
    WindowIdentifier                         nextWindowIdentifier = 1;
};

} // namespace engine

#endif // ENGINE_MODULES_WINDOW_HPP
