#include "window.hpp"

#include <vulkan/vulkan.h> // window module implementation detail, not part of the window contract
#include <GLFW/glfw3.h>

namespace engine
{

namespace
{

// GLFW callbacks carry no user data for the error path; the active module is
// tracked here for the duration of start() .. stop().
WindowModule* activeWindowModule = nullptr;

std::string describeGlfwError(int errorCode)
{
    switch (errorCode)
    {
    case GLFW_NOT_INITIALIZED:
        return "GLFW_NOT_INITIALIZED";
    case GLFW_NO_CURRENT_CONTEXT:
        return "GLFW_NO_CURRENT_CONTEXT";
    case GLFW_INVALID_ENUM:
        return "GLFW_INVALID_ENUM";
    case GLFW_INVALID_VALUE:
        return "GLFW_INVALID_VALUE";
    case GLFW_OUT_OF_MEMORY:
        return "GLFW_OUT_OF_MEMORY";
    case GLFW_API_UNAVAILABLE:
        return "GLFW_API_UNAVAILABLE";
    case GLFW_VERSION_UNAVAILABLE:
        return "GLFW_VERSION_UNAVAILABLE";
    case GLFW_PLATFORM_ERROR:
        return "GLFW_PLATFORM_ERROR";
    case GLFW_FORMAT_UNAVAILABLE:
        return "GLFW_FORMAT_UNAVAILABLE";
    default:
        return "GLFW_ERROR_" + std::to_string(errorCode);
    }
}

} // namespace

WindowModule::WindowModule()
{
    componentManifest.name = "window";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"window"};
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions = {"capability.logging"};
    componentManifest.knownConfigKeys = {"window.title", "window.width", "window.height", "window.resizable"};
}

WindowModule::~WindowModule()
{
    stop();
}

void WindowModule::construct(ComponentContext& context)
{
    boundContext = &context;
    logger = &context.acquireCapability<LoggerCapability>();

    const std::string title = context.configurationString("title").value_or("engine");
    const auto width = context.configurationInteger("width").value_or(1280);
    const auto height = context.configurationInteger("height").value_or(720);
    const bool resizable = context.configurationBoolean("resizable").value_or(true);

    // The main window is created during start so that GLFW initialization
    // happens inside the start phase, never during dry validation.
    pendingDescription.title = title;
    pendingDescription.width = static_cast<std::uint32_t>(width);
    pendingDescription.height = static_cast<std::uint32_t>(height);
    pendingDescription.resizable = resizable;
}

void WindowModule::start()
{
    if (!glfwInit())
    {
        throw std::runtime_error("window module: glfwInit failed");
    }
    glfwInitialized = true;
    activeWindowModule = this;
    glfwSetErrorCallback(&WindowModule::errorCallback);

    createMainWindow(pendingDescription);
    if (logger != nullptr)
    {
        logger->logMessage(LogLevel::Information, "window module started");
    }
}

void WindowModule::stop()
{
    for (auto& entry : windows)
    {
        if (entry.second.window != nullptr)
        {
            glfwDestroyWindow(entry.second.window);
            entry.second.window = nullptr;
        }
    }
    windows.clear();

    if (glfwInitialized)
    {
        glfwTerminate();
        glfwInitialized = false;
    }
    activeWindowModule = nullptr;
    boundContext = nullptr;
}

void* WindowModule::capabilityPointerFor(const std::string& capabilityName)
{
    if (capabilityName == WindowCapability::capabilityName)
    {
        return static_cast<WindowCapability*>(this);
    }
    return nullptr;
}

WindowIdentifier WindowModule::createMainWindow(const WindowDescription& description)
{
    if (!glfwInitialized)
    {
        throw std::runtime_error("window module: window creation requested before start");
    }

    // No OpenGL or OpenGL ES context: the renderer drives Vulkan exclusively.
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, description.resizable ? GLFW_TRUE : GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(static_cast<int>(description.width), static_cast<int>(description.height),
                                          description.title.c_str(), nullptr, nullptr);
    if (window == nullptr)
    {
        throw std::runtime_error("window module: glfwCreateWindow failed");
    }

    WindowRecord record;
    record.identifier = nextWindowIdentifier++;
    record.window = window;

    installCallbacks(window);
    windows.emplace(record.identifier, record);

    if (logger != nullptr)
    {
        logger->logMessage(LogLevel::Information,
                           "created window '" + description.title + "' (" + std::to_string(description.width) + "x" +
                               std::to_string(description.height) + ")");
    }
    return record.identifier;
}

bool WindowModule::windowCloseRequested(WindowIdentifier windowIdentifier)
{
    WindowRecord* record = findRecord(windowIdentifier);
    if (record == nullptr || record->window == nullptr)
    {
        return true;
    }
    return glfwWindowShouldClose(record->window) != 0;
}

void WindowModule::pollEvents()
{
    if (!glfwInitialized)
    {
        return;
    }
    glfwPollEvents();

    // Report close requests exactly once per window.
    for (auto& entry : windows)
    {
        WindowRecord& record = entry.second;
        if (!record.closeReported && record.window != nullptr && glfwWindowShouldClose(record.window))
        {
            record.closeReported = true;
            if (boundContext != nullptr)
            {
                WindowClosedEvent event;
                event.windowIdentifier = record.identifier;
                boundContext->eventBus().publish(event);
            }
        }
    }
}

std::uint32_t WindowModule::framebufferWidth(WindowIdentifier windowIdentifier)
{
    WindowRecord* record = findRecord(windowIdentifier);
    if (record == nullptr || record->window == nullptr)
    {
        return 0;
    }
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(record->window, &width, &height);
    return static_cast<std::uint32_t>(width);
}

std::uint32_t WindowModule::framebufferHeight(WindowIdentifier windowIdentifier)
{
    WindowRecord* record = findRecord(windowIdentifier);
    if (record == nullptr || record->window == nullptr)
    {
        return 0;
    }
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(record->window, &width, &height);
    return static_cast<std::uint32_t>(height);
}

std::vector<WindowIdentifier> WindowModule::windowIdentifiers() const
{
    std::vector<WindowIdentifier> identifiers;
    identifiers.reserve(windows.size());
    for (const auto& entry : windows)
    {
        identifiers.push_back(entry.first);
    }
    return identifiers;
}

std::vector<std::string> WindowModule::requiredInstanceExtensions(const std::string& graphicsApi)
{
    if (graphicsApi != "vulkan")
    {
        throw std::invalid_argument("window module: unsupported graphics API family '" + graphicsApi + "'");
    }
    if (!glfwInitialized)
    {
        return {};
    }
    std::uint32_t extensionCount = 0;
    const char** extensionNames = glfwGetRequiredInstanceExtensions(&extensionCount);
    std::vector<std::string> extensions;
    extensions.reserve(extensionCount);
    for (std::uint32_t index = 0; index < extensionCount; ++index)
    {
        extensions.emplace_back(extensionNames[index]);
    }
    return extensions;
}

void* WindowModule::createPresentationSurface(const std::string& graphicsApi, void* graphicsInstance,
                                              WindowIdentifier windowIdentifier)
{
    if (graphicsApi != "vulkan")
    {
        throw std::invalid_argument("window module: unsupported graphics API family '" + graphicsApi + "'");
    }
    WindowRecord* record = findRecord(windowIdentifier);
    if (record == nullptr || record->window == nullptr)
    {
        throw std::runtime_error("window module: surface requested for an unknown window");
    }
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    const VkResult result =
        glfwCreateWindowSurface(static_cast<VkInstance>(graphicsInstance), record->window, nullptr, &surface);
    if (result != VK_SUCCESS)
    {
        throw std::runtime_error("window module: glfwCreateWindowSurface failed with VkResult " +
                                 std::to_string(static_cast<int>(result)));
    }
    return surface;
}

void* WindowModule::nativeWindowHandle(WindowIdentifier windowIdentifier)
{
    WindowRecord* record = findRecord(windowIdentifier);
    if (record == nullptr)
    {
        return nullptr;
    }
    return record->window;
}

WindowIdentifier WindowModule::windowIdentifierFromNativeHandle(void* nativeWindowHandle)
{
    for (const auto& entry : windows)
    {
        if (entry.second.window == nativeWindowHandle)
        {
            return entry.first;
        }
    }
    return invalidWindowIdentifier;
}

WindowModule::WindowRecord* WindowModule::findRecord(WindowIdentifier windowIdentifier)
{
    const auto found = windows.find(windowIdentifier);
    if (found == windows.end())
    {
        return nullptr;
    }
    return &found->second;
}

void WindowModule::installCallbacks(GLFWwindow* window)
{
    glfwSetFramebufferSizeCallback(window, &WindowModule::framebufferResizeCallback);
}

void WindowModule::framebufferResizeCallback(GLFWwindow* window, int width, int height)
{
    if (activeWindowModule == nullptr || activeWindowModule->boundContext == nullptr)
    {
        return;
    }
    const WindowIdentifier identifier = activeWindowModule->windowIdentifierFromNativeHandle(window);
    if (identifier == invalidWindowIdentifier)
    {
        return;
    }
    WindowResizedEvent event;
    event.windowIdentifier = identifier;
    event.width = static_cast<std::uint32_t>(width);
    event.height = static_cast<std::uint32_t>(height);
    activeWindowModule->boundContext->eventBus().publish(event);
}

void WindowModule::errorCallback(int errorCode, const char* description)
{
    if (activeWindowModule != nullptr && activeWindowModule->logger != nullptr)
    {
        activeWindowModule->logger->logMessage(LogLevel::Error, "window module: " + describeGlfwError(errorCode) +
                                                                    " (" + std::to_string(errorCode) + "): " +
                                                                    (description != nullptr ? description : ""));
    }
}

} // namespace engine
