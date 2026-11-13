// window module implementation: the four capability tables, backend
// selection (GLFW preferred, headless fallback), and the facade that
// translates capability calls into backend calls. [window] configuration
// parsing lives in window_configuration.*; per-window record rules and the
// backend interface live in backend/window_backend.hpp.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/window/capabilities.hpp>
#include <engine/util/ppm.hpp>

#include "window_configuration.hpp"

#if defined(ENGINE_WINDOW_BACKEND_GLFW)
#include "glfw_backend.hpp"
#endif
#include "headless_window_backend.hpp"
#include "window_backend.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace engine
{
namespace window
{
namespace
{

// Module entry: provides the four window capabilities, reads [window]
// configuration in construct(), lazily initializes a backend on the first
// createWindow, and owns all windows until stop().
class WindowModule final : public ModuleInterface
{
public:
    WindowModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads the configured defaults and subscribes to the configuration
    // reload event so later [window] changes take effect.
    void construct(ParticipantContext &context) override
    {
        context_ = &context;
        readConfiguration();
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<WindowModule, &WindowModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    void stop() override
    {
        // Windows and the backend must die before the event subscription so
        // no callback can touch them afterwards.
        destroyAllWindows();
        shutdownBackend();
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == WindowCapability::capabilityName)
        {
            return &windowCapability_;
        }
        if (capabilityName == VulkanSurfaceCapability::capabilityName)
        {
            return &vulkanCapability_;
        }
        if (capabilityName == GlfwWindowCapability::capabilityName)
        {
            return &glfwCapability_;
        }
        if (capabilityName == HeadlessWindowCapability::capabilityName)
        {
            return &headlessCapability_;
        }
        return nullptr;
    }

private:
    // Re-reads the [window] section and forwards the runtime gamepad knobs
    // to the active backend (they apply immediately; creation defaults only
    // affect windows created afterwards).
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        configuration_ = readWindowModuleConfiguration(*context_);
        if (activeBackend_ != nullptr)
        {
            applyGamepadOptions();
        }
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        if (touchesWindowConfiguration(event))
        {
            readConfiguration();
        }
    }

    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "window",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"window", "window.vulkan", "window.glfw", "window.headless"}};
    }

    static void reportError(WindowError *error, WindowError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
    }

    WindowRecord *recordFor(WindowHandle window)
    {
        const auto found = windows_.find(window);
        return found != windows_.end() ? &found->second : nullptr;
    }

    const WindowRecord *recordFor(WindowHandle window) const
    {
        const auto found = windows_.find(window);
        return found != windows_.end() ? &found->second : nullptr;
    }

    // Lazily picks the backend: GLFW when allowed (backend "auto"/"glfw" and
    // the GLFW build is available), otherwise the headless backend. A forced
    // "glfw" that cannot initialize fails; "auto" falls back with a note.
    bool ensureBackend()
    {
        if (activeBackend_ != nullptr)
        {
            return true;
        }

        const bool glfwAllowed = configuration_.backend.empty() || configuration_.backend == "glfw";
        if (glfwAllowed)
        {
#if defined(ENGINE_WINDOW_BACKEND_GLFW)
            auto glfwBackend = std::make_unique<GlfwWindowBackend>();
            if (glfwBackend->initialize())
            {
                activeBackend_ = std::move(glfwBackend);
                applyGamepadOptions();
                return true;
            }
#endif
            if (configuration_.backend == "glfw")
            {
                return false;
            }
            std::fprintf(stderr, "window module: GLFW is unavailable; falling back to the headless backend\n");
        }

        auto headlessBackend = std::make_unique<HeadlessWindowBackend>();
        if (!headlessBackend->initialize())
        {
            return false;
        }
        activeBackend_ = std::move(headlessBackend);
        return true;
    }

    void applyGamepadOptions()
    {
        if (activeBackend_ != nullptr)
        {
            activeBackend_->configureGamepads(
                GamepadPollingOptions{configuration_.gamepadPolling, configuration_.gamepadAxisEpsilon});
        }
    }

    void shutdownBackend()
    {
        if (activeBackend_ != nullptr)
        {
            activeBackend_->shutdown();
            activeBackend_ = nullptr;
        }
    }

    void destroyAllWindows()
    {
        if (activeBackend_ != nullptr)
        {
            for (auto &entry : windows_)
            {
                activeBackend_->destroyWindow(entry.second);
            }
        }
        windows_.clear();
    }

    const char *activeBackendNameImpl() const
    {
        return activeBackend_ != nullptr ? activeBackend_->name() : "none";
    }

    SurfaceApi surfaceApiImpl(WindowHandle window) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return SurfaceApi::None;
        }
        return activeBackend_->surfaceApi(*record);
    }

    // Resolves the effective description (configured values fill fields the
    // caller left unset), clamps the size into the effective limits, then asks
    // the backend to create the OS window. The half-built record is removed on
    // any failure and CreationFailed is reported.
    WindowHandle createWindow(const WindowDescription &description, WindowError *error)
    {
        if (!ensureBackend())
        {
            reportError(error, WindowError::BackendUnavailable);
            return invalidWindow;
        }

        if (nextWindowId_ == invalidWindow)
        {
            reportError(error, WindowError::CreationFailed);
            return invalidWindow;
        }

        WindowRecord *recordPointer = nullptr;
        try
        {
            recordPointer = &windows_[nextWindowId_];
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, WindowError::CreationFailed);
            return invalidWindow;
        }
        WindowRecord &record                   = *recordPointer;
        record                                 = WindowRecord{};
        record.id                              = nextWindowId_;
        WindowDescription effectiveDescription = description;
        if (configuration_.resizable.has_value() && !effectiveDescription.hasResizable)
        {
            effectiveDescription.resizable = *configuration_.resizable;
        }
        if (configuration_.visible.has_value() && !effectiveDescription.hasVisible)
        {
            effectiveDescription.visible = *configuration_.visible;
        }
        if (configuration_.decorated.has_value() && !effectiveDescription.hasDecorated)
        {
            effectiveDescription.decorated = *configuration_.decorated;
        }
        if (configuration_.floating.has_value() && !effectiveDescription.hasFloating)
        {
            effectiveDescription.floating = *configuration_.floating;
        }
        if (configuration_.hasMaximized && !effectiveDescription.hasMaximized)
        {
            effectiveDescription.maximized    = configuration_.maximized;
            effectiveDescription.hasMaximized = true;
        }
        if (configuration_.hasFocusOnShow && !effectiveDescription.hasFocusOnShow)
        {
            effectiveDescription.focusOnShow    = configuration_.focusOnShow;
            effectiveDescription.hasFocusOnShow = true;
        }
        if (configuration_.hasOpacity && !effectiveDescription.hasOpacity)
        {
            effectiveDescription.opacity    = configuration_.opacity;
            effectiveDescription.hasOpacity = true;
        }
        if (!effectiveDescription.hasPosition && configuration_.hasPosition)
        {
            effectiveDescription.hasPosition = true;
            effectiveDescription.x           = configuration_.positionX;
            effectiveDescription.y           = configuration_.positionY;
        }
        if (!effectiveDescription.ignoreConfiguredLimits)
        {
            if (effectiveDescription.minimumWidth == 0)
            {
                effectiveDescription.minimumWidth = configuration_.minimumWidth;
            }
            if (effectiveDescription.minimumHeight == 0)
            {
                effectiveDescription.minimumHeight = configuration_.minimumHeight;
            }
            if (effectiveDescription.maximumWidth == 0)
            {
                effectiveDescription.maximumWidth = configuration_.maximumWidth;
            }
            if (effectiveDescription.maximumHeight == 0)
            {
                effectiveDescription.maximumHeight = configuration_.maximumHeight;
            }
        }

        record.width  = description.width != 0 ? description.width : configuration_.width;
        record.height = description.height != 0 ? description.height : configuration_.height;
        try
        {
            record.title = description.title != nullptr ? description.title : configuration_.title;
        }
        catch (const std::bad_alloc &)
        {
            windows_.erase(nextWindowId_);
            reportError(error, WindowError::CreationFailed);
            return invalidWindow;
        }
        // Pure has-flag semantics: the caller's capacity is used only when
        // hasEventQueueCapacity is set, so every value (including a deliberate
        // 4096 or the unbounded 0) reaches the queue. Without the flag the
        // configured default applies.
        record.eventQueueCapacity
            = description.hasEventQueueCapacity ? description.eventQueueCapacity : configuration_.eventQueueCapacity;
        record.minimumWidth  = effectiveDescription.minimumWidth;
        record.minimumHeight = effectiveDescription.minimumHeight;
        record.maximumWidth  = effectiveDescription.maximumWidth;
        record.maximumHeight = effectiveDescription.maximumHeight;

        // Reject limits above INT_MAX or min > max, and sizes whose pixel
        // area (width * height * 4 bytes) would overflow.
        const std::uint32_t maximumDimension = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
        const std::size_t   maximumPixels    = std::numeric_limits<std::size_t>::max() / 4;
        const bool invalidLimits             = effectiveDescription.minimumWidth > maximumDimension
                                            || effectiveDescription.minimumHeight > maximumDimension
                                            || effectiveDescription.maximumWidth > maximumDimension
                                            || effectiveDescription.maximumHeight > maximumDimension
                                            || (effectiveDescription.maximumWidth != 0
                                    && effectiveDescription.minimumWidth > effectiveDescription.maximumWidth)
                                            || (effectiveDescription.maximumHeight != 0
                                    && effectiveDescription.minimumHeight > effectiveDescription.maximumHeight);
        if (effectiveDescription.minimumWidth != 0)
        {
            record.width = std::max(record.width, effectiveDescription.minimumWidth);
        }
        if (effectiveDescription.maximumWidth != 0)
        {
            record.width = std::min(record.width, effectiveDescription.maximumWidth);
        }
        if (effectiveDescription.minimumHeight != 0)
        {
            record.height = std::max(record.height, effectiveDescription.minimumHeight);
        }
        if (effectiveDescription.maximumHeight != 0)
        {
            record.height = std::min(record.height, effectiveDescription.maximumHeight);
        }
        if (record.width == 0 || record.height == 0 || record.width > maximumDimension
            || record.height > maximumDimension
            || static_cast<std::size_t>(record.width) > maximumPixels / record.height || invalidLimits)
        {
            windows_.erase(nextWindowId_);
            reportError(error, WindowError::CreationFailed);
            return invalidWindow;
        }

        if (!activeBackend_->createWindow(record, effectiveDescription))
        {
            windows_.erase(nextWindowId_);
            reportError(error, WindowError::CreationFailed);
            return invalidWindow;
        }

        const WindowHandle id = record.id;
        ++nextWindowId_;
        reportError(error, WindowError::None);
        return id;
    }

    void destroyWindowImpl(WindowHandle window)
    {
        const auto found = windows_.find(window);
        if (found == windows_.end())
        {
            return;
        }
        if (activeBackend_ != nullptr)
        {
            activeBackend_->destroyWindow(found->second);
        }
        windows_.erase(found);
    }

    // Unknown windows report "should close" so callers' main loops terminate
    // after an invalid handle.
    bool shouldCloseImpl(WindowHandle window) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr)
        {
            return true;
        }
        return activeBackend_ != nullptr && activeBackend_->shouldClose(*record);
    }

    void requestCloseImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->requestClose(*record);
    }

    void setTitleImpl(WindowHandle window, const char *title)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setTitle(*record, title);
    }

    // Clamps into the record's limits before forwarding; zero or oversized
    // sizes fail without touching the backend.
    bool setSizeImpl(WindowHandle window, std::uint32_t width, std::uint32_t height)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr || width == 0 || height == 0
            || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
            || height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        {
            return false;
        }
        if (record->minimumWidth != 0)
            width = std::max(width, record->minimumWidth);
        if (record->minimumHeight != 0)
            height = std::max(height, record->minimumHeight);
        if (record->maximumWidth != 0)
            width = std::min(width, record->maximumWidth);
        if (record->maximumHeight != 0)
            height = std::min(height, record->maximumHeight);
        return activeBackend_->setSize(*record, width, height);
    }

    bool setPositionImpl(WindowHandle window, std::int32_t x, std::int32_t y)
    {
        WindowRecord *record = recordFor(window);
        return record != nullptr && activeBackend_ != nullptr && activeBackend_->setPosition(*record, x, y);
    }

    bool getPositionImpl(WindowHandle window, std::int32_t *outX, std::int32_t *outY) const
    {
        const WindowRecord *record = recordFor(window);
        return record != nullptr && activeBackend_ != nullptr && activeBackend_->getPosition(*record, outX, outY);
    }

    void setOpacityImpl(WindowHandle window, float opacity)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setOpacity(*record, opacity);
    }

    void setVisibleImpl(WindowHandle window, bool visible)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setVisible(*record, visible);
    }

    void setResizableImpl(WindowHandle window, bool resizable)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setResizable(*record, resizable);
    }

    void setDecoratedImpl(WindowHandle window, bool decorated)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setDecorated(*record, decorated);
    }

    void setFloatingImpl(WindowHandle window, bool floating)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setFloating(*record, floating);
    }

    void focusImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->focus(*record);
    }

    void requestAttentionImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->requestAttention(*record);
    }

    void iconifyImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->iconify(*record);
    }

    void restoreImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->restore(*record);
    }

    void maximizeImpl(WindowHandle window)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->maximize(*record);
    }

    bool getWindowAttributesImpl(WindowHandle window, WindowAttributes *outAttributes) const
    {
        const WindowRecord *record = recordFor(window);
        return record != nullptr && activeBackend_ != nullptr
            && activeBackend_->getWindowAttributes(*record, outAttributes);
    }

    void getSizeImpl(WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight) const
    {
        const WindowRecord *record = recordFor(window);
        if (outWidth != nullptr)
        {
            *outWidth = record != nullptr ? record->width : 0;
        }
        if (outHeight != nullptr)
        {
            *outHeight = record != nullptr ? record->height : 0;
        }
    }

    void getFramebufferSizeImpl(WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight) const
    {
        const WindowRecord *record = recordFor(window);
        if (outWidth != nullptr)
        {
            *outWidth = record != nullptr ? record->framebufferWidth : 0;
        }
        if (outHeight != nullptr)
        {
            *outHeight = record != nullptr ? record->framebufferHeight : 0;
        }
    }

    bool getContentScaleImpl(WindowHandle window, float *outX, float *outY) const
    {
        const WindowRecord *record = recordFor(window);
        return record != nullptr && activeBackend_ != nullptr && activeBackend_->getContentScale(*record, outX, outY);
    }

    bool rawMouseMotionSupportedImpl() const
    {
        return activeBackend_ != nullptr && activeBackend_->rawMouseMotionSupported();
    }

    void pollEventsImpl()
    {
        if (activeBackend_ != nullptr)
        {
            activeBackend_->pollEvents();
        }
    }

    void waitEventsImpl(double timeoutSeconds)
    {
        if (activeBackend_ != nullptr)
        {
            activeBackend_->waitEvents(timeoutSeconds);
        }
    }

    bool popEventImpl(WindowHandle window, WindowEvent *outEvent)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || outEvent == nullptr || record->events.empty())
        {
            return false;
        }
        *outEvent = record->events.front();
        record->events.pop_front();
        return true;
    }

    std::uint32_t popEventsImpl(WindowHandle window, WindowEvent *outEvents, std::uint32_t capacity)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || outEvents == nullptr || capacity == 0)
        {
            return 0;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(std::min<std::size_t>(record->events.size(), capacity));
        for (std::uint32_t index = 0; index < count; ++index)
        {
            outEvents[index] = record->events.front();
            record->events.pop_front();
        }
        return count;
    }

    bool isKeyDownImpl(WindowHandle window, Key key) const
    {
        const WindowRecord *record = recordFor(window);
        const std::uint32_t value  = static_cast<std::uint32_t>(key);
        return record != nullptr && value != 0 && value < 64
            && (record->keyStateMask & (std::uint64_t{1} << value)) != 0;
    }

    std::uint64_t keyStateMaskImpl(WindowHandle window, std::uint64_t requestedMask) const
    {
        const WindowRecord *record = recordFor(window);
        return record != nullptr ? record->keyStateMask & requestedMask : 0;
    }

    bool isMouseButtonDownImpl(WindowHandle window, MouseButton button) const
    {
        const WindowRecord *record = recordFor(window);
        const std::uint32_t value  = static_cast<std::uint32_t>(button);
        return record != nullptr && value < 8
            && (record->mouseButtonStateMask & (std::uint8_t{1} << value)) != 0;
    }

    // Headless windows only: maps the software RGBA buffer; GLFW windows
    // have no CPU surface to lock.
    bool lockSurfacePixelsImpl(WindowHandle window, SurfacePixels *outPixels)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr || outPixels == nullptr || record->headlessRgba.empty())
        {
            return false;
        }
        outPixels->rgba        = record->headlessRgba.data();
        outPixels->width       = record->framebufferWidth;
        outPixels->height      = record->framebufferHeight;
        outPixels->strideBytes = static_cast<std::size_t>(record->framebufferWidth) * 4;
        return true;
    }

    void unlockSurfacePixelsImpl(WindowHandle)
    {}

    void setCursorModeImpl(WindowHandle window, CursorMode mode)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return;
        }
        activeBackend_->setCursorMode(*record, mode);
    }

    bool cursorPositionImpl(WindowHandle window, double *outX, double *outY) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return false;
        }
        return activeBackend_->cursorPosition(*record, outX, outY);
    }

    bool setClipboardTextImpl(WindowHandle window, const char *text)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr || text == nullptr)
            return false;
        return activeBackend_->setClipboardText(*record, text);
    }

    // Borrows the backend's clipboard text, reports the required buffer size
    // (including NUL) through outRequired, and copies only when the caller's
    // buffer is large enough; too-small buffers are left untouched.
    bool copyClipboardTextImpl(WindowHandle window, char *output, std::uint32_t capacity,
        std::uint32_t *outRequired) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
            return false;
        const char *text = nullptr;
        if (!activeBackend_->copyClipboardText(*record, &text) || text == nullptr)
            return false;
        std::size_t length = 0;
        while (length <= maximumClipboardTextBytes && text[length] != '\0')
            ++length;
        if (length > maximumClipboardTextBytes || length + 1 > std::numeric_limits<std::uint32_t>::max())
            return false;
        const auto required = static_cast<std::uint32_t>(length + 1);
        if (outRequired != nullptr)
            *outRequired = required;
        if (output == nullptr || capacity < required)
            return false;
        std::copy(text, text + length, output);
        output[length] = '\0';
        return true;
    }

    bool submitTouchEventImpl(WindowHandle window, const TouchEventDescription *description)
    {
        WindowRecord *record = recordFor(window);
        return record != nullptr && description != nullptr && submitTouchContact(*record, *description);
    }

    bool copyDroppedFilesImpl(WindowHandle window, std::uint64_t batchId,
        char *output, std::uint32_t capacity, std::uint32_t *outRequired) const
    {
        const WindowRecord *record = recordFor(window);
        return record != nullptr && copyDroppedFileBatch(*record, batchId, output, capacity, outRequired);
    }

    bool releaseDroppedFilesImpl(WindowHandle window, std::uint64_t batchId)
    {
        WindowRecord *record = recordFor(window);
        return record != nullptr && releaseDroppedFileBatch(*record, batchId);
    }

    bool submitFileDropEventImpl(WindowHandle window, const char *const *paths, std::uint32_t count)
    {
        WindowRecord *record = recordFor(window);
        return record != nullptr && queueDroppedFileBatch(*record, paths, count);
    }

    const char *const *requiredInstanceExtensionsImpl(std::uint32_t *outCount) const
    {
        if (activeBackend_ == nullptr)
        {
            if (outCount != nullptr)
            {
                *outCount = 0;
            }
            return nullptr;
        }
        return activeBackend_->requiredInstanceExtensions(outCount);
    }

    bool nativeSurfaceHandlesImpl(WindowHandle window, NativeWindowHandles *outHandles) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
        {
            return false;
        }
        return activeBackend_->nativeSurfaceHandles(*record, outHandles);
    }

    const char *glfwVersionImpl() const
    {
#if defined(ENGINE_WINDOW_BACKEND_GLFW)
        return glfwBackendVersionString();
#else
        return nullptr;
#endif
    }

    // Raw GLFWwindow* for the window.glfw capability; null for headless
    // windows or when GLFW support is not compiled in.
    void *glfwWindowPointerImpl(WindowHandle window) const
    {
#if defined(ENGINE_WINDOW_BACKEND_GLFW)
        const WindowRecord *record = recordFor(window);
        if (record != nullptr && record->platformWindow != nullptr)
        {
            return record->platformWindow;
        }
#else
        (void)window;
#endif
        return nullptr;
    }

    bool nativeHandlesImpl(WindowHandle window, NativeWindowHandles *outHandles) const
    {
#if defined(ENGINE_WINDOW_BACKEND_GLFW)
        return nativeSurfaceHandlesImpl(window, outHandles);
#else
        (void)window;
        (void)outHandles;
        return false;
#endif
    }

    // Writes the software surface as a binary P6 PPM (row-wise RGB).
    // Unsupported when no headless surface exists, CreationFailed on I/O
    // errors.
    bool exportSurfacePpmImpl(WindowHandle window, const char *filePath, WindowError *error) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || record->headlessRgba.empty())
        {
            reportError(error, WindowError::Unsupported);
            return false;
        }
        if (filePath == nullptr)
        {
            reportError(error, WindowError::CreationFailed);
            return false;
        }
        const std::uint32_t width  = record->framebufferWidth;
        const std::uint32_t height = record->framebufferHeight;
        if (static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 4)
        {
            reportError(error, WindowError::CreationFailed);
            return false;
        }
        const std::size_t stride = static_cast<std::size_t>(width) * 4;
        if (!util::writePpmRgba(filePath, width, height, record->headlessRgba.data(), stride, false, false))
        {
            reportError(error, WindowError::CreationFailed);
            return false;
        }
        reportError(error, WindowError::None);
        return true;
    }

    // Wires every capability function pointer to a lambda that forwards to
    // the matching *Impl method on this module (same pattern as the ui
    // module); the void* self is this WindowModule instance.
    void assignCapabilityPointers()
    {
        windowCapability_.self   = this;
        vulkanCapability_.self   = this;
        glfwCapability_.self     = this;
        headlessCapability_.self = this;
#define ENGINE_WINDOW_BIND(capability, field, method)                                                          \
    capability.field = [](void *self, auto... args) -> decltype(auto)                                          \
    { return static_cast<WindowModule *>(self)->method(args...); }

        ENGINE_WINDOW_BIND(windowCapability_, activeBackendName, activeBackendNameImpl);
        ENGINE_WINDOW_BIND(windowCapability_, surfaceApi, surfaceApiImpl);
        ENGINE_WINDOW_BIND(windowCapability_, createWindow, createWindow);
        ENGINE_WINDOW_BIND(windowCapability_, destroyWindow, destroyWindowImpl);
        ENGINE_WINDOW_BIND(windowCapability_, shouldClose, shouldCloseImpl);
        ENGINE_WINDOW_BIND(windowCapability_, requestClose, requestCloseImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setTitle, setTitleImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setSize, setSizeImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setPosition, setPositionImpl);
        ENGINE_WINDOW_BIND(windowCapability_, getPosition, getPositionImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setOpacity, setOpacityImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setVisible, setVisibleImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setResizable, setResizableImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setDecorated, setDecoratedImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setFloating, setFloatingImpl);
        ENGINE_WINDOW_BIND(windowCapability_, focus, focusImpl);
        ENGINE_WINDOW_BIND(windowCapability_, requestAttention, requestAttentionImpl);
        ENGINE_WINDOW_BIND(windowCapability_, iconify, iconifyImpl);
        ENGINE_WINDOW_BIND(windowCapability_, restore, restoreImpl);
        ENGINE_WINDOW_BIND(windowCapability_, maximize, maximizeImpl);
        ENGINE_WINDOW_BIND(windowCapability_, getWindowAttributes, getWindowAttributesImpl);
        ENGINE_WINDOW_BIND(windowCapability_, getSize, getSizeImpl);
        ENGINE_WINDOW_BIND(windowCapability_, getFramebufferSize, getFramebufferSizeImpl);
        ENGINE_WINDOW_BIND(windowCapability_, getContentScale, getContentScaleImpl);
        ENGINE_WINDOW_BIND(windowCapability_, rawMouseMotionSupported, rawMouseMotionSupportedImpl);
        ENGINE_WINDOW_BIND(windowCapability_, pollEvents, pollEventsImpl);
        ENGINE_WINDOW_BIND(windowCapability_, waitEvents, waitEventsImpl);
        ENGINE_WINDOW_BIND(windowCapability_, popEvent, popEventImpl);
        ENGINE_WINDOW_BIND(windowCapability_, popEvents, popEventsImpl);
        ENGINE_WINDOW_BIND(windowCapability_, isKeyDown, isKeyDownImpl);
        ENGINE_WINDOW_BIND(windowCapability_, keyStateMask, keyStateMaskImpl);
        ENGINE_WINDOW_BIND(windowCapability_, isMouseButtonDown, isMouseButtonDownImpl);
        ENGINE_WINDOW_BIND(windowCapability_, lockSurfacePixels, lockSurfacePixelsImpl);
        ENGINE_WINDOW_BIND(windowCapability_, unlockSurfacePixels, unlockSurfacePixelsImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setCursorMode, setCursorModeImpl);
        ENGINE_WINDOW_BIND(windowCapability_, cursorPosition, cursorPositionImpl);
        ENGINE_WINDOW_BIND(windowCapability_, setClipboardText, setClipboardTextImpl);
        ENGINE_WINDOW_BIND(windowCapability_, copyClipboardText, copyClipboardTextImpl);
        ENGINE_WINDOW_BIND(windowCapability_, submitTouchEvent, submitTouchEventImpl);
        ENGINE_WINDOW_BIND(windowCapability_, copyDroppedFiles, copyDroppedFilesImpl);
        ENGINE_WINDOW_BIND(windowCapability_, releaseDroppedFiles, releaseDroppedFilesImpl);
        ENGINE_WINDOW_BIND(windowCapability_, submitFileDropEvent, submitFileDropEventImpl);

        ENGINE_WINDOW_BIND(vulkanCapability_, requiredInstanceExtensions, requiredInstanceExtensionsImpl);
        ENGINE_WINDOW_BIND(vulkanCapability_, nativeSurfaceHandles, nativeSurfaceHandlesImpl);

        ENGINE_WINDOW_BIND(glfwCapability_, glfwVersion, glfwVersionImpl);
        ENGINE_WINDOW_BIND(glfwCapability_, glfwWindowPointer, glfwWindowPointerImpl);
        ENGINE_WINDOW_BIND(glfwCapability_, nativeHandles, nativeHandlesImpl);

        ENGINE_WINDOW_BIND(headlessCapability_, exportSurfacePpm, exportSurfacePpmImpl);
#undef ENGINE_WINDOW_BIND
    }

    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    // Parsed [window] section; re-read on construct and on every
    // ConfigurationReloadedEvent touching "window." keys.
    WindowModuleConfiguration configuration_;

    std::unique_ptr<WindowBackend>       activeBackend_;
    std::map<WindowHandle, WindowRecord> windows_;
    WindowHandle                         nextWindowId_ = 1;

    ParticipantManifest manifest_;

    WindowCapability         windowCapability_{};
    VulkanSurfaceCapability  vulkanCapability_{};
    GlfwWindowCapability     glfwCapability_{};
    HeadlessWindowCapability headlessCapability_{};
};

} // namespace

} // namespace window
} // namespace engine

// Module entry point queried by the kernel after dlopen: provides all four
// window capabilities; no required/optional capabilities, no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"window", "window.vulkan", "window.glfw", "window.headless"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "window",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::window::WindowModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 4,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
