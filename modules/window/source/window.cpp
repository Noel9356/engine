// window module implementation: the four capability tables, [window]
// configuration handling, backend selection (GLFW preferred, headless
// fallback), and the facade that translates capability calls into backend
// calls.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/window/capabilities.hpp>
#include <engine/util/ppm.hpp>

#if defined(ENGINE_WINDOW_BACKEND_GLFW)
#include "glfw_backend.hpp"
#endif
#include "headless_window_backend.hpp"
#include "window_backend.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace engine
{
namespace window
{
namespace
{

// Fallbacks used when [window] configuration does not say otherwise.
constexpr std::uint32_t defaultWindowWidth  = 1280;
constexpr std::uint32_t defaultWindowHeight = 720;
constexpr const char   *defaultWindowTitle  = "engine";

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
    // Re-reads the [window] section into the configured* defaults. Invalid
    // values are reported on stderr and ignored, keeping the built-in
    // default; booleans use optional<> because "unset" must stay
    // distinguishable from false when a window is created.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;
        configuredBackend_.clear();
        configuredWidth_              = defaultWindowWidth;
        configuredHeight_             = defaultWindowHeight;
        configuredMinimumWidth_       = 0;
        configuredMinimumHeight_      = 0;
        configuredMaximumWidth_       = 0;
        configuredMaximumHeight_      = 0;
        configuredEventQueueCapacity_ = 4096;
        configuredPositionX_          = 0;
        configuredPositionY_          = 0;
        configuredHasPosition_        = false;
        configuredResizable_.reset();
        configuredVisible_.reset();
        configuredDecorated_.reset();
        configuredFloating_.reset();
        configuredTitle_ = defaultWindowTitle;

        // Only "glfw" and "headless" are known; unknown values warn and
        // behave as "auto".
        const std::optional<std::string> backend = context.configurationStringInSection("window", "backend");
        if (backend.has_value() && *backend != "auto")
        {
            if (*backend != "glfw" && *backend != "headless")
            {
                std::fprintf(
                    stderr, "window module: unknown [window] backend '%s'; using \"auto\"\n", backend->c_str());
            }
            else
            {
                configuredBackend_ = *backend;
            }
        }

        // Sizes must be positive; limits and queue capacity are clamped to
        // their integer ranges (0 = no limit / default queue).
        const std::optional<std::int64_t> width = context.configurationIntegerInSection("window", "width");
        if (width.has_value() && *width > 0 && *width <= std::numeric_limits<std::uint32_t>::max())
        {
            configuredWidth_ = (std::uint32_t)*width;
        }
        else if (width.has_value())
        {
            std::fprintf(
                stderr, "window module: ignoring [window] width %lld; not a positive size\n", (long long)*width);
        }
        const std::optional<std::int64_t> height = context.configurationIntegerInSection("window", "height");
        if (height.has_value() && *height > 0 && *height <= std::numeric_limits<std::uint32_t>::max())
        {
            configuredHeight_ = (std::uint32_t)*height;
        }
        else if (height.has_value())
        {
            std::fprintf(
                stderr, "window module: ignoring [window] height %lld; not a positive size\n", (long long)*height);
        }
        const std::optional<std::string> title = context.configurationStringInSection("window", "title");
        if (title.has_value() && !title->empty())
        {
            configuredTitle_ = *title;
        }
        configuredResizable_ = context.configurationBooleanInSection("window", "resizable");
        configuredVisible_   = context.configurationBooleanInSection("window", "visible");
        configuredDecorated_ = context.configurationBooleanInSection("window", "decorated");
        configuredFloating_  = context.configurationBooleanInSection("window", "floating");

        const auto readDimension = [&context](const char *key, std::uint32_t &destination)
        {
            const std::optional<std::int64_t> value = context.configurationIntegerInSection("window", key);
            if (!value.has_value())
            {
                return;
            }
            if (*value < 0 || *value > std::numeric_limits<int>::max())
            {
                std::fprintf(stderr,
                    "window module: ignoring [window] %s; expected 0..%d\n",
                    key,
                    std::numeric_limits<int>::max());
                return;
            }
            destination = static_cast<std::uint32_t>(*value);
        };
        readDimension("minimum_width", configuredMinimumWidth_);
        readDimension("minimum_height", configuredMinimumHeight_);
        readDimension("maximum_width", configuredMaximumWidth_);
        readDimension("maximum_height", configuredMaximumHeight_);

        const std::optional<std::int64_t> queueCapacity
            = context.configurationIntegerInSection("window", "event_queue_capacity");
        if (queueCapacity.has_value() && *queueCapacity >= 0
            && static_cast<std::uint64_t>(*queueCapacity) <= std::numeric_limits<std::uint32_t>::max())
        {
            configuredEventQueueCapacity_ = static_cast<std::uint32_t>(*queueCapacity);
        }
        else if (queueCapacity.has_value())
        {
            std::fprintf(stderr,
                "window module: ignoring [window] event_queue_capacity; expected 0..%u\n",
                std::numeric_limits<std::uint32_t>::max());
        }

        // A position is only used when both coordinates are valid int32s.
        const std::optional<std::int64_t> positionX = context.configurationIntegerInSection("window", "position_x");
        const std::optional<std::int64_t> positionY = context.configurationIntegerInSection("window", "position_y");
        if (positionX.has_value() && positionY.has_value() && *positionX >= std::numeric_limits<std::int32_t>::min()
            && *positionX <= std::numeric_limits<std::int32_t>::max()
            && *positionY >= std::numeric_limits<std::int32_t>::min()
            && *positionY <= std::numeric_limits<std::int32_t>::max())
        {
            configuredPositionX_   = static_cast<std::int32_t>(*positionX);
            configuredPositionY_   = static_cast<std::int32_t>(*positionY);
            configuredHasPosition_ = true;
        }
        else if (positionX.has_value() || positionY.has_value())
        {
            std::fprintf(stderr,
                "window module: ignoring [window] position_x/position_y; both valid coordinates are required\n");
        }
    }

    // Re-reads configuration when any changed key starts with "window.".
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("window.", 0) == 0; });
        if (changed)
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

        const bool glfwAllowed = configuredBackend_.empty() || configuredBackend_ == "glfw";
        if (glfwAllowed)
        {
#if defined(ENGINE_WINDOW_BACKEND_GLFW)
            auto glfwBackend = std::make_unique<GlfwWindowBackend>();
            if (glfwBackend->initialize())
            {
                activeBackend_ = std::move(glfwBackend);
                return true;
            }
#endif
            if (configuredBackend_ == "glfw")
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
        if (configuredResizable_.has_value() && !effectiveDescription.hasResizable)
        {
            effectiveDescription.resizable = *configuredResizable_;
        }
        if (configuredVisible_.has_value() && !effectiveDescription.hasVisible)
        {
            effectiveDescription.visible = *configuredVisible_;
        }
        if (configuredDecorated_.has_value() && !effectiveDescription.hasDecorated)
        {
            effectiveDescription.decorated = *configuredDecorated_;
        }
        if (configuredFloating_.has_value() && !effectiveDescription.hasFloating)
        {
            effectiveDescription.floating = *configuredFloating_;
        }
        if (!effectiveDescription.hasPosition && configuredHasPosition_)
        {
            effectiveDescription.hasPosition = true;
            effectiveDescription.x           = configuredPositionX_;
            effectiveDescription.y           = configuredPositionY_;
        }
        if (!effectiveDescription.ignoreConfiguredLimits)
        {
            if (effectiveDescription.minimumWidth == 0)
            {
                effectiveDescription.minimumWidth = configuredMinimumWidth_;
            }
            if (effectiveDescription.minimumHeight == 0)
            {
                effectiveDescription.minimumHeight = configuredMinimumHeight_;
            }
            if (effectiveDescription.maximumWidth == 0)
            {
                effectiveDescription.maximumWidth = configuredMaximumWidth_;
            }
            if (effectiveDescription.maximumHeight == 0)
            {
                effectiveDescription.maximumHeight = configuredMaximumHeight_;
            }
        }

        record.width  = description.width != 0 ? description.width : configuredWidth_;
        record.height = description.height != 0 ? description.height : configuredHeight_;
        try
        {
            record.title = description.title != nullptr ? description.title : configuredTitle_;
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
        // configured default applies; the old "value != 4096" guess is gone.
        record.eventQueueCapacity
            = description.hasEventQueueCapacity ? description.eventQueueCapacity : configuredEventQueueCapacity_;
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

    void pollEventsImpl()
    {
        if (activeBackend_ != nullptr)
        {
            activeBackend_->pollEvents();
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
        const WindowRecord  *record = recordFor(window);
        const std::uint32_t  value  = static_cast<std::uint32_t>(button);
        return record != nullptr && value < 8
            && (record->mouseButtonStateMask & (std::uint8_t{1} << value)) != 0;
    }

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
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr || text == nullptr)
            return false;
        return activeBackend_->setClipboardText(*record, text);
    }

    bool copyClipboardTextImpl(WindowHandle window, char *output, std::uint32_t capacity,
        std::uint32_t *outRequired) const
    {
        const WindowRecord *record = recordFor(window);
        if (record == nullptr || activeBackend_ == nullptr)
            return false;
        std::string text;
        if (!activeBackend_->copyClipboardText(*record, text)
            || text.size() > maximumClipboardTextBytes
            || text.size() + 1 > std::numeric_limits<std::uint32_t>::max())
            return false;
        const auto required = static_cast<std::uint32_t>(text.size() + 1);
        if (outRequired != nullptr)
            *outRequired = required;
        if (output == nullptr || capacity < required)
            return false;
        std::copy(text.begin(), text.end(), output);
        output[text.size()] = '\0';
        return true;
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

    bool submitTouchEventImpl(WindowHandle window, const TouchEventDescription *description)
    {
        WindowRecord *record = recordFor(window);
        if (record == nullptr || description == nullptr
            || static_cast<std::uint32_t>(description->phase) > static_cast<std::uint32_t>(TouchPhase::Cancelled)
            || !std::isfinite(description->x) || !std::isfinite(description->y)
            || description->touchId == invalidTouchId || !std::isfinite(description->pressure) || description->x < 0.0f || description->x > 1.0f
            || description->y < 0.0f || description->y > 1.0f || description->pressure < 0.0f
            || description->pressure > 1.0f)
            return false;

        const std::uint64_t key = (static_cast<std::uint64_t>(description->device) << 32u) | description->touchId;
        auto contact = record->activeTouches.find(key);
        if (description->phase == TouchPhase::Began)
        {
            if (contact != record->activeTouches.end()
                || record->activeTouches.size() >= maximumTouchContactsPerWindow)
                return false;
            try
            {
                contact = record->activeTouches.emplace(
                    key, WindowRecord::TouchContact{description->x, description->y, description->pressure}).first;
            }
            catch (...)
            {
                return false;
            }
        }
        else if (contact == record->activeTouches.end())
        {
            return false;
        }

        const WindowRecord::TouchContact previousContact = contact->second;
        const float deltaX = description->phase == TouchPhase::Began ? 0.0f : description->x - contact->second.x;
        const float deltaY = description->phase == TouchPhase::Began ? 0.0f : description->y - contact->second.y;
        contact->second = {description->x, description->y, description->pressure};

        WindowEventKind kind = WindowEventKind::TouchBegan;
        switch (description->phase)
        {
        case TouchPhase::Began: kind = WindowEventKind::TouchBegan; break;
        case TouchPhase::Moved: kind = WindowEventKind::TouchMoved; break;
        case TouchPhase::Ended: kind = WindowEventKind::TouchEnded; break;
        case TouchPhase::Cancelled: kind = WindowEventKind::TouchCancelled; break;
        }
        const bool queued = queueWindowEvent(*record, WindowEvent{.kind = kind,
            .touchDevice = description->device,
            .touchId = description->touchId,
            .touchX = description->x,
            .touchY = description->y,
            .touchPressure = description->pressure,
            .touchDeltaX = deltaX,
            .touchDeltaY = deltaY});

        if (description->phase == TouchPhase::Ended || description->phase == TouchPhase::Cancelled)
        {
            record->activeTouches.erase(key);
        }
        else if (!queued)
        {
            if (description->phase == TouchPhase::Began)
                record->activeTouches.erase(key);
            else
                contact->second = previousContact;
        }
        return queued;
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

    void assignCapabilityPointers()
    {
        windowCapability_.self   = this;
        vulkanCapability_.self   = this;
        glfwCapability_.self     = this;
        headlessCapability_.self = this;

        windowCapability_.activeBackendName
            = [](void *self) { return static_cast<WindowModule *>(self)->activeBackendNameImpl(); };
        windowCapability_.createWindow = [](void *self, const WindowDescription &description, WindowError *error)
        { return static_cast<WindowModule *>(self)->createWindow(description, error); };
        windowCapability_.destroyWindow
            = [](void *self, WindowHandle window) { static_cast<WindowModule *>(self)->destroyWindowImpl(window); };
        windowCapability_.shouldClose = [](void *self, WindowHandle window)
        { return static_cast<WindowModule *>(self)->shouldCloseImpl(window); };
        windowCapability_.requestClose
            = [](void *self, WindowHandle window) { static_cast<WindowModule *>(self)->requestCloseImpl(window); };
        windowCapability_.setTitle = [](void *self, WindowHandle window, const char *title)
        { static_cast<WindowModule *>(self)->setTitleImpl(window, title); };
        windowCapability_.setSize = [](void *self, WindowHandle window, std::uint32_t width, std::uint32_t height)
        { return static_cast<WindowModule *>(self)->setSizeImpl(window, width, height); };
        windowCapability_.setPosition = [](void *self, WindowHandle window, std::int32_t x, std::int32_t y)
        { return static_cast<WindowModule *>(self)->setPositionImpl(window, x, y); };
        windowCapability_.getSize
            = [](void *self, WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight)
        { static_cast<WindowModule *>(self)->getSizeImpl(window, outWidth, outHeight); };
        windowCapability_.getFramebufferSize
            = [](void *self, WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight)
        { static_cast<WindowModule *>(self)->getFramebufferSizeImpl(window, outWidth, outHeight); };
        windowCapability_.pollEvents = [](void *self) { static_cast<WindowModule *>(self)->pollEventsImpl(); };
        windowCapability_.popEvent   = [](void *self, WindowHandle window, WindowEvent *outEvent)
        { return static_cast<WindowModule *>(self)->popEventImpl(window, outEvent); };
        windowCapability_.popEvents
            = [](void *self, WindowHandle window, WindowEvent *outEvents, std::uint32_t capacity)
        { return static_cast<WindowModule *>(self)->popEventsImpl(window, outEvents, capacity); };
        windowCapability_.isKeyDown = [](void *self, WindowHandle window, Key key)
        { return static_cast<WindowModule *>(self)->isKeyDownImpl(window, key); };
        windowCapability_.keyStateMask = [](void *self, WindowHandle window, std::uint64_t requestedMask)
        { return static_cast<WindowModule *>(self)->keyStateMaskImpl(window, requestedMask); };
        windowCapability_.isMouseButtonDown = [](void *self, WindowHandle window, MouseButton button)
        { return static_cast<const WindowModule *>(self)->isMouseButtonDownImpl(window, button); };
        windowCapability_.setCursorMode = [](void *self, WindowHandle window, CursorMode mode)
        { static_cast<WindowModule *>(self)->setCursorModeImpl(window, mode); };
        windowCapability_.cursorPosition = [](void *self, WindowHandle window, double *outX, double *outY)
        { return static_cast<const WindowModule *>(self)->cursorPositionImpl(window, outX, outY); };
        windowCapability_.setClipboardText = [](void *self, WindowHandle window, const char *text)
        { return static_cast<WindowModule *>(self)->setClipboardTextImpl(window, text); };
        windowCapability_.copyClipboardText = [](void *self, WindowHandle window, char *output,
            std::uint32_t capacity, std::uint32_t *outRequired)
        { return static_cast<const WindowModule *>(self)->copyClipboardTextImpl(window, output, capacity, outRequired); };
        windowCapability_.submitTouchEvent = [](void *self, WindowHandle window, const TouchEventDescription *event)
        { return static_cast<WindowModule *>(self)->submitTouchEventImpl(window, event); };
        windowCapability_.copyDroppedFiles = [](void *self, WindowHandle window, std::uint64_t batchId,
            char *output, std::uint32_t capacity, std::uint32_t *outRequired)
        { return static_cast<const WindowModule *>(self)->copyDroppedFilesImpl(window, batchId, output, capacity, outRequired); };
        windowCapability_.releaseDroppedFiles = [](void *self, WindowHandle window, std::uint64_t batchId)
        { return static_cast<WindowModule *>(self)->releaseDroppedFilesImpl(window, batchId); };
        windowCapability_.submitFileDropEvent = [](void *self, WindowHandle window,
            const char *const *paths, std::uint32_t count)
        { return static_cast<WindowModule *>(self)->submitFileDropEventImpl(window, paths, count); };
        windowCapability_.lockSurfacePixels = [](void *self, WindowHandle window, SurfacePixels *outPixels)
        { return static_cast<WindowModule *>(self)->lockSurfacePixelsImpl(window, outPixels); };
        windowCapability_.unlockSurfacePixels = [](void *self, WindowHandle window)
        { static_cast<WindowModule *>(self)->unlockSurfacePixelsImpl(window); };

        windowCapability_.surfaceApi = [](void *self, WindowHandle window)
        { return static_cast<const WindowModule *>(self)->surfaceApiImpl(window); };

        vulkanCapability_.requiredInstanceExtensions = [](void *self, std::uint32_t *outCount)
        { return static_cast<const WindowModule *>(self)->requiredInstanceExtensionsImpl(outCount); };
        vulkanCapability_.nativeSurfaceHandles = [](void *self, WindowHandle window, NativeWindowHandles *outHandles)
        { return static_cast<const WindowModule *>(self)->nativeSurfaceHandlesImpl(window, outHandles); };

        glfwCapability_.glfwVersion
            = [](void *self) { return static_cast<const WindowModule *>(self)->glfwVersionImpl(); };
        glfwCapability_.glfwWindowPointer = [](void *self, WindowHandle window)
        { return static_cast<const WindowModule *>(self)->glfwWindowPointerImpl(window); };
        glfwCapability_.nativeHandles = [](void *self, WindowHandle window, NativeWindowHandles *outHandles)
        { return static_cast<const WindowModule *>(self)->nativeHandlesImpl(window, outHandles); };

        headlessCapability_.exportSurfacePpm
            = [](void *self, WindowHandle window, const char *filePath, WindowError *error)
        { return static_cast<const WindowModule *>(self)->exportSurfacePpmImpl(window, filePath, error); };
    }

    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    // Configured defaults; re-read on construct and on every
    // ConfigurationReloadedEvent touching "window." keys.
    std::string         configuredBackend_;
    std::uint32_t       configuredWidth_              = defaultWindowWidth;
    std::uint32_t       configuredHeight_             = defaultWindowHeight;
    std::uint32_t       configuredMinimumWidth_       = 0;
    std::uint32_t       configuredMinimumHeight_      = 0;
    std::uint32_t       configuredMaximumWidth_       = 0;
    std::uint32_t       configuredMaximumHeight_      = 0;
    std::uint32_t       configuredEventQueueCapacity_ = 4096;
    std::int32_t        configuredPositionX_          = 0;
    std::int32_t        configuredPositionY_          = 0;
    bool                configuredHasPosition_        = false;
    std::optional<bool> configuredResizable_;
    std::optional<bool> configuredVisible_;
    std::optional<bool> configuredDecorated_;
    std::optional<bool> configuredFloating_;
    std::string         configuredTitle_ = defaultWindowTitle;

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
