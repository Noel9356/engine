// Internal backend interface shared between the window module's source
// files: the per-window record, the event queueing rules, the touch-ingress
// bookkeeping, and the abstract WindowBackend that the GLFW and headless
// implementations fulfill.

#ifndef ENGINE_WINDOW_MODULE_BACKEND_HPP
#define ENGINE_WINDOW_MODULE_BACKEND_HPP

#include <engine/window/capabilities.hpp>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{
namespace window
{

inline constexpr std::size_t maximumClipboardTextBytes = 1024u * 1024u;
inline constexpr std::size_t maximumDroppedFilePathBytes = 32768u;
inline constexpr std::size_t maximumDroppedFileBatchBytes = 8u * 1024u * 1024u;
inline constexpr std::size_t maximumDroppedFilesPerBatch = 256u;
inline constexpr std::size_t maximumPendingDroppedFileBatches = 64u;

// Runtime knobs for backend gamepad polling. enabled skips the per-frame
// joystick scan entirely; axisEpsilon is the minimum absolute change an axis
// must report before a GamepadAxisMoved event is queued (smaller changes are
// treated as noise).
struct GamepadPollingOptions
{
    bool  enabled    = true;
    float axisEpsilon = 0.0005f;
};

// Validates a bounded UTF-8 byte sequence without allocating. Rejects overlong
// encodings, surrogate code points, truncated sequences, and values above U+10FFFF.
inline bool isValidUtf8(const char *text, std::size_t length) noexcept
{
    if (text == nullptr && length != 0)
        return false;

    std::size_t offset = 0;
    while (offset < length)
    {
        const auto lead = static_cast<unsigned char>(text[offset]);
        if (lead <= 0x7fu)
        {
            ++offset;
            continue;
        }

        std::uint32_t codePoint = 0;
        std::size_t continuationCount = 0;
        if (lead >= 0xc2u && lead <= 0xdfu)
        {
            codePoint = lead & 0x1fu;
            continuationCount = 1;
        }
        else if (lead >= 0xe0u && lead <= 0xefu)
        {
            codePoint = lead & 0x0fu;
            continuationCount = 2;
        }
        else if (lead >= 0xf0u && lead <= 0xf4u)
        {
            codePoint = lead & 0x07u;
            continuationCount = 3;
        }
        else
        {
            return false;
        }

        if (continuationCount > length - offset - 1)
            return false;
        for (std::size_t index = 1; index <= continuationCount; ++index)
        {
            const auto continuation = static_cast<unsigned char>(text[offset + index]);
            if ((continuation & 0xc0u) != 0x80u)
                return false;
            codePoint = (codePoint << 6u) | (continuation & 0x3fu);
        }

        if ((continuationCount == 1 && codePoint < 0x80u)
            || (continuationCount == 2 && codePoint < 0x800u)
            || (continuationCount == 3 && codePoint < 0x10000u)
            || (codePoint >= 0xd800u && codePoint <= 0xdfffu) || codePoint > 0x10ffffu)
        {
            return false;
        }
        offset += continuationCount + 1;
    }
    return true;
}

// Module-side state for one window, shared with the backend that created it.
// width/height track the current client-area size and framebuffer* the
// swapchain-size pixels (both pixels); keyStateMask mirrors pressed keys;
// events is the queue drained by the module's popEvent(s). headlessRgba backs
// the software surface; platformWindow is the backend-native handle
// (GLFWwindow* for the GLFW backend). The window-state block holds the state
// requested through the runtime window-control calls: the headless backend
// reports it directly, while the GLFW backend seeds it at creation and then
// prefers the live window-system attributes.
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

    // Window state requested through the runtime-control calls, seeded from
    // the creation description. The headless backend reports them directly;
    // the GLFW backend seeds them and then prefers live attributes.
    bool        resizable  = true;
    bool        decorated  = true;
    bool        floating   = false;
    bool        visible    = true;
    bool        iconified  = false;
    bool        maximized  = false;
    float       opacity    = 1.0f;
    std::int32_t positionX = 0;
    std::int32_t positionY = 0;

    bool closeRequested = false;

    // Bit (1 << Key value) set for each pressed key.
    std::uint64_t keyStateMask = 0;

    // Bit (1 << MouseButton value) set for each held mouse button.
    std::uint8_t mouseButtonStateMask = 0;

    bool cursorPositionValid = false;
    double lastCursorX = 0.0;
    double lastCursorY = 0.0;

    struct TouchContact
    {
        float x = 0.0f;
        float y = 0.0f;
        float pressure = 1.0f;
    };
    std::unordered_map<std::uint64_t, TouchContact> activeTouches;

    struct DroppedFileBatch
    {
        std::uint64_t id = 0;
        std::uint32_t totalBytes = 0;
        std::vector<std::string> paths;
    };
    std::deque<DroppedFileBatch> droppedFileBatches;
    std::uint64_t nextDroppedFileBatchId = 1;

    std::deque<WindowEvent> events;
    std::size_t             eventQueueCapacity = 4096;

    void *platformWindow = nullptr;

    std::vector<std::uint8_t> headlessRgba;
};

// Removes retained path storage associated with a queued FilesDropped event.
inline void eraseDroppedFileBatch(WindowRecord &record, std::uint64_t batchId) noexcept
{
    const auto found = std::find_if(record.droppedFileBatches.begin(), record.droppedFileBatches.end(),
        [batchId](const WindowRecord::DroppedFileBatch &batch) { return batch.id == batchId; });
    if (found != record.droppedFileBatches.end())
        record.droppedFileBatches.erase(found);
}

// Appends to the window's queue: trailing resize events are replaced, cursor
// and same-contact touch moves keep the latest position and accumulate deltas,
// and consecutive scroll events are accumulated. When full, high-frequency
// state events are evicted before transitions; if none exist, the oldest event
// is evicted. Allocation failure drops the new event and reports false to
// callers that own payloads.
inline bool queueWindowEvent(WindowRecord &record, const WindowEvent &event)
{
    const bool coalescible = event.kind == WindowEventKind::Resized
        || event.kind == WindowEventKind::FramebufferResized || event.kind == WindowEventKind::CursorMoved;
    if (coalescible && !record.events.empty() && record.events.back().kind == event.kind)
    {
        if (event.kind == WindowEventKind::CursorMoved)
        {
            WindowEvent &previous = record.events.back();
            previous.cursorDeltaX += event.cursorDeltaX;
            previous.cursorDeltaY += event.cursorDeltaY;
            previous.cursorX = event.cursorX;
            previous.cursorY = event.cursorY;
        }
        else
        {
            record.events.back() = event;
        }
        return true;
    }
    if (event.kind == WindowEventKind::GamepadAxisMoved && !record.events.empty()
        && record.events.back().kind == WindowEventKind::GamepadAxisMoved
        && record.events.back().gamepadId == event.gamepadId
        && record.events.back().gamepadControl == event.gamepadControl)
    {
        record.events.back() = event;
        return true;
    }
    if (event.kind == WindowEventKind::TouchMoved && !record.events.empty()
        && record.events.back().kind == WindowEventKind::TouchMoved
        && record.events.back().touchDevice == event.touchDevice
        && record.events.back().touchId == event.touchId)
    {
        WindowEvent &previous = record.events.back();
        previous.touchDeltaX += event.touchDeltaX;
        previous.touchDeltaY += event.touchDeltaY;
        previous.touchX = event.touchX;
        previous.touchY = event.touchY;
        previous.touchPressure = event.touchPressure;
        return true;
    }
    if (event.kind == WindowEventKind::MouseScrolled && !record.events.empty()
        && record.events.back().kind == WindowEventKind::MouseScrolled)
    {
        record.events.back().scrollX += event.scrollX;
        record.events.back().scrollY += event.scrollY;
        return true;
    }
    if (record.eventQueueCapacity != 0 && record.events.size() >= record.eventQueueCapacity)
    {
        const auto disposable = std::find_if(record.events.begin(),
            record.events.end(),
            [](const WindowEvent &queued)
            {
                switch (queued.kind)
                {
                case WindowEventKind::Resized:
                case WindowEventKind::FramebufferResized:
                case WindowEventKind::CursorMoved:
                case WindowEventKind::MouseScrolled:
                case WindowEventKind::GamepadAxisMoved:
                case WindowEventKind::TouchMoved: return true;
                default: return false;
                }
            });
        auto discarded = disposable != record.events.end() ? disposable : record.events.begin();
        if (discarded->kind == WindowEventKind::FilesDropped)
            eraseDroppedFileBatch(record, discarded->fileDropBatchId);
        record.events.erase(discarded);
    }
    try
    {
        record.events.push_back(event);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// Terminates every active contact after focus loss. A host may not receive
// native end callbacks once the window is unfocused, so cancellation is
// emitted before the contact table is cleared.
inline void cancelActiveTouches(WindowRecord &record) noexcept
{
    for (const auto &[key, contact] : record.activeTouches)
    {
        (void)queueWindowEvent(record, WindowEvent{
            .kind = WindowEventKind::TouchCancelled,
            .touchDevice = static_cast<std::uint32_t>(key >> 32u),
            .touchId = static_cast<std::uint32_t>(key),
            .touchX = contact.x,
            .touchY = contact.y,
            .touchPressure = contact.pressure});
    }
    record.activeTouches.clear();
}

// Applies one platform-adapter touch contact to a window record: validates
// the description, tracks the (device, touchId) contact, queues the matching
// event, and rolls the contact table back when the queue rejects the event so
// the stored state always matches the published events.
inline bool submitTouchContact(WindowRecord &record, const TouchEventDescription &description)
{
    if (static_cast<std::uint32_t>(description.phase) > static_cast<std::uint32_t>(TouchPhase::Cancelled)
        || !std::isfinite(description.x) || !std::isfinite(description.y)
        || description.touchId == invalidTouchId || !std::isfinite(description.pressure) || description.x < 0.0f || description.x > 1.0f
        || description.y < 0.0f || description.y > 1.0f || description.pressure < 0.0f
        || description.pressure > 1.0f)
        return false;

    const std::uint64_t key = (static_cast<std::uint64_t>(description.device) << 32u) | description.touchId;
    auto contact = record.activeTouches.find(key);
    if (description.phase == TouchPhase::Began)
    {
        if (contact != record.activeTouches.end()
            || record.activeTouches.size() >= maximumTouchContactsPerWindow)
            return false;
        try
        {
            contact = record.activeTouches.emplace(
                key, WindowRecord::TouchContact{description.x, description.y, description.pressure}).first;
        }
        catch (...)
        {
            return false;
        }
    }
    else if (contact == record.activeTouches.end())
    {
        return false;
    }

    const WindowRecord::TouchContact previousContact = contact->second;
    const float deltaX = description.phase == TouchPhase::Began ? 0.0f : description.x - contact->second.x;
    const float deltaY = description.phase == TouchPhase::Began ? 0.0f : description.y - contact->second.y;
    contact->second = {description.x, description.y, description.pressure};

    WindowEventKind kind = WindowEventKind::TouchBegan;
    switch (description.phase)
    {
    case TouchPhase::Began: kind = WindowEventKind::TouchBegan; break;
    case TouchPhase::Moved: kind = WindowEventKind::TouchMoved; break;
    case TouchPhase::Ended: kind = WindowEventKind::TouchEnded; break;
    case TouchPhase::Cancelled: kind = WindowEventKind::TouchCancelled; break;
    }
    const bool queued = queueWindowEvent(record, WindowEvent{.kind = kind,
        .touchDevice = description.device,
        .touchId = description.touchId,
        .touchX = description.x,
        .touchY = description.y,
        .touchPressure = description.pressure,
        .touchDeltaX = deltaX,
        .touchDeltaY = deltaY});

    if (description.phase == TouchPhase::Ended || description.phase == TouchPhase::Cancelled)
    {
        record.activeTouches.erase(key);
    }
    else if (!queued)
    {
        if (description.phase == TouchPhase::Began)
            record.activeTouches.erase(key);
        else
            contact->second = previousContact;
    }
    return queued;
}

// Stores a complete drop batch before publishing its event. The count, each
// path length, total bytes, UTF-8 encoding and outstanding-batch count are
// bounded so native callbacks cannot retain arbitrary memory.
inline bool queueDroppedFileBatch(WindowRecord &record, const char *const *paths, std::uint32_t count)
{
    if (paths == nullptr || count == 0 || count > maximumDroppedFilesPerBatch
        || record.droppedFileBatches.size() >= maximumPendingDroppedFileBatches
        || record.nextDroppedFileBatchId == 0)
        return false;

    WindowRecord::DroppedFileBatch batch;
    batch.id = record.nextDroppedFileBatchId;
    try
    {
        batch.paths.reserve(count);
        std::size_t totalBytes = 0;
        for (std::uint32_t index = 0; index < count; ++index)
        {
            if (paths[index] == nullptr)
                return false;
            std::size_t length = 0;
            while (length <= maximumDroppedFilePathBytes && paths[index][length] != '\0')
                ++length;
            if (length == 0 || length > maximumDroppedFilePathBytes
                || length + 1 > maximumDroppedFileBatchBytes - totalBytes
                || !isValidUtf8(paths[index], length))
                return false;
            totalBytes += length + 1;
            batch.paths.emplace_back(paths[index], length);
        }
        batch.totalBytes = static_cast<std::uint32_t>(totalBytes);
        record.droppedFileBatches.push_back(std::move(batch));
    }
    catch (...)
    {
        return false;
    }

    const std::uint64_t batchId = record.nextDroppedFileBatchId;
    record.nextDroppedFileBatchId = batchId == std::numeric_limits<std::uint64_t>::max() ? 0 : batchId + 1;
    if (!queueWindowEvent(record, WindowEvent{
            .kind = WindowEventKind::FilesDropped,
            .fileDropBatchId = batchId,
            .fileDropCount = count}))
    {
        eraseDroppedFileBatch(record, batchId);
        return false;
    }
    return true;
}

inline bool copyDroppedFileBatch(const WindowRecord &record, std::uint64_t batchId,
    char *output, std::uint32_t capacity, std::uint32_t *outRequired)
{
    if (outRequired == nullptr || batchId == 0)
        return false;
    const auto found = std::find_if(record.droppedFileBatches.begin(), record.droppedFileBatches.end(),
        [batchId](const WindowRecord::DroppedFileBatch &batch) { return batch.id == batchId; });
    if (found == record.droppedFileBatches.end())
        return false;
    *outRequired = found->totalBytes;
    if (output == nullptr || capacity < found->totalBytes)
        return false;

    std::size_t offset = 0;
    for (const std::string &path : found->paths)
    {
        std::copy(path.begin(), path.end(), output + offset);
        offset += path.size();
        output[offset++] = '\0';
    }
    return true;
}

inline bool releaseDroppedFileBatch(WindowRecord &record, std::uint64_t batchId) noexcept
{
    if (batchId == 0)
        return false;
    const std::size_t originalSize = record.droppedFileBatches.size();
    eraseDroppedFileBatch(record, batchId);
    return record.droppedFileBatches.size() != originalSize;
}

// Interface every window backend implements (GLFW, headless). Calls come
// from the module on the main thread; OS callbacks fire during pollEvents()
// and land in the WindowRecord queues. Runtime window-control calls have
// no-op defaults so a backend only overrides what its platform provides.
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

    virtual bool getPosition(const WindowRecord &record, std::int32_t *outX, std::int32_t *outY) const
    {
        (void)record;
        (void)outX;
        (void)outY;
        return false;
    }

    virtual void setOpacity(WindowRecord &record, float opacity)
    {
        (void)record;
        (void)opacity;
    }

    virtual void setVisible(WindowRecord &record, bool visible)
    {
        (void)record;
        (void)visible;
    }

    virtual void setResizable(WindowRecord &record, bool resizable)
    {
        (void)record;
        (void)resizable;
    }

    virtual void setDecorated(WindowRecord &record, bool decorated)
    {
        (void)record;
        (void)decorated;
    }

    virtual void setFloating(WindowRecord &record, bool floating)
    {
        (void)record;
        (void)floating;
    }

    virtual void focus(const WindowRecord &record)
    {
        (void)record;
    }

    virtual void requestAttention(const WindowRecord &record)
    {
        (void)record;
    }

    virtual void iconify(WindowRecord &record)
    {
        (void)record;
    }

    virtual void restore(WindowRecord &record)
    {
        (void)record;
    }

    virtual void maximize(WindowRecord &record)
    {
        (void)record;
    }

    virtual bool getWindowAttributes(const WindowRecord &record, WindowAttributes *outAttributes) const
    {
        (void)record;
        (void)outAttributes;
        return false;
    }

    virtual bool getContentScale(const WindowRecord &record, float *outX, float *outY) const
    {
        (void)record;
        (void)outX;
        (void)outY;
        return false;
    }

    virtual bool rawMouseMotionSupported() const
    {
        return false;
    }

    virtual void pollEvents() = 0;

    // Idle-friendly alternative to pollEvents: block until events arrive or
    // the timeout passes (<= 0 waits indefinitely).
    virtual void waitEvents(double timeoutSeconds)
    {
        (void)timeoutSeconds;
    }

    virtual void setCursorMode(const WindowRecord &record, CursorMode mode)                   = 0;
    virtual bool cursorPosition(const WindowRecord &record, double *outX, double *outY) const = 0;

    virtual bool setClipboardText(const WindowRecord &, const char *)
    {
        return false;
    }

    // Borrows the backend's clipboard text as a NUL-terminated UTF-8 string;
    // the pointer is only valid until the next clipboard call.
    virtual bool copyClipboardText(const WindowRecord &, const char **outText) const
    {
        (void)outText;
        return false;
    }

    // Applies gamepad polling options at runtime (enable/disable, axis noise
    // threshold); backends without gamepads ignore it.
    virtual void configureGamepads(const GamepadPollingOptions &options)
    {
        (void)options;
    }

    virtual const char *const *requiredInstanceExtensions(std::uint32_t *outCount) const = 0;

    virtual bool nativeSurfaceHandles(const WindowRecord &record, NativeWindowHandles *outHandles) const = 0;
};

} // namespace window
} // namespace engine

#endif
