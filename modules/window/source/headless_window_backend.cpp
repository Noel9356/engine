// Headless backend implementation: window creation allocates the zeroed CPU
// RGBA surface, setSize resizes it (copying the overlapping region), and
// Vulkan support exists only when ENGINE_WINDOW_HEADLESS_VULKAN is defined.

#include "headless_window_backend.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <new>
#include <string>
#include <vector>

namespace engine
{
namespace window
{

// Allocates the CPU surface (width * height * 4 zeroed bytes) after
// overflow-checking the pixel count; the framebuffer size follows the window
// size.
bool HeadlessWindowBackend::createWindow(WindowRecord &record, const WindowDescription &description)
{
    (void)description;
    const std::size_t width  = record.width;
    const std::size_t height = record.height;
    if (height != 0 && width > std::numeric_limits<std::size_t>::max() / height / 4)
    {
        return false;
    }
    const std::size_t byteCount = width * height * 4;
    if (byteCount > record.headlessRgba.max_size())
    {
        return false;
    }
    try
    {
        record.headlessRgba.assign(byteCount, 0);
    }
    catch (const std::bad_alloc &)
    {
        record.headlessRgba.clear();
        return false;
    }
    record.framebufferWidth  = record.width;
    record.framebufferHeight = record.height;
    return true;
}

void HeadlessWindowBackend::destroyWindow(WindowRecord &record)
{
    (void)record;
}

void HeadlessWindowBackend::setTitle(WindowRecord &record, const char *title)
{
    if (title != nullptr)
    {
        record.title = title;
    }
}

// Resizes the surface: the overlapping region is copied, the rest stays
// zeroed, and Resized + FramebufferResized events are queued on an actual
// size change.
bool HeadlessWindowBackend::setSize(WindowRecord &record, std::uint32_t width, std::uint32_t height)
{
    if (width == 0 || height == 0 || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
        || height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
    {
        return false;
    }
    const std::size_t maxPixels = std::numeric_limits<std::size_t>::max() / 4;
    if (static_cast<std::size_t>(width) > maxPixels / height)
    {
        return false;
    }
    std::vector<std::uint8_t> resized;
    const std::size_t         byteCount = static_cast<std::size_t>(width) * height * 4;
    if (byteCount > resized.max_size())
    {
        return false;
    }
    try
    {
        resized.assign(byteCount, 0);
    }
    catch (const std::bad_alloc &)
    {
        return false;
    }
    const std::uint32_t copyWidth  = std::min(record.width, width);
    const std::uint32_t copyHeight = std::min(record.height, height);
    for (std::uint32_t row = 0; row < copyHeight; ++row)
    {
        const std::size_t sourceOffset      = static_cast<std::size_t>(row) * record.width * 4;
        const std::size_t destinationOffset = static_cast<std::size_t>(row) * width * 4;
        std::copy_n(record.headlessRgba.data() + sourceOffset,
            static_cast<std::size_t>(copyWidth) * 4,
            resized.data() + destinationOffset);
    }
    record.headlessRgba.swap(resized);
    if (record.width != width || record.height != height)
    {
        record.width             = width;
        record.height            = height;
        record.framebufferWidth  = width;
        record.framebufferHeight = height;
        queueWindowEvent(record, WindowEvent{.kind = WindowEventKind::Resized, .width = width, .height = height});
        queueWindowEvent(record,
            WindowEvent{
                .kind = WindowEventKind::FramebufferResized, .framebufferWidth = width, .framebufferHeight = height});
    }
    return true;
}

bool HeadlessWindowBackend::setPosition(WindowRecord &, std::int32_t, std::int32_t)
{
    return true;
}

bool HeadlessWindowBackend::setClipboardText(const WindowRecord &, const char *text)
{
    if (text == nullptr)
        return false;
    std::size_t length = 0;
    while (length <= maximumClipboardTextBytes && text[length] != '\0')
        ++length;
    if (length > maximumClipboardTextBytes || !isValidUtf8(text, length))
        return false;
    try
    {
        clipboardText_.assign(text, length);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool HeadlessWindowBackend::copyClipboardText(const WindowRecord &, std::string &output) const
{
    try
    {
        output = clipboardText_;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// VK_KHR_surface + VK_EXT_headless_surface when the build has headless
// Vulkan; no extensions otherwise.
const char *const *HeadlessWindowBackend::requiredInstanceExtensions(std::uint32_t *outCount) const
{
#if defined(ENGINE_WINDOW_HEADLESS_VULKAN)

    static const char *extensions[] = {"VK_KHR_surface", "VK_EXT_headless_surface"};
    if (outCount != nullptr)
    {
        *outCount = sizeof(extensions) / sizeof(extensions[0]);
    }
    return extensions;
#else
    (void)outCount;
    return nullptr;
#endif
}

} // namespace window
} // namespace engine
