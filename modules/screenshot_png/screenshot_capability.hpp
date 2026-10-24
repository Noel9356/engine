#ifndef ENGINE_MODULES_SCREENSHOT_SCREENSHOT_CAPABILITY_HPP
#define ENGINE_MODULES_SCREENSHOT_SCREENSHOT_CAPABILITY_HPP

// The published capability surface of the screenshot module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic.

#include <string>

namespace engine
{

// Published after the provider wrote one capture to disk.
struct ScreenshotSavedEvent
{
    std::string filePath;
};

// Capability "screenshot": key triggered and programmatic captures. The
// trigger key and the output directory come from the provider's own
// configuration section; mods call requestScreenshot to queue a capture
// without touching the keyboard path.
class ScreenshotCapability
{
public:
    static constexpr const char *capabilityName = "screenshot";

    virtual ~ScreenshotCapability() = default;

    // Queues one capture of the next rendered frame.
    virtual void requestScreenshot() = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_SCREENSHOT_SCREENSHOT_CAPABILITY_HPP
