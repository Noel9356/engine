#ifndef ENGINE_CAPABILITIES_SCREENSHOT_CONTRACT_HPP
#define ENGINE_CAPABILITIES_SCREENSHOT_CONTRACT_HPP

// Mod-facing contract: implementation-, platform- and vendor-agnostic.
// See mods/README.md, section "Portability".

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

#endif // ENGINE_CAPABILITIES_SCREENSHOT_CONTRACT_HPP
