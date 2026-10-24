#ifndef ENGINE_MODULES_SCREENSHOT_HPP
#define ENGINE_MODULES_SCREENSHOT_HPP

#include "input_capability.hpp"
#include "logging_capability.hpp"
#include "renderer_capability.hpp"
#include "screenshot_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <cstdint>
#include <filesystem>
#include <string>

namespace engine
{

class InputCapability;
class RendererCapability;

// Module "screenshot_png": watches the configured trigger key and queues one frame
// capture per press. The capture itself travels renderer -> FrameCapturedEvent
// -> PNG file, so the module never touches Vulkan and the renderer never
// touches the filesystem: each side stays behind its capability contract.
class ScreenshotModule final : public ModuleInterface, public ScreenshotCapability
{
public:
    ScreenshotModule();
    ~ScreenshotModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    void requestScreenshot() override;

private:
    void        handleKeyPressed(const KeyPressedEvent &event);
    void        handleFrameBegin(const FrameBeginEvent &event);
    void        handleFrameCaptured(const FrameCapturedEvent &event);
    std::string nextCaptureFilePath() const;

    ComponentManifest   componentManifest;
    ComponentContext   *boundContext       = nullptr;
    LoggerCapability   *logger             = nullptr;
    RendererCapability *rendererCapability = nullptr;
    EventSubscription   keyPressedSubscription;
    EventSubscription   frameBeginSubscription;
    EventSubscription   frameCapturedSubscription;
    // Default trigger: F12 from the input contract's key enum (one source of
    // the numeric code; the config [screenshot].key replaces it by name).
    std::uint32_t         triggerKeyCode = static_cast<std::uint32_t>(KeyCode::F12);
    std::string           triggerKeyName = "F12";
    std::string           outputDirectory; // empty = platform user media directory
    std::filesystem::path resolvedOutputDirectory;
    bool                  captureQueued  = false;
    std::uint64_t         captureCounter = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_SCREENSHOT_HPP
