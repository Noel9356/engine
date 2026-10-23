#include "screenshot.hpp"

#include "engine/capabilities/input_contract.hpp"

#include <stb_image_write.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace engine
{

ScreenshotModule::ScreenshotModule()
{
    componentManifest.name                 = "screenshot";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {"screenshot"};
    // "input" is deliberately NOT required: the module reacts to key presses
    // through the event bus (input publishes KeyPressedEvent), so a build
    // without the input module loses only the hotkey, not the capability API.
    componentManifest.requiredCapabilities = {"logging", "renderer"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.renderer", "filesystem.write"};
    componentManifest.knownConfigKeys      = {"screenshot.key", "screenshot.output_directory"};
}

ScreenshotModule::~ScreenshotModule()
{
    stop();
}

void ScreenshotModule::construct(
    ComponentContext &context
)
{
    boundContext       = &context;
    logger             = &context.acquireCapability<LoggerCapability>();
    rendererCapability = &context.acquireCapability<RendererCapability>();

    triggerKeyName          = context.configurationString("key").value_or("F12");
    const KeyCode parsedKey = keyCodeFromName(triggerKeyName);
    if (parsedKey == KeyCode::Unknown)
    {
        throw std::runtime_error(
            "screenshot module: configuration key '" + triggerKeyName + "' is not a known key name"
        );
    }
    triggerKeyCode = static_cast<std::uint32_t>(parsedKey);

    outputDirectory = context.configurationString("output_directory").value_or(std::string());

    EventBus                      &bus            = context.eventBus();
    EventChannel<KeyPressedEvent> &pressedChannel = bus.channel<KeyPressedEvent>();
    keyPressedSubscription                        = EventSubscription(
        &bus,
        typeid(KeyPressedEvent),
        pressedChannel.subscribe<ScreenshotModule, &ScreenshotModule::handleKeyPressed>(this)
    );

    EventChannel<FrameBeginEvent> &frameBeginChannel = bus.channel<FrameBeginEvent>();
    frameBeginSubscription                           = EventSubscription(
        &bus,
        typeid(FrameBeginEvent),
        frameBeginChannel.subscribe<ScreenshotModule, &ScreenshotModule::handleFrameBegin>(this)
    );

    EventChannel<FrameCapturedEvent> &capturedChannel = bus.channel<FrameCapturedEvent>();
    frameCapturedSubscription                         = EventSubscription(
        &bus,
        typeid(FrameCapturedEvent),
        capturedChannel.subscribe<ScreenshotModule, &ScreenshotModule::handleFrameCaptured>(this)
    );
}

void ScreenshotModule::start()
{
    // Destination resolution: relative paths resolve against the process
    // working directory, which is the project root for the documented quick
    // start (./build/runtime/engine from the engine tree). The default is a
    // working-directory 'screenshots' folder, so a normal run writes to
    // <project root>/screenshots where the user actually looks for captures.
    // Absolute paths are used as-is.
    if (outputDirectory.empty())
    {
        resolvedOutputDirectory = std::filesystem::current_path() / "screenshots";
    }
    else
    {
        resolvedOutputDirectory = std::filesystem::path(outputDirectory).is_relative()
                                    ? (std::filesystem::current_path() / outputDirectory)
                                    : std::filesystem::path(outputDirectory);
    }
    logger->logMessage(
        LogLevel::Information,
        "screenshot module started; trigger key is '" + triggerKeyName + "', captures are written to '"
            + resolvedOutputDirectory.string() + "'"
    );
}

void ScreenshotModule::stop()
{
    keyPressedSubscription.release();
    frameBeginSubscription.release();
    frameCapturedSubscription.release();
    boundContext       = nullptr;
    rendererCapability = nullptr;
    logger             = nullptr;
}

void *ScreenshotModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == ScreenshotCapability::capabilityName)
    {
        return static_cast<ScreenshotCapability *>(this);
    }
    return nullptr;
}

void ScreenshotModule::requestScreenshot()
{
    captureQueued = true;
}

void ScreenshotModule::handleKeyPressed(
    const KeyPressedEvent &event
)
{
    if (static_cast<std::uint32_t>(event.keyCode) == triggerKeyCode)
    {
        captureQueued = true;
        logger->logMessage(LogLevel::Debug, "screenshot module: capture queued by key press");
    }
}

void ScreenshotModule::handleFrameBegin(
    const FrameBeginEvent & /*event*/
)
{
    if (captureQueued && rendererCapability != nullptr)
    {
        // The renderer publishes FrameBegin before recording commands, so the
        // capture lands inside the very same frame.
        rendererCapability->requestFrameCapture();
    }
}

void ScreenshotModule::handleFrameCaptured(
    const FrameCapturedEvent &event
)
{
    if (!captureQueued || boundContext == nullptr)
    {
        return;
    }
    captureQueued = false;

    const std::filesystem::path &directory = resolvedOutputDirectory;
    std::filesystem::create_directories(directory);

    const std::string filePath    = (directory / nextCaptureFilePath()).string();
    const int         strideBytes = static_cast<int>(event.width) * 4;
    if (stbi_write_png(
            filePath.c_str(),
            static_cast<int>(event.width),
            static_cast<int>(event.height),
            4,
            event.pixelData,
            strideBytes
        )
        == 0)
    {
        logger->logMessage(LogLevel::Error, "screenshot module: failed to write '" + filePath + "'");
        return;
    }

    ++captureCounter;
    logger->logMessage(LogLevel::Information, "screenshot module: saved '" + filePath + "'");

    ScreenshotSavedEvent saved;
    saved.filePath = filePath;
    boundContext->eventBus().publish(saved);
}

std::string ScreenshotModule::nextCaptureFilePath() const
{
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm           timeInformation{};
#if defined(_WIN32)
    ::localtime_s(&timeInformation, &now);
#else
    ::localtime_r(&now, &timeInformation);
#endif
    std::ostringstream name;
    name << "screenshot_" << std::put_time(&timeInformation, "%Y%m%d_%H%M%S") << "_" << std::setw(3)
         << std::setfill('0') << (captureCounter % 1000) << ".png";
    return name.str();
}

} // namespace engine
