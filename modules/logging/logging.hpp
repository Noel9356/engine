#ifndef ENGINE_MODULES_LOGGING_HPP
#define ENGINE_MODULES_LOGGING_HPP

#include "engine/capabilities/logging_contract.hpp"
#include "engine/kernel/component_interface.hpp"

#include <fstream>
#include <mutex>
#include <string>

namespace engine
{

// Module "logging": provides the LoggerCapability through a console sink and an
// optional file sink. Console output is colored when the stream is a terminal.
class LoggingModule final : public ModuleInterface, public LoggerCapability
{
public:
    LoggingModule();

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    void logMessage(LogLevel level, const std::string &message) override;

private:
    static const char *levelTag(LogLevel level);
    static const char *levelColor(LogLevel level);

    ComponentManifest componentManifest;
    LogLevel          minimumLevel = LogLevel::Information;
    std::ofstream     fileSink;
    std::mutex        sinkMutex;
};

} // namespace engine

#endif // ENGINE_MODULES_LOGGING_HPP
