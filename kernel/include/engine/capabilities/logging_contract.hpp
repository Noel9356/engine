#ifndef ENGINE_CAPABILITIES_LOGGING_CONTRACT_HPP
#define ENGINE_CAPABILITIES_LOGGING_CONTRACT_HPP

// Mod-facing contract: implementation-, platform- and vendor-agnostic.
// See mods/README.md, section "Portability".

#include <string>

namespace engine
{

enum class LogLevel
{
    Trace,
    Debug,
    Information,
    Warning,
    Error
};

// Capability "logging": message sink provided by the logging module. Every
// other component depends on it first, which makes it the root of the
// dependency graph.
class LoggerCapability
{
public:
    static constexpr const char* capabilityName = "logging";

    virtual ~LoggerCapability() = default;

    virtual void logMessage(LogLevel level, const std::string& message) = 0;
};

} // namespace engine

#endif // ENGINE_CAPABILITIES_LOGGING_CONTRACT_HPP
