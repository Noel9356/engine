#ifndef ENGINE_MODULES_LOGGING_LOGGING_CAPABILITY_HPP
#define ENGINE_MODULES_LOGGING_LOGGING_CAPABILITY_HPP

// The published capability surface of the logging module. Every consumer -
// other modules, native mods, the app - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic; the per-module
// purity guard in the module's CMakeLists enforces that mechanically.
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

// Capability "logging": message sink. Every other component depends on it
// first, which makes it the root of the dependency graph; whichever component
// provides it is just the graph's first provider.
class LoggerCapability
{
public:
    static constexpr const char *capabilityName = "logging";

    virtual ~LoggerCapability() = default;

    virtual void logMessage(LogLevel level, const std::string &message) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_LOGGING_LOGGING_CAPABILITY_HPP
