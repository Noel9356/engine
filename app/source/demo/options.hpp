// options.hpp - demo application command-line options.
//
// Parses the engine-assigned application arguments (--screenshot, --quit-after,
// --camera, --view) after the kernel has consumed its own flags.

#ifndef ENGINE_DEMO_OPTIONS_HPP
#define ENGINE_DEMO_OPTIONS_HPP

#include <string>
#include <vector>

namespace demo
{

// Demo options after parsing; empty strings mean "use the configured default".
struct Options
{
    std::string screenshotPath;          // --screenshot: PPM file to write at frame 90; empty = off.
    double      quitAfterSeconds = -1.0; // --quit-after: < 0 = unset (falls back to quit_after_seconds).
    std::string cameraMode;              // --camera fly|orbit|fixed; empty = use [camera] mode config.
    std::string viewName = "benchmark";  // --view benchmark|sun; picks the initial viewpoint.
};

// One-line description of the application flags, appended to --help output.
[[nodiscard]] const char *optionsUsage() noexcept;

// Parses application arguments into outOptions. Returns false on the first
// unknown flag, missing value, or invalid value with a message in outError.
[[nodiscard]] bool parseOptions(const std::vector<std::string> &arguments, Options &outOptions, std::string &outError);

} // namespace demo

#endif
