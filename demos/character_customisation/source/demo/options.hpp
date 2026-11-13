// options.hpp - demo application command-line options.
//
// Parses the engine-assigned application arguments (--screenshot, --quit-after,
// --camera, --view) after the kernel has consumed its own flags.

#ifndef ENGINE_DEMO_OPTIONS_HPP
#define ENGINE_DEMO_OPTIONS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace demo
{

// Demo options after parsing; empty strings mean "use the configured default".
struct Options
{
    std::string   screenshotPath;              // --screenshot: PPM file to write at screenshotFrame; empty = off.
    std::uint64_t screenshotFrame  = 90;       // --screenshot-frame: frame index for the --screenshot export.
    double        quitAfterSeconds = -1.0;     // --quit-after: < 0 = unset (falls back to quit_after_seconds).
    std::string   cameraMode;                  // --camera fly|orbit|fixed; empty = use [camera] mode config.
    float         cameraYawDegrees = 0.0f;     // --camera-yaw: character-screen orbit azimuth around the target.
    float         cameraPitchDegrees = 18.0f;  // --camera-pitch: character-screen orbit elevation.
    float         cameraTargetY = 1.0f;        // --camera-target-y: orbit target height (junction close-ups).
    float         orbitDistance = 2.6f;        // --orbit-distance: character-screen orbit radius in meters.
    bool          cameraFramingSet = false;    // true once any framing flag was given.
    bool          hidePanel = false;           // --hide-panel: capture the viewport without the studio UI.
    std::string   viewName      = "character"; // --view character|benchmark|sun; picks the screen.
    bool          characterTest = false;       // --character-test: run the assembly test operations and exit.
    bool          characterLab = false;        // --character-lab: run the merged character_lab Lua exercise.
    std::string   characterDrive;              // --character-drive: named configuration for the character screen.
    std::string   partPreviewsDirectory;       // --part-previews: render one PPM per part into this directory.
    int           panelTab = -1;               // --panel-tab: open the studio panel on this tab (captures).
};

// One-line description of the application flags, appended to --help output.
[[nodiscard]] const char *optionsUsage() noexcept;

// Parses application arguments into outOptions. Returns false on the first
// unknown flag, missing value, or invalid value with a message in outError.
[[nodiscard]] bool parseOptions(const std::vector<std::string> &arguments, Options &outOptions, std::string &outError);

} // namespace demo

#endif
