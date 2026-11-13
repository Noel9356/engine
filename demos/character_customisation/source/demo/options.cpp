// options.cpp - parsing of the demo's application command-line arguments.
//
// The kernel/application layer strips its own flags first; everything that
// reaches parseOptions() here must be one of the demo flags below.

#include "options.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace demo
{

const char *optionsUsage() noexcept
{
    return "Application options:\n"
           "  --screenshot <file.ppm>  Export a frame when supported.\n"
           "  --screenshot-frame <n>   Frame index for --screenshot (default 90).\n"
           "  --quit-after <seconds>   Exit after the specified duration.\n"
           "  --camera fly|orbit|fixed Select the camera control mode.\n"
           "  --camera-yaw <deg>       Character screen orbit azimuth (default 56).\n"
           "  --camera-pitch <deg>     Character screen orbit elevation (default 10).\n"
           "  --orbit-distance <m>     Character screen orbit radius (default 2.6).\n"
           "  --hide-panel             Draw the character viewport without the studio panel.\n"
           "  --view character|benchmark|sun\n"
           "                           Select the screen (default: character).\n"
           "  --character-test         Run the character assembly test operations, then exit.\n"
           "  --character-lab          Run the merged Character Lab Lua exercise (script_runtime bridge).\n"
           "  --character-drive <name> Apply a named configuration (human, dragonhead, furryfox, furrywolf,\n"
           "                           bare, taur, quadruped, fox, shiba, husky, wolf, donkey, cow, deer,\n"
           "                           bull, alpaca, horse, stag, taur_<species>, winged, tall, broad,\n"
           "                           longmuzzle, marked, longtail, bigtaur, compact, noappend).\n"
           "  --part-previews <dir>    Render one standalone preview frame per part (PPM) and exit.\n"
           "  --panel-tab <0-4>        Open the studio panel on this tab (Plan/Body/Parts/Motion/File).\n";
}

// Strict flag parser: every recognized flag consumes the next token as its
// value (a "--..." token there is reported as a missing value). Unknown flags
// fail; outOptions is only written on full success.
bool parseOptions(const std::vector<std::string> &arguments, Options &outOptions, std::string &outError)
{
    Options parsedOptions;
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        if (argument != "--screenshot" && argument != "--screenshot-frame" && argument != "--quit-after"
            && argument != "--camera" && argument != "--camera-yaw" && argument != "--camera-pitch"
            && argument != "--orbit-distance" && argument != "--camera-target-y" && argument != "--hide-panel"
            && argument != "--view" && argument != "--character-test" && argument != "--character-lab"
            && argument != "--character-drive" && argument != "--part-previews"
            && argument != "--panel-tab")
        {
            outError = "unknown application argument '" + argument + "'";
            return false;
        }
        if (argument == "--character-test")
        {
            parsedOptions.characterTest = true;
            continue;
        }
        if (argument == "--character-lab")
        {
            parsedOptions.characterLab = true;
            continue;
        }
        if (argument == "--part-previews")
        {
            if (index + 1 >= arguments.size() || arguments[index + 1].starts_with("--"))
            {
                outError = "missing value for --part-previews";
                return false;
            }
            parsedOptions.partPreviewsDirectory = arguments[++index];
            continue;
        }
        if (argument == "--hide-panel")
        {
            parsedOptions.hidePanel = true;
            continue;
        }
        if (index + 1 >= arguments.size() || arguments[index + 1].starts_with("--"))
        {
            outError = "missing value for " + argument;
            return false;
        }

        const std::string &value = arguments[++index];
        if (argument == "--panel-tab")
        {
            char           *end    = nullptr;
            const long long parsed = std::strtoll(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0' || parsed < 0 || parsed > 4)
            {
                outError = "--panel-tab expects a value in 0..4";
                return false;
            }
            parsedOptions.panelTab = static_cast<int>(parsed);
        }
        else if (argument == "--screenshot")
        {
            parsedOptions.screenshotPath = value;
        }
        else if (argument == "--screenshot-frame")
        {
            char           *end    = nullptr;
            const long long parsed = std::strtoll(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0' || parsed <= 0)
            {
                outError = "--screenshot-frame expects a positive frame index";
                return false;
            }
            parsedOptions.screenshotFrame = static_cast<std::uint64_t>(parsed);
        }
        else if (argument == "--character-drive")
        {
            static const char *const allowedPresets[] = {"human", "dragonhead", "furryfox", "furrywolf",
                "bare", "taur", "quadruped", "fox", "shiba", "husky", "wolf", "donkey", "cow", "deer",
                "bull", "alpaca", "horse", "stag", "winged", "tall", "broad", "longmuzzle", "marked",
                "longtail", "bigtaur", "compact", "noappend",
                "taur_fox", "taur_shibainu", "taur_husky", "taur_wolf", "taur_donkey",
                "taur_cow", "taur_deer", "taur_bull", "taur_alpaca", "taur_horse", "taur_stag"};
            const bool allowed = std::find(std::begin(allowedPresets), std::end(allowedPresets), value)
                              != std::end(allowedPresets);
            if (!allowed)
            {
                outError = "invalid character drive preset '" + value
                         + "'; expected human, dragonhead, furryfox, furrywolf, bare, taur, quadruped, fox,"
                           " shiba, husky, wolf, donkey, cow, deer, bull, alpaca, horse, stag, taur_<species>,"
                           " winged, tall, broad, longmuzzle, marked, longtail, bigtaur, compact, or noappend";
                return false;
            }
            parsedOptions.characterDrive = value;
        }
        else if (argument == "--quit-after")
        {
            char        *end    = nullptr;
            const double parsed = std::strtod(value.c_str(), &end);
            if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0)
            {
                outError = "--quit-after expects a finite positive number";
                return false;
            }
            parsedOptions.quitAfterSeconds = parsed;
        }
        else if (argument == "--camera")
        {
            if (value != "fly" && value != "orbit" && value != "fixed")
            {
                outError = "invalid camera mode '" + value + "'";
                return false;
            }
            parsedOptions.cameraMode = value;
        }
        else if (argument == "--camera-yaw" || argument == "--camera-pitch" || argument == "--orbit-distance"
                 || argument == "--camera-target-y")
        {
            char        *numericEnd = nullptr;
            const double parsed     = std::strtod(value.c_str(), &numericEnd);
            if (numericEnd == value.c_str() || *numericEnd != '\0' || !std::isfinite(parsed))
            {
                outError = argument + " expects a finite number";
                return false;
            }
            parsedOptions.cameraFramingSet = true;
            if (argument == "--camera-yaw")
            {
                parsedOptions.cameraYawDegrees = static_cast<float>(parsed);
            }
            else if (argument == "--camera-pitch")
            {
                parsedOptions.cameraPitchDegrees = static_cast<float>(std::clamp(parsed, -80.0, 80.0));
            }
            else if (argument == "--camera-target-y")
            {
                parsedOptions.cameraTargetY = static_cast<float>(std::clamp(parsed, -2.0, 6.0));
            }
            else
            {
                parsedOptions.orbitDistance = static_cast<float>(std::clamp(parsed, 1.0, 12.0));
            }
        }
        else
        {
            // "interior" is accepted for compatibility but behaves like benchmark.
            if (value != "character" && value != "benchmark" && value != "interior" && value != "sun")
            {
                outError = "invalid view '" + value + "'; expected 'character', 'benchmark', or 'sun'";
                return false;
            }
            parsedOptions.viewName = value;
        }
    }

    outOptions = std::move(parsedOptions);
    outError.clear();
    return true;
}

} // namespace demo
