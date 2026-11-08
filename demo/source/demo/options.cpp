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
           "  --character-drive <name> Apply a named configuration (dragon, serpent, tall, broad,\n"
           "                           longtail, compact, noappend) to the character screen.\n";
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
            && argument != "--orbit-distance" && argument != "--hide-panel" && argument != "--view"
            && argument != "--character-test" && argument != "--character-drive")
        {
            outError = "unknown application argument '" + argument + "'";
            return false;
        }
        if (argument == "--character-test")
        {
            parsedOptions.characterTest = true;
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
        if (argument == "--screenshot")
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
            if (value != "dragon" && value != "serpent" && value != "tall" && value != "broad" && value != "longtail"
                && value != "compact" && value != "noappend")
            {
                outError = "invalid character drive preset '" + value
                         + "'; expected dragon, serpent, tall, broad, longtail, compact, or noappend";
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
        else if (argument == "--camera-yaw" || argument == "--camera-pitch" || argument == "--orbit-distance")
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
