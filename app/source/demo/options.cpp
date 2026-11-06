// options.cpp - parsing of the demo's application command-line arguments.
//
// The kernel/application layer strips its own flags first; everything that
// reaches parseOptions() here must be one of the four demo flags below.

#include "options.hpp"

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
           "  --quit-after <seconds>   Exit after the specified duration.\n"
           "  --camera fly|orbit|fixed Select the camera control mode.\n"
           "  --view benchmark|sun      Select the initial viewpoint.\n";
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
        if (argument != "--screenshot" && argument != "--quit-after" && argument != "--camera" && argument != "--view")
        {
            outError = "unknown application argument '" + argument + "'";
            return false;
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
        else
        {
            // "interior" is accepted for compatibility but behaves like benchmark.
            if (value != "benchmark" && value != "interior" && value != "sun")
            {
                outError = "invalid view '" + value + "'; expected 'benchmark' or 'sun'";
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
