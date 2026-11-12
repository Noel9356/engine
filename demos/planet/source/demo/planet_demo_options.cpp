#include "planet_demo_support.hpp"

#include <cstdlib>

namespace demo::planet_demo
{

const char *planetOptionsUsage() noexcept
{
    return "planet demo options:\n"
           "  --view globe|orbit|coast|cliffs|surface  View to render (default globe).\n"
           "  --seed <n>                Planet seed override (0 keeps config).\n"
           "  --radius <meters>         Planet radius override (0 keeps config).\n"
           "  --frames <n>              Render exactly n frames, then exit.\n"
           "  --screenshot <file.ppm>   Export the rendered frame as PPM.\n"
           "  --screenshot-frame <n>    Frame index for --screenshot (default last).\n"
           "  --stats-every <n>         Print planet statistics every n frames.\n"
           "  --orbit                   Sweep the camera (globe: around the planet;\n"
           "                            surface views: a short circle at the pose).\n"
           "  --caves                   Enable the cave worm carving field.\n"
           "  --self-test               Run determinism/order-independence checks\n"
           "                            and exit (exit code 0 = pass).\n"
           "Windowed controls: mouse look, WASD move, Space/Ctrl up/down, Shift sprint,\n"
           "                  arrows look, Left Alt toggles cursor capture.\n"
           "--orbit uses the automated camera path instead of manual input.\n";
}

bool parsePlanetOptions(const std::vector<std::string> &arguments, PlanetDemoOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const bool         hasValue = index + 1 < arguments.size();
        if (argument == "--view" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "globe" && value != "orbit" && value != "coast" && value != "cliffs" && value != "surface")
            {
                outError = "invalid view '" + value + "'; expected globe|orbit|coast|cliffs|surface";
                return false;
            }
            options.viewName = value;
        }
        else if (argument == "--seed" && hasValue)
        {
            options.seed = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--radius" && hasValue)
        {
            options.radius = std::strtod(arguments[++index].c_str(), nullptr);
        }
        else if (argument == "--frames" && hasValue)
        {
            options.frames = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--screenshot" && hasValue)
        {
            options.screenshotPath = arguments[++index];
        }
        else if (argument == "--screenshot-frame" && hasValue)
        {
            options.screenshotFrame = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--stats-every" && hasValue)
        {
            options.statsEveryFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--warmup" && hasValue)
        {
            options.warmupFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--orbit")
        {
            options.orbit = true;
        }
        else if (argument == "--caves")
        {
            options.caves = true;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
        }
        else
        {
            outError = "unknown or incomplete option '" + argument + "'\n" + planetOptionsUsage();
            return false;
        }
    }
    return true;
}

// ---- deterministic view-direction search -----------------------------------------

// Golden-spiral direction set: evenly spread sample directions over the
// sphere, fixed for a given count (no noise, no runtime randomness).

} // namespace demo::planet_demo
