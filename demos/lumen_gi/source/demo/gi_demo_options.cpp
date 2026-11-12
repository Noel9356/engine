#include "gi_demo_support.hpp"

#include <cstdlib>

namespace demo::lumen_gi
{

const char *giOptionsUsage() noexcept
{
    return "GI demo options:\n"
           "  --view interior|outdoor  Scene to render (default interior).\n"
           "  --frames <n>             Render exactly n frames, then exit (deterministic).\n"
           "                           Without it a windowed run keeps going until the\n"
           "                           window is closed or Escape is pressed; headless\n"
           "                           runs default to 30 frames.\n"
           "  --screenshot <file.ppm>  Export the rendered frame (hook output) as PPM.\n"
           "  --screenshot-frame <n>   Frame index for --screenshot (default 29, the\n"
           "                           last frame of a default 30-frame run).\n"
           "  --export-debug <prefix>  Also write lumen_gi debug views <prefix>_<view>.ppm\n"
           "                           (final, gionly, lumenscene, surfacecache, probes,\n"
           "                           reference) at the screenshot frame.\n"
           "  --stats-every <n>        Print lumen_gi statistics every n frames (default 30).\n"
           "  --orbit                  Sweep the camera on a circular path around the view\n"
           "                           target (hands-free dynamic-GI testing; deterministic\n"
           "                           under --frames).\n";
}

bool parseGiOptions(const std::vector<std::string> &arguments, GiDemoOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const bool         hasValue = index + 1 < arguments.size();
        if (argument == "--view" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "interior" && value != "outdoor")
            {
                outError = "invalid view '" + value + "'; expected 'interior' or 'outdoor'";
                return false;
            }
            options.viewName = value;
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
        else if (argument == "--export-debug" && hasValue)
        {
            options.exportDebugPrefix = arguments[++index];
        }
        else if (argument == "--stats-every" && hasValue)
        {
            options.statsEveryFrames
                = static_cast<std::uint32_t>(std::strtoull(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--orbit")
        {
            options.orbit = true;
        }
        else
        {
            outError = "unknown or incomplete option '" + argument + "'\n" + giOptionsUsage();
            return false;
        }
    }
    return true;
}

} // namespace demo::lumen_gi
