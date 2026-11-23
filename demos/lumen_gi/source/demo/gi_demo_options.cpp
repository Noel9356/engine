#include "gi_demo_support.hpp"

#include <cstdlib>
#include <string>
#include <vector>

namespace demo::lumen_gi
{

const char *giOptionsUsage() noexcept
{
    return "GI demo options:\n"
           "  --view interior|outdoor|gallery  Scene to render (default interior).\n"
           "  --frames <n>             Render exactly n frames, then exit (deterministic).\n"
           "                           Without it a windowed run keeps going until the\n"
           "                           window is closed or Escape is pressed; headless\n"
           "                           runs default to 30 frames.\n"
           "  --screenshot <file.png>  Export the rendered frame (hook output) as PNG\n"
           "                            (default; a .ppm suffix keeps the P6 PPM).\n"
           "  --screenshot-frame <n>   Frame index for --screenshot (default 29, the\n"
           "                           last frame of a default 30-frame run).\n"
           "  --frame-times <file.csv> Export per-frame timings as CSV (a percentile summary\n"
           "                           of the rendered frames is always printed).\n"
           "  --export-debug <prefix>  Also write lumen_gi debug views <prefix>_<view>.ppm\n"
           "                           (final, gionly, lumenscene, surfacecache, probes,\n"
           "                           reference) and lumen_reflection views\n"
           "                           (composite, reflections, resolvemask, reference)\n"
           "                           at the screenshot frame.\n"
           "  --stats-every <n>        Print lumen_gi statistics every n frames (default 30).\n"
           "  --orbit                  Sweep the camera on a circular path around the view\n"
           "                           target (hands-free dynamic-GI testing; deterministic\n"
           "                           under --frames).\n"
           "  --spin                   Rotate one scene object around its own center every\n"
           "                           frame (the interior occluder, the outdoor chrome\n"
           "                           sphere, the gallery chrome cube): renderer, lumen_gi,\n"
           "                           and lumen_reflection all receive the same per-frame\n"
           "                           model matrix, so the dynamic-surface path runs\n"
           "                           without manual input.\n"
           "  --no-reflections         Skip the lumen_reflection module: the same scene\n"
           "                           renders without the indirect specular term (A/B\n"
           "                           against the default run).\n"
           "  --self-test              Deterministic headless validation run: 24 interior\n"
           "                           frames, then module statistics, snapshot versions,\n"
           "                           transmission counters, and the exported screenshot\n"
           "                           are checked (exit 0 = pass).\n";
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
            if (value != "interior" && value != "outdoor" && value != "gallery")
            {
                outError = "invalid view '" + value + "'; expected 'interior', 'outdoor' or 'gallery'";
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
        else if (argument == "--frame-times" && hasValue)
        {
            options.frameTimesPath = arguments[++index];
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
        else if (argument == "--spin")
        {
            options.spin = true;
        }
        else if (argument == "--no-reflections")
        {
            options.reflections = false;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
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
