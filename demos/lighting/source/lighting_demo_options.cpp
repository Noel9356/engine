#include "lighting_demo_options.hpp"

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace demo::lighting_demo
{

const char *lightingOptionsUsage() noexcept
{
    return "lighting demo options:\n"
           "  --objects <n>            Scattered instances in the field (default 2048; the\n"
           "                            lighting modules register one GI and one reflection\n"
           "                            surface each, so the whole pipeline runs at that count).\n"
           "  --dynamic <fraction>     Share of instances that animate every frame (default\n"
           "                            0.05; their GI/reflection surfaces take the dynamic\n"
           "                            transform path and the draw list re-poses them).\n"
           "  --lights <n>             Point lights through the direct_lighting registry\n"
           "                            (default 8; half of them orbit the field, the rest\n"
           "                            stand still). The [pbr] sun lights the scene on top.\n"
           "  --seed <n>               Placement seed (default 20261126).\n"
           "  --field <meters>         Object placement radius (default 140).\n"
           "  --origin <x>,<z>         Ground-plane offset of the whole scene in meters; the\n"
           "                            camera-relative composition keeps the rendered image\n"
           "                            identical at any placement, and the lighting modules\n"
           "                            evaluate in the same anchor-relative frame.\n"
           "  --view <name>            Camera framing: 'field' sweeps the whole plaza (the\n"
           "                            default orbit), 'showcase' frames the reflecting pool\n"
           "                            and the glass pieces from a three-quarter angle,\n"
           "                            'pool' looks low across the water toward the sun\n"
           "                            (glints, refraction and the caustics on the floor).\n"
           "  --showcase <on|off>      The reflection/refraction/caustics cluster at the\n"
           "                            plaza's center (default on; off restores the bare\n"
           "                            scatter field).\n"
           "  --frames <n>             Render exactly n frames, then exit.\n"
           "  --warmup <n>             Converge-only frames before the rendered run: the\n"
           "                            lighting budgets fill their caches without drawing.\n"
           "  --stats-every <n>        Print lighting statistics every n frames (default 30).\n"
           "  --orbit                  Automated camera path (a slow circle over the field).\n"
           "  --screenshot <file.png>  Export the rendered frame as PNG (a .ppm suffix keeps\n"
           "                            the legacy P6 PPM).\n"
           "  --screenshot-frame <n>   Frame index for --screenshot (default last rendered;\n"
           "                            0 is a valid index).\n"
           "  --frame-times <file.csv> Export per-frame timings as CSV.\n"
           "  --self-test              Audit registration counts, handle bookkeeping and\n"
           "                            placement determinism, then exit (exit code 0 = pass).\n"
           "The lighting terms can be toggled without code through the renderer's config:\n"
           "  --set renderer.gather_indirect_lighting=false   (contact-scale indirect)\n"
           "  --set renderer.gi_indirect_lighting=false       (probe-grid indirect)\n"
           "  --set renderer.reflection_indirect_specular=false (reflections)\n"
           "  --set renderer.sun_shadows=false                (sun shadow map)\n"
           "  --set renderer.sky_shadows=false                (sky occlusion)\n"
           "Windowed controls: mouse look, WASD move, Shift sprint, Left Alt toggles the\n"
           "                  cursor capture. --orbit uses the automated camera path.\n";
}

bool parseLightingOptions(const std::vector<std::string> &arguments, LightingDemoOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const bool         hasValue = index + 1 < arguments.size();
        if (argument == "--objects" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value < 1 || value > 1000000)
            {
                outError = "invalid --objects '" + arguments[index] + "'; expected 1..1000000";
                return false;
            }
            options.objectCount = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--dynamic" && hasValue)
        {
            options.dynamicShare = std::strtod(arguments[++index].c_str(), nullptr);
            if (!(options.dynamicShare >= 0.0 && options.dynamicShare <= 1.0))
            {
                outError = "invalid --dynamic '" + arguments[index] + "'; expected 0..1";
                return false;
            }
        }
        else if (argument == "--lights" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value < 1 || value > 1024)
            {
                outError = "invalid --lights '" + arguments[index] + "'; expected 1..1024";
                return false;
            }
            options.lightCount = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--seed" && hasValue)
        {
            options.seed = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--field" && hasValue)
        {
            options.fieldMeters = std::strtod(arguments[++index].c_str(), nullptr);
            if (!(options.fieldMeters > 8.0 && options.fieldMeters <= 10000.0))
            {
                outError = "invalid --field '" + arguments[index] + "'; expected 8..10000 meters";
                return false;
            }
        }
        else if (argument == "--origin" && hasValue)
        {
            const std::string &value    = arguments[++index];
            char              *scanEnd = nullptr;
            const double       originX = std::strtod(value.c_str(), &scanEnd);
            if (scanEnd == value.c_str() || *scanEnd != ',')
            {
                outError = "invalid --origin '" + value + "'; expected <x>,<z> in meters";
                return false;
            }
            const double originZ = std::strtod(scanEnd + 1, &scanEnd);
            if (scanEnd == nullptr || *scanEnd != '\0' || !std::isfinite(originX) || !std::isfinite(originZ)
                || std::abs(originX) > 1.0e9 || std::abs(originZ) > 1.0e9)
            {
                outError = "invalid --origin '" + value + "'; expected finite <x>,<z> within 1e9 meters";
                return false;
            }
            options.originMeters[0] = originX;
            options.originMeters[1] = originZ;
        }
        else if (argument == "--view" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "field" && value != "showcase" && value != "pool")
            {
                outError = "invalid --view '" + value + "'; expected field|showcase|pool";
                return false;
            }
            options.view = value;
        }
        else if (argument == "--showcase" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "on" && value != "off")
            {
                outError = "invalid --showcase '" + value + "'; expected on|off";
                return false;
            }
            options.showcaseEnabled = value == "on";
        }
        else if (argument == "--frames" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value < 1 || value > 1000000)
            {
                outError = "invalid --frames '" + arguments[index] + "'; expected 1..1000000";
                return false;
            }
            options.frames = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--warmup" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value > 1000000)
            {
                outError = "invalid --warmup '" + arguments[index] + "'; expected 0..1000000";
                return false;
            }
            options.warmupFrames = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--stats-every" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value > 1000000)
            {
                outError = "invalid --stats-every '" + arguments[index] + "'; expected 0..1000000";
                return false;
            }
            options.statsEveryFrames = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--orbit")
        {
            options.orbit = true;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
        }
        else if (argument == "--screenshot" && hasValue)
        {
            options.screenshotPath = arguments[++index];
        }
        else if (argument == "--screenshot-frame" && hasValue)
        {
            const unsigned long long value = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            if (value > 1000000)
            {
                outError = "invalid --screenshot-frame '" + arguments[index] + "'";
                return false;
            }
            options.screenshotFrame = static_cast<std::uint32_t>(value);
        }
        else if (argument == "--frame-times" && hasValue)
        {
            options.frameTimesPath = arguments[++index];
        }
        else
        {
            outError = "unknown lighting demo option '" + argument + "' (see --help)";
            return false;
        }
    }
    return true;
}

} // namespace demo::lighting_demo
