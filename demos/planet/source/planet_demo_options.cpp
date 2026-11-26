#include "planet_demo_support.hpp"

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace demo::planet_demo
{

const char *planetOptionsUsage() noexcept
{
    return "planet demo options:\n"
           "  --view globe|orbit|coast|cliffs|surface|walk  View to render (default globe).\n"
           "  --biome <name>             Pin the surface view to one climate zone (any of\n"
           "                            deep_ocean|ocean|shallow_ocean|beach|desert|savanna|\n"
           "                            grassland|shrubland|temperate_forest|seasonal_forest|\n"
           "                            tropical_rainforest|temperate_rainforest|taiga|\n"
           "                            tundra|snow|rock); the picker then requires the\n"
           "                            spot's biome to match exactly (water biomes stand\n"
           "                            over open sea).\n"
           "  --seed <n>                Planet seed override (0 keeps config).\n"
           "  --radius <meters>         Planet radius override (0 keeps config).\n"
           "  --frames <n>              Render exactly n frames, then exit.\n"
           "  --screenshot <file.png>   Export the rendered frame as PNG (default; a\n"
           "                            .ppm suffix keeps the legacy P6 PPM).\n"
           "  --screenshot-frame <n>    Frame index for --screenshot (default last rendered;\n"
           "                            0 is a valid index).\n"
           "  --frame-times <file.csv>  Export per-frame timings as CSV (a percentile summary\n"
           "                            of the rendered frames is always printed).\n"
           "  --stats-every <n>         Print planet statistics every n frames.\n"
           "  --biome-census <n>        Sample the climate field over n points and print\n"
           "                            the biome histogram (which climate zones this seed\n"
           "                            actually realizes), then exit.\n"
           "  --warmup <n>              Build-only frames before the rendered run (they extend it).\n"
           "                            Warmup is a budget: it ends early once the visible set has\n"
           "                            held fully built through the build pool's wave troughs, so a\n"
           "                            value just needs to cover the view's chunk population on the\n"
           "                            current machine.\n"
           "  --settle-seconds <s>      Wall-clock warmup window: while it is open the\n"
           "                            warmup budget is not consumed - warmup runs until the\n"
           "                            visible set is fully built or this many seconds elapse\n"
           "                            (0 keeps the pure frame budget; when the window closes\n"
           "                            warmup ends, built or not). Deep ground views want\n"
           "                            thousands of chunks whose delivery rate is the build\n"
           "                            pool's, not the frame count's - use this for captures.\n"
           "  --orbit                   Sweep the camera (globe: around the planet;\n"
           "                            surface views: a short circle at the pose;\n"
           "                            walk: a deterministic forward walk).\n"
           "  --caves                   Enable the cave worm carving field.\n"
           "  --chunk-borders           Highlight chunk borders (see the module's\n"
           "                            debug_chunk_borders key).\n"
           "  --self-test               Run determinism/order-independence checks\n"
           "                            and exit (exit code 0 = pass).\n"
           "  --bench                   Benchmark the generation core (field sampling,\n"
           "                            per-depth chunk builds, streaming rate) and exit.\n"
           "  --texture-tile-pixels <n> Screen pixels one terrain texture tile must span for\n"
           "                            the detail multiply to apply (default 64; larger values\n"
           "                            keep the texture further out, 0 disables it). The budget\n"
           "                            cuts the multiply where the tile's motif has minified into\n"
           "                            a repeating stamp - past a few dozen pixels per tile the\n"
           "                            lattice itself starts reading, and the multiply photographs\n"
           "                            as wallpaper over the terrain. The vertex colors' own\n"
           "                            large-scale mottle carries those distances instead.\n"
           "Windowed controls: mouse look, WASD move, Space jump (walk) or up, Ctrl down,\n"
           "                  Shift sprint, arrows look, Left Alt toggles cursor capture.\n"
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
            if (value != "globe" && value != "orbit" && value != "coast" && value != "cliffs" && value != "surface"
                && value != "walk")
            {
                outError = "invalid view '" + value + "'; expected globe|orbit|coast|cliffs|surface|walk";
                return false;
            }
            options.viewName = value;
        }
        else if (argument == "--biome" && hasValue)
        {
            options.biomeName = arguments[++index];
            engine::planet::Biome biome = engine::planet::Biome::Ocean;
            if (!biomeFromName(options.biomeName, biome))
            {
                outError = "invalid biome '" + options.biomeName + "'; expected one of "
                    + std::string("deep_ocean|ocean|shallow_ocean|beach|desert|savanna|grassland|shrubland|")
                    + "temperate_forest|seasonal_forest|tropical_rainforest|temperate_rainforest|taiga|tundra|snow|rock";
                return false;
            }
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
        else if (argument == "--frame-times" && hasValue)
        {
            options.frameTimesPath = arguments[++index];
        }
        else if (argument == "--screenshot-frame" && hasValue)
        {
            options.screenshotFrame = std::strtoull(arguments[++index].c_str(), nullptr, 10);
            options.screenshotFrameSet = true;
        }
        else if (argument == "--stats-every" && hasValue)
        {
            options.statsEveryFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--biome-census" && hasValue)
        {
            options.biomeCensusSamples = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
            if (options.biomeCensusSamples == 0)
            {
                outError = "--biome-census needs at least 1 sample";
                return false;
            }
        }
        else if (argument == "--warmup" && hasValue)
        {
            options.warmupFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--settle-seconds" && hasValue)
        {
            options.settleSeconds = std::strtod(arguments[++index].c_str(), nullptr);
            if (options.settleSeconds < 0.0)
            {
                outError = std::string("--settle-seconds must be >= 0\n") + planetOptionsUsage();
                return false;
            }
        }
        else if (argument == "--orbit")
        {
            options.orbit = true;
        }
        else if (argument == "--caves")
        {
            options.caves = true;
        }
        else if (argument == "--chunk-borders")
        {
            options.chunkBorders = true;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
        }
        else if (argument == "--bench")
        {
            options.benchmark = true;
        }
        else if (argument == "--texture-tile-pixels" && hasValue)
        {
            options.textureTilePixels = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else
        {
            outError = "unknown or incomplete option '" + argument + "'\n" + planetOptionsUsage();
            return false;
        }
    }
    return true;
}

} // namespace demo::planet_demo
