// Environment-variable options of the planet demo (see
// planet_demo_support.hpp): the demo takes no command-line arguments, so the
// headless and capture hooks ride DEMO_* variables parsed through the shared
// demo environment helper, with the same validation bounds the flags they
// replace carried.
#include "planet_demo_support.hpp"

#include "demo_environment.hpp"

#include <array>
#include <cstdlib>
#include <string>
#include <vector>

namespace demo::planet_demo
{
namespace
{

using demo::environment::readDouble;
using demo::environment::readFlag;
using demo::environment::readUint64;

bool readViewName(PlanetDemoOptions &options, std::string &outError)
{
    const char *raw = std::getenv("DEMO_VIEW");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value(raw);
    static const std::array<const char *, 6> kViews{"globe", "orbit", "coast", "cliffs", "surface", "walk"};
    for (const char *view : kViews)
    {
        if (value == view)
        {
            options.viewName = value;
            return true;
        }
    }
    outError = "invalid DEMO_VIEW '" + value + "'; expected globe|orbit|coast|cliffs|surface|walk";
    return false;
}

bool readBiomeName(PlanetDemoOptions &options, std::string &outError)
{
    const char *raw = std::getenv("DEMO_BIOME");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    options.biomeName = raw;
    engine::planet::Biome biome = engine::planet::Biome::Ocean;
    if (!biomeFromName(options.biomeName, biome))
    {
        outError = "invalid DEMO_BIOME '" + options.biomeName + "'; expected one of "
            + std::string("deep_ocean|ocean|shallow_ocean|beach|desert|savanna|grassland|shrubland|")
            + "temperate_forest|seasonal_forest|tropical_rainforest|temperate_rainforest|taiga|tundra|snow|rock";
        return false;
    }
    return true;
}

bool readScreenshot(PlanetDemoOptions &options, std::string &outError)
{
    const char *raw = std::getenv("DEMO_SCREENSHOT");
    if (raw != nullptr && *raw != '\0')
    {
        options.screenshotPath = raw;
    }
    std::uint64_t frame = 0;
    if (!readUint64("DEMO_SCREENSHOT_FRAME", frame, 0, 1000000000ull, outError))
    {
        return false;
    }
    if (raw != nullptr && *raw != '\0' && std::getenv("DEMO_SCREENSHOT_FRAME") != nullptr)
    {
        options.screenshotFrame = frame;
        options.screenshotFrameSet = true;
    }
    return true;
}

} // namespace

bool planetDemoOptionsFromEnvironment(PlanetDemoOptions &options, std::string &outError)
{
    if (!readViewName(options, outError) || !readBiomeName(options, outError))
    {
        return false;
    }
    std::uint64_t seed = options.seed;
    if (!readUint64("DEMO_SEED", seed, 0, 0xffffffffffffffffull, outError))
    {
        return false;
    }
    options.seed = seed;
    if (!readDouble("DEMO_RADIUS", options.radius, 0.0, 1.0e10, outError))
    {
        return false;
    }
    std::uint64_t frames = options.frames;
    if (!readUint64("DEMO_FRAMES", frames, 0, 1000000000ull, outError))
    {
        return false;
    }
    options.frames = frames;
    if (!readScreenshot(options, outError))
    {
        return false;
    }
    const char *frameTimes = std::getenv("DEMO_FRAME_TIMES");
    if (frameTimes != nullptr && *frameTimes != '\0')
    {
        options.frameTimesPath = frameTimes;
    }
    std::uint64_t statsEvery = options.statsEveryFrames;
    if (!readUint64("DEMO_STATS_EVERY", statsEvery, 0, 1000000000ull, outError))
    {
        return false;
    }
    options.statsEveryFrames = static_cast<std::uint32_t>(statsEvery);
    std::uint64_t biomeCensus = options.biomeCensusSamples;
    if (!readUint64("DEMO_BIOME_CENSUS", biomeCensus, 0, 1000000000ull, outError))
    {
        return false;
    }
    if (std::getenv("DEMO_BIOME_CENSUS") != nullptr && biomeCensus == 1)
    {
        outError = "invalid DEMO_BIOME_CENSUS '1'; expected at least 2 samples (0 keeps the demo running)";
        return false;
    }
    options.biomeCensusSamples = static_cast<std::uint32_t>(biomeCensus);
    std::uint64_t warmup = options.warmupFrames;
    if (!readUint64("DEMO_WARMUP", warmup, 0, 1000000000ull, outError))
    {
        return false;
    }
    options.warmupFrames = static_cast<std::uint32_t>(warmup);
    if (!readDouble("DEMO_SETTLE_SECONDS", options.settleSeconds, 0.0, 100000.0, outError))
    {
        return false;
    }
    std::uint64_t tilePixels = options.textureTilePixels;
    if (!readUint64("DEMO_TEXTURE_TILE_PIXELS", tilePixels, 0, 4096ull, outError))
    {
        return false;
    }
    options.textureTilePixels = static_cast<std::uint32_t>(tilePixels);
    return readFlag("DEMO_ORBIT", options.orbit, outError)
        && readFlag("DEMO_CAVES", options.caves, outError)
        && readFlag("DEMO_CHUNK_BORDERS", options.chunkBorders, outError)
        && readFlag("DEMO_SELF_TEST", options.selfTest, outError)
        && readFlag("DEMO_BENCH", options.benchmark, outError);
}

} // namespace demo::planet_demo
