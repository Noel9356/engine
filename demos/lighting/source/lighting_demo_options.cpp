// Environment-variable options of the lighting demo (see
// lighting_demo_options.hpp): the DEMO_* variables replace the flags the
// demo used to carry, with the same validation bounds.
#include "lighting_demo_options.hpp"

#include "demo_environment.hpp"

#include <array>
#include <cstdlib>
#include <string>

namespace demo::lighting_demo
{
namespace
{

using demo::environment::readDouble;
using demo::environment::readFlag;
using demo::environment::readUint64;

bool readView(LightingDemoOptions &options, std::string &outError)
{
    const char *raw = std::getenv("DEMO_VIEW");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value(raw);
    static const std::array<const char *, 3> kViews{"field", "showcase", "pool"};
    for (const char *view : kViews)
    {
        if (value == view)
        {
            options.view = value;
            return true;
        }
    }
    outError = "invalid DEMO_VIEW '" + value + "'; expected field|showcase|pool";
    return false;
}

bool readOrigin(LightingDemoOptions &options, std::string &outError)
{
    const char *raw = std::getenv("DEMO_ORIGIN");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value(raw);
    std::size_t split = value.find(',');
    if (split == std::string::npos || split == 0 || split + 1 == value.size())
    {
        outError = "invalid DEMO_ORIGIN '" + value + "'; expected <x>,<z> in meters";
        return false;
    }
    try
    {
        options.originMeters[0] = std::stod(value.substr(0, split));
        options.originMeters[1] = std::stod(value.substr(split + 1));
    }
    catch (const std::exception &)
    {
        outError = "invalid DEMO_ORIGIN '" + value + "'; expected <x>,<z> in meters";
        return false;
    }
    return true;
}

} // namespace

bool lightingDemoOptionsFromEnvironment(LightingDemoOptions &options, std::string &outError)
{
    if (!readView(options, outError) || !readOrigin(options, outError))
    {
        return false;
    }
    std::uint64_t objectCount = options.objectCount;
    if (!readUint64("DEMO_OBJECTS", objectCount, 1, 1000000ull, outError))
    {
        return false;
    }
    options.objectCount = static_cast<std::uint32_t>(objectCount);
    if (!readDouble("DEMO_DYNAMIC", options.dynamicShare, 0.0, 1.0, outError))
    {
        return false;
    }
    std::uint64_t lightCount = options.lightCount;
    if (!readUint64("DEMO_LIGHTS", lightCount, 1, 1024ull, outError))
    {
        return false;
    }
    options.lightCount = static_cast<std::uint32_t>(lightCount);
    std::uint64_t seed = options.seed;
    if (!readUint64("DEMO_SEED", seed, 0, 0xffffffffffffffffull, outError))
    {
        return false;
    }
    options.seed = seed;
    if (!readDouble("DEMO_FIELD", options.fieldMeters, 8.0, 10000.0, outError))
    {
        return false;
    }
    const char *showcase = std::getenv("DEMO_SHOWCASE");
    if (showcase != nullptr && *showcase != '\0')
    {
        const std::string value(showcase);
        if (value == "on")
        {
            options.showcaseEnabled = true;
        }
        else if (value == "off")
        {
            options.showcaseEnabled = false;
        }
        else
        {
            outError = "invalid DEMO_SHOWCASE '" + value + "'; expected on|off";
            return false;
        }
    }
    std::uint64_t frames = options.frames;
    if (!readUint64("DEMO_FRAMES", frames, 1, 1000000ull, outError))
    {
        return false;
    }
    options.frames = static_cast<std::uint32_t>(frames);
    std::uint64_t warmup = options.warmupFrames;
    if (!readUint64("DEMO_WARMUP", warmup, 0, 1000000ull, outError))
    {
        return false;
    }
    options.warmupFrames = static_cast<std::uint32_t>(warmup);
    std::uint64_t statsEvery = options.statsEveryFrames;
    if (!readUint64("DEMO_STATS_EVERY", statsEvery, 0, 1000000ull, outError))
    {
        return false;
    }
    options.statsEveryFrames = static_cast<std::uint32_t>(statsEvery);
    const char *screenshot = std::getenv("DEMO_SCREENSHOT");
    if (screenshot != nullptr && *screenshot != '\0')
    {
        options.screenshotPath = screenshot;
    }
    std::uint64_t screenshotFrame = 0;
    if (!readUint64("DEMO_SCREENSHOT_FRAME", screenshotFrame, 0, 1000000ull, outError))
    {
        return false;
    }
    if (std::getenv("DEMO_SCREENSHOT_FRAME") != nullptr)
    {
        options.screenshotFrame = static_cast<std::uint32_t>(screenshotFrame);
    }
    const char *frameTimes = std::getenv("DEMO_FRAME_TIMES");
    if (frameTimes != nullptr && *frameTimes != '\0')
    {
        options.frameTimesPath = frameTimes;
    }
    return readFlag("DEMO_ORBIT", options.orbit, outError)
        && readFlag("DEMO_SELF_TEST", options.selfTest, outError);
}

} // namespace demo::lighting_demo
