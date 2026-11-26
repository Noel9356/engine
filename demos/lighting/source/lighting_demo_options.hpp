// CLI options of the lighting demo (parsed after the application-level
// options; see applicationCommandLineUsage for those).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace demo::lighting_demo
{

struct LightingDemoOptions
{
    std::uint32_t objectCount      = 2048;   // scattered instances in the field
    double        dynamicShare     = 0.05;   // fraction of instances that animate
    std::uint32_t lightCount       = 8;      // point lights through direct_lighting
    std::uint64_t seed             = 20261126u;
    double        fieldMeters      = 140.0;  // object placement radius
    double        originMeters[2]  = {0.0, 0.0}; // scene anchor on the ground plane (x, z)
    bool          showcaseEnabled  = true;   // reflection/refraction/caustics cluster at the pool
    std::string   view             = "field"; // field | showcase | pool
    std::uint32_t frames           = 90;     // headless default; windowed runs ignore this
    std::uint32_t warmupFrames     = 60;     // build/converge-only frames before rendering
    std::uint32_t statsEveryFrames = 30;
    bool          orbit            = false;  // automated camera path
    bool          selfTest         = false;  // registration/determinism audit, then exit
    std::string   screenshotPath;
    std::uint32_t screenshotFrame  = 0xffffffffu; // default: last rendered frame
    std::string   frameTimesPath;
};

const char *lightingOptionsUsage() noexcept;
bool parseLightingOptions(
    const std::vector<std::string> &arguments, LightingDemoOptions &options, std::string &outError);

} // namespace demo::lighting_demo
