// sandbox_options.hpp - command-line options of the physics sandbox demo.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace demo::physics_sandbox
{

struct SandboxOptions
{
    // Scene layout selector: "showcase" (default) builds the full sandbox
    // with stacks, a chain, a driven pendulum figure, and a hanging cloth
    // sheet; "stacks" and "cloth" build reduced scenes for focused captures.
    // (Named --layout because the engine command line already owns --scene.)
    std::string sceneName = "showcase";

    // Camera behavior: "orbit" circles the scene center (deterministic,
    // default for headless runs), "free" uses keyboard/mouse flight, and
    // "fixed" parks the camera at the showcase viewpoint.
    std::string cameraMode = "orbit";

    // Physics rate (fixed steps per second); 0 keeps the configured value.
    double fixedHz = 0.0;

    // Frame budget: 0 renders until the window closes in windowed runs; a
    // positive count renders exactly that many frames (headless captures).
    std::uint64_t frames = 0;

    // Simulated seconds executed before the first rendered frame, so captures
    // start from a settled or mid-motion state.
    double warmupSeconds = 0.0;

    std::string   screenshotPath;
    std::uint64_t screenshotFrame = 0;
    std::string   frameTimesPath;
    std::uint32_t statsEveryFrames = 60;

    // Deterministic scripted interactions for captures: spawn objects and
    // apply impulses on a fixed timeline so headless screenshots show motion.
    bool scripted = false;

    bool selfTest = false;
    bool pause    = false;

    // Prints every tracked body's position at each stats interval, for
    // scene diagnostics without a renderer.
    bool dumpBodies = false;

    // Reduced feature switches for A/B captures.
    bool noArticulation = false;
    bool noCloth        = false;
};

// Parses the demo's application arguments (after the engine's -- separator);
// returns false with a message on unknown or malformed options.
bool parseSandboxOptions(const std::vector<std::string> &arguments, SandboxOptions &options, std::string &outError);

std::string sandboxOptionsUsage();

} // namespace demo::physics_sandbox
