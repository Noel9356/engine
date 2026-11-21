// Private option, view-placement, and validation helpers for the planet demo.
#pragma once

#include <engine/planet/capabilities.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/planet.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/mesh.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace demo::planet_demo
{
struct PlanetDemoOptions
{
    std::string viewName = "globe";
    std::uint64_t seed = 0;
    double radius = 0.0;
    std::uint64_t frames = 0;
    std::string screenshotPath;
    std::string frameTimesPath;
    std::uint64_t screenshotFrame = 0;
    bool screenshotFrameSet = false; // --screenshot-frame appeared (0 is a valid index)
    std::uint32_t warmupFrames = 0;
    std::uint32_t statsEveryFrames = 30;
    bool orbit = false;
    bool caves = false;
    bool chunkBorders = false;
    bool selfTest = false;
    bool benchmark = false;
    // Screen pixels one texture tile must span for the terrain detail
    // multiply to apply (see the gate setup in main.cpp); 0 disables the
    // terrain textures outright.
    std::uint32_t textureTilePixels = 6;
};

struct Viewpoint
{
    engine::planet::Vec3 direction;
    double surfaceRadius = 0.0;
    bool hasLookTangent = false;
    engine::planet::Vec3 lookTangent = {};
};

enum class ProbePolicy { Inland, Coastline, Mountain };

struct SelfTestResult
{
    bool pass = true;
    std::string report;
};

int runBenchmark(const engine::planet::PlanetGenerationParams &params);

const char *planetOptionsUsage() noexcept;
bool parsePlanetOptions(const std::vector<std::string> &arguments, PlanetDemoOptions &options, std::string &outError);
std::vector<engine::planet::Vec3> goldenSpiralDirections(std::uint32_t count);
engine::planet::Vec3 rotateDirection(const engine::planet::Vec3 &direction, const engine::planet::Vec3 &axis,
    double angleRadians);
void clampCameraPitch(const glm::dvec3 &eye, const glm::dvec3 &localUp, glm::dvec3 &target,
    double minPitchRadians, double maxPitchRadians);
bool pickViewpoint(engine::planet::PlanetCapability &planetCapability, engine::planet::PlanetHandle planet,
    const std::function<bool(double, engine::planet::Biome, double, const engine::planet::PlanetGenerationParams &)> &predicate,
    ProbePolicy policy, double eyeHeight, const engine::planet::Vec3 &sunDirection, Viewpoint &out);
SelfTestResult runSelfTest(engine::planet::PlanetCapability &capability, void *capabilitySelf,
    const PlanetDemoOptions &options);
// Camera-relative composition threshold: beyond this distance from the
// planet's center the demo renders the world shifted by the f64 eye, so the
// renderer's f32 pipeline only ever sees eye-relative coordinates (f32
// quantization at planet scale would otherwise step chunk placement in
// half-meter increments and tear terrain seams). Below the threshold the
// shift is zero and the historical absolute frame is reproduced exactly.
inline constexpr double kCameraRelativeOriginMeters = 1048576.0; // 2^20

// The render-origin shift for this eye: the eye itself beyond the
// threshold, the zero vector below it.
inline glm::dvec3 cameraRelativeShift(const glm::dvec3 &eyePlanetLocal)
{
    return glm::length(eyePlanetLocal) > kCameraRelativeOriginMeters ? eyePlanetLocal : glm::dvec3(0.0);
}

std::array<engine::renderer::VertexAttribute, 4> canonicalAttributes();
} // namespace demo::planet_demo

