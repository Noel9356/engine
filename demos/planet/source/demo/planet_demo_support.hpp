// Private option, view-placement, and validation helpers for the planet demo.
#ifndef ENGINE_DEMO_PLANET_DEMO_SUPPORT_HPP
#define ENGINE_DEMO_PLANET_DEMO_SUPPORT_HPP

#include <engine/planet/capabilities.hpp>
#include <engine/planet/planet.hpp>
#include <engine/renderer/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <functional>
#include <array>
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
    std::uint64_t screenshotFrame = 0;
    std::uint32_t warmupFrames = 0;
    std::uint32_t statsEveryFrames = 30;
    bool orbit = false;
    bool caves = false;
    bool selfTest = false;
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
std::array<engine::renderer::VertexAttribute, 4> canonicalAttributes();
} // namespace demo::planet_demo

#endif
