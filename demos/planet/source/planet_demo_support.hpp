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
    // Optional biome-pin for the surface view ("--biome taiga"): the view
    // picker then requires the spot's biome to be exactly this one, so a
    // capture shows that climate zone's ground, flora and water. Empty = the
    // standard per-view windows.
    std::string biomeName;
    std::uint64_t seed = 0;
    double radius = 0.0;
    std::uint64_t frames = 0;
    std::string screenshotPath;
    std::string frameTimesPath;
    std::uint64_t screenshotFrame = 0;
    bool screenshotFrameSet = false; // --screenshot-frame appeared (0 is a valid index)
    std::uint32_t warmupFrames = 0;
    // Wall-clock warmup window: while it is open the warmup budget is not
    // consumed - warmup runs until the visible set is fully built or this
    // many seconds elapse, whichever comes first (0 keeps the pure frame
    // budget). Deep ground views want thousands of chunks and deliver at
    // the build pool's own rate, so a FRAME count large enough for them
    // depends on the machine; the seconds window expresses "wait until it
    // looks right" directly.
    double settleSeconds = 0.0;
    std::uint32_t statsEveryFrames = 30;
    // --biome-census <n>: sample the climate field over a golden-spiral of n
    // points and print the biome histogram, then exit. Answers "which climate
    // zones does this planet actually realize" straight from the generator
    // (a classifier zone can be structurally absent on a given seed - the
    // census reports zero coverage for it instead of a sample silently
    // degrading to the fallback view).
    std::uint32_t biomeCensusSamples = 0;
    bool orbit = false;
    bool caves = false;
    bool chunkBorders = false;
    bool selfTest = false;
    bool benchmark = false;
    // Screen pixels one texture tile must span for the terrain detail
    // multiply to apply (see the gate setup in main.cpp): the budget keeps
    // the multiply where a tile still photographs as detail and cuts it
    // where the scan's motif has minified into a repeating stamp (the
    // per-chunk gate plus the mip chain fade the boundary so no ring
    // shows where the multiply stops). The floor sits at the edge of the
    // wallpaper band: between roughly 12 and 30 pixels per tile the eye
    // stops reading a tile's content as grain and starts matching the tile
    // lattice itself, so the multiply only earns its cost past that band.
    // 0 disables the terrain textures outright.
    std::uint32_t textureTilePixels = 64;
};

struct Viewpoint
{
    engine::planet::Vec3 direction;
    double surfaceRadius = 0.0;
    bool hasLookTangent = false;
    engine::planet::Vec3 lookTangent = {};
};

enum class ProbePolicy { Inland, Coastline, Mountain, Pinned };

struct SelfTestResult
{
    bool pass = true;
    std::string report;
};

int runBenchmark(const engine::planet::PlanetGenerationParams &params);

// Biome name table (the 16 climate zones the classifier produces): maps the
// CLI spelling to the Biome id and back. Shared by --biome validation and
// the view placement.
bool biomeFromName(const std::string &name, engine::planet::Biome &outBiome);
const char *biomeName(engine::planet::Biome biome);
const std::vector<std::string> &biomeNames();

const char *planetOptionsUsage() noexcept;
bool parsePlanetOptions(const std::vector<std::string> &arguments, PlanetDemoOptions &options, std::string &outError);
std::vector<engine::planet::Vec3> goldenSpiralDirections(std::uint32_t count);
engine::planet::Vec3 rotateDirection(const engine::planet::Vec3 &direction, const engine::planet::Vec3 &axis,
    double angleRadians);
void clampCameraPitch(const glm::dvec3 &eye, const glm::dvec3 &localUp, glm::dvec3 &target,
    double minPitchRadians, double maxPitchRadians);
bool pickViewpoint(engine::planet::PlanetCapability &planetCapability, engine::planet::PlanetHandle planet,
    const std::function<bool(double, engine::planet::Biome, double, const engine::planet::PlanetGenerationParams &)> &predicate,
    ProbePolicy policy, double eyeHeight, const engine::planet::Vec3 &sunDirection, Viewpoint &out,
    const engine::planet::Biome *neighborhoodBiome = nullptr, std::size_t choiceIndex = 0);
SelfTestResult runSelfTest(engine::planet::PlanetCapability &capability, void *capabilitySelf,
    const PlanetDemoOptions &options);
// Biome census (--biome-census <n>): samples the climate field over n
// golden-spiral directions and prints each biome's sample count and share of
// the globe. Returns the number of distinct biomes observed.
std::size_t runBiomeCensus(engine::planet::PlanetCapability &capability, engine::planet::PlanetHandle planet,
    const engine::planet::PlanetGenerationParams &params, std::uint32_t samples);
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

