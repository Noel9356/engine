// Public capability surface of the planet module.
//
// Declares the POD types and function-pointer table consumers receive through
// ParticipantContext::acquireCapability for the "planet" capability (ABI 1):
// creating deterministic procedural planets, streaming their LOD chunk meshes
// as renderer-ready vertex/index views, sampling surface elevation and
// climate, and hashing chunks for determinism self-tests.

#ifndef ENGINE_PLANET_CAPABILITIES_HPP
#define ENGINE_PLANET_CAPABILITIES_HPP

#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <cstdint>

namespace engine
{
namespace planet
{

// Defined in planet.hpp (borrowed view of one drawable chunk mesh); only
// referenced by pointer here so capability consumers may forward-declare it.
struct PlanetChunkView;

// ABI version of this capability struct; consumers must match it.
inline constexpr std::uint32_t planetCapabilityAbi = 1u;

using PlanetHandle        = std::uint32_t;
inline constexpr PlanetHandle invalidPlanet = 0;

// Status codes written through optional PlanetError out-parameters.
enum class PlanetError : std::uint32_t
{
    None = 0,
    InvalidDescription,
    InvalidHandle,
    CapacityReached
};

// Per-planet overrides layered over the module's [planet] configuration.
// radiusMeters <= 0 keeps the configured radius; seed == 0 keeps the
// configured seed; overrideCenter false keeps the configured center.
struct PlanetDescription
{
    std::uint64_t seed              = 0;
    double        radiusMeters      = 0.0;
    double        seaLevelOffsetMeters = 0.0;
    bool          overrideCenter    = false;
    double        centerMeters[3]   = {0.0, 0.0, 0.0};
};

// Function-pointer table for the "planet" capability. All calls are
// main-thread only; failures are reported via the error out-parameters.
struct PlanetCapability
{
    static constexpr const char *capabilityName = "planet";

    std::uint32_t abi  = planetCapabilityAbi;
    void         *self = nullptr;

    PlanetHandle (*createPlanet)(void *self, const PlanetDescription *description, PlanetError *error);
    void (*destroyPlanet)(void *self, PlanetHandle planet);

    // Advances LOD selection/builds for this frame's eye position (world
    // coordinates; the module translates into planet-local space).
    void (*updatePlanet)(void *self, PlanetHandle planet, const double eyeWorld[3]);

    // Copies borrowed views of the visible ready chunks (terrain + water).
    // Pointers stay valid until the next updatePlanet. Returns the count
    // written; call visibleChunkCapacity first to size the buffer.
    std::uint32_t (*visibleChunkCapacity)(void *self, PlanetHandle planet);
    std::uint32_t (*copyVisibleChunks)(void *self, PlanetHandle planet, PlanetChunkView *output, std::uint32_t capacity);

    // Pure sampling: surface radius along a unit direction (world-oriented;
    // false when the planet handle is invalid).
    bool (*sampleSurfaceRadius)(void *self, PlanetHandle planet, const double directionUnit[3], double *outRadiusMeters);
    // Signed field density in meters at a world point (positive = inside
    // rock): consumers placing cameras or probes can detect buried eyes.
    bool (*sampleDensity)(void *self, PlanetHandle planet, const double pointWorld[3], double *outDensity);
    bool (*sampleClimate)(
        void *self, PlanetHandle planet, const double directionUnit[3], double radiusMeters, ClimateSample *outSample);

    // Radial relief (heightfield part) along a unit direction: terrain height
    // above the datum plus the mountain/cliff masks - consumers can locate
    // specific landforms (terraced cliff country, mountains) without
    // evaluating the density field themselves.
    bool (*sampleRelief)(
        void *self, PlanetHandle planet, const double directionUnit[3], ReliefSample *outRelief);

    // Statistics snapshot (chunks, triangles, timings).
    bool (*copyStatistics)(void *self, PlanetHandle planet, PlanetStatistics *outStatistics);
    bool (*copyGenerationParams)(void *self, PlanetHandle planet, PlanetGenerationParams *outParams);

    // Determinism self-test hook: synchronous pure build + FNV-1a hash of
    // one chunk's mesh (independent of the streaming cache).
    std::uint64_t (*hashChunk)(
        void *self, PlanetHandle planet, std::uint32_t depth, std::uint32_t x, std::uint32_t y, std::uint32_t z);
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_CAPABILITIES_HPP
