// capabilities.hpp - public C capability surface of the physics_water module.
//
// Ocean and large water surface simulation plus local interactions over a
// physics_rigid world: a Gerstner wave sum (deep-water dispersion, analytic
// surface height and normal), localized ripple sources that propagate and
// decay over the surface (impacts, splashes, wakes), and a buoyancy pass
// that registers rigid bodies and applies submersion-sampled buoyancy
// forces, water drag, and angular damping through the rigid module's apply
// entry points. Consumers acquire it via
// ParticipantContext::acquireCapability<WaterCapability>(); the module
// declares physics_rigid as a required capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_water
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsWaterCapabilityAbi = 1u;

using OceanHandle = std::uint32_t;

inline constexpr OceanHandle invalidOcean = 0;

enum class WaterError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown ocean handle or unregistered body
    InvalidDescription, // null pointers, non-finite values, wrong wave counts
    CapacityExceeded,   // configured ocean, wave, or ripple budget exhausted
    SimulationFailure   // a force application rejected by the rigid world
};

inline constexpr std::uint32_t maximumWavesPerOcean = 8;

// One Gerstner wave component: a trochoidal deep-water wave. The wavelength
// sets the dispersion (phase speed follows the deep-water relation from
// gravity), the amplitude the crest height, and the steepness (0..1) the
// horizontal sharpening of the crests.
struct WaveDescription
{
    // Horizontal travel direction in the XZ plane (normalized on entry).
    double directionX = 1.0;
    double directionZ = 0.0;

    double wavelength = 30.0; // meters, > 0
    double amplitude  = 0.5;  // meters, >= 0
    double steepness  = 0.3;  // 0 = sinusoidal, 1 = sharpened crest, clamped [0, 1)
};

struct OceanDescription
{
    // Mean surface height (meters); waves displace around it.
    double waterLevel = 0.0;

    WaveDescription waves[maximumWavesPerOcean];
    std::uint32_t   waveCount = 3;
};

struct WaterStatistics
{
    std::uint32_t oceanCount       = 0;
    std::uint32_t registeredBodies = 0;
    std::uint32_t activeRipples    = 0;
    std::uint64_t buoyancyApplications = 0; // bodies the last buoyancy pass touched
};

// The "physics_water" capability. Calls are main-thread only; the module
// takes no internal locks. Positions are world-space meters, time is the
// consumer's fixed-step clock in seconds.
struct WaterCapability
{
    static constexpr const char *capabilityName = "physics_water";

    std::uint32_t abi = physicsWaterCapabilityAbi;

    void *self = nullptr;

    OceanHandle (*createOcean)(
        void *self, engine::physics_rigid::WorldHandle world, const OceanDescription *description, WaterError *error);
    void (*destroyOcean)(void *self, OceanHandle ocean);

    // Re-tunes the wave set and level (existing ripples survive).
    bool (*setOceanDescription)(void *self, OceanHandle ocean, const OceanDescription *description);

    // Surface query: writes the surface height at the world (x, z) and the
    // surface normal (unit, pointing out of the water) computed by central
    // differences of the wave sum plus every active ripple.
    bool (*sampleSurface)(void *self, OceanHandle ocean, double x, double z, double timeSeconds, double *outHeight,
        double outNormal[3]);

    // Adds a local ripple: a ring that expands from the center at the
    // module's configured propagation speed and decays exponentially. The
    // oldest ripple is recycled once the configured budget is full.
    bool (*addRipple)(void *self, OceanHandle ocean, const double center[3], double strength);

    // Advances the ripple field by one fixed step (expansion and decay).
    bool (*stepRipples)(void *self, OceanHandle ocean, double fixedDeltaSeconds);

    // Buoyancy registration: dynamic bodies only. Registered bodies receive
    // forces in applyBuoyancy until unregistered; destroying the body in the
    // rigid world makes the registration dormant (it is skipped and freed at
    // the next pass).
    bool (*registerBody)(void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body, WaterError *error);
    bool (*unregisterBody)(void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body);

    // Submersion query for one registered body: writes the estimated
    // submerged volume fraction in [0, 1] and the deepest sample point's
    // depth below the surface (meters, negative when fully above water).
    bool (*querySubmersion)(
        void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body, double timeSeconds, double *outFraction,
        double *outDepth);

    // The buoyancy pass: for every registered awake dynamic body, samples
    // the surface under the shape, applies the buoyancy force at the
    // submerged centroid, quadratic water drag, and angular damping through
    // the rigid module. Call once per fixed update before stepping the
    // rigid world.
    bool (*applyBuoyancy)(void *self, OceanHandle ocean, double timeSeconds, double fixedDeltaSeconds);

    bool (*waterStatistics)(void *self, WaterStatistics *outStatistics);
};

} // namespace engine::physics_water
