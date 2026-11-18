// capabilities.hpp - public C capability surface of the physics_foliage
// module.
//
// Procedural foliage deformation: grass blades, bushes, and tree crowns
// represented as instances that bend under a wind field with a per-instance
// spring-damper (target bend from the wind sample plus a phase-offset sway
// cycle), radial disturbances (footsteps, vehicles, blasts) that push
// instances away with a spring-back, and per-instance stiffness so young
// shoots and thick trunks react differently. The module samples the
// physics_fields wind capability when its wind handle is bound and falls
// back to a configured ambient breeze otherwise. Consumers acquire it via
// ParticipantContext::acquireCapability<FoliageCapability>(); the module
// declares physics_rigid as a required capability and physics_fields as an
// optional one.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_foliage
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsFoliageCapabilityAbi = 1u;

using FieldHandle = std::uint32_t;

inline constexpr FieldHandle invalidField = 0;

enum class FoliageError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown field handle or instance index
    InvalidDescription, // null pointers, non-finite values, non-positive sizes
    CapacityExceeded,   // configured field or instance budget exhausted
    SimulationFailure   // non-finite state entered during a step; the field is left unmodified
};

// One deformable foliage instance. The base is planted at `position`; the
// instance bends around its base. Grass blades use small heights and low
// stiffness, bushes medium, tree crowns large heights with high stiffness
// so the bend angle stays small but visible at the tips.
struct FoliageInstance
{
    double position[3] = {0.0, 0.0, 0.0}; // base (root) point, world space

    double height = 0.5; // meters, > 0

    // Bend resistance in [0, 1]: 0 bends fully with the wind target, 1 stays
    // near vertical.
    double stiffness = 0.4;

    // Per-instance sway phase seed (radians) and amplitude scale so a field
    // waves in traveling patterns instead of moving in unison.
    double phaseSeed    = 0.0;
    double swayScale    = 1.0;
};

struct FoliageFieldDescription
{
    const FoliageInstance *instances     = nullptr;
    std::uint32_t          instanceCount = 0;

    // Sway frequency of the phase cycle (hertz) shared by the field; the
    // per-instance phaseSeed offsets it.
    double swayFrequency = 1.1;

    // How strongly the wind velocity maps into the bend target (meters of
    // tip displacement per meter per second of wind).
    double bendScale = 0.06;

    // Spring-damper for the per-instance bend state: higher stiffness values
    // snap back faster after a disturbance; damping suppresses ringing.
    double springStiffness = 26.0;
    double springDamping   = 6.5;
};

// One bend result per instance: the tip displacement direction (unit,
// horizontal) and the bend angle in radians (the instance rotates about the
// horizontal axis perpendicular to the bend direction). Consumers compose
// the instance's model matrix as translate(position) * rotate(axis, angle).
struct FoliageBend
{
    double bendDirectionX = 0.0;
    double bendDirectionZ = 0.0;
    double bendAngleRadians = 0.0;
};

struct FoliageStatistics
{
    std::uint32_t fieldCount       = 0;
    std::uint32_t instanceCount    = 0;
    std::uint64_t disturbanceTotal = 0;
};

// The "physics_foliage" capability. Calls are main-thread only; the module
// takes no internal locks. Positions are world-space meters; wind velocity
// sampling uses the physics_fields capability when bound.
struct FoliageCapability
{
    static constexpr const char *capabilityName = "physics_foliage";

    std::uint32_t abi = physicsFoliageCapabilityAbi;

    void *self = nullptr;

    // Creates a field over the instances (copied). windField binds a
    // physics_fields wind handle for wind sampling; 0 uses the configured
    // ambient breeze.
    FieldHandle (*createField)(void *self, const FoliageFieldDescription *description,
        std::uint32_t windFieldHandle, FoliageError *error);
    void (*destroyField)(void *self, FieldHandle field);

    // Bend results: fills up to capacity FoliageBend records in instance
    // order; outRequired receives the instance count.
    bool (*copyBends)(void *self, FieldHandle field, FoliageBend *outBends, std::uint32_t capacity,
        std::uint32_t *outFilled, std::uint32_t *outRequired);

    // Radial disturbance: instances within `radius` of the point get pushed
    // away (a footstep, a passing vehicle, a blast) and spring back.
    bool (*applyRadialDisturbance)(
        void *self, FieldHandle field, const double position[3], double radius, double strength);

    // Advances every field's spring state by one fixed step. Keep the delta
    // constant across a running simulation.
    bool (*step)(void *self, double fixedDeltaSeconds);

    // Accumulator convenience mirroring the rigid module's simulate: adds
    // elapsedSeconds and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed.
    std::uint32_t (*simulate)(void *self, double elapsedSeconds);

    bool (*foliageStatistics)(void *self, FoliageStatistics *outStatistics);
};

} // namespace engine::physics_foliage
