// world.hpp - world and body lifecycle, the fixed-step pipeline, and the
// query API (ray casts, sphere overlap, statistics). The module class in
// physics_rigid.cpp binds these operations into the capability table.

#pragma once

#include "records.hpp"
#include "solver.hpp"

#include <chrono>
#include <cstdint>

namespace engine::physics_rigid
{

// Creates a world with the module's settings and the description's gravity.
World *createWorld(std::vector<World> &worlds, std::unordered_map<WorldHandle, std::size_t> &indexByHandle,
    WorldHandle &nextHandle, const SimulationSettings &settings, const WorldDescription &description);

// Validates a body description against the module's budgets and the shape
// rules (planes must be static); fills the mass properties. Returns false
// with the error code filled on rejection.
bool validateBodyDescription(const World &world, const SimulationSettings &settings, const BodyDescription &description,
    Body &outBody, PhysicsError *error);

// Recomputes mass properties after a shape/kind/mass change.
void recomputeMassProperties(Body &body);

// Removes a body together with its joints and contact manifolds.
bool destroyBodyFromWorld(World &world, BodyHandle handle);

// Removes one joint record.
bool destroyJointFromWorld(World &world, JointHandle handle);

// Refreshes a body's world-space AABB from its transform.
void refreshBodyAabb(Body &body);

// Scratches reused across steps to avoid per-step allocations.
struct StepScratch
{
    std::vector<ContactConstraint>                   contacts;
    std::vector<JointConstraint>                     jointConstraints;
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    std::vector<std::size_t>                         islandParent;
};

// One fixed simulation step: force integration, broadphase, narrowphase,
// warm-started sequential impulse solving, position integration, continuous
// collision detection for CCD-enabled bodies, island-based sleeping.
// Returns false (and leaves the world untouched) when the step would produce
// non-finite state.
bool stepWorld(World &world, double fixedDeltaSeconds, StepScratch &scratch);

// Closest-hit ray cast; direction is normalized internally.
bool rayCastWorld(
    const World &world, const math::Vec3 &origin, const math::Vec3 &direction, double maximumDistance, RayHit &outHit);
// Closest hit that skips the listed bodies (a character controller probing
// the ground under its own rig).
bool rayCastWorldIgnoring(const World &world, const math::Vec3 &origin, const math::Vec3 &direction,
    double maximumDistance, const BodyHandle *ignoredBodies, std::uint32_t ignoredBodyCount, RayHit &outHit);

// Closest-hit sphere sweep against every body and plane: the query sphere
// moves from start along delta and the first contact wins.
bool sphereCastWorld(const World &world, const math::Vec3 &start, const math::Vec3 &delta, double radius, SweepHit &outHit);

// Deepest sphere overlap against bodies and planes.
bool collideSphereWorld(const World &world, const math::Vec3 &center, double radius, math::Vec3 &outNormal,
    double &outDepth, BodyHandle &outBody);

// Fills the cumulative statistics record.
void fillWorldStatistics(const World &world, WorldStatistics &outStatistics);

} // namespace engine::physics_rigid
