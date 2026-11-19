// world.hpp - world and body lifecycle, the fixed-step pipeline, and the
// query API (ray casts, sphere overlap, statistics). The module class in
// physics_rigid.cpp binds these operations into the capability table.

#pragma once

#include "broadphase.hpp"
#include "records.hpp"
#include "solver.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

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

// Validates a trigger description (solid shapes only, attached bodies must
// exist); fills the trigger record. Returns false with the error code filled
// on rejection.
bool validateTriggerDescription(
    const World &world, const TriggerDescription &description, Trigger &outTrigger, PhysicsError *error);

// Removes one trigger record and drops its pending events.
bool destroyTriggerFromWorld(World &world, TriggerHandle handle);

// Evaluates every trigger once: composes attached poses, collects overlaps
// in deterministic order, and appends enter/exit events to the world queue.
void evaluateTriggers(World &world, std::vector<BodyHandle> &overlapScratch);

// Refreshes a body's world-space AABB from its transform.
void refreshBodyAabb(Body &body);

// Pre-integration transforms of one pipeline pass, restored when the pass
// diverges into non-finite state. Lives in the step scratch so the capture
// reuses warmed-up buffers instead of allocating per substep.
struct StateSnapshot
{
    std::vector<math::Vec3>       positions;
    std::vector<math::Quaternion> rotations;
    std::vector<math::Vec3>       linearVelocities;
    std::vector<math::Vec3>       angularVelocities;

    void capture(const World &world);
    void restore(World &world) const;
};

// Scratches reused across steps to avoid per-step allocations.
struct StepScratch
{
    std::vector<ContactConstraint>                   contacts;
    std::vector<JointConstraint>                     jointConstraints;
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    std::vector<std::size_t>                         islandParent;
    std::vector<BodyHandle>                          triggerOverlapScratch;

    // Broadphase grid entries, rebuilt every step.
    BroadphaseScratch broadphase;

    // Per-island "whole island below the sleep tolerance" flags, indexed by
    // body index (a body index is also its union-find root slot), plus the
    // list of roots the pass touched so the flags reset without clearing the
    // whole array.
    std::vector<std::uint8_t> islandBelowByRoot;
    std::vector<std::size_t>  islandTouchedRoots;

    // Pre-integration positions of the CCD-enabled dynamic bodies.
    std::vector<math::Vec3> ccdPreviousPositions;

    // Non-finite rollback snapshot.
    StateSnapshot snapshot;
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

// Deepest sphere overlap for a batch of spheres in one pass: each query
// resolves exactly as collideSphereWorld does, with the solid bodies
// pre-filtered through their shape bounds. Results are written in input
// order; the return value is the number of queries processed.
std::uint32_t collideSpheresWorld(const World &world, const SphereOverlapQuery *queries, std::uint32_t queryCount,
    SphereOverlapResult *outResults);

// Fills the cumulative statistics record.
void fillWorldStatistics(const World &world, WorldStatistics &outStatistics);

} // namespace engine::physics_rigid
