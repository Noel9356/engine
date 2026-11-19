// sandbox_extended.hpp - the extended physics scenes: the force-field, rope,
// water, fluid, particle, foliage, and character scenarios built on the
// extended physics modules. The scene rules from sandbox_scene.hpp apply
// unchanged: no simulation state lives here, every object is created and
// stepped through capability calls, and the scripted timeline keys off the
// fixed-step clock only.
#pragma once

#include "sandbox_scene.hpp"

namespace demo::physics_sandbox
{

// Builds one of the extended scenes ("fields", "rope", "water", "fluid",
// "particles", "foliage", "character"); returns false with a report when a
// required capability is missing or a creation call was rejected.
bool buildExtendedScene(const SceneDependencies &dependencies, const SceneConfig &config, const std::string &sceneName,
    SceneHandles &outHandles, std::string &outReport);

// Destroys every extended-scene object the modules own (ropes, ocean,
// fluid, emitters, foliage field, character records). Call before the rigid
// world goes away.
void destroyExtendedScene(const SceneDependencies &dependencies, SceneHandles &handles);

// Per-fixed-update driving passes. stepExtendedBeforeRigid applies forces
// (wind drag, force volumes, buoyancy) and advances the stateless-ish
// systems (foliage springs, particles, character control) so the rigid step
// integrates with them in the same tick; stepExtendedAfterRigid steps the
// rope and fluid chains against the freshly integrated bodies.
void stepExtendedBeforeRigid(const SceneDependencies &dependencies, SceneHandles &handles, double simulatedSeconds,
    double fixedDeltaSeconds);
void stepExtendedAfterRigid(
    const SceneDependencies &dependencies, SceneHandles &handles, double simulatedSeconds, double fixedDeltaSeconds);

// Scripted events for the extended scenes (deterministic, keyed off the
// fixed-step clock): periodic splash ripples, foliage disturbances, the
// character's pose wave, and the fluid's dam-break trigger.
void updateExtendedScriptedTimeline(const SceneDependencies &dependencies, SceneHandles &handles,
    double simulatedSeconds, double fixedDeltaSeconds);

// Simulation-LOD driver for the "lod" layout: writes every tracked body's
// importance from its distance to the camera (1 inside lodNearDistance,
// falling linearly to 0 at lodFarDistance), so the solver spends its
// iteration budgets where the camera actually looks. Call once per fixed
// update; the modules' default importance of 1.0 keeps every other layout
// at full fidelity without calling this.
void driveSimulationLod(const SceneDependencies &dependencies, const SceneHandles &handles,
    const SceneConfig &config, const double cameraPosition[3]);

} // namespace demo::physics_sandbox
