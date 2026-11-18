// sandbox_scene.hpp - scene construction and scripted interaction for the
// physics sandbox demo. The scene owns no simulation state itself: it creates
// bodies, figures, and cloths inside a physics_rigid world through the three
// physics capabilities and keeps only handles plus the scripted timeline.
#pragma once

#include <engine/kernel/participant_interface.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace demo::physics_sandbox
{

// Primitive kinds the demo spawns and renders.
enum class PrimitiveKind : std::uint32_t
{
    Box = 0,
    Sphere,
    Capsule
};

// One demo-tracked rigid body: the handle plus its render palette entry.
struct SceneBody
{
    engine::physics_rigid::BodyHandle body         = engine::physics_rigid::invalidBody;
    PrimitiveKind                     kind         = PrimitiveKind::Box;
    float                             baseColor[3] = {0.8f, 0.8f, 0.8f};
    bool                              scripted = false; // created by the capture timeline (kept out of reset counting)
};

// Handles to everything the scene built inside the rigid world.
struct SceneHandles
{
    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    std::vector<SceneBody>                                 bodies;
    std::vector<engine::physics_articulated::FigureHandle> figures;
    std::vector<engine::physics_cloth::ClothHandle>        cloths;

    // Link body handles of the grabbable chain figure (for stats).
    std::uint32_t chainLinkCount    = 0;
    std::uint32_t pendulumLinkCount = 0;

    // The kinematic paddle swept by the scripted timeline.
    engine::physics_rigid::BodyHandle paddle = engine::physics_rigid::invalidBody;
};

struct SceneDependencies
{
    engine::physics_rigid::PhysicsRigidCapability      *rigid        = nullptr;
    engine::physics_articulated::ArticulatedCapability *articulation = nullptr;
    engine::physics_cloth::ClothCapability             *cloth        = nullptr;
};

// Scene tuning read once from the [physics_sandbox] configuration section.
struct SceneConfig
{
    double gravity[3]        = {0.0, -9.81, 0.0};
    double groundExtent      = 40.0;
    double spawnAhead        = 3.0;
    double grabStiffness     = 60.0;
    double grabDamping       = 6.0;
    double grabMaximumForce  = 4000.0;
    double blastRadius       = 5.0;
    double blastImpulse      = 260.0;
    double windStrength      = 3.0;
    double paddleSpan        = 6.0;
    double paddleSpeed       = 0.9;
    double pendulumSwing     = 0.6;
    double pendulumFrequency = 0.25;
};

// Reads the demo's tuning section with built-in defaults.
SceneConfig readSceneConfig(engine::ParticipantContext &context);

// Creates the rigid world and populates it per the scene selector. Returns
// false when a capability rejects a creation request (the report names it).
bool buildScene(const SceneDependencies &dependencies, const SceneConfig &config, const std::string &sceneName,
    SceneHandles &outHandles, std::string &outReport);

// Destroys every created figure, cloth, and body plus the world itself.
void destroyScene(const SceneDependencies &dependencies, SceneHandles &handles);

// Spawns one dynamic primitive 2 meters in front of the given pose.
engine::physics_rigid::BodyHandle spawnPrimitive(const SceneDependencies &dependencies, const SceneConfig &config,
    SceneHandles &handles, PrimitiveKind kind, const double origin[3], const double forward[3]);

// Radial impulse (the "blast" action): pushes every dynamic body within
// radius of the center away from it.
void applyBlast(
    const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles, const double center[3]);

// One scripted-timeline step: deterministic spawns, impulses, paddle motion,
// and cloth wind driven purely by the fixed-step index.
void updateScriptedTimeline(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    double simulatedSeconds, double fixedDeltaSeconds);

} // namespace demo::physics_sandbox
