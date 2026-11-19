// sandbox_scene.hpp - scene construction and scripted interaction for the
// physics sandbox demo. The scene owns no simulation state itself: it creates
// bodies, figures, and cloths inside a physics_rigid world through the three
// physics capabilities and keeps only handles plus the scripted timeline.
#pragma once

#include <engine/kernel/participant_interface.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_character/capabilities.hpp>
#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_foliage/capabilities.hpp>
#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_particles/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/physics_rope/capabilities.hpp>
#include <engine/physics_scheduling/capabilities.hpp>
#include <engine/physics_water/capabilities.hpp>

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

// Handles to the extended-module objects one scene may carry. Everything
// here is owned by its module; the demo only keeps the ids.
struct ExtendedHandles
{
    // physics_fields.
    engine::physics_fields::WindHandle        windField        = engine::physics_fields::invalidWind;
    engine::physics_fields::ForceVolumeHandle forceVolume      = engine::physics_fields::invalidForceVolume;

    // physics_rope.
    std::vector<engine::physics_rope::RopeHandle> ropes;

    // physics_water: the ocean plus the bodies registered for buoyancy.
    engine::physics_water::OceanHandle ocean = engine::physics_water::invalidOcean;
    std::vector<engine::physics_rigid::BodyHandle> buoyantBodies;

    // physics_fluid.
    engine::physics_fluid::FluidHandle fluid = engine::physics_fluid::invalidFluid;

    // physics_particles: one emitter per kind, in EmitterKind order (rain,
    // snow, dust, fire, hail, ash, leaves, pollen, insects); unused slots
    // stay invalid.
    static constexpr std::uint32_t kEmitterSlotCount = 9;
    engine::physics_particles::EmitterHandle emitters[kEmitterSlotCount] = {};

    // The "lod" layout's kinematic shaker platforms (driven every fixed
    // update by the extended timeline so their crates never settle).
    std::vector<engine::physics_rigid::BodyHandle> shakerPlatforms;

    // physics_foliage: the bend field plus a copy of the instance layout for
    // the renderer (base positions, heights, seeds).
    engine::physics_foliage::FieldHandle foliageField = engine::physics_foliage::invalidField;
    std::vector<engine::physics_foliage::FoliageInstance> foliageInstances;

    // physics_character.
    engine::physics_character::CharacterHandle   character   = engine::physics_character::invalidCharacter;
    engine::physics_character::JiggleChainHandle jiggleChain = engine::physics_character::invalidJiggleChain;
    engine::physics_character::FootPlanterHandle footPlanter = engine::physics_character::invalidFootPlanter;
    engine::physics_rigid::BodyHandle            characterTorsoBody = engine::physics_rigid::invalidBody;

    // The jiggle chain layout the renderer replays (rest offsets per bone in
    // the torso's frame).
    std::uint32_t jiggleBoneCount      = 0;
    double        jiggleOffsets[4][3] = {};

    // The fur ball: a rigid sphere drawn through the renderer's shell-fur
    // path with the physics_fur module's texture; the wind field drives its sway.
    bool                              furBall     = false;
    engine::physics_rigid::BodyHandle furBallBody = engine::physics_rigid::invalidBody;
};

// Handles to everything the scene built inside the rigid world.
struct SceneHandles
{
    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    std::vector<SceneBody>                                 bodies;
    std::vector<engine::physics_articulated::FigureHandle> figures;
    std::vector<engine::physics_cloth::ClothHandle>        cloths;

    ExtendedHandles extended;

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
    engine::physics_fields::FieldsCapability           *fields       = nullptr;
    engine::physics_rope::RopeCapability               *rope         = nullptr;
    engine::physics_water::WaterCapability             *water        = nullptr;
    engine::physics_fluid::FluidCapability             *fluid        = nullptr;
    engine::physics_particles::ParticlesCapability     *particles    = nullptr;
    engine::physics_foliage::FoliageCapability         *foliage      = nullptr;
    engine::physics_character::CharacterCapability     *character    = nullptr;
    engine::physics_scheduling::SchedulingCapability   *scheduling   = nullptr;
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
    double paddleFrequency   = 0.15;
    double pendulumSwing     = 0.6;
    double pendulumFrequency = 0.25;

    // Simulation-LOD driving (the "lod" layout): every tracked body's
    // importance falls linearly from 1 inside lodNearDistance of the camera
    // to 0 at lodFarDistance, so the near platform stacks solve at full
    // iteration budgets while the distant ones ride the reduced bands.
    bool   driveSimulationLod = true;
    double lodNearDistance    = 12.0;
    double lodFarDistance     = 90.0;
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
