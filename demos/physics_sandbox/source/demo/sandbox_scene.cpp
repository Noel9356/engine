// sandbox_scene.cpp - scene construction and the scripted capture timeline.
// Every object is created through a capability call, so the scene module
// holds no simulation state of its own; the physics modules own everything.
#include "sandbox_scene.hpp"

#include "sandbox_extended.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>

namespace demo::physics_sandbox
{
namespace
{

namespace math = engine::physics_rigid::math;

using engine::physics_rigid::BodyDescription;
using engine::physics_rigid::BodyHandle;
using engine::physics_rigid::ShapeDescription;
using engine::physics_rigid::ShapeKind;
using engine::physics_rigid::Vec3;

constexpr double kPi = 3.14159265358979323846;

BodyHandle createBody(
    const SceneDependencies &dependencies, engine::physics_rigid::WorldHandle world, const BodyDescription &description)
{
    return dependencies.rigid->createBody(dependencies.rigid->self, world, &description, nullptr);
}

// Colors are linear-space RGB values tuned for the demo's linear headless
// exposure (see the [pbr] notes in config/engine.toml).
void pushBody(SceneHandles &handles, BodyHandle body, PrimitiveKind kind, const float color[3], bool scripted)
{
    handles.bodies.push_back(SceneBody{body, kind, {color[0], color[1], color[2]}, scripted});
}

ShapeDescription boxShape(double x, double y, double z)
{
    ShapeDescription shape;
    shape.kind              = ShapeKind::Box;
    shape.boxHalfExtents[0] = x;
    shape.boxHalfExtents[1] = y;
    shape.boxHalfExtents[2] = z;
    return shape;
}

ShapeDescription sphereShape(double radius)
{
    ShapeDescription shape;
    shape.kind         = ShapeKind::Sphere;
    shape.sphereRadius = radius;
    return shape;
}

ShapeDescription capsuleShape(double radius, double halfLength)
{
    ShapeDescription shape;
    shape.kind              = ShapeKind::Capsule;
    shape.capsuleRadius     = radius;
    shape.capsuleHalfLength = halfLength;
    return shape;
}

ShapeDescription planeShape(double y)
{
    ShapeDescription shape;
    shape.kind           = ShapeKind::Plane;
    shape.planeNormal[0] = 0.0;
    shape.planeNormal[1] = 1.0;
    shape.planeNormal[2] = 0.0;
    shape.planeOffset    = y;
    return shape;
}

// Pitch about the X axis for the ramps (rotation applied in the body frame).
engine::physics_rigid::Quaternion pitchQuaternion(double radians)
{
    return engine::physics_rigid::Quaternion{std::sin(radians * 0.5), 0.0, 0.0, std::cos(radians * 0.5)};
}

void addStaticScenery(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles)
{
    // The catching plane plus a thin visual slab so the floor extent reads
    // in captures (the plane is infinite; the slab is decorative).
    BodyDescription ground;
    ground.shape              = planeShape(0.0);
    ground.transform.position = Vec3{0.0, 0.0, 0.0};
    ground.kind               = engine::physics_rigid::BodyKind::Static;
    ground.material.friction  = 0.8;
    createBody(dependencies, handles.world, ground);

    BodyDescription floorSlab;
    floorSlab.shape              = boxShape(config.groundExtent * 0.5, 0.05, config.groundExtent * 0.5);
    floorSlab.transform.position = Vec3{0.0, -0.05, 0.0};
    floorSlab.kind               = engine::physics_rigid::BodyKind::Static;
    floorSlab.material.friction  = 0.8;
    floorSlab.userTag            = 1;
    const float groundColor[3]   = {0.30f, 0.33f, 0.38f};
    pushBody(handles, createBody(dependencies, handles.world, floorSlab), PrimitiveKind::Box, groundColor, false);

    // A ramp the cloth can drape over and the stack can be pushed against.
    BodyDescription ramp;
    ramp.shape               = boxShape(3.0, 0.15, 2.0);
    ramp.transform.position  = Vec3{-1.5, 1.1, -1.5};
    ramp.transform.rotation  = pitchQuaternion(0.30);
    ramp.kind                = engine::physics_rigid::BodyKind::Static;
    ramp.material.friction   = 0.9;
    const float rampColor[3] = {0.42f, 0.35f, 0.28f};
    pushBody(handles, createBody(dependencies, handles.world, ramp), PrimitiveKind::Box, rampColor, false);

    // A back wall stops stray objects without hiding the scene.
    BodyDescription wall;
    wall.shape               = boxShape(9.0, 2.0, 0.2);
    wall.transform.position  = Vec3{0.0, 2.0, -7.5};
    wall.kind                = engine::physics_rigid::BodyKind::Static;
    wall.material.friction   = 0.7;
    const float wallColor[3] = {0.52f, 0.48f, 0.43f};
    pushBody(handles, createBody(dependencies, handles.world, wall), PrimitiveKind::Box, wallColor, false);
}

void addBoxStack(const SceneDependencies &dependencies, SceneHandles &handles)
{
    const float colors[5][3]
        = {{0.85f, 0.52f, 0.18f},
            {0.72f, 0.27f, 0.20f},
            {0.87f, 0.68f, 0.24f},
            {0.55f, 0.66f, 0.30f},
            {0.30f, 0.55f, 0.72f}};
    for (std::uint32_t index = 0; index < 5; ++index)
    {
        BodyDescription box;
        box.shape              = boxShape(0.35, 0.35, 0.35);
        box.transform.position = Vec3{-3.2, 0.351 + index * 0.704, 2.4};
        box.kind               = engine::physics_rigid::BodyKind::Dynamic;
        box.density            = 320.0;
        box.material.friction  = 0.55;
        pushBody(handles, createBody(dependencies, handles.world, box), PrimitiveKind::Box, colors[index], false);
    }
}

void addSphereCluster(const SceneDependencies &dependencies, SceneHandles &handles)
{
    const float colors[3][3] = {{0.80f, 0.24f, 0.22f}, {0.24f, 0.52f, 0.82f}, {0.30f, 0.74f, 0.40f}};
    for (std::uint32_t index = 0; index < 3; ++index)
    {
        BodyDescription sphere;
        sphere.shape                = sphereShape(0.32);
        sphere.transform.position   = Vec3{3.0 + (index % 2) * 0.75, 0.5 + index * 0.62, 2.2 + (index / 2) * 0.75};
        sphere.kind                 = engine::physics_rigid::BodyKind::Dynamic;
        sphere.density              = 420.0;
        sphere.material.restitution = 0.35;
        sphere.material.friction    = 0.4;
        pushBody(handles, createBody(dependencies, handles.world, sphere), PrimitiveKind::Sphere, colors[index], false);
    }
}

void addCapsuleRow(const SceneDependencies &dependencies, SceneHandles &handles)
{
    const float colors[3][3] = {{0.62f, 0.35f, 0.72f}, {0.72f, 0.55f, 0.30f}, {0.35f, 0.72f, 0.68f}};
    for (std::uint32_t index = 0; index < 3; ++index)
    {
        BodyDescription capsule;
        capsule.shape              = capsuleShape(0.22, 0.4);
        capsule.transform.position = Vec3{0.4 + index * 0.8, 0.24, 3.4};
        // Lay the capsules along Z by rotating 90 degrees about X.
        capsule.transform.rotation
            = engine::physics_rigid::Quaternion{std::sin(kPi * 0.25), 0.0, 0.0, std::cos(kPi * 0.25)};
        capsule.kind              = engine::physics_rigid::BodyKind::Dynamic;
        capsule.density           = 380.0;
        capsule.material.friction = 0.6;
        pushBody(
            handles, createBody(dependencies, handles.world, capsule), PrimitiveKind::Capsule, colors[index], false);
    }
}

void addChainFigure(const SceneDependencies &dependencies, SceneHandles &handles)
{
    if (dependencies.articulation == nullptr)
    {
        return;
    }
    constexpr std::uint32_t                                   linkCount = 8;
    std::vector<engine::physics_articulated::LinkDescription> links(linkCount);
    for (std::uint32_t index = 0; index < linkCount; ++index)
    {
        engine::physics_articulated::LinkDescription &link = links[index];
        link.parentIndex                                   = index == 0 ? -1 : static_cast<std::int32_t>(index - 1);
        if (index == 0)
        {
            // Anchor knob: a small static sphere the chain hangs from.
            link.shape                      = sphereShape(0.16);
            link.relativeTransform.position = Vec3{5.2, 6.4, -3.4};
        }
        else
        {
            link.shape                      = capsuleShape(0.09, 0.32);
            link.relativeTransform.position = Vec3{0.0, -0.82, 0.0};
            link.density                    = 900.0;
        }
        link.jointToParent = engine::physics_articulated::ArticulatedJointKind::BallSocket;
        // Ball anchor at the top end of each capsule.
        link.jointAnchorLocalParent = Vec3{0.0, index == 0 ? -0.16 : -0.42, 0.0};
        link.jointAnchorLocalChild  = Vec3{0.0, 0.41, 0.0};
        link.limitsEnabled          = true;
        link.coneHalfAngleRadians   = 0.55;
        link.twistHalfAngleRadians  = 0.6;
    }
    engine::physics_articulated::FigureDescription figure;
    figure.links                                          = links.data();
    figure.linkCount                                      = linkCount;
    figure.rootDynamic                                    = false;
    engine::physics_articulated::ArticulatedError   error = engine::physics_articulated::ArticulatedError::None;
    const engine::physics_articulated::FigureHandle figureHandle
        = dependencies.articulation->createFigure(dependencies.articulation->self, handles.world, &figure, &error);
    if (figureHandle != engine::physics_articulated::invalidFigure)
    {
        handles.figures.push_back(figureHandle);
        handles.chainLinkCount = linkCount - 1;
    }
}

void addPendulumFigure(const SceneDependencies &dependencies, SceneHandles &handles)
{
    if (dependencies.articulation == nullptr)
    {
        return;
    }
    constexpr std::uint32_t                                   linkCount = 3;
    std::vector<engine::physics_articulated::LinkDescription> links(linkCount);
    for (std::uint32_t index = 0; index < linkCount; ++index)
    {
        engine::physics_articulated::LinkDescription &link = links[index];
        link.parentIndex                                   = index == 0 ? -1 : static_cast<std::int32_t>(index - 1);
        if (index == 0)
        {
            link.shape                      = boxShape(0.25, 0.25, 0.25);
            link.relativeTransform.position = Vec3{-5.4, 6.0, -3.0};
        }
        else
        {
            link.shape                      = boxShape(0.16, 0.55, 0.16);
            link.relativeTransform.position = Vec3{0.0, -1.35, 0.0};
            link.density                    = 640.0;
        }
        link.jointToParent          = engine::physics_articulated::ArticulatedJointKind::Hinge;
        link.hingeAxisLocalParent   = Vec3{1.0, 0.0, 0.0}; // swing in the YZ plane
        link.jointAnchorLocalParent = Vec3{0.0, index == 0 ? -0.25 : -0.55, 0.0};
        link.jointAnchorLocalChild  = Vec3{0.0, 0.55, 0.0};
        link.limitsEnabled          = true;
        link.limitLowerRadians      = -1.2;
        link.limitUpperRadians      = 1.2;
    }
    engine::physics_articulated::FigureDescription figure;
    figure.links                                          = links.data();
    figure.linkCount                                      = linkCount;
    figure.rootDynamic                                    = false;
    engine::physics_articulated::ArticulatedError   error = engine::physics_articulated::ArticulatedError::None;
    const engine::physics_articulated::FigureHandle figureHandle
        = dependencies.articulation->createFigure(dependencies.articulation->self, handles.world, &figure, &error);
    if (figureHandle != engine::physics_articulated::invalidFigure)
    {
        handles.figures.push_back(figureHandle);
        handles.pendulumLinkCount = linkCount - 1;
    }
}

void addClothSheet(const SceneDependencies &dependencies, SceneHandles &handles, bool draped)
{
    if (dependencies.cloth == nullptr)
    {
        return;
    }
    engine::physics_cloth::ClothDescription description;
    description.columns            = draped ? 18 : 16;
    description.rows               = draped ? 12 : 16;
    description.spacing            = 0.22;
    description.particleMass       = 0.16;
    description.stretchCompliance  = 0.0;
    description.shearCompliance    = 0.00003;
    description.bendCompliance     = 0.0002;
    description.damping            = 0.05;
    description.friction           = 0.55;
    // The draped sheet spawns parallel to the ramp's slope (the ramp pitches
    // 0.30 rad about X, its top surface rising toward -z), 6 cm above the
    // surface: its pinned first row sits just off the slope's mid line and
    // the sheet lies down the ramp, hanging off the low edge.
    description.transform.rotation = draped ? pitchQuaternion(0.30) : description.transform.rotation;
    description.transform.position = draped ? Vec3{-3.4, 1.32, -1.5} : Vec3{0.0, 4.6, 1.4};
    // Both sheets pin their whole first row: the draped sheet holds onto the
    // slope this way, the free-hanging one stays a curtain.
    std::vector<std::uint32_t> pins;
    for (std::uint32_t column = 0; column < description.columns; ++column)
    {
        pins.push_back(column);
    }
    description.pinnedParticles                    = pins.data();
    description.pinnedCount                        = static_cast<std::uint32_t>(pins.size());
    engine::physics_cloth::ClothError        error = engine::physics_cloth::ClothError::None;
    const engine::physics_cloth::ClothHandle clothHandle
        = dependencies.cloth->createCloth(dependencies.cloth->self, handles.world, &description, &error);
    if (clothHandle != engine::physics_cloth::invalidCloth)
    {
        handles.cloths.push_back(clothHandle);
    }
}

void addPaddle(const SceneDependencies &dependencies, SceneHandles &handles)
{
    BodyDescription paddle;
    paddle.shape               = boxShape(0.5, 0.9, 0.5);
    paddle.transform.position  = Vec3{-6.0, 0.9, -5.4};
    paddle.kind                = engine::physics_rigid::BodyKind::Kinematic;
    paddle.material.friction   = 0.5;
    const float paddleColor[3] = {0.55f, 0.62f, 0.70f};
    handles.paddle             = createBody(dependencies, handles.world, paddle);
    pushBody(handles, handles.paddle, PrimitiveKind::Box, paddleColor, false);
}

} // namespace

SceneConfig readSceneConfig(engine::ParticipantContext &context)
{
    SceneConfig config;
    if (const auto gravity = context.configurationFloatingArrayInSection("physics_sandbox", "gravity");
        gravity.size() == 3)
    {
        config.gravity[0] = gravity[0];
        config.gravity[1] = gravity[1];
        config.gravity[2] = gravity[2];
    }
    const auto readNumber = [&](const char *key, double &target, double minimum, double maximum)
    {
        if (const auto value = context.configurationFloatingInSection("physics_sandbox", key);
            value && *value >= minimum && *value <= maximum)
        {
            target = *value;
        }
    };
    readNumber("ground_extent", config.groundExtent, 8.0, 500.0);
    readNumber("spawn_ahead", config.spawnAhead, 1.0, 20.0);
    readNumber("grab_stiffness", config.grabStiffness, 1.0, 100000.0);
    readNumber("grab_damping", config.grabDamping, 0.0, 100000.0);
    readNumber("grab_maximum_force", config.grabMaximumForce, 1.0, 1000000.0);
    readNumber("blast_radius", config.blastRadius, 0.5, 100.0);
    readNumber("blast_impulse", config.blastImpulse, 1.0, 100000.0);
    readNumber("wind_strength", config.windStrength, 0.0, 100.0);
    readNumber("paddle_span", config.paddleSpan, 0.0, 50.0);
    readNumber("paddle_speed", config.paddleSpeed, 0.0, 20.0);
    readNumber("pendulum_swing", config.pendulumSwing, 0.0, 3.0);
    readNumber("pendulum_frequency", config.pendulumFrequency, 0.01, 10.0);
    return config;
}

// Registers a figure's link bodies in the render list with the palette entry
// the demo uses for that figure (links are created by the articulated module
// and must be adopted for rendering).
void adoptFigureLinks(const SceneDependencies &dependencies, SceneHandles &handles,
    engine::physics_articulated::FigureHandle figure, PrimitiveKind kind, const float color[3])
{
    // The first call is a size query: it reports false with the required
    // count when the caller passes no buffer.
    std::uint32_t required = 0;
    dependencies.articulation->copyLinkBodyHandles(dependencies.articulation->self, figure, nullptr, 0, &required);
    if (required == 0)
    {
        return;
    }
    std::vector<engine::physics_rigid::BodyHandle> linkBodies(required);
    if (!dependencies.articulation->copyLinkBodyHandles(
            dependencies.articulation->self, figure, linkBodies.data(), required, nullptr))
    {
        return;
    }
    for (const engine::physics_rigid::BodyHandle linkBody : linkBodies)
    {
        pushBody(handles, linkBody, kind, color, false);
    }
}

bool buildScene(const SceneDependencies &dependencies, const SceneConfig &config, const std::string &sceneName,
    SceneHandles &outHandles, std::string &outReport)
{
    if (dependencies.rigid == nullptr)
    {
        outReport = "the rigid capability is unavailable\n";
        return false;
    }
    engine::physics_rigid::WorldDescription worldDescription;
    worldDescription.gravity                  = Vec3{config.gravity[0], config.gravity[1], config.gravity[2]};
    engine::physics_rigid::PhysicsError error = engine::physics_rigid::PhysicsError::None;
    outHandles.world = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, &error);
    if (outHandles.world == engine::physics_rigid::invalidWorld)
    {
        std::ostringstream report;
        report << "the rigid module rejected the world (error " << static_cast<std::uint32_t>(error) << ")\n";
        outReport = report.str();
        return false;
    }

    addStaticScenery(dependencies, config, outHandles);
    if (sceneName == "stacks")
    {
        addBoxStack(dependencies, outHandles);
        addSphereCluster(dependencies, outHandles);
        addCapsuleRow(dependencies, outHandles);
        addPaddle(dependencies, outHandles);
    }
    else if (sceneName == "cloth")
    {
        addClothSheet(dependencies, outHandles, true);
    }
    else if (
        sceneName == "fields" || sceneName == "rope" || sceneName == "water" || sceneName == "fluid"
        || sceneName == "particles" || sceneName == "foliage" || sceneName == "character")
    {
        // Extended scenes build their own scenery through the extended
        // modules; the shared static scenery stays available to them.
        std::string extendedReport;
        if (!buildExtendedScene(dependencies, config, sceneName, outHandles, extendedReport))
        {
            outReport += extendedReport;
            return false;
        }
        outReport += extendedReport;
    }
    else // showcase
    {
        addBoxStack(dependencies, outHandles);
        addSphereCluster(dependencies, outHandles);
        addCapsuleRow(dependencies, outHandles);
        addPaddle(dependencies, outHandles);
        addChainFigure(dependencies, outHandles);
        addPendulumFigure(dependencies, outHandles);
        addClothSheet(dependencies, outHandles, true);
        if (!outHandles.figures.empty())
        {
            const float chainColor[3] = {0.78f, 0.81f, 0.86f};
            adoptFigureLinks(dependencies, outHandles, outHandles.figures[0], PrimitiveKind::Capsule, chainColor);
        }
        if (outHandles.figures.size() > 1)
        {
            const float pendulumColor[3] = {0.88f, 0.52f, 0.16f};
            adoptFigureLinks(dependencies, outHandles, outHandles.figures[1], PrimitiveKind::Box, pendulumColor);
        }
    }
    return true;
}

void destroyScene(const SceneDependencies &dependencies, SceneHandles &handles)
{
    if (dependencies.rigid == nullptr || handles.world == engine::physics_rigid::invalidWorld)
    {
        handles = SceneHandles{};
        return;
    }
    // Extended-module objects first: they reference the world's bodies.
    destroyExtendedScene(dependencies, handles);
    handles.bodies.clear();
    handles.figures.clear();
    handles.cloths.clear();
    handles.paddle            = engine::physics_rigid::invalidBody;
    handles.chainLinkCount    = 0;
    handles.pendulumLinkCount = 0;
    dependencies.rigid->destroyWorld(dependencies.rigid->self, handles.world);
    handles.world = engine::physics_rigid::invalidWorld;
}

engine::physics_rigid::BodyHandle spawnPrimitive(const SceneDependencies &dependencies, const SceneConfig &config,
    SceneHandles &handles, PrimitiveKind kind, const double origin[3], const double forward[3])
{
    if (dependencies.rigid == nullptr || handles.world == engine::physics_rigid::invalidWorld)
    {
        return engine::physics_rigid::invalidBody;
    }
    BodyDescription description;
    switch (kind)
    {
    case PrimitiveKind::Box:
        description.shape = boxShape(0.32, 0.32, 0.32);
        break;
    case PrimitiveKind::Sphere:
        description.shape = sphereShape(0.3);
        break;
    case PrimitiveKind::Capsule:
    default:
        description.shape = capsuleShape(0.22, 0.34);
        break;
    }
    const Vec3 forwardVector = math::normalized(Vec3{forward[0], forward[1], forward[2]});
    const Vec3 spawnPosition
        = Vec3{origin[0], origin[1], origin[2]} + forwardVector * config.spawnAhead + Vec3{0.0, 0.6, 0.0};
    description.transform.position    = spawnPosition;
    description.kind                  = engine::physics_rigid::BodyKind::Dynamic;
    description.density               = 340.0;
    description.material.friction     = 0.5;
    description.material.restitution  = 0.15;
    const float      spawnedColors[3] = {0.90f, 0.58f, 0.16f};
    const BodyHandle body             = createBody(dependencies, handles.world, description);
    if (body != engine::physics_rigid::invalidBody)
    {
        pushBody(handles, body, kind, spawnedColors, true);
    }
    return body;
}

void applyBlast(
    const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles, const double center[3])
{
    if (dependencies.rigid == nullptr || handles.world == engine::physics_rigid::invalidWorld)
    {
        return;
    }
    const Vec3 centerVector{center[0], center[1], center[2]};
    for (const SceneBody &sceneBody : handles.bodies)
    {
        engine::physics_rigid::BodyState state{};
        if (!dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, sceneBody.body, &state))
        {
            continue;
        }
        const Vec3   delta    = state.transform.position - centerVector;
        const double distance = math::length(delta);
        if (distance > config.blastRadius || distance <= 1.0e-6)
        {
            continue;
        }
        const double falloff   = 1.0 - distance / config.blastRadius;
        const Vec3   direction = delta * (1.0 / distance);
        Vec3         impulse   = direction * (config.blastImpulse * falloff);
        impulse.y += config.blastImpulse * 0.35 * falloff;
        dependencies.rigid->applyImpulseAtPoint(
            dependencies.rigid->self, handles.world, sceneBody.body, &impulse.x, &state.transform.position.x);
    }
}

// Capture-timeline latches. The demo runs one timeline per process, so
// function-local statics keep the event flags out of the handle records.
bool boxSpawned_     = false;
bool sphereSpawned_  = false;
bool capsuleSpawned_ = false;
bool blastDone_      = false;
bool chainKicked_    = false;
bool windApplied_    = false;

void updateScriptedTimeline(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    double simulatedSeconds, double fixedDeltaSeconds)
{
    (void)fixedDeltaSeconds;
    // Deterministic capture timeline: events key off simulated time so any
    // fixed rate produces the same sequence.
    static constexpr double kSpawnBoxAt     = 0.6;
    static constexpr double kSpawnSphereAt  = 1.3;
    static constexpr double kSpawnCapsuleAt = 2.1;
    static constexpr double kBlastAt        = 2.9;
    static constexpr double kChainKickAt    = 1.9;
    static constexpr double kWindAt         = 3.4;

    if (handles.world == engine::physics_rigid::invalidWorld)
    {
        return;
    }
    if (!boxSpawned_ && simulatedSeconds >= kSpawnBoxAt)
    {
        boxSpawned_             = true;
        const double origin[3]  = {-3.2, 5.4, 1.2};
        const double forward[3] = {0.0, -0.15, 0.4};
        spawnPrimitive(dependencies, config, handles, PrimitiveKind::Box, origin, forward);
    }
    if (!sphereSpawned_ && simulatedSeconds >= kSpawnSphereAt)
    {
        sphereSpawned_          = true;
        const double origin[3]  = {2.6, 5.6, 1.0};
        const double forward[3] = {0.0, -0.2, 0.35};
        spawnPrimitive(dependencies, config, handles, PrimitiveKind::Sphere, origin, forward);
    }
    if (!capsuleSpawned_ && simulatedSeconds >= kSpawnCapsuleAt)
    {
        capsuleSpawned_         = true;
        const double origin[3]  = {-0.4, 5.8, 0.6};
        const double forward[3] = {0.1, -0.25, 0.4};
        spawnPrimitive(dependencies, config, handles, PrimitiveKind::Capsule, origin, forward);
    }
    if (!blastDone_ && simulatedSeconds >= kBlastAt)
    {
        blastDone_             = true;
        const double center[3] = {-3.2, 1.6, 2.4};
        applyBlast(dependencies, config, handles, center);
    }
    // The chain kick belongs to the showcase scene's hanging chain; scenes
    // without it (chainLinkCount == 0) keep their figures untouched.
    if (!chainKicked_ && simulatedSeconds >= kChainKickAt && dependencies.articulation != nullptr
        && handles.chainLinkCount > 0 && !handles.figures.empty())
    {
        chainKicked_                                                = true;
        const engine::physics_articulated::FigureHandle chainFigure = handles.figures[0];
        engine::physics_rigid::BodyHandle               bottomLink  = engine::physics_rigid::invalidBody;
        const std::uint32_t                             bottomIndex = handles.chainLinkCount;
        if (dependencies.articulation->linkBodyHandle(
                dependencies.articulation->self, chainFigure, bottomIndex, &bottomLink)
            && bottomLink != engine::physics_rigid::invalidBody)
        {
            const double impulse[3] = {14.0, 0.0, 9.0};
            dependencies.rigid->applyImpulseAtPoint(
                dependencies.rigid->self, handles.world, bottomLink, impulse, nullptr);
        }
    }
    if (!windApplied_ && simulatedSeconds >= kWindAt && dependencies.cloth != nullptr)
    {
        windApplied_ = true;
        for (const engine::physics_cloth::ClothHandle clothHandle : handles.cloths)
        {
            const double wind[3] = {config.windStrength, 0.6, config.windStrength * 0.4};
            dependencies.cloth->setWindAcceleration(dependencies.cloth->self, clothHandle, wind);
        }
    }

    // The pendulum figure's drives trace a deterministic sine wave so
    // captures show active articulation control; the wave belongs to the
    // pendulum figure only (scenes without one keep their figures free).
    if (dependencies.articulation != nullptr && handles.pendulumLinkCount > 0 && handles.figures.size() > 1)
    {
        const engine::physics_articulated::FigureHandle pendulumFigure = handles.figures[1];
        const double                                    target
            = config.pendulumSwing * std::sin(6.283185307179586 * config.pendulumFrequency * simulatedSeconds);
        for (std::uint32_t linkIndex = 1; linkIndex <= handles.pendulumLinkCount; ++linkIndex)
        {
            engine::physics_articulated::JointDrive drive{};
            drive.enabled            = true;
            drive.stiffness          = 26.0;
            drive.damping            = 3.2;
            drive.targetAngleRadians = target * (linkIndex % 2 == 0 ? 1.0 : 0.6);
            drive.maximumVelocity    = 6.0;
            drive.maximumTorque      = 90.0;
            dependencies.articulation->setJointDrive(
                dependencies.articulation->self, pendulumFigure, linkIndex, &drive);
        }
    }

    // The kinematic paddle sweeps back and forth along X; its velocity is
    // set (not teleported) so contacts push objects with physical momentum.
    if (handles.paddle != engine::physics_rigid::invalidBody)
    {
        const double phase     = simulatedSeconds * config.paddleSpeed * 6.283185307179586 * 0.15;
        const double positionX = std::sin(phase) * config.paddleSpan * 0.5;
        const double velocityX
            = std::cos(phase) * config.paddleSpan * 0.5 * config.paddleSpeed * 6.283185307179586 * 0.15;
        engine::physics_rigid::BodyState paddleState{};
        if (dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, handles.paddle, &paddleState))
        {
            paddleState.transform.position = Vec3{positionX, 0.9, -5.4};
            paddleState.linearVelocity     = Vec3{velocityX, 0.0, 0.0};
            dependencies.rigid->setBodyTransform(
                dependencies.rigid->self, handles.world, handles.paddle, &paddleState.transform, false);
            const double paddleVelocity[3] = {velocityX, 0.0, 0.0};
            dependencies.rigid->setBodyLinearVelocity(
                dependencies.rigid->self, handles.world, handles.paddle, paddleVelocity);
        }
    }
}

} // namespace demo::physics_sandbox
