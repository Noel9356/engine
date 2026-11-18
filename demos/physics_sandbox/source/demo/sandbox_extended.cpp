// sandbox_extended.cpp - the extended physics scenes. Each scenario wires
// one or two extended modules to the shared rigid world: fields drive wind
// drag and blasts, ropes hang and tow, the ocean carries buoyant bodies and
// takes splash ripples, the SPH fluid pours through a container, particle
// emitters run rain/snow/dust/fire, the foliage field bends in the wind and
// takes trampling, and the character rig demonstrates the humanoid ragdoll,
// pose-tracking drives, the jiggle chain, and procedural foot placement.
#include "sandbox_extended.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

// Flat static scenery shared by the extended scenes (no ramp, no wall: the
// camera framing stays centered on the scenario).
void addFlatScenery(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles)
{
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
}

// ---- individual scenes -----------------------------------------------------

void addFieldsScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.fields == nullptr)
    {
        report << "the physics_fields capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // One wind field blowing across the scene; its drag pass pushes every
    // awake dynamic body each fixed update.
    engine::physics_fields::WindFieldDescription wind;
    wind.direction[0] = 1.0;
    wind.direction[1] = 0.0;
    wind.direction[2] = 0.25;
    wind.baseSpeed    = 7.5;
    engine::physics_fields::FieldsError fieldsError = engine::physics_fields::FieldsError::None;
    handles.extended.windField
        = dependencies.fields->createWindField(dependencies.fields->self, handles.world, &wind, &fieldsError);
    if (handles.extended.windField == engine::physics_fields::invalidWind)
    {
        report << "the wind field was rejected (error " << static_cast<std::uint32_t>(fieldsError) << ")\n";
        return;
    }

    // A radial attractor volume to one side (magnetic-style pull).
    engine::physics_fields::ForceVolumeDescription volume;
    volume.kind            = engine::physics_fields::ForceVolumeDescription::Kind::Radial;
    volume.center[0]       = -4.0;
    volume.center[1]       = 1.2;
    volume.center[2]       = 0.0;
    volume.radius          = 4.5;
    volume.radialStrength  = -9.0;
    handles.extended.forceVolume
        = dependencies.fields->createForceVolume(dependencies.fields->self, handles.world, &volume, &fieldsError);

    // Light props the wind can push: boxes, spheres, and a rolling barrel.
    const float boxColor[3]    = {0.85f, 0.52f, 0.18f};
    const float sphereColor[3] = {0.24f, 0.52f, 0.82f};
    const float barrelColor[3] = {0.30f, 0.74f, 0.40f};
    for (std::uint32_t index = 0; index < 4; ++index)
    {
        BodyDescription box;
        box.shape              = boxShape(0.3, 0.3, 0.3);
        box.transform.position = Vec3{-2.0 + index * 1.6, 0.4 + index * 0.75, 2.0};
        box.kind               = engine::physics_rigid::BodyKind::Dynamic;
        box.density            = 60.0;
        box.material.friction  = 0.4;
        pushBody(handles, createBody(dependencies, handles.world, box), PrimitiveKind::Box, boxColor, false);
    }
    BodyDescription sphere;
    sphere.shape              = sphereShape(0.35);
    sphere.transform.position = Vec3{0.5, 1.6, -1.2};
    sphere.kind               = engine::physics_rigid::BodyKind::Dynamic;
    sphere.density            = 80.0;
    pushBody(handles, createBody(dependencies, handles.world, sphere), PrimitiveKind::Sphere, sphereColor, false);
    BodyDescription barrel;
    barrel.shape              = engine::physics_rigid::ShapeDescription{};
    barrel.shape.kind         = ShapeKind::Capsule;
    barrel.shape.capsuleRadius = 0.3;
    barrel.shape.capsuleHalfLength = 0.45;
    barrel.transform.rotation      = engine::physics_rigid::Quaternion{
        std::sin(kPi * 0.25), 0.0, 0.0, std::cos(kPi * 0.25)};
    barrel.transform.position      = Vec3{2.4, 0.4, 0.6};
    barrel.kind                    = engine::physics_rigid::BodyKind::Dynamic;
    barrel.density                 = 70.0;
    barrel.material.friction       = 0.5;
    barrel.material.rollingResistance = 0.04;
    pushBody(handles, createBody(dependencies, handles.world, barrel), PrimitiveKind::Capsule, barrelColor, false);

    // The fur ball rides the same wind so the shell-fur sway is visible.
    BodyDescription furBall;
    furBall.shape              = sphereShape(0.55);
    furBall.transform.position = Vec3{-1.2, 2.6, -2.4};
    furBall.kind               = engine::physics_rigid::BodyKind::Dynamic;
    furBall.density            = 120.0;
    furBall.material.friction  = 0.6;
    const float furColor[3]    = {0.62f, 0.45f, 0.30f};
    handles.extended.furBallBody = createBody(dependencies, handles.world, furBall);
    if (handles.extended.furBallBody != engine::physics_rigid::invalidBody)
    {
        handles.extended.furBall = true;
        pushBody(handles, handles.extended.furBallBody, PrimitiveKind::Sphere, furColor, false);
    }
}

void addRopeScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.rope == nullptr)
    {
        report << "the physics_rope capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // Crane cable: pinned at the anchor, attached to a hanging crate.
    BodyDescription crate;
    crate.shape              = boxShape(0.35, 0.35, 0.35);
    crate.transform.position = Vec3{-3.0, 1.2, 0.0};
    crate.kind               = engine::physics_rigid::BodyKind::Dynamic;
    crate.density            = 300.0;
    crate.material.friction  = 0.6;
    const BodyHandle crateBody = createBody(dependencies, handles.world, crate);
    const float crateColor[3]  = {0.72f, 0.27f, 0.20f};
    pushBody(handles, crateBody, PrimitiveKind::Box, crateColor, false);

    engine::physics_rope::RopeDescription cable;
    cable.segments       = 18;
    cable.restLength     = 3.2;
    cable.nodeRadius     = 0.07;
    cable.nodeMass       = 0.5;
    cable.start[0]       = -3.0;
    cable.start[1]       = 4.6;
    cable.start[2]       = 0.0;
    cable.end[0]         = -3.0;
    cable.end[1]         = 1.55;
    cable.end[2]         = 0.0;
    cable.pinStart       = true;
    cable.attachEndToBody = true;
    cable.endBody        = crateBody;
    engine::physics_rope::RopeError ropeError = engine::physics_rope::RopeError::None;
    engine::physics_rope::RopeHandle ropeHandle
        = dependencies.rope->createRope(dependencies.rope->self, handles.world, &cable, &ropeError);
    if (ropeHandle != engine::physics_rope::invalidRope)
    {
        handles.extended.ropes.push_back(ropeHandle);
    }
    else
    {
        report << "the crane cable was rejected (error " << static_cast<std::uint32_t>(ropeError) << ")\n";
    }

    // A free rope draped over a static ledge: pinned at one end only.
    BodyDescription ledge;
    ledge.shape               = boxShape(1.2, 0.15, 1.2);
    ledge.transform.position  = Vec3{2.6, 1.5, -1.6};
    ledge.kind                = engine::physics_rigid::BodyKind::Static;
    ledge.material.friction   = 0.9;
    const float ledgeColor[3] = {0.42f, 0.35f, 0.28f};
    pushBody(handles, createBody(dependencies, handles.world, ledge), PrimitiveKind::Box, ledgeColor, false);

    engine::physics_rope::RopeDescription drape;
    drape.segments   = 16;
    drape.restLength = 3.0;
    drape.nodeRadius = 0.07;
    drape.nodeMass   = 0.4;
    drape.start[0]   = 2.6;
    drape.start[1]   = 1.7;
    drape.start[2]   = -1.6;
    drape.end[0]     = 2.6;
    drape.end[1]     = -1.2;
    drape.end[2]     = -1.6;
    drape.pinStart   = true;
    ropeHandle = dependencies.rope->createRope(dependencies.rope->self, handles.world, &drape, &ropeError);
    if (ropeHandle != engine::physics_rope::invalidRope)
    {
        handles.extended.ropes.push_back(ropeHandle);
    }

    // A taut tow line between two crates: pinned to both bodies.
    BodyDescription towA;
    towA.shape              = boxShape(0.3, 0.3, 0.3);
    towA.transform.position = Vec3{4.4, 0.4, 1.6};
    towA.kind               = engine::physics_rigid::BodyKind::Dynamic;
    towA.density            = 260.0;
    towA.material.friction  = 0.7;
    const BodyHandle towABody = createBody(dependencies, handles.world, towA);
    BodyDescription towB;
    towB.shape              = boxShape(0.3, 0.3, 0.3);
    towB.transform.position = Vec3{6.2, 0.4, 1.6};
    towB.kind               = engine::physics_rigid::BodyKind::Dynamic;
    towB.density            = 260.0;
    towB.material.friction  = 0.7;
    const BodyHandle towBBody = createBody(dependencies, handles.world, towB);
    const float towColor[3]   = {0.87f, 0.68f, 0.24f};
    pushBody(handles, towABody, PrimitiveKind::Box, towColor, false);
    pushBody(handles, towBBody, PrimitiveKind::Box, towColor, false);

    engine::physics_rope::RopeDescription tow;
    tow.segments          = 12;
    tow.restLength        = 1.6;
    tow.nodeRadius        = 0.06;
    tow.nodeMass          = 0.35;
    tow.start[0]          = 4.4;
    tow.start[1]          = 0.4;
    tow.start[2]          = 1.6;
    tow.end[0]            = 6.2;
    tow.end[1]            = 0.4;
    tow.end[2]            = 1.6;
    tow.attachStartToBody = true;
    tow.startBody         = towABody;
    tow.attachEndToBody   = true;
    tow.endBody           = towBBody;
    ropeHandle = dependencies.rope->createRope(dependencies.rope->self, handles.world, &tow, &ropeError);
    if (ropeHandle != engine::physics_rope::invalidRope)
    {
        handles.extended.ropes.push_back(ropeHandle);
    }
}

void addWaterScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.water == nullptr)
    {
        report << "the physics_water capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // A small ocean: three Gerstner components with different wavelengths.
    engine::physics_water::OceanDescription oceanDescription;
    oceanDescription.waterLevel               = 0.55;
    oceanDescription.waveCount                = 3;
    oceanDescription.waves[0].directionX      = 1.0;
    oceanDescription.waves[0].directionZ      = 0.15;
    oceanDescription.waves[0].wavelength      = 14.0;
    oceanDescription.waves[0].amplitude       = 0.16;
    oceanDescription.waves[0].steepness       = 0.25;
    oceanDescription.waves[1].directionX      = 0.7;
    oceanDescription.waves[1].directionZ      = -0.7;
    oceanDescription.waves[1].wavelength      = 7.0;
    oceanDescription.waves[1].amplitude       = 0.09;
    oceanDescription.waves[1].steepness       = 0.2;
    oceanDescription.waves[2].directionX      = 0.2;
    oceanDescription.waves[2].directionZ      = 1.0;
    oceanDescription.waves[2].wavelength      = 3.4;
    oceanDescription.waves[2].amplitude       = 0.05;
    oceanDescription.waves[2].steepness       = 0.15;
    engine::physics_water::WaterError waterError = engine::physics_water::WaterError::None;
    handles.extended.ocean
        = dependencies.water->createOcean(dependencies.water->self, handles.world, &oceanDescription, &waterError);
    if (handles.extended.ocean == engine::physics_water::invalidOcean)
    {
        report << "the ocean was rejected (error " << static_cast<std::uint32_t>(waterError) << ")\n";
        return;
    }

    // Floating props with different masses and a fully submerged block.
    const float woodColor[3] = {0.72f, 0.51f, 0.28f};
    const float metalColor[3] = {0.58f, 0.62f, 0.68f};
    struct Floater
    {
        ShapeDescription shape;
        Vec3             position;
        double           density;
    };
    ShapeDescription logShape;
    logShape.kind                = ShapeKind::Capsule;
    logShape.capsuleRadius       = 0.22;
    logShape.capsuleHalfLength   = 0.8;
    const Floater floaters[] = {
        {logShape, Vec3{-2.6, 1.75, 1.2}, 420.0},
        {sphereShape(0.4), Vec3{0.6, 1.2, -0.8}, 480.0},
        {boxShape(0.45, 0.3, 0.45), Vec3{2.8, 1.2, 1.4}, 900.0},
        {boxShape(0.35, 0.35, 0.35), Vec3{-0.9, 1.6, 2.2}, 150.0},
    };
    for (const Floater &floater : floaters)
    {
        BodyDescription body;
        body.shape              = floater.shape;
        body.transform.position = floater.position;
        body.kind               = engine::physics_rigid::BodyKind::Dynamic;
        body.density            = floater.density;
        body.material.friction  = 0.45;
        body.material.rollingResistance = 0.02;
        const BodyHandle handle = createBody(dependencies, handles.world, body);
        if (handle == engine::physics_rigid::invalidBody)
        {
            continue;
        }
        pushBody(handles,
            handle,
            floater.shape.kind == ShapeKind::Sphere ? PrimitiveKind::Sphere
            : floater.shape.kind == ShapeKind::Capsule  ? PrimitiveKind::Capsule
                                                        : PrimitiveKind::Box,
            floater.density > 800.0 ? metalColor : woodColor,
            false);
        handles.extended.buoyantBodies.push_back(handle);
        dependencies.water->registerBody(dependencies.water->self, handles.extended.ocean, handle, nullptr);
    }
}

void addFluidScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.fluid == nullptr)
    {
        report << "the physics_fluid capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // A tank built from static boxes: floor walls and side walls the SPH
    // particles pile into (the fluid's own bounds box also confines them).
    const float wallColor[3] = {0.52f, 0.48f, 0.43f};
    struct Wall
    {
        Vec3 position;
        Vec3 halfExtents;
    };
    const Wall walls[] = {
        {Vec3{0.0, 1.0, -2.0}, Vec3{2.2, 1.0, 0.1}},
        {Vec3{0.0, 1.0, 2.0}, Vec3{2.2, 1.0, 0.1}},
        {Vec3{-2.2, 1.0, 0.0}, Vec3{0.1, 1.0, 2.1}},
        {Vec3{2.2, 1.0, 0.0}, Vec3{0.1, 1.0, 2.1}},
    };
    for (const Wall &wall : walls)
    {
        BodyDescription body;
        body.shape              = boxShape(wall.halfExtents.x, wall.halfExtents.y, wall.halfExtents.z);
        body.transform.position = wall.position;
        body.kind               = engine::physics_rigid::BodyKind::Static;
        body.material.friction  = 0.6;
        pushBody(handles, createBody(dependencies, handles.world, body), PrimitiveKind::Box, wallColor, false);
    }

    engine::physics_fluid::FluidDescription fluidDescription;
    fluidDescription.interactionRadius = 0.30;
    fluidDescription.particleRadius    = 0.10;
    fluidDescription.particleMass      = 0.18;
    fluidDescription.restDensity       = 620.0;
    fluidDescription.gasStiffness      = 7.0;
    fluidDescription.viscosity         = 2.6;
    fluidDescription.useBounds         = true;
    fluidDescription.boundsMinimum[0]  = -2.1;
    fluidDescription.boundsMinimum[1]  = 0.12;
    fluidDescription.boundsMinimum[2]  = -1.9;
    fluidDescription.boundsMaximum[0]  = 2.1;
    fluidDescription.boundsMaximum[1]  = 2.6;
    fluidDescription.boundsMaximum[2]  = 1.9;
    engine::physics_fluid::FluidError fluidError = engine::physics_fluid::FluidError::None;
    handles.extended.fluid
        = dependencies.fluid->createFluid(dependencies.fluid->self, handles.world, &fluidDescription, &fluidError);
    if (handles.extended.fluid == engine::physics_fluid::invalidFluid)
    {
        report << "the fluid was rejected (error " << static_cast<std::uint32_t>(fluidError) << ")\n";
        return;
    }

    // The dam-break block: a dense cube of particles held on one side.
    constexpr std::uint32_t kColumns = 8;
    constexpr std::uint32_t kRows    = 7;
    constexpr std::uint32_t kDepth   = 6;
    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(kColumns) * kRows * kDepth * 3);
    const double spacing = fluidDescription.interactionRadius * 0.55;
    for (std::uint32_t depth = 0; depth < kDepth; ++depth)
    {
        for (std::uint32_t row = 0; row < kRows; ++row)
        {
            for (std::uint32_t column = 0; column < kColumns; ++column)
            {
                positions.push_back(-1.7 + column * spacing);
                positions.push_back(0.18 + row * spacing);
                positions.push_back(-0.85 + depth * spacing);
            }
        }
    }
    dependencies.fluid->addParticles(dependencies.fluid->self,
        handles.extended.fluid,
        positions.data(),
        nullptr,
        static_cast<std::uint32_t>(positions.size() / 3),
        &fluidError);
}

void addParticlesScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.particles == nullptr)
    {
        report << "the physics_particles capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // Rain over the whole scene, snow drifting on one side, a dust volume,
    // and a fire pit in the middle.
    engine::physics_particles::EmitterDescription rain;
    rain.kind                  = engine::physics_particles::EmitterKind::Rain;
    rain.origin[1]             = 10.0;
    rain.halfExtents[0]        = 9.0;
    rain.halfExtents[2]        = 7.0;
    rain.emitRate              = 320.0;
    rain.maximumParticles      = 1400;
    rain.minimumLifetime       = 1.1;
    rain.maximumLifetime       = 1.6;
    rain.initialVelocity[1]    = -11.0;
    rain.gravityScale          = 1.0;
    rain.airDrag               = 0.1;
    rain.swayAmplitude         = 0.15;
    rain.particleRadius        = 0.04;
    rain.minimumSize           = 0.03;
    rain.maximumSize           = 0.05;
    rain.collideWithWorld      = true;
    rain.useGroundPlane        = true;
    rain.groundHeight          = 0.02;
    engine::physics_particles::ParticlesError particlesError = engine::physics_particles::ParticlesError::None;
    handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Rain)]
        = dependencies.particles->createEmitter(dependencies.particles->self, handles.world, &rain, &particlesError);

    engine::physics_particles::EmitterDescription snow;
    snow.kind                  = engine::physics_particles::EmitterKind::Snow;
    snow.origin[0]             = -4.5;
    snow.origin[1]             = 8.0;
    snow.halfExtents[0]        = 3.5;
    snow.halfExtents[2]        = 5.0;
    snow.emitRate              = 90.0;
    snow.maximumParticles      = 900;
    snow.minimumLifetime       = 5.0;
    snow.maximumLifetime       = 8.0;
    snow.initialVelocity[1]    = -1.1;
    snow.gravityScale          = 0.35;
    snow.airDrag               = 2.4;
    snow.swayAmplitude         = 0.9;
    snow.swayFrequency         = 0.8;
    snow.particleRadius        = 0.05;
    snow.minimumSize           = 0.05;
    snow.maximumSize           = 0.09;
    snow.collideWithWorld      = true;
    handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Snow)]
        = dependencies.particles->createEmitter(dependencies.particles->self, handles.world, &snow, &particlesError);

    engine::physics_particles::EmitterDescription dust;
    dust.kind                  = engine::physics_particles::EmitterKind::Dust;
    dust.origin[0]             = 5.0;
    dust.origin[1]             = 1.2;
    dust.halfExtents[0]        = 2.2;
    dust.halfExtents[1]        = 1.2;
    dust.halfExtents[2]        = 2.2;
    dust.emitRate              = 40.0;
    dust.maximumParticles      = 300;
    dust.minimumLifetime       = 2.5;
    dust.maximumLifetime       = 4.5;
    dust.initialVelocity[1]    = 0.12;
    dust.gravityScale          = 0.0;
    dust.airDrag               = 1.6;
    dust.swayAmplitude         = 0.5;
    dust.particleRadius        = 0.03;
    dust.minimumSize           = 0.02;
    dust.maximumSize           = 0.05;
    handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Dust)]
        = dependencies.particles->createEmitter(dependencies.particles->self, handles.world, &dust, &particlesError);

    engine::physics_particles::EmitterDescription fire;
    fire.kind                  = engine::physics_particles::EmitterKind::Fire;
    fire.origin[0]             = 0.0;
    fire.origin[1]             = 0.25;
    fire.halfExtents[0]        = 0.55;
    fire.halfExtents[1]        = 0.1;
    fire.halfExtents[2]        = 0.55;
    fire.emitRate              = 130.0;
    fire.maximumParticles      = 500;
    fire.minimumLifetime       = 0.8;
    fire.maximumLifetime       = 1.5;
    fire.initialVelocity[1]    = 1.6;
    fire.gravityScale          = -0.55;
    fire.airDrag               = 0.8;
    fire.swayAmplitude         = 0.35;
    fire.particleRadius        = 0.06;
    fire.minimumSize           = 0.12;
    fire.maximumSize           = 0.22;
    fire.flicker               = 0.8;
    handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Fire)]
        = dependencies.particles->createEmitter(dependencies.particles->self, handles.world, &fire, &particlesError);

    // A stone ring around the fire pit.
    const float stoneColor[3] = {0.44f, 0.44f, 0.46f};
    for (std::uint32_t index = 0; index < 8; ++index)
    {
        const double angle = static_cast<double>(index) / 8.0 * 6.283185307179586;
        BodyDescription stone;
        stone.shape              = boxShape(0.16, 0.14, 0.16);
        stone.transform.position = Vec3{std::cos(angle) * 1.15, 0.14, std::sin(angle) * 1.15};
        stone.kind               = engine::physics_rigid::BodyKind::Static;
        pushBody(handles, createBody(dependencies, handles.world, stone), PrimitiveKind::Box, stoneColor, false);
    }
}

void addFoliageScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.foliage == nullptr)
    {
        report << "the physics_foliage capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // A grass field of bending blades; the wind handle comes from the
    // fields scene wiring when present, otherwise the ambient breeze blows.
    std::vector<engine::physics_foliage::FoliageInstance> instances;
    instances.reserve(900);
    constexpr std::uint32_t kRows = 30;
    constexpr std::uint32_t kColumns = 30;
    for (std::uint32_t row = 0; row < kRows; ++row)
    {
        for (std::uint32_t column = 0; column < kColumns; ++column)
        {
            const double x = -7.0 + static_cast<double>(column) * (14.0 / (kColumns - 1));
            const double z = -6.0 + static_cast<double>(row) * (12.0 / (kRows - 1));
            engine::physics_foliage::FoliageInstance instance;
            instance.position[0] = x;
            instance.position[1] = 0.0;
            instance.position[2] = z;
            // Height grows toward the back so the sway reads in captures.
            instance.height    = 0.35 + 0.45 * (static_cast<double>(row) / (kRows - 1));
            instance.stiffness = 0.25 + 0.35 * (static_cast<double>(column) / (kColumns - 1));
            instance.phaseSeed = static_cast<double>((row * 7 + column * 13) % 360) / 360.0 * 6.283185307179586;
            instance.swayScale = 1.0;
            instances.push_back(instance);
        }
    }

    engine::physics_foliage::FoliageFieldDescription fieldDescription;
    fieldDescription.instances      = instances.data();
    fieldDescription.instanceCount  = static_cast<std::uint32_t>(instances.size());
    fieldDescription.swayFrequency  = 1.0;
    fieldDescription.bendScale      = 0.05;
    fieldDescription.springStiffness = 24.0;
    fieldDescription.springDamping  = 6.0;
    engine::physics_foliage::FoliageError foliageError = engine::physics_foliage::FoliageError::None;
    // Bind the fields module's wind when it is available so the blades bend
    // with the gust field; otherwise the configured ambient breeze blows.
    std::uint32_t windHandle = 0;
    if (dependencies.fields != nullptr)
    {
        engine::physics_fields::WindFieldDescription wind;
        wind.direction[0] = 1.0;
        wind.direction[2] = 0.2;
        wind.baseSpeed    = 5.5;
        wind.gustStrength = 0.45;
        engine::physics_fields::FieldsError fieldsError = engine::physics_fields::FieldsError::None;
        windHandle = dependencies.fields->createWindField(
            dependencies.fields->self, handles.world, &wind, &fieldsError);
        if (windHandle != engine::physics_fields::invalidWind)
        {
            handles.extended.windField = windHandle;
        }
    }
    handles.extended.foliageField = dependencies.foliage->createField(
        dependencies.foliage->self, &fieldDescription, windHandle, &foliageError);
    if (handles.extended.foliageField == engine::physics_foliage::invalidField)
    {
        report << "the foliage field was rejected (error " << static_cast<std::uint32_t>(foliageError) << ")\n";
        return;
    }
    handles.extended.foliageInstances = std::move(instances);

    // A roller barrel the scripted timeline drives across the field so the
    // blades trample and spring back.
    BodyDescription roller;
    roller.shape                   = engine::physics_rigid::ShapeDescription{};
    roller.shape.kind              = ShapeKind::Capsule;
    roller.shape.capsuleRadius     = 0.35;
    roller.shape.capsuleHalfLength = 0.9;
    roller.transform.rotation      = engine::physics_rigid::Quaternion{0.0, 0.0, std::sin(kPi * 0.25), std::cos(kPi * 0.25)};
    roller.transform.position      = Vec3{-6.5, 0.35, 0.0};
    roller.kind                    = engine::physics_rigid::BodyKind::Kinematic;
    roller.material.friction       = 0.8;
    const float rollerColor[3]     = {0.55f, 0.62f, 0.70f};
    handles.paddle = createBody(dependencies, handles.world, roller);
    pushBody(handles, handles.paddle, PrimitiveKind::Capsule, rollerColor, false);
}

void addCharacterScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    std::ostringstream &report)
{
    if (dependencies.character == nullptr)
    {
        report << "the physics_character capability is unavailable\n";
        return;
    }
    addFlatScenery(dependencies, config, handles);

    // The humanoid ragdoll puppet: anchored root, pose-tracking drives, an
    // upright assist, and a jiggle chain on the torso.
    // The puppet stands on the open floor in front of the scenery ramp
    // (feet land at the template's foot height, 0.944 m below the pelvis).
    engine::physics_character::RagdollParameters parameters;
    parameters.position[0]  = -4.2;
    parameters.position[1]  = 0.95;
    parameters.position[2]  = 3.0;
    parameters.scale        = 1.0;
    parameters.totalMass    = 72.0;
    parameters.rootAnchored = true;
    engine::physics_character::RagdollInfo ragdoll
        = dependencies.character->createHumanoidRagdoll(dependencies.character->self, handles.world, &parameters, nullptr);
    if (ragdoll.figure == engine::physics_articulated::invalidFigure)
    {
        report << "the humanoid ragdoll was rejected\n";
        return;
    }
    handles.figures.push_back(ragdoll.figure);
    {
        const float puppetColor[3] = {0.85f, 0.52f, 0.18f};
        // adoptFigureLinks is declared in sandbox_scene.cpp; replicate the
        // adoption inline to avoid including a private header.
        std::uint32_t required = 0;
        dependencies.articulation->copyLinkBodyHandles(
            dependencies.articulation->self, ragdoll.figure, nullptr, 0, &required);
        std::vector<BodyHandle> linkBodies(required);
        if (required > 0
            && dependencies.articulation->copyLinkBodyHandles(
                dependencies.articulation->self, ragdoll.figure, linkBodies.data(), required, nullptr))
        {
            for (const BodyHandle linkBody : linkBodies)
            {
                pushBody(handles, linkBody, PrimitiveKind::Capsule, puppetColor, false);
            }
        }
    }

    engine::physics_character::CharacterDescription characterDescription;
    characterDescription.world          = handles.world;
    characterDescription.figure         = ragdoll.figure;
    characterDescription.torsoLinkIndex = 1;
    characterDescription.uprightTorque  = 260.0;
    engine::physics_character::CharacterError characterError = engine::physics_character::CharacterError::None;
    handles.extended.character
        = dependencies.character->createCharacter(dependencies.character->self, &characterDescription, &characterError);
    if (handles.extended.character == engine::physics_character::invalidCharacter)
    {
        report << "the character was rejected (error " << static_cast<std::uint32_t>(characterError) << ")\n";
        return;
    }
    dependencies.character->setCharacterMode(
        dependencies.character->self, handles.extended.character, engine::physics_character::CharacterMode::Kinematic);
    dependencies.character->setCharacterDriveStrength(dependencies.character->self, handles.extended.character, 0.9);
    // Pose targets: a relaxed stance close to the assembly pose (nearly
    // straight knees, slightly bent elbows). The assembly pose IS standing,
    // so small targets keep the servo in its linear zone instead of
    // slamming the joints at spawn.
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 4, -0.1);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 6, -0.1);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 8, -0.05);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 10, -0.05);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 7, 0.0);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 9, 0.0);
    dependencies.character->setCharacterJointTarget(dependencies.character->self, handles.extended.character, 1, 0.0);
    // Weight-bearing gains: the pose torque PD needs stiffness well above
    // the gravity torque divided by the acceptable sag (a stance knee
    // carries roughly half the body weight on a 0.45 m lever).
    dependencies.character->setCharacterJointGains(
        dependencies.character->self, handles.extended.character, 1, 2600.0, 90.0, 220.0);
    dependencies.character->setCharacterJointGains(
        dependencies.character->self, handles.extended.character, 7, 2600.0, 90.0, 220.0);
    dependencies.character->setCharacterJointGains(
        dependencies.character->self, handles.extended.character, 9, 2600.0, 90.0, 220.0);
    dependencies.character->setCharacterJointGains(
        dependencies.character->self, handles.extended.character, 8, 2600.0, 90.0, 220.0);
    dependencies.character->setCharacterJointGains(
        dependencies.character->self, handles.extended.character, 10, 2600.0, 90.0, 220.0);
    engine::physics_rigid::BodyHandle torsoBody = engine::physics_rigid::invalidBody;
    if (dependencies.articulation->linkBodyHandle(
            dependencies.articulation->self, ragdoll.figure, 1, &torsoBody))
    {
        handles.extended.characterTorsoBody = torsoBody;
    }

    // The jiggle chain hangs off the torso (a tail or tool pouch).
    std::array<double, 3> offsets[4] = {{0.0, -0.18, 0.0}, {0.0, -0.16, 0.0}, {0.0, -0.16, 0.0}, {0.0, -0.15, 0.0}};
    engine::physics_character::JiggleChainDescription chainDescription;
    chainDescription.boneCount            = 4;
    chainDescription.restOffsets          = offsets[0].data();
    chainDescription.stiffness            = 110.0;
    chainDescription.damping              = 10.0;
    chainDescription.gravityInfluence     = 0.7;
    chainDescription.maximumAngleRadians  = 0.8;
    handles.extended.jiggleChain
        = dependencies.character->createJiggleChain(dependencies.character->self, &chainDescription, &characterError);
    if (handles.extended.jiggleChain != engine::physics_character::invalidJiggleChain)
    {
        handles.extended.jiggleBoneCount = 4;
        for (std::uint32_t bone = 0; bone < 4; ++bone)
        {
            handles.extended.jiggleOffsets[bone][0] = offsets[bone][0];
            handles.extended.jiggleOffsets[bone][1] = offsets[bone][1];
            handles.extended.jiggleOffsets[bone][2] = offsets[bone][2];
        }
    }

    // The foot planter tracks the puppet's feet against the ground. The two
    // bone lengths and the hip anchor match the template's legs (hip 0.44 m
    // below the torso center, 0.45 m thigh and shank segments).
    engine::physics_character::FootPlanterDescription planterDescription;
    planterDescription.legUpperLength = 0.45;
    planterDescription.legLowerLength = 0.45;
    planterDescription.hipAnchor[1]   = -0.44;
    // Feet rest about 1.25 m below the torso center (the planter's root),
    // straight under the hips.
    planterDescription.footOffsets[0][0] = -0.09;
    planterDescription.footOffsets[0][1] = -1.25;
    planterDescription.footOffsets[1][0] = 0.09;
    planterDescription.footOffsets[1][1] = -1.25;
    planterDescription.footRadius        = 0.05;
    planterDescription.rayCastUp         = 1.6;
    planterDescription.leadTime          = 0.0;
    // The ground probes skip the puppet's own links so a cast through a leg
    // reaches the ground instead of the limb.
    std::vector<engine::physics_rigid::BodyHandle> puppetLinks;
    {
        std::uint32_t required = 0;
        dependencies.articulation->copyLinkBodyHandles(
            dependencies.articulation->self, ragdoll.figure, nullptr, 0, &required);
        puppetLinks.resize(required);
        if (required > 0
            && dependencies.articulation->copyLinkBodyHandles(
                dependencies.articulation->self, ragdoll.figure, puppetLinks.data(), required, nullptr))
        {
            planterDescription.ignoredBodies     = puppetLinks.data();
            planterDescription.ignoredBodyCount  = static_cast<std::uint32_t>(puppetLinks.size());
        }
    }
    handles.extended.footPlanter = dependencies.character->createFootPlanter(
        dependencies.character->self, handles.world, &planterDescription, &characterError);

    // A free ragdoll dropped from the wall ledge rolls down the slope: the
    // pure-dynamics contrast to the driven puppet.
    engine::physics_character::RagdollParameters dropped;
    dropped.position[0] = 3.2;
    dropped.position[1] = 1.4;
    dropped.position[2] = -2.4;
    dropped.scale       = 0.85;
    dropped.totalMass   = 60.0;
    const engine::physics_character::RagdollInfo droppedRagdoll
        = dependencies.character->createHumanoidRagdoll(dependencies.character->self, handles.world, &dropped, nullptr);
    if (droppedRagdoll.figure != engine::physics_articulated::invalidFigure)
    {
        handles.figures.push_back(droppedRagdoll.figure);
        const float ragdollColor[3] = {0.62f, 0.45f, 0.30f};
        std::uint32_t required = 0;
        dependencies.articulation->copyLinkBodyHandles(
            dependencies.articulation->self, droppedRagdoll.figure, nullptr, 0, &required);
        std::vector<BodyHandle> linkBodies(required);
        if (required > 0
            && dependencies.articulation->copyLinkBodyHandles(
                dependencies.articulation->self, droppedRagdoll.figure, linkBodies.data(), required, nullptr))
        {
            for (const BodyHandle linkBody : linkBodies)
            {
                pushBody(handles, linkBody, PrimitiveKind::Capsule, ragdollColor, false);
            }
        }
    }
}

// A small ledge ramp shared by the character scene's drop ragdoll.
void addLedgeRamp(const SceneDependencies &dependencies, SceneHandles &handles)
{
    BodyDescription ledge;
    ledge.shape               = engine::physics_rigid::ShapeDescription{};
    ledge.shape.kind          = ShapeKind::Box;
    ledge.shape.boxHalfExtents[0] = 1.6;
    ledge.shape.boxHalfExtents[1] = 0.12;
    ledge.shape.boxHalfExtents[2] = 1.6;
    ledge.transform.position      = Vec3{3.2, 1.2, -2.4};
    ledge.transform.rotation      = engine::physics_rigid::Quaternion{
        std::sin(0.18 * 0.5), 0.0, 0.0, std::cos(0.18 * 0.5)};
    ledge.kind                = engine::physics_rigid::BodyKind::Static;
    ledge.material.friction   = 0.7;
    const float ledgeColor[3] = {0.42f, 0.35f, 0.28f};
    pushBody(handles, createBody(dependencies, handles.world, ledge), PrimitiveKind::Box, ledgeColor, false);
}

} // namespace

bool buildExtendedScene(const SceneDependencies &dependencies, const SceneConfig &config, const std::string &sceneName,
    SceneHandles &outHandles, std::string &outReport)
{
    std::ostringstream report;
    if (sceneName == "fields")
    {
        addFieldsScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "rope")
    {
        addRopeScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "water")
    {
        addWaterScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "fluid")
    {
        addFluidScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "particles")
    {
        addParticlesScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "foliage")
    {
        addFoliageScene(dependencies, config, outHandles, report);
    }
    else if (sceneName == "character")
    {
        addLedgeRamp(dependencies, outHandles);
        addCharacterScene(dependencies, config, outHandles, report);
    }
    else
    {
        report << "unknown extended scene '" << sceneName << "'\n";
        outReport = report.str();
        return false;
    }
    outReport = report.str();
    return true;
}

void destroyExtendedScene(const SceneDependencies &dependencies, SceneHandles &handles)
{
    if (dependencies.fields != nullptr && handles.extended.windField != engine::physics_fields::invalidWind)
    {
        dependencies.fields->destroyWindField(dependencies.fields->self, handles.extended.windField);
    }
    if (dependencies.fields != nullptr && handles.extended.forceVolume != engine::physics_fields::invalidForceVolume)
    {
        dependencies.fields->destroyForceVolume(dependencies.fields->self, handles.extended.forceVolume);
    }
    if (dependencies.rope != nullptr)
    {
        for (const engine::physics_rope::RopeHandle ropeHandle : handles.extended.ropes)
        {
            dependencies.rope->destroyRope(dependencies.rope->self, ropeHandle);
        }
    }
    if (dependencies.water != nullptr && handles.extended.ocean != engine::physics_water::invalidOcean)
    {
        dependencies.water->destroyOcean(dependencies.water->self, handles.extended.ocean);
    }
    if (dependencies.fluid != nullptr && handles.extended.fluid != engine::physics_fluid::invalidFluid)
    {
        dependencies.fluid->destroyFluid(dependencies.fluid->self, handles.extended.fluid);
    }
    if (dependencies.particles != nullptr)
    {
        for (engine::physics_particles::EmitterHandle emitter : handles.extended.emitters)
        {
            if (emitter != engine::physics_particles::invalidEmitter)
            {
                dependencies.particles->destroyEmitter(dependencies.particles->self, emitter);
            }
        }
    }
    if (dependencies.foliage != nullptr && handles.extended.foliageField != engine::physics_foliage::invalidField)
    {
        dependencies.foliage->destroyField(dependencies.foliage->self, handles.extended.foliageField);
    }
    if (dependencies.character != nullptr)
    {
        if (handles.extended.character != engine::physics_character::invalidCharacter)
        {
            dependencies.character->destroyCharacter(dependencies.character->self, handles.extended.character);
        }
        if (handles.extended.jiggleChain != engine::physics_character::invalidJiggleChain)
        {
            dependencies.character->destroyJiggleChain(dependencies.character->self, handles.extended.jiggleChain);
        }
        if (handles.extended.footPlanter != engine::physics_character::invalidFootPlanter)
        {
            dependencies.character->destroyFootPlanter(dependencies.character->self, handles.extended.footPlanter);
        }
    }
    handles.extended = ExtendedHandles{};
}

void stepExtendedBeforeRigid(const SceneDependencies &dependencies, SceneHandles &handles, double simulatedSeconds,
    double fixedDeltaSeconds)
{
    if (handles.world == engine::physics_rigid::invalidWorld)
    {
        return;
    }
    // Force fields: wind drag, then placed volumes.
    if (dependencies.fields != nullptr && handles.extended.windField != engine::physics_fields::invalidWind)
    {
        dependencies.fields->applyWindDrag(
            dependencies.fields->self, handles.extended.windField, simulatedSeconds, fixedDeltaSeconds);
    }
    if (dependencies.fields != nullptr && handles.extended.forceVolume != engine::physics_fields::invalidForceVolume)
    {
        dependencies.fields->applyForceVolumes(dependencies.fields->self, fixedDeltaSeconds);
    }

    // Water: ripple propagation and the buoyancy pass.
    if (dependencies.water != nullptr && handles.extended.ocean != engine::physics_water::invalidOcean)
    {
        dependencies.water->stepRipples(dependencies.water->self, handles.extended.ocean, fixedDeltaSeconds);
        dependencies.water->applyBuoyancy(
            dependencies.water->self, handles.extended.ocean, simulatedSeconds, fixedDeltaSeconds);
    }

    // Foliage bend springs.
    if (dependencies.foliage != nullptr && handles.extended.foliageField != engine::physics_foliage::invalidField)
    {
        dependencies.foliage->step(dependencies.foliage->self, fixedDeltaSeconds);
    }

    // Particles.
    if (dependencies.particles != nullptr)
    {
        for (engine::physics_particles::EmitterHandle emitter : handles.extended.emitters)
        {
            if (emitter != engine::physics_particles::invalidEmitter)
            {
                dependencies.particles->step(dependencies.particles->self, emitter, fixedDeltaSeconds);
            }
        }
    }

    // Character control: pose drives plus the upright assist, then the foot
    // planter and the jiggle chain from the torso's fresh velocity sample.
    if (dependencies.character != nullptr
        && handles.extended.character != engine::physics_character::invalidCharacter)
    {
        dependencies.character->updateCharacter(
            dependencies.character->self, handles.extended.character, fixedDeltaSeconds);
    }
}

void stepExtendedAfterRigid(
    const SceneDependencies &dependencies, SceneHandles &handles, double simulatedSeconds, double fixedDeltaSeconds)
{
    if (handles.world == engine::physics_rigid::invalidWorld)
    {
        return;
    }
    // Ropes and the SPH fluid step against the freshly integrated bodies.
    if (dependencies.rope != nullptr)
    {
        for (const engine::physics_rope::RopeHandle ropeHandle : handles.extended.ropes)
        {
            dependencies.rope->step(dependencies.rope->self, ropeHandle, fixedDeltaSeconds);
        }
    }
    if (dependencies.fluid != nullptr && handles.extended.fluid != engine::physics_fluid::invalidFluid)
    {
        dependencies.fluid->step(dependencies.fluid->self, handles.extended.fluid, fixedDeltaSeconds);
    }

    // The foot planter and jiggle chain read the post-step transforms.
    if (dependencies.character != nullptr)
    {
        if (handles.extended.footPlanter != engine::physics_character::invalidFootPlanter
            && handles.extended.characterTorsoBody != engine::physics_rigid::invalidBody)
        {
            engine::physics_rigid::BodyState torso{};
            if (dependencies.rigid->copyBodyState(
                    dependencies.rigid->self, handles.world, handles.extended.characterTorsoBody, &torso))
            {
                const double velocity[3] = {torso.linearVelocity.x, torso.linearVelocity.y, torso.linearVelocity.z};
                dependencies.character->updateFootPlanter(dependencies.character->self,
                    handles.extended.footPlanter,
                    &torso.transform,
                    velocity,
                    fixedDeltaSeconds);
            }
        }
        if (handles.extended.jiggleChain != engine::physics_character::invalidJiggleChain
            && handles.extended.characterTorsoBody != engine::physics_rigid::invalidBody)
        {
            engine::physics_rigid::BodyState torso{};
            if (dependencies.rigid->copyBodyState(
                    dependencies.rigid->self, handles.world, handles.extended.characterTorsoBody, &torso))
            {
                const double velocity[3] = {torso.linearVelocity.x, torso.linearVelocity.y, torso.linearVelocity.z};
                dependencies.character->updateJiggleChain(dependencies.character->self,
                    handles.extended.jiggleChain,
                    &torso.transform,
                    velocity,
                    fixedDeltaSeconds);
            }
        }
    }
}

// Scripted-timeline latches (one timeline per process).
bool rippleScheduled_[3]   = {false, false, false};
bool trampleDone_          = false;
bool towKickDone_          = false;
bool rainBurstDone_        = false;

void updateExtendedScriptedTimeline(const SceneDependencies &dependencies, SceneHandles &handles,
    double simulatedSeconds, double fixedDeltaSeconds)
{
    (void)fixedDeltaSeconds;
    if (handles.world == engine::physics_rigid::invalidWorld)
    {
        return;
    }

    // Water: three splash ripples land at fixed times.
    if (dependencies.water != nullptr && handles.extended.ocean != engine::physics_water::invalidOcean)
    {
        static constexpr double kRippleTimes[3] = {0.8, 2.0, 3.2};
        static constexpr double kRippleX[3]     = {-2.6, 0.6, 2.8};
        static constexpr double kRippleZ[3]     = {1.2, -0.8, 1.4};
        for (std::uint32_t index = 0; index < 3; ++index)
        {
            if (!rippleScheduled_[index] && simulatedSeconds >= kRippleTimes[index])
            {
                rippleScheduled_[index] = true;
                const double center[3]  = {kRippleX[index], 0.55, kRippleZ[index]};
                dependencies.water->addRipple(dependencies.water->self, handles.extended.ocean, center, 0.16);
            }
        }
    }

    // Foliage: the roller sweeps across the field; every 0.5 s of simulated
    // time the blades under it take a trample disturbance.
    if (dependencies.foliage != nullptr && handles.extended.foliageField != engine::physics_foliage::invalidField
        && handles.paddle != engine::physics_rigid::invalidBody)
    {
        // The roller advances along +X at 1.1 m/s across the field.
        engine::physics_rigid::BodyState roller{};
        if (dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, handles.paddle, &roller))
        {
            const double positionX  = -6.5 + simulatedSeconds * 1.1;
            const double velocity[3] = {1.1, 0.0, 0.0};
            roller.transform.position = Vec3{positionX, 0.35, 0.0};
            dependencies.rigid->setBodyTransform(
                dependencies.rigid->self, handles.world, handles.paddle, &roller.transform, false);
            dependencies.rigid->setBodyLinearVelocity(dependencies.rigid->self, handles.world, handles.paddle, velocity);
            static double lastTrampleTime = -1.0;
            if (simulatedSeconds - lastTrampleTime >= 0.25)
            {
                lastTrampleTime = simulatedSeconds;
                const double trampleCenter[3] = {positionX, 0.0, 0.0};
                dependencies.foliage->applyRadialDisturbance(
                    dependencies.foliage->self, handles.extended.foliageField, trampleCenter, 1.6, 2.4);
            }
        }
    }

    // Rope: a one-time kick into the tow line's first crate.
    if (!towKickDone_ && simulatedSeconds >= 1.6 && dependencies.rope != nullptr
        && handles.extended.ropes.size() >= 3)
    {
        towKickDone_ = true;
        // The tow line's start crate: the third rope's start body is not
        // tracked separately, so kick by impulse on the nearest dynamic box
        // (deterministic: the scene builds them in fixed order).
        for (const SceneBody &sceneBody : handles.bodies)
        {
            engine::physics_rigid::BodyState state{};
            if (dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, sceneBody.body, &state)
                && state.transform.position.x > 4.0 && state.transform.position.x < 5.0
                && state.transform.position.y < 1.0)
            {
                const double impulse[3] = {0.0, 0.0, 26.0};
                dependencies.rigid->applyImpulseAtPoint(
                    dependencies.rigid->self, handles.world, sceneBody.body, impulse, nullptr);
                break;
            }
        }
    }

    // Particles: a one-time dust burst at the fire pit.
    if (!rainBurstDone_ && simulatedSeconds >= 1.1 && dependencies.particles != nullptr
        && handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Dust)]
               != engine::physics_particles::invalidEmitter)
    {
        rainBurstDone_ = true;
        const double burstCenter[3] = {0.0, 0.6, 1.8};
        dependencies.particles->burst(dependencies.particles->self,
            handles.extended.emitters[static_cast<std::uint32_t>(engine::physics_particles::EmitterKind::Dust)],
            burstCenter,
            60,
            3.2);
    }

    // Character: the puppet's arms trace a slow wave (physical animation).
    if (dependencies.character != nullptr
        && handles.extended.character != engine::physics_character::invalidCharacter)
    {
        const double wave = 0.6 * std::sin(6.283185307179586 * 0.35 * simulatedSeconds);
        dependencies.character->setCharacterJointTarget(
            dependencies.character->self, handles.extended.character, 3, wave);
        dependencies.character->setCharacterJointTarget(
            dependencies.character->self, handles.extended.character, 5, -wave);
    }
}

} // namespace demo::physics_sandbox
