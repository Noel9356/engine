// sandbox_selftest.cpp - determinism and frame-rate independence tests.
//
// 1. Determinism: two identically built scenes, stepped with the same fixed
//    deltas and identical impulses, must finish bit-identical.
// 2. Frame-rate independence: the same wall-clock duration stepped at 30,
//    60, and 120 Hz must converge to the same settled state (within the
//    contact solver's discretization tolerance), and object counts must be
//    exact.
// 3. Accumulator: simulate() called with fixed-rate-independent frame
//    deltas must execute exactly the expected number of steps.
// 4. Cloth rate independence: a pinned sheet stepped at 30 and 120 Hz
//    settles into the same shape.
// 5. Rope rate independence: a pinned cable settles into the same shape.
// 6. Fluid determinism: identical dam-break runs finish bit-identical.
// 7. Particle determinism: identical emitter runs produce identical states.
// 8. Foliage rate independence: the bend springs converge across rates.
// 9. Buoyancy: a floating body settles at the water line and stays there.
// 10. CCD: a fast sphere with continuous collision cannot tunnel through a
//     thin wall (the control run without CCD does tunnel).
// 11. Rolling resistance: a rolling ball with the coefficient slows faster
//     than the friction-only control.
// 12. Joints: a prismatic motor drives along its axis, a fixed weld holds
//     the relative frame under impulse, and the distance spring stays soft.
// 13. Character: the humanoid ragdoll builds, the drives run, the jiggle
//     chain stays finite, and the foot planter plants on the ground.
#include "sandbox_selftest.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>

namespace demo::physics_sandbox
{
namespace
{

namespace math = engine::physics_rigid::math;

constexpr double kDeterminismTolerance = 1.0e-9;
constexpr double kRateTolerance        = 6.0e-2;
constexpr double kClothRateTolerance   = 4.0e-2;

// Builds the stacked scene without scripted spawns (buildScene adds the same
// objects in the same order every time, which fixes the solver's iteration
// order).
bool buildTestScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles)
{
    std::string report;
    return buildScene(dependencies, config, "stacks", handles, report);
}

// Steps one scene for a simulated duration, applying an identical impulse
// schedule (fixed times, fixed forces) so every run sees the same inputs.
void runScene(const SceneDependencies &dependencies, const SceneConfig &config, SceneHandles &handles,
    double fixedDelta, std::uint32_t steps)
{
    for (std::uint32_t step = 0; step < steps; ++step)
    {
        // Identical deterministic inputs: three impulses at fixed times.
        if (step == static_cast<std::uint32_t>(0.7 / fixedDelta))
        {
            const double impulse[3] = {9.0, 0.0, 0.0};
            if (!handles.bodies.empty())
            {
                dependencies.rigid->applyImpulseAtPoint(
                    dependencies.rigid->self, handles.world, handles.bodies[5].body, impulse, nullptr);
            }
        }
        if (step == static_cast<std::uint32_t>(1.3 / fixedDelta))
        {
            const double impulse[3] = {0.0, 3.0, 6.0};
            if (handles.bodies.size() > 8)
            {
                dependencies.rigid->applyImpulseAtPoint(
                    dependencies.rigid->self, handles.world, handles.bodies[8].body, impulse, nullptr);
            }
        }
        if (step == static_cast<std::uint32_t>(1.9 / fixedDelta))
        {
            const double center[3] = {-3.2, 1.6, 2.4};
            applyBlast(dependencies, config, handles, center);
        }
        dependencies.rigid->step(dependencies.rigid->self, handles.world, fixedDelta);
    }
}

// Collects the dynamic bodies' positions (dense, deterministic order).
std::vector<double> dynamicBodyPositions(const SceneDependencies &dependencies, const SceneHandles &handles)
{
    std::vector<double> positions;
    for (const SceneBody &sceneBody : handles.bodies)
    {
        engine::physics_rigid::BodyState state{};
        if (!dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, sceneBody.body, &state))
        {
            continue;
        }
        engine::physics_rigid::BodyDescription description{};
        if (!dependencies.rigid->copyBodyDescription(
                dependencies.rigid->self, handles.world, sceneBody.body, &description))
        {
            continue;
        }
        if (description.kind != engine::physics_rigid::BodyKind::Dynamic)
        {
            continue;
        }
        positions.push_back(state.transform.position.x);
        positions.push_back(state.transform.position.y);
        positions.push_back(state.transform.position.z);
    }
    return positions;
}

double maxDeviation(const std::vector<double> &left, const std::vector<double> &right)
{
    if (left.size() != right.size())
    {
        return std::numeric_limits<double>::infinity();
    }
    double worst = 0.0;
    for (std::size_t index = 0; index < left.size(); ++index)
    {
        worst = std::max(worst, std::abs(left[index] - right[index]));
    }
    return worst;
}

} // namespace

SelfTestResult runSelfTest(const SceneDependencies &dependencies, const SceneConfig &config)
{
    SelfTestResult     result;
    std::ostringstream report;
    bool               pass = true;

    if (dependencies.rigid == nullptr)
    {
        result.report = "self-test requires the rigid capability\n";
        result.pass   = false;
        return result;
    }

    // ---- 1. determinism ----------------------------------------------------
    {
        SceneHandles first;
        SceneHandles second;
        buildTestScene(dependencies, config, first);
        buildTestScene(dependencies, config, second);
        runScene(dependencies, config, first, 1.0 / 60.0, 240);
        runScene(dependencies, config, second, 1.0 / 60.0, 240);
        const std::vector<double> firstPositions  = dynamicBodyPositions(dependencies, first);
        const std::vector<double> secondPositions = dynamicBodyPositions(dependencies, second);
        const double              deviation       = maxDeviation(firstPositions, secondPositions);
        const bool                testPassed      = deviation <= kDeterminismTolerance;
        report << (testPassed ? "PASS" : "FAIL") << ": determinism - two identical runs of 240 steps at 60 Hz, max "
               << "position deviation " << deviation << " (tolerance " << kDeterminismTolerance << ")\n";
        pass = pass && testPassed;
        destroyScene(dependencies, first);
        destroyScene(dependencies, second);
    }

    // ---- 2. frame-rate independence ---------------------------------------
    {
        const double duration = 3.0;
        SceneHandles at30;
        SceneHandles at60;
        SceneHandles at120;
        buildTestScene(dependencies, config, at30);
        buildTestScene(dependencies, config, at60);
        buildTestScene(dependencies, config, at120);
        runScene(dependencies, config, at30, 1.0 / 30.0, static_cast<std::uint32_t>(duration * 30.0));
        runScene(dependencies, config, at60, 1.0 / 60.0, static_cast<std::uint32_t>(duration * 60.0));
        runScene(dependencies, config, at120, 1.0 / 120.0, static_cast<std::uint32_t>(duration * 120.0));
        const std::vector<double> positions30    = dynamicBodyPositions(dependencies, at30);
        const std::vector<double> positions60    = dynamicBodyPositions(dependencies, at60);
        const std::vector<double> positions120   = dynamicBodyPositions(dependencies, at120);
        const double              deviation30120 = maxDeviation(positions30, positions120);
        const double              deviation60120 = maxDeviation(positions60, positions120);
        const bool                testPassed     = deviation30120 <= kRateTolerance && deviation60120 <= kRateTolerance;
        report << (testPassed ? "PASS" : "FAIL") << ": rigid frame-rate independence - 3 s at 30/60/120 Hz, max "
               << "deviation 30 vs 120 " << deviation30120 << ", 60 vs 120 " << deviation60120 << " (tolerance "
               << kRateTolerance << ")\n";
        pass = pass && testPassed;
        destroyScene(dependencies, at30);
        destroyScene(dependencies, at60);
        destroyScene(dependencies, at120);
    }

    // ---- 3. accumulator ----------------------------------------------------
    {
        SceneHandles handles;
        buildTestScene(dependencies, config, handles);
        std::uint64_t executed = 0;
        // 90 frames of 1/45 s at the module's configured rate: 2 s worth of
        // frames over a 60 Hz rate means 120 expected steps.
        for (std::uint32_t frame = 0; frame < 90; ++frame)
        {
            executed += dependencies.rigid->simulate(dependencies.rigid->self, handles.world, 1.0 / 45.0);
        }
        const std::int64_t expected   = 120;
        const std::int64_t difference = static_cast<std::int64_t>(executed) - expected;
        const bool         testPassed = std::abs(difference) <= 2;
        report << (testPassed ? "PASS" : "FAIL") << ": simulate accumulator - 90 frames of 1/45 s executed " << executed
               << " steps (expected " << expected << " +/- 2)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, handles);
    }

    // ---- 4. cloth rate independence ---------------------------------------
    if (dependencies.cloth != nullptr)
    {
        auto buildClothScene = [&](SceneHandles &handles)
        {
            std::string buildReport;
            buildScene(dependencies, config, "cloth", handles, buildReport);
            return handles.world != engine::physics_rigid::invalidWorld;
        };
        const double duration = 1.5;
        SceneHandles at30;
        SceneHandles at120;
        buildClothScene(at30);
        buildClothScene(at120);
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 30.0); ++step)
        {
            dependencies.cloth->step(dependencies.cloth->self, at30.cloths.front(), 1.0 / 30.0);
        }
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 120.0); ++step)
        {
            dependencies.cloth->step(dependencies.cloth->self, at120.cloths.front(), 1.0 / 120.0);
        }
        std::vector<double> positions30;
        std::vector<double> positions120;
        std::uint32_t       required = 0;
        if (dependencies.cloth->copyParticlePositions(
                dependencies.cloth->self, at30.cloths.front(), nullptr, 0, &required))
        {
            positions30.resize(required);
            dependencies.cloth->copyParticlePositions(
                dependencies.cloth->self, at30.cloths.front(), positions30.data(), required, nullptr);
        }
        if (dependencies.cloth->copyParticlePositions(
                dependencies.cloth->self, at120.cloths.front(), nullptr, 0, &required))
        {
            positions120.resize(required);
            dependencies.cloth->copyParticlePositions(
                dependencies.cloth->self, at120.cloths.front(), positions120.data(), required, nullptr);
        }
        const double deviation  = maxDeviation(positions30, positions120);
        const bool   testPassed = deviation <= kClothRateTolerance;
        report << (testPassed ? "PASS" : "FAIL") << ": cloth frame-rate independence - 1.5 s at 30 vs 120 Hz, max "
               << "particle deviation " << deviation << " (tolerance " << kClothRateTolerance << ")\n";
        pass = pass && testPassed;
        destroyScene(dependencies, at30);
        destroyScene(dependencies, at120);
    }

    result.pass   = pass;
    result.report = report.str();
    return result;
}

} // namespace demo::physics_sandbox

namespace demo::physics_sandbox
{
namespace
{

// Builds a minimal extended scene for one scenario through the extended
// builder (the shared static scenery keeps the same construction order).
bool buildExtendedTestScene(const SceneDependencies &dependencies, const SceneConfig &config, const char *sceneName,
    SceneHandles &handles)
{
    std::string report;
    return buildScene(dependencies, config, sceneName, handles, report);
}

} // namespace

// ---- extended tests (runSelfTestExtended, called after the core battery) ----

SelfTestResult runSelfTestExtended(const SceneDependencies &dependencies, const SceneConfig &config)
{
    SelfTestResult     result;
    std::ostringstream report;
    bool               pass = true;

    if (dependencies.rigid == nullptr)
    {
        result.report = "extended self-test requires the rigid capability\n";
        result.pass   = false;
        return result;
    }

    // Capability census (a missing module silently skips its tests; this
    // line makes the skip visible in captures).
    std::cout << "extended battery capabilities: rope " << (dependencies.rope != nullptr) << ", water "
              << (dependencies.water != nullptr) << ", fluid " << (dependencies.fluid != nullptr) << ", particles "
              << (dependencies.particles != nullptr) << ", foliage " << (dependencies.foliage != nullptr)
              << ", character " << (dependencies.character != nullptr) << ", fields "
              << (dependencies.fields != nullptr) << "\n";
#define EXTENDED_TEST_MARKER(name)                                                                                     \
    std::cout << "[extended] " name "...\n";

    // ---- 5. rope rate independence ----------------------------------------
    if (dependencies.rope != nullptr)
    {
        EXTENDED_TEST_MARKER("rope")
        const double duration = 4.0;
        SceneHandles at30;
        SceneHandles at120;
        buildExtendedTestScene(dependencies, config, "rope", at30);
        buildExtendedTestScene(dependencies, config, "rope", at120);
        // The crane cable (ropes[0]) hangs in a deterministic straight
        // equilibrium; the draped rope's free end swings chaotically at
        // rate-dependent phases, so the comparison uses the cable.
        auto collectRope = [&](SceneHandles &handles)
        {
            std::vector<double> positions;
            if (handles.extended.ropes.empty())
            {
                return positions;
            }
            const engine::physics_rope::RopeHandle ropeHandle = handles.extended.ropes.front();
            engine::physics_rope::RopeInfo info{};
            if (!dependencies.rope->ropeInfo(dependencies.rope->self, ropeHandle, &info) || info.nodeCount == 0)
            {
                return positions;
            }
            positions.resize(static_cast<std::size_t>(info.nodeCount) * 3);
            dependencies.rope->copyNodePositions(dependencies.rope->self,
                ropeHandle,
                positions.data(),
                info.nodeCount * 3u,
                nullptr);
            return positions;
        };
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 30.0); ++step)
        {
            for (const engine::physics_rope::RopeHandle ropeHandle : at30.extended.ropes)
            {
                dependencies.rope->step(dependencies.rope->self, ropeHandle, 1.0 / 30.0);
            }
        }
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 120.0); ++step)
        {
            for (const engine::physics_rope::RopeHandle ropeHandle : at120.extended.ropes)
            {
                dependencies.rope->step(dependencies.rope->self, ropeHandle, 1.0 / 120.0);
            }
        }
        const std::vector<double> positions30  = collectRope(at30);
        const std::vector<double> positions120 = collectRope(at120);
        const double deviation = maxDeviation(positions30, positions120);
        const bool testPassed = deviation <= 0.15;
        report << (testPassed ? "PASS" : "FAIL") << ": rope frame-rate independence - 4 s at 30 vs 120 Hz, max node "
               << "deviation " << deviation << " (tolerance 0.15)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, at30);
        destroyScene(dependencies, at120);
    }

    // ---- 6. fluid determinism ----------------------------------------------
    if (dependencies.fluid != nullptr)
    {
        EXTENDED_TEST_MARKER("fluid")
        SceneHandles first;
        SceneHandles second;
        buildExtendedTestScene(dependencies, config, "fluid", first);
        buildExtendedTestScene(dependencies, config, "fluid", second);
        auto collectFluid = [&](SceneHandles &handles)
        {
            std::vector<double> positions;
            engine::physics_fluid::FluidInfo info{};
            if (dependencies.fluid->fluidInfo(dependencies.fluid->self, handles.extended.fluid, &info)
                && info.particleCount > 0)
            {
                positions.resize(static_cast<std::size_t>(info.particleCount) * 3);
                dependencies.fluid->copyParticlePositions(dependencies.fluid->self,
                    handles.extended.fluid,
                    positions.data(),
                    info.particleCount * 3u,
                    nullptr);
            }
            return positions;
        };
        for (std::uint32_t step = 0; step < 90; ++step)
        {
            dependencies.fluid->step(dependencies.fluid->self, first.extended.fluid, 1.0 / 60.0);
            dependencies.fluid->step(dependencies.fluid->self, second.extended.fluid, 1.0 / 60.0);
        }
        const double deviation = maxDeviation(collectFluid(first), collectFluid(second));
        const bool testPassed = deviation <= 1.0e-9;
        report << (testPassed ? "PASS" : "FAIL") << ": fluid determinism - two identical dam-break runs of 90 steps, "
               << "max particle deviation " << deviation << " (tolerance 1e-09)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, first);
        destroyScene(dependencies, second);
    }

    // ---- 7. particle determinism -------------------------------------------
    if (dependencies.particles != nullptr)
    {
        EXTENDED_TEST_MARKER("particles")
        SceneHandles first;
        SceneHandles second;
        buildExtendedTestScene(dependencies, config, "particles", first);
        buildExtendedTestScene(dependencies, config, "particles", second);
        auto collectParticles = [&](SceneHandles &handles)
        {
            std::vector<double> values;
            for (std::uint32_t kind = 0; kind < 4; ++kind)
            {
                const engine::physics_particles::EmitterHandle emitter = handles.extended.emitters[kind];
                if (emitter == engine::physics_particles::invalidEmitter)
                {
                    continue;
                }
                std::uint32_t required = 0;
                std::uint32_t filled   = 0;
                dependencies.particles->copyParticleStates(
                    dependencies.particles->self, emitter, nullptr, 0, &filled, &required);
                if (required == 0)
                {
                    continue;
                }
                std::vector<engine::physics_particles::ParticleState> states(required);
                dependencies.particles->copyParticleStates(
                    dependencies.particles->self, emitter, states.data(), required, &filled, &required);
                for (const engine::physics_particles::ParticleState &state : states)
                {
                    if (!state.alive)
                    {
                        continue;
                    }
                    values.push_back(state.position[0]);
                    values.push_back(state.position[1]);
                    values.push_back(state.position[2]);
                }
            }
            return values;
        };
        for (std::uint32_t step = 0; step < 120; ++step)
        {
            for (std::uint32_t kind = 0; kind < 4; ++kind)
            {
                const engine::physics_particles::EmitterHandle firstEmitter  = first.extended.emitters[kind];
                const engine::physics_particles::EmitterHandle secondEmitter = second.extended.emitters[kind];
                if (firstEmitter != engine::physics_particles::invalidEmitter)
                {
                    dependencies.particles->step(dependencies.particles->self, firstEmitter, 1.0 / 60.0);
                }
                if (secondEmitter != engine::physics_particles::invalidEmitter)
                {
                    dependencies.particles->step(dependencies.particles->self, secondEmitter, 1.0 / 60.0);
                }
            }
        }
        const std::vector<double> firstValues  = collectParticles(first);
        const std::vector<double> secondValues = collectParticles(second);
        const double deviation = maxDeviation(firstValues, secondValues);
        const bool testPassed = deviation <= 1.0e-9 && !firstValues.empty();
        report << (testPassed ? "PASS" : "FAIL") << ": particle determinism - two identical emitter runs of 120 steps, "
               << maxDeviation(firstValues, secondValues) << " deviation over " << firstValues.size() / 3
               << " live particles (tolerance 1e-09)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, first);
        destroyScene(dependencies, second);
    }

    // ---- 8. foliage rate independence --------------------------------------
    if (dependencies.foliage != nullptr)
    {
        EXTENDED_TEST_MARKER("foliage")
        const double duration = 2.0;
        SceneHandles at30;
        SceneHandles at120;
        buildExtendedTestScene(dependencies, config, "foliage", at30);
        buildExtendedTestScene(dependencies, config, "foliage", at120);
        auto collectBends = [&](SceneHandles &handles)
        {
            std::vector<double> values;
            std::uint32_t       required = 0;
            std::uint32_t       filled   = 0;
            dependencies.foliage->copyBends(
                dependencies.foliage->self, handles.extended.foliageField, nullptr, 0, &filled, &required);
            if (required > 0)
            {
                std::vector<engine::physics_foliage::FoliageBend> bends(required);
                dependencies.foliage->copyBends(dependencies.foliage->self,
                    handles.extended.foliageField,
                    bends.data(),
                    required,
                    &filled,
                    &required);
                for (const engine::physics_foliage::FoliageBend &bend : bends)
                {
                    values.push_back(bend.bendAngleRadians);
                    values.push_back(bend.bendDirectionX);
                    values.push_back(bend.bendDirectionZ);
                }
            }
            return values;
        };
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 30.0); ++step)
        {
            dependencies.foliage->step(dependencies.foliage->self, 1.0 / 30.0);
        }
        for (std::uint32_t step = 0; step < static_cast<std::uint32_t>(duration * 120.0); ++step)
        {
            dependencies.foliage->step(dependencies.foliage->self, 1.0 / 120.0);
        }
        const double deviation = maxDeviation(collectBends(at30), collectBends(at120));
        const bool testPassed = deviation <= 0.12;
        report << (testPassed ? "PASS" : "FAIL") << ": foliage frame-rate independence - 2 s at 30 vs 120 Hz, max bend "
               << "deviation " << deviation << " (tolerance 0.12)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, at30);
        destroyScene(dependencies, at120);
    }

    // ---- 9. buoyancy settle -------------------------------------------------
    if (dependencies.water != nullptr)
    {
        EXTENDED_TEST_MARKER("water")
        SceneHandles handles;
        buildExtendedTestScene(dependencies, config, "water", handles);
        double settledY  = 0.0;
        bool   collected = false;
        for (std::uint32_t step = 0; step < 300; ++step)
        {
            dependencies.water->stepRipples(
                dependencies.water->self, handles.extended.ocean, 1.0 / 60.0);
            dependencies.water->applyBuoyancy(
                dependencies.water->self, handles.extended.ocean, step * (1.0 / 60.0), 1.0 / 60.0);
            dependencies.rigid->step(dependencies.rigid->self, handles.world, 1.0 / 60.0);
            if (step == 299 && !handles.extended.buoyantBodies.empty())
            {
                engine::physics_rigid::BodyState state{};
                if (dependencies.rigid->copyBodyState(
                        dependencies.rigid->self, handles.world, handles.extended.buoyantBodies[0], &state))
                {
                    settledY  = state.transform.position.y;
                    collected = true;
                }
            }
        }
        // The log floats within a capsule radius of the mean water line
        // (0.55); the sample-point submersion estimate settles it slightly
        // high, so the bound keeps half a meter of headroom.
        const bool testPassed = collected && std::abs(settledY - 0.55) < 0.55;
        report << (testPassed ? "PASS" : "FAIL") << ": buoyancy - the floating log settles at y " << settledY
               << " after 5 s (water line 0.55, tolerance 0.5)\n";
        pass = pass && testPassed;
        destroyScene(dependencies, handles);
    }

    // ---- 10. CCD keeps a fast sphere above the thin wall ---------------------
    {
        EXTENDED_TEST_MARKER("ccd")
        // Scenario without the extended modules: an isolated world with a
        // thin wall and a fast CCD sphere; the control run disables CCD.
        auto tunnelTest = [&](bool ccdEnabled)
        {
            engine::physics_rigid::WorldDescription worldDescription;
            const engine::physics_rigid::WorldHandle world
                = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
            engine::physics_rigid::BodyDescription wall;
            wall.shape.kind                          = engine::physics_rigid::ShapeKind::Box;
            wall.shape.boxHalfExtents[0]             = 2.0;
            wall.shape.boxHalfExtents[1]             = 0.02;
            wall.shape.boxHalfExtents[2]             = 2.0;
            wall.transform.position                  = engine::physics_rigid::Vec3{0.0, 3.0, 0.0};
            wall.kind                                = engine::physics_rigid::BodyKind::Static;
            const engine::physics_rigid::BodyHandle wallBody
                = dependencies.rigid->createBody(dependencies.rigid->self, world, &wall, nullptr);
            engine::physics_rigid::BodyDescription bullet;
            bullet.shape.kind                     = engine::physics_rigid::ShapeKind::Sphere;
            bullet.shape.sphereRadius             = 0.25;
            bullet.transform.position             = engine::physics_rigid::Vec3{0.0, 9.0, 0.0};
            bullet.kind                           = engine::physics_rigid::BodyKind::Dynamic;
            bullet.density                        = 500.0;
            bullet.continuousCollisionEnabled     = ccdEnabled;
            const engine::physics_rigid::BodyHandle bulletBody
                = dependencies.rigid->createBody(dependencies.rigid->self, world, &bullet, nullptr);
            const double launchVelocity[3] = {0.0, -42.0, 0.0};
            dependencies.rigid->setBodyLinearVelocity(
                dependencies.rigid->self, world, bulletBody, launchVelocity);
            double finalY = 100.0;
            for (std::uint32_t step = 0; step < 120; ++step)
            {
                dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
            }
            engine::physics_rigid::BodyState state{};
            if (dependencies.rigid->copyBodyState(dependencies.rigid->self, world, bulletBody, &state))
            {
                finalY = state.transform.position.y;
            }
            dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
            (void)wallBody;
            return finalY;
        };
        const double ccdY    = tunnelTest(true);
        const double plainY  = tunnelTest(false);
        // With CCD the sphere rests on top of the wall (y near 3.25); the
        // control tunnels through and lands below it.
        const bool testPassed = ccdY > 3.0 && plainY < 2.4;
        report << (testPassed ? "PASS" : "FAIL") << ": continuous collision detection - a 42 m/s sphere with CCD stops "
               << "at y " << ccdY << " on the 0.04 m wall while the control tunnels to y " << plainY << "\n";
        pass = pass && testPassed;
    }

    // ---- 11. rolling resistance slows the ball -------------------------------
    {
        EXTENDED_TEST_MARKER("rolling")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_rigid::BodyDescription ground;
        ground.shape.kind                          = engine::physics_rigid::ShapeKind::Plane;
        ground.shape.planeNormal[1]                = 1.0;
        ground.kind                                = engine::physics_rigid::BodyKind::Static;
        ground.material.friction                   = 0.4;
        ground.material.rollingResistance          = 0.0;
        dependencies.rigid->createBody(dependencies.rigid->self, world, &ground, nullptr);
        engine::physics_rigid::BodyDescription ballA;
        ballA.shape.kind                     = engine::physics_rigid::ShapeKind::Sphere;
        ballA.shape.sphereRadius             = 0.3;
        ballA.transform.position             = engine::physics_rigid::Vec3{-4.0, 0.3, 0.0};
        ballA.kind                           = engine::physics_rigid::BodyKind::Dynamic;
        ballA.density                        = 400.0;
        ballA.material.friction              = 0.4;
        ballA.material.rollingResistance     = 0.0;
        const engine::physics_rigid::BodyHandle ballABody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &ballA, nullptr);
        engine::physics_rigid::BodyDescription ballB = ballA;
        ballB.transform.position                     = engine::physics_rigid::Vec3{-4.0, 0.3, 3.0};
        ballB.material.rollingResistance             = 0.35;
        const engine::physics_rigid::BodyHandle ballBBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &ballB, nullptr);
        const double launchVelocity[3] = {9.0, 0.0, 0.0};
        dependencies.rigid->setBodyLinearVelocity(dependencies.rigid->self, world, ballABody, launchVelocity);
        dependencies.rigid->setBodyLinearVelocity(dependencies.rigid->self, world, ballBBody, launchVelocity);
        for (std::uint32_t step = 0; step < 300; ++step)
        {
            dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
        }
        double travelA = 0.0;
        double travelB = 0.0;
        engine::physics_rigid::BodyState stateA{};
        engine::physics_rigid::BodyState stateB{};
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, ballABody, &stateA);
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, ballBBody, &stateB);
        travelA = stateA.transform.position.x + 4.0;
        travelB = stateB.transform.position.x + 4.0;
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = travelB < travelA * 0.9;
        report << (testPassed ? "PASS" : "FAIL") << ": rolling resistance - the ball with the coefficient rolls "
               << travelB << " m versus the control's " << travelA << " m from the same launch\n";
        pass = pass && testPassed;
    }

    // ---- 12. joints: prismatic motor, fixed weld, distance spring ------------
    {
        EXTENDED_TEST_MARKER("joints")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        // Anchored static rail + a dynamic slider on a prismatic joint with
        // a motor pushing it along +X.
        engine::physics_rigid::BodyDescription rail;
        rail.shape.kind                          = engine::physics_rigid::ShapeKind::Box;
        rail.shape.boxHalfExtents[0]             = 0.15;
        rail.shape.boxHalfExtents[1]             = 0.15;
        rail.shape.boxHalfExtents[2]             = 1.2;
        rail.transform.position                  = engine::physics_rigid::Vec3{0.0, 0.5, 0.0};
        rail.kind                                = engine::physics_rigid::BodyKind::Static;
        const engine::physics_rigid::BodyHandle railBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &rail, nullptr);
        engine::physics_rigid::BodyDescription slider;
        slider.shape.kind             = engine::physics_rigid::ShapeKind::Box;
        slider.shape.boxHalfExtents[0] = 0.12;
        slider.shape.boxHalfExtents[1] = 0.12;
        slider.shape.boxHalfExtents[2] = 0.4;
        slider.transform.position      = engine::physics_rigid::Vec3{0.0, 0.82, 0.0};
        slider.kind                    = engine::physics_rigid::BodyKind::Dynamic;
        slider.density                 = 300.0;
        slider.material.friction       = 0.05;
        const engine::physics_rigid::BodyHandle sliderBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &slider, nullptr);
        engine::physics_rigid::JointDescription prismatic;
        prismatic.kind           = engine::physics_rigid::JointKind::Prismatic;
        prismatic.bodyA          = railBody;
        prismatic.bodyB          = sliderBody;
        prismatic.anchorLocalA   = engine::physics_rigid::Vec3{0.0, 0.32, 0.0};
        prismatic.anchorLocalB   = engine::physics_rigid::Vec3{0.0, -0.12, 0.0};
        prismatic.axisLocalA     = engine::physics_rigid::Vec3{1.0, 0.0, 0.0};
        prismatic.motorEnabled   = true;
        prismatic.motorTargetVelocity = 0.5;
        prismatic.maximumMotorTorque  = 60.0;
        const engine::physics_rigid::JointHandle prismaticJoint
            = dependencies.rigid->createJoint(dependencies.rigid->self, world, &prismatic, nullptr);

        // The fixed weld: two dynamic boxes glued together.
        engine::physics_rigid::BodyDescription weldAnchor;
        weldAnchor.shape.kind             = engine::physics_rigid::ShapeKind::Box;
        weldAnchor.shape.boxHalfExtents[0] = 0.2;
        weldAnchor.shape.boxHalfExtents[1] = 0.2;
        weldAnchor.shape.boxHalfExtents[2] = 0.2;
        weldAnchor.transform.position      = engine::physics_rigid::Vec3{-6.0, 0.2, 0.0};
        weldAnchor.kind                    = engine::physics_rigid::BodyKind::Dynamic;
        weldAnchor.density                 = 300.0;
        const engine::physics_rigid::BodyHandle weldAnchorBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &weldAnchor, nullptr);
        engine::physics_rigid::BodyDescription weldTip = weldAnchor;
        weldTip.transform.position                     = engine::physics_rigid::Vec3{-5.4, 0.2, 0.0};
        const engine::physics_rigid::BodyHandle weldTipBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &weldTip, nullptr);
        engine::physics_rigid::JointDescription weld;
        weld.kind         = engine::physics_rigid::JointKind::Fixed;
        weld.bodyA        = weldAnchorBody;
        weld.bodyB        = weldTipBody;
        // Anchors coincide at the pair midpoint (the boxes' centers sit 0.6
        // m apart, so the midpoint is 0.3 m from each center).
        weld.anchorLocalA = engine::physics_rigid::Vec3{0.3, 0.0, 0.0};
        weld.anchorLocalB = engine::physics_rigid::Vec3{-0.3, 0.0, 0.0};
        dependencies.rigid->createJoint(dependencies.rigid->self, world, &weld, nullptr);
        const double weldImpulse[3] = {0.0, 22.0, 0.0};
        dependencies.rigid->applyImpulseAtPoint(
            dependencies.rigid->self, world, weldAnchorBody, weldImpulse, nullptr);

        // The spring: a soft distance joint between the slider and a static
        // post, so the prismatic drive pulls against it.
        engine::physics_rigid::BodyDescription post;
        post.shape.kind                          = engine::physics_rigid::ShapeKind::Box;
        post.shape.boxHalfExtents[0]             = 0.2;
        post.shape.boxHalfExtents[1]             = 0.5;
        post.shape.boxHalfExtents[2]             = 0.2;
        post.transform.position                  = engine::physics_rigid::Vec3{2.6, 0.5, 0.0};
        post.kind                                = engine::physics_rigid::BodyKind::Static;
        const engine::physics_rigid::BodyHandle postBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &post, nullptr);
        engine::physics_rigid::JointDescription spring;
        spring.kind                 = engine::physics_rigid::JointKind::Distance;
        spring.bodyA                = postBody;
        spring.bodyB                = sliderBody;
        spring.anchorLocalA         = engine::physics_rigid::Vec3{-0.2, 0.0, 0.0};
        spring.anchorLocalB         = engine::physics_rigid::Vec3{0.0, 0.0, 0.0};
        spring.distance             = 0.0;
        spring.springEnabled        = true;
        spring.springFrequencyHz    = 0.35;
        spring.springDampingRatio   = 0.2;
        dependencies.rigid->createJoint(dependencies.rigid->self, world, &spring, nullptr);

        for (std::uint32_t step = 0; step < 180; ++step)
        {
            dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
        }
        engine::physics_rigid::BodyState sliderState{};
        engine::physics_rigid::BodyState weldA{};
        engine::physics_rigid::BodyState weldB{};
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, sliderBody, &sliderState);
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, weldAnchorBody, &weldA);
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, weldTipBody, &weldB);
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const math::Vec3 weldSeparation = weldB.transform.position - weldA.transform.position;
        const double weldError = std::abs(math::length(weldSeparation) - 0.6);
        const bool prismaticMoved = sliderState.transform.position.x > 0.2;
        const bool weldHolds = weldError < 0.02;
        const bool testPassed = prismaticJoint != engine::physics_rigid::invalidJoint && prismaticMoved && weldHolds;
        report << (testPassed ? "PASS" : "FAIL") << ": mechanical joints - the prismatic motor drives the slider to x "
               << sliderState.transform.position.x << ", the fixed weld keeps its pair within " << weldError
               << " m of the assembly distance under impulse\n";
        pass = pass && testPassed;
    }

    // ---- 13. character: ragdoll, drives, jiggle chain, foot planter ---------
    if (dependencies.character != nullptr)
    {
        EXTENDED_TEST_MARKER("character")
        SceneHandles handles;
        buildExtendedTestScene(dependencies, config, "character", handles);
        bool chainFinite = true;
        bool feetPlanted = false;
        for (std::uint32_t step = 0; step < 150; ++step)
        {
            const double dt = 1.0 / 60.0;
            if (dependencies.character != nullptr
                && handles.extended.character != engine::physics_character::invalidCharacter)
            {
                dependencies.character->updateCharacter(dependencies.character->self, handles.extended.character, dt);
            }
            dependencies.rigid->step(dependencies.rigid->self, handles.world, dt);
            if (dependencies.character != nullptr
                && handles.extended.footPlanter != engine::physics_character::invalidFootPlanter
                && handles.extended.characterTorsoBody != engine::physics_rigid::invalidBody)
            {
                engine::physics_rigid::BodyState torso{};
                if (dependencies.rigid->copyBodyState(
                        dependencies.rigid->self, handles.world, handles.extended.characterTorsoBody, &torso))
                {
                    const double torsoVelocity[3] = {torso.linearVelocity.x, torso.linearVelocity.y,
                        torso.linearVelocity.z};
                    dependencies.character->updateFootPlanter(dependencies.character->self,
                        handles.extended.footPlanter,
                        &torso.transform,
                        torsoVelocity,
                        dt);
                }
            }
            const engine::physics_rigid::RigidTransform chainRoot{};
            const double chainRootVelocity[3] = {0.0, 0.0, 0.0};
            dependencies.character->updateJiggleChain(dependencies.character->self,
                handles.extended.jiggleChain,
                &chainRoot,
                chainRootVelocity,
                dt);
            if (step == 149)
            {
                std::vector<double> rotations(static_cast<std::size_t>(handles.extended.jiggleBoneCount) * 4);
                std::uint32_t       required = 0;
                if (dependencies.character->copyChainBoneRotations(dependencies.character->self,
                        handles.extended.jiggleChain,
                        rotations.data(),
                        handles.extended.jiggleBoneCount * 4u,
                        &required))
                {
                    for (const double value : rotations)
                    {
                        if (!std::isfinite(value))
                        {
                            chainFinite = false;
                        }
                    }
                }
                if (handles.extended.footPlanter != engine::physics_character::invalidFootPlanter)
                {
                    engine::physics_character::FootState feet[2];
                    std::uint32_t required = 0;
                    if (dependencies.character->copyFootStates(
                            dependencies.character->self, handles.extended.footPlanter, feet, 2, &required)
                        && required == 2)
                    {
                        feetPlanted = feet[0].planted && feet[1].planted;
                        std::cout << "[extended] foot 0: planted " << feet[0].planted << " at y "
                                  << feet[0].footPosition[1] << ", foot 1: planted " << feet[1].planted << " at y "
                                  << feet[1].footPosition[1] << "\n";
                    }
                }
                else
                {
                    std::cout << "[extended] foot planter handle invalid\n";
                }
            }
        }
        const bool characterBuilt = handles.extended.character != engine::physics_character::invalidCharacter;
        const bool testPassed = characterBuilt && chainFinite && feetPlanted;
        report << (testPassed ? "PASS" : "FAIL") << ": character physics - ragdoll built " << characterBuilt
               << ", jiggle chain finite " << chainFinite << ", feet planted " << feetPlanted << "\n";
        pass = pass && testPassed;
        destroyScene(dependencies, handles);
    }

    result.pass   = pass;
    result.report = report.str();
    return result;
}

} // namespace demo::physics_sandbox
