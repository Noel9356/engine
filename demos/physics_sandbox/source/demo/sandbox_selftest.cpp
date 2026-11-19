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
// Extended battery additions:
// 21. Batched sphere queries: collideSpheres matches per-sphere collideSphere
//     exactly for a mixed static/dynamic field (hit flags, normals, depths,
//     and bodies).
// 22. Cloth sleep: a settling sheet reaches the sleeping state, holds it
//     while undisturbed, and wakes on a wind change.
// 23. Particle simulation LOD: an emitter at Simplified importance skips
//     about half the steps, a Cached emitter freezes, and Full runs every
//     step again; emission totals stay rate-correct across the bands.
// 24. Per-ocean gravity: two oceans over worlds with different gravity
//     produce different buoyancy for identical bodies.
// 25. Rigid simulation LOD: importance 1.0 keeps every body in the Full
//     band, reduced-importance bodies count as lod-reduced while their
//     stacks still rest stably on the ground, and identically scored twin
//     worlds stay bit-identical (the budgets follow the scores
//     deterministically).
// 26. Articulated simulation LOD: a Cached figure skips every drive
//     refresh (the motors keep their last target) and the skip counter
//     reports it.
// 27. Character simulation LOD: a Cached character stops driving and the
//     module-wide skip counter reports it.
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
            for (std::uint32_t kind = 0; kind < ExtendedHandles::kEmitterSlotCount; ++kind)
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
            for (std::uint32_t kind = 0; kind < ExtendedHandles::kEmitterSlotCount; ++kind)
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

    // ---- 14. cylinder shapes rest, stack, and ray cast ----------------------
    {
        EXTENDED_TEST_MARKER("cylinder")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_rigid::BodyDescription ground;
        ground.shape.kind           = engine::physics_rigid::ShapeKind::Plane;
        ground.shape.planeNormal[1] = 1.0;
        ground.kind                 = engine::physics_rigid::BodyKind::Static;
        ground.material.friction    = 0.7;
        dependencies.rigid->createBody(dependencies.rigid->self, world, &ground, nullptr);
        engine::physics_rigid::BodyDescription barrel;
        barrel.shape.kind               = engine::physics_rigid::ShapeKind::Cylinder;
        barrel.shape.cylinderRadius     = 0.4;
        barrel.shape.cylinderHalfHeight = 0.5;
        barrel.transform.position       = engine::physics_rigid::Vec3{0.0, 0.5, 0.0};
        barrel.kind                     = engine::physics_rigid::BodyKind::Dynamic;
        barrel.density                  = 300.0;
        barrel.material.friction        = 0.6;
        const engine::physics_rigid::BodyHandle barrelBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &barrel, nullptr);
        engine::physics_rigid::BodyDescription barrelTop = barrel;
        barrelTop.transform.position                     = engine::physics_rigid::Vec3{0.05, 1.52, 0.0};
        const engine::physics_rigid::BodyHandle barrelTopBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &barrelTop, nullptr);
        for (std::uint32_t step = 0; step < 240; ++step)
        {
            dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
        }
        engine::physics_rigid::BodyState baseState{};
        engine::physics_rigid::BodyState topState{};
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, barrelBody, &baseState);
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, barrelTopBody, &topState);
        // Mass properties: the solid-cylinder inertia about the axis is
        // m r^2 / 2 with m = density pi r^2 (2h).
        engine::physics_rigid::BodyMassProperties mass{};
        dependencies.rigid->copyBodyMassProperties(dependencies.rigid->self, world, barrelBody, &mass);
        const double expectedMass  = 300.0 * 3.14159265358979323846 * 0.16 * 1.0;
        const double expectedAxial = 0.5 * expectedMass * 0.16;
        // Ground probe straight down onto the cap.
        const double probeOrigin[3] = {0.0, 4.0, 0.0};
        const double probeDirection[3] = {0.0, -1.0, 0.0};
        engine::physics_rigid::RayHit probe{};
        const bool probed = dependencies.rigid->rayCast(
            dependencies.rigid->self, world, probeOrigin, probeDirection, 10.0, &probe);
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const double expectedProbe = 4.0 - (topState.transform.position.y + 0.5);
        const bool resting  = std::abs(baseState.transform.position.y - 0.5) < 0.05
            && std::abs(topState.transform.position.y - 1.52) < 0.15;
        const bool massOk   = std::abs(mass.mass - expectedMass) / expectedMass < 1.0e-9
            && std::abs(mass.inertiaLocal.y - expectedAxial) / expectedAxial < 1.0e-9;
        const bool probeOk  = probed && std::abs(probe.distance - expectedProbe) < 0.02;
        const bool testPassed = resting && massOk && probeOk;
        report << (testPassed ? "PASS" : "FAIL") << ": cylinder shape - the barrel rests at y "
               << baseState.transform.position.y << " with the stacked one at y " << topState.transform.position.y
               << ", the axial inertia matches the closed form, and the cap ray cast hits at " << probe.distance
               << " m (expected " << expectedProbe << ")\n";
        pass = pass && testPassed;
    }

    // ---- 15. trigger volumes report enter and exit --------------------------
    {
        EXTENDED_TEST_MARKER("triggers")
        auto runTriggerRun = [&](std::uint32_t &outEnters, std::uint32_t &outExits, double &outFinalZ)
        {
            engine::physics_rigid::WorldDescription worldDescription;
            const engine::physics_rigid::WorldHandle world
                = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
            engine::physics_rigid::BodyDescription ground;
            ground.shape.kind           = engine::physics_rigid::ShapeKind::Plane;
            ground.shape.planeNormal[1] = 1.0;
            ground.kind                 = engine::physics_rigid::BodyKind::Static;
            ground.material.friction    = 0.8;
            dependencies.rigid->createBody(dependencies.rigid->self, world, &ground, nullptr);
            engine::physics_rigid::TriggerDescription zone;
            zone.shape.kind              = engine::physics_rigid::ShapeKind::Box;
            zone.shape.boxHalfExtents[0] = 0.75;
            zone.shape.boxHalfExtents[1] = 0.75;
            zone.shape.boxHalfExtents[2] = 0.75;
            zone.transform.position      = engine::physics_rigid::Vec3{0.0, 0.75, 0.0};
            engine::physics_rigid::PhysicsError triggerError = engine::physics_rigid::PhysicsError::None;
            const engine::physics_rigid::TriggerHandle trigger
                = dependencies.rigid->createTrigger(dependencies.rigid->self, world, &zone, &triggerError);
            engine::physics_rigid::BodyDescription ball;
            ball.shape.kind             = engine::physics_rigid::ShapeKind::Sphere;
            ball.shape.sphereRadius     = 0.2;
            ball.transform.position     = engine::physics_rigid::Vec3{0.0, 0.4, 5.0};
            ball.kind                   = engine::physics_rigid::BodyKind::Dynamic;
            ball.density                = 500.0;
            ball.material.friction      = 0.05;
            ball.linearDamping          = 0.0;
            const engine::physics_rigid::BodyHandle ballBody
                = dependencies.rigid->createBody(dependencies.rigid->self, world, &ball, nullptr);
            const double launch[3] = {0.0, 0.0, -4.0};
            dependencies.rigid->setBodyLinearVelocity(dependencies.rigid->self, world, ballBody, launch);
            outEnters = 0;
            outExits  = 0;
            for (std::uint32_t step = 0; step < 150; ++step)
            {
                dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
                std::uint32_t pending = 0;
                bool          overflowed = false;
                if (dependencies.rigid->triggerEventCount(dependencies.rigid->self, world, &pending, &overflowed)
                    && pending > 0)
                {
                    std::vector<engine::physics_rigid::TriggerEvent> events(pending);
                    std::uint32_t                                    copied = 0;
                    if (dependencies.rigid->copyTriggerEvents(
                            dependencies.rigid->self, world, 0, events.data(), pending, &copied))
                    {
                        for (const engine::physics_rigid::TriggerEvent &event : events)
                        {
                            if (event.trigger != trigger)
                            {
                                continue;
                            }
                            if (event.kind == engine::physics_rigid::TriggerEventKind::Enter)
                            {
                                ++outEnters;
                            }
                            else
                            {
                                ++outExits;
                            }
                        }
                    }
                    dependencies.rigid->clearTriggerEvents(dependencies.rigid->self, world);
                }
            }
            engine::physics_rigid::BodyState state{};
            dependencies.rigid->copyBodyState(dependencies.rigid->self, world, ballBody, &state);
            outFinalZ = state.transform.position.z;
            dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        };
        std::uint32_t entersA = 0;
        std::uint32_t exitsA  = 0;
        double        finalZA = 0.0;
        std::uint32_t entersB = 0;
        std::uint32_t exitsB  = 0;
        double        finalZB = 0.0;
        runTriggerRun(entersA, exitsA, finalZA);
        runTriggerRun(entersB, exitsB, finalZB);
        const bool transitions = entersA == 1 && exitsA == 1;
        const bool deterministic = entersA == entersB && exitsA == exitsB
            && std::abs(finalZA - finalZB) <= kDeterminismTolerance;
        const bool testPassed = transitions && deterministic;
        report << (testPassed ? "PASS" : "FAIL") << ": trigger volumes - the ball through the zone reports " << entersA
               << " enter and " << exitsA << " exit events, and the repeat run is bit-identical (final z " << finalZA
               << " vs " << finalZB << ")\n";
        pass = pass && testPassed;
    }

    // ---- 16. cloth tear, self-collision, surface sampling -------------------
    if (dependencies.cloth != nullptr)
    {
        EXTENDED_TEST_MARKER("cloth-ext")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_rigid::BodyDescription ground;
        ground.shape.kind           = engine::physics_rigid::ShapeKind::Plane;
        ground.shape.planeNormal[1] = 1.0;
        ground.kind                 = engine::physics_rigid::BodyKind::Static;
        ground.material.friction    = 0.8;
        dependencies.rigid->createBody(dependencies.rigid->self, world, &ground, nullptr);
        // Two pinned top corners, weak middle: the falling sheet strains past
        // the tear tolerance and rips through the middle.
        engine::physics_cloth::ClothDescription torn;
        torn.columns             = 8;
        torn.rows                = 8;
        torn.spacing             = 0.18;
        torn.transform.position  = engine::physics_rigid::Vec3{-0.63, 2.5, 0.0};
        torn.particleMass        = 0.3;
        torn.tearStrain          = 0.22;
        torn.selfCollisionRadius = 0.045;
        std::uint32_t pinned[2]  = {0, 7}; // the top row's two ends
        torn.pinnedParticles     = pinned;
        torn.pinnedCount         = 2;
        engine::physics_cloth::ClothError clothError = engine::physics_cloth::ClothError::None;
        const engine::physics_cloth::ClothHandle tornCloth
            = dependencies.cloth->createCloth(dependencies.cloth->self, world, &torn, &clothError);
        std::uint32_t brokenCount = 0;
        double        centerSample[3] = {0.0, 0.0, 0.0};
        double        centerNormal[3] = {0.0, 0.0, 0.0};
        bool          sampled = false;
        bool          finiteRun = true;
        if (tornCloth != engine::physics_cloth::invalidCloth)
        {
            // Stretch the pinned ends far beyond the span's rest length: the
            // corner constraints strain past the tear tolerance immediately.
            const double leftPin[3]  = {-1.5, 2.5, 0.0};
            const double rightPin[3] = {1.5, 2.5, 0.0};
            dependencies.cloth->pinParticleToPosition(dependencies.cloth->self, tornCloth, 0, leftPin);
            dependencies.cloth->pinParticleToPosition(dependencies.cloth->self, tornCloth, 7, rightPin);
            for (std::uint32_t step = 0; step < 150; ++step)
            {
                if (!dependencies.cloth->step(dependencies.cloth->self, tornCloth, 1.0 / 60.0))
                {
                    finiteRun = false;
                    break;
                }
            }
            engine::physics_cloth::ClothStatistics clothStats{};
            dependencies.cloth->clothStatistics(dependencies.cloth->self, tornCloth, &clothStats);
            brokenCount = clothStats.brokenConstraintCount;
            sampled = dependencies.cloth->sampleClothSurface(
                dependencies.cloth->self, tornCloth, 0.5, 0.5, centerSample, centerNormal);
            for (const double value : centerSample)
            {
                finiteRun = finiteRun && std::isfinite(value);
            }
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool tore = tornCloth != engine::physics_cloth::invalidCloth && brokenCount > 0;
        const bool testPassed = tore && sampled && finiteRun;
        report << (testPassed ? "PASS" : "FAIL") << ": cloth extensions - the strained sheet tears " << brokenCount
               << " constraints, the surface sample stays finite, and the self-collision run never diverges\n";
        pass = pass && testPassed;
    }

    // ---- 17. rope tension, reeling, breaking --------------------------------
    if (dependencies.rope != nullptr)
    {
        EXTENDED_TEST_MARKER("rope-ext")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_rope::RopeDescription hanging;
        hanging.segments    = 5;
        hanging.restLength  = 2.0;
        hanging.nodeMass    = 0.4;
        hanging.start[0]    = 0.0;
        hanging.start[1]    = 3.0;
        hanging.end[1]      = 1.0;
        hanging.pinStart    = true;
        hanging.breakStrain = 0.35;
        engine::physics_rope::RopeError ropeError = engine::physics_rope::RopeError::None;
        const engine::physics_rope::RopeHandle rope
            = dependencies.rope->createRope(dependencies.rope->self, world, &hanging, &ropeError);
        double topTension = 0.0;
        bool   tensionFinite = true;
        bool   reeled = false;
        std::uint32_t brokenSegments = 0;
        if (rope != engine::physics_rope::invalidRope)
        {
            for (std::uint32_t step = 0; step < 120; ++step)
            {
                dependencies.rope->step(dependencies.rope->self, rope, 1.0 / 60.0);
            }
            double tension[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
            std::uint32_t required = 0;
            if (dependencies.rope->copyRopeTension(dependencies.rope->self, rope, tension, 5, &required)
                && required == 5)
            {
                topTension = tension[0];
                for (const double value : tension)
                {
                    tensionFinite = tensionFinite && std::isfinite(value) && value >= 0.0;
                }
            }
            // Reel in to half length; the segment rest lengths follow.
            reeled = dependencies.rope->setRopeLength(dependencies.rope->self, rope, 1.0);
            // Then pin both ends farther apart than the reeled length allows:
            // the whole chain must stretch past the break strain.
            if (reeled)
            {
                const double lowerPin[3] = {0.0, 0.5, 0.0};
                if (dependencies.rope->pinNodeToPosition(dependencies.rope->self, rope, 5, lowerPin))
                {
                    for (std::uint32_t step = 0; step < 90; ++step)
                    {
                        dependencies.rope->step(dependencies.rope->self, rope, 1.0 / 60.0);
                    }
                }
            }
            engine::physics_rope::RopeStatistics ropeStats{};
            dependencies.rope->ropeStatistics(dependencies.rope->self, rope, &ropeStats);
            brokenSegments = ropeStats.brokenSegmentCount;
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool tensionOk = rope != engine::physics_rope::invalidRope && tensionFinite && topTension > 0.0;
        const bool broke = brokenSegments > 0;
        const bool testPassed = tensionOk && reeled && broke;
        report << (testPassed ? "PASS" : "FAIL") << ": rope extensions - the hanging rope reads top tension " << topTension
               << " N, reeling accepts half length, and the over-stretch breaks " << brokenSegments << " segments\n";
        pass = pass && testPassed;
    }

    // ---- 18. water Gerstner displacement, currents, wakes --------------------
    if (dependencies.water != nullptr)
    {
        EXTENDED_TEST_MARKER("water-ext")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_water::OceanDescription oceanDescription;
        oceanDescription.waterLevel          = 0.0;
        oceanDescription.waveCount           = 2;
        oceanDescription.waves[0].directionX = 1.0;
        oceanDescription.waves[0].wavelength = 12.0;
        oceanDescription.waves[0].amplitude  = 0.35;
        oceanDescription.waves[0].steepness  = 0.8;
        oceanDescription.waves[1].directionX = 0.0;
        oceanDescription.waves[1].directionZ = 1.0;
        oceanDescription.waves[1].wavelength = 7.0;
        oceanDescription.waves[1].amplitude  = 0.2;
        oceanDescription.waves[1].steepness  = 0.6;
        oceanDescription.current[0]          = 0.6;
        engine::physics_water::WaterError waterError = engine::physics_water::WaterError::None;
        const engine::physics_water::OceanHandle ocean = dependencies.water->createOcean(
            dependencies.water->self, world, &oceanDescription, &waterError);
        bool surfaceFinite = true;
        bool displacedFinite = false;
        double displacement[3] = {0.0, 0.0, 0.0};
        const double displacedProbe[3] = {1.7, 0.0, -2.3};
        std::uint32_t wakeRipples = 0;
        bool currentDrift = false;
        if (ocean != engine::physics_water::invalidOcean)
        {
            double height = 0.0;
            double normal[3] = {0.0, 1.0, 0.0};
            surfaceFinite = dependencies.water->sampleSurface(
                dependencies.water->self, ocean, 1.7, -2.3, 0.75, &height, normal);
            surfaceFinite = surfaceFinite && std::isfinite(height);
            displacedFinite = dependencies.water->sampleSurfaceDisplaced(
                dependencies.water->self, ocean, displacedProbe, 0.75, displacement);
            for (const double value : displacement)
            {
                displacedFinite = displacedFinite && std::isfinite(value);
            }
            // A fast raft drives straight through the ocean for the wake pass.
            engine::physics_rigid::BodyDescription raft;
            raft.shape.kind             = engine::physics_rigid::ShapeKind::Box;
            raft.shape.boxHalfExtents[0] = 0.5;
            raft.shape.boxHalfExtents[1] = 0.15;
            raft.shape.boxHalfExtents[2] = 0.35;
            raft.transform.position      = engine::physics_rigid::Vec3{-6.0, 0.0, 0.0};
            raft.kind                    = engine::physics_rigid::BodyKind::Dynamic;
            raft.density                 = 220.0;
            raft.linearDamping           = 0.0;
            const engine::physics_rigid::BodyHandle raftBody
                = dependencies.rigid->createBody(dependencies.rigid->self, world, &raft, nullptr);
            dependencies.water->registerBody(dependencies.water->self, ocean, raftBody, nullptr);
            const double drive[3] = {3.0, 0.0, 0.0};
            for (std::uint32_t step = 0; step < 180; ++step)
            {
                const double waterTime = static_cast<double>(step) / 60.0;
                dependencies.rigid->setBodyLinearVelocity(dependencies.rigid->self, world, raftBody, drive);
                dependencies.water->applyBuoyancy(dependencies.water->self, ocean, waterTime, 1.0 / 60.0);
                dependencies.rigid->step(dependencies.rigid->self, world, 1.0 / 60.0);
                dependencies.water->stepRipples(dependencies.water->self, ocean, 1.0 / 60.0);
            }
            engine::physics_water::WaterStatistics waterStats{};
            dependencies.water->waterStatistics(dependencies.water->self, &waterStats);
            wakeRipples = waterStats.activeRipples;
            engine::physics_rigid::BodyState raftState{};
            dependencies.rigid->copyBodyState(dependencies.rigid->self, world, raftBody, &raftState);
            // The current drags the floating raft beyond its driven position.
            currentDrift = raftState.transform.position.x > 0.2;
            dependencies.water->unregisterBody(dependencies.water->self, ocean, raftBody);
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = ocean != engine::physics_water::invalidOcean && surfaceFinite && displacedFinite
            && wakeRipples > 0 && currentDrift;
        report << (testPassed ? "PASS" : "FAIL") << ": water extensions - the steep Gerstner surface samples finite ("
               << surfaceFinite << "), the displaced query stays finite (" << displacedFinite
               << "), the fast raft sheds " << wakeRipples << " wake ripples, and the current carries it ("
               << currentDrift << ")\n";
        pass = pass && testPassed;
    }

    // ---- 19. scheduling determinism, staggering, hysteresis ------------------
    if (dependencies.scheduling != nullptr)
    {
        EXTENDED_TEST_MARKER("scheduling")
        engine::physics_scheduling::ScheduleDescription description;
        engine::physics_scheduling::SchedulingError schedulingError = engine::physics_scheduling::SchedulingError::None;
        const engine::physics_scheduling::ScheduleHandle first
            = dependencies.scheduling->createSchedule(dependencies.scheduling->self, &description, &schedulingError);
        const engine::physics_scheduling::ScheduleHandle second
            = dependencies.scheduling->createSchedule(dependencies.scheduling->self, &description, &schedulingError);
        bool schedulesMatch = first != engine::physics_scheduling::invalidSchedule
            && second != engine::physics_scheduling::invalidSchedule;
        if (schedulesMatch)
        {
            for (std::uint64_t id = 1; id <= 12; ++id)
            {
                engine::physics_scheduling::ItemDescription item;
                item.id             = id;
                item.updateInterval = 4;
                dependencies.scheduling->addItem(dependencies.scheduling->self, first, &item, nullptr);
                dependencies.scheduling->addItem(dependencies.scheduling->self, second, &item, nullptr);
            }
            for (std::uint32_t step = 0; step < 32; ++step)
            {
                dependencies.scheduling->updateSchedule(dependencies.scheduling->self, first);
                dependencies.scheduling->updateSchedule(dependencies.scheduling->self, second);
                std::uint64_t dueIdsA[16] = {};
                std::uint64_t dueIdsB[16] = {};
                std::uint32_t requiredA = 0;
                std::uint32_t requiredB = 0;
                dependencies.scheduling->copyDueItems(dependencies.scheduling->self, first, dueIdsA, 16, &requiredA);
                dependencies.scheduling->copyDueItems(dependencies.scheduling->self, second, dueIdsB, 16, &requiredB);
                if (requiredA != requiredB)
                {
                    schedulesMatch = false;
                    continue;
                }
                for (std::uint32_t index = 0; index < requiredA; ++index)
                {
                    if (dueIdsA[index] != dueIdsB[index])
                    {
                        schedulesMatch = false;
                    }
                }
            }
        }
        // Hysteresis: a score between the release threshold and the entry
        // threshold holds the Full band.
        bool hysteresisHolds = false;
        bool demoteWorks = false;
        if (schedulesMatch)
        {
            const std::uint64_t probe = 7;
            dependencies.scheduling->setItemImportance(dependencies.scheduling->self, first, probe, 0.9);
            dependencies.scheduling->updateSchedule(dependencies.scheduling->self, first);
            engine::physics_scheduling::FidelityLevel level = engine::physics_scheduling::FidelityLevel::Cached;
            dependencies.scheduling->itemLevel(dependencies.scheduling->self, first, probe, &level);
            hysteresisHolds = level == engine::physics_scheduling::FidelityLevel::Full;
            dependencies.scheduling->setItemImportance(dependencies.scheduling->self, first, probe, 0.62);
            dependencies.scheduling->updateSchedule(dependencies.scheduling->self, first);
            dependencies.scheduling->itemLevel(dependencies.scheduling->self, first, probe, &level);
            hysteresisHolds = hysteresisHolds && level == engine::physics_scheduling::FidelityLevel::Full;
            dependencies.scheduling->setItemImportance(dependencies.scheduling->self, first, probe, 0.55);
            dependencies.scheduling->updateSchedule(dependencies.scheduling->self, first);
            dependencies.scheduling->itemLevel(dependencies.scheduling->self, first, probe, &level);
            demoteWorks = level == engine::physics_scheduling::FidelityLevel::Simplified;
        }
        dependencies.scheduling->destroySchedule(dependencies.scheduling->self, first);
        dependencies.scheduling->destroySchedule(dependencies.scheduling->self, second);
        const bool testPassed = schedulesMatch && hysteresisHolds && demoteWorks;
        report << (testPassed ? "PASS" : "FAIL") << ": simulation scheduling - two identically driven schedules emit "
               << "identical due sets, the hysteresis band holds Full at 0.62, and 0.55 demotes\n";
        pass = pass && testPassed;
    }

    // ---- 20. hand planter, lean controller, character push -------------------
    if (dependencies.character != nullptr)
    {
        EXTENDED_TEST_MARKER("character-ext")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_character::HandPlanterDescription handDescription;
        engine::physics_character::CharacterError characterError = engine::physics_character::CharacterError::None;
        const engine::physics_character::HandPlanterHandle hands = dependencies.character->createHandPlanter(
            dependencies.character->self, &handDescription, &characterError);
        engine::physics_character::LeanControllerDescription leanDescription;
        const engine::physics_character::LeanHandle lean = dependencies.character->createLeanController(
            dependencies.character->self, &leanDescription, &characterError);
        engine::physics_rigid::RigidTransform root{};
        root.position = engine::physics_rigid::Vec3{0.0, 1.0, 0.0};
        bool reachClamps = false;
        bool reachHolds = false;
        bool leanTilts = false;
        bool pushMoves = false;
        if (hands != engine::physics_character::invalidHandPlanter)
        {
            const double farTarget[3] = {0.0, 1.2, 3.0};
            const double nearTarget[3] = {0.05, 1.15, 0.45};
            dependencies.character->setHandTarget(dependencies.character->self, hands, 0, farTarget);
            dependencies.character->updateHandPlanter(dependencies.character->self, hands, &root);
            engine::physics_character::HandState states[2];
            std::uint32_t required = 0;
            if (dependencies.character->copyHandStates(dependencies.character->self, hands, states, 2, &required)
                && required == 2)
            {
                reachClamps = !states[0].withinReach && states[0].stretchMeters > 0.5;
            }
            dependencies.character->setHandTarget(dependencies.character->self, hands, 0, nearTarget);
            dependencies.character->updateHandPlanter(dependencies.character->self, hands, &root);
            if (dependencies.character->copyHandStates(dependencies.character->self, hands, states, 2, &required)
                && required == 2)
            {
                reachHolds = states[0].withinReach && std::abs(states[0].stretchMeters) < 1.0e-9;
            }
        }
        if (lean != engine::physics_character::invalidLeanController)
        {
            const double zeroVelocity[3] = {0.0, 0.0, 0.0};
            const double runVelocity[3] = {0.0, 0.0, 4.0};
            engine::physics_character::LeanState warmup{};
            dependencies.character->updateLeanController(
                dependencies.character->self, lean, &root, zeroVelocity, 1.0 / 60.0, &warmup);
            bool finiteLean = true;
            for (std::uint32_t step = 0; step < 30; ++step)
            {
                engine::physics_character::LeanState state{};
                if (!dependencies.character->updateLeanController(
                        dependencies.character->self, lean, &root, runVelocity, 1.0 / 60.0, &state))
                {
                    finiteLean = false;
                    break;
                }
                leanTilts = leanTilts || std::abs(state.leanRadians) > 1.0e-3;
                for (const double value : state.leanRotation)
                {
                    finiteLean = finiteLean && std::isfinite(value);
                }
            }
            leanTilts = leanTilts && finiteLean;
        }
        // Push: a crate sits in front of the character; one push pass shoves it.
        engine::physics_rigid::BodyDescription crate;
        crate.shape.kind             = engine::physics_rigid::ShapeKind::Box;
        crate.shape.boxHalfExtents[0] = 0.25;
        crate.shape.boxHalfExtents[1] = 0.25;
        crate.shape.boxHalfExtents[2] = 0.25;
        crate.transform.position     = engine::physics_rigid::Vec3{0.0, 0.25, 0.6};
        crate.kind                   = engine::physics_rigid::BodyKind::Dynamic;
        crate.density                = 200.0;
        const engine::physics_rigid::BodyHandle crateBody
            = dependencies.rigid->createBody(dependencies.rigid->self, world, &crate, nullptr);
        const double pushCenter[3] = {0.0, 1.0, 0.0};
        const double pushVelocity[3] = {0.0, 0.0, 1.0};
        dependencies.character->applyCharacterPush(dependencies.character->self, world, pushCenter, 2.0, pushVelocity, 60.0);
        engine::physics_rigid::BodyState crateState{};
        dependencies.rigid->copyBodyState(dependencies.rigid->self, world, crateBody, &crateState);
        pushMoves = crateState.linearVelocity.z > 0.5;
        if (hands != engine::physics_character::invalidHandPlanter)
        {
            dependencies.character->destroyHandPlanter(dependencies.character->self, hands);
        }
        if (lean != engine::physics_character::invalidLeanController)
        {
            dependencies.character->destroyLeanController(dependencies.character->self, lean);
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = reachClamps && reachHolds && leanTilts && pushMoves;
        report << (testPassed ? "PASS" : "FAIL") << ": character extensions - far-hand clamp " << reachClamps
               << ", near-hand reach " << reachHolds << ", lean tilt " << leanTilts << ", crate push " << pushMoves << "\n";
        pass = pass && testPassed;
    }

    // ---- 21. batched sphere queries -----------------------------------------
    if (dependencies.rigid != nullptr)
    {
        EXTENDED_TEST_MARKER("batched-queries")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);

        // A mixed field: a ground plane, static boxes, kinematic movers, and
        // dynamic spheres at varied positions so results exercise the
        // deepest-overlap selection and the shape-bound pre-filter.
        engine::physics_rigid::BodyDescription body;
        body.shape.kind    = engine::physics_rigid::ShapeKind::Plane;
        body.shape.planeOffset = 0.0;
        body.kind          = engine::physics_rigid::BodyKind::Static;
        dependencies.rigid->createBody(dependencies.rigid->self, world, &body, nullptr);
        body.shape.kind = engine::physics_rigid::ShapeKind::Box;
        body.shape.planeOffset = 0.0;
        body.shape.boxHalfExtents[0] = 0.6;
        body.shape.boxHalfExtents[1] = 0.6;
        body.shape.boxHalfExtents[2] = 0.6;
        for (std::uint32_t index = 0; index < 6; ++index)
        {
            body.kind              = index % 2 == 0 ? engine::physics_rigid::BodyKind::Static
                                                    : engine::physics_rigid::BodyKind::Kinematic;
            body.transform.position = engine::physics_rigid::Vec3{-2.5 + 1.0 * index, 0.6, 0.0};
            dependencies.rigid->createBody(dependencies.rigid->self, world, &body, nullptr);
        }
        std::vector<engine::physics_rigid::SphereOverlapQuery> queries;
        std::vector<engine::physics_rigid::SphereOverlapResult> single(64);
        for (std::uint32_t index = 0; index < 64; ++index)
        {
            engine::physics_rigid::SphereOverlapQuery query;
            const double x = -3.0 + 0.1 * index;
            query.center[0] = x;
            query.center[1] = index % 3 == 0 ? 0.3 : (index % 3 == 1 ? 0.55 : 1.4);
            query.center[2] = 0.0;
            query.radius    = 0.2 + 0.01 * (index % 4);
            queries.push_back(query);
            double normal[3] = {0.0, 0.0, 0.0};
            double depth = 0.0;
            engine::physics_rigid::BodyHandle bodyHit = engine::physics_rigid::invalidBody;
            single[index].hit = dependencies.rigid->collideSphere(dependencies.rigid->self, world, query.center,
                query.radius, normal, &depth, &bodyHit);
            single[index].normal[0] = normal[0];
            single[index].normal[1] = normal[1];
            single[index].normal[2] = normal[2];
            single[index].depth     = depth;
            single[index].body      = bodyHit;
        }
        std::vector<engine::physics_rigid::SphereOverlapResult> batched(64, engine::physics_rigid::SphereOverlapResult{});
        const std::uint32_t processed = dependencies.rigid->collideSpheres(dependencies.rigid->self, world,
            queries.data(), static_cast<std::uint32_t>(queries.size()), batched.data());
        bool matches = processed == queries.size();
        for (std::size_t index = 0; index < queries.size() && matches; ++index)
        {
            const engine::physics_rigid::SphereOverlapResult &expected = single[index];
            const engine::physics_rigid::SphereOverlapResult &actual   = batched[index];
            matches = expected.hit == actual.hit && expected.body == actual.body
                      && std::abs(expected.depth - actual.depth) <= 1.0e-12
                      && std::abs(expected.normal[0] - actual.normal[0]) <= 1.0e-12
                      && std::abs(expected.normal[1] - actual.normal[1]) <= 1.0e-12
                      && std::abs(expected.normal[2] - actual.normal[2]) <= 1.0e-12;
        }
        // Batched body reads echo the same states the per-body copies give.
        engine::physics_rigid::BodyHandle handles[8] = {};
        const std::uint32_t handleCount = dependencies.rigid->bodyCount(dependencies.rigid->self, world);
        engine::physics_rigid::BodyState statesBatched[8] = {};
        std::uint32_t listed = 0;
        bool batchedStates = false;
        if (handleCount <= 8
            && dependencies.rigid->copyBodyHandles(
                dependencies.rigid->self, world, handles, handleCount, &listed)
            && listed == handleCount)
        {
            const std::uint32_t resolved = dependencies.rigid->copyBodyStates(
                dependencies.rigid->self, world, handles, handleCount, statesBatched);
            batchedStates = resolved == handleCount;
            for (std::uint32_t index = 0; index < handleCount && batchedStates; ++index)
            {
                engine::physics_rigid::BodyState individual{};
                dependencies.rigid->copyBodyState(dependencies.rigid->self, world, handles[index], &individual);
                const bool same = std::abs(individual.transform.position.x - statesBatched[index].transform.position.x)
                                        <= 1.0e-12
                                && std::abs(individual.transform.position.y - statesBatched[index].transform.position.y)
                                        <= 1.0e-12
                                && individual.awake == statesBatched[index].awake;
                batchedStates = same;
            }
            // Batched description reads echo the same shapes the per-body copy
            // gives: compare kind, radii, half extents, and position so a
            // dropped shape kind (everything reading as the default sphere)
            // cannot pass unnoticed.
            if (batchedStates)
            {
                std::vector<engine::physics_rigid::BodyDescription> descriptionsBatched(
                    handleCount, engine::physics_rigid::BodyDescription{});
                const std::uint32_t descriptionsResolved = dependencies.rigid->copyBodyDescriptions(
                    dependencies.rigid->self, world, handles, handleCount, descriptionsBatched.data());
                batchedStates = descriptionsResolved == handleCount;
                for (std::uint32_t index = 0; index < handleCount && batchedStates; ++index)
                {
                    engine::physics_rigid::BodyDescription individual{};
                    batchedStates = dependencies.rigid->copyBodyDescription(
                        dependencies.rigid->self, world, handles[index], &individual);
                    const engine::physics_rigid::BodyDescription &batchedDescription = descriptionsBatched[index];
                    const bool same = individual.kind == batchedDescription.kind
                                      && individual.shape.kind == batchedDescription.shape.kind
                                      && std::abs(individual.shape.sphereRadius - batchedDescription.shape.sphereRadius)
                                             <= 1.0e-12
                                      && std::abs(individual.shape.boxHalfExtents[0]
                                              - batchedDescription.shape.boxHalfExtents[0])
                                             <= 1.0e-12
                                      && std::abs(individual.shape.boxHalfExtents[1]
                                              - batchedDescription.shape.boxHalfExtents[1])
                                             <= 1.0e-12
                                      && std::abs(individual.shape.boxHalfExtents[2]
                                              - batchedDescription.shape.boxHalfExtents[2])
                                             <= 1.0e-12
                                      && std::abs(individual.transform.position.x - batchedDescription.transform.position.x)
                                             <= 1.0e-12
                                      && std::abs(individual.transform.position.y - batchedDescription.transform.position.y)
                                             <= 1.0e-12
                                      && std::abs(individual.transform.position.z - batchedDescription.transform.position.z)
                                             <= 1.0e-12
                                      && individual.shape.kind != engine::physics_rigid::ShapeKind::Sphere;
                    batchedStates = same;
                }
            }
        }
        else
        {
            batchedStates = handleCount > 8; // more bodies than the probe holds is fine
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = matches && batchedStates;
        report << (testPassed ? "PASS" : "FAIL") << ": batched queries - collideSpheres matches collideSphere "
               << matches << ", batched body reads match per-body copies " << batchedStates << "\n";
        pass = pass && testPassed;
    }

    // ---- 22. cloth settle, sleep, and wake ----------------------------------
    if (dependencies.cloth != nullptr)
    {
        EXTENDED_TEST_MARKER("cloth-sleep")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_cloth::ClothDescription clothDescription;
        clothDescription.columns = 8;
        clothDescription.rows    = 8;
        clothDescription.spacing = 0.2;
        // A heavy-curtain damping so the pinned sheet settles well inside
        // the test's step budget (the sleep thresholds are module config).
        clothDescription.damping = 0.6;
        clothDescription.transform.position = engine::physics_rigid::Vec3{-0.7, 1.2, 0.0};
        std::uint32_t pinned[8];
        for (std::uint32_t column = 0; column < 8; ++column)
        {
            pinned[column] = column;
        }
        clothDescription.pinnedParticles = pinned;
        clothDescription.pinnedCount     = 8;
        const engine::physics_cloth::ClothHandle cloth
            = dependencies.cloth->createCloth(dependencies.cloth->self, world, &clothDescription, nullptr);
        bool slept = false;
        bool heldWhileQuiet = true;
        bool wokeOnWind = false;
        const double zeroWind[3] = {0.0, 0.0, 0.0};
        const double gustWind[3] = {2.5, 0.0, 0.0};
        if (cloth != engine::physics_cloth::invalidCloth)
        {
            // The pinned sheet swings down and settles over a few seconds
            // of damping; the sleep velocity threshold is module config.
            for (std::uint32_t step = 0; step < 1200 && !slept; ++step)
            {
                dependencies.cloth->step(dependencies.cloth->self, cloth, 1.0 / 60.0);
                engine::physics_cloth::ClothStatistics statistics{};
                if (dependencies.cloth->clothStatistics(dependencies.cloth->self, cloth, &statistics))
                {
                    slept = statistics.sleeping;
                }
            }
            for (std::uint32_t step = 0; step < 30 && slept; ++step)
            {
                dependencies.cloth->step(dependencies.cloth->self, cloth, 1.0 / 60.0);
                engine::physics_cloth::ClothStatistics statistics{};
                if (dependencies.cloth->clothStatistics(dependencies.cloth->self, cloth, &statistics))
                {
                    heldWhileQuiet = heldWhileQuiet && statistics.sleeping;
                }
            }
            dependencies.cloth->setWindAcceleration(dependencies.cloth->self, cloth, gustWind);
            dependencies.cloth->step(dependencies.cloth->self, cloth, 1.0 / 60.0);
            dependencies.cloth->setWindAcceleration(dependencies.cloth->self, cloth, zeroWind);
            for (std::uint32_t step = 0; step < 10; ++step)
            {
                dependencies.cloth->step(dependencies.cloth->self, cloth, 1.0 / 60.0);
            }
            engine::physics_cloth::ClothStatistics statistics{};
            if (dependencies.cloth->clothStatistics(dependencies.cloth->self, cloth, &statistics))
            {
                wokeOnWind = !statistics.sleeping;
            }
            dependencies.cloth->destroyCloth(dependencies.cloth->self, cloth);
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = slept && heldWhileQuiet && wokeOnWind;
        report << (testPassed ? "PASS" : "FAIL") << ": cloth sleep - settles to sleep " << slept
               << ", holds while quiet " << heldWhileQuiet << ", wakes on wind " << wokeOnWind << "\n";
        pass = pass && testPassed;
    }

    // ---- 23. particle simulation LOD -----------------------------------------
    if (dependencies.particles != nullptr)
    {
        EXTENDED_TEST_MARKER("particle-lod")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        engine::physics_particles::EmitterDescription emitterDescription;
        emitterDescription.kind            = engine::physics_particles::EmitterKind::Snow;
        emitterDescription.maximumParticles = 512;
        emitterDescription.emitRate        = 120.0;
        const engine::physics_particles::EmitterHandle emitter = dependencies.particles->createEmitter(
            dependencies.particles->self, world, &emitterDescription, nullptr);
        bool staggered = false;
        bool rateHolds = false;
        bool frozen = false;
        if (emitter != engine::physics_particles::invalidEmitter)
        {
            // Simplified band (importance 0.55 crosses out of Full through
            // the hysteresis margin): about half the steps skip.
            dependencies.particles->setEmitterImportance(dependencies.particles->self, emitter, 0.55);
            const std::uint64_t skippedBefore = [&]
            {
                engine::physics_particles::ParticlesStatistics statistics{};
                dependencies.particles->particlesStatistics(dependencies.particles->self, &statistics);
                return statistics.lodSkippedSteps;
            }();
            for (std::uint32_t step = 0; step < 100; ++step)
            {
                dependencies.particles->step(dependencies.particles->self, emitter, 1.0 / 60.0);
            }
            engine::physics_particles::ParticlesStatistics statistics{};
            dependencies.particles->particlesStatistics(dependencies.particles->self, &statistics);
            const std::uint64_t skippedDuring = statistics.lodSkippedSteps - skippedBefore;
            staggered = skippedDuring >= 44 && skippedDuring <= 56;
            // Emission rate stays wall-time correct across the bands: about
            // 100 steps at 60 Hz is 1.67 s, so roughly 200 snowflakes.
            rateHolds = statistics.spawnedTotal >= 160 && statistics.spawnedTotal <= 240;

            // Cached band (importance 0.05 with the default thresholds):
            // every step skips.
            dependencies.particles->setEmitterImportance(dependencies.particles->self, emitter, 0.05);
            const std::uint64_t frozenBefore = statistics.lodSkippedSteps;
            for (std::uint32_t step = 0; step < 10; ++step)
            {
                dependencies.particles->step(dependencies.particles->self, emitter, 1.0 / 60.0);
            }
            engine::physics_particles::ParticlesStatistics frozenStatistics{};
            dependencies.particles->particlesStatistics(dependencies.particles->self, &frozenStatistics);
            frozen = frozenStatistics.lodSkippedSteps - frozenBefore == 10u;

            dependencies.particles->destroyEmitter(dependencies.particles->self, emitter);
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = staggered && rateHolds && frozen;
        report << (testPassed ? "PASS" : "FAIL") << ": particle LOD - staggered cadence " << staggered
               << ", emission rate holds " << rateHolds << ", cached freezes " << frozen << "\n";
        pass = pass && testPassed;
    }

    // ---- 24. per-ocean gravity ------------------------------------------------
    if (dependencies.water != nullptr)
    {
        EXTENDED_TEST_MARKER("ocean-gravity")
        engine::physics_rigid::WorldDescription worldDescriptionA;
        const engine::physics_rigid::WorldHandle worldA
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescriptionA, nullptr);
        engine::physics_rigid::WorldDescription worldDescriptionB;
        const double heavyGravity[3] = {0.0, -30.0, 0.0};
        worldDescriptionB.gravity = engine::physics_rigid::Vec3{heavyGravity[0], heavyGravity[1], heavyGravity[2]};
        const engine::physics_rigid::WorldHandle worldB
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescriptionB, nullptr);
        engine::physics_water::OceanHandle oceanA = engine::physics_water::invalidOcean;
        engine::physics_water::OceanHandle oceanB = engine::physics_water::invalidOcean;
        engine::physics_rigid::BodyHandle bodyA = engine::physics_rigid::invalidBody;
        engine::physics_rigid::BodyHandle bodyB = engine::physics_rigid::invalidBody;
        const auto buildFloatScene = [&](engine::physics_rigid::WorldHandle world, engine::physics_water::OceanHandle &outOcean,
            engine::physics_rigid::BodyHandle &outBody)
        {
            engine::physics_water::OceanDescription oceanDescription;
            outOcean = dependencies.water->createOcean(dependencies.water->self, world, &oceanDescription, nullptr);
            engine::physics_rigid::BodyDescription body;
            body.shape.kind         = engine::physics_rigid::ShapeKind::Sphere;
            body.shape.sphereRadius = 0.4;
            body.transform.position = engine::physics_rigid::Vec3{0.0, -1.0, 0.0};
            body.density            = 400.0;
            outBody = dependencies.rigid->createBody(dependencies.rigid->self, world, &body, nullptr);
            if (outOcean == engine::physics_water::invalidOcean || outBody == engine::physics_rigid::invalidBody)
            {
                return false;
            }
            return dependencies.water->registerBody(dependencies.water->self, outOcean, outBody, nullptr);
        };
        const bool sceneA = buildFloatScene(worldA, oceanA, bodyA);
        const bool sceneB = buildFloatScene(worldB, oceanB, bodyB);
        // The heavy-gravity world's ocean pushes 30/9.81 times harder, so
        // the submerged sphere rises faster under identical bodies.
        auto riseOf = [&](engine::physics_rigid::WorldHandle world, engine::physics_rigid::BodyHandle body)
        {
            engine::physics_rigid::BodyState state{};
            if (dependencies.rigid->copyBodyState(dependencies.rigid->self, world, body, &state))
            {
                return state.transform.position.y;
            }
            return -100.0;
        };
        // A short fully-submerged window: both spheres start a radius below
        // the surface, and the heavy-gravity ocean pushes proportionally
        // harder, so B climbs higher while neither has broken the surface.
        for (std::uint32_t step = 0; step < 12 && sceneA && sceneB; ++step)
        {
            const double delta = 1.0 / 60.0;
            dependencies.water->applyBuoyancy(dependencies.water->self, oceanA, step * delta, delta);
            dependencies.rigid->step(dependencies.rigid->self, worldA, delta);
            dependencies.water->applyBuoyancy(dependencies.water->self, oceanB, step * delta, delta);
            dependencies.rigid->step(dependencies.rigid->self, worldB, delta);
        }
        const double riseA = sceneA ? riseOf(worldA, bodyA) : -100.0;
        const double riseB = sceneB ? riseOf(worldB, bodyB) : -100.0;
        dependencies.rigid->destroyWorld(dependencies.rigid->self, worldA);
        dependencies.rigid->destroyWorld(dependencies.rigid->self, worldB);
        const bool testPassed = sceneA && sceneB && riseB > riseA;
        report << (testPassed ? "PASS" : "FAIL") << ": per-ocean gravity - heavy world rises faster (A " << riseA
               << ", B " << riseB << ")\n";
        pass = pass && testPassed;
    }

    // ---- 25. rigid simulation LOD ---------------------------------------------
    if (dependencies.rigid != nullptr)
    {
        EXTENDED_TEST_MARKER("rigid-lod")
        const auto buildStackWorld = [&](std::vector<engine::physics_rigid::BodyHandle> &outBodies)
        {
            engine::physics_rigid::WorldDescription worldDescription;
            const engine::physics_rigid::WorldHandle world
                = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
            engine::physics_rigid::BodyDescription ground;
            ground.shape.kind          = engine::physics_rigid::ShapeKind::Plane;
            ground.shape.planeNormal[0] = 0.0;
            ground.shape.planeNormal[1] = 1.0;
            ground.shape.planeNormal[2] = 0.0;
            ground.kind               = engine::physics_rigid::BodyKind::Static;
            ground.material.friction  = 0.8;
            dependencies.rigid->createBody(dependencies.rigid->self, world, &ground, nullptr);
            for (std::uint32_t index = 0; index < 5; ++index)
            {
                engine::physics_rigid::BodyDescription box;
                box.shape.kind              = engine::physics_rigid::ShapeKind::Box;
                box.shape.boxHalfExtents[0] = 0.5;
                box.shape.boxHalfExtents[1] = 0.5;
                box.shape.boxHalfExtents[2] = 0.5;
                box.transform.position      = engine::physics_rigid::Vec3{0.0, 0.5 + index, 0.0};
                box.kind                    = engine::physics_rigid::BodyKind::Dynamic;
                box.density                 = 500.0;
                outBodies.push_back(dependencies.rigid->createBody(dependencies.rigid->self, world, &box, nullptr));
            }
            return world;
        };

        std::vector<engine::physics_rigid::BodyHandle> stackA;
        std::vector<engine::physics_rigid::BodyHandle> stackB;
        const engine::physics_rigid::WorldHandle worldA = buildStackWorld(stackA);
        const engine::physics_rigid::WorldHandle worldB = buildStackWorld(stackB);
        bool fullCountsZero = false;
        bool reducedCounts  = false;
        bool stacksRest = false;
        bool twinsMatch = false;
        if (worldA != engine::physics_rigid::invalidWorld && worldB != engine::physics_rigid::invalidWorld
            && stackA.size() == 5 && stackB.size() == 5)
        {
            // A: every body keeps the default importance 1.0 (the Full
            // band); 120 steps settle the stack.
            for (std::uint32_t step = 0; step < 120; ++step)
            {
                dependencies.rigid->step(dependencies.rigid->self, worldA, 1.0 / 60.0);
            }
            engine::physics_rigid::WorldStatistics statisticsA{};
            dependencies.rigid->worldStatistics(dependencies.rigid->self, worldA, &statisticsA);
            fullCountsZero = statisticsA.lodReducedBodyCount == 0u;

            // B: the same stack with the top three boxes scored down to the
            // Cached band; the budget follows the score while the pile
            // keeps resting on the ground. The A twin gets the same scores
            // so the worlds stay comparable.
            for (std::uint32_t index = 0; index < 5; ++index)
            {
                const double importance = index < 2 ? 1.0 : 0.05;
                dependencies.rigid->setBodyImportance(dependencies.rigid->self, worldB, stackB[index], importance);
                dependencies.rigid->setBodyImportance(dependencies.rigid->self, worldA, stackA[index], importance);
            }
            for (std::uint32_t step = 0; step < 120; ++step)
            {
                dependencies.rigid->step(dependencies.rigid->self, worldA, 1.0 / 60.0);
                dependencies.rigid->step(dependencies.rigid->self, worldB, 1.0 / 60.0);
            }
            engine::physics_rigid::WorldStatistics statisticsB{};
            dependencies.rigid->worldStatistics(dependencies.rigid->self, worldB, &statisticsB);
            reducedCounts = statisticsB.lodReducedBodyCount >= 1u && statisticsB.lodReducedBodyCount <= 3u;

            engine::physics_rigid::BodyState topA{};
            engine::physics_rigid::BodyState topB{};
            dependencies.rigid->copyBodyState(dependencies.rigid->self, worldA, stackA.back(), &topA);
            dependencies.rigid->copyBodyState(dependencies.rigid->self, worldB, stackB.back(), &topB);
            // The reduced pile rests (no explosion, still on the stack):
            // the top box sits within half a meter of its full-rate twin.
            stacksRest = topB.transform.position.y > 0.5
                && topB.transform.position.y < topA.transform.position.y + 0.5
                && std::isfinite(topB.transform.position.y);

            // Determinism with LOD scores: two fresh worlds scored
            // identically from creation and stepped in lockstep must finish
            // bit-identical - the budgets follow the scores deterministically.
            std::vector<engine::physics_rigid::BodyHandle> twinStacks[2];
            const engine::physics_rigid::WorldHandle twinWorlds[2] = {
                buildStackWorld(twinStacks[0]),
                buildStackWorld(twinStacks[1]),
            };
            for (std::uint32_t twin = 0; twin < 2; ++twin)
            {
                for (std::uint32_t index = 0; index < 5; ++index)
                {
                    const double importance = index < 2 ? 1.0 : 0.05;
                    dependencies.rigid->setBodyImportance(
                        dependencies.rigid->self, twinWorlds[twin], twinStacks[twin][index], importance);
                }
            }
            for (std::uint32_t step = 0; step < 240; ++step)
            {
                dependencies.rigid->step(dependencies.rigid->self, twinWorlds[0], 1.0 / 60.0);
                dependencies.rigid->step(dependencies.rigid->self, twinWorlds[1], 1.0 / 60.0);
            }
            twinsMatch = true;
            for (std::size_t index = 0; index < twinStacks[0].size(); ++index)
            {
                engine::physics_rigid::BodyState stateA{};
                engine::physics_rigid::BodyState stateB{};
                dependencies.rigid->copyBodyState(dependencies.rigid->self, twinWorlds[0], twinStacks[0][index], &stateA);
                dependencies.rigid->copyBodyState(dependencies.rigid->self, twinWorlds[1], twinStacks[1][index], &stateB);
                if (std::abs(stateA.transform.position.x - stateB.transform.position.x) > kDeterminismTolerance
                    || std::abs(stateA.transform.position.y - stateB.transform.position.y) > kDeterminismTolerance
                    || std::abs(stateA.transform.position.z - stateB.transform.position.z) > kDeterminismTolerance)
                {
                    twinsMatch = false;
                    break;
                }
            }
            dependencies.rigid->destroyWorld(dependencies.rigid->self, twinWorlds[0]);
            dependencies.rigid->destroyWorld(dependencies.rigid->self, twinWorlds[1]);
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, worldA);
        dependencies.rigid->destroyWorld(dependencies.rigid->self, worldB);
        const bool testPassed = fullCountsZero && reducedCounts && stacksRest && twinsMatch;
        report << (testPassed ? "PASS" : "FAIL") << ": rigid LOD - full counts zero " << fullCountsZero
               << ", reduced counts " << reducedCounts << ", stack rests " << stacksRest << ", twins match "
               << twinsMatch << "\n";
        pass = pass && testPassed;
    }

    // ---- 26. articulated simulation LOD ---------------------------------------
    if (dependencies.articulation != nullptr)
    {
        EXTENDED_TEST_MARKER("articulated-lod")
        engine::physics_rigid::WorldDescription worldDescription;
        const engine::physics_rigid::WorldHandle world
            = dependencies.rigid->createWorld(dependencies.rigid->self, &worldDescription, nullptr);
        bool skipsCounted = false;
        if (world != engine::physics_rigid::invalidWorld)
        {
            // A minimal parent-child hinge chain: a static anchor box with
            // a driven box hanging below it.
            engine::physics_articulated::LinkDescription links[2];
            links[0].parentIndex = -1;
            links[0].shape.kind  = engine::physics_rigid::ShapeKind::Box;
            links[0].shape.boxHalfExtents[0] = 0.15;
            links[0].shape.boxHalfExtents[1] = 0.15;
            links[0].shape.boxHalfExtents[2] = 0.15;
            links[0].density    = 0.0;
            links[0].mass       = 20.0;
            links[1].parentIndex = 0;
            links[1].shape.kind  = engine::physics_rigid::ShapeKind::Box;
            links[1].shape.boxHalfExtents[0] = 0.2;
            links[1].shape.boxHalfExtents[1] = 0.2;
            links[1].shape.boxHalfExtents[2] = 0.2;
            links[1].density                   = 500.0;
            links[1].relativeTransform.position = engine::physics_rigid::Vec3{0.0, -0.7, 0.0};
            links[1].jointAnchorLocalChild     = engine::physics_rigid::Vec3{0.0, 0.35, 0.0};
            links[1].jointAnchorLocalParent    = engine::physics_rigid::Vec3{0.0, -0.35, 0.0};
            engine::physics_articulated::FigureDescription description;
            description.links       = links;
            description.linkCount   = 2;
            description.rootDynamic = false;
            const engine::physics_articulated::FigureHandle figure = dependencies.articulation->createFigure(
                dependencies.articulation->self, world, &description, nullptr);
            if (figure != engine::physics_articulated::invalidFigure)
            {
                // Cached importance: every drive refresh skips and the
                // counter reports it.
                dependencies.articulation->setFigureImportance(dependencies.articulation->self, figure, 0.05);
                for (std::uint32_t step = 0; step < 10; ++step)
                {
                    dependencies.articulation->updateDrives(dependencies.articulation->self, figure, 1.0 / 60.0);
                }
                engine::physics_articulated::ArticulatedFigureStatistics statistics{};
                dependencies.articulation->figureStatistics(dependencies.articulation->self, figure, &statistics);
                skipsCounted = statistics.lodSkippedSteps >= 10u;
                dependencies.articulation->destroyFigure(dependencies.articulation->self, figure);
            }
        }
        dependencies.rigid->destroyWorld(dependencies.rigid->self, world);
        const bool testPassed = skipsCounted;
        report << (testPassed ? "PASS" : "FAIL") << ": articulated LOD - cached drive refreshes skip " << skipsCounted
               << "\n";
        pass = pass && testPassed;
    }

    // ---- 27. character simulation LOD -----------------------------------------
    if (dependencies.character != nullptr)
    {
        EXTENDED_TEST_MARKER("character-lod")
        SceneHandles handles;
        buildExtendedTestScene(dependencies, config, "character", handles);
        bool skipsCounted = false;
        if (handles.extended.character != engine::physics_character::invalidCharacter)
        {
            engine::physics_character::CharacterStatistics before{};
            dependencies.character->characterStatistics(dependencies.character->self, &before);
            // Cached importance: the character stops driving and every
            // update skips.
            dependencies.character->setCharacterImportance(
                dependencies.character->self, handles.extended.character, 0.05);
            for (std::uint32_t step = 0; step < 10; ++step)
            {
                dependencies.character->updateCharacter(
                    dependencies.character->self, handles.extended.character, 1.0 / 60.0);
            }
            engine::physics_character::CharacterStatistics after{};
            dependencies.character->characterStatistics(dependencies.character->self, &after);
            skipsCounted = after.lodSkippedSteps - before.lodSkippedSteps >= 10u;
        }
        destroyScene(dependencies, handles);
        const bool testPassed = skipsCounted;
        report << (testPassed ? "PASS" : "FAIL") << ": character LOD - cached updates skip " << skipsCounted << "\n";
        pass = pass && testPassed;
    }

    result.pass   = pass;
    result.report = report.str();
    return result;
}

} // namespace demo::physics_sandbox
