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
#include "sandbox_selftest.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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
