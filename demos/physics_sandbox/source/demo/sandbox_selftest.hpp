// sandbox_selftest.hpp - the demo's determinism and frame-rate independence
// test battery. Tests build their own worlds through the physics
// capabilities and need no renderer.
#pragma once

#include "sandbox_scene.hpp"

#include <string>

namespace demo::physics_sandbox
{

struct SelfTestResult
{
    bool        pass = false;
    std::string report;
};

SelfTestResult runSelfTest(const SceneDependencies &dependencies, const SceneConfig &config);

// The extended-modules battery (rope, fluid, particles, foliage, water,
// fields-driven CCD and rolling resistance, joints, character). Called after
// the core battery when the extended capabilities are available; each test
// guards on its own capability, so a partial module set reports fewer lines.
SelfTestResult runSelfTestExtended(const SceneDependencies &dependencies, const SceneConfig &config);

} // namespace demo::physics_sandbox
