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

} // namespace demo::physics_sandbox
