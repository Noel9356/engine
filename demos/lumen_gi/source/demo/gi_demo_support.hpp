// Private data and helper API for the Lumen GI sample application.
#pragma once

#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace demo::lumen_gi
{
struct GiDemoOptions
{
    std::string   viewName         = "interior";
    std::uint64_t frames           = 0;
    std::string   screenshotPath;
    std::uint64_t screenshotFrame  = 29;
    std::string   frameTimesPath;
    std::string   exportDebugPrefix;
    std::uint32_t statsEveryFrames = 30;
    bool          orbit            = false;
    // --spin: rotate one scene object around its own center every frame
    // (renderer, lumen_gi, and lumen_reflection all receive the same
    // per-frame model matrix), exercising the dynamic-surface path without
    // manual input.
    bool          spin             = false;
    // --no-reflections: skip the lumen_reflection module (no surfaces
    // registered, no evaluate) so the same scene renders without the
    // indirect specular term for A/B comparison. The module itself stays
    // loaded and its absence is silent.
    bool          reflections      = true;
    // --self-test: short deterministic headless run (interior view, 24
    // frames) followed by module-level checks: GI and reflection evaluates
    // ran, the world radiance cache and the reflection field published
    // snapshots, the transparent scene props produced transmission segments,
    // and the exported screenshot carries non-uniform content. Exits 0 on
    // pass, 1 on failure.
    bool          selfTest         = false;
};

struct SceneVertex
{
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec3 color{1.0f};
    glm::vec2 uv{0.0f};
};

struct SceneMesh
{
    // Stable identifier for demo-side object selection (--spin).
    std::string name;
    std::vector<SceneVertex> vertices;
    std::vector<std::uint32_t> indices;
    engine::renderer::MeshHandle rendererMesh = engine::renderer::invalidMesh;
    engine::lumen_gi::GiSurfaceHandle giSurface = engine::lumen_gi::invalidGiSurface;
    engine::lumen_reflection::ReflectionSurfaceHandle reflectionSurface
        = engine::lumen_reflection::invalidReflectionSurface;
    engine::renderer::MeshMaterial material{};
    float giAlbedo[3] = {1.0f, 1.0f, 1.0f};
    float giEmissive[3] = {0.0f, 0.0f, 0.0f};
    float giEmissiveIntensity = 1.0f;
    std::uint32_t giFlagBits = 0;
    // PBR specular parameters the lumen_reflection module traces with (the
    // renderer material carries the same values).
    float rfRoughness = 1.0f;
    float rfMetallic  = 0.0f;
};

const char *giOptionsUsage() noexcept;
bool parseGiOptions(const std::vector<std::string> &arguments, GiDemoOptions &options, std::string &outError);
std::vector<SceneMesh> buildInteriorScene();
std::vector<SceneMesh> buildOutdoorScene();
std::vector<SceneMesh> buildGalleryScene();
} // namespace demo::lumen_gi

