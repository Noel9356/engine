// Private data and helper API for the Lumen GI sample application.
#ifndef ENGINE_DEMO_GI_DEMO_SUPPORT_HPP
#define ENGINE_DEMO_GI_DEMO_SUPPORT_HPP

#include <engine/lumen_gi/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>

#include <cstdint>
#include <string>
#include <vector>
#include <glm/glm.hpp>

namespace demo::lumen_gi
{
struct GiDemoOptions
{
    std::string   viewName         = "interior";
    std::uint64_t frames           = 0;
    std::string   screenshotPath;
    std::uint64_t screenshotFrame  = 29;
    std::string   exportDebugPrefix;
    std::uint32_t statsEveryFrames = 30;
    bool          orbit            = false;
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
    std::vector<SceneVertex> vertices;
    std::vector<std::uint32_t> indices;
    engine::renderer::MeshHandle rendererMesh = engine::renderer::invalidMesh;
    engine::lumen_gi::GiSurfaceHandle giSurface = engine::lumen_gi::invalidGiSurface;
    engine::renderer::MeshMaterial material{};
    float giAlbedo[3] = {1.0f, 1.0f, 1.0f};
    float giEmissive[3] = {0.0f, 0.0f, 0.0f};
    float giEmissiveIntensity = 1.0f;
    std::uint32_t giFlagBits = 0;
};

const char *giOptionsUsage() noexcept;
bool parseGiOptions(const std::vector<std::string> &arguments, GiDemoOptions &options, std::string &outError);
std::vector<SceneMesh> buildInteriorScene();
std::vector<SceneMesh> buildOutdoorScene();
} // namespace demo::lumen_gi

#endif
