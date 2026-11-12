// Benchmark environment, imported model, and object-grid resource ownership.
#ifndef ENGINE_DEMO_BENCHMARK_SCENE_HPP
#define ENGINE_DEMO_BENCHMARK_SCENE_HPP

#include <engine/models/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <filesystem>
#include <vector>

namespace demo
{
struct StaticBatch
{
    engine::renderer::MeshHandle mesh = engine::renderer::invalidMesh;
    std::uint32_t modelType = 0; // 0 = rock, 1 = torus; selects the material.
};

struct DynamicObject
{
    std::uint32_t modelType = 0; // 0 = rock, 1 = torus.
    glm::vec3 basePosition{0.0f};
    float scale = 1.0f;
    float phase = 0.0f;
    float angularSpeed = 1.0f;
    glm::mat4 transform{1.0f};
};

struct BenchmarkScene
{
    engine::renderer::TextureHandle floorAlbedo = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle floorNormal = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle floorMetalRough = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle floorOcclusion = engine::renderer::invalidTexture;

    engine::renderer::MeshHandle floorMesh = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle skyDomeMesh = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle sunDiscMesh = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle rockMesh = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle torusMesh = engine::renderer::invalidMesh;
    std::vector<engine::renderer::MeshHandle> dragonMeshes;

    std::vector<StaticBatch> staticBatches;
    std::vector<DynamicObject> dynamicObjects;
    glm::vec3 sunDiscRadiance{1.0f, 0.9f, 0.7f};
};

void createBenchmarkScene(BenchmarkScene &scene, engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, engine::models::ModelsCapability &modelsCapability,
    engine::window::WindowCapability &windowCapability, const std::filesystem::path &assetRoot,
    bool characterView, const engine::pbr::PbrEnvironment *pbrEnvironment);

void destroyBenchmarkScene(BenchmarkScene &scene, engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle);
} // namespace demo

#endif
