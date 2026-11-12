// Benchmark environment, model, and instanced-grid resources for the showcase demo.
#include "benchmark_scene.hpp"

#include "diagnostics.hpp"
#include "geometry.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace demo
{
namespace renderer = engine::renderer;
namespace models = engine::models;

namespace
{
constexpr float roomHalfExtent = 36.0f;
constexpr float skyDomeRadius = 1990.0f;
constexpr float sunDistance = 1940.0f;
constexpr float sunRadius = 50.0f;
}

void createBenchmarkScene(BenchmarkScene &scene, renderer::RenderCapability &renderCapability,
    renderer::RendererHandle rendererHandle, models::ModelsCapability &modelsCapability,
    engine::window::WindowCapability &windowCapability, const std::filesystem::path &assetRoot,
    bool characterView, const engine::pbr::PbrEnvironment *pbrEnvironment)
{
    const auto assetPath = [&assetRoot](const char *relativeName) { return (assetRoot / relativeName).string(); };
    const auto pumpWindowEvents = [&windowCapability]() { windowCapability.pollEvents(windowCapability.self); };
    renderer::RenderError renderError = renderer::RenderError::None;

    // Scene setup can upload substantial mesh/texture data; keep the window responsive.
    // Load every floor texture up front; failures log and yield invalidTexture,
    // which the renderer treats as "no texture". The character screen builds
    // its own untextured stage, so this is benchmark-only.
    if (!characterView)
    {
        std::cout << "demo: loading floor textures (albedo, normal, metal-roughness, occlusion)...\n";
        auto loadTexture = [&](const char *path)
        {
            const renderer::TextureHandle handle
                = renderCapability.createTextureFromFile(renderCapability.self, rendererHandle, path, &renderError);
            if (handle == renderer::invalidTexture)
            {
                std::cerr << "demo: texture '" << path << "' failed (" << renderErrorName(renderError) << ")\n";
            }
            return handle;
        };
        scene.floorAlbedo = loadTexture(assetPath("textures/floor_tiles.png").c_str());
        pumpWindowEvents();
        scene.floorNormal = loadTexture(assetPath("textures/floor_tiles_normal.png").c_str());
        pumpWindowEvents();
        scene.floorMetalRough = loadTexture(assetPath("textures/floor_tiles_mr.png").c_str());
        pumpWindowEvents();
        scene.floorOcclusion = loadTexture(assetPath("textures/floor_tiles_ao.png").c_str());
        pumpWindowEvents();
        std::cout << "demo: floor textures loaded\n";
    }

    glm::vec3 sunDiscColor(1.0f, 0.9f, 0.7f);
    glm::vec3 sunDirection(0.45f, 0.62f, 0.35f);
    float     sunIntensity = 3.0f;
    if (pbrEnvironment != nullptr)
    {
        sunDirection = glm::make_vec3(pbrEnvironment->sun.direction);
        sunIntensity = pbrEnvironment->sun.intensity;
        sunDiscColor = glm::make_vec3(pbrEnvironment->sun.color);
    }
    sunDirection = glm::normalize(sunDirection);
    // The sun disc is unlit; boost its base color so it reads as emissive.
    scene.sunDiscRadiance = sunDiscColor * (sunIntensity * 1.5f + 0.6f);

    // Uploads CPU mesh data via the renderer capability; failures log and leave
    // the outHandle invalid, which every draw path skips.
    auto uploadMesh = [&](const MeshData &mesh, renderer::MeshHandle &outHandle)
    {
        const renderer::MeshDescription description = mesh.description();
        outHandle = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &renderError);
        if (outHandle == renderer::invalidMesh)
        {
            std::cerr << "demo: renderer module rejected a scene mesh (" << renderErrorName(renderError) << ")\n";
        }
    };

    // Inigo model + benchmark environment: only the benchmark views load
    // them; the character screen builds its own stage and body parts.
    engine::models::ModelStatistics   dragonStatistics{};
    if (!characterView)
    {
        engine::models::ModelLoadOptions dragonLoadOptions{
            .normalMode         = engine::models::ModelNormalMode::GenerateSmooth,
            .importVertexColors = true,
            .weldVertices       = true,
            .importMaterials    = false};
        engine::models::ModelLoadError dragonLoadError = engine::models::ModelLoadError::None;
        const std::string              dragonPath      = assetPath("models/inigo_the_dragon/dragon_meshed.obj");
        std::cout << "demo: importing the Inigo model (weld + smooth normals)...\n";
        const engine::models::ModelHandle dragonModel = modelsCapability.loadModelWithOptions(
            modelsCapability.self, dragonPath.c_str(), &dragonLoadOptions, &dragonLoadError);
        // One GPU mesh per imported part; parts that fail to upload are skipped.
        if (dragonModel == engine::models::invalidModel)
        {
            std::cerr << "demo: Inigo model import failed (" << static_cast<std::uint32_t>(dragonLoadError) << ")\n";
        }
        else
        {
            (void)modelsCapability.statistics(modelsCapability.self, dragonModel, &dragonStatistics);
            const std::uint32_t dragonPartCount = modelsCapability.partCount(modelsCapability.self, dragonModel);
            scene.dragonMeshes.reserve(dragonPartCount);
            for (std::uint32_t partIndex = 0; partIndex < dragonPartCount; ++partIndex)
            {
                renderer::MeshDescription description;
                if (!modelsCapability.part(modelsCapability.self, dragonModel, partIndex, &description))
                {
                    continue;
                }
                const renderer::MeshHandle handle
                    = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &renderError);
                pumpWindowEvents();
                if (handle == renderer::invalidMesh)
                {
                    std::cerr << "demo: Inigo mesh upload failed (" << renderErrorName(renderError) << ")\n";
                    continue;
                }
                scene.dragonMeshes.push_back(handle);
            }
            modelsCapability.destroyPluginel(modelsCapability.self, dragonModel);
            std::cout << "demo: loaded Inigo model " << dragonStatistics.vertexCount << " vertices, "
                      << dragonStatistics.indexCount / 3 << " triangles, " << scene.dragonMeshes.size()
                      << " GPU mesh parts\n";
        }

        // Static environment geometry: floor box, sky dome, sun disc.
        std::cout << "demo: uploading environment meshes (floor, sky dome, sun disc)...\n";
        const MeshData floorGeometry
            = makeBoxMesh({0.0f, -0.05f, 0.0f}, {roomHalfExtent, 0.05f, roomHalfExtent}, {1, 1, 1}, roomHalfExtent);
        uploadMesh(floorGeometry, scene.floorMesh);
        const MeshData skyGeometry
            = makeSphereMesh({0.0f, 0.0f, 0.0f}, skyDomeRadius, 48, 24, {0.72f, 0.78f, 0.86f}, {0.30f, 0.48f, 0.78f});
        uploadMesh(skyGeometry, scene.skyDomeMesh);
        const MeshData sunGeometry
            = makeSphereMesh(sunDirection * sunDistance, sunRadius, 24, 12, scene.sunDiscRadiance, scene.sunDiscRadiance);
        uploadMesh(sunGeometry, scene.sunDiscMesh);
    }

    // Benchmark grid: gridWidth^2 objects, spatially grouped into chunkWidth x
    // chunkWidth chunks. Each chunk gets one merged batch per model type (rock/
    // torus), so the static scene costs batchCount draw calls instead of one
    // per object. Benchmark-only: the character screen does not need it; the
    // scene object retains the handles and instance data for rendering/teardown.
    MeshData             rockGeometry{};
    MeshData             torusGeometry{};
    if (!characterView)
    {
        constexpr std::uint32_t gridWidth            = 10;
        constexpr std::uint32_t chunkWidth           = 3;
        constexpr std::uint32_t chunksPerAxis        = (gridWidth + chunkWidth - 1) / chunkWidth;
        constexpr std::uint32_t batchCount           = chunksPerAxis * chunksPerAxis * 2;
        constexpr std::uint32_t benchmarkObjectCount = gridWidth * gridWidth;
        constexpr float         objectSpacing        = 6.2f;

        std::cout << "demo: building benchmark meshes (rock 160x80, torus 128x48)...\n";
        rockGeometry  = demo::makeRockMesh(1.0f, 160, 80, {0.48f, 0.56f, 0.62f});
        torusGeometry = demo::makeTorusMesh(1.0f, 0.32f, 128, 48, {0.82f, 0.43f, 0.16f});
        uploadMesh(rockGeometry, scene.rockMesh);
        pumpWindowEvents();
        uploadMesh(torusGeometry, scene.torusMesh);
        pumpWindowEvents();

        // Deterministic layout: checkerboard model assignment, hashed scale/yaw,
        // grid-centered positions. Every 5th object becomes a moving instance; the
        // rest are baked into its chunk's batch geometry.
        std::cout << "demo: merging " << benchmarkObjectCount << " static objects into " << batchCount
                  << " spatial batches...\n";
        std::vector<MeshData> staticBatchGeometry(batchCount);
        scene.dynamicObjects.reserve(benchmarkObjectCount / 5);
        std::uint32_t staticObjectCount = 0;
        for (std::uint32_t row = 0; row < gridWidth; ++row)
        {
            for (std::uint32_t column = 0; column < gridWidth; ++column)
            {
                const std::uint32_t modelType = (row + column) & 1u;
                const float         scale     = 0.82f + static_cast<float>((row * 3 + column * 5) % 4) * 0.11f;
                const float         yaw       = static_cast<float>((row * 7 + column * 11) % 31) * 0.17f;
                const glm::vec3     position((static_cast<float>(column) - 4.5f) * objectSpacing,
                    (modelType == 0 ? 0.94f : 0.70f) * scale,
                    (static_cast<float>(row) - 4.5f) * objectSpacing);
                const glm::mat4     transform = glm::translate(glm::mat4(1.0f), position)
                                              * glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0, 1, 0))
                                              * glm::scale(glm::mat4(1.0f), glm::vec3(scale));

                if (((row * gridWidth + column) % 5u) == 0u)
                {
                    scene.dynamicObjects.push_back(DynamicObject{
                        .modelType    = modelType,
                        .basePosition = position,
                        .scale        = scale,
                        .phase        = static_cast<float>(row * gridWidth + column) * 0.37f,
                        .angularSpeed = 0.25f + static_cast<float>((row + column) % 4) * 0.13f,
                        .transform    = transform});
                    continue;
                }

                const std::uint32_t chunkX     = column / chunkWidth;
                const std::uint32_t chunkZ     = row / chunkWidth;
                const std::uint32_t chunkIndex = chunkZ * chunksPerAxis + chunkX;
                const std::uint32_t batchIndex = chunkIndex * 2 + modelType;
                appendTransformedMesh(
                    staticBatchGeometry[batchIndex], modelType == 0 ? rockGeometry : torusGeometry, transform);
                ++staticObjectCount;
            }
        }

        // Upload the merged batches, then release all CPU-side geometry: batches
        // own their vertices now, and the per-object data is no longer needed.
        std::cout << "demo: uploading benchmark batches to the GPU...\n";
        scene.staticBatches.reserve(batchCount);
        for (std::uint32_t batchIndex = 0; batchIndex < batchCount; ++batchIndex)
        {
            MeshData &geometry = staticBatchGeometry[batchIndex];
            if (geometry.vertices.empty())
            {
                continue;
            }
            StaticBatch batch{.modelType = batchIndex & 1u};
            uploadMesh(geometry, batch.mesh);
            pumpWindowEvents();
            if (batch.mesh != renderer::invalidMesh)
            {
                scene.staticBatches.push_back(batch);
            }
            geometry = MeshData{};
        }
        staticBatchGeometry.clear();
        staticBatchGeometry.shrink_to_fit();
        const std::uint64_t rockTriangleCount  = rockGeometry.indices.size() / 3;
        const std::uint64_t torusTriangleCount = torusGeometry.indices.size() / 3;
        const std::uint64_t benchmarkTriangleCount
            = (benchmarkObjectCount / 2) * (rockTriangleCount + torusTriangleCount);
        std::cout << "demo: benchmark scene " << benchmarkObjectCount << " high-detail objects (" << staticObjectCount
                  << " static in " << scene.staticBatches.size() << " spatial batches, " << scene.dynamicObjects.size()
                  << " moving), " << benchmarkTriangleCount << " model triangles (rock=" << rockTriangleCount
                  << ", torus=" << torusTriangleCount << ")\n";
        rockGeometry  = MeshData{};
        torusGeometry = MeshData{};
    }

}

void destroyBenchmarkScene(BenchmarkScene &scene, renderer::RenderCapability &renderCapability,
    renderer::RendererHandle rendererHandle)
{
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, scene.floorMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, scene.skyDomeMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, scene.sunDiscMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, scene.rockMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, scene.torusMesh);
    for (const renderer::MeshHandle dragonMesh : scene.dragonMeshes)
    {
        renderCapability.destroyMesh(renderCapability.self, rendererHandle, dragonMesh);
    }
    for (const StaticBatch &batch : scene.staticBatches)
    {
        renderCapability.destroyMesh(renderCapability.self, rendererHandle, batch.mesh);
    }
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, scene.floorAlbedo);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, scene.floorNormal);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, scene.floorMetalRough);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, scene.floorOcclusion);
}

} // namespace demo
