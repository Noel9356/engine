#include "planet_chunk_renderer.hpp"

#include "planet_demo_support.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

namespace demo::planet_demo
{
namespace planet = engine::planet;
namespace renderer = engine::renderer;

namespace
{
constexpr std::uint32_t kVertexStride = sizeof(planet::PlanetVertex);
}

PlanetChunkRenderer::PlanetChunkRenderer(planet::PlanetCapability &planetCapability, planet::PlanetHandle planetHandle,
    renderer::RenderCapability &renderCapability, renderer::RendererHandle rendererHandle,
    const planet::PlanetGenerationParams &generationParams)
    : planet_(&planetCapability)
    , planetHandle_(planetHandle)
    , renderer_(&renderCapability)
    , rendererHandle_(rendererHandle)
    , params_(generationParams)
{
}

PlanetChunkRenderer::~PlanetChunkRenderer()
{
    destroy();
}

void PlanetChunkRenderer::destroyChunkMeshes(ChunkMeshHandles &handles)
{
    if (renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer)
    {
        handles = {};
        return;
    }
    if (handles.terrain != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, handles.terrain);
        handles.terrain = renderer::invalidMesh;
    }
    if (handles.water != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, handles.water);
        handles.water = renderer::invalidMesh;
    }
}

void PlanetChunkRenderer::updateVisibleChunks()
{
    if (planet_ == nullptr || renderer_ == nullptr)
    {
        return;
    }
    const std::uint32_t capacity = planet_->visibleChunkCapacity(planet_->self, planetHandle_);
    if (capacity == 0)
    {
        return;
    }

    std::vector<planet::PlanetChunkView> views(capacity);
    const std::uint32_t count = std::min(
        planet_->copyVisibleChunks(planet_->self, planetHandle_, views.data(), capacity), capacity);
    std::map<planet::ChunkKey, bool> seen;
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const planet::PlanetChunkView &view = views[index];
        seen[view.key] = true;
        auto found = uploadedMeshes_.try_emplace(view.key, ChunkMeshHandles{}).first;
        ChunkMeshHandles &handles = found->second;

        const bool wantTerrain = !view.isWater;
        renderer::MeshHandle &slot = wantTerrain ? handles.terrain : handles.water;
        if (slot == renderer::invalidMesh && view.vertexCount > 0)
        {
            const auto attributes = canonicalAttributes();
            renderer::MeshDescription description;
            description.vertices = view.vertices;
            description.vertexCount = view.vertexCount;
            description.indices = view.indices;
            description.indexCount = view.indexCount;
            description.indexFormat = renderer::MeshIndexFormat::UInt32;
            description.vertexStrideBytes = kVertexStride;
            description.vertexAttributes = attributes.data();
            description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());

            renderer::RenderError error = renderer::RenderError::None;
            slot = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
            if (slot == renderer::invalidMesh)
            {
                std::cerr << "demo_planet: chunk mesh upload failed (error "
                          << static_cast<std::uint32_t>(error) << ")\n";
            }
        }

        const glm::mat4 transform = glm::translate(glm::mat4(1.0f),
            glm::vec3(static_cast<float>(view.centerMeters.x - params_.centerMeters[0]),
                static_cast<float>(view.centerMeters.y - params_.centerMeters[1]),
                static_cast<float>(view.centerMeters.z - params_.centerMeters[2])));
        if (wantTerrain)
        {
            chunkTransforms_[view.key].first = transform;
        }
        else
        {
            chunkTransforms_[view.key].second = transform;
        }
    }

    for (auto it = uploadedMeshes_.begin(); it != uploadedMeshes_.end();)
    {
        if (seen.find(it->first) == seen.end())
        {
            destroyChunkMeshes(it->second);
            chunkTransforms_.erase(it->first);
            it = uploadedMeshes_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void PlanetChunkRenderer::rebuildDrawCommands()
{
    drawCommands_.clear();
    terrainMaterials_.clear();
    waterMaterials_.clear();

    for (const auto &entry : uploadedMeshes_)
    {
        const auto transformIt = chunkTransforms_.find(entry.first);
        if (transformIt == chunkTransforms_.end())
        {
            continue;
        }
        if (entry.second.terrain != renderer::invalidMesh)
        {
            renderer::MeshMaterial material{};
            material.baseColor[0] = 1.0f;
            material.baseColor[1] = 1.0f;
            material.baseColor[2] = 1.0f;
            material.roughnessFactor = 0.95f;
            terrainMaterials_.push_back(material);

            renderer::RenderDrawCommand command{};
            command.mesh = entry.second.terrain;
            command.modelMatrix = glm::value_ptr(transformIt->second.first);
            command.material = &terrainMaterials_.back();
            drawCommands_.push_back(command);
        }
        if (entry.second.water != renderer::invalidMesh)
        {
            renderer::MeshMaterial material{};
            material.baseColor[0] = 1.0f;
            material.baseColor[1] = 1.0f;
            material.baseColor[2] = 1.0f;
            material.roughnessFactor = params_.waterRoughness;
            waterMaterials_.push_back(material);

            renderer::RenderDrawCommand command{};
            command.mesh = entry.second.water;
            command.modelMatrix = glm::value_ptr(transformIt->second.second);
            command.material = &waterMaterials_.back();
            drawCommands_.push_back(command);
        }
    }
}

void PlanetChunkRenderer::destroy()
{
    for (auto &entry : uploadedMeshes_)
    {
        destroyChunkMeshes(entry.second);
    }
    uploadedMeshes_.clear();
    chunkTransforms_.clear();
    terrainMaterials_.clear();
    waterMaterials_.clear();
    drawCommands_.clear();
    planet_ = nullptr;
    planetHandle_ = planet::invalidPlanet;
    renderer_ = nullptr;
    rendererHandle_ = renderer::invalidRenderer;
}

} // namespace demo::planet_demo
