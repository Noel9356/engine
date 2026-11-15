#include "planet_chunk_renderer.hpp"

#include <engine/planet/capabilities.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/planet.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include "planet_demo_support.hpp"

#include <cstdint>
#include <engine/planet/atmosphere.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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

    ++updateCounter_;
    if (visibleViews_.size() < capacity)
    {
        visibleViews_.resize(capacity);
    }
    const std::uint32_t count = std::min(
        planet_->copyVisibleChunks(planet_->self, planetHandle_, visibleViews_.data(), capacity), capacity);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const planet::PlanetChunkView &view = visibleViews_[index];
        auto found = uploadedMeshes_.try_emplace(view.key, ChunkMeshHandles{}).first;
        ChunkMeshHandles &handles = found->second;
        handles.lastSeenUpdate = updateCounter_;

        const bool wantTerrain = !view.isWater;
        renderer::MeshHandle &slot = wantTerrain ? handles.terrain : handles.water;
        std::uint64_t &uploadedHash = wantTerrain ? handles.terrainHash : handles.waterHash;
        // A chunk key can outlive one mesh REBUILD: the planet module
        // re-meshes a visible chunk when its conforming mask changes (LOD
        // boundary geometry), and the rebuilt mesh keeps the key but carries
        // a new content hash. Re-upload whenever the hash moved so the GPU
        // copy always matches the CPU mesh the module reports.
        if (slot != renderer::invalidMesh && uploadedHash != view.meshHash)
        {
            renderer_->destroyMesh(renderer_->self, rendererHandle_, slot);
            slot = renderer::invalidMesh;
        }
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
            if (slot != renderer::invalidMesh)
            {
                uploadedHash = view.meshHash;
            }
            else
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
            handles.terrainTransform = transform;
        }
        else
        {
            handles.waterTransform = transform;
        }
    }

    for (auto it = uploadedMeshes_.begin(); it != uploadedMeshes_.end();)
    {
        if (it->second.lastSeenUpdate != updateCounter_)
        {
            destroyChunkMeshes(it->second);
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

    // The dome leads the command list so it reaches the depth buffer first;
    // every terrain/water triangle is closer and overwrites it, while sky
    // pixels keep the gradient (or, where chunks are still streaming in, a
    // haze color instead of a hole to the clear color).
    if (skyDomeActive_ && skyDome_ != renderer::invalidMesh && !skyDomeMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyDome_;
        command.modelMatrix = glm::value_ptr(skyDomeTransform_);
        command.material = &skyDomeMaterials_.front();
        drawCommands_.push_back(command);
    }

    for (const auto &entry : uploadedMeshes_)
    {
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
            command.modelMatrix = glm::value_ptr(entry.second.terrainTransform);
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
            command.modelMatrix = glm::value_ptr(entry.second.waterTransform);
            command.material = &waterMaterials_.back();
            drawCommands_.push_back(command);
        }
    }
}

void PlanetChunkRenderer::destroySkyDome()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && skyDome_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, skyDome_);
    }
    skyDome_ = renderer::invalidMesh;
    skyDomeMaterials_.clear();
    skyDomeActive_ = false;
}

void PlanetChunkRenderer::destroy()
{
    for (auto &entry : uploadedMeshes_)
    {
        destroyChunkMeshes(entry.second);
    }
    uploadedMeshes_.clear();
    terrainMaterials_.clear();
    waterMaterials_.clear();
    destroySkyDome();
    drawCommands_.clear();
    planet_ = nullptr;
    planetHandle_ = planet::invalidPlanet;
    renderer_ = nullptr;
    rendererHandle_ = renderer::invalidRenderer;
}

void PlanetChunkRenderer::ensureSkyDome()
{
    if (!params_.atmosphereEnabled || renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || skyDome_ != renderer::invalidMesh)
    {
        return;
    }

    // Lat-long sphere on the unit ball. The gradient is keyed to each
    // vertex's elevation above the local horizon (dot with the dome's +Y),
    // so the dome is rotationally symmetric about its up axis and the
    // per-frame transform only has to align +Y with the eye's local up.
    constexpr std::uint32_t kSegments = 48;
    constexpr std::uint32_t kBands = 24;
    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
    indices.reserve(kSegments * kBands * 6u);
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = glm::pi<double>() * (static_cast<double>(band) / static_cast<double>(kBands) - 0.5);
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const glm::dvec3 direction(std::cos(azimuth) * cosLatitude, sinLatitude, std::sin(azimuth) * cosLatitude);
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            const planet::Vec3 color = planet::skyGradientColor(params_, direction.y);
            vertex.color[0] = static_cast<float>(color.x);
            vertex.color[1] = static_cast<float>(color.y);
            vertex.color[2] = static_cast<float>(color.z);
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            const std::uint32_t a = band * (kSegments + 1u) + segment;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (kSegments + 1u);
            const std::uint32_t d = c + 1u;
            indices.insert(indices.end(), {a, b, c, b, d, c});
        }
    }

    const auto attributes = canonicalAttributes();
    renderer::MeshDescription description;
    description.vertices = vertices.data();
    description.vertexCount = static_cast<std::uint32_t>(vertices.size());
    description.indices = indices.data();
    description.indexCount = static_cast<std::uint32_t>(indices.size());
    description.indexFormat = renderer::MeshIndexFormat::UInt32;
    description.vertexStrideBytes = kVertexStride;
    description.vertexAttributes = attributes.data();
    description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());

    renderer::RenderError error = renderer::RenderError::None;
    skyDome_ = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
    if (skyDome_ == renderer::invalidMesh)
    {
        std::cerr << "demo_planet: sky dome upload failed (error " << static_cast<std::uint32_t>(error) << ")\n";
        return;
    }
    skyDomeMaterials_.emplace_back();
    skyDomeMaterials_.front().unlit = true;
}

bool PlanetChunkRenderer::updateSkyDome(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up,
    const glm::dvec3 &sunDirection, bool sunEnabled, double nearZ, double farZ, float outClearColor[3])
{
    outClearColor[0] = 0.0f;
    outClearColor[1] = 0.0f;
    outClearColor[2] = 0.0f;
    skyDomeActive_ = false;
    if (!params_.atmosphereEnabled || skyDome_ == renderer::invalidMesh || skyDomeMaterials_.empty())
    {
        return false;
    }

    const glm::dvec3 upNormalized = glm::length(up) > 1.0e-9 ? glm::normalize(up) : glm::dvec3(0.0, 1.0, 0.0);
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    const double weight = planet::atmosphereWeight(params_, eyeAltitude);
    if (weight <= 0.0)
    {
        return false;
    }

    // Dome radius: past the farthest terrain sight line (the horizon of the
    // highest possible relief, with margin), inside the far plane. Below
    // that horizon every sky pixel is genuinely sky; nearer domes would
    // paint over distant mountains, farther ones clip away.
    const double reliefTop = std::max(0.0, params_.derivedMaxReliefMeters);
    const double sightCeiling = std::max(1.0, eyeAltitude + reliefTop);
    const double horizonDistance = std::sqrt(std::max(0.0, sightCeiling * (2.0 * params_.radiusMeters + sightCeiling)));
    const double domeRadius = glm::clamp(1.25 * horizonDistance, nearZ * 8.0 + 10.0, farZ * 0.4);

    // Orientation: dome +Y onto the local up. Any tangent works for the
    // third axis - the gradient is symmetric about the up axis.
    glm::dvec3 tangent = glm::cross(upNormalized, glm::dvec3(0.0, 0.0, 1.0));
    if (glm::length(tangent) < 1.0e-6)
    {
        tangent = glm::cross(upNormalized, glm::dvec3(1.0, 0.0, 0.0));
    }
    tangent = glm::normalize(tangent);
    const glm::dvec3 third = glm::cross(tangent, upNormalized);
    glm::mat3 rotation(tangent, upNormalized, third);
    skyDomeTransform_ = glm::translate(glm::mat4(1.0f), glm::vec3(eyePlanetLocal))
        * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), glm::vec3(domeRadius));

    const double sunUp = sunEnabled && glm::length(sunDirection) > 1.0e-9
        ? glm::dot(glm::normalize(sunDirection), upNormalized)
        : 1.0;
    const planet::Vec3 tint = planet::sunSkyTint(params_, sunUp, sunEnabled);
    renderer::MeshMaterial &material = skyDomeMaterials_.front();
    material.baseColor[0] = static_cast<float>(tint.x * weight);
    material.baseColor[1] = static_cast<float>(tint.y * weight);
    material.baseColor[2] = static_cast<float>(tint.z * weight);
    skyDomeActive_ = true;

    // Clear color: the haze a sight line just under the dome's rim would
    // hit. Streaming chunk gaps and any sliver the dome's geometry leaves
    // below the horizon then photograph as atmosphere instead of void.
    const planet::Vec3 horizon = planet::skyGradientColor(params_, 0.0);
    outClearColor[0] = static_cast<float>(horizon.x * tint.x * weight);
    outClearColor[1] = static_cast<float>(horizon.y * tint.y * weight);
    outClearColor[2] = static_cast<float>(horizon.z * tint.z * weight);
    return true;
}

} // namespace demo::planet_demo
