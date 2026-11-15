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

    // The cloud shell sits above the terrain and below the dome's far rim;
    // depth testing sorts it against terrain, water and the dome (the dome
    // is unlit background, the clouds a lit geometry layer).
    if (cloudLayerActive_ && cloudLayerVisible_ && cloudLayer_ != renderer::invalidMesh
        && !cloudLayerMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = cloudLayer_;
        command.modelMatrix = glm::value_ptr(cloudLayerTransform_);
        command.material = &cloudLayerMaterials_.front();
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

void PlanetChunkRenderer::destroyCloudLayer()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && cloudLayer_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, cloudLayer_);
    }
    cloudLayer_ = renderer::invalidMesh;
    cloudLayerMaterials_.clear();
    cloudLayerActive_ = false;
}

void PlanetChunkRenderer::ensureCloudLayer()
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || cloudLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // Lat-long grid on the cloud shell (datum radius + cloud altitude).
    // 288 x 144 quads keep the vertex spacing (~1.3 km at the equator of the
    // 60 km showcase) well under the configured cloud wavelength, so the
    // shell resolves the banks while the coverage field's band limit (the
    // spacing is passed as the cutoff) keeps sub-spacing octaves out. One
    // box-smoothing pass over the coverage grid rounds the inclusion
    // boundary the quad classification would otherwise pixelate.
    constexpr std::uint32_t kSegments = 288;
    constexpr std::uint32_t kBands = 144;
    if (cloudField_ == nullptr)
    {
        cloudField_ = std::make_unique<planet::CloudField>(params_);
    }

    const double shellRadius = params_.radiusMeters + params_.cloudAltitudeMeters;
    const double vertexSpacing = glm::two_pi<double>() * shellRadius / static_cast<double>(kSegments);
    // A quad renders when its corner coverage AVERAGE clears the floor; the
    // per-vertex color shades with coverage so the kept edge quads read as
    // thinning cloud instead of a hard polygon boundary.
    constexpr double kCoverageFloor = 0.42;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
    indices.reserve(kSegments * kBands * 6u);
    std::vector<float> coverageGrid;
    coverageGrid.reserve((kSegments + 1u) * (kBands + 1u));
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = glm::pi<double>() * (static_cast<double>(band) / static_cast<double>(kBands) - 0.5);
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const glm::dvec3 direction(std::cos(azimuth) * cosLatitude, sinLatitude, std::sin(azimuth) * cosLatitude);
            const planet::Vec3 unitDirection{direction.x, direction.y, direction.z};
            const float coverage = static_cast<float>(planet::clamp01(
                cloudField_->coverage(unitDirection, vertexSpacing)));
            coverageGrid.push_back(coverage);

            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x * shellRadius);
            vertex.position[1] = static_cast<float>(direction.y * shellRadius);
            vertex.position[2] = static_cast<float>(direction.z * shellRadius);
            vertex.normal[0] = static_cast<float>(direction.x);
            vertex.normal[1] = static_cast<float>(direction.y);
            vertex.normal[2] = static_cast<float>(direction.z);
            // Coverage shades the cloud color: dense banks at full albedo,
            // thin edges dimmer (a soft read for the geometry edges at shell
            // resolution).
            const float shade = 0.88f + 0.12f * coverage;
            vertex.color[0] = params_.cloudColor[0] * shade;
            vertex.color[1] = params_.cloudColor[1] * shade;
            vertex.color[2] = params_.cloudColor[2] * shade;
            vertex.uv[0] = 0.0f;
            vertex.uv[1] = 0.0f;
            vertices.push_back(vertex);
        }
    }
    // One 3x3 box pass over the coverage grid (wrap the segments; clamp the
    // bands): rounds the quad inclusion boundary the same way an alpha edge
    // would soften a textured deck.
    {
        std::vector<float> smoothed(coverageGrid.size());
        const std::uint32_t stride = kSegments + 1u;
        for (std::uint32_t band = 0; band <= kBands; ++band)
        {
            const std::uint32_t bandDown = band > 0 ? band - 1u : 0u;
            const std::uint32_t bandUp = band < kBands ? band + 1u : kBands;
            for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
            {
                const std::uint32_t left = (segment + kSegments) % kSegments;
                const std::uint32_t right = (segment + 1u) % kSegments;
                const float sum = coverageGrid[band * stride + left] + coverageGrid[band * stride + segment]
                    + coverageGrid[band * stride + right]
                    + coverageGrid[bandDown * stride + left] + coverageGrid[bandDown * stride + segment]
                    + coverageGrid[bandDown * stride + right]
                    + coverageGrid[bandUp * stride + left] + coverageGrid[bandUp * stride + segment]
                    + coverageGrid[bandUp * stride + right];
                smoothed[band * stride + segment] = sum / 9.0f;
            }
        }
        coverageGrid.swap(smoothed);
    }

    const auto coverageAt = [&coverageGrid, stride = kSegments + 1u](std::uint32_t band, std::uint32_t segment)
    {
        return coverageGrid[static_cast<std::size_t>(band) * stride + segment];
    };

    // Marching squares over the coverage grid: each cell contributes the
    // polygon where coverage >= the floor, with the crossing points linearly
    // interpolated along the cell edges. The contour then follows the
    // coverage field instead of the quad grid, and the shell reads as
    // organic cloud shapes (the same contouring idea the terrain applies in
    // 3D, in 2D on the shell). Crossing vertices are shared per cell
    // boundary by construction (both neighbors interpolate the same grid
    // edge), so the shapes weld without duplicated slivers.
    const auto interpolatedVertex = [&vertices](std::uint32_t indexA, std::uint32_t indexB, float t) -> std::uint32_t
    {
        const planet::PlanetVertex &va = vertices[indexA];
        const planet::PlanetVertex &vb = vertices[indexB];
        planet::PlanetVertex v{};
        for (std::uint32_t k = 0; k < 3; ++k)
        {
            v.position[k] = va.position[k] + (vb.position[k] - va.position[k]) * t;
            v.normal[k] = va.normal[k] + (vb.normal[k] - va.normal[k]) * t;
            v.color[k] = va.color[k] + (vb.color[k] - va.color[k]) * t;
        }
        const std::uint32_t slot = static_cast<std::uint32_t>(vertices.size());
        vertices.push_back(v);
        return slot;
    };

    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            // Ring order: top-left, top-right, bottom-right, bottom-left;
            // each ring step crosses one cell edge (top, right, bottom,
            // left).
            const std::uint32_t stride = kSegments + 1u;
            const std::uint32_t tl = band * stride + segment;
            const std::uint32_t tr = tl + 1u;
            const std::uint32_t br = tl + stride + 1u;
            const std::uint32_t bl = tl + stride;
            const std::uint32_t ring[4] = {tl, tr, br, bl};
            const float cornerCoverage[4] = {coverageAt(band, segment), coverageAt(band, segment + 1u),
                coverageAt(band + 1u, segment + 1u), coverageAt(band + 1u, segment)};
            bool inside[4];
            std::uint32_t insideCount = 0;
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                inside[corner] = cornerCoverage[corner] >= kCoverageFloor;
                insideCount += inside[corner] ? 1u : 0u;
            }
            if (insideCount == 0)
            {
                continue; // clear sky: no geometry, the surface shows through
            }
            if (insideCount == 4)
            {
                indices.insert(indices.end(), {tl, tr, br, tl, br, bl});
                continue;
            }

            // Ambiguous diagonal case (TL/BR or TR/BL inside): the saddle
            // either joins the diagonal pair into one bank or splits it into
            // two wedges - the cell average picks which corners join.
            const float cellMean = 0.25f * (cornerCoverage[0] + cornerCoverage[1] + cornerCoverage[2] + cornerCoverage[3]);
            if (insideCount == 2 && inside[0] != inside[1] && inside[1] != inside[2])
            {
                for (std::uint32_t corner = 0; corner < 4; ++corner)
                {
                    inside[corner] = cornerCoverage[corner] >= kCoverageFloor
                        && cornerCoverage[corner] >= cellMean;
                }
                insideCount = (inside[0] ? 1u : 0u) + (inside[1] ? 1u : 0u) + (inside[2] ? 1u : 0u)
                    + (inside[3] ? 1u : 0u);
                if (insideCount == 0)
                {
                    continue;
                }
                if (insideCount == 4)
                {
                    indices.insert(indices.end(), {tl, tr, br, tl, br, bl});
                    continue;
                }
            }

            // Boundary walk: for each ring edge, the crossing point (if the
            // edge straddles the floor) or the inside corner itself; the
            // polygon is the crossing points plus inside corners in ring
            // order, fan-triangulated from its first point.
            std::uint32_t polygon[8];
            std::uint32_t polygonCount = 0;
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                const std::uint32_t next = (corner + 1u) % 4u;
                const bool straddles = inside[corner] != inside[next];
                if (inside[corner])
                {
                    polygon[polygonCount++] = ring[corner];
                }
                if (straddles)
                {
                    const float coverageA = cornerCoverage[corner];
                    const float coverageB = cornerCoverage[next];
                    const float denominator = coverageA - coverageB;
                    const float t = std::abs(denominator) < 1.0e-6f
                        ? 0.5f
                        : std::clamp(static_cast<float>((coverageA - kCoverageFloor) / denominator), 0.0f, 1.0f);
                    polygon[polygonCount++] = interpolatedVertex(ring[corner], ring[next], t);
                }
            }
            for (std::uint32_t fan = 1; fan + 1 < polygonCount; ++fan)
            {
                indices.insert(indices.end(), {polygon[0], polygon[fan], polygon[fan + 1]});
            }
        }
    }
    if (indices.empty())
    {
        return; // coverage configured to zero: nothing to draw
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
    cloudLayer_ = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
    if (cloudLayer_ == renderer::invalidMesh)
    {
        std::cerr << "demo_planet: cloud layer upload failed (error " << static_cast<std::uint32_t>(error) << ")\n";
        return;
    }
    // Planet-local shell vertices: the placement mirrors the chunk transform
    // (planet-local geometry shifted by the planet's center offset).
    cloudLayerTransform_ = glm::translate(glm::mat4(1.0f),
        glm::vec3(static_cast<float>(-params_.centerMeters[0]), static_cast<float>(-params_.centerMeters[1]),
            static_cast<float>(-params_.centerMeters[2])));
    renderer::MeshMaterial material{};
    material.baseColor[0] = 1.0f;
    material.baseColor[1] = 1.0f;
    material.baseColor[2] = 1.0f;
    material.roughnessFactor = 0.9f; // matte decks; the sun does the shading
    cloudLayerMaterials_.push_back(material);
    cloudLayerActive_ = true;
}

void PlanetChunkRenderer::setCloudLayerVisible(bool visible)
{
    cloudLayerVisible_ = visible;
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
    destroyCloudLayer();
    cloudField_.reset();
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
