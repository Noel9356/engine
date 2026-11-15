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

// Spec for one contoured cloud-coverage shell mesh (see
// engine/planet/atmosphere.hpp CloudField). Both backdrop layers the demo
// draws from the coverage field - the world-anchored macro shell and the
// eye-centered sky deck - go through this one builder; they differ only in
// their latitude span, radius, sample cutoff and quad floor.
struct CoverageShellSpec
{
    std::uint32_t segments = 0;                    // longitude quads
    std::uint32_t bands = 0;                       // latitude quads over the span
    double latitudeLowRadians = 0.0;               // band 0's latitude
    double latitudeHighRadians = 0.0;              // band `bands`' latitude
    double shellRadiusMeters = 0.0;                // vertex radius (unit ball: 1.0)
    double lowPassCutoffMeters = 0.0;              // coverage field band limit at the sample spacing
    double coverageFloor = 0.0;                    // coverage a quad corner needs to render
    double minQuadLatitudeRadians = -6.283185307179586; // quads below this latitude are skipped
    // Edge softening: when set, quad corners with coverage near the floor
    // blend their vertex color toward the sky gradient behind them (sampled
    // at the corner's own elevation), so a deck's contoured boundary reads
    // as thinning cloud instead of a hard polygon cutout. The per-frame sun
    // tint applies to both the cloud and the sky colors, so the blend tracks
    // the dome behind it.
    bool hazeEdgeBlend = false;
    // Frame applied to each unit direction before coverage sampling: the sky
    // deck is built in the sky dome's initial frame (its +Y on the spawn
    // up), the macro shell samples directions directly (identity).
    glm::dmat3 frame{1.0};
};

// Builds the contoured shell mesh: coverage is sampled on the lat-long grid,
// smoothed once, then every quad whose corner coverage clears the floor
// contributes its inside polygon (marching squares with linearly
// interpolated crossings, ambiguous saddles resolved by the cell mean).
// Vertices carry the coverage-shaded cloud color and the radial normal, so
// kept edge quads read as thinning cloud instead of a hard polygon boundary.
// Crossing vertices are shared per cell boundary by construction (both
// neighbors interpolate the same grid edge), so the shapes weld without
// duplicated slivers. Returns false when nothing clears the floor.
bool buildCoverageShellMesh(const planet::CloudField &field, const planet::PlanetGenerationParams &params,
    const CoverageShellSpec &spec, std::vector<planet::PlanetVertex> &vertices,
    std::vector<std::uint32_t> &indices)
{
    const std::uint32_t stride = spec.segments + 1u;
    std::vector<float> coverageGrid;
    coverageGrid.reserve(static_cast<std::size_t>(stride) * (spec.bands + 1u));
    for (std::uint32_t band = 0; band <= spec.bands; ++band)
    {
        const double latitude = spec.latitudeLowRadians
            + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                * (static_cast<double>(band) / static_cast<double>(spec.bands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= spec.segments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(spec.segments);
            const glm::dvec3 shellDirection(std::cos(azimuth) * cosLatitude, sinLatitude, std::sin(azimuth) * cosLatitude);
            const glm::dvec3 fieldDirection = spec.frame * shellDirection;
            const float coverage = static_cast<float>(planet::clamp01(field.coverage(
                planet::Vec3{fieldDirection.x, fieldDirection.y, fieldDirection.z}, spec.lowPassCutoffMeters)));
            coverageGrid.push_back(coverage);

            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(shellDirection.x * spec.shellRadiusMeters);
            vertex.position[1] = static_cast<float>(shellDirection.y * spec.shellRadiusMeters);
            vertex.position[2] = static_cast<float>(shellDirection.z * spec.shellRadiusMeters);
            vertex.normal[0] = static_cast<float>(shellDirection.x);
            vertex.normal[1] = static_cast<float>(shellDirection.y);
            vertex.normal[2] = static_cast<float>(shellDirection.z);
            // Coverage shades the cloud color: dense banks at full albedo,
            // thin edges dimmer (a soft read for the geometry edges at shell
            // resolution).
            const float shade = 0.88f + 0.12f * coverage;
            glm::dvec3 color(params.cloudColor[0] * shade, params.cloudColor[1] * shade, params.cloudColor[2] * shade);
            if (spec.hazeEdgeBlend)
            {
                const planet::Vec3 sky = planet::skyGradientColor(params, sinLatitude);
                const double blend = planet::smoothstep(spec.coverageFloor,
                    std::min(1.0, spec.coverageFloor + 0.25), static_cast<double>(coverage));
                color = glm::mix(glm::dvec3(sky.x, sky.y, sky.z), color, blend);
            }
            vertex.color[0] = static_cast<float>(color.x);
            vertex.color[1] = static_cast<float>(color.y);
            vertex.color[2] = static_cast<float>(color.z);
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
        for (std::uint32_t band = 0; band <= spec.bands; ++band)
        {
            const std::uint32_t bandDown = band > 0 ? band - 1u : 0u;
            const std::uint32_t bandUp = band < spec.bands ? band + 1u : spec.bands;
            for (std::uint32_t segment = 0; segment <= spec.segments; ++segment)
            {
                const std::uint32_t left = (segment + spec.segments) % spec.segments;
                const std::uint32_t right = (segment + 1u) % spec.segments;
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

    const auto coverageAt = [&coverageGrid, stride](std::uint32_t band, std::uint32_t segment)
    {
        return coverageGrid[static_cast<std::size_t>(band) * stride + segment];
    };

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

    for (std::uint32_t band = 0; band < spec.bands; ++band)
    {
        const double quadLatitude = spec.latitudeLowRadians
            + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                * (static_cast<double>(band) / static_cast<double>(spec.bands));
        if (quadLatitude < spec.minQuadLatitudeRadians)
        {
            continue; // below the guard latitude (the horizon rim, for sky decks)
        }
        for (std::uint32_t segment = 0; segment < spec.segments; ++segment)
        {
            // Ring order: top-left, top-right, bottom-right, bottom-left;
            // each ring step crosses one cell edge (top, right, bottom,
            // left).
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
                inside[corner] = cornerCoverage[corner] >= spec.coverageFloor;
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
                    inside[corner] = cornerCoverage[corner] >= spec.coverageFloor
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
                        : std::clamp(static_cast<float>((coverageA - spec.coverageFloor) / denominator), 0.0f, 1.0f);
                    polygon[polygonCount++] = interpolatedVertex(ring[corner], ring[next], t);
                }
            }
            for (std::uint32_t fan = 1; fan + 1 < polygonCount; ++fan)
            {
                indices.insert(indices.end(), {polygon[0], polygon[fan], polygon[fan + 1]});
            }
        }
    }
    return !indices.empty();
}

// Uploads a built shell mesh as a canonical renderer mesh.
renderer::MeshHandle uploadShellMesh(renderer::RenderCapability &renderCapability, renderer::RendererHandle rendererHandle,
    const std::vector<planet::PlanetVertex> &vertices, const std::vector<std::uint32_t> &indices)
{
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
    const renderer::MeshHandle handle
        = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &error);
    if (handle == renderer::invalidMesh)
    {
        std::cerr << "demo_planet: shell mesh upload failed (error " << static_cast<std::uint32_t>(error) << ")\n";
    }
    return handle;
}

// Gribb-Hartmann frustum plane extraction from a column-major clip matrix
// (projection * view). Planes point INTO the visible half-space.
void extractFrustumPlanes(const glm::mat4 &viewProjection, glm::dvec4 outPlanes[6])
{
    const auto row = [&viewProjection](std::uint32_t r)
    {
        return glm::vec4(viewProjection[0][r], viewProjection[1][r], viewProjection[2][r], viewProjection[3][r]);
    };
    const glm::vec4 rowX = row(0);
    const glm::vec4 rowY = row(1);
    const glm::vec4 rowZ = row(2);
    const glm::vec4 rowW = row(3);
    const glm::vec4 raw[6] = {rowW + rowX, rowW - rowX, rowW + rowY, rowW - rowY, rowW + rowZ, rowW - rowZ};
    for (std::uint32_t plane = 0; plane < 6; ++plane)
    {
        const float normalLength = glm::length(glm::vec3(raw[plane]));
        outPlanes[plane] = normalLength > 1.0e-8f
            ? glm::dvec4(raw[plane]) / static_cast<double>(normalLength)
            : glm::dvec4(0.0, 0.0, 0.0, 1.0);
    }
}
} // namespace

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

        // Chunk vertices are planet-local offsets from the chunk center; the
        // world placement adds the planet's own center offset (world =
        // planet-local + center, matching the module's worldPointToPlanetLocal).
        const glm::mat4 transform = glm::translate(glm::mat4(1.0f),
            glm::vec3(static_cast<float>(params_.centerMeters[0] + view.centerMeters.x),
                static_cast<float>(params_.centerMeters[1] + view.centerMeters.y),
                static_cast<float>(params_.centerMeters[2] + view.centerMeters.z)));
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

void PlanetChunkRenderer::rebuildDrawCommands(const glm::mat4 &viewProjection, double eyeAltitudeMeters)
{
    drawCommands_.clear();
    terrainMaterials_.clear();
    waterMaterials_.clear();

    // Backdrop order: every shell reaches the depth buffer before the scene
    // geometry and each later layer overwrites the previous one it sits in
    // front of.
    //   1. atmosphere limb (space views): unlit, depth writes off; the ring
    //      outside the terrain silhouette survives as glowing air.
    //   2. sky dome: fills the sky pixels the terrain does not reach (or,
    //      where chunks are still streaming in, a haze color instead of a
    //      hole to the clear color).
    //   3. sky cloud deck (ground views): coverage-contoured banks overhead,
    //      tinted with the dome.
    //   4. world-anchored cloud shell (macro views): lit geometry between
    //      the terrain and the dome's far rim.
    if (atmosphereShellActive_ && atmosphereShell_ != renderer::invalidMesh && !atmosphereShellMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = atmosphereShell_;
        command.modelMatrix = glm::value_ptr(atmosphereShellTransform_);
        command.material = &atmosphereShellMaterials_.front();
        drawCommands_.push_back(command);
    }
    if (skyDomeActive_ && skyDome_ != renderer::invalidMesh && !skyDomeMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyDome_;
        command.modelMatrix = glm::value_ptr(skyDomeTransform_);
        command.material = &skyDomeMaterials_.front();
        drawCommands_.push_back(command);
    }
    if (skyCloudActive_ && skyCloudVisible_ && skyCloudLayer_ != renderer::invalidMesh && !skyCloudMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyCloudLayer_;
        command.modelMatrix = glm::value_ptr(skyCloudTransform_);
        command.material = &skyCloudMaterials_.front();
        drawCommands_.push_back(command);
    }
    if (cloudLayerActive_ && cloudLayerVisible_ && cloudLayer_ != renderer::invalidMesh
        && !cloudLayerMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = cloudLayer_;
        command.modelMatrix = glm::value_ptr(cloudLayerTransform_);
        command.material = &cloudLayerMaterials_.front();
        drawCommands_.push_back(command);
    }

    // Frustum culling: the planet's visible-chunk selection spans the whole
    // horizon disk, so roughly half the set sits behind the camera plane and
    // never rasterizes a pixel. Sphere-vs-plane rejection (Gribb-Hartmann
    // planes from the clip matrix; the chunk's bounding sphere from its
    // octree depth) drops those before any command storage.
    glm::dvec4 frustumPlanes[6];
    extractFrustumPlanes(viewProjection, frustumPlanes);

    // Water roughness by eye altitude: the per-vertex wave noise fades with
    // the LOD band limit, so from above the sea is a sheet of flat coarse
    // facets and the sun's specular term turns each facet into a separate
    // sharp glint - a field of sparkles around the sun point. The material
    // therefore spreads toward matte as the eye climbs the atmosphere column
    // (fully spread by twice the atmosphere height, so a mid-orbit camera is
    // already past the facet-glint band); close-range views keep the
    // configured sea texture.
    const double orbitBlend = planet::smoothstep(params_.atmosphereHeightMeters, 2.0 * params_.atmosphereHeightMeters,
        eyeAltitudeMeters);
    const float waterRoughness = static_cast<float>(params_.waterRoughness + (0.85f - params_.waterRoughness) * orbitBlend);

    const glm::dvec3 planetCenter(params_.centerMeters[0], params_.centerMeters[1], params_.centerMeters[2]);
    for (const auto &entry : uploadedMeshes_)
    {
        // Bounding sphere from the chunk key's octree depth: the node cube's
        // circumradius around the placement center.
        std::uint32_t depth = 0;
        std::uint32_t cellX = 0;
        std::uint32_t cellY = 0;
        std::uint32_t cellZ = 0;
        planet::unpackChunkKey(entry.first, depth, cellX, cellY, cellZ);
        const double nodeSize = planet::chunkSizeMeters(params_, depth);
        // Placement center from whichever draw the chunk actually carries
        // (all-water chunks have an empty terrain mesh and an untouched
        // terrain transform).
        const glm::mat4 &placement
            = entry.second.terrain != renderer::invalidMesh ? entry.second.terrainTransform : entry.second.waterTransform;
        const glm::dvec3 worldCenter = planetCenter
            + glm::dvec3(placement[3][0], placement[3][1], placement[3][2]);
        const double boundRadius = 0.5 * nodeSize * glm::sqrt(3.0);
        bool visible = true;
        for (const glm::dvec4 &plane : frustumPlanes)
        {
            if (glm::dot(plane, glm::dvec4(worldCenter, 1.0)) < -boundRadius)
            {
                visible = false;
                break;
            }
        }
        if (!visible)
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
            material.roughnessFactor = waterRoughness;
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
    // spacing is passed as the cutoff) keeps sub-spacing octaves out.
    constexpr std::uint32_t kSegments = 288;
    constexpr std::uint32_t kBands = 144;
    constexpr double kCoverageFloor = 0.42;

    if (cloudField_ == nullptr)
    {
        cloudField_ = std::make_unique<planet::CloudField>(params_);
    }

    const double shellRadius = params_.radiusMeters + params_.cloudAltitudeMeters;
    const double vertexSpacing = glm::two_pi<double>() * shellRadius / static_cast<double>(kSegments);

    CoverageShellSpec spec;
    spec.segments = kSegments;
    spec.bands = kBands;
    spec.latitudeLowRadians = -glm::pi<double>() * 0.5;
    spec.latitudeHighRadians = glm::pi<double>() * 0.5;
    spec.shellRadiusMeters = shellRadius;
    spec.lowPassCutoffMeters = vertexSpacing;
    spec.coverageFloor = kCoverageFloor;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    if (!buildCoverageShellMesh(*cloudField_, params_, spec, vertices, indices))
    {
        return; // coverage configured to zero: nothing to draw
    }

    cloudLayer_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (cloudLayer_ == renderer::invalidMesh)
    {
        return;
    }
    // Planet-local shell vertices: the placement mirrors the chunk transform
    // (planet-local geometry shifted by the planet's center offset).
    cloudLayerTransform_ = glm::translate(glm::mat4(1.0f),
        glm::vec3(static_cast<float>(params_.centerMeters[0]), static_cast<float>(params_.centerMeters[1]),
            static_cast<float>(params_.centerMeters[2])));
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

void PlanetChunkRenderer::ensureAtmosphereShell()
{
    if (!params_.atmosphereEnabled || renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || atmosphereShell_ != renderer::invalidMesh)
    {
        return;
    }

    // Hemisphere around the planet, latitudes 0 (its equator rim) to 90
    // degrees (its pole). The pole points at the eye each frame, so mesh
    // latitude runs from the silhouette edge (0, equator) to the disk center
    // (90, pole). Colors key on the angle from the pole: bright horizon
    // color over the disk interior (the opaque terrain/cloud draws overwrite
    // it), fading to black at the rim - the ring between the terrain
    // silhouette and the shell's own silhouette then reads as glowing air
    // instead of a hard terrain edge on black.
    constexpr std::uint32_t kSegments = 96;
    constexpr std::uint32_t kBands = 48;
    // Fade band in the angle from the pole. The shell is scaled per frame to
    // a visual radius a few percent above the surface (see
    // updateAtmosphereShell - the configured air column can span a large
    // fraction of a showcase radius, but the VISIBLE glow hugs the ground
    // the way density does); from there, perspective compresses the ring
    // between the terrain silhouette and the shell rim into roughly 35-90
    // degrees from orbit and 56-90 from the globe view, so the gradient
    // starts fading at 30 degrees and lands on black at 85.
    constexpr double kLimbFadeStartRadians = 0.5235987755982988; // 30 deg
    constexpr double kLimbFadeEndRadians = 1.4835298641951802;   // 85 deg

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
    indices.reserve(kSegments * kBands * 6u);
    const planet::Vec3 limbColor = planet::skyGradientColor(params_, 0.0); // the horizon band color
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = glm::pi<double>() * 0.5 * (static_cast<double>(band) / static_cast<double>(kBands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        const double angleFromPole = glm::pi<double>() * 0.5 - latitude;
        // Exponential-ish falloff: dense air right at the terrain silhouette,
        // thinning rapidly toward the top of the glow (a smoothstep alone
        // reads as a flat band with a hard edge).
        const double fade = std::pow(1.0 - planet::smoothstep(kLimbFadeStartRadians, kLimbFadeEndRadians, angleFromPole), 1.6);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const glm::dvec3 direction(std::cos(azimuth) * cosLatitude, sinLatitude, std::sin(azimuth) * cosLatitude);
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            vertex.color[0] = static_cast<float>(limbColor.x * fade);
            vertex.color[1] = static_cast<float>(limbColor.y * fade);
            vertex.color[2] = static_cast<float>(limbColor.z * fade);
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

    atmosphereShell_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (atmosphereShell_ == renderer::invalidMesh)
    {
        return;
    }
    atmosphereShellMaterials_.emplace_back();
    renderer::MeshMaterial &material = atmosphereShellMaterials_.front();
    material.unlit = true;
    // Depth writes off: the shell is a backdrop drawn ahead of the scene.
    // Depth testing stays on (the cleared buffer passes it) so the opaque
    // terrain/cloud/water draws that follow overwrite the disk interior and
    // only the silhouette ring keeps the shell's color.
    material.depthWrite = false;
    atmosphereShellActive_ = true;
}

void PlanetChunkRenderer::updateAtmosphereShell(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection,
    bool sunEnabled)
{
    atmosphereShellActive_ = atmosphereShell_ != renderer::invalidMesh && !atmosphereShellMaterials_.empty();
    if (!atmosphereShellActive_)
    {
        return;
    }
    // The limb is a space-view backdrop: once the eye is inside the air the
    // sky dome owns the whole sky, and the shell's far side would smear
    // through it. atmosphereWeight reaches 0 at the configured atmosphere
    // height; the shell activates exactly below that.
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    if (planet::atmosphereWeight(params_, eyeAltitude) > 0.0)
    {
        atmosphereShellActive_ = false;
        return;
    }

    // Rotate the hemisphere's pole onto the eye direction and scale it to
    // the limb glow's visual radius: a quarter of the configured air column
    // above the surface, clamped to 2-8% of the radius so the band hugs the
    // horizon the way atmospheric density does instead of inflating with a
    // deep showcase atmosphere. Translate to the planet's world center.
    const glm::dvec3 eyeDirection = glm::normalize(eyePlanetLocal);
    glm::dvec3 tangent = glm::cross(eyeDirection, glm::dvec3(0.0, 0.0, 1.0));
    if (glm::length(tangent) < 1.0e-6)
    {
        tangent = glm::cross(eyeDirection, glm::dvec3(1.0, 0.0, 0.0));
    }
    tangent = glm::normalize(tangent);
    const glm::dvec3 third = glm::cross(tangent, eyeDirection);
    const glm::dmat3 rotation(tangent, eyeDirection, third);
    const double glowHeight = glm::clamp(0.25 * std::max(1.0, params_.atmosphereHeightMeters),
        0.02 * params_.radiusMeters, 0.08 * params_.radiusMeters);
    const double shellRadius = params_.radiusMeters + glowHeight;
    atmosphereShellTransform_ = glm::translate(glm::mat4(1.0f),
            glm::vec3(static_cast<float>(params_.centerMeters[0]), static_cast<float>(params_.centerMeters[1]),
                static_cast<float>(params_.centerMeters[2])))
        * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(shellRadius)));

    // The limb glows where the sun hits the visible side: tint the shell
    // with the same sun sky tint the dome uses (the night limb fades out).
    const double sunUp = sunEnabled && glm::length(sunDirection) > 1.0e-9
        ? glm::dot(glm::normalize(sunDirection), eyeDirection)
        : 1.0;
    const planet::Vec3 tint = planet::sunSkyTint(params_, sunUp, sunEnabled);
    renderer::MeshMaterial &material = atmosphereShellMaterials_.front();
    material.baseColor[0] = static_cast<float>(tint.x);
    material.baseColor[1] = static_cast<float>(tint.y);
    material.baseColor[2] = static_cast<float>(tint.z);
}

void PlanetChunkRenderer::ensureSkyCloudLayer(const glm::dvec3 &eyePlanetLocal)
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || skyCloudLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // The deck is built on the unit ball in the sky dome's INITIAL frame
    // (its +Y on the eye's local up, so band latitude equals elevation
    // angle); the dome's per-frame update rotates it with the sky and scales
    // it inside the dome's radius. A quad guard above the horizon keeps the
    // contour off the rim, where a deck edge would paint a hard line across
    // the haze band; the coverage field is sampled along the frame-rotated
    // directions (from the planet center, which for a standing eye differs
    // from the eye-local sky direction by well under the field's feature
    // size).
    constexpr std::uint32_t kSegments = 128;
    constexpr std::uint32_t kBands = 64;
    constexpr double kCoverageFloor = 0.42;
    constexpr double kHorizonGuardRadians = 0.05235987755982988; // 3 deg above the horizon

    if (cloudField_ == nullptr)
    {
        cloudField_ = std::make_unique<planet::CloudField>(params_);
    }

    // Initial dome frame: +Y onto the spawn up (same construction as the
    // dome's per-frame rotation).
    const glm::dvec3 upNormalized = glm::length(eyePlanetLocal) > 1.0e-9
        ? glm::normalize(eyePlanetLocal)
        : glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 tangent = glm::cross(upNormalized, glm::dvec3(0.0, 0.0, 1.0));
    if (glm::length(tangent) < 1.0e-6)
    {
        tangent = glm::cross(upNormalized, glm::dvec3(1.0, 0.0, 0.0));
    }
    tangent = glm::normalize(tangent);
    const glm::dvec3 third = glm::cross(tangent, upNormalized);

    const double frameRadius = std::max(1.0, params_.radiusMeters);
    const double vertexSpacing = glm::two_pi<double>() * frameRadius / static_cast<double>(kSegments);

    CoverageShellSpec spec;
    spec.segments = kSegments;
    spec.bands = kBands;
    spec.latitudeLowRadians = 0.0;
    spec.latitudeHighRadians = glm::pi<double>() * 0.5;
    spec.shellRadiusMeters = 1.0; // scaled per frame with the dome
    spec.lowPassCutoffMeters = vertexSpacing;
    spec.coverageFloor = kCoverageFloor;
    spec.minQuadLatitudeRadians = kHorizonGuardRadians;
    spec.hazeEdgeBlend = true;
    spec.frame = glm::dmat3(tangent, upNormalized, third);

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    if (!buildCoverageShellMesh(*cloudField_, params_, spec, vertices, indices))
    {
        return;
    }

    skyCloudLayer_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (skyCloudLayer_ == renderer::invalidMesh)
    {
        return;
    }
    skyCloudMaterials_.emplace_back();
    renderer::MeshMaterial &material = skyCloudMaterials_.front();
    material.unlit = true; // the dome's per-frame tint (sun elevation, haze weight) shades it
    skyCloudActive_ = true;
}

void PlanetChunkRenderer::setSkyCloudLayerVisible(bool visible)
{
    skyCloudVisible_ = visible;
}

void PlanetChunkRenderer::destroyAtmosphereShell()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && atmosphereShell_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, atmosphereShell_);
    }
    atmosphereShell_ = renderer::invalidMesh;
    atmosphereShellMaterials_.clear();
    atmosphereShellActive_ = false;
}

void PlanetChunkRenderer::destroySkyCloudLayer()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && skyCloudLayer_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, skyCloudLayer_);
    }
    skyCloudLayer_ = renderer::invalidMesh;
    skyCloudMaterials_.clear();
    skyCloudActive_ = false;
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
    destroyAtmosphereShell();
    destroySkyCloudLayer();
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
    // The dome vertices are unit-ball offsets: the placement adds the
    // planet's world center to the planet-local eye (world = planet-local +
    // center), so the dome follows the world eye exactly.
    const glm::vec3 worldEye(static_cast<float>(params_.centerMeters[0] + eyePlanetLocal.x),
        static_cast<float>(params_.centerMeters[1] + eyePlanetLocal.y),
        static_cast<float>(params_.centerMeters[2] + eyePlanetLocal.z));
    skyDomeTransform_ = glm::translate(glm::mat4(1.0f), worldEye)
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

    // Sky cloud deck rides the dome: same eye placement, same rotation, a
    // scale just inside the dome's rim (the gradient keeps reading behind
    // the deck's contoured gaps), and the dome's tint so sun elevation and
    // haze weight shade both layers identically.
    if (skyCloudActive_ && skyCloudLayer_ != renderer::invalidMesh && !skyCloudMaterials_.empty())
    {
        skyCloudTransform_ = glm::translate(glm::mat4(1.0f), worldEye)
            * glm::mat4(rotation)
            * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(domeRadius * 0.9)));
        renderer::MeshMaterial &skyMaterial = skyCloudMaterials_.front();
        skyMaterial.baseColor[0] = static_cast<float>(tint.x * weight);
        skyMaterial.baseColor[1] = static_cast<float>(tint.y * weight);
        skyMaterial.baseColor[2] = static_cast<float>(tint.z * weight);
    }

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
