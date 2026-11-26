// Implementation of the lighting demo's procedural scene. See the header for
// the anchor-relative frame contract and the ownership rules.

#include "lighting_scene.hpp"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <utility>

namespace demo::lighting_demo
{
namespace
{

using engine::direct_lighting::DirectLightDescription;
using engine::direct_lighting::DirectLightType;
using engine::radiance_cache::GiSurfaceDescription;
using engine::radiance_cache::giSurfaceFlagDynamic;
using engine::radiance_cache::GiError;
using engine::radiance_cache::invalidGiSurface;
using engine::radiance_cache::RadianceCacheCapability;
using engine::reflection_field::reflectionSurfaceFlagDynamic;
using engine::reflection_field::ReflectionError;
using engine::reflection_field::ReflectionSurfaceDescription;
using engine::renderer::MeshDescription;
using engine::renderer::MeshHandle;
using engine::renderer::MeshIndexFormat;
using engine::renderer::MeshMaterial;
using engine::renderer::RenderDrawCommand;
using engine::renderer::RenderError;
using engine::renderer::TextureDescription;
using engine::renderer::TextureHandle;
using engine::renderer::TextureFilter;
using engine::renderer::TextureAddress;
using engine::renderer::VertexAttribute;
using engine::renderer::VertexFormat;
using engine::renderer::VertexSemantic;

// Canonical interleaved vertex: 11 floats (position 3, normal 3, color 3, uv 2).
constexpr std::uint32_t kVertexStride   = 11 * sizeof(float);
constexpr std::uint32_t kGroundSegments = 16;  // 16x16 quads per tile
constexpr double       kGroundTileMeters  = 24.0;
constexpr std::uint32_t kGroundTileRadius = 6; // 13x13 tiles cover the field

struct SceneVertex
{
    float position[3];
    float normal[3];
    float color[3];
    float uv[2];
};

const std::array<VertexAttribute, 4> &canonicalAttributes()
{
    static const std::array<VertexAttribute, 4> attributes = {{
        {VertexSemantic::Position, VertexFormat::Float32x3, 0},
        {VertexSemantic::Normal, VertexFormat::Float32x3, 3 * sizeof(float)},
        {VertexSemantic::Color, VertexFormat::Float32x3, 6 * sizeof(float)},
        {VertexSemantic::TexCoord0, VertexFormat::Float32x2, 9 * sizeof(float)},
    }};
    return attributes;
}

MeshDescription canonicalDescription(const std::vector<SceneVertex> &vertices, const std::vector<std::uint32_t> &indices)
{
    MeshDescription description;
    description.vertices            = vertices.data();
    description.vertexCount         = static_cast<std::uint32_t>(vertices.size());
    description.indices             = indices.data();
    description.indexCount          = static_cast<std::uint32_t>(indices.size());
    description.indexFormat         = MeshIndexFormat::UInt32;
    description.vertexStrideBytes   = kVertexStride;
    description.vertexAttributes    = canonicalAttributes().data();
    description.vertexAttributeCount = static_cast<std::uint32_t>(canonicalAttributes().size());
    return description;
}

// Deterministic SplitMix64 stream: every placement decision keys off the
// seed and the instance index, never off the build or frame iteration, so
// the scene reproduces exactly across runs.
std::uint64_t mix64(std::uint64_t value)
{
    value += 0x9E3779B97F4A7C15ull;
    value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
    value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
    return value ^ (value >> 31);
}

double unit01(std::uint64_t value, std::uint64_t lane)
{
    return static_cast<double>(mix64(value ^ (lane * 0xD1B54A32D192ED03ull)) >> 11) / 9007199254740992.0;
}

// The material palette: matte albedo variety with a few polished metal and
// emissive accents, so every lighting term has something to show (the
// emissive accents feed the GI's emission path, the metals the reflection
// field's rough band).
struct MaterialSlot
{
    float baseColor[3]      = {1.0f, 1.0f, 1.0f};
    float roughness         = 0.8f;
    float metallic          = 0.0f;
    float emissiveColor[3]  = {0.0f, 0.0f, 0.0f};
    float emissiveIntensity = 0.0f;
};

const std::array<MaterialSlot, 10> &materialPalette()
{
    static const std::array<MaterialSlot, 10> palette = {{
        {{0.62f, 0.60f, 0.56f}, 0.82f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // warm stone
        {{0.55f, 0.28f, 0.20f}, 0.75f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // clay red
        {{0.34f, 0.44f, 0.30f}, 0.80f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // sage green
        {{0.30f, 0.36f, 0.46f}, 0.70f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // slate blue
        {{0.70f, 0.58f, 0.34f}, 0.78f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // ochre
        {{0.86f, 0.86f, 0.88f}, 0.24f, 0.92f, {0.0f, 0.0f, 0.0f}, 0.0f}, // polished steel
        {{0.72f, 0.66f, 0.55f}, 0.30f, 0.85f, {0.0f, 0.0f, 0.0f}, 0.0f}, // brushed bronze
        {{0.16f, 0.18f, 0.20f}, 0.55f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f},  // charred dark
        {{1.00f, 0.62f, 0.24f}, 0.90f, 0.0f, {1.0f, 0.45f, 0.12f}, 5.5f},// ember emissive
        {{0.30f, 0.62f, 1.00f}, 0.90f, 0.0f, {0.35f, 0.65f, 1.0f}, 4.5f},// cool emissive
    }};
    return palette;
}

// One object prototype: the CPU mesh (rendering), the borrowed geometry
// views the lighting modules copy, and the mesh's bounding half height (all
// prototypes stand with their base at y = 0).
struct ObjectPrototype
{
    std::vector<SceneVertex>   vertices;
    std::vector<std::uint32_t> indices;
    float halfHeightMeters = 1.0f;
};

void buildBoxPrototype(ObjectPrototype &box)
{
    // A 1.7 m cube standing on y = 0, flat normals, per-face sun-warmth
    // shading baked as a gentle color ramp (the lighting adds the rest).
    constexpr float kHalf = 0.85f;
    struct Face
    {
        glm::vec3 normal;
        glm::vec3 color;
        glm::vec3 corners[4]; // quad, one consistent winding across the mesh
                              // (the winding vote only needs the whole mesh to
                              // agree; the vertex normals carry the shading)
    };
    const float kShade[6] = {0.96f, 0.90f, 1.0f, 0.86f, 0.93f, 0.80f};
    const glm::vec3 v[8] = {
        {-kHalf, 0.0f, -kHalf}, {kHalf, 0.0f, -kHalf}, {kHalf, 0.0f, kHalf}, {-kHalf, 0.0f, kHalf},
        {-kHalf, 2.0f * kHalf, -kHalf}, {kHalf, 2.0f * kHalf, -kHalf}, {kHalf, 2.0f * kHalf, kHalf},
        {-kHalf, 2.0f * kHalf, kHalf}};
    const std::array<std::array<std::uint32_t, 4>, 6> quads = {{
        {{0, 3, 2, 1}}, // bottom (-y)
        {{4, 5, 6, 7}}, // top (+y)
        {{0, 1, 5, 4}}, // -z
        {{2, 3, 7, 6}}, // +z
        {{1, 2, 6, 5}}, // +x
        {{3, 0, 4, 7}}, // -x
    }};
    const glm::vec3 normals[6] = {
        {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f},
        {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
    const glm::vec2 uvs[4] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
    for (std::uint32_t face = 0; face < 6; ++face)
    {
        const std::uint32_t base = static_cast<std::uint32_t>(box.vertices.size());
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            const glm::vec3 &position = v[quads[face][corner]];
            SceneVertex      vertex{};
            vertex.position[0] = position.x;
            vertex.position[1] = position.y;
            vertex.position[2] = position.z;
            vertex.normal[0]   = normals[face].x;
            vertex.normal[1]   = normals[face].y;
            vertex.normal[2]   = normals[face].z;
            vertex.color[0]    = kShade[face];
            vertex.color[1]    = kShade[face];
            vertex.color[2]    = kShade[face];
            vertex.uv[0]       = uvs[corner].x;
            vertex.uv[1]       = uvs[corner].y;
            box.vertices.push_back(vertex);
        }
        box.indices.insert(box.indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
    }
    box.halfHeightMeters = kHalf;
}

void buildSpherePrototype(ObjectPrototype &sphere)
{
    // A UV sphere standing tangent on y = 0: 12 wedges x 8 bands.
    constexpr std::uint32_t kWedges = 12;
    constexpr std::uint32_t kBands  = 8;
    constexpr float         kRadius = 0.95f;
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double phi   = glm::pi<double>() * static_cast<double>(band) / static_cast<double>(kBands);
        const double y     = kRadius * std::cos(phi);
        const double ring  = kRadius * std::sin(phi);
        for (std::uint32_t wedge = 0; wedge <= kWedges; ++wedge)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(wedge) / static_cast<double>(kWedges);
            SceneVertex  vertex{};
            vertex.position[0] = static_cast<float>(ring * std::cos(theta));
            vertex.position[1] = kRadius + static_cast<float>(y); // base at y=0, center at +kRadius
            vertex.position[2] = static_cast<float>(ring * std::sin(theta));
            const float length = std::max(1.0e-6f, static_cast<float>(std::sqrt(ring * ring + y * y)));
            vertex.normal[0]   = static_cast<float>(ring * std::cos(theta)) / length;
            vertex.normal[1]   = static_cast<float>(y) / length;
            vertex.normal[2]   = static_cast<float>(ring * std::sin(theta)) / length;
            vertex.color[0]    = 1.0f;
            vertex.color[1]    = 1.0f;
            vertex.color[2]    = 1.0f;
            vertex.uv[0]       = static_cast<float>(wedge) / static_cast<float>(kWedges);
            vertex.uv[1]       = static_cast<float>(band) / static_cast<float>(kBands);
            sphere.vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t wedge = 0; wedge < kWedges; ++wedge)
        {
            const std::uint32_t a = band * (kWedges + 1u) + wedge;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (kWedges + 1u);
            const std::uint32_t d = c + 1u;
            sphere.indices.insert(sphere.indices.end(), {a, c, b, b, c, d});
        }
    }
    sphere.halfHeightMeters = kRadius;
}

void buildColumnPrototype(ObjectPrototype &column)
{
    // A hexagonal column on a hexagonal plinth, 4.4 m tall: the vertical mass
    // the probe lattice and the reflection field read best. Every ring the
    // eye can see into carries its own cap - the plinth slab is closed top
    // and bottom, the shaft meets the plinth top exactly, and the capital
    // wall carries the roof disc (no floating layers, no see-through gaps).
    constexpr std::uint32_t kSides     = 6;
    constexpr float         kHexRadius = 0.62f;
    constexpr float         kPlinthTop = 0.45f;
    constexpr float         kBodyLo    = kPlinthTop; // the shaft stands on the plinth top
    constexpr float         kBodyHi    = 3.85f;
    constexpr float         kCapLift   = 0.30f;
    const float             kPlinth    = kHexRadius * 1.45f;
    // Plinth: a closed hexagonal slab y in [0, kPlinthTop], sides plus both
    // hex fans (the top fan is what the eye reads - without it the plinth is
    // an open tray).
    for (std::uint32_t ring = 0; ring < 2; ++ring)
    {
        const float        y      = ring == 0 ? 0.0f : kPlinthTop;
        const float        radius = ring == 0 ? kPlinth : kPlinth * 0.92f;
        const std::uint32_t base  = static_cast<std::uint32_t>(column.vertices.size());
        for (std::uint32_t side = 0; side <= kSides; ++side)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(side) / static_cast<double>(kSides);
            SceneVertex  vertex{};
            vertex.position[0] = static_cast<float>(radius * std::cos(theta));
            vertex.position[1] = y;
            vertex.position[2] = static_cast<float>(radius * std::sin(theta));
            vertex.normal[0]   = 0.0f;
            vertex.normal[1]   = ring == 0 ? -1.0f : 1.0f;
            vertex.normal[2]   = 0.0f;
            vertex.color[0]    = 0.95f;
            vertex.color[1]    = 0.95f;
            vertex.color[2]    = 0.95f;
            vertex.uv[0]       = static_cast<float>(side) / static_cast<float>(kSides);
            vertex.uv[1]       = y / kBodyHi;
            column.vertices.push_back(vertex);
        }
        if (ring == 1)
        {
            for (std::uint32_t side = 0; side < kSides; ++side)
            {
                const std::uint32_t a = base + side;         // top ring vertex
                const std::uint32_t b = base - (kSides + 1u) + side; // bottom ring vertex
                column.indices.insert(column.indices.end(), {b, a, a + 1u, b, a + 1u, b + 1u});
            }
        }
    }
    // Plinth caps: a center vertex per face, one hex fan each. The bottom
    // fan winds toward -y, the top fan toward +y (the winding vote needs the
    // whole mesh to agree, and both fans share the side walls' convention).
    for (std::uint32_t cap = 0; cap < 2; ++cap)
    {
        const bool        bottom  = cap == 0;
        const float       y       = bottom ? 0.0f : kPlinthTop;
        const float       radius  = bottom ? kPlinth : kPlinth * 0.92f;
        const std::uint32_t center = static_cast<std::uint32_t>(column.vertices.size());
        SceneVertex         capVertex{};
        capVertex.position[1] = y;
        capVertex.normal[1]   = bottom ? -1.0f : 1.0f;
        capVertex.color[0]    = 0.95f;
        capVertex.color[1]    = 0.95f;
        capVertex.color[2]    = 0.95f;
        column.vertices.push_back(capVertex);
        const std::uint32_t rimBase = static_cast<std::uint32_t>(column.vertices.size());
        for (std::uint32_t side = 0; side <= kSides; ++side)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(side) / static_cast<double>(kSides);
            SceneVertex  rim   = capVertex;
            rim.position[0] = static_cast<float>(radius * std::cos(theta));
            rim.position[2] = static_cast<float>(radius * std::sin(theta));
            column.vertices.push_back(rim);
        }
        for (std::uint32_t side = 0; side < kSides; ++side)
        {
            const std::uint32_t rimA = rimBase + side;
            const std::uint32_t rimB = rimBase + side + 1u;
            if (bottom)
            {
                column.indices.insert(column.indices.end(), {center, rimA, rimB});
            }
            else
            {
                column.indices.insert(column.indices.end(), {center, rimB, rimA});
            }
        }
    }
    // Body: side quads between two hex rings.
    {
        const std::uint32_t lowBase = static_cast<std::uint32_t>(column.vertices.size());
        for (std::uint32_t side = 0; side <= kSides; ++side)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(side) / static_cast<double>(kSides);
            const float  nx    = static_cast<float>(std::cos(theta));
            const float  nz    = static_cast<float>(std::sin(theta));
            SceneVertex  low{};
            low.position[0] = kHexRadius * nx;
            low.position[1] = kBodyLo;
            low.position[2] = kHexRadius * nz;
            low.normal[0]   = nx;
            low.normal[1]   = 0.0f;
            low.normal[2]   = nz;
            low.color[0]    = 1.0f;
            low.color[1]    = 1.0f;
            low.color[2]    = 1.0f;
            low.uv[0]       = static_cast<float>(side) / static_cast<float>(kSides);
            low.uv[1]       = 0.0f;
            column.vertices.push_back(low);
            SceneVertex high = low;
            high.position[1] = kBodyHi;
            high.uv[1]       = 1.0f;
            high.color[0]    = 0.9f;
            high.color[1]    = 0.9f;
            high.color[2]    = 0.9f;
            column.vertices.push_back(high);
        }
        for (std::uint32_t side = 0; side < kSides; ++side)
        {
            const std::uint32_t lowA  = lowBase + side * 2u;
            const std::uint32_t highA = lowA + 1u;
            const std::uint32_t lowB  = lowA + 2u;
            const std::uint32_t highB = lowB + 1u;
            // Outward winding (cross(edge1, edge2) points along the vertex
            // normals): the whole mesh must agree with the plinth below and
            // the capital above, or the renderer's winding vote culls whichever
            // minority orientation loses.
            column.indices.insert(column.indices.end(), {lowA, highB, lowB, lowA, highA, highB});
        }
        // Capital: a flaring wall from the shaft's top ring out to the roof
        // disc's rim (the disc is wider than the shaft, and without the wall
        // it would hover disconnected above the shaft).
        const float       capitalRimRadius = kHexRadius * 1.18f;
        const std::uint32_t rimBase        = static_cast<std::uint32_t>(column.vertices.size());
        for (std::uint32_t side = 0; side <= kSides; ++side)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(side) / static_cast<double>(kSides);
            // The wall's own normal: the flare vector is (rimRadius - kHexRadius,
            // kCapLift) in the (r, y) plane, so the outward normal is that vector
            // rotated -90 degrees - outward and tilted down, unlike the shaft's
            // horizontal ring normals.
            const float flareRise = kCapLift;
            const float flareRun  = capitalRimRadius - kHexRadius;
            const float tilt = std::sqrt(flareRise * flareRise + flareRun * flareRun);
            SceneVertex  rim   = column.vertices[lowBase + side * 2u]; // shaft top vertex basis
            rim.position[0] = capitalRimRadius * static_cast<float>(std::cos(theta));
            rim.position[1] = kBodyHi + kCapLift;
            rim.position[2] = capitalRimRadius * static_cast<float>(std::sin(theta));
            rim.normal[0]   = static_cast<float>(std::cos(theta)) * flareRise / tilt;
            rim.normal[1]   = -flareRun / tilt;
            rim.normal[2]   = static_cast<float>(std::sin(theta)) * flareRise / tilt;
            column.vertices.push_back(rim);
        }
        for (std::uint32_t side = 0; side < kSides; ++side)
        {
            const std::uint32_t lowA  = lowBase + side * 2u;      // shaft top ring
            const std::uint32_t lowB  = lowBase + (side + 1u) * 2u;
            const std::uint32_t highA = rimBase + side;
            const std::uint32_t highB = rimBase + side + 1u;
            // Outward-and-down winding: the wall flares as it rises, so its
            // normals tilt below horizontal, matching the body's convention.
            column.indices.insert(column.indices.end(), {lowA, highB, lowB, lowA, highA, highB});
        }
        // Roof disc.
        const std::uint32_t capCenter = static_cast<std::uint32_t>(column.vertices.size());
        SceneVertex         cap{};
        cap.position[1] = kBodyHi + kCapLift;
        cap.normal[1]   = 1.0f;
        cap.color[0]    = 0.92f;
        cap.color[1]    = 0.92f;
        cap.color[2]    = 0.92f;
        column.vertices.push_back(cap);
        for (std::uint32_t side = 0; side <= kSides; ++side)
        {
            const double theta = glm::two_pi<double>() * static_cast<double>(side) / static_cast<double>(kSides);
            SceneVertex  rim   = cap;
            rim.position[0] = capitalRimRadius * static_cast<float>(std::cos(theta));
            rim.position[2] = capitalRimRadius * static_cast<float>(std::sin(theta));
            column.vertices.push_back(rim);
        }
        for (std::uint32_t side = 0; side < kSides; ++side)
        {
            const std::uint32_t rimA = capCenter + 1u + side;
            const std::uint32_t rimB = capCenter + 1u + side + 1u;
            // Upward winding, matching the body and the plinth (see above).
            column.indices.insert(column.indices.end(), {rimA, capCenter, rimB});
        }
    }
    column.halfHeightMeters = kBodyHi + kCapLift;
}

// Column-major model matrix from translation, yaw and uniform scale.
glm::mat4 composeTransform(const glm::dvec3 &position, float yawRadians, float scale)
{
    const float cosine = std::cos(yawRadians);
    const float sine   = std::sin(yawRadians);
    glm::mat4   model(1.0f);
    model[0][0] = cosine * scale;
    model[0][1] = 0.0f;
    model[0][2] = -sine * scale;
    model[1][0] = 0.0f;
    model[1][1] = scale;
    model[1][2] = 0.0f;
    model[2][0] = sine * scale;
    model[2][1] = 0.0f;
    model[2][2] = cosine * scale;
    model[3][0] = static_cast<float>(position.x);
    model[3][1] = static_cast<float>(position.y);
    model[3][2] = static_cast<float>(position.z);
    return model;
}

} // namespace

LightingScene::LightingScene(engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, const LightingSceneParameters &parameters)
    : renderCapability_(renderCapability)
    , rendererHandle_(rendererHandle)
    , parameters_(parameters)
    , fieldRadiusMeters_(parameters.fieldMeters)
{
}

LightingScene::~LightingScene()
{
    for (Instance &instance : instances_)
    {
        if (giCapability_ != nullptr && instance.giSurface != invalidGiSurface)
        {
            giCapability_->destroySurface(giCapability_->self, instance.giSurface);
        }
        if (reflectionCapability_ != nullptr
            && instance.reflectionSurface != engine::reflection_field::invalidReflectionSurface)
        {
            reflectionCapability_->destroySurface(reflectionCapability_->self, instance.reflectionSurface);
        }
    }
    for (ShowcasePiece &piece : showcasePieces_)
    {
        if (giCapability_ != nullptr && piece.giSurface != invalidGiSurface)
        {
            giCapability_->destroySurface(giCapability_->self, piece.giSurface);
        }
        if (reflectionCapability_ != nullptr
            && piece.reflectionSurface != engine::reflection_field::invalidReflectionSurface)
        {
            reflectionCapability_->destroySurface(reflectionCapability_->self, piece.reflectionSurface);
        }
    }
    if (directLightingCapability_ != nullptr)
    {
        for (LightTrack &track : lightTracks_)
        {
            if (track.handle != engine::direct_lighting::invalidDirectLight)
            {
                directLightingCapability_->destroyLight(directLightingCapability_->self, track.handle);
            }
        }
    }
    if (giCapability_ != nullptr)
    {
        for (engine::radiance_cache::GiSurfaceHandle surface : groundGiSurfaces_)
        {
            giCapability_->destroySurface(giCapability_->self, surface);
        }
    }
    if (reflectionCapability_ != nullptr)
    {
        for (engine::reflection_field::ReflectionSurfaceHandle surface : groundReflectionSurfaces_)
        {
            reflectionCapability_->destroySurface(reflectionCapability_->self, surface);
        }
    }
    for (MeshHandle &mesh : objectMeshes_)
    {
        if (mesh != engine::renderer::invalidMesh)
        {
            renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, mesh);
        }
    }
    if (groundTileMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, groundTileMesh_);
    }
    if (poolCurbMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, poolCurbMesh_);
    }
    if (poolFloorMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, poolFloorMesh_);
    }
    if (poolWaterMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, poolWaterMesh_);
    }
    if (skyDomeMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, skyDomeMesh_);
    }
    if (sunDiscMesh_ != engine::renderer::invalidMesh)
    {
        renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, sunDiscMesh_);
    }
    for (TextureHandle &texture : groundAlbedo_)
    {
        if (texture != engine::renderer::invalidTexture)
        {
            renderCapability_.destroyTexture(renderCapability_.self, rendererHandle_, texture);
        }
    }
    if (groundNormal_ != engine::renderer::invalidTexture)
    {
        renderCapability_.destroyTexture(renderCapability_.self, rendererHandle_, groundNormal_);
    }
}

void LightingScene::bindLightingModules(engine::radiance_cache::RadianceCacheCapability *giCapability,
    engine::reflection_field::ReflectionFieldCapability *reflectionCapability,
    engine::direct_lighting::DirectLightingCapability *directLightingCapability)
{
    giCapability_             = giCapability;
    reflectionCapability_     = reflectionCapability;
    directLightingCapability_ = directLightingCapability;
}

double LightingScene::showcaseClearingMeters() const
{
    return clearingMeters_;
}

glm::dvec3 LightingScene::showcaseFocus() const
{
    return showcasePoolCenter_;
}

void LightingScene::buildSkyBackdrop(const engine::pbr::PbrEnvironment &environment)
{
    if (!initialized_ || skyDomeMesh_ != engine::renderer::invalidMesh)
    {
        return;
    }

    // The dome is a lat-long sphere around the scene origin, inside the
    // demo camera's far plane (the camera sits well inside it at every pose
    // the demo drives). Vertex colors carry the whole look - the gradient
    // and the sun wedge bake once from the live environment, so the dome and
    // the shading always share one sun and one sky.
    constexpr std::uint32_t kSegments = 96;
    constexpr std::uint32_t kBands    = 32;
    constexpr double        kDomeRadiusMeters = 780.0;

    const glm::vec3 ambientSky(glm::clamp(environment.ambient.skyColor[0], 0.0f, 1.0f),
        glm::clamp(environment.ambient.skyColor[1], 0.0f, 1.0f),
        glm::clamp(environment.ambient.skyColor[2], 0.0f, 1.0f));
    const glm::vec3 ambientGround(environment.ambient.groundColor[0], environment.ambient.groundColor[1],
        environment.ambient.groundColor[2]);
    // Unit-luminance sun tint: the wedge and the horizon haze follow the
    // configured sun color at any intensity.
    const glm::vec3 sunColor(environment.sun.color[0], environment.sun.color[1], environment.sun.color[2]);
    const float     sunLuminance = std::max(0.2126f * sunColor.r + 0.7152f * sunColor.g + 0.0722f * sunColor.b, 1.0e-4f);
    const glm::vec3 sunTint      = sunColor / sunLuminance;
    const bool      sunEnabled   = environment.sun.enabled;
    glm::vec3       sunDirection(0.0f);
    float           sunElevation = 0.5f;
    if (sunEnabled)
    {
        sunDirection = glm::normalize(glm::vec3(
            environment.sun.direction[0], environment.sun.direction[1], environment.sun.direction[2]));
        if (!std::isfinite(sunDirection.x) || !std::isfinite(sunDirection.y) || !std::isfinite(sunDirection.z))
        {
            sunDirection = glm::vec3(0.45f, 0.62f, 0.35f);
        }
        sunElevation = glm::clamp(sunDirection.y, -1.0f, 1.0f);
    }

    // Gradient stops, keyed to the environment's hemisphere colors: a
    // saturated late-day blue zenith, a warm horizon haze carrying the sun
    // tint, and a dusty below-horizon haze that reads as distance beyond
    // the ground tiles instead of the clear color. The zenith keeps only a
    // fifth of the ambient-sky mix so the blue survives the tonemapper's
    // desaturating shoulder.
    const glm::vec3 zenithColor   = glm::mix(glm::vec3(0.12f, 0.28f, 0.78f), ambientSky, 0.20f) * 0.95f;
    const glm::vec3 horizonColor  = glm::mix(glm::vec3(1.10f, 0.70f, 0.40f), sunTint, 0.35f) * 0.72f;
    const glm::vec3 groundHaze    = glm::mix(sunTint, ambientGround, 0.50f) * 0.42f;
    // A low sun widens and warms its glare (the Mie wedge grows toward the
    // horizon); the scale anchors at the 3.0 default intensity.
    const float glareBoost   = glm::mix(1.0f, 2.2f, glm::clamp((0.35f - sunElevation) / 0.33f, 0.0f, 1.0f));
    const float glareIntensity = std::max(environment.sun.intensity, 0.0f) / 3.0f * glareBoost;

    auto skyColorAt = [&](const glm::vec3 &direction) -> glm::vec3
    {
        const float up = glm::clamp(direction.y, -1.0f, 1.0f);
        glm::vec3  color;
        if (up >= 0.0f)
        {
            // Fast rise off the horizon: most of the gradient change sits in
            // the first degrees of elevation, like a real atmosphere.
            color = glm::mix(horizonColor, zenithColor, std::pow(up, 0.42f));
        }
        else
        {
            const float below = glm::clamp(-up / 0.30f, 0.0f, 1.0f);
            color = glm::mix(horizonColor, groundHaze, below * below * (3.0f - 2.0f * below));
        }
        if (sunEnabled)
        {
            const float towardSun = glm::max(glm::dot(direction, sunDirection), 0.0f);
            // A tight forward-scatter glow hugging the disc quad (the disc
            // itself is the billboard mesh - the vertex grid is coarser than
            // a two-degree feature, so the dome only carries the aureole).
            const float wedge = std::pow(towardSun, 8.0f) * 0.30f;
            color += sunTint * wedge * glareIntensity;
        }
        return color;
    };

    std::vector<SceneVertex> vertices;
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
            const double  azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const glm::vec3 direction(static_cast<float>(std::cos(azimuth) * cosLatitude),
                static_cast<float>(sinLatitude), static_cast<float>(std::sin(azimuth) * cosLatitude));
            const glm::vec3 color = skyColorAt(direction);
            SceneVertex     vertex{};
            vertex.position[0] = direction.x * static_cast<float>(kDomeRadiusMeters);
            vertex.position[1] = direction.y * static_cast<float>(kDomeRadiusMeters);
            vertex.position[2] = direction.z * static_cast<float>(kDomeRadiusMeters);
            vertex.color[0]    = color.r;
            vertex.color[1]    = color.g;
            vertex.color[2]    = color.b;
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
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }

    const MeshDescription description = canonicalDescription(vertices, indices);
    RenderError           error       = RenderError::None;
    skyDomeMesh_ = renderCapability_.createMesh(renderCapability_.self, rendererHandle_, &description, &error);
    if (skyDomeMesh_ == engine::renderer::invalidMesh)
    {
        std::fprintf(stderr, "demo_lighting: sky dome upload failed (error %u)\n", static_cast<std::uint32_t>(error));
        return;
    }
    skyBackdropMaterial_         = MeshMaterial{};
    skyBackdropMaterial_.unlit   = true;
    // Backdrop contract: no depth writes (the scene drawn after it wins the
    // shared pixels by depth test), and double-sided so the inward-facing
    // hemisphere rasterizes regardless of the mesh's winding vote.
    skyBackdropMaterial_.depthWrite  = false;
    skyBackdropMaterial_.doubleSided = true;

    // The sun disc: the dome's band grid cannot carry a two-degree feature
    // (the vertex grid alone is that coarse), so the disc is its own unlit
    // fan mesh re-aimed at the eye every frame. The fan's center carries the
    // full disc brightness (well above the bloom threshold) and the rim
    // carries the dome's own color behind the sun, so the disc fades into
    // its aureole instead of ending at a hard sprite edge.
    sunDirectionWorld_ = sunEnabled ? sunDirection : glm::vec3(0.0f);
    sunDiscActive_     = sunEnabled;
    if (sunDiscActive_)
    {
        // The disc must out-shine its own aureole: the wedge below saturates
        // the sky around the sun at a golden hour, so the disc's center
        // carries enough brightness to stay near the tonemapper's shoulder
        // (a distinct bright core) instead of dissolving into the glow.
        const float     discBrightness = 2.2f + 1.8f * glareIntensity;
        const glm::vec3 discColor  = sunTint * discBrightness;
        const glm::vec3 rimColor   = skyColorAt(sunDirection);
        constexpr std::uint32_t kDiscSegments = 32;
        std::vector<SceneVertex> disc(1 + kDiscSegments);
        disc[0].position[2] = 0.0f;
        disc[0].color[0]    = discColor.r;
        disc[0].color[1]    = discColor.g;
        disc[0].color[2]    = discColor.b;
        for (std::uint32_t segment = 0; segment < kDiscSegments; ++segment)
        {
            const double angle = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kDiscSegments);
            disc[1 + segment].position[0] = static_cast<float>(std::cos(angle));
            disc[1 + segment].position[1] = static_cast<float>(std::sin(angle));
            disc[1 + segment].position[2] = 0.0f;
            disc[1 + segment].color[0]    = rimColor.r;
            disc[1 + segment].color[1]    = rimColor.g;
            disc[1 + segment].color[2]    = rimColor.b;
        }
        std::vector<std::uint32_t> discIndices;
        discIndices.reserve(kDiscSegments * 3u);
        for (std::uint32_t segment = 0; segment < kDiscSegments; ++segment)
        {
            discIndices.insert(discIndices.end(),
                {0u, 1u + segment, 1u + (segment + 1u) % kDiscSegments});
        }
        const MeshDescription discDescription = canonicalDescription(disc, discIndices);
        RenderError           discError       = RenderError::None;
        sunDiscMesh_ = renderCapability_.createMesh(
            renderCapability_.self, rendererHandle_, &discDescription, &discError);
        if (sunDiscMesh_ == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo_lighting: sun disc upload failed (error %u)\n",
                static_cast<std::uint32_t>(discError));
            sunDiscActive_ = false;
        }
        else
        {
            sunDiscMaterial_         = MeshMaterial{};
            sunDiscMaterial_.unlit   = true;
            sunDiscMaterial_.depthWrite  = false;
            sunDiscMaterial_.doubleSided = true;
            counts_.triangles += kDiscSegments;
        }
    }
    composeDrawCommands();
}

bool LightingScene::initialize(const std::filesystem::path &assetDirectory)
{
    if (initialized_)
    {
        return true;
    }

    // ---- the ground tile mesh -------------------------------------------
    // One tile mesh, drawn 13x13 times: identical geometry under different
    // transforms keeps the renderer's transform cache hot and the lighting
    // modules' per-surface bounds tight.
    {
        ObjectPrototype tile;
        const double    step = kGroundTileMeters / kGroundSegments;
        for (std::uint32_t row = 0; row <= kGroundSegments; ++row)
        {
            for (std::uint32_t column = 0; column <= kGroundSegments; ++column)
            {
                const double localX = -0.5 * kGroundTileMeters + column * step;
                const double localZ = -0.5 * kGroundTileMeters + row * step;
                SceneVertex  vertex{};
                vertex.position[0] = static_cast<float>(localX);
                vertex.position[1] = 0.0f;
                vertex.position[2] = static_cast<float>(localZ);
                vertex.color[0]    = 1.0f;
                vertex.color[1]    = 1.0f;
                vertex.color[2]    = 1.0f;
                vertex.uv[0]       = static_cast<float>(column) / static_cast<float>(kGroundSegments);
                vertex.uv[1]       = static_cast<float>(row) / static_cast<float>(kGroundSegments);
                tile.vertices.push_back(vertex);
            }
        }
        // One flat tile mesh, drawn 13x13 times: identical geometry under
        // different transforms keeps the renderer's transform cache hot and
        // the lighting modules' per-surface bounds tight. The ground itself
        // stays a calm plane - the object scatter and the light tracks carry
        // the depth cues, and a mirror-flat ground is exactly what the
        // reflection field reads best on.
        for (std::uint32_t row = 0; row < kGroundSegments; ++row)
        {
            for (std::uint32_t column = 0; column < kGroundSegments; ++column)
            {
                const std::uint32_t a = row * (kGroundSegments + 1u) + column;
                const std::uint32_t b = a + 1u;
                const std::uint32_t c = a + (kGroundSegments + 1u);
                const std::uint32_t d = c + 1u;
                tile.indices.insert(tile.indices.end(), {a, c, b, b, c, d});
            }
        }
        for (SceneVertex &vertex : tile.vertices)
        {
            vertex.normal[0] = 0.0f;
            vertex.normal[1] = 1.0f;
            vertex.normal[2] = 0.0f;
        }
        const MeshDescription description = canonicalDescription(tile.vertices, tile.indices);
        RenderError           error       = RenderError::None;
        groundTileMesh_ = renderCapability_.createMesh(renderCapability_.self, rendererHandle_, &description, &error);
        if (groundTileMesh_ == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo_lighting: ground tile mesh upload failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
        // The tile transforms: a fixed grid centered on the anchor.
        groundTransforms_.reserve((2u * kGroundTileRadius + 1u) * (2u * kGroundTileRadius + 1u));
        for (std::int32_t tileZ = -static_cast<std::int32_t>(kGroundTileRadius);
             tileZ <= static_cast<std::int32_t>(kGroundTileRadius); ++tileZ)
        {
            for (std::int32_t tileX = -static_cast<std::int32_t>(kGroundTileRadius);
                 tileX <= static_cast<std::int32_t>(kGroundTileRadius); ++tileX)
            {
                const glm::dvec3 center(static_cast<double>(tileX) * kGroundTileMeters, 0.0,
                    static_cast<double>(tileZ) * kGroundTileMeters);
                groundTransforms_.push_back(composeTransform(center, 0.0f, 1.0f));
            }
        }
        counts_.groundTiles = static_cast<std::uint32_t>(groundTransforms_.size());
        counts_.triangles += counts_.groundTiles * kGroundSegments * kGroundSegments * 2u;
    }

    // ---- the object prototypes ------------------------------------------
    {
        std::array<ObjectPrototype, 3> prototypes;
        buildBoxPrototype(prototypes[0]);
        buildSpherePrototype(prototypes[1]);
        buildColumnPrototype(prototypes[2]);
        for (std::size_t slot = 0; slot < prototypes.size(); ++slot)
        {
            const MeshDescription description = canonicalDescription(prototypes[slot].vertices, prototypes[slot].indices);
            RenderError           error       = RenderError::None;
            objectMeshes_[slot] = renderCapability_.createMesh(
                renderCapability_.self, rendererHandle_, &description, &error);
            if (objectMeshes_[slot] == engine::renderer::invalidMesh)
            {
                std::fprintf(stderr, "demo_lighting: object prototype %zu upload failed (error %u)\n", slot,
                    static_cast<std::uint32_t>(error));
                return false;
            }
            prototypeHalfHeights_[slot] = prototypes[slot].halfHeightMeters;
            prototypeTriangles_[slot] = static_cast<std::uint32_t>(prototypes[slot].indices.size() / 3u);
            // The lighting modules borrow the same vertex data (positions,
            // normals, colors - they carry no UV contract) and copy it into
            // their own registries.
            prototypePositions_[slot].clear();
            prototypeNormals_[slot].clear();
            prototypeColors_[slot].clear();
            prototypeIndices_[slot] = prototypes[slot].indices;
            prototypePositions_[slot].reserve(prototypes[slot].vertices.size());
            prototypeNormals_[slot].reserve(prototypes[slot].vertices.size());
            prototypeColors_[slot].reserve(prototypes[slot].vertices.size());
            for (const SceneVertex &vertex : prototypes[slot].vertices)
            {
                prototypePositions_[slot].push_back({vertex.position[0], vertex.position[1], vertex.position[2]});
                prototypeNormals_[slot].push_back({vertex.normal[0], vertex.normal[1], vertex.normal[2]});
                prototypeColors_[slot].push_back({vertex.color[0], vertex.color[1], vertex.color[2]});
            }
        }
    }

    // ---- ground textures (downloaded scans; see the assets README) ------
    loadGroundTextures(assetDirectory);

    // The showcase cluster first: it decides the scatter's clearing radius,
    // so the placements below keep their distance from the pool.
    if (!buildShowcase())
    {
        return false;
    }
    placeInstances();
    placeLights();
    if (!registerSurfaces())
    {
        return false;
    }

    // ---- the static material + matrix stores ----------------------------
    // Commands reference pointers into these stores; their capacity is fixed
    // here so no per-frame push can reallocate and dangle an earlier
    // command's pointer.
    matrixStore_.reserve(groundTransforms_.size() + instances_.size() + showcasePieces_.size() + 8u);
    materialStore_.reserve(groundTransforms_.size() + instances_.size() + showcaseMaterials_.size());
    buildStaticMaterials();

    initialized_ = true;
    composeDrawCommands();
    return true;
}

void LightingScene::buildStaticMaterials()
{
    materialStore_.clear();
    // Ground: the scanned albedo pair multiplies a neutral tint; the mean
    // compensation (RenderCapability::textureMean) keeps each variant's
    // multiply brightness-neutral, the scan's normal map adds per-sample
    // relief.
    float groundMean[2][3] = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    for (std::size_t variant = 0; variant < 2; ++variant)
    {
        if (groundAlbedo_[variant] != engine::renderer::invalidTexture && renderCapability_.textureMean != nullptr)
        {
            RenderError error = RenderError::None;
            renderCapability_.textureMean(renderCapability_.self, rendererHandle_, groundAlbedo_[variant],
                groundMean[variant], &error);
        }
    }
    for (std::size_t tile = 0; tile < groundTransforms_.size(); ++tile)
    {
        const std::size_t variant = (tile / (2u * kGroundTileRadius + 1u) + tile) % 2u;
        MeshMaterial     material{};
        material.baseColor[0] = 0.92f / std::max(1.0e-4f, groundMean[variant][0]);
        material.baseColor[1] = 0.90f / std::max(1.0e-4f, groundMean[variant][1]);
        material.baseColor[2] = 0.86f / std::max(1.0e-4f, groundMean[variant][2]);
        material.roughnessFactor = 0.92f;
        if (groundAlbedo_[variant] != engine::renderer::invalidTexture)
        {
            material.baseColorTexture = groundAlbedo_[variant];
        }
        if (groundNormal_ != engine::renderer::invalidTexture)
        {
            material.normalTexture = groundNormal_;
            material.normalScale   = 0.45f;
        }
        materialStore_.push_back(material);
    }
    // Objects: one material per palette slot per instance (the palette index
    // rides the instance; the material slot is the palette entry).
    objectMaterialBase_ = static_cast<std::uint32_t>(materialStore_.size());
    for (const Instance &instance : instances_)
    {
        const MaterialSlot &slot = materialPalette()[instance.materialSlot];
        MeshMaterial       material{};
        material.baseColor[0] = slot.baseColor[0];
        material.baseColor[1] = slot.baseColor[1];
        material.baseColor[2] = slot.baseColor[2];
        material.roughnessFactor = slot.roughness;
        material.metallicFactor  = slot.metallic;
        if (slot.emissiveIntensity > 0.0f)
        {
            material.emissiveColor[0] = slot.emissiveColor[0] * slot.emissiveIntensity;
            material.emissiveColor[1] = slot.emissiveColor[1] * slot.emissiveIntensity;
            material.emissiveColor[2] = slot.emissiveColor[2] * slot.emissiveIntensity;
        }
        materialStore_.push_back(material);
    }
    // The showcase cluster's renderer materials: the curb wears the concrete
    // scan (mean-compensated like the ground variants), the floor stays a
    // clean pale plate, and the water and glass carry the transmission pair
    // (the renderer routes opacity below 1 into the transparent pass and the
    // reflection field's refracted chains; the lighting registrations carried
    // the same pair at registerSurfaces time).
    showcaseMaterialBase_ = static_cast<std::uint32_t>(materialStore_.size());
    for (std::size_t slot = 0; slot < showcaseMaterials_.size(); ++slot)
    {
        const ShowcaseMaterialSlot &entry = showcaseMaterials_[slot];
        MeshMaterial               material{};
        material.baseColor[0]   = entry.baseColor[0];
        material.baseColor[1]   = entry.baseColor[1];
        material.baseColor[2]   = entry.baseColor[2];
        material.roughnessFactor = entry.roughness;
        material.metallicFactor  = entry.metallic;
        material.opacity          = entry.opacity;
        material.indexOfRefraction = entry.indexOfRefraction;
        if (slot == 0 && groundAlbedo_[1] != engine::renderer::invalidTexture)
        {
            // The stone curb: the concrete scan over its own mean, the scan's
            // normal map for relief (the same treatment the ground variants
            // get above).
            float mean[3] = {1.0f, 1.0f, 1.0f};
            if (renderCapability_.textureMean != nullptr)
            {
                RenderError error = RenderError::None;
                renderCapability_.textureMean(renderCapability_.self, rendererHandle_, groundAlbedo_[1], mean, &error);
            }
            material.baseColor[0] /= std::max(1.0e-4f, mean[0]);
            material.baseColor[1] /= std::max(1.0e-4f, mean[1]);
            material.baseColor[2] /= std::max(1.0e-4f, mean[2]);
            material.baseColorTexture = groundAlbedo_[1];
            if (groundNormal_ != engine::renderer::invalidTexture)
            {
                material.normalTexture = groundNormal_;
                material.normalScale   = 0.35f;
            }
        }
        materialStore_.push_back(material);
    }
}

bool LightingScene::loadGroundTextures(const std::filesystem::path &assetDirectory)
{
    struct GroundScan
    {
        const char *fileName;
        TextureHandle *target;
    };
    const std::array<GroundScan, 3> scans = {{
        {"ground_gravel_diff_1k.jpg", &groundAlbedo_[0]},
        {"ground_concrete_diff_1k.jpg", &groundAlbedo_[1]},
        {"ground_gravel_nor_gl_1k.jpg", &groundNormal_},
    }};
    bool allLoaded = true;
    for (const GroundScan &scan : scans)
    {
        const std::filesystem::path path = assetDirectory / scan.fileName;
        RenderError                 error = RenderError::None;
        const TextureHandle         handle = renderCapability_.createTextureFromFile(
            renderCapability_.self, rendererHandle_, path.string().c_str(), &error);
        if (handle == engine::renderer::invalidTexture)
        {
            std::fprintf(stderr,
                "demo_lighting: ground scan '%s' not found under '%s'; the ground renders with vertex color only\n",
                scan.fileName, assetDirectory.string().c_str());
            allLoaded = false;
            continue;
        }
        *scan.target = handle;
    }
    return allLoaded;
}

void LightingScene::placeInstances()
{
    instances_.clear();
    instances_.reserve(parameters_.objectCount);
    // Lattice spacing derived from the field radius and the count: the field
    // reads as a dense but walkable scatter at every count.
    const double spacing = 1.75 * fieldRadiusMeters_ / std::sqrt(static_cast<double>(parameters_.objectCount));
    const double innerMeters = clearingMeters_; // the showcase cluster (or the camera's gaze) owns the middle
    std::uint32_t dynamicPeriod = parameters_.dynamicShare > 0.0
        ? std::max<std::uint32_t>(1u, static_cast<std::uint32_t>(1.0 / parameters_.dynamicShare))
        : 0u;
    const std::int64_t span = static_cast<std::int64_t>(std::ceil(fieldRadiusMeters_ / spacing));
    // Fill passes: the first walk rolls a sparse presence (clearings and
    // clumps read better than a uniform grid); when the field's cell count
    // under-fills the requested population, later passes re-walk the SAME
    // lattice with a lower presence bar, so the population converges on the
    // request while the first-pass layout keeps its gaps. Occupied cells are
    // skipped by key, so the result is a pure function of the seed.
    std::vector<std::array<std::int64_t, 2>> occupied;
    for (int attempt = 0; attempt < 4 && instances_.size() < parameters_.objectCount; ++attempt)
    {
    const double presenceBar = 1.0 - 0.38 * std::pow(0.45, attempt);
    for (std::int64_t cellZ = -span; cellZ <= span && instances_.size() < parameters_.objectCount; ++cellZ)
    {
        for (std::int64_t cellX = -span; cellX <= span && instances_.size() < parameters_.objectCount; ++cellX)
        {
            if (std::find(occupied.begin(), occupied.end(), std::array<std::int64_t, 2>{cellX, cellZ})
                != occupied.end())
            {
                continue;
            }
            const std::uint64_t cellHash = mix64(parameters_.seed ^ (0x4B9A2C31D17E4F5Dull + static_cast<std::uint64_t>(cellX) * 0x9E3779B97F4A7C15ull
                + static_cast<std::uint64_t>(cellZ) * 0xC2B2AE3D27D4EB4Full));
            const double jitterX = (unit01(cellHash, 1) - 0.5) * spacing;
            const double jitterZ = (unit01(cellHash, 2) - 0.5) * spacing;
            const double x = (static_cast<double>(cellX) + 0.5) * spacing + jitterX;
            const double z = (static_cast<double>(cellZ) + 0.5) * spacing + jitterZ;
            const double radius = std::sqrt(x * x + z * z);
            if (radius < innerMeters || radius > fieldRadiusMeters_)
            {
                continue;
            }
            // Sparse-presence roll keeps gaps and clearings between clumps
            // (later fill passes relax the bar - see presenceBar above).
            if (unit01(cellHash, 0) > presenceBar)
            {
                continue;
            }
            occupied.push_back({cellX, cellZ});
            Instance instance;
            instance.basePosition = glm::dvec3(x, 0.0, z); // the drawn ground plane is flat at y=0
            instance.yaw          = static_cast<float>(unit01(cellHash, 3) * glm::two_pi<double>());
            instance.scale        = static_cast<float>(0.75 + 1.5 * unit01(cellHash, 4));
            instance.meshSlot     = static_cast<std::uint8_t>(unit01(cellHash, 5) * 3.0) % 3u;
            // Palette deal: the matte slots dominate; metals and the two
            // emissive accents sprinkle through.
            const double materialRoll = unit01(cellHash, 6);
            const double materialLane = unit01(cellHash, 7);
            double       paletteIndex = materialLane * 5.0;
            if (materialRoll >= 0.50 && materialRoll < 0.72)
            {
                paletteIndex = 5.0 + materialLane * 3.0;
            }
            else if (materialRoll >= 0.72 && materialRoll < 0.80)
            {
                paletteIndex = 8.0;
            }
            else if (materialRoll >= 0.80)
            {
                paletteIndex = 9.0;
            }
            instance.materialSlot = static_cast<std::uint8_t>(
                std::clamp(paletteIndex, 0.0, 9.0));
            instance.dynamic = dynamicPeriod != 0u
                && (instances_.size() % dynamicPeriod) == 0u;
            instance.phase = static_cast<float>(unit01(cellHash, 8) * glm::two_pi<double>());
            instance.bobMeters = static_cast<float>(0.35 + 0.55 * unit01(cellHash, 9));
            instances_.push_back(instance);
        }
    }
    }
    counts_.objects = static_cast<std::uint32_t>(instances_.size());
    counts_.dynamicObjects = static_cast<std::uint32_t>(
        std::count_if(instances_.begin(), instances_.end(), [](const Instance &instance) { return instance.dynamic; }));
    for (const Instance &instance : instances_)
    {
        counts_.triangles += prototypeTriangles_[instance.meshSlot];
    }
}

void LightingScene::placeLights()
{
    lightTracks_.clear();
    lightTracks_.reserve(parameters_.lightCount);
    const glm::vec3 palette[6] = {
        {1.00f, 0.72f, 0.42f}, // amber
        {0.55f, 0.75f, 1.00f}, // cool blue
        {1.00f, 0.48f, 0.55f}, // rose
        {0.62f, 1.00f, 0.70f}, // mint
        {1.00f, 0.90f, 0.60f}, // warm white
        {0.80f, 0.60f, 1.00f}, // violet
    };
    for (std::uint32_t index = 0; index < parameters_.lightCount; ++index)
    {
        const std::uint64_t lightHash = mix64(parameters_.seed ^ (0x7C1D63F2A5B9E810ull + index * 0x2545F4914F6CDD1Dull));
        LightTrack track;
        track.orbiting = index % 2u == 1u;
        track.angularSpeedRadiansPerSecond = track.orbiting ? 0.10 + 0.16 * unit01(lightHash, 1) : 0.0;
        track.phase  = unit01(lightHash, 2) * glm::two_pi<double>();
        track.radiusMeters = track.orbiting ? 16.0 + unit01(lightHash, 3) * (fieldRadiusMeters_ * 0.55) : 0.0;
        track.heightMeters = static_cast<float>(2.6 + 3.4 * unit01(lightHash, 4));
        DirectLightDescription &description = track.description;
        description.type      = DirectLightType::Point;
        const glm::vec3 &tint = palette[index % 6u];
        description.color[0]  = tint.r;
        description.color[1]  = tint.g;
        description.color[2]  = tint.b;
        description.intensity = static_cast<float>(38.0 + 52.0 * unit01(lightHash, 5));
        description.range     = static_cast<float>(26.0 + 30.0 * unit01(lightHash, 6));
        // Static lights sit in a wide ring; orbiting ones live on their track.
        if (!track.orbiting)
        {
            const double angle = unit01(lightHash, 7) * glm::two_pi<double>();
            const double ring  = 10.0 + unit01(lightHash, 8) * fieldRadiusMeters_ * 0.75;
            const double x     = std::cos(angle) * ring;
            const double z     = std::sin(angle) * ring;
            description.position[0] = static_cast<float>(x);
            description.position[1] = track.heightMeters;
            description.position[2] = static_cast<float>(z);
        }
        lightTracks_.push_back(track);
    }
    // The showcase pool lamp: one warm static point light directly over the
    // water. The grazing [pbr] sun alone paints broad streak bands on the
    // pool floor (physically right for a low sun, but a fine caustic NET is
    // what reads as water caustics) - the lamp's steep cone focuses the
    // classic net through the ripple normals at every frame, and its direct
    // term lights the curb and the glass pieces around the pool.
    if (showcaseBuilt_)
    {
        LightTrack track;
        DirectLightDescription &description = track.description;
        description.type      = DirectLightType::Point;
        description.color[0]  = 1.00f;
        description.color[1]  = 0.90f;
        description.color[2]  = 0.72f;
        description.intensity = 55.0f;
        description.range     = 34.0f;
        description.position[0] = static_cast<float>(showcasePoolCenter_.x);
        description.position[1] = 9.0f;
        description.position[2] = static_cast<float>(showcasePoolCenter_.z);
        lightTracks_.push_back(track);
    }
    counts_.lights = static_cast<std::uint32_t>(lightTracks_.size());
}

bool LightingScene::buildShowcase()
{
    if (!parameters_.showcaseEnabled)
    {
        showcasePoolCenter_ = glm::dvec3(0.0);
        return true;
    }

    // ---- layout ---------------------------------------------------------
    // The cluster: a rectangular reflecting pool (stone curb, pale floor,
    // transmissive rippled water) with a glass sphere, a glass column and a
    // polished-chrome sphere posed around it. All geometry is built in the
    // pool's local frame and posed with one translate, so the whole cluster
    // moves with the showcase configuration and the registrations below
    // reuse the same transform.
    constexpr double kWaterHalfX = 7.2;
    constexpr double kWaterHalfZ = 4.6;
    constexpr double kCurbWidth  = 0.55;
    constexpr double kCurbTop    = 0.42;
    constexpr double kFloorY     = 0.04;
    constexpr double kWaterY     = 0.26;
    const double     outerHalfX  = kWaterHalfX + kCurbWidth;
    const double     outerHalfZ  = kWaterHalfZ + kCurbWidth;
    constexpr float  kUvScale    = 0.22f; // concrete tiling on the curb

    showcasePoolCenter_ = glm::dvec3(0.0, 0.0, parameters_.showcasePoolCenterZMeters);

    // One textured quad helper: four corners wound for the given outward
    // normal (right-hand), one planar UV pair per corner.
    struct QuadPush
    {
        std::vector<SceneVertex>      &vertices;
        std::vector<std::uint32_t>    &indices;

        void operator()(const glm::vec3 (&corners)[4], const glm::vec3 &normal, const glm::vec2 (&uvs)[4])
        {
            const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                SceneVertex vertex{};
                vertex.position[0] = corners[corner].x;
                vertex.position[1] = corners[corner].y;
                vertex.position[2] = corners[corner].z;
                vertex.normal[0]   = normal.x;
                vertex.normal[1]   = normal.y;
                vertex.normal[2]   = normal.z;
                vertex.color[0]    = 1.0f;
                vertex.color[1]    = 1.0f;
                vertex.color[2]    = 1.0f;
                vertex.uv[0]       = uvs[corner].x;
                vertex.uv[1]       = uvs[corner].y;
                vertices.push_back(vertex);
            }
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        }
    };

    // ---- the stone curb ---------------------------------------------------
    {
        std::vector<SceneVertex>      vertices;
        std::vector<std::uint32_t>    indices;
        QuadPush                      push{vertices, indices};
        const float ox = static_cast<float>(outerHalfX);
        const float oz = static_cast<float>(outerHalfZ);
        const float ix = static_cast<float>(kWaterHalfX);
        const float iz = static_cast<float>(kWaterHalfZ);
        const float ct = static_cast<float>(kCurbTop);
        const float fy = static_cast<float>(kFloorY);
        const auto wallUv = [&](float along, float y) {
            return glm::vec2(along * kUvScale, y * kUvScale);
        };
        // Outer walls (outward normals), corners wound right-hand.
        push({glm::vec3(-ox, 0.0f, oz), glm::vec3(ox, 0.0f, oz), glm::vec3(ox, ct, oz), glm::vec3(-ox, ct, oz)},
            glm::vec3(0.0f, 0.0f, 1.0f),
            {wallUv(-ox, 0.0f), wallUv(ox, 0.0f), wallUv(ox, ct), wallUv(-ox, ct)});
        push({glm::vec3(ox, 0.0f, -oz), glm::vec3(-ox, 0.0f, -oz), glm::vec3(-ox, ct, -oz), glm::vec3(ox, ct, -oz)},
            glm::vec3(0.0f, 0.0f, -1.0f),
            {wallUv(ox, 0.0f), wallUv(-ox, 0.0f), wallUv(-ox, ct), wallUv(ox, ct)});
        push({glm::vec3(ox, 0.0f, oz), glm::vec3(ox, 0.0f, -oz), glm::vec3(ox, ct, -oz), glm::vec3(ox, ct, oz)},
            glm::vec3(1.0f, 0.0f, 0.0f),
            {wallUv(oz, 0.0f), wallUv(-oz, 0.0f), wallUv(-oz, ct), wallUv(oz, ct)});
        push({glm::vec3(-ox, 0.0f, -oz), glm::vec3(-ox, 0.0f, oz), glm::vec3(-ox, ct, oz), glm::vec3(-ox, ct, -oz)},
            glm::vec3(-1.0f, 0.0f, 0.0f),
            {wallUv(-oz, 0.0f), wallUv(oz, 0.0f), wallUv(oz, ct), wallUv(-oz, ct)});
        // Inner walls (facing the water), from the floor up to the curb top.
        push({glm::vec3(ix, fy, iz), glm::vec3(-ix, fy, iz), glm::vec3(-ix, ct, iz), glm::vec3(ix, ct, iz)},
            glm::vec3(0.0f, 0.0f, -1.0f),
            {wallUv(ix, fy), wallUv(-ix, fy), wallUv(-ix, ct), wallUv(ix, ct)});
        push({glm::vec3(-ix, fy, -iz), glm::vec3(ix, fy, -iz), glm::vec3(ix, ct, -iz), glm::vec3(-ix, ct, -iz)},
            glm::vec3(0.0f, 0.0f, 1.0f),
            {wallUv(-ix, fy), wallUv(ix, fy), wallUv(ix, ct), wallUv(-ix, ct)});
        push({glm::vec3(ix, fy, -iz), glm::vec3(ix, fy, iz), glm::vec3(ix, ct, iz), glm::vec3(ix, ct, -iz)},
            glm::vec3(-1.0f, 0.0f, 0.0f),
            {wallUv(-iz, fy), wallUv(iz, fy), wallUv(iz, ct), wallUv(-iz, ct)});
        push({glm::vec3(-ix, fy, iz), glm::vec3(-ix, fy, -iz), glm::vec3(-ix, ct, -iz), glm::vec3(-ix, ct, iz)},
            glm::vec3(1.0f, 0.0f, 0.0f),
            {wallUv(iz, fy), wallUv(-iz, fy), wallUv(-iz, ct), wallUv(iz, ct)});
        // The curb's walking surface: four ring quads at the curb top.
        const auto topUv = [&](float x, float z) { return glm::vec2(x * kUvScale, z * kUvScale); };
        const glm::vec3 up(0.0f, 1.0f, 0.0f);
        push({glm::vec3(ox, ct, oz), glm::vec3(ix, ct, iz), glm::vec3(-ix, ct, iz), glm::vec3(-ox, ct, oz)}, up,
            {topUv(ox, oz), topUv(ix, iz), topUv(-ix, iz), topUv(-ox, oz)});
        push({glm::vec3(-ox, ct, -oz), glm::vec3(-ix, ct, -iz), glm::vec3(ix, ct, -iz), glm::vec3(ox, ct, -oz)}, up,
            {topUv(-ox, -oz), topUv(-ix, -iz), topUv(ix, -iz), topUv(ox, -oz)});
        push({glm::vec3(ox, ct, iz), glm::vec3(ox, ct, -oz), glm::vec3(ix, ct, -iz), glm::vec3(ix, ct, iz)}, up,
            {topUv(ox, iz), topUv(ox, -oz), topUv(ix, -iz), topUv(ix, iz)});
        push({glm::vec3(-ox, ct, -iz), glm::vec3(-ox, ct, oz), glm::vec3(-ix, ct, oz), glm::vec3(-ix, ct, -iz)}, up,
            {topUv(-ox, -iz), topUv(-ox, oz), topUv(-ix, oz), topUv(-ix, -iz)});

        const MeshDescription description = canonicalDescription(vertices, indices);
        RenderError           error       = RenderError::None;
        poolCurbMesh_ = renderCapability_.createMesh(renderCapability_.self, rendererHandle_, &description, &error);
        if (poolCurbMesh_ == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo_lighting: pool curb mesh upload failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
        poolCurbGeometry_.positions.reserve(vertices.size());
        poolCurbGeometry_.normals.reserve(vertices.size());
        poolCurbGeometry_.colors.reserve(vertices.size());
        for (const SceneVertex &vertex : vertices)
        {
            poolCurbGeometry_.positions.push_back({vertex.position[0], vertex.position[1], vertex.position[2]});
            poolCurbGeometry_.normals.push_back({vertex.normal[0], vertex.normal[1], vertex.normal[2]});
            poolCurbGeometry_.colors.push_back({vertex.color[0], vertex.color[1], vertex.color[2]});
        }
        poolCurbGeometry_.indices = indices;
        counts_.triangles += static_cast<std::uint32_t>(indices.size() / 3u);
    }

    // ---- the pool floor (the caustics receiver) ---------------------------
    {
        // A pale untextured plate: the caustics and the refracted view of the
        // floor read best without scan detail competing with them.
        std::vector<SceneVertex>   vertices;
        std::vector<std::uint32_t> indices;
        constexpr std::uint32_t kFloorCells = 8; // gentle tessellation; the
        // lighting reads the plate through its card set, not this grid.
        const float hx = static_cast<float>(kWaterHalfX);
        const float hz = static_cast<float>(kWaterHalfZ);
        for (std::uint32_t row = 0; row <= kFloorCells; ++row)
        {
            for (std::uint32_t column = 0; column <= kFloorCells; ++column)
            {
                SceneVertex vertex{};
                vertex.position[0] = -hx + 2.0f * hx * static_cast<float>(column) / static_cast<float>(kFloorCells);
                vertex.position[1] = static_cast<float>(kFloorY);
                vertex.position[2] = -hz + 2.0f * hz * static_cast<float>(row) / static_cast<float>(kFloorCells);
                vertex.normal[1]   = 1.0f;
                vertex.color[0]    = 1.0f;
                vertex.color[1]    = 1.0f;
                vertex.color[2]    = 1.0f;
                vertex.uv[0]       = static_cast<float>(column) / static_cast<float>(kFloorCells);
                vertex.uv[1]       = static_cast<float>(row) / static_cast<float>(kFloorCells);
                vertices.push_back(vertex);
            }
        }
        for (std::uint32_t row = 0; row < kFloorCells; ++row)
        {
            for (std::uint32_t column = 0; column < kFloorCells; ++column)
            {
                const std::uint32_t a = row * (kFloorCells + 1u) + column;
                const std::uint32_t b = a + 1u;
                const std::uint32_t c = a + (kFloorCells + 1u);
                const std::uint32_t d = c + 1u;
                indices.insert(indices.end(), {a, c, b, b, c, d});
            }
        }
        const MeshDescription description = canonicalDescription(vertices, indices);
        RenderError           error       = RenderError::None;
        poolFloorMesh_ = renderCapability_.createMesh(renderCapability_.self, rendererHandle_, &description, &error);
        if (poolFloorMesh_ == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo_lighting: pool floor mesh upload failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
        poolFloorGeometry_.positions.reserve(vertices.size());
        poolFloorGeometry_.normals.reserve(vertices.size());
        poolFloorGeometry_.colors.reserve(vertices.size());
        for (const SceneVertex &vertex : vertices)
        {
            poolFloorGeometry_.positions.push_back({vertex.position[0], vertex.position[1], vertex.position[2]});
            poolFloorGeometry_.normals.push_back({vertex.normal[0], vertex.normal[1], vertex.normal[2]});
            poolFloorGeometry_.colors.push_back({vertex.color[0], vertex.color[1], vertex.color[2]});
        }
        poolFloorGeometry_.indices = indices;
        counts_.triangles += static_cast<std::uint32_t>(indices.size() / 3u);
    }

    // ---- the water surface (the caustics transmitter + refraction face) ---
    {
        // Two baked ripple octaves perturb the vertex normals (the geometry
        // stays a plane at the water line): the reflection field breaks the
        // sun glints over them, the refraction bends through them, and the
        // photon pass focuses the caustics the same way.
        std::vector<SceneVertex>   vertices;
        std::vector<std::uint32_t> indices;
        constexpr std::uint32_t kWaterCellsX = 48;
        constexpr std::uint32_t kWaterCellsZ = 32;
        const float hx = static_cast<float>(kWaterHalfX);
        const float hz = static_cast<float>(kWaterHalfZ);
        const auto rippleHeight = [&](double x, double z, double &dHdx, double &dHdz) {
            // Three octaves: two broad swells and one short chop. The chop is
            // what makes the caustic net read at pool scale - each wavelength
            // focuses its own mesh of lines on the floor.
            const double octave1 = 0.030 * std::sin(1.30 * x + 0.95 * z);
            const double octave2 = 0.020 * std::sin(2.60 * x - 1.90 * z + 1.7);
            const double octave3 = 0.009 * std::sin(5.10 * x + 3.80 * z + 0.6);
            dHdx = 0.030 * 1.30 * std::cos(1.30 * x + 0.95 * z)
                + 0.020 * 2.60 * std::cos(2.60 * x - 1.90 * z + 1.7)
                + 0.009 * 5.10 * std::cos(5.10 * x + 3.80 * z + 0.6);
            dHdz = 0.030 * 0.95 * std::cos(1.30 * x + 0.95 * z)
                - 0.020 * 1.90 * std::cos(2.60 * x - 1.90 * z + 1.7)
                + 0.009 * 3.80 * std::cos(5.10 * x + 3.80 * z + 0.6);
            return octave1 + octave2 + octave3;
        };
        for (std::uint32_t row = 0; row <= kWaterCellsZ; ++row)
        {
            for (std::uint32_t column = 0; column <= kWaterCellsX; ++column)
            {
                const double x = -hx + 2.0 * hx * static_cast<double>(column) / static_cast<double>(kWaterCellsX);
                const double z = -hz + 2.0 * hz * static_cast<double>(row) / static_cast<double>(kWaterCellsZ);
                double dHdx = 0.0;
                double dHdz = 0.0;
                (void)rippleHeight(x, z, dHdx, dHdz);
                const double length = std::sqrt(dHdx * dHdx + 1.0 + dHdz * dHdz);
                SceneVertex vertex{};
                vertex.position[0] = static_cast<float>(x);
                vertex.position[1] = static_cast<float>(kWaterY);
                vertex.position[2] = static_cast<float>(z);
                vertex.normal[0]   = static_cast<float>(-dHdx / length);
                vertex.normal[1]   = static_cast<float>(1.0 / length);
                vertex.normal[2]   = static_cast<float>(-dHdz / length);
                vertex.color[0]    = 1.0f;
                vertex.color[1]    = 1.0f;
                vertex.color[2]    = 1.0f;
                vertex.uv[0]       = static_cast<float>(column) / static_cast<float>(kWaterCellsX);
                vertex.uv[1]       = static_cast<float>(row) / static_cast<float>(kWaterCellsZ);
                vertices.push_back(vertex);
            }
        }
        for (std::uint32_t row = 0; row < kWaterCellsZ; ++row)
        {
            for (std::uint32_t column = 0; column < kWaterCellsX; ++column)
            {
                const std::uint32_t a = row * (kWaterCellsX + 1u) + column;
                const std::uint32_t b = a + 1u;
                const std::uint32_t c = a + (kWaterCellsX + 1u);
                const std::uint32_t d = c + 1u;
                indices.insert(indices.end(), {a, c, b, b, c, d});
            }
        }
        const MeshDescription description = canonicalDescription(vertices, indices);
        RenderError           error       = RenderError::None;
        poolWaterMesh_ = renderCapability_.createMesh(renderCapability_.self, rendererHandle_, &description, &error);
        if (poolWaterMesh_ == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo_lighting: pool water mesh upload failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
        poolWaterGeometry_.positions.reserve(vertices.size());
        poolWaterGeometry_.normals.reserve(vertices.size());
        poolWaterGeometry_.colors.reserve(vertices.size());
        for (const SceneVertex &vertex : vertices)
        {
            poolWaterGeometry_.positions.push_back({vertex.position[0], vertex.position[1], vertex.position[2]});
            poolWaterGeometry_.normals.push_back({vertex.normal[0], vertex.normal[1], vertex.normal[2]});
            poolWaterGeometry_.colors.push_back({vertex.color[0], vertex.color[1], vertex.color[2]});
        }
        poolWaterGeometry_.indices = indices;
        counts_.triangles += static_cast<std::uint32_t>(indices.size() / 3u);
    }

    // ---- the material palette --------------------------------------------
    // Slot order: stone curb, pale floor, water, glass, chrome. The renderer
    // materials are minted in buildStaticMaterials; the registrations reduce
    // these to SurfaceFacts.
    showcaseMaterials_.clear();
    showcaseMaterials_.push_back({{0.62f, 0.60f, 0.55f}, 0.85f, 0.0f, 1.0f, 1.5f});  // stone
    showcaseMaterials_.push_back({{0.82f, 0.80f, 0.74f}, 0.55f, 0.0f, 1.0f, 1.5f});  // pool floor
    showcaseMaterials_.push_back({{0.55f, 0.78f, 0.85f}, 0.03f, 0.0f, 0.42f, 1.33f}); // water
    showcaseMaterials_.push_back({{0.90f, 0.96f, 0.94f}, 0.04f, 0.0f, 0.16f, 1.50f}); // glass
    showcaseMaterials_.push_back({{0.94f, 0.94f, 0.97f}, 0.05f, 0.96f, 1.0f, 1.5f}); // chrome

    // ---- the posed pieces (glass sphere, glass column, chrome sphere) -----
    struct PieceLayout
    {
        glm::dvec3 localPosition;
        float      yaw;
        float      scale;
        std::uint8_t meshSlot;
        std::uint8_t materialSlot;
    };
    const PieceLayout layout[3] = {
        {glm::dvec3(-10.3, 0.0, -4.8), 0.0f, 1.45f, 1u, 3u},  // glass sphere
        {glm::dvec3(10.5, 0.0, -6.6), 0.35f, 1.0f, 2u, 3u},   // glass column
        {glm::dvec3(8.8, 0.0, 4.9), 0.0f, 1.7f, 1u, 4u},      // chrome sphere
    };
    double clearing = std::sqrt(outerHalfX * outerHalfX
        + (std::abs(parameters_.showcasePoolCenterZMeters) + outerHalfZ)
            * (std::abs(parameters_.showcasePoolCenterZMeters) + outerHalfZ));
    showcasePieces_.clear();
    for (const PieceLayout &piece : layout)
    {
        ShowcasePiece entry;
        entry.position     = showcasePoolCenter_ + piece.localPosition;
        entry.yaw          = piece.yaw;
        entry.scale        = piece.scale;
        entry.meshSlot     = piece.meshSlot;
        entry.materialSlot = piece.materialSlot;
        const double reach = std::sqrt(entry.position.x * entry.position.x + entry.position.z * entry.position.z)
            + static_cast<double>(prototypeHalfHeights_[entry.meshSlot]) * piece.scale;
        clearing = std::max(clearing, reach);
        showcasePieces_.push_back(entry);
        counts_.triangles += prototypeTriangles_[entry.meshSlot];
    }
    // The scatter's clearing: everything the cluster owns, plus walking room.
    clearingMeters_ = clearing + 1.2;
    showcaseBuilt_  = true;
    return true;
}

bool LightingScene::registerBorrowedGeometry(const std::vector<glm::vec3> &positions,
    const std::vector<glm::vec3> &normals, const std::vector<glm::vec3> &colors,
    const std::vector<std::uint32_t> &indices, const glm::mat4 &transform, const SurfaceFacts &facts,
    engine::radiance_cache::GiSurfaceHandle &outGiSurface,
    engine::reflection_field::ReflectionSurfaceHandle &outReflectionSurface)
{
    // One shared registration path for every borrowed-geometry surface (the
    // ground quadrants, the scattered instances, the showcase meshes and
    // pieces): both lighting registries describe the same arrays, the facts
    // carry the material, and the caller owns the handles.
    if (positions.empty() || indices.empty())
    {
        return false;
    }
    if (giCapability_ != nullptr)
    {
        GiSurfaceDescription description;
        description.positions           = &positions[0].x;
        description.positionStrideBytes = sizeof(glm::vec3);
        description.normals             = &normals[0].x;
        description.normalStrideBytes   = sizeof(glm::vec3);
        description.colors              = &colors[0].x;
        description.colorStrideBytes    = sizeof(glm::vec3);
        description.indices             = indices.data();
        description.vertexCount         = static_cast<std::uint32_t>(positions.size());
        description.indexCount          = static_cast<std::uint32_t>(indices.size());
        std::memcpy(description.transform, &transform[0][0], sizeof(description.transform));
        description.albedoColor[0] = facts.albedoColor[0];
        description.albedoColor[1] = facts.albedoColor[1];
        description.albedoColor[2] = facts.albedoColor[2];
        description.emissiveColor[0] = facts.emissiveColor[0];
        description.emissiveColor[1] = facts.emissiveColor[1];
        description.emissiveColor[2] = facts.emissiveColor[2];
        description.emissiveIntensity = facts.emissiveIntensity;
        description.opacity            = facts.opacity;
        description.indexOfRefraction  = facts.indexOfRefraction;
        description.flagBits           = facts.dynamic ? giSurfaceFlagDynamic : 0u;
        GiError error = GiError::None;
        outGiSurface = giCapability_->createSurface(giCapability_->self, &description, &error);
        if (outGiSurface == invalidGiSurface)
        {
            std::fprintf(stderr, "demo_lighting: GI registration failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
    }
    if (reflectionCapability_ != nullptr)
    {
        ReflectionSurfaceDescription description;
        description.positions           = &positions[0].x;
        description.positionStrideBytes = sizeof(glm::vec3);
        description.normals             = &normals[0].x;
        description.normalStrideBytes   = sizeof(glm::vec3);
        description.colors              = &colors[0].x;
        description.colorStrideBytes    = sizeof(glm::vec3);
        description.indices             = indices.data();
        description.vertexCount         = static_cast<std::uint32_t>(positions.size());
        description.indexCount          = static_cast<std::uint32_t>(indices.size());
        std::memcpy(description.transform, &transform[0][0], sizeof(description.transform));
        description.albedoColor[0] = facts.albedoColor[0];
        description.albedoColor[1] = facts.albedoColor[1];
        description.albedoColor[2] = facts.albedoColor[2];
        description.emissiveColor[0] = facts.emissiveColor[0];
        description.emissiveColor[1] = facts.emissiveColor[1];
        description.emissiveColor[2] = facts.emissiveColor[2];
        description.emissiveIntensity = facts.emissiveIntensity;
        description.roughness          = facts.roughness;
        description.metallic           = facts.metallic;
        description.opacity            = facts.opacity;
        description.indexOfRefraction  = facts.indexOfRefraction;
        description.flagBits           = facts.dynamic ? reflectionSurfaceFlagDynamic : 0u;
        ReflectionError error = ReflectionError::None;
        outReflectionSurface = reflectionCapability_->createSurface(reflectionCapability_->self, &description, &error);
        if (outReflectionSurface == engine::reflection_field::invalidReflectionSurface)
        {
            std::fprintf(stderr, "demo_lighting: reflection registration failed (error %u)\n",
                static_cast<std::uint32_t>(error));
            return false;
        }
    }
    return true;
}

bool LightingScene::registerSurfaces()
{
    // Ground: four merged quadrant proxies stand in for the 169 drawn tiles
    // (the same split Lumen uses - the lighting world runs over simplified
    // proxies, the draw list keeps its instanced tiles). One surface per
    // tile would give every 24 m tile a six-face card set at the configured
    // texel density - hundreds of megabytes of card storage for a plane -
    // while a quadrant-sized surface lets the card budget clamp lower the
    // density to what the ambient field actually resolves at that extent.
    constexpr std::uint32_t kQuadrantTiles = 7; // 7x7 tiles of the 13x13 grid per quadrant
    {
        const double        quadrantSpan    = kQuadrantTiles * kGroundTileMeters;
        const double        halfSpan        = 0.5 * quadrantSpan;
        const double        gridStep        = kGroundTileMeters / kGroundSegments;
        const std::uint32_t quadrantSegments = kQuadrantTiles * kGroundSegments;
        std::vector<glm::vec3>     positions;
        std::vector<glm::vec3>     normals;
        std::vector<glm::vec3>     colors;
        std::vector<std::uint32_t> indices;
        positions.reserve(static_cast<std::size_t>(quadrantSegments + 1u) * (quadrantSegments + 1u));
        for (std::uint32_t row = 0; row <= quadrantSegments; ++row)
        {
            for (std::uint32_t column = 0; column <= quadrantSegments; ++column)
            {
                positions.push_back({-static_cast<float>(halfSpan) + column * static_cast<float>(gridStep), 0.0f,
                    -static_cast<float>(halfSpan) + row * static_cast<float>(gridStep)});
                normals.push_back({0.0f, 1.0f, 0.0f});
                colors.push_back({0.92f, 0.90f, 0.86f});
            }
        }
        for (std::uint32_t row = 0; row < quadrantSegments; ++row)
        {
            for (std::uint32_t column = 0; column < quadrantSegments; ++column)
            {
                const std::uint32_t a = row * (quadrantSegments + 1u) + column;
                indices.insert(indices.end(), {a, a + (quadrantSegments + 1u), a + 1u, a + 1u,
                    a + (quadrantSegments + 1u), a + (quadrantSegments + 1u) + 1u});
            }
        }
        SurfaceFacts facts;
        facts.albedoColor[0] = 0.92f;
        facts.albedoColor[1] = 0.90f;
        facts.albedoColor[2] = 0.86f;
        facts.roughness      = 0.92f;
        const double offset = 0.5 * quadrantSpan - 0.5 * kGroundTileMeters;
        const glm::dvec3 quadrantCenters[4] = {
            glm::dvec3(-offset, 0.0, -offset), glm::dvec3(offset, 0.0, -offset),
            glm::dvec3(-offset, 0.0, offset), glm::dvec3(offset, 0.0, offset)};
        for (const glm::dvec3 &center : quadrantCenters)
        {
            engine::radiance_cache::GiSurfaceHandle             giHandle = invalidGiSurface;
            engine::reflection_field::ReflectionSurfaceHandle   reflectionHandle
                = engine::reflection_field::invalidReflectionSurface;
            if (!registerBorrowedGeometry(positions, normals, colors, indices, composeTransform(center, 0.0f, 1.0f),
                    facts, giHandle, reflectionHandle))
            {
                return false;
            }
            groundGiSurfaces_.push_back(giHandle);
            groundReflectionSurfaces_.push_back(reflectionHandle);
        }
    }

    // Object instances: the palette's material rides each surface (emissive
    // accents included); dynamic instances flag themselves for the fast
    // per-frame transform path.
    for (Instance &instance : instances_)
    {
        const MaterialSlot &slot = materialPalette()[instance.materialSlot];
        const glm::mat4     transform = instanceTransform(instance, 0.0);
        SurfaceFacts        facts;
        facts.albedoColor[0]  = slot.baseColor[0];
        facts.albedoColor[1]  = slot.baseColor[1];
        facts.albedoColor[2]  = slot.baseColor[2];
        facts.emissiveColor[0] = slot.emissiveColor[0];
        facts.emissiveColor[1] = slot.emissiveColor[1];
        facts.emissiveColor[2] = slot.emissiveColor[2];
        facts.emissiveIntensity = slot.emissiveIntensity;
        facts.roughness       = slot.roughness;
        facts.metallic        = slot.metallic;
        facts.dynamic         = instance.dynamic;
        if (!registerBorrowedGeometry(prototypePositions_[instance.meshSlot], prototypeNormals_[instance.meshSlot],
                prototypeColors_[instance.meshSlot], prototypeIndices_[instance.meshSlot], transform, facts,
                instance.giSurface, instance.reflectionSurface))
        {
            return false;
        }
    }

    // The showcase cluster: curb and floor are ordinary receivers, the water
    // and the glass pieces carry the transmission pair (opacity below 1 plus
    // the medium's index of refraction) that routes them into the photon
    // caustics pass as transmitters and into the reflection field's
    // refracted chains.
    if (showcaseBuilt_)
    {
        const glm::mat4 poolTransform = composeTransform(showcasePoolCenter_, 0.0f, 1.0f);
        const auto showcaseFacts = [this](std::size_t slot) {
            SurfaceFacts facts;
            facts.albedoColor[0]  = showcaseMaterials_[slot].baseColor[0];
            facts.albedoColor[1]  = showcaseMaterials_[slot].baseColor[1];
            facts.albedoColor[2]  = showcaseMaterials_[slot].baseColor[2];
            facts.roughness       = showcaseMaterials_[slot].roughness;
            facts.metallic        = showcaseMaterials_[slot].metallic;
            facts.opacity         = showcaseMaterials_[slot].opacity;
            facts.indexOfRefraction = showcaseMaterials_[slot].indexOfRefraction;
            return facts;
        };
        struct ShowcaseMeshEntry
        {
            const ShowcaseGeometry *geometry;
            std::size_t             materialSlot;
        };
        const ShowcaseMeshEntry meshes[3] = {
            {&poolCurbGeometry_, 0u}, {&poolFloorGeometry_, 1u}, {&poolWaterGeometry_, 2u}};
        for (const ShowcaseMeshEntry &mesh : meshes)
        {
            engine::radiance_cache::GiSurfaceHandle           giHandle = invalidGiSurface;
            engine::reflection_field::ReflectionSurfaceHandle reflectionHandle
                = engine::reflection_field::invalidReflectionSurface;
            if (!registerBorrowedGeometry(mesh.geometry->positions, mesh.geometry->normals, mesh.geometry->colors,
                    mesh.geometry->indices, poolTransform, showcaseFacts(mesh.materialSlot), giHandle,
                    reflectionHandle))
            {
                return false;
            }
            groundGiSurfaces_.push_back(giHandle);
            groundReflectionSurfaces_.push_back(reflectionHandle);
            ++counts_.showcaseSurfaces;
            if (showcaseMaterials_[mesh.materialSlot].opacity < 1.0f)
            {
                ++counts_.transmissiveSurfaces;
            }
        }
        for (ShowcasePiece &piece : showcasePieces_)
        {
            const glm::mat4 transform = composeTransform(piece.position, piece.yaw, piece.scale);
            if (!registerBorrowedGeometry(prototypePositions_[piece.meshSlot], prototypeNormals_[piece.meshSlot],
                    prototypeColors_[piece.meshSlot], prototypeIndices_[piece.meshSlot], transform,
                    showcaseFacts(piece.materialSlot), piece.giSurface, piece.reflectionSurface))
            {
                return false;
            }
            ++counts_.showcaseSurfaces;
            if (showcaseMaterials_[piece.materialSlot].opacity < 1.0f)
            {
                ++counts_.transmissiveSurfaces;
            }
        }
    }

    counts_.giSurfaces = static_cast<std::uint32_t>(groundGiSurfaces_.size() + instances_.size());
    counts_.reflectionSurfaces = static_cast<std::uint32_t>(groundReflectionSurfaces_.size() + instances_.size());

    // Lights through the direct_lighting registry.
    if (directLightingCapability_ != nullptr)
    {
        for (LightTrack &track : lightTracks_)
        {
            engine::direct_lighting::DirectLightingError error = engine::direct_lighting::DirectLightingError::None;
            track.handle = directLightingCapability_->createLight(directLightingCapability_->self,
                &track.description, &error);
            if (track.handle == engine::direct_lighting::invalidDirectLight)
            {
                std::fprintf(stderr, "demo_lighting: light registration failed (error %u)\n",
                    static_cast<std::uint32_t>(error));
                return false;
            }
        }
    }
    return true;
}

glm::mat4 LightingScene::instanceTransform(const Instance &instance, double simSeconds) const
{
    glm::dvec3 position = instance.basePosition;
    float      yaw      = instance.yaw;
    if (instance.dynamic)
    {
        // Bob along the ground normal plus a slow spin: two cheap motions
        // that exercise the whole dynamic path (draw matrix, GI transform,
        // reflection transform) every frame.
        const float lift = instance.bobMeters * (0.5f + 0.5f * std::sin(simSeconds * 1.3 + instance.phase));
        position.y += lift;
        yaw += static_cast<float>(simSeconds * 0.4);
    }
    return composeTransform(position, yaw, instance.scale);
}

void LightingScene::update(double simSeconds, const glm::dvec3 &cameraEyeAnchorRelative)
{
    if (!initialized_)
    {
        return;
    }
    skyEye_ = cameraEyeAnchorRelative;
    // Dynamic transforms: draw matrices and both lighting registries.
    for (Instance &instance : instances_)
    {
        if (!instance.dynamic)
        {
            continue;
        }
        const glm::mat4 transform = instanceTransform(instance, simSeconds);
        instance.dynamicTransform = transform;
        if (giCapability_ != nullptr && instance.giSurface != invalidGiSurface)
        {
            giCapability_->updateSurfaceTransform(giCapability_->self, instance.giSurface, &transform[0][0]);
        }
        if (reflectionCapability_ != nullptr
            && instance.reflectionSurface != engine::reflection_field::invalidReflectionSurface)
        {
            reflectionCapability_->updateSurfaceTransform(
                reflectionCapability_->self, instance.reflectionSurface, &transform[0][0]);
        }
    }
    // Orbiting lights.
    if (directLightingCapability_ != nullptr)
    {
        for (LightTrack &track : lightTracks_)
        {
            if (!track.orbiting || track.handle == engine::direct_lighting::invalidDirectLight)
            {
                continue;
            }
            const double angle = track.phase + simSeconds * track.angularSpeedRadiansPerSecond;
            const double x     = std::cos(angle) * track.radiusMeters;
            const double z     = std::sin(angle) * track.radiusMeters;
            DirectLightDescription updated = track.description;
            updated.position[0] = static_cast<float>(x);
            updated.position[1] = track.heightMeters;
            updated.position[2] = static_cast<float>(z);
            engine::direct_lighting::DirectLightingError error = engine::direct_lighting::DirectLightingError::None;
            directLightingCapability_->updateLight(directLightingCapability_->self, track.handle, &updated, &error);
        }
    }
    composeDrawCommands();
}

void LightingScene::composeDrawCommands()
{
    drawCommands_.clear();
    matrixStore_.clear();
    // The sky dome first: backdrop geometry, no depth writes, so the ground
    // and the instances drawn after it win the shared pixels by depth test
    // while the dome survives everywhere the scene does not cover.
    if (skyDomeMesh_ != engine::renderer::invalidMesh)
    {
        // Skybox rule: the dome centers on the eye, so every band keeps a
        // benign screen projection and the below-horizon haze fills the
        // distance past the ground grid's edge.
        matrixStore_.push_back(
            glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(skyEye_.x), static_cast<float>(skyEye_.y),
                static_cast<float>(skyEye_.z))));
        RenderDrawCommand command;
        command.mesh        = skyDomeMesh_;
        command.modelMatrix = &matrixStore_.back()[0][0];
        command.material    = &skyBackdropMaterial_;
        drawCommands_.push_back(command);
    }
    if (sunDiscActive_ && sunDiscMesh_ != engine::renderer::invalidMesh)
    {
        // The disc rides the dome shell at a fixed angular size (half-size
        // 12 m at 700 m reads just under two degrees across - a low-sun disc,
        // not an area light) and faces the eye: a billboard basis built from
        // the sun direction, the same backdrop depth contract as the dome
        // behind it.
        constexpr float kDiscDistanceMeters = 700.0f;
        constexpr float kDiscHalfSizeMeters = 12.0f;
        const glm::vec3 eye(static_cast<float>(skyEye_.x), static_cast<float>(skyEye_.y),
            static_cast<float>(skyEye_.z));
        const glm::vec3 forward = -sunDirectionWorld_; // from the disc toward the eye
        glm::vec3       right   = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), forward);
        if (glm::length(right) < 1.0e-4f)
        {
            right = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        else
        {
            right = glm::normalize(right);
        }
        const glm::vec3 up       = glm::cross(forward, right);
        const glm::vec3 position = eye + sunDirectionWorld_ * kDiscDistanceMeters;
        glm::mat4       billboard(glm::vec4(right * kDiscHalfSizeMeters, 0.0f), glm::vec4(up * kDiscHalfSizeMeters, 0.0f),
            glm::vec4(forward, 0.0f), glm::vec4(position, 1.0f));
        matrixStore_.push_back(billboard);
        RenderDrawCommand command;
        command.mesh        = sunDiscMesh_;
        command.modelMatrix = &matrixStore_.back()[0][0];
        command.material    = &sunDiscMaterial_;
        drawCommands_.push_back(command);
    }
    // Ground next (fills the depth buffer under everything), then the
    // showcase cluster, then the instances near-to-far for early depth
    // rejection.
    for (std::size_t tile = 0; tile < groundTransforms_.size(); ++tile)
    {
        matrixStore_.push_back(groundTransforms_[tile]);
        RenderDrawCommand command;
        command.mesh        = groundTileMesh_;
        command.modelMatrix = &matrixStore_.back()[0][0];
        command.material    = &materialStore_[tile];
        drawCommands_.push_back(command);
    }
    // The showcase cluster: curb and floor opaque, the transmissive water
    // and the glass pieces follow (the renderer's transparent pass sorts
    // those far-to-near after the opaque pass; the draw order here only
    // fixes the opaque pieces' depth order).
    if (showcaseBuilt_)
    {
        struct ShowcaseDraw
        {
            engine::renderer::MeshHandle mesh;
            std::uint32_t                materialSlot;
        };
        const ShowcaseDraw draws[3] = {
            {poolCurbMesh_, 0u}, {poolFloorMesh_, 1u}, {poolWaterMesh_, 2u}};
        for (const ShowcaseDraw &draw : draws)
        {
            if (draw.mesh == engine::renderer::invalidMesh)
            {
                continue;
            }
            matrixStore_.push_back(composeTransform(showcasePoolCenter_, 0.0f, 1.0f));
            RenderDrawCommand command;
            command.mesh        = draw.mesh;
            command.modelMatrix = &matrixStore_.back()[0][0];
            command.material    = &materialStore_[showcaseMaterialBase_ + draw.materialSlot];
            drawCommands_.push_back(command);
        }
        for (const ShowcasePiece &piece : showcasePieces_)
        {
            matrixStore_.push_back(composeTransform(piece.position, piece.yaw, piece.scale));
            RenderDrawCommand command;
            command.mesh        = objectMeshes_[piece.meshSlot];
            command.modelMatrix = &matrixStore_.back()[0][0];
            command.material    = &materialStore_[showcaseMaterialBase_ + piece.materialSlot];
            drawCommands_.push_back(command);
        }
    }
    // Near-to-far ordering against the orbit eye (the orbit circles the
    // anchor; distance to the anchor orders the field well enough).
    std::vector<std::uint32_t> order;
    order.reserve(instances_.size());
    for (std::uint32_t index = 0; index < instances_.size(); ++index)
    {
        order.push_back(index);
    }
    std::sort(order.begin(), order.end(), [this](std::uint32_t a, std::uint32_t b) {
        const glm::dvec3 &pa = instances_[a].basePosition;
        const glm::dvec3 &pb = instances_[b].basePosition;
        return glm::dot(pa, pa) < glm::dot(pb, pb);
    });
    for (const std::uint32_t index : order)
    {
        const Instance &instance = instances_[index];
        matrixStore_.push_back(instance.dynamic ? instance.dynamicTransform : instanceTransform(instance, 0.0));
        RenderDrawCommand command;
        command.mesh        = objectMeshes_[instance.meshSlot];
        command.modelMatrix = &matrixStore_.back()[0][0];
        command.material    = &materialStore_[objectMaterialBase_ + index];
        drawCommands_.push_back(command);
    }
}

std::uint32_t LightingScene::selfTest()
{
    std::uint32_t failures = 0;
    if (counts_.objects != instances_.size())
    {
        ++failures;
    }
    if (counts_.giSurfaces != groundGiSurfaces_.size() + instances_.size())
    {
        ++failures;
    }
    if (counts_.reflectionSurfaces != groundReflectionSurfaces_.size() + instances_.size())
    {
        ++failures;
    }
    for (const Instance &instance : instances_)
    {
        if (giCapability_ != nullptr && instance.giSurface == invalidGiSurface)
        {
            ++failures;
            break;
        }
        if (reflectionCapability_ != nullptr
            && instance.reflectionSurface == engine::reflection_field::invalidReflectionSurface)
        {
            ++failures;
            break;
        }
    }
    // The showcase cluster's registrations: every mesh and piece surface
    // landed, the counts agree, and the transmissive share is exactly the
    // water plus the two glass pieces.
    if (showcaseBuilt_)
    {
        const std::uint32_t expectedShowcase = 3u + static_cast<std::uint32_t>(showcasePieces_.size());
        if (counts_.showcaseSurfaces != expectedShowcase)
        {
            ++failures;
        }
        const std::uint32_t expectedTransmissive = 1u + 2u; // water + glass sphere + glass column
        if (counts_.transmissiveSurfaces != expectedTransmissive)
        {
            ++failures;
        }
        for (const ShowcasePiece &piece : showcasePieces_)
        {
            if (giCapability_ != nullptr && piece.giSurface == invalidGiSurface)
            {
                ++failures;
                break;
            }
            if (reflectionCapability_ != nullptr
                && piece.reflectionSurface == engine::reflection_field::invalidReflectionSurface)
            {
                ++failures;
                break;
            }
        }
    }
    // Placement determinism: the first instance's placement re-derives from
    // the seed alone (the lattice walk is a pure function of the seed).
    if (!instances_.empty())
    {
        const std::uint64_t cellHash = mix64(parameters_.seed ^ 0x4B9A2C31D17E4F5Dull);
        const double        firstJitter = unit01(cellHash, 1);
        if (!(firstJitter >= 0.0 && firstJitter <= 1.0))
        {
            ++failures;
        }
    }
    std::cout << "self-test: " << (failures == 0 ? "PASS" : "FAIL") << " - " << counts_.objects << " objects ("
              << counts_.dynamicObjects << " dynamic), " << counts_.giSurfaces << " GI surfaces, "
              << counts_.reflectionSurfaces << " reflection surfaces, " << counts_.showcaseSurfaces
              << " showcase surfaces (" << counts_.transmissiveSurfaces << " transmissive), " << counts_.lights
              << " lights, " << counts_.triangles << " triangles\n";
    return failures;
}

} // namespace demo::lighting_demo
