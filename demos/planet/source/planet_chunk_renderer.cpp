#include "planet_chunk_renderer.hpp"
#include "planet_scatter.hpp"

#include <engine/planet/capabilities.hpp>
#include <engine/planet/fbm.hpp>
#include <engine/planet/noise.hpp>
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
#include <cstdio>
#include <iostream>
#include <vector>

namespace demo::planet_demo
{
namespace planet = engine::planet;
namespace renderer = engine::renderer;

// The one vertex-layout description every upload in this file (and the
// self-test) uses; defined beside the upload helpers so the renderer-facing
// layout lives with the renderer-facing code instead of in the test file.
std::array<renderer::VertexAttribute, 4> canonicalAttributes()
{
    return {{
        {renderer::VertexSemantic::Position, renderer::VertexFormat::Float32x3, 0},
        {renderer::VertexSemantic::Normal, renderer::VertexFormat::Float32x3, 12},
        {renderer::VertexSemantic::Color, renderer::VertexFormat::Float32x3, 24},
        {renderer::VertexSemantic::TexCoord0, renderer::VertexFormat::Float32x2, 36},
    }};
}

namespace
{
constexpr std::uint32_t kVertexStride = sizeof(planet::PlanetVertex);

// Winding of the demo's outward spherical shells (the world-anchored cloud
// shell, the atmosphere limb annulus) as emitted by the backdrop builders
// below. Declaring it per draw lets the headless backend cull each shell's
// far hemisphere per triangle (see MeshMaterial::frontFaceWinding); the
// sign was verified against the builders' vertex emission (a globe capture
// with the shells intact confirms it - the wrong sign erases them).
constexpr float macroShellFrontFaceWinding = 1.0f;

// Builds a radial disc mesh on the unit ball around +Z: a triangle fan of
// `bands` rings whose UVs map the unit disc onto a radial halo texture (the
// alpha channel carries the core-to-rim falloff; see ensureSunDisc) and whose
// vertex colors stay neutral - the material's per-frame baseColor tints the
// whole disc (the sun tint), the texture's alpha does the fading. Vertex
// colors fall to zero NO MORE: an opaque draw with a dark rim painted a
// black ring over the sky dome behind it.
void buildSunDiscMesh(std::vector<planet::PlanetVertex> &vertices, std::vector<std::uint32_t> &indices,
    std::uint32_t segments, std::uint32_t bands)
{
    vertices.reserve(1u + segments * bands);
    indices.reserve(segments * bands * 6u);
    auto pushVertex = [&vertices](const glm::dvec2 &direction, double radius)
    {
        planet::PlanetVertex vertex{};
        vertex.position[0] = static_cast<float>(direction.x * radius);
        vertex.position[1] = static_cast<float>(direction.y * radius);
        vertex.position[2] = 0.0f;
        vertex.normal[0]   = 0.0f;
        vertex.normal[1]   = 0.0f;
        vertex.normal[2]   = 1.0f;
        vertex.color[0]    = 1.0f;
        vertex.color[1]    = 1.0f;
        vertex.color[2]    = 1.0f;
        // Unit disc -> texture space (the texture samples a radial falloff
        // centered at its middle): the UV must scale with the ring's OWN
        // radius (position on the unit disc), not the unit direction alone -
        // a UV built from the direction alone maps every ring onto the same
        // texture circle and the halo collapses to one constant-alpha ring.
        vertex.uv[0]       = static_cast<float>(direction.x * radius * 0.5 + 0.5);
        vertex.uv[1]       = static_cast<float>(direction.y * radius * 0.5 + 0.5);
        vertices.push_back(vertex);
        return static_cast<std::uint32_t>(vertices.size() - 1u);
    };
    const std::uint32_t center = pushVertex(glm::dvec2{0.0, 0.0}, 0.0);
    std::uint32_t previousRingStart = center;
    bool previousIsCenter = true;
    for (std::uint32_t band = 1; band <= bands; ++band)
    {
        const double radius = static_cast<double>(band) / static_cast<double>(bands);
        std::uint32_t ringStart = 0;
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(segments);
            const std::uint32_t current
                = pushVertex(glm::dvec2{std::cos(azimuth), std::sin(azimuth)}, radius);
            if (segment == 0)
            {
                ringStart = current;
            }
            const std::uint32_t next = segment + 1u == segments ? ringStart : current + 1u;
            if (previousIsCenter)
            {
                indices.insert(indices.end(), {center, current, next});
            }
            else
            {
                const std::uint32_t previous = previousRingStart + segment;
                const std::uint32_t previousNext = segment + 1u == segments ? previousRingStart : previous + 1u;
                indices.insert(indices.end(), {previous, current, next, previous, next, previousNext});
            }
        }
        previousRingStart = ringStart;
        previousIsCenter  = false;
    }
}

// Builds the moon companion's sphere on the unit ball: a lat-long grid
// whose vertex colors bake the regolith albedo - dark basaltic maria
// provinces under bright cratered highlands (a provincial simplex-fBm plus a
// fine mottling octave, both seed-derived) - and whose normals point
// outward, so the LIT draw shades the phase from the sun direction by
// itself. The maria span a third of the ball at most, the way a real
// companion's dark provinces read against its cratered highs.
void buildMoonMesh(const planet::PlanetGenerationParams &params, std::vector<planet::PlanetVertex> &vertices,
    std::vector<std::uint32_t> &indices, std::uint32_t segments, std::uint32_t bands)
{
    const std::int64_t mariaSeed
        = static_cast<std::int64_t>(planet::hashCombine64(params.seed, 0x4D4142ULL)); // "MAB" maria basalt
    const planet::FbmStreamSeeds mariaSeeds = planet::makeFbmSeeds(mariaSeed, 3);
    const planet::FbmStreamSeeds mottleSeeds = planet::makeFbmSeeds(
        static_cast<std::int64_t>(planet::hashCombine64(static_cast<std::uint64_t>(mariaSeed), 0x4077ULL)), 2);

    vertices.reserve(static_cast<std::size_t>((segments + 1u) * (bands + 1u)));
    indices.reserve(static_cast<std::size_t>(segments * bands * 6u));
    for (std::uint32_t band = 0; band <= bands; ++band)
    {
        const double latitude = -glm::pi<double>() * 0.5
            + glm::pi<double>() * (static_cast<double>(band) / static_cast<double>(bands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= segments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(segments);
            const glm::dvec3 direction(std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude);
            // Maria provinces: one three-octave stream at hemispheric scale;
            // the threshold picks the dark basalt flats out of the highlands.
            const planet::Vec3 unit{direction.x, direction.y, direction.z};
            const double maria = planet::fbm(mariaSeeds, unit, 1.1, planet::FbmParams{3, 2.0, 0.5}, 0.0);
            double albedo = planet::mix(0.27, 0.60, planet::smoothstep(-0.22, 0.28, maria));
            // Fine mottling: crater-floor brightness noise so the highs do
            // not read as one flat tone.
            const double mottle = planet::fbm(mottleSeeds, unit, 0.14, planet::FbmParams{2, 2.0, 0.5}, 0.0);
            albedo *= 0.93 + 0.07 * planet::clamp01(0.5 + 0.5 * mottle);
            // Regolith grey with a faint warm lean (the real surface's
            // spectral slope); the albedo carries the maria structure.
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            vertex.normal[0]   = static_cast<float>(direction.x);
            vertex.normal[1]   = static_cast<float>(direction.y);
            vertex.normal[2]   = static_cast<float>(direction.z);
            vertex.color[0]    = static_cast<float>(albedo * 0.985);
            vertex.color[1]    = static_cast<float>(albedo * 0.965);
            vertex.color[2]    = static_cast<float>(albedo * 0.945);
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < bands; ++band)
    {
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const std::uint32_t a = band * (segments + 1u) + segment;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (segments + 1u);
            const std::uint32_t d = c + 1u;
            indices.insert(indices.end(), {a, b, c, b, d, c});
        }
    }
}

// Builds the aurora curtain: one narrow oval above each spin pole whose
// CURTAINS STAND VERTICALLY - the latitude band spans a few degrees while
// the altitude climbs from the curtain's foot to its top (see the aurora
// constants in atmosphere.hpp; real curtains rise radially from a tight
// oval, and a latitude-equals-altitude tilt reads as a tent from space).
// The split of work matters: the VERTEX COLORS carry only the curtain's
// chroma (oxygen-green foot, nitrogen-violet crown) while the whole
// INTENSITY field - night gate, ray striations, altitude fade - is baked
// into a small RGBA texture's alpha channel (white RGB, alpha = intensity).
// The transparent route composites at opacity * texel alpha, so a day-side
// column's texels carry alpha ~0 and vanish instead of alpha-darkening the
// sky behind them - the old bake put the intensity into the colors alone,
// and a drawn-but-black column then read as a dark smear across the bright
// limb and the daylight sky. The night gate is keyed against the run's sun
// direction (the mesh is static after upload). Quads whose every corner
// falls below the gate are still skipped outright so the day side does not
// even rasterize. The vertices are planet-local meters, world-anchored at
// the planet center like the cloud shell.
void buildAuroraMesh(const planet::PlanetGenerationParams &params, const glm::dvec3 &sunDirection,
    std::vector<planet::PlanetVertex> &vertices, std::vector<std::uint32_t> &indices,
    std::vector<std::uint8_t> &alphaPixels)
{
    const glm::dvec3 spin = glm::normalize(glm::dvec3(params.axis[0], params.axis[1], params.axis[2]));
    glm::dvec3 prime = glm::cross(spin, glm::dvec3(0.0, 0.0, 1.0));
    if (glm::length(prime) < 1.0e-6)
    {
        prime = glm::cross(spin, glm::dvec3(1.0, 0.0, 0.0));
    }
    prime = glm::normalize(prime);
    const glm::dvec3 coPrime = glm::cross(spin, prime);
    const glm::dvec3 sun = glm::length(sunDirection) > 1.0e-9 ? glm::normalize(sunDirection) : glm::dvec3(0.0, 1.0, 0.0);
    const std::int64_t raySeed
        = static_cast<std::int64_t>(planet::hashCombine64(params.seed, 0xA04AULL)); // "AOA" aurora rays
    // Curtain rays ~2.6 deg of azimuth apart: the striation field is keyed
    // to the ring's azimuth only (constant along a ray's climb), so the
    // rays stand vertically around the pole the way real curtains do.
    const double rayWavenumber = 24.0;

    constexpr std::uint32_t kSegments = 192;
    constexpr std::uint32_t kBands = 7;
    // Green line at the foot, nitrogen violet at the crown (the real
    // curtain's altitude chemistry); linear RGB emissive tones.
    constexpr double kAuroraGreen[3]  = {0.18, 0.98, 0.45};
    constexpr double kAuroraViolet[3] = {0.44, 0.22, 0.62};
    // A curtain column renders only when its foot clears this gate value:
    // fully-day columns would alpha-darken the terrain behind them, and the
    // vertical cut reads as a curtain end.
    constexpr double kQuadGateFloor = 0.10;
    // The intensity texture: one azimuth column per 256 texels, one band row
    // per pole half (rows 0..kBands north, kBands+1..2*kBands+1 south).
    constexpr std::uint32_t kAlphaWidth = 256;
    constexpr std::uint32_t kAlphaHeight = 2u * (kBands + 1u);

    // The curtain's intensity field, shared by the alpha texture and the
    // column gate: night gate x ray striation x altitude fade.
    const auto intensityAt = [&](int pole, double azimuth, double bandT)
    {
        const double latitude = planet::auroraOvalLatitudeLowRadians
            + (planet::auroraOvalLatitudeHighRadians - planet::auroraOvalLatitudeLowRadians) * bandT;
        const glm::dvec3 direction = prime * (std::cos(azimuth) * std::cos(latitude))
            + coPrime * (std::sin(azimuth) * std::cos(latitude))
            + spin * (static_cast<double>(pole) * std::sin(latitude));
        const double ray = 0.5 + 0.5 * planet::noise::simplex3ImproveXY(raySeed,
            std::cos(azimuth) * rayWavenumber, std::sin(azimuth) * rayWavenumber,
            static_cast<double>(pole) * 7.0);
        // Night gate: the curtain glows through the terminator into deep
        // night (orbital photographs show the oval as a faint arc crossing
        // even the twilight limb) and fades out on the day side, so a
        // bright-blue morning sky stays clean.
        const double nightGate = 1.0 - planet::smoothstep(-0.05, 0.32, glm::dot(direction, sun));
        // Vertical profile: brightest at the foot and thinning to NOTHING
        // at the crown (a curtain that ends at partial brightness shows a
        // hard rectangular rim against the sky); the ray field breaks the
        // ring into curtains.
        const double fade = std::pow(1.0 - bandT, 1.45);
        return nightGate * (0.35 + 0.65 * std::pow(ray, 1.7)) * fade;
    };

    struct RingPoint
    {
        planet::PlanetVertex vertex;
    };
    const auto ringPoint = [&](int pole, std::uint32_t band, std::uint32_t segment)
    {
        const double bandT = static_cast<double>(band) / static_cast<double>(kBands);
        const double latitude = planet::auroraOvalLatitudeLowRadians
            + (planet::auroraOvalLatitudeHighRadians - planet::auroraOvalLatitudeLowRadians) * bandT;
        const double altitude = planet::auroraBaseAltitudeMeters
            + (planet::auroraTopAltitudeMeters - planet::auroraBaseAltitudeMeters) * bandT;
        const double radius = params.radiusMeters + altitude;
        const double azimuth
            = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
        const glm::dvec3 direction = prime * (std::cos(azimuth) * std::cos(latitude))
            + coPrime * (std::sin(azimuth) * std::cos(latitude))
            + spin * (static_cast<double>(pole) * std::sin(latitude));
        RingPoint point{}; // value-init: the untouched uv pair must read as finite zeros
        // Chroma only - the intensity field rides the alpha texture, so the
        // blend carries the glow's brightness (the composite multiplies the
        // same way: chroma x strength x opacity x intensity).
        const double violet = bandT * bandT;
        point.vertex.color[0]    = static_cast<float>(planet::mix(kAuroraGreen[0], kAuroraViolet[0], violet));
        point.vertex.color[1]    = static_cast<float>(planet::mix(kAuroraGreen[1], kAuroraViolet[1], violet));
        point.vertex.color[2]    = static_cast<float>(planet::mix(kAuroraGreen[2], kAuroraViolet[2], violet));
        // Intensity texture coordinates: u wraps the ring (the seam vertex
        // repeats direction azimuth 0 with u = 1), v lands on the pole's own
        // row block with the +0.5 putting band centers on texel centers.
        const std::uint32_t row = static_cast<std::uint32_t>(band)
            + (pole == 1 ? 0u : kBands + 1u);
        point.vertex.uv[0]       = static_cast<float>(static_cast<double>(segment) / static_cast<double>(kSegments));
        point.vertex.uv[1]       = static_cast<float>((static_cast<double>(row) + 0.5)
            / static_cast<double>(kAlphaHeight));
        point.vertex.position[0] = static_cast<float>(direction.x * radius);
        point.vertex.position[1] = static_cast<float>(direction.y * radius);
        point.vertex.position[2] = static_cast<float>(direction.z * radius);
        point.vertex.normal[0]   = static_cast<float>(direction.x);
        point.vertex.normal[1]   = static_cast<float>(direction.y);
        point.vertex.normal[2]   = static_cast<float>(direction.z);
        return point;
    };

    // The intensity texture: RGB is a neutral white multiplier (the vertex
    // colors carry the chroma); the alpha channel is the curtain's intensity
    // field - night gate x ray striation x altitude fade - sampled at the
    // same bandT the ring rows interpolate. The transparent route's blend
    // factor is opacity * texel alpha, so a day-side column's texels carry
    // alpha ~0 and composite to nothing.
    alphaPixels.assign(static_cast<std::size_t>(kAlphaWidth) * kAlphaHeight * 4u, 255u);
    for (int pole = 1; pole >= -1; pole -= 2)
    {
        const std::uint32_t rowOffset = pole == 1 ? 0u : kBands + 1u;
        for (std::uint32_t band = 0; band <= kBands; ++band)
        {
            const double bandT = static_cast<double>(band) / static_cast<double>(kBands);
            for (std::uint32_t column = 0; column < kAlphaWidth; ++column)
            {
                const double azimuth = glm::two_pi<double>() * static_cast<double>(column)
                    / static_cast<double>(kAlphaWidth);
                const double intensity = intensityAt(pole, azimuth, bandT);
                const std::size_t index
                    = (static_cast<std::size_t>(rowOffset + band) * kAlphaWidth + column) * 4u;
                alphaPixels[index + 3] = static_cast<std::uint8_t>(
                    std::clamp(intensity, 0.0, 1.0) * 255.0);
            }
        }
    }

    vertices.reserve(static_cast<std::size_t>(2u * (kSegments + 1u) * (kBands + 1u)));
    indices.reserve(static_cast<std::size_t>(2u * kSegments * kBands * 6u));
    for (int pole = 1; pole >= -1; pole -= 2)
    {
        // One grid per pole: the pole sign folds into the ring direction
        // (and into the night gate and the ray field's seed offset), so the
        // southern oval glows on ITS night side with its own curtain rays.
        const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
        for (std::uint32_t band = 0; band <= kBands; ++band)
        {
            for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
            {
                vertices.push_back(ringPoint(pole, band, segment).vertex);
            }
        }
        // Curtain columns: a column whose foot falls below the gate floor is
        // skipped WHOLE (every altitude band), so the ring's boundary is a
        // set of vertical cuts aligned with its own ray striations - the
        // edges read as curtain ends, never as a sliced sheet. Inside a kept
        // column the altitude fade rides the alpha texture alone.
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            const auto footIntensity = [&](std::uint32_t column)
            {
                const double azimuth = glm::two_pi<double>() * static_cast<double>(column % kSegments)
                    / static_cast<double>(kSegments);
                return intensityAt(pole, azimuth, 0.0);
            };
            if (std::max(footIntensity(segment), footIntensity(segment + 1u)) < kQuadGateFloor)
            {
                continue;
            }
            for (std::uint32_t band = 0; band < kBands; ++band)
            {
                const std::uint32_t a = base + band * (kSegments + 1u) + segment;
                const std::uint32_t b = a + 1u;
                const std::uint32_t c = a + (kSegments + 1u);
                const std::uint32_t d = c + 1u;
                indices.insert(indices.end(), {a, b, c, b, d, c});
            }
        }
    }
}

// The one description shape every upload in this file uses: canonical
// attributes, 32-bit indices, the canonical vertex stride. The attribute
// array is cached once (canonicalAttributes returns by value).
renderer::MeshDescription canonicalMeshDescription(
    const planet::PlanetVertex *vertices, std::uint32_t vertexCount, const std::uint32_t *indices, std::uint32_t indexCount)
{
    static const auto attributes = canonicalAttributes();
    renderer::MeshDescription description;
    description.vertices = vertices;
    description.vertexCount = vertexCount;
    description.indices = indices;
    description.indexCount = indexCount;
    description.indexFormat = renderer::MeshIndexFormat::UInt32;
    description.vertexStrideBytes = kVertexStride;
    description.vertexAttributes = attributes.data();
    description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());
    return description;
}

// Uploads a built shell mesh as a canonical renderer mesh.
renderer::MeshHandle uploadShellMesh(renderer::RenderCapability &renderCapability, renderer::RendererHandle rendererHandle,
    const std::vector<planet::PlanetVertex> &vertices, const std::vector<std::uint32_t> &indices)
{
    const renderer::MeshDescription description = canonicalMeshDescription(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()), indices.data(),
        static_cast<std::uint32_t>(indices.size()));

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

void PlanetChunkRenderer::setBackdropLighting(const BackdropLighting &lighting)
{
    backdropLighting_ = lighting;
    backdropLightingValid_ = true;
}

void PlanetChunkRenderer::setTerrainTextures(const std::vector<TerrainTextureVariant> &groundVariants,
    const std::vector<TerrainTextureVariant> &rockVariants, double groundTextureMaxDistanceMeters,
    double rockTextureMaxDistanceMeters, double groundReliefMaxDistanceMeters)
{
    // Variants with no loadable detail map cannot multiply - drop them so
    // the per-chunk pick only ever selects a textured variant.
    groundTextureVariants_.clear();
    rockTextureVariants_.clear();
    for (const TerrainTextureVariant &variant : groundVariants)
    {
        if (variant.detail != renderer::invalidTexture)
        {
            groundTextureVariants_.push_back(variant);
        }
    }
    for (const TerrainTextureVariant &variant : rockVariants)
    {
        if (variant.detail != renderer::invalidTexture)
        {
            rockTextureVariants_.push_back(variant);
        }
    }
    groundTextureMaxDistanceMetersSquared_ = groundTextureMaxDistanceMeters > 0.0
        ? groundTextureMaxDistanceMeters * groundTextureMaxDistanceMeters : 0.0;
    rockTextureMaxDistanceMetersSquared_ = rockTextureMaxDistanceMeters > 0.0
        ? rockTextureMaxDistanceMeters * rockTextureMaxDistanceMeters : 0.0;
    // The relief gate stays TIGHTER than the albedo gate: a minified normal
    // map aliases (its bilinear footprint flattens back toward the vertex
    // gradient - harmless - but an under-sampled relief map flickers as the
    // mip chain steps), so the per-sample normal shuts off at a tenth of
    // the albedo gate's distance: one tile still spans ten times the pixel
    // budget the albedo multiply needs. The relief reads at the eye's feet
    // - a first-person view concentrates its ground pixels there - while
    // the midground keeps the albedo detail and the vertex shading.
    groundReliefMaxDistanceMetersSquared_ = groundReliefMaxDistanceMeters > 0.0
        ? groundReliefMaxDistanceMeters * groundReliefMaxDistanceMeters : 0.0;
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
    if (handles.rock != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, handles.rock);
        handles.rock = renderer::invalidMesh;
    }
    if (handles.water != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, handles.water);
        handles.water = renderer::invalidMesh;
    }
    handles.terrainPlacementBuilt = false;
    handles.waterPlacementBuilt = false;
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

        const bool wantWater = view.isWater;
        // Terrain views carry their surface group (see the view struct): the
        // ground and rock groups upload as separate renderer meshes sharing
        // the chunk's placement, so each group draws with its own material.
        const bool wantRock = !wantWater && view.surfaceGroup != 0u;
        renderer::MeshHandle &slot = wantWater ? handles.water : (wantRock ? handles.rock : handles.terrain);
        std::uint64_t &uploadedHash = wantWater ? handles.waterHash : (wantRock ? handles.rockHash : handles.terrainHash);
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
            const renderer::MeshDescription description
                = canonicalMeshDescription(view.vertices, view.vertexCount, view.indices, view.indexCount);

            renderer::RenderError error = renderer::RenderError::None;
            slot = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
            if (slot != renderer::invalidMesh)
            {
                uploadedHash = view.meshHash;
            }
            else if (uploadFailureReports_ < 8)
            {
                // A backend that cannot upload would otherwise repeat this
                // line every frame for every chunk; the first few reports
                // carry the diagnosis, the rest are noise.
                ++uploadFailureReports_;
                std::cerr << "demo_planet: chunk mesh upload failed (error "
                          << static_cast<std::uint32_t>(error) << ")\n";
            }
        }

        // Chunk vertices are planet-local offsets from the chunk center; the
        // world placement adds the planet's own center offset (world =
        // planet-local + center, matching the module's worldPointToPlanetLocal).
        // The placement is a pure function of the chunk key (the module's
        // centers never move), so it is built only once per chunk and pass
        // instead of once per visible chunk per frame - including for
        // all-water chunks, whose terrain slot never opens but whose water
        // draw still needs the placement.
        if (wantWater ? handles.waterPlacementBuilt : handles.terrainPlacementBuilt)
        {
            continue;
        }
        const glm::dvec3 placementCenter(params_.centerMeters[0] + view.centerMeters.x,
            params_.centerMeters[1] + view.centerMeters.y, params_.centerMeters[2] + view.centerMeters.z);
        const glm::mat4 transform = glm::translate(glm::mat4(1.0f),
            glm::vec3(static_cast<float>(placementCenter.x), static_cast<float>(placementCenter.y),
                static_cast<float>(placementCenter.z)));
        handles.placementCenter = placementCenter;
        if (wantWater)
        {
            handles.waterTransform = transform;
            handles.waterPlacementBuilt = true;
        }
        else
        {
            handles.terrainTransform = transform;
            handles.terrainPlacementBuilt = true;
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

void PlanetChunkRenderer::setSurfaceScatter(PlanetScatterSystem *scatter)
{
    surfaceScatter_ = scatter;
}

void PlanetChunkRenderer::rebuildDrawCommands(const glm::mat4 &viewProjection, const glm::dvec3 &eyePlanetLocal,
    const planet::Vec3 &sunTint, const glm::dvec3 &renderShift)
{
    drawCommands_.clear();
    // The material stores clear (and below, reserve) before any command
    // takes pointers into them.

    // Draw order: opaque scene geometry first, transparent/cloudy geometry
    // depth-tested against it, then the backdrop shells.
    //   0. star dome (space views): the deepest backdrop - the limb shell's
    //      airglow ring and the terrain silhouette both overwrite it, so
    //      stars stay hidden behind the sunlit air the way real starlight
    //      is lost through the limb's slanted air column instead of shining
    //      through the glow ring.
    //   1. atmosphere limb (space views): unlit, depth writes off; the ring
    //      outside the terrain silhouette survives as glowing air - the
    //      limb's disk interior is MEANT to be overwritten by the opaque
    //      draws, so this shell stays ahead of the scene.
    //   2. chunk terrain (both surface groups) and water, near to far: the
    //      depth buffer fills with the surfaces the eye actually sees, so
    //      every later draw's hidden pixels reject early.
    //   3. sky cloud deck (ground views) and world-anchored cloud shell
    //      (macro views): drawn against the filled depth buffer, so a
    //      shell's far side (hidden behind the planet) rejects without
    //      shading a pixel, and near-side banks composite over the terrain
    //      the way the previous backdrop-first order composited them over
    //      the limb.
    //   4. sun disc, moon, sky dome: the unlit backdrop shells. Every
    //      pixel they cover that the terrain already drew depth-fails (they
    //      all sit beyond the terrain from any camera the demo uses), so
    //      they only fill the sky the terrain does not reach - or, where
    //      chunks are still streaming in, a haze color instead of a hole to
    //      the clear color.
    // Camera-relative pass: every model matrix is composed in f64 against
    // the render shift, so the f32 pipeline only sees eye-relative
    // coordinates (a zero shift reproduces the historical absolute frame
    // bit-for-bit - the compose is exact at small magnitudes).
    const glm::dmat4 shiftToEye = glm::translate(glm::dmat4(1.0), -renderShift);
    const auto composeShifted = [&shiftToEye](const glm::mat4 &absolute)
    {
        return glm::mat4(shiftToEye * glm::dmat4(absolute));
    };
    frameBackdropMatrices_.clear();
    frameBackdropMatrices_.reserve(10);
    // The scatter matrices back the OPAQUE scatter pass and the transparent
    // scatter tail below; clearing per frame keeps the store bounded and
    // every push inside the reserve, so the draw commands' modelMatrix
    // pointers stay valid until the frame is submitted (an uncleared,
    // growing store reallocates mid-frame and the earlier commands then
    // read freed memory - observed as a billboard stretched across the
    // whole sky by a garbage transform).
    frameScatterMatrices_.clear();
    frameScatterMatrices_.reserve(16);
    frameBackdropMatrices_.push_back(composeShifted(atmosphereShellTransform_));
    frameBackdropMatrices_.push_back(composeShifted(skyCloudTransform_));
    frameBackdropMatrices_.push_back(composeShifted(cloudLayerTransform_));
    frameBackdropMatrices_.push_back(composeShifted(moonTransform_));
    frameBackdropMatrices_.push_back(composeShifted(sunDiscTransform_));
    frameBackdropMatrices_.push_back(composeShifted(starDomeTransform_));
    frameBackdropMatrices_.push_back(composeShifted(skyDomeTransform_));
    frameBackdropMatrices_.push_back(composeShifted(auroraTransform_));
    frameBackdropMatrices_.push_back(composeShifted(cirrusLayerTransform_));

    if (starDomeActive_ && starDome_ != renderer::invalidMesh && !starDomeMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = starDome_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[5]);
        command.material = &starDomeMaterials_.front();
        drawCommands_.push_back(command);
    }

    if (atmosphereShellActive_ && atmosphereShell_ != renderer::invalidMesh && !atmosphereShellMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = atmosphereShell_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[0]);
        command.material = &atmosphereShellMaterials_.front();
        drawCommands_.push_back(command);
    }

    // The sky dome (in-air views): the gradient's own geometry, drawn ahead
    // of the scene with depth writes off - the terrain the eye stands on
    // was pushed first and wins the sky-facing pixels by depth test, the
    // dome fills every sky pixel the terrain leaves, and the later sun disc
    // and cloud deck composite over it. Without this submission the sky's
    // only paint was the flat clear color plus the cloud deck's fringe.
    if (skyDomeActive_ && skyDome_ != renderer::invalidMesh && !skyDomeMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyDome_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[6]);
        command.material = &skyDomeMaterials_.front();
        drawCommands_.push_back(command);
    }

    // Frustum culling: the planet's visible-chunk selection spans the whole
    // horizon disk, so roughly half the set sits behind the camera plane and
    // never rasterizes a pixel. Sphere-vs-plane rejection (Gribb-Hartmann
    // planes from the clip matrix; the chunk's bounding sphere from its
    // octree depth) drops those before any command storage.
    glm::dvec4 frustumPlanes[6];
    extractFrustumPlanes(viewProjection, frustumPlanes);

    const double eyeAltitudeMeters = glm::length(eyePlanetLocal) - params_.radiusMeters;

    // Water roughness by eye altitude: the per-vertex wave noise fades with
    // the LOD band limit, so from above the sea is a sheet of flat coarse
    // facets and a tight specular term turns each facet into a separate
    // sharp glint - a field of sparkles around the sun point. The material
    // therefore widens toward the configured orbital gloss as the eye
    // climbs the atmosphere column: a broad, soft sun glint like the one
    // real oceans throw at orbital eye heights, instead of per-facet
    // sparkles; close-range views keep the configured sea texture.
    const double orbitBlend = planet::smoothstep(params_.atmosphereHeightMeters, 2.0 * params_.atmosphereHeightMeters,
        eyeAltitudeMeters);
    const float waterRoughness = static_cast<float>(
        params_.waterRoughness + (params_.waterRoughnessOrbital - params_.waterRoughness) * orbitBlend);

    // Aerial perspective: the atmosphere model's haze for the eye's sight
    // lines, composed as exact fog through the material - the renderer's
    // shading is albedo * light, so scaling baseColor by (1 - w) and setting
    // the emissive term to hazeColor * w yields terrainRadiance * (1 - w) +
    // haze * w per pixel: dark backlit slopes LIFT toward the haze color and
    // bright snow keeps poking through it, the way real air scatters.
    const planet::Vec3 eyeLocal{eyePlanetLocal.x, eyePlanetLocal.y, eyePlanetLocal.z};

    const glm::dvec3 planetCenter(params_.centerMeters[0], params_.centerMeters[1], params_.centerMeters[2]);

    // ---- pass 1: collect the surviving chunks ------------------------------
    // Cull against the frustum (sphere-vs-plane; the bounding sphere comes
    // from the chunk key's octree depth) and score each chunk by its squared
    // distance to the eye. The map's own iteration order is depth-major, so
    // the collect cannot emit directly.
    frameVisibleDraws_.clear();
    for (const auto &entry : uploadedMeshes_)
    {
        // Placement center in f64 (already planet-local + center): the
        // shift-aware culling and the command matrix both compose against
        // it, so chunk placement never quantizes at planet scale.
        const glm::dvec3 localCenter = entry.second.placementCenter - planetCenter;
        const double distanceSquared = glm::dot(localCenter - eyePlanetLocal, localCenter - eyePlanetLocal);

        std::uint32_t depth = 0;
        std::uint32_t cellX = 0;
        std::uint32_t cellY = 0;
        std::uint32_t cellZ = 0;
        planet::unpackChunkKey(entry.first, depth, cellX, cellY, cellZ);
        const double boundRadius = 0.5 * planet::chunkSizeMeters(params_, depth) * glm::sqrt(3.0);

        const glm::dvec3 worldCenter = entry.second.placementCenter - renderShift;
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
        frameVisibleDraws_.push_back(VisibleChunkDraw{&entry, distanceSquared});
    }

    // ---- pass 2: order near to far -----------------------------------------
    // Opaque draws are order-independent for the final image (depth test and
    // write decide the winner), but the rasterizer's per-sample depth
    // early-out fires far more often when the near geometry lands first:
    // the map's depth-major order hands the machine the far shells first,
    // which is exactly backwards. The chunk key is the deterministic
    // tie-break, so equal-distance chunks (concentric rings) keep a stable,
    // reproducible order; the winner of an equal-depth z-fight is therefore
    // a pure function of the scene, like every other frame.
    std::sort(frameVisibleDraws_.begin(), frameVisibleDraws_.end(),
        [](const VisibleChunkDraw &a, const VisibleChunkDraw &b)
        {
            if (a.distanceSquared != b.distanceSquared)
            {
                return a.distanceSquared < b.distanceSquared;
            }
            return a.entry->first < b.entry->first;
        });

    // ---- pass 3: emit (stable storage, then the commands pointing into it) -
    // The per-frame matrix and material stores are sized BEFORE any command
    // takes pointers into them: a growing vector reallocates and leaves
    // earlier commands pointing at freed memory, which only "works" while
    // the freed blocks still happen to hold their old bytes.
    frameChunkMatrices_.clear();
    frameChunkMatrices_.reserve(frameVisibleDraws_.size());
    drawCommands_.reserve(drawCommands_.size() + 2u * frameVisibleDraws_.size());
    for (const VisibleChunkDraw &draw : frameVisibleDraws_)
    {
        const ChunkMeshHandles &handles = draw.entry->second;
        // The command matrix composes from the f64 center (the stored f32
        // transform's translation is already planet-scale-quantized; the
        // vertex payload itself is chunk-local and small either way).
        const glm::dvec3 worldCenter = handles.placementCenter - renderShift;
        frameChunkMatrices_.push_back(glm::mat4(glm::translate(glm::dmat4(1.0), worldCenter)));

        // The aerial perspective keeps the REAL planet-local center - the
        // physics does not shift with the render origin.
        const planet::AerialPerspective haze
            = planet::aerialPerspective(params_, eyeLocal, planet::Vec3{draw.entry->second.placementCenter.x - planetCenter.x,
                                                                    draw.entry->second.placementCenter.y - planetCenter.y,
                                                                    draw.entry->second.placementCenter.z - planetCenter.z},
                sunTint);
        const float keepTerrain = static_cast<float>(1.0 - haze.weight);
        const float addHaze[3] = {static_cast<float>(haze.color.x * haze.weight),
            static_cast<float>(haze.color.y * haze.weight),
            static_cast<float>(haze.color.z * haze.weight)};

        if (handles.terrain != renderer::invalidMesh)
        {
            // This chunk's ground variant, hashed from the chunk address so
            // the pick is deterministic across frames and build orders (see
            // setTerrainTextures): neighboring chunks photograph different
            // scans and no single scan repeats across the visible disk. An
            // empty variant set (every scan failed to load) draws the group
            // untextured through the neutral stand-in, like a missing gate.
            static const TerrainTextureVariant untextured{};
            // Snow gate: the chunk's center climate classifies once, lazily
            // (the sample is cheap but per-frame would be waste). Snow-biome
            // chunks keep the detail multiply (the composed scan's blades
            // read as wind-textured snow at a distance) but drop the normal
            // scale hard - the blade-relief sheet magnified across a
            // snowfield photographs as combed parallel lines no real
            // snowpack shows, while a faint diffuse mottle is exactly the
            // wind-ripple texture fresh snow photographs with.
            if (!handles.groundSnowyBuilt)
            {
                handles.groundSnowyBuilt = true;
                if (planet_->sampleClimate != nullptr)
                {
                    const glm::dvec3 chunkLocalCenter
                        = glm::dvec3(handles.placementCenter.x - params_.centerMeters[0],
                            handles.placementCenter.y - params_.centerMeters[1],
                            handles.placementCenter.z - params_.centerMeters[2]);
                    const double centerRadius = glm::length(chunkLocalCenter);
                    if (centerRadius > 1.0e-6)
                    {
                        const double centerDirection[3] = {chunkLocalCenter.x / centerRadius,
                            chunkLocalCenter.y / centerRadius, chunkLocalCenter.z / centerRadius};
                        planet::ClimateSample centerSample;
                        if (planet_->sampleClimate(planet_->self, planetHandle_, centerDirection,
                                centerRadius, &centerSample))
                        {
                            handles.groundSnowy = centerSample.biome == planet::Biome::Snow;
                        }
                    }
                }
            }
            const bool groundTextured
                = !groundTextureVariants_.empty()
                && draw.distanceSquared <= groundTextureMaxDistanceMetersSquared_;
            const std::uint64_t chunkSeed = engine::planet::hashCombine64(
                draw.entry->first, static_cast<std::uint64_t>(groundTextureVariants_.size()));
            const TerrainTextureVariant &groundVariant = groundTextured
                ? groundTextureVariants_[static_cast<std::size_t>(chunkSeed % groundTextureVariants_.size())]
                : untextured;
            renderer::MeshMaterial material{};
            // Inside the texture gate the base color carries the SELECTED
            // variant's mean compensation (see setTerrainTextures), so the
            // multiply preserves the vertex albedo on average; outside the
            // gate neither applies and the boundary cannot read.
            material.baseColor[0] = keepTerrain * (groundTextured ? groundVariant.meanCompensation[0] : 1.0f);
            material.baseColor[1] = keepTerrain * (groundTextured ? groundVariant.meanCompensation[1] : 1.0f);
            material.baseColor[2] = keepTerrain * (groundTextured ? groundVariant.meanCompensation[2] : 1.0f);
            material.emissiveColor[0] = addHaze[0];
            material.emissiveColor[1] = addHaze[1];
            material.emissiveColor[2] = addHaze[2];
            material.roughnessFactor = 0.95f;
            // The ground detail multiplies in only where one tile still
            // spans its pixel budget (see the gate setup in main.cpp);
            // beyond the gate the minified sample could no longer move the
            // pixel and the sampler would only pay for it.
            material.baseColorTexture = groundTextured ? groundVariant.detail : renderer::invalidTexture;
            // Ground relief: variants that carry a normal map shade through
            // the same per-sample tangent frame the rock group uses, so
            // soil-crumb and leaf-litter relief lights with the sun instead
            // of the vertex gradient alone. The relief's own gate is a
            // third of the albedo gate's distance (see setTerrainTextures):
            // the sward's relief photographs well past the eye's feet
            // (flat vertex-gradient country is what an untextured sward
            // reads as at mid distance), and a minified relief map still
            // flickers where the albedo multiply no longer reads. The scale
            // stays under the rock group's: soil and sward
            // read as soft but present relief - strong enough that the
            // near-camera ground breaks its smooth vertex-gradient shading
            // (the clay look flat light gives untextured ground), while the
            // per-triangle tangent frame the backend rebuilds from the
            // tiled UVs stays continuous where the projection switches
            // axes.
            const bool groundRelief = groundTextured
                && groundVariant.normal != renderer::invalidTexture
                && draw.distanceSquared <= groundReliefMaxDistanceMetersSquared_;
            material.normalTexture = groundRelief ? groundVariant.normal : renderer::invalidTexture;
            material.normalScale = handles.groundSnowy ? 0.16f : 0.72f;
            // Procedural mottle (MeshMaterial::noiseScaleMeters): the world-
            // anchored value noise the renderer shades per pixel. The base
            // wavelength rides between the tussock scale and the soil-patch
            // scale a real sward photographs - the texture pair's gate ends
            // near the camera, the vertex colors carry the tens-of-metres
            // mottle, and this fills the band between with variation no
            // lattice repeats and no gate cuts off.
            material.noiseScaleMeters = 1.0f;
            material.noiseStrength = handles.groundSnowy ? 0.06f : 0.18f;
            // The planet mesher's terrain winding is fixed (see the
            // meshing module): declaring it enables exact per-triangle
            // back-face culling, so a rugged chunk's hidden far side
            // rejects per triangle instead of relying on the renderer's
            // per-mesh winding vote.
            material.frontFaceWinding = planet::terrainFrontFaceWinding;
            terrainMaterials_.push_back(material);

            renderer::RenderDrawCommand command{};
            command.mesh = handles.terrain;
            command.modelMatrix = glm::value_ptr(frameChunkMatrices_.back());
            command.material = &terrainMaterials_.back();
            drawCommands_.push_back(command);
        }
        if (handles.rock != renderer::invalidMesh)
        {
            // This chunk's rock variant (see the ground pick above); each
            // variant carries its own matching normal map so relief and
            // albedo stay the same scan.
            static const TerrainTextureVariant untextured{};
            const bool rockTextured
                = !rockTextureVariants_.empty() && draw.distanceSquared <= rockTextureMaxDistanceMetersSquared_;
            const std::uint64_t chunkSeed = engine::planet::hashCombine64(
                draw.entry->first, static_cast<std::uint64_t>(rockTextureVariants_.size()) + 0x9E3779B97F4A7C15ull);
            const TerrainTextureVariant &rockVariant = rockTextured
                ? rockTextureVariants_[static_cast<std::size_t>(chunkSeed % rockTextureVariants_.size())]
                : untextured;
            renderer::MeshMaterial material{};
            material.baseColor[0] = keepTerrain * (rockTextured ? rockVariant.meanCompensation[0] : 1.0f);
            material.baseColor[1] = keepTerrain * (rockTextured ? rockVariant.meanCompensation[1] : 1.0f);
            material.baseColor[2] = keepTerrain * (rockTextured ? rockVariant.meanCompensation[2] : 1.0f);
            material.emissiveColor[0] = addHaze[0];
            material.emissiveColor[1] = addHaze[1];
            material.emissiveColor[2] = addHaze[2];
            material.roughnessFactor = 0.85f;
            // Rock tiles span twice the ground tile's meters, so the rock
            // texture stays useful to twice the distance (same pixel
            // budget, see main.cpp).
            material.baseColorTexture = rockTextured ? rockVariant.detail : renderer::invalidTexture;
            material.normalTexture = rockTextured ? rockVariant.normal : renderer::invalidTexture;
            // Tangent-space relief: the headless backend rebuilds the tangent
            // frame per triangle from the tiled UVs, so the value stays
            // moderate to keep the frame continuous where the projection
            // switches axes - but strong enough that cliff faces read as
            // fractured rock under the sun instead of painted tone (the
            // cliff-side and rock-face scans carry real crack geometry in
            // their normal maps, and a timid scale erases it).
            material.normalScale = 0.75f;
            // The rock group's mottle rides a coarser scale: strata-scale
            // tonal drift (the same field the ground group samples, so a
            // slope's grass-to-rock handover stays continuous).
            material.noiseScaleMeters = 2.2f;
            material.noiseStrength = 0.14f;
            material.frontFaceWinding = planet::terrainFrontFaceWinding;
            rockMaterials_.push_back(material);

            renderer::RenderDrawCommand command{};
            command.mesh = handles.rock;
            command.modelMatrix = glm::value_ptr(frameChunkMatrices_.back());
            command.material = &rockMaterials_.back();
            drawCommands_.push_back(command);
        }
        if (handles.water != renderer::invalidMesh)
        {
            renderer::MeshMaterial material{};
            material.baseColor[0] = keepTerrain;
            material.baseColor[1] = keepTerrain;
            material.baseColor[2] = keepTerrain;
            material.emissiveColor[0] = addHaze[0];
            material.emissiveColor[1] = addHaze[1];
            material.emissiveColor[2] = addHaze[2];
            material.roughnessFactor = waterRoughness;
            // Water is a DIELECTRIC mirror: the sun glint and the grazing
            // sky reflection carry most of its radiance from orbit (the
            // classic bright ocean glint wedge), while the diffuse share
            // stays the deep body color. Roughness follows the sea state
            // (see waterRoughnessOrbital above): calm seas concentrate the
            // glint, rough seas spread it.
            material.metallicFactor = 0.45f;
            // Close-range sea relief: inside the near-water gate the draw
            // samples the wave-normal sheet (sun glints break across the
            // wave facets) and the albedo variation sheet (the depth tint
            // ripples instead of reading as a flat fill). Beyond the gate
            // both drop away - a minified normal sheet aliases into
            // open-ocean sparkle, and the orbital gloss owns the glint.
            const bool waterTextured = waterNormalTexture_ != renderer::invalidTexture
                && draw.distanceSquared <= waterTextureMaxDistanceMetersSquared_;
            material.baseColorTexture = waterTextured ? waterAlbedoTexture_ : renderer::invalidTexture;
            material.normalTexture = waterTextured ? waterNormalTexture_ : renderer::invalidTexture;
            material.normalScale = 0.55f;
            // The sea sheet is a thin surface a swimmer sees from below as
            // well as above: double-sided keeps the sheet visible
            // underwater (the shading normal flips toward the viewer's
            // side) and disables the per-triangle culling that would
            // otherwise drop the seen-from-below surface.
            material.doubleSided = true;
            waterMaterials_.push_back(material);

            renderer::RenderDrawCommand command{};
            command.mesh = handles.water;
            command.modelMatrix = glm::value_ptr(frameChunkMatrices_.back());
            command.material = &waterMaterials_.back();
            drawCommands_.push_back(command);
        }
    }

    // Surface scatter (opaque pass): vegetation, surface rocks and birds
    // right after the terrain they stand on (depth-tested against it, depth
    // writes on - the alpha-cutout foliage keeps its own depth consistent).
    // The matrices compose the scatter anchors exactly like the chunk
    // placements: f64 subtract of the render shift, then the f32 pipeline
    // sees eye-relative coordinates only.
    if (surfaceScatter_ != nullptr)
    {
        const std::vector<renderer::RenderDrawCommand> &scatterCommands = surfaceScatter_->drawCommands();
        const std::vector<glm::dvec3> &scatterAnchors = surfaceScatter_->commandAnchors();
        for (std::size_t index = 0; index < scatterCommands.size(); ++index)
        {
            const renderer::MeshMaterial *material = scatterCommands[index].material;
            const bool transparent = material != nullptr && material->opacity < 1.0f;
            if (transparent)
            {
                continue; // foam and mist ride the transparent tail below
            }
            const glm::dmat4 placement = glm::translate(
                glm::dmat4(1.0), planetCenter + scatterAnchors[index]);
            frameScatterMatrices_.push_back(composeShifted(glm::mat4(placement)));
            renderer::RenderDrawCommand command = scatterCommands[index];
            command.modelMatrix = glm::value_ptr(frameScatterMatrices_.back());
            drawCommands_.push_back(command);
        }
    }

    // The moon rides the backdrop list AFTER the sun disc and BEFORE the
    // sky dome: the depth comparison is strict-less, so a companion placed
    // just inside the dome's radius both survives the later dome draw (the
    // dome's equal-or-farther fragments fail against the moon's depth) and
    // stays hidden behind the nearer cloud deck's banks. Its own depth
    // writes then hide the stars behind it.
    if (moonActive_ && moon_ != renderer::invalidMesh && !moonMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = moon_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[3]);
        command.material = &moonMaterials_.front();
        drawCommands_.push_back(command);
    }
    // Backdrop shells behind and above the scene: the cloud geometry first
    // (depth-tested against the terrain drawn above), then the unlit space
    // shells and the sky dome. See the draw-order comment at the top.
    if (skyCloudActive_ && skyCloudVisible_ && skyCloudLayer_ != renderer::invalidMesh && !skyCloudMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyCloudLayer_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[1]);
        command.material = &skyCloudMaterials_.front();
        drawCommands_.push_back(command);
    }
    if (cloudLayerActive_ && cloudLayerVisible_ && cloudLayer_ != renderer::invalidMesh
        && !cloudLayerMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = cloudLayer_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[2]);
        command.material = &cloudLayerMaterials_.front();
        drawCommands_.push_back(command);
    }
    // The sun disc follows the decks (see its comment above).
    if (sunDiscActive_ && sunDisc_ != renderer::invalidMesh && !sunDiscMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = sunDisc_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[4]);
        command.material = &sunDiscMaterials_.front();
        drawCommands_.push_back(command);
    }
    // The aurora is translucent (opacity < 1): the backend routes it into
    // the transparent pass regardless of its list position, alpha-blending
    // it over the finished opaque scene. Emitted ahead of the cirrus veil
    // and the transparent scatter tail so the command list reads far-to-near
    // inside its own class (the rings sit behind everything but the star
    // dome; the cirrus deck and the near mist banks follow, each nearer).
    if (auroraActive_ && aurora_ != renderer::invalidMesh && !auroraMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = aurora_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[7]);
        command.material = &auroraMaterials_.front();
        drawCommands_.push_back(command);
    }
    // The cirrus veil rides the transparent class AFTER the aurora (its
    // deck sits below the curtain's altitude, so from an orbital eye it is
    // the nearer surface of the two) and stays ahead of the transparent
    // scatter tail - the aurora stays the farthest member of the class,
    // the near mist banks the nearest, and every member composites over the
    // farther ones the way alpha blending assumes.
    if (cirrusLayerActive_ && cirrusLayer_ != renderer::invalidMesh && !cirrusLayerMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = cirrusLayer_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[8]);
        command.material = &cirrusLayerMaterials_.front();
        drawCommands_.push_back(command);
    }

    // Surface scatter (transparent tail): breaking-wave foam and valley mist
    // alpha-blend over the finished opaque scene, depth writes off. The
    // banks re-roll their pool slots on a churn interval, so slot order is
    // depth-chaotic; the tail is ordered far to near by anchor distance
    // (chunk key as the deterministic tie-break) so overlapping banks
    // composite back-to-front the way alpha blending assumes - a near mist
    // bank gathers the far bank behind it instead of stranding its order in
    // the pool's churn. Stable ordering also keeps the aurora (emitted just
    // above, planet-scale distance) the farthest member of the class.
    if (surfaceScatter_ != nullptr)
    {
        const std::vector<renderer::RenderDrawCommand> &scatterCommands = surfaceScatter_->drawCommands();
        const std::vector<glm::dvec3> &scatterAnchors = surfaceScatter_->commandAnchors();
        frameTransparentScatter_.clear();
        for (std::size_t index = 0; index < scatterCommands.size(); ++index)
        {
            const renderer::MeshMaterial *material = scatterCommands[index].material;
            const bool transparent = material != nullptr && material->opacity < 1.0f;
            if (!transparent)
            {
                continue;
            }
            // Planet-local distance: the anchors and the eye both live in
            // planet-local meters (the model matrix adds planetCenter back
            // for the f32 pipeline) - the same frame the chunk ordering uses.
            const glm::dvec3 eyeVector = scatterAnchors[index] - eyePlanetLocal;
            frameTransparentScatter_.push_back(
                TransparentScatterDraw{index, glm::dot(eyeVector, eyeVector)});
        }
        std::sort(frameTransparentScatter_.begin(), frameTransparentScatter_.end(),
            [](const TransparentScatterDraw &a, const TransparentScatterDraw &b)
            {
                if (a.distanceSquared != b.distanceSquared)
                {
                    return a.distanceSquared > b.distanceSquared;
                }
                return a.index < b.index;
            });
        for (const TransparentScatterDraw &draw : frameTransparentScatter_)
        {
            const glm::dmat4 placement = glm::translate(
                glm::dmat4(1.0), planetCenter + scatterAnchors[draw.index]);
            frameScatterMatrices_.push_back(composeShifted(glm::mat4(placement)));
            renderer::RenderDrawCommand command = scatterCommands[draw.index];
            command.modelMatrix = glm::value_ptr(frameScatterMatrices_.back());
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

namespace
{
// Equirectangular bake mapping (the space cloud bake's own convention):
// u wraps the azimuth around the spin axis (+Y), v runs 0 at the north
// pole to 1 at the south pole, matching TextureOrigin::TopLeft rows (row
// 0 = north). The bake texels and the shell's UVs share the convention,
// so a texel is baked for exactly the direction its UV maps.
planet::Vec3 equirectDirection(std::uint32_t x, std::uint32_t y, std::uint32_t width, std::uint32_t height)
{
    const double azimuth = glm::two_pi<double>() * (static_cast<double>(x) + 0.5) / static_cast<double>(width);
    const double latitude = glm::pi<double>() * 0.5
        - glm::pi<double>() * (static_cast<double>(y) + 0.5) / static_cast<double>(height);
    const double cosLatitude = std::cos(latitude);
    return planet::Vec3{std::cos(azimuth) * cosLatitude, std::sin(latitude),
        std::sin(azimuth) * cosLatitude};
}

// Low-frequency domain warp shared by both cloud bakes (the space shell and
// the ground deck): bending the sample position before the coverage lookup
// breaks the wind shear's aligned streaking the way real turbulence does -
// the cyclone arms and wisp fringes keep their shapes but lose the combed,
// single-direction alignment a straight lookup shows. Two octaves of bend: a
// broad one at a fraction of the field's own feature width (the bank-scale
// meander) and a fine one an order of magnitude smaller (the turbulence that
// breaks the wind shear's aligned, combed streaking). Pure in (params,
// direction), so both bakes and every rebuild describe the same warped field.
planet::Vec3 warpCloudSampleDirection(const planet::PlanetGenerationParams &params, const planet::Vec3 &direction)
{
    const std::int64_t warpSeed = static_cast<std::int64_t>(params.seed) + 0x2B7E151628AED2A6LL;
    double bendAccumX = 0.0;
    double bendAccumY = 0.0;
    planet::Vec3 tangentA{};
    planet::Vec3 tangentB{};
    for (int octave = 0; octave < 2; ++octave)
    {
        const double featureMeters = octave == 0
            ? 0.6 * std::max(1.0, params.cloudWavelengthMeters)
            : 0.07 * std::max(1.0, params.cloudWavelengthMeters);
        const double amplitudeMeters = octave == 0
            ? 0.30 * std::max(1.0, params.cloudWavelengthMeters)
            : 0.045 * std::max(1.0, params.cloudWavelengthMeters);
        const double frequency = 1.0 / featureMeters;
        const std::int64_t octaveSeed = warpSeed + octave;
        const double bx = planet::noise::simplex3ImproveXY(octaveSeed,
            direction.x * frequency, direction.y * frequency, direction.z * frequency);
        const double by = planet::noise::simplex3ImproveXY(octaveSeed + 1,
            direction.x * frequency, direction.y * frequency, direction.z * frequency);
        if (octave == 0)
        {
            // Two tangents at the sample point carry the bend (any orthonormal
            // pair spans the tangent plane; smoothness matters, not orientation).
            tangentA = std::abs(direction.y) < 0.9
                ? planet::Vec3{-direction.y, direction.x, 0.0}
                : planet::Vec3{1.0, 0.0, 0.0};
            tangentA = planet::normalized(tangentA);
            tangentB = planet::normalized(planet::cross(direction, tangentA));
        }
        bendAccumX += bx * amplitudeMeters;
        bendAccumY += by * amplitudeMeters;
    }
    const double totalBend = (0.30 + 0.045) * std::max(1.0, params.cloudWavelengthMeters);
    const double bendRadians = totalBend / std::max(1.0, params.radiusMeters);
    return planet::normalized(direction + tangentA * (bendAccumX / totalBend * bendRadians)
        + tangentB * (bendAccumY / totalBend * bendRadians));
}

// Bilinear alpha lookup in the renderer's own cloud bake (see
// ensureCloudTexture): the mesh builders gate their quads against exactly the
// pixels the shell will draw, so the geometry cutout and the texture's own
// thinning fringes agree. u wraps (the seam column repeats column 0).
double cloudBakeAlpha(const std::vector<std::uint8_t> &pixels, std::uint32_t width, std::uint32_t height,
    double u, double v)
{
    u = u - std::floor(u);
    v = std::clamp(v, 0.0, 1.0);
    const double x = u * static_cast<double>(width) - 0.5;
    const double y = v * static_cast<double>(height) - 0.5;
    const std::int64_t x0 = static_cast<std::int64_t>(std::floor(x)) - (x < 0.0 ? 1 : 0);
    const std::int64_t y0 = static_cast<std::int64_t>(std::floor(y)) - (y < 0.0 ? 1 : 0);
    const double fx = x - static_cast<double>(x0);
    const double fy = y - static_cast<double>(y0);
    const auto texel = [&](std::int64_t column, std::int64_t row)
    {
        column = ((column % static_cast<std::int64_t>(width)) + width) % static_cast<std::int64_t>(width);
        row = std::clamp(row, static_cast<std::int64_t>(0), static_cast<std::int64_t>(height) - 1);
        const std::size_t index
            = (static_cast<std::size_t>(row) * width + static_cast<std::size_t>(column)) * 4u + 3u;
        return static_cast<double>(pixels[index]) / 255.0;
    };
    const double a = texel(x0, y0);
    const double b = texel(x0 + 1, y0);
    const double c = texel(x0, y0 + 1);
    const double d = texel(x0 + 1, y0 + 1);
    return (a * (1.0 - fx) + b * fx) * (1.0 - fy) + (c * (1.0 - fx) + d * fx) * fy;
}
} // namespace

void PlanetChunkRenderer::ensureCloudTexture(const glm::dvec3 &sunDirection)
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || cloudTexture_ != renderer::invalidTexture)
    {
        return;
    }

    if (cloudField_ == nullptr)
    {
        cloudField_ = std::make_unique<planet::CloudField>(params_);
    }

    // One equirect bake serves both cloud consumers (the world-anchored
    // shell the macro views draw and the sky deck the ground views draw):
    // 2048x1024 keeps the texel footprint under the finest wisp octave's
    // feature size on a small planet, so the full field resolves - the
    // cyclone arms, the wisp fringes and the bank bodies all draw at their
    // own scale instead of band-limiting to whatever grid a mesh could
    // carry. At planet scale the equatorial texel grows past the METRIC
    // octaves (cumulus clumps, fringes), so the bake band-limits them out
    // through the coverageTexel cutoff instead of aliasing them into
    // salt-and-pepper flecks: from orbit a real cumulus deck integrates
    // into its bank's brightness anyway (real Earth photography resolves
    // weather systems, not individual cumulus).
    constexpr std::uint32_t kWidth = 2048;
    constexpr std::uint32_t kHeight = 1024;
    // The cutoff rides the equatorial texel footprint (2 * pi * R / width):
    // equirect columns converge toward the poles, so the equator's footprint
    // is the COARSEST column spacing and the safest single bound.
    const double texelFootprintMeters
        = glm::two_pi<double>() * (params_.radiusMeters + params_.cloudAltitudeMeters)
            / static_cast<double>(kWidth);
    const bool sunValid = glm::length(sunDirection) > 1.0e-9;
    const glm::dvec3 sun = sunValid ? glm::normalize(sunDirection) : glm::dvec3(0.0, 1.0, 0.0);
    const glm::dvec3 ambientSky = backdropLightingValid_
        ? glm::dvec3(backdropLighting_.ambientSky.x, backdropLighting_.ambientSky.y, backdropLighting_.ambientSky.z)
        : glm::dvec3(0.08, 0.10, 0.16);
    const glm::dvec3 ambientGround = backdropLightingValid_
        ? glm::dvec3(backdropLighting_.ambientGround.x, backdropLighting_.ambientGround.y,
            backdropLighting_.ambientGround.z)
        : glm::dvec3(0.05, 0.045, 0.04);
    const glm::dvec3 sunRadiance = backdropLightingValid_ && backdropLighting_.sunEnabled
        ? glm::dvec3(backdropLighting_.sunRadianceOverPi.x, backdropLighting_.sunRadianceOverPi.y,
            backdropLighting_.sunRadianceOverPi.z)
        : glm::dvec3(0.0);

    // Phase 1: the coverage field alone (see the deck bake's recipe below -
    // the shell follows the same three-phase structure): the raw thresholded
    // coverage photographs as torn flakes from orbit, because every wisp
    // octave's single-texel threshold crossing becomes an isolated fleck
    // with a hard one-texel edge. Storing the coverage separately lets the
    // fringe blur dissolve those flecks before anything reads them.
    std::vector<std::uint8_t> coveragePixels(static_cast<std::size_t>(kWidth) * kHeight, 0u);
    for (std::uint32_t y = 0; y < kHeight; ++y)
    {
        for (std::uint32_t x = 0; x < kWidth; ++x)
        {
            const planet::Vec3 direction = equirectDirection(x, y, kWidth, kHeight);
            const double coverage = planet::clamp01(cloudField_->coverageTexel(
                warpCloudSampleDirection(params_, direction), 1.0, texelFootprintMeters));
            if (coverage <= 0.003)
            {
                continue; // clear texel: keep the coverage buffer at zero
            }
            coveragePixels[static_cast<std::size_t>(y) * kWidth + x]
                = static_cast<std::uint8_t>(coverage * 255.0);
        }
    }

    // Phase 2: fringe softening - the deck bake's own three separable
    // 3-tap passes over the coverage buffer. From orbit a 2048-wide bake
    // minifies hard, and the mip chain would average the flecks away only
    // where the deck minifies; the shell's near-limb texels photograph at
    // one-to-one, where a hard one-texel flake edge survives every filter.
    // The blur dissolves them at the source, keeping the bank bodies (whose
    // coverage rides well above the fringe tail) untouched.
    for (int blurPass = 0; blurPass < 3; ++blurPass)
    {
        std::vector<std::uint8_t> blurred(coveragePixels.size());
        const auto alphaAt = [&](std::int64_t column, std::int64_t row)
        {
            column = ((column % static_cast<std::int64_t>(kWidth)) + static_cast<std::int64_t>(kWidth))
                % static_cast<std::int64_t>(kWidth);
            row = std::clamp(row, std::int64_t{0}, static_cast<std::int64_t>(kHeight) - 1);
            return static_cast<double>(coveragePixels[static_cast<std::size_t>(row) * kWidth
                + static_cast<std::size_t>(column)]);
        };
        for (std::uint32_t y = 0; y < kHeight; ++y)
        {
            for (std::uint32_t x = 0; x < kWidth; ++x)
            {
                const double horizontal = (alphaAt(x - 1, y) + 2.0 * alphaAt(x, y) + alphaAt(x + 1, y)) * 0.25;
                const double vertical = (alphaAt(x, y - 1) + 2.0 * alphaAt(x, y) + alphaAt(x, y + 1)) * 0.25;
                blurred[static_cast<std::size_t>(y) * kWidth + x]
                    = static_cast<std::uint8_t>(std::clamp((horizontal + vertical) * 0.5, 0.0, 255.0));
            }
        }
        coveragePixels.swap(blurred);
    }

    // Phase 3: the lit RGB, derived from the BLURRED coverage for every
    // texel whose alpha survived it (deriving RGB from the raw field left
    // the blur's fringe texels carrying alpha with a zero RGB - the dark
    // outline band the deck bake's own note describes).
    cloudTexturePixels_.assign(static_cast<std::size_t>(kWidth) * kHeight * 4u, 0u);
    for (std::uint32_t y = 0; y < kHeight; ++y)
    {
        for (std::uint32_t x = 0; x < kWidth; ++x)
        {
            const double coverage = static_cast<double>(
                coveragePixels[static_cast<std::size_t>(y) * kWidth + x]) / 255.0;
            if (coverage <= 0.003)
            {
                continue; // clear texel: keep RGBA at zero (the blend skips it)
            }
            const planet::Vec3 direction = equirectDirection(x, y, kWidth, kHeight);

            // Optical thickness ramp: thick cores scatter nearly all the
            // incident light while the thinning rims transmit part of it, so
            // the bank reads as a volume of scatterers; pow keeps the rim's
            // own curvature where it matters (a linear ramp reads as paint).
            // The floor sits high: a real cloud deck's shaded bases still
            // scatter ~60% of the incident light (clouds are near-white
            // scatterers at every thickness), and bases darker than that
            // photograph as gray paint smears over the terrain.
            const double thickness = std::pow(coverage, 0.7);
            // Bank-top mottle: two seed-fixed simplex octaves at cloud-field
            // feature fractions (a few kilometres on the showcase), riding
            // the shading the way real decks photograph - a bank's interior
            // carries bright crests and dimmer troughs from the convection
            // that built it. Without this the minified bake reads as smooth
            // poured cream: the thresholded coverage saturates thick
            // interiors to one flat shade, and nothing inside a bank
            // suggests the volume it stands for. The jitter stays gentle
            // (the deck must stay near-white) but breaks every interior.
            const double bankFreqA = params_.radiusMeters / 3000.0;
            const double bankFreqB = params_.radiusMeters / 1200.0;
            const double bankJitterA = planet::noise::simplex3ImproveXY(
                static_cast<std::int64_t>(params_.seed) + 0x9E3779B9LL,
                direction.x * bankFreqA, direction.y * bankFreqA, direction.z * bankFreqA);
            const double bankJitterB = planet::noise::simplex3ImproveXY(
                static_cast<std::int64_t>(params_.seed) + 0x71E9AD24LL,
                direction.x * bankFreqB, direction.y * bankFreqB, direction.z * bankFreqB);
            const double shade = (0.52 + 0.48 * thickness)
                * (1.0 + 0.10 * bankJitterA + 0.06 * bankJitterB);
            // Sun lighting: the same environment bake the lit terrain
            // receives (hemispheric ambient plus the sun's Lambertian term),
            // evaluated per texel - bright banks under the sun, cool banks
            // at the limb, ambient-only banks through the night side. The
            // ambient rides a twilight gate: a real night-side deck is
            // black (only airglow and city light lift it, and neither lives
            // here), and a constant ambient floor photographs the night side
            // as permanent dusk, which flattens the terminator into the
            // mushy gradient real Earth imagery never shows - the day/night
            // boundary must fall to black within a few degrees of the
            // terminator for the disc to read as lit by one distant sun.
            const double sunDot = sunValid
                ? planet::clamp01(planet::dot(direction,
                    planet::Vec3{sun.x, sun.y, sun.z}))
                : 1.0;
            const double upMix = direction.y * 0.5 + 0.5;
            const glm::dvec3 ambient = glm::mix(ambientGround, ambientSky, upMix);
            const double nightGate = planet::smoothstep(-0.10, 0.06, sunDot);
            glm::dvec3 light = ambient * nightGate + sunRadiance * sunDot;
            // Terminator silver lining: banks whose tops graze the sun ray
            // forward-scatter strongly, so the cloud rims near the terminator
            // catch a warm bright edge the way sunrise photos show. The gate
            // is the twilight band itself - the lining fades OUT through the
            // terminator (a lining that survived into the night would light
            // the dark side's banks like a second sun).
            const double silver = 0.55 * planet::smoothstep(-0.02, 0.10, sunDot)
                * (1.0 - planet::smoothstep(0.10, 0.35, sunDot))
                * std::pow(coverage, 0.5);
            light += silver * glm::dvec3(1.0, 0.78, 0.55);

            const std::size_t index = (static_cast<std::size_t>(y) * kWidth + x) * 4u;
            for (int channel = 0; channel < 3; ++channel)
            {
                const double value = params_.cloudColor[channel] * shade
                    * light[channel] * (channel == 0 ? 1.0 : channel == 1 ? 0.99 : 0.97);
                cloudTexturePixels_[index + static_cast<std::size_t>(channel)]
                    = static_cast<std::uint8_t>(std::clamp(value, 0.0, 1.0) * 255.0);
            }
            cloudTexturePixels_[index + 3u] = static_cast<std::uint8_t>(coverage * 255.0);
        }
    }

    // Relief pass: the coverage field itself is the deck's height proxy -
    // its per-texel gradient against the sun's tangential projection shades
    // the billow slopes (sun-facing lobes brighten, lee lobes dim), the
    // puffy self-shaded read a flat albedo texture cannot carry. The slope
    // magnitude stays small: this is relief on top of the sun bake, not a
    // replacement for it.
    {
        const auto alphaAt = [&](std::int64_t column, std::int64_t row)
        {
            column = ((column % static_cast<std::int64_t>(kWidth)) + static_cast<std::int64_t>(kWidth))
                % static_cast<std::int64_t>(kWidth);
            row = std::clamp(row, std::int64_t{0}, static_cast<std::int64_t>(kHeight) - 1);
            return static_cast<double>(cloudTexturePixels_[(static_cast<std::size_t>(row) * kWidth
                + static_cast<std::size_t>(column)) * 4u + 3u]) / 255.0;
        };
        std::vector<double> relief(kWidth * kHeight, 0.0);
        for (std::uint32_t y = 0; y < kHeight; ++y)
        {
            const double latitude = glm::pi<double>() * 0.5
                - glm::pi<double>() * (static_cast<double>(y) + 0.5) / static_cast<double>(kHeight);
            const double cosLatitude = std::max(1.0e-6, std::cos(latitude));
            for (std::uint32_t x = 0; x < kWidth; ++x)
            {
                const double azimuth = glm::two_pi<double>() * (static_cast<double>(x) + 0.5)
                    / static_cast<double>(kWidth);
                // The sun's projection onto the texel's own tangent frame:
                // east (u+) and north (v-, since v grows toward the south
                // pole in the bake's row order).
                const planet::Vec3 east{-std::sin(azimuth), 0.0, std::cos(azimuth)};
                const planet::Vec3 north{-std::sin(latitude) * std::cos(azimuth), std::cos(latitude),
                    -std::sin(latitude) * std::sin(azimuth)};
                const planet::Vec3 sunVector{sun.x, sun.y, sun.z};
                const double sunEast = planet::dot(east, sunVector);
                const double sunNorth = planet::dot(north, sunVector);
                const std::size_t texel = static_cast<std::size_t>(y) * kWidth + x;
                const double center = alphaAt(x, y);
                if (center <= 0.02)
                {
                    continue; // clear sky has no relief
                }
                // Height gradient in texture space (per texel step).
                const double dAlphaDu = (alphaAt(x + 1, y) - alphaAt(x - 1, y)) * 0.5;
                const double dAlphaDv = (alphaAt(x, y + 1) - alphaAt(x, y - 1)) * 0.5;
                // East shrinks toward the poles (u steps shrink by cos of
                // latitude); normalize the physical slope by that factor.
                const double slope = dAlphaDu * sunEast / cosLatitude - dAlphaDv * sunNorth;
                relief[texel] = std::clamp(2.0 * slope, -0.45, 0.45);
            }
        }
        for (std::size_t texel = 0; texel < relief.size(); ++texel)
        {
            const double gain = 1.0 + relief[texel];
            for (int channel = 0; channel < 3; ++channel)
            {
                const std::size_t index = texel * 4u + static_cast<std::size_t>(channel);
                const double value = static_cast<double>(cloudTexturePixels_[index]) * gain;
                cloudTexturePixels_[index] = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
            }
        }
    }

    // Post-bake fringe softening: three separable 3-tap passes over the alpha
    // channel (a wide effective pyramid). The wisp octaves' finest fringe
    // rides at under two texels, and the threshold band maps it to
    // single-texel on/off flecks at the bank boundaries - salt-and-pepper
    // that reads as dithered noise, not ragged cloud. The wider fringe also
    // carries the alpha through the mesh gate's own coverage floor, so the
    // quads the gate keeps never show their polygonal edges against the
    // texels they cut off.
    for (int blurPass = 0; blurPass < 3; ++blurPass)
    {
        std::vector<std::uint8_t> blurred(cloudTexturePixels_.size());
        const auto alphaAt = [&](std::int64_t column, std::int64_t row)
        {
            column = ((column % static_cast<std::int64_t>(kWidth)) + static_cast<std::int64_t>(kWidth))
                % static_cast<std::int64_t>(kWidth);
            row = std::clamp(row, std::int64_t{0}, static_cast<std::int64_t>(kHeight) - 1);
            return static_cast<double>(cloudTexturePixels_[(static_cast<std::size_t>(row) * kWidth
                + static_cast<std::size_t>(column)) * 4u + 3u]);
        };
        for (std::uint32_t y = 0; y < kHeight; ++y)
        {
            for (std::uint32_t x = 0; x < kWidth; ++x)
            {
                const double horizontal = (alphaAt(x - 1, y) + 2.0 * alphaAt(x, y) + alphaAt(x + 1, y)) * 0.25;
                const double vertical = (alphaAt(x, y - 1) + 2.0 * alphaAt(x, y) + alphaAt(x, y + 1)) * 0.25;
                const std::size_t index = (static_cast<std::size_t>(y) * kWidth + x) * 4u;
                blurred[index + 0u] = cloudTexturePixels_[index + 0u];
                blurred[index + 1u] = cloudTexturePixels_[index + 1u];
                blurred[index + 2u] = cloudTexturePixels_[index + 2u];
                blurred[index + 3u] = static_cast<std::uint8_t>(std::clamp((horizontal + vertical) * 0.5, 0.0, 255.0));
            }
        }
        cloudTexturePixels_.swap(blurred);
    }

    renderer::TextureDescription description;
    description.width = kWidth;
    description.height = kHeight;
    description.pixels = cloudTexturePixels_.data();
    description.filter = renderer::TextureFilter::Linear;
    description.address = renderer::TextureAddress::Repeat;
    renderer::RenderError error = renderer::RenderError::None;
    cloudTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &description, &error);
    if (cloudTexture_ == renderer::invalidTexture)
    {
        cloudTexturePixels_.clear();
        return;
    }
    cloudTextureWidth_ = kWidth;
    cloudTextureHeight_ = kHeight;

    // Optional bake inspection: CLOUD_BAKE_DUMP writes the raw RGBA bake to
    // a PNG beside the run (tuning the field's coverage/structure is far
    // easier against the exact pixels the shell draws).
    if (std::getenv("CLOUD_BAKE_DUMP") != nullptr)
    {
        std::vector<std::uint8_t> rgb(static_cast<std::size_t>(kWidth) * kHeight * 3u);
        for (std::size_t texel = 0; texel < static_cast<std::size_t>(kWidth) * kHeight; ++texel)
        {
            rgb[texel * 3u + 0u] = cloudTexturePixels_[texel * 4u + 0u];
            rgb[texel * 3u + 1u] = cloudTexturePixels_[texel * 4u + 1u];
            rgb[texel * 3u + 2u] = cloudTexturePixels_[texel * 4u + 2u];
        }
        const char *rawPath = std::getenv("CLOUD_BAKE_DUMP");
        const std::string ppmPath = std::string(rawPath) + ".ppm";
        std::FILE *out = std::fopen(ppmPath.c_str(), "wb");
        if (out != nullptr)
        {
            std::fprintf(out, "P6\n%u %u\n255\n", kWidth, kHeight);
            std::fwrite(rgb.data(), 1u, rgb.size(), out);
            std::fclose(out);
        }
    }
}

void PlanetChunkRenderer::destroyCloudTexture()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && cloudTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, cloudTexture_);
    }
    cloudTexture_ = renderer::invalidTexture;
    cloudTexturePixels_.clear();
    cloudTextureWidth_ = 0;
    cloudTextureHeight_ = 0;
}

void PlanetChunkRenderer::ensureWaterDetailTextures()
{
    if (renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || waterNormalTexture_ != renderer::invalidTexture || params_.waterTextureTileMeters <= 0.0)
    {
        return;
    }

    // Close-range sea surface detail, baked as a pair of tileable sheets the
    // near-water draws sample over the mesher's own 8 m tile lattice (the
    // lattice's per-chunk quarter-turns and UV warp keep the tile from
    // reading as a repeat):
    //   - a wave NORMAL sheet: a sum of directional sine waves whose cycle
    //     counts are integers over the tile (perfectly tileable), spanning
    //     swell to chop. The set SPREADS over azimuths around one dominant
    //     wind axis instead of marching along a single direction: waves all
    //     parallel to one axis minify into the parallel brightness bands of
    //     a brushed-metal sheet (the venetian-blind sea a single-axis set
    //     photographed as), while a real sea runs a spectrum of crossed
    //     swells whose glints break into irregular sparkle.
    //   - an ALBEDO variation sheet: a low-contrast brightness mottle (mean
    //     exactly 1.0) built from periodic value noise instead of directional
    //     ripple bands - banded ripples minified into the same stripe read as
    //     the normal sheet's, and the eye then sees stripes everywhere it
    //     looks. Three wrapped noise octaves give the depth tint the uneven,
    //     directionless breakup a sunlit sea photographs.
    // Both gate by draw distance (see the water emit): a normal sheet
    // minified to sub-pixel waves aliases into sparkle noise.
    constexpr std::uint32_t kSize = 256;
    // Wave set: {cycles per tile along u, cycles per tile along v, amplitude,
    // phase} - integer cycles keep the sum seamless across the wrap. The
    // directions fan out around the dominant swell axis (veered sets at
    // +-20-50 degrees plus cross swells) so no direction owns the sheet.
    const struct
    {
        double cu, cv, amplitude, phase;
    } waves[] = {
        {3.0, 1.0, 0.034, 0.0},
        {7.0, 3.0, 0.026, 2.1},
        {13.0, 6.0, 0.018, 4.2},
        {4.0, 3.0, 0.022, 1.3},
        {9.0, 8.0, 0.016, 5.0},
        {6.0, 5.0, 0.014, 0.9},
        {2.0, 3.0, 0.018, 3.4},
        {5.0, 6.0, 0.012, 5.8},
        {11.0, 4.0, 0.010, 2.6},
    };
    std::vector<std::uint8_t> normalPixels(static_cast<std::size_t>(kSize) * kSize * 4u, 255u);
    std::vector<std::uint8_t> albedoPixels(static_cast<std::size_t>(kSize) * kSize * 4u, 255u);
    double albedoSum = 0.0;
    for (std::uint32_t y = 0; y < kSize; ++y)
    {
        for (std::uint32_t x = 0; x < kSize; ++x)
        {
            // Sample at the texel center of the tile's own lattice.
            const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(kSize);
            const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(kSize);

            // Height and central-difference slope of the wave sum.
            const auto waveAt = [&](double uu, double vv)
            {
                double height = 0.0;
                for (const auto &wave : waves)
                {
                    height += wave.amplitude
                        * std::sin(glm::two_pi<double>() * (wave.cu * uu + wave.cv * vv) + wave.phase);
                }
                return height;
            };
            const double eps = 0.5 / static_cast<double>(kSize);
            const double dhdu = (waveAt(u + eps, v) - waveAt(u - eps, v)) / (2.0 * eps);
            const double dhdv = (waveAt(u, v + eps) - waveAt(u, v - eps)) / (2.0 * eps);
            glm::dvec3 normal(-dhdu, -dhdv, 1.0);
            normal /= std::max(1.0e-9, glm::length(normal));
            const std::size_t index = (static_cast<std::size_t>(y) * kSize + x) * 4u;
            normalPixels[index + 0u] = static_cast<std::uint8_t>(std::clamp(normal.x * 0.5 + 0.5, 0.0, 1.0) * 255.0);
            normalPixels[index + 1u] = static_cast<std::uint8_t>(std::clamp(normal.y * 0.5 + 0.5, 0.0, 1.0) * 255.0);
            normalPixels[index + 2u] = static_cast<std::uint8_t>(std::clamp(normal.z * 0.5 + 0.5, 0.0, 1.0) * 255.0);
            normalPixels[index + 3u] = 255u;

            // Albedo variation: three wrapped value-noise octaves (integer
            // grids over the tile keep every octave perfectly periodic), a
            // broad mottle plus two finer ones. Directionless by design -
            // the depth tint breaks up unevenly the way wind-brightened
            // water does, instead of stacking into the parallel ripple
            // bands a sine set minifies into.
            const auto latticeHash = [](std::int64_t ix, std::int64_t iy, std::uint64_t tag)
            {
                std::uint64_t mixed = static_cast<std::uint64_t>(ix) * 0x9E3779B97F4A7C15ull
                    ^ static_cast<std::uint64_t>(iy) * 0xC2B2AE3D27D4EB4Full ^ tag;
                mixed ^= mixed >> 33;
                mixed *= 0xFF51AFD7ED558CCDull;
                mixed ^= mixed >> 33;
                mixed *= 0xC4CEB9FE1A85EC53ull;
                mixed ^= mixed >> 33;
                return static_cast<double>(mixed >> 11) * (1.0 / 9007199254740992.0);
            };
            const auto periodicNoise = [&latticeHash](double uu, double vv, double grid, std::uint64_t tag)
            {
                const double fx = uu * grid;
                const double fy = vv * grid;
                const std::int64_t gx = static_cast<std::int64_t>(grid);
                std::int64_t ix = static_cast<std::int64_t>(std::floor(fx));
                std::int64_t iy = static_cast<std::int64_t>(std::floor(fy));
                const double tx = fx - static_cast<double>(ix);
                const double ty = fy - static_cast<double>(iy);
                const double sx = tx * tx * tx * (tx * (tx * 6.0 - 15.0) + 10.0);
                const double sy = ty * ty * ty * (ty * (ty * 6.0 - 15.0) + 10.0);
                const auto wrap = [gx](std::int64_t c)
                {
                    return ((c % gx) + gx) % gx;
                };
                const std::int64_t x0 = wrap(ix);
                const std::int64_t y0 = wrap(iy);
                const std::int64_t x1 = wrap(ix + 1);
                const std::int64_t y1 = wrap(iy + 1);
                const double h00 = latticeHash(x0, y0, tag);
                const double h10 = latticeHash(x1, y0, tag);
                const double h01 = latticeHash(x0, y1, tag);
                const double h11 = latticeHash(x1, y1, tag);
                return (h00 * (1.0 - sx) + h10 * sx) * (1.0 - sy)
                    + (h01 * (1.0 - sx) + h11 * sx) * sy;
            };
            double variation = 0.0;
            variation += 0.036 * (periodicNoise(u, v, 5.0, 0xA11CE5C0DE5ull) - 0.5);
            variation += 0.024 * (periodicNoise(u, v, 13.0, 0xB0B5EDC0DE5ull) - 0.5);
            variation += 0.010 * (periodicNoise(u, v, 29.0, 0xC0FFEEC0DE5ull) - 0.5);
            const double value = 1.0 + variation;
            albedoSum += value;
            const std::uint8_t encoded = static_cast<std::uint8_t>(std::clamp(value, 0.0, 1.0) * 255.0);
            albedoPixels[index + 0u] = encoded;
            albedoPixels[index + 1u] = encoded;
            albedoPixels[index + 2u] = encoded;
            albedoPixels[index + 3u] = 255u;
        }
    }
    // Renormalize the albedo sheet's mean to exactly 1.0: the water draw's
    // vertex colors carry the depth tint, and the sheet must not shift it.
    const double mean = albedoSum / static_cast<double>(kSize * kSize);
    const double correction = std::abs(mean) > 1.0e-6 ? 1.0 / mean : 1.0;
    for (std::size_t index = 0; index + 3u < albedoPixels.size(); index += 4u)
    {
        const double value = static_cast<double>(albedoPixels[index]) * correction;
        const std::uint8_t encoded = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
        albedoPixels[index + 0u] = encoded;
        albedoPixels[index + 1u] = encoded;
        albedoPixels[index + 2u] = encoded;
    }

    renderer::RenderError error = renderer::RenderError::None;
    renderer::TextureDescription normalDescription;
    normalDescription.width = kSize;
    normalDescription.height = kSize;
    normalDescription.pixels = normalPixels.data();
    normalDescription.filter = renderer::TextureFilter::Linear;
    normalDescription.address = renderer::TextureAddress::Repeat;
    waterNormalTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &normalDescription, &error);

    error = renderer::RenderError::None;
    renderer::TextureDescription albedoDescription;
    albedoDescription.width = kSize;
    albedoDescription.height = kSize;
    albedoDescription.pixels = albedoPixels.data();
    albedoDescription.filter = renderer::TextureFilter::Linear;
    albedoDescription.address = renderer::TextureAddress::Repeat;
    waterAlbedoTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &albedoDescription, &error);

    // The near-water detail gate: ~75 tile lattices of view depth. The
    // sampler's own mip chain (sampleTrilinear with per-pixel UV
    // derivatives) folds the minified wave sheet into its mips, so the far
    // third of the gate fades gently to the mean - no sub-pixel sparkle -
    // while the near and mid water carry the ripple relief and the depth
    // tint's per-pixel breakup the coast and swimmer views read.
    waterTextureMaxDistanceMetersSquared_ = 1100.0 * 1100.0;
    if (waterNormalTexture_ == renderer::invalidTexture || waterAlbedoTexture_ == renderer::invalidTexture)
    {
        destroyWaterDetailTextures();
    }
}

void PlanetChunkRenderer::destroyWaterDetailTextures()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && waterNormalTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, waterNormalTexture_);
    }
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && waterAlbedoTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, waterAlbedoTexture_);
    }
    waterNormalTexture_ = renderer::invalidTexture;
    waterAlbedoTexture_ = renderer::invalidTexture;
    waterTextureMaxDistanceMetersSquared_ = 0.0;
}

void PlanetChunkRenderer::ensureDeckTexture(const glm::dvec3 &sunDirection, const glm::dmat3 &domeToPlanet,
    const glm::dvec3 &eyePlanetLocal)
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || deckTexture_ != renderer::invalidTexture)
    {
        return;
    }

    if (cloudField_ == nullptr)
    {
        cloudField_ = std::make_unique<planet::CloudField>(params_);
    }

    // The deck bake lives in the SKY DOME'S INITIAL FRAME - the same frame
    // the deck mesh's (segment, band) grid lives in - and its texel grid
    // maps one-to-one onto that mesh grid: columns follow the mesh's
    // azimuth stations, rows span the mesh's own latitude span (the
    // horizon guard up to the zenith). The deck mesh's UVs are that same
    // (segment, band) pair, so every per-pixel sample runs on
    // well-conditioned UVs and reads the field at its own resolution.
    // (The deck's first texture route sampled the PLANET-frame equirect
    // through per-vertex remapped UVs; wherever the dome's frame crossed
    // the planet's polar axis the UV footprint degenerated and the
    // sampler collapsed those triangles into blocky texel picks.)
    //
    // The sample POINTS run through the true chord geometry: the eye sits
    // at (0, eyeRadius, 0) in this frame, and each dome texel's direction
    // carries the ray the eye actually looks along, so the sample lands
    // where that ray pierces the cloud shell (radius + cloud altitude).
    // A radial (center-of-planet) mapping puts the texel overhead at
    // 0 degrees of arc and the texel 30 degrees up at a full 60 degrees of
    // arc - the whole sky reads the field ~60x too fast with elevation, and
    // banks that truly tower overhead (cloud altitude 1200 m, wavelength
    // 13 km: an ~80 degree object) shrink to ~12-degree far decals. The
    // chord geometry is what a real camera at the eye would see: banks
    // overhead photograph at their own towering angular size, the ones
    // toward the horizon compress with distance into the classic distant
    // bank band, and the field the deck shows stays the SAME field the
    // space shell and the terrain's cloud shadows sample - the shadow a
    // bank casts on the ground now falls under the bank the eye sees.
    // (A viewer at or above the shell has no upward chord; the bake keeps
    // the radial mapping there, the geometry the space views themselves
    // use.)
    constexpr std::uint32_t kWidth = 2048;
    constexpr std::uint32_t kHeight = 1024;
    constexpr double kDeckGuardRadians = 0.03490658503988659;  // 2 deg above the horizon
    constexpr double kDeckFadeRadians = 0.06981317007977318;   // full by 6 deg
    const bool sunValid = glm::length(sunDirection) > 1.0e-9;
    const glm::dvec3 sun = sunValid ? glm::normalize(sunDirection) : glm::dvec3(0.0, 1.0, 0.0);

    // Chord constants: the eye's radius from the planet center, the cloud
    // shell's radius, and the squared difference between them (the chord's
    // constant term). The per-texel ray solve below only needs the eye
    // radius and the ray's own up component.
    const double eyeRadius = glm::length(eyePlanetLocal);
    const double shellRadius = params_.radiusMeters + params_.cloudAltitudeMeters;
    const bool chordValid = eyeRadius < shellRadius - 1.0;
    const double chordC = shellRadius * shellRadius - eyeRadius * eyeRadius;
    // The shell point a dome ray lands on, in DOME coordinates (rotated to
    // the planet frame only for the field sample: the field is isotropic
    // under the rotation, so sampling in either frame's coordinates agrees
    // once the direction is rotated through domeToPlanet).
    const auto domeShellDirection = [&](const glm::dvec3 &domeDirection) -> glm::dvec3
    {
        if (!chordValid)
        {
            return domeDirection;
        }
        const double alongUp = eyeRadius * domeDirection.y;
        const double t = -alongUp + std::sqrt(alongUp * alongUp + chordC);
        const glm::dvec3 shellPoint = glm::dvec3(0.0, eyeRadius, 0.0) + domeDirection * t;
        return glm::normalize(shellPoint);
    };

    // Phase 1: the coverage field alone (plus the horizon dissolve), as a
    // single-channel buffer. The RGB is derived AFTER the fringe blur - see
    // the phase-3 note.
    std::vector<std::uint8_t> coveragePixels(static_cast<std::size_t>(kWidth) * kHeight, 0u);
    for (std::uint32_t y = 0; y < kHeight; ++y)
    {
        const double latitude = kDeckGuardRadians
            + (glm::pi<double>() * 0.5 - kDeckGuardRadians)
                * (static_cast<double>(y) + 0.5) / static_cast<double>(kHeight);
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        // Horizon dissolve: alpha rises from nothing at the mesh's own
        // lower rim to full ~6 degrees above it - the mesh boundary is a
        // hard polygon cut, and the place it cuts (the low sky) is exactly
        // where the atmosphere's haze band swallows detail, so a bank that
        // fades in there reads as distance while the polygon edge hides
        // inside the fade.
        const double rimFade = planet::smoothstep(kDeckGuardRadians,
            kDeckGuardRadians + kDeckFadeRadians, latitude);
        for (std::uint32_t x = 0; x < kWidth; ++x)
        {
            const double azimuth = glm::two_pi<double>()
                * (static_cast<double>(x) + 0.5) / static_cast<double>(kWidth);
            const glm::dvec3 domeDirection(std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude);
            const glm::dvec3 planetDirection = domeToPlanet * domeShellDirection(domeDirection);
            // The ground twin (no cyclone shear): the horizon band a ground
            // eye photographs spans a few dozen kilometers, and a storm's
            // Rankine displacement combs that band into converging fan arcs
            // (see coverageTexelGround's note in atmosphere.hpp).
            double coverage = planet::clamp01(cloudField_->coverageTexelGround(
                warpCloudSampleDirection(params_,
                    planet::Vec3{planetDirection.x, planetDirection.y, planetDirection.z})));
            if (coverage <= 0.003)
            {
                continue;
            }
            coverage *= rimFade;
            if (coverage <= 0.003)
            {
                continue;
            }
            coveragePixels[static_cast<std::size_t>(y) * kWidth + x]
                = static_cast<std::uint8_t>(coverage * 255.0);
        }
    }

    // Phase 2: fringe softening - three separable 3-tap passes over the
    // coverage buffer (the space bake's own recipe). The deck samples this
    // bake PER PIXEL - a 0.176 degree texel magnifies to several screen
    // pixels, so bilinear runs on the base level and no vertex-grid band
    // limit applies - the passes only dissolve the coverage threshold's
    // single-texel flecks.
    for (int blurPass = 0; blurPass < 3; ++blurPass)
    {
        std::vector<std::uint8_t> blurred(coveragePixels.size());
        const auto alphaAt = [&](std::int64_t column, std::int64_t row)
        {
            column = ((column % static_cast<std::int64_t>(kWidth)) + static_cast<std::int64_t>(kWidth))
                % static_cast<std::int64_t>(kWidth);
            row = std::clamp(row, std::int64_t{0}, static_cast<std::int64_t>(kHeight) - 1);
            return static_cast<double>(coveragePixels[static_cast<std::size_t>(row) * kWidth
                + static_cast<std::size_t>(column)]);
        };
        for (std::uint32_t y = 0; y < kHeight; ++y)
        {
            for (std::uint32_t x = 0; x < kWidth; ++x)
            {
                const double horizontal = (alphaAt(x - 1, y) + 2.0 * alphaAt(x, y) + alphaAt(x + 1, y)) * 0.25;
                const double vertical = (alphaAt(x, y - 1) + 2.0 * alphaAt(x, y) + alphaAt(x, y + 1)) * 0.25;
                blurred[static_cast<std::size_t>(y) * kWidth + x]
                    = static_cast<std::uint8_t>(std::clamp((horizontal + vertical) * 0.5, 0.0, 255.0));
            }
        }
        coveragePixels.swap(blurred);
    }

    // Phase 3: the lit RGB, derived from the BLURRED coverage for every
    // texel whose alpha survived it. Deriving RGB only where the RAW
    // coverage cleared the threshold (the previous order) left the blur's
    // fringe texels carrying alpha with a ZERO RGB - every bank edge and
    // the deck's own rim then blended a thin dark band into the sky behind
    // them (the dark wisp outlines and the dark line the first textured
    // deck photographed along its lower edge). The lighting is the
    // GROUND-SKY substitute: a high brightness floor keeps the banks white
    // even on the anti-sun side, the sun side runs brighter, and the
    // low-sun rim keeps a whisper of warm.
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(kWidth) * kHeight * 4u, 0u);
    for (std::uint32_t y = 0; y < kHeight; ++y)
    {
        const double latitude = kDeckGuardRadians
            + (glm::pi<double>() * 0.5 - kDeckGuardRadians)
                * (static_cast<double>(y) + 0.5) / static_cast<double>(kHeight);
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t x = 0; x < kWidth; ++x)
        {
            const double coverage = static_cast<double>(
                coveragePixels[static_cast<std::size_t>(y) * kWidth + x]) / 255.0;
            if (coverage <= 0.003)
            {
                continue;
            }
            const double azimuth = glm::two_pi<double>()
                * (static_cast<double>(x) + 0.5) / static_cast<double>(kWidth);
            const glm::dvec3 domeDirection(std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude);
            // The shell point's own radial carries the lighting response
            // (the sun lights the cloud AT its position, so the bank's sun
            // share and warm rim come from the point the chord landed on,
            // not the eye's raw line of sight).
            const glm::dvec3 planetDirection = domeToPlanet * domeShellDirection(domeDirection);
            const double thickness = std::pow(coverage, 0.7);
            // Bank body shading: thin wisps run BRIGHTER than the pale sky
            // behind them (sun-lit vapor against blue), and the ramp eases
            // toward the bank bodies' grayer base as coverage thickens - a
            // single bright floor for every coverage read the thin wisps as
            // gray smudges darker than the sky they crossed. The ramp runs
            // deep enough that a dense bank carries a clearly grayer core
            // than its lit fringe: a real cumulus deck reads as a volume
            // whose shaded interior occupies most of the bank (flat white
            // pancakes pasted on the sky is exactly the billboard tell);
            // the lit fringe keeps a bright crown over that gray core the
            // way a real bank's sunlit tops read above its shadowed mass.
            const double shade = 1.05 - 0.42 * thickness;
            const double sunDot = sunValid
                ? planet::clamp01(planet::dot(
                    planet::Vec3{planetDirection.x, planetDirection.y, planetDirection.z},
                    planet::Vec3{sun.x, sun.y, sun.z}))
                : 1.0;
            // Ground-sky light: a high floor with the sun's share on top -
            // the sun-side banks read distinctly brighter than the anti-sun
            // ones against the (now sun-aware) sky behind them.
            const double light = 0.80 + 0.45 * sunDot;
            // A whisper of warm rim where the sun grazes the sky's own
            // horizon band (the sunrise/sunset side of the dome).
            const double rim = 0.16 * planet::smoothstep(-0.02, 0.10, sunDot)
                * (1.0 - planet::smoothstep(0.10, 0.35, sunDot));
            const std::size_t index = (static_cast<std::size_t>(y) * kWidth + x) * 4u;
            for (int channel = 0; channel < 3; ++channel)
            {
                const double warm = channel == 0 ? 1.0 : channel == 1 ? 0.82 : 0.62;
                const double value = params_.cloudColor[channel] * shade
                    * (light + rim * warm) * (channel == 0 ? 1.0 : channel == 1 ? 0.99 : 0.97);
                pixels[index + static_cast<std::size_t>(channel)]
                    = static_cast<std::uint8_t>(std::clamp(value, 0.0, 1.0) * 255.0);
            }
            pixels[index + 3u] = static_cast<std::uint8_t>(coverage * 255.0);
        }
    }

    renderer::TextureDescription description;
    description.width = kWidth;
    description.height = kHeight;
    description.pixels = pixels.data();
    description.filter = renderer::TextureFilter::Linear;
    description.address = renderer::TextureAddress::Repeat;
    renderer::RenderError error = renderer::RenderError::None;
    deckTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &description, &error);
    // The pixel copy stays owned here (like the space bake's own copy):
    // the sky deck's quad gate bilinear this array per rebuild - the
    // texture handle alone cannot answer those reads.
    deckTexturePixels_ = std::move(pixels);
    deckTextureWidth_ = kWidth;
    deckTextureHeight_ = kHeight;
}

void PlanetChunkRenderer::ensureCloudLayer(const glm::dvec3 &sunDirection)
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || cloudLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // The bake first (both the shell and the sky deck draw through it, and
    // the mesh gates below read its pixels).
    ensureCloudTexture(sunDirection);
    if (cloudTexture_ == renderer::invalidTexture || cloudTexturePixels_.empty())
    {
        return;
    }

    // Per-pixel cloud shell: a lat-long UV sphere over the cloud altitude.
    // The 192x96 grid is dense enough that the geometry itself never
    // limits the cloud structure - every bank edge, wisp fringe and cyclone
    // arm comes from the texture's alpha, sampled per pixel through the
    // transparent route (the bake's own fringe blur dissolves the hard
    // one-texel flake edges, so the quad grid's straight cuts hide inside
    // the softened fringes). Quads whose four corners all read clear in the
    // bake are skipped outright, so the mesh only covers the cloudy
    // provinces and the rasterizer never walks the clear two-thirds of the
    // shell - the transparent per-sample path is the frame's most expensive
    // shading, and the gate is what keeps it bounded.
    constexpr std::uint32_t kSegments = 192;
    constexpr std::uint32_t kBands = 96;
    // The quad gate rides well above the blurred fringe's own tail: a quad
    // whose corners sit at sub-visible alpha renders as a pale hard-edged
    // rectangle (the geometry cut reads no matter what the texel alpha
    // says), so the gate keeps only quads whose whole extent carries solid
    // coverage - the fringe below it belongs to the texture alone.
    constexpr double kCornerGate = 0.06;
    const double shellRadius = params_.radiusMeters + params_.cloudAltitudeMeters;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(static_cast<std::size_t>(kSegments + 1u) * (kBands + 1u));
    indices.reserve(static_cast<std::size_t>(kSegments) * kBands * 6u);
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = -glm::pi<double>() * 0.5
            + glm::pi<double>() * (static_cast<double>(band) / static_cast<double>(kBands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment)
                / static_cast<double>(kSegments);
            const planet::Vec3 direction{std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude};
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x * shellRadius);
            vertex.position[1] = static_cast<float>(direction.y * shellRadius);
            vertex.position[2] = static_cast<float>(direction.z * shellRadius);
            vertex.normal[0] = static_cast<float>(direction.x);
            vertex.normal[1] = static_cast<float>(direction.y);
            vertex.normal[2] = static_cast<float>(direction.z);
            // The texture's RGB carries the full sun bake (see
            // ensureCloudTexture); the vertex colors stay neutral and the
            // per-frame tint rides the material.
            vertex.color[0] = 1.0f;
            vertex.color[1] = 1.0f;
            vertex.color[2] = 1.0f;
            const double u = static_cast<double>(segment) / static_cast<double>(kSegments);
            const double v = (glm::pi<double>() * 0.5 - latitude) / glm::pi<double>();
            vertex.uv[0] = static_cast<float>(u);
            vertex.uv[1] = static_cast<float>(v);
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            const auto cornerUv = [&](std::uint32_t row, std::uint32_t column)
            {
                const double u = static_cast<double>(column) / static_cast<double>(kSegments);
                const double latitude = -glm::pi<double>() * 0.5
                    + glm::pi<double>() * (static_cast<double>(row) / static_cast<double>(kBands));
                return std::pair<double, double>{u, (glm::pi<double>() * 0.5 - latitude) / glm::pi<double>()};
            };
            const auto cornerAlpha = [&](std::uint32_t row, std::uint32_t column)
            {
                const auto [u, v] = cornerUv(row, column);
                return cloudBakeAlpha(cloudTexturePixels_, cloudTextureWidth_, cloudTextureHeight_, u, v);
            };
            if (std::max(std::max(cornerAlpha(band, segment), cornerAlpha(band, segment + 1u)),
                    std::max(cornerAlpha(band + 1u, segment), cornerAlpha(band + 1u, segment + 1u)))
                < kCornerGate)
            {
                continue; // clear sky over this whole block: nothing to draw
            }
            const std::uint32_t a = band * (kSegments + 1u) + segment;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (kSegments + 1u);
            const std::uint32_t d = c + 1u;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
    if (indices.empty())
    {
        return; // the configured coverage keeps the whole shell clear
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
    // Unlit translucent deck: the RGB bake carries the whole lit response
    // (thickness, sun gradient, silver lining), the alpha channel the
    // per-texel coverage. Opacity just under 1 routes the draw into the
    // renderer's transparent pass - alpha-blended per texel over the opaque
    // scene (terrain, water, the limb glow) with depth writes
    // off, depth-tested so the planet hides the far side of the shell.
    material.unlit = true;
    material.opacity = 0.999f;
    material.baseColorTexture = cloudTexture_;
    // Outward shell seen from outside at every view that draws it (globe and
    // orbit keep the eye above the shell): the declared winding culls the
    // far hemisphere per triangle where culling applies at all.
    material.frontFaceWinding = macroShellFrontFaceWinding;
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

    // Rim annulus around the planet, latitudes 0 (its equator rim) to 40
    // degrees. The pole points at the eye each frame, so mesh latitude 0 is
    // the silhouette edge and latitude 40 sits 23% of the disk radius inside
    // it - the annulus covers the ring the eye can actually see (the glow's
    // own fade starts at 50 degrees from the pole, and the opaque terrain
    // silhouette eats everything inside roughly 70 degrees) while skipping
    // the disk interior the terrain draws over anyway: the old full
    // hemisphere shaded the whole disk's worth of pixels per frame only for
    // the opaque draws to overwrite them. Colors key on the angle from the
    // pole: bright horizon color over the annulus interior (the opaque
    // terrain/cloud draws overwrite it), fading to black at the rim - the
    // ring between the terrain silhouette and the shell's own silhouette
    // then reads as glowing air instead of a hard terrain edge on black.
    //
    // The gradient also runs around the AZIMUTH: the shell's local +X points
    // at the sun each frame (see updateAtmosphereShell), so azimuth 0 is the
    // sunward limb and azimuth 180 the night limb. The day side glows, the
    // night side keeps a faint blue airglow - a lit ring on one side of the
    // planet and a dark one on the other reads as a real sunlit atmosphere
    // (the uniform ring read as a painted halo).
    constexpr std::uint32_t kSegments = 128;
    constexpr std::uint32_t kBands = 24;
    // Latitude span of the annulus (the fade band below starts at 50
    // degrees from the pole = 40 degrees of latitude).
    constexpr double kAnnulusLatitudeRadians = 0.6981317007977318; // 40 deg
    // Fade band in the angle from the pole. The shell is scaled per frame to
    // a visual radius a few percent above the surface (see
    // updateAtmosphereShell - the configured air column can span a large
    // fraction of a showcase radius, but the VISIBLE glow hugs the ground
    // the way density does); from there, perspective compresses the ring
    // between the terrain silhouette and the shell rim into roughly 35-90
    // degrees from orbit and 56-90 from the globe view. The glow therefore
    // stays dense past half the ring and thins out only over the last
    // stretch before the rim - an early fade leaves a dark ring between the
    // terrain silhouette and the air glow, which reads as a gap instead of
    // atmosphere. The fade end sits exactly at the rim (90 degrees) so the
    // outer boundary reaches zero brightness at the mesh edge instead of
    // stopping a few degrees short with a visible step.
    constexpr double kLimbFadeStartRadians = 0.8726646259971648; // 50 deg
    constexpr double kLimbFadeEndRadians = 1.5707963267948966;  // 90 deg (the annulus rim)
    // Fade exponent: 1.6 concentrates the glow against the terrain
    // silhouette the way the limb's real exponential air column does - a
    // dense bright inner line falling off fast, instead of the flat wide
    // band a lower exponent paints (a fat uniform ring reads as a glass
    // marble coating; the thin bright one reads as air).
    constexpr double kLimbFadeExponent = 1.6;
    // Day/night azimuth falloff: full glow toward the sun, a whisper of
    // faint airglow residue on the night side. The floor stays just above
    // zero so the night limb carries a breakable-thin ring (from space a
    // real atmosphere does hold a faint limb all around - airglow plus
    // scattered starlight) while the night side of the disc itself falls to
    // black; a fat floor here photographs as a frosted rim that flattens the
    // terminator's day/night contrast the way a painted halo does.
    constexpr double kLimbNightFloor = 0.05;
    // Sunset warmth: the limb ring glows warm where the terminator crosses
    // it (the sun grazes the atmosphere there, so the light path reddens the
    // way sunrise/sunset light does). The warm band rides the azimuthal
    // gradient - azimuth 0 is the sunward limb (see updateAtmosphereShell),
    // so the warmth peaks where cos(azimuth) crosses zero and melts back to
    // the plain horizon hue well inside the day side. The strength is the
    // majority share of the horizon hue through the band's core: orbital
    // photography shows a striking orange-red arc where the terminator
    // meets the limb, while a subtle-only tint reads as a faint salmon
    // smear; the band stays NARROW so the day side keeps its blue.
    const planet::Vec3 warmTint{1.0, 0.42, 0.18};
    constexpr double kWarmStrength = 0.45;
    constexpr double kWarmBandWidth = 0.20;
    // Forward-scattering crescent: the air glows brightest around the sun's
    // limb - aerosols and air molecules scatter strongly forward, so the
    // ring carries a bright arc on the sunward side that eases off around
    // the disc instead of a uniform band (the azimuthal day gradient alone
    // saturates flat at 1 through the whole sun side). The gain rides on top
    // of the day factor as a halving-weighted arc - half again as bright at
    // the sunward limb, easing through the sun side the way a crescent
    // photographs - and the HDR pipeline (exposure + tone mapping)
    // compresses the overshoot the way film does a bright rim.
    constexpr double kForwardScatterGain = 0.95;
    // Radial Rayleigh hue: the dense air at the ring's inner edge reads the
    // horizon color, while the thinning air toward the rim scatters a paler,
    // washed-out blue (the deepest optical paths redden into the warm band
    // above; the shallow ones desaturate toward white). A single flat hue
    // through the annulus reads as a painted ring; the two-tone ramp reads
    // as a stratified air column.
    constexpr double kRimPaleFraction = 0.48;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
    indices.reserve(kSegments * kBands * 6u);
    // The dense air at the silhouette reads the MID-sky blue (the deeper
    // scattering color a sight line through the thickest air returns), not
    // the pale horizon band - an inner edge keyed to the horizon color went
    // near-white with the pale horizon palette and read as a thin white
    // ring instead of glowing air. The base hue keeps a measured white
    // fraction: the full mid-sky saturation minified into a few-pixel ring
    // photographs as a hard painted band, while the slightly lifted tone
    // reads as dense air over the terrain edge.
    const planet::Vec3 midSky = planet::skyGradientColor(params_, 0.30);
    const planet::Vec3 limbColor{planet::mix(midSky.x, 1.0, 0.16),
        planet::mix(midSky.y, 1.0, 0.16), planet::mix(midSky.z, 1.0, 0.16)};
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = kAnnulusLatitudeRadians * (static_cast<double>(band) / static_cast<double>(kBands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        const double angleFromPole = glm::pi<double>() * 0.5 - latitude;
        // Exponential-ish falloff: dense air right at the terrain silhouette,
        // thinning rapidly toward the top of the glow (a smoothstep alone
        // reads as a flat band with a hard edge). The exponent and the fade
        // end above both shape the outer approach; see their notes.
        const double fade = std::pow(1.0 - planet::smoothstep(kLimbFadeStartRadians, kLimbFadeEndRadians, angleFromPole), kLimbFadeExponent);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const glm::dvec3 direction(std::cos(azimuth) * cosLatitude, sinLatitude, std::sin(azimuth) * cosLatitude);
            const double day = planet::mix(kLimbNightFloor, 1.0,
                planet::smoothstep(-0.85, 0.45, std::cos(azimuth)));
            // Forward-scattering arc (see kForwardScatterGain): brightest at
            // the sunward limb (azimuth 0), easing around the day side.
            const double forward = 1.0
                + kForwardScatterGain * std::pow(planet::clamp01(std::cos(azimuth)), 2.0);
            const double terminatorBand = 1.0 - planet::smoothstep(0.0, kWarmBandWidth, std::abs(std::cos(azimuth)));
            // Radial Rayleigh ramp (see kRimPaleFraction): fade is the
            // annulus's own air-column weight (1 at the terrain silhouette,
            // 0 at the rim), so 1 - fade measures how far out through the
            // thinning air the vertex sits.
            const planet::Vec3 rimTint{
                planet::mix(limbColor.x, planet::mix(limbColor.x, 1.0, 0.55), kRimPaleFraction * (1.0 - fade)),
                planet::mix(limbColor.y, planet::mix(limbColor.y, 1.0, 0.55), kRimPaleFraction * (1.0 - fade)),
                planet::mix(limbColor.z, planet::mix(limbColor.z, 1.0, 0.55), kRimPaleFraction * (1.0 - fade))};
            const planet::Vec3 limbTint{
                planet::mix(rimTint.x, warmTint.x, kWarmStrength * terminatorBand),
                planet::mix(rimTint.y, warmTint.y, kWarmStrength * terminatorBand),
                planet::mix(rimTint.z, warmTint.z, kWarmStrength * terminatorBand)};
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            // The limb's own brightness lift: the visible ring is a thin
            // sliver of the annulus, and a line-brightness glow there is what
            // orbital photography actually shows.
            constexpr double kLimbBrightness = 1.5;
            vertex.color[0] = static_cast<float>(limbTint.x * fade * day * forward * kLimbBrightness);
            vertex.color[1] = static_cast<float>(limbTint.y * fade * day * forward * kLimbBrightness);
            vertex.color[2] = static_cast<float>(limbTint.z * fade * day * forward * kLimbBrightness);
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
            // Outward winding for the +1 convention: (a, c, b) / (b, c, d)
            // (rows climb with latitude, columns advance with azimuth).
            indices.insert(indices.end(), {a, c, b, b, c, d});
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
    // Outward annulus seen from outside (space views): the declared winding
    // culls the far half of the ring per triangle (its pixels only ever
    // failed the depth test after scanning their bounding boxes).
    material.frontFaceWinding = macroShellFrontFaceWinding;
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
    // the limb glow's visual radius: three quarters of the configured air
    // column above the surface (the Kármán-line shell's dense-glow share -
    // real orbital photographs read the bright limb band at roughly 1% of
    // Earth's radius, a thin bright line, not a marble coating), clamped to
    // 3.2-10% of the radius so small-planet showcases with proportionally
    // fat air columns keep a readable ring. Translate to the planet's world
    // center.
    // The shell's local +X lands on the SUNWARD tangent: the built-in
    // azimuthal day gradient (see ensureAtmosphereShell) then glows on the
    // sun side of the ring and fades on the night side. When the sun is
    // disabled or sits along the eye direction, any tangent works (the
    // tint below takes over).
    const glm::dvec3 eyeDirection = glm::normalize(eyePlanetLocal);
    glm::dvec3 tangent = glm::length(sunDirection) > 1.0e-9 && sunEnabled
        ? glm::normalize(sunDirection) - eyeDirection * glm::dot(glm::normalize(sunDirection), eyeDirection)
        : glm::dvec3(0.0);
    if (glm::length(tangent) < 1.0e-6)
    {
        tangent = glm::cross(eyeDirection, glm::dvec3(0.0, 0.0, 1.0));
        if (glm::length(tangent) < 1.0e-6)
        {
            tangent = glm::cross(eyeDirection, glm::dvec3(1.0, 0.0, 0.0));
        }
    }
    tangent = glm::normalize(tangent);
    const glm::dvec3 third = glm::cross(tangent, eyeDirection);
    const glm::dmat3 rotation(tangent, eyeDirection, third);
    const double glowHeight = glm::clamp(0.75 * std::max(1.0, params_.atmosphereHeightMeters),
        0.032 * params_.radiusMeters, 0.10 * params_.radiusMeters);
    const double shellRadius = params_.radiusMeters + glowHeight;
    atmosphereShellTransform_ = glm::translate(glm::mat4(1.0f),
            glm::vec3(static_cast<float>(params_.centerMeters[0]), static_cast<float>(params_.centerMeters[1]),
                static_cast<float>(params_.centerMeters[2])))
        * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(shellRadius)));

    // The limb glows where the sun hits the visible side: tint the shell
    // with the same sun sky tint the dome uses (the night limb fades out).
    // The azimuthal gradient baked into the mesh carries the day/night
    // asymmetry around the ring; this tint tracks the sun's elevation at
    // the sub-eye point.
    const double sunUp = sunEnabled && glm::length(sunDirection) > 1.0e-9
        ? glm::dot(glm::normalize(sunDirection), eyeDirection)
        : 1.0;
    const planet::Vec3 tint = planet::sunSkyTint(params_, sunUp, sunEnabled);
    renderer::MeshMaterial &material = atmosphereShellMaterials_.front();
    material.baseColor[0] = static_cast<float>(tint.x);
    material.baseColor[1] = static_cast<float>(tint.y);
    material.baseColor[2] = static_cast<float>(tint.z);
}

void PlanetChunkRenderer::ensureStarDome()
{
    if (!params_.starsEnabled || renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || starDome_ != renderer::invalidMesh)
    {
        return;
    }

    // One small disc per star (see forEachStar): stars as GEOMETRY instead
    // of a field sampled on a fixed dome lattice. A sampled dome either
    // wastes a huge mesh on empty sky or turns every star into a bilinear
    // smear across a couple of quads (quad-shaped blobs); the per-star discs
    // stay round at any resolution and cost one tiny fan each. Disc radius
    // scales with the square root of the star's brightness (bright stars
    // read as slightly larger points, the way bright stars do through an
    // exposure); the whole field stays a few thousand discs. Ten segments
    // keep even the smallest discs round at capture resolutions (six-sided
    // fans read as hexagons once a disc covers more than a couple of
    // pixels).
    struct StarDiscContext
    {
        std::vector<planet::PlanetVertex> *vertices;
        std::vector<std::uint32_t> *indices;
        // Disc size range from the module configuration
        // (star_disc_radius_min/max_radians); StarVisitor is a plain function
        // pointer, so the lambda reads them through the context.
        double discRadiusMinRadians = 0.0011;
        double discRadiusMaxRadians = 0.0040;
        std::size_t count = 0;
    };
    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    StarDiscContext context{&vertices, &indices, params_.starDiscRadiusMinRadians, params_.starDiscRadiusMaxRadians};
    constexpr std::uint32_t kDiscSegments = 10;
    const auto visitor = [](const planet::Vec3 &direction, const planet::Vec3 &color, double brightness, void *raw)
    {
        auto *ctx       = static_cast<StarDiscContext *>(raw);
        const double radius = ctx->discRadiusMinRadians
            + (ctx->discRadiusMaxRadians - ctx->discRadiusMinRadians) * planet::clamp01(std::sqrt(brightness));
        // Tangent basis at the star direction (any perpendicular pair).
        planet::Vec3 tangent = planet::cross(direction, std::abs(direction.y) < 0.9
                ? planet::Vec3{0.0, 1.0, 0.0}
                : planet::Vec3{1.0, 0.0, 0.0});
        tangent             = planet::normalized(tangent);
        const planet::Vec3 bitangent = planet::cross(direction, tangent);
        const std::uint32_t centerSlot = static_cast<std::uint32_t>(ctx->vertices->size());
        {
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            vertex.normal[0]   = static_cast<float>(direction.x);
            vertex.normal[1]   = static_cast<float>(direction.y);
            vertex.normal[2]   = static_cast<float>(direction.z);
            vertex.color[0]    = color.x;
            vertex.color[1]    = color.y;
            vertex.color[2]    = color.z;
            ctx->vertices->push_back(vertex);
        }
        std::uint32_t firstRim = 0;
        for (std::uint32_t segment = 0; segment < kDiscSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kDiscSegments);
            const planet::Vec3 offset = tangent * (std::cos(azimuth) * radius) + bitangent * (std::sin(azimuth) * radius);
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x + offset.x);
            vertex.position[1] = static_cast<float>(direction.y + offset.y);
            vertex.position[2] = static_cast<float>(direction.z + offset.z);
            vertex.normal[0]   = static_cast<float>(direction.x);
            vertex.normal[1]   = static_cast<float>(direction.y);
            vertex.normal[2]   = static_cast<float>(direction.z);
            vertex.color[0]    = color.x;
            vertex.color[1]    = color.y;
            vertex.color[2]    = color.z;
            ctx->vertices->push_back(vertex);
            if (segment == 0)
            {
                firstRim = centerSlot + 1u;
            }
        }
        for (std::uint32_t segment = 0; segment < kDiscSegments; ++segment)
        {
            const std::uint32_t current = firstRim + segment;
            const std::uint32_t next    = firstRim + (segment + 1u) % kDiscSegments;
            ctx->indices->insert(ctx->indices->end(), {centerSlot, current, next});
        }
        ++ctx->count;
        return true;
    };
    planet::forEachStar(params_, visitor, &context);
    if (indices.empty())
    {
        return; // stars disabled by density/brightness: no dome
    }

    // Milky Way glow: the module's galaxy band weight paints the band's
    // unresolved-star glow as dim quads between the band's own stars, so the
    // strip reads as a continuous band (the star lattice alone is far too
    // sparse at capture resolutions - a ~4 deg lattice leaves the visible
    // sky a handful of band stars). One quad per ~1.4 deg cell whose corner
    // weights clear a floor; each corner samples the weight at its own
    // direction, so the strip's edges and the dust rift fade smoothly
    // instead of stepping at cell boundaries.
    if (params_.galaxyBandStrength > 0.0)
    {
        constexpr std::uint32_t kGlowAzimuthSteps  = 256;
        constexpr std::uint32_t kGlowLatitudeSteps = 128;
        // Linear RGB of the glow at full band weight: a faint warm wash.
        // The values sit far under the presentation bloom's bright-pass
        // threshold and under the tone curve's contrast band, so the band
        // reads as the Milky Way's diffuse river behind the star discs but
        // never blooms and never photographs its quad grid - a glow quad
        // driven hot (linear values past 1) displays near-white through the
        // tone curve and the bloom smears the whole band into a blown-out
        // wall across half the space field, while even mid tones expose the
        // 1.4-deg quad staircase at the band's soft edges.
        constexpr double kGlowTone[3] = {0.12, 0.11, 0.09};
        constexpr double kGlowFloor    = 0.16;
        // Corner weights first (shared by adjacent quads; the seam row/column
        // repeats the first to keep the grid watertight).
        std::vector<double> weightGrid(static_cast<std::size_t>(kGlowLatitudeSteps + 1u)
            * (kGlowAzimuthSteps + 1u));
        for (std::uint32_t latitudeStep = 0; latitudeStep <= kGlowLatitudeSteps; ++latitudeStep)
        {
            const double latitude = -glm::pi<double>() * 0.5
                + glm::pi<double>() * static_cast<double>(latitudeStep) / static_cast<double>(kGlowLatitudeSteps);
            const double cosLatitude = std::cos(latitude);
            const double sinLatitude = std::sin(latitude);
            for (std::uint32_t azimuthStep = 0; azimuthStep <= kGlowAzimuthSteps; ++azimuthStep)
            {
                const double azimuth = glm::two_pi<double>() * static_cast<double>(azimuthStep)
                    / static_cast<double>(kGlowAzimuthSteps);
                const planet::Vec3 direction{std::cos(azimuth) * cosLatitude, sinLatitude,
                    std::sin(azimuth) * cosLatitude};
                weightGrid[static_cast<std::size_t>(latitudeStep) * (kGlowAzimuthSteps + 1u) + azimuthStep]
                    = planet::galaxyBandWeight(params_, direction);
            }
        }
        for (std::uint32_t latitudeStep = 0; latitudeStep < kGlowLatitudeSteps; ++latitudeStep)
        {
            const double latitudeLow = -glm::pi<double>() * 0.5
                + glm::pi<double>() * static_cast<double>(latitudeStep) / static_cast<double>(kGlowLatitudeSteps);
            const double latitudeHigh = -glm::pi<double>() * 0.5
                + glm::pi<double>() * static_cast<double>(latitudeStep + 1u) / static_cast<double>(kGlowLatitudeSteps);
            for (std::uint32_t azimuthStep = 0; azimuthStep < kGlowAzimuthSteps; ++azimuthStep)
            {
                const std::size_t rowLow  = static_cast<std::size_t>(latitudeStep) * (kGlowAzimuthSteps + 1u);
                const std::size_t rowHigh = rowLow + (kGlowAzimuthSteps + 1u);
                const std::size_t corner00 = rowLow + azimuthStep;
                const std::size_t corner01 = corner00 + 1u;
                const std::size_t corner10 = rowHigh + azimuthStep;
                const std::size_t corner11 = corner10 + 1u;
                const double maxWeight = std::max(std::max(weightGrid[corner00], weightGrid[corner01]),
                    std::max(weightGrid[corner10], weightGrid[corner11]));
                if (maxWeight <= kGlowFloor)
                {
                    continue;
                }
                const double azimuthLow = glm::two_pi<double>() * static_cast<double>(azimuthStep)
                    / static_cast<double>(kGlowAzimuthSteps);
                const double azimuthHigh = glm::two_pi<double>() * static_cast<double>(azimuthStep + 1u)
                    / static_cast<double>(kGlowAzimuthSteps);
                const double cornerWeights[4] = {weightGrid[corner00], weightGrid[corner01],
                    weightGrid[corner11], weightGrid[corner10]};
                const double cornerAngles[4][2] = {{azimuthLow, latitudeLow}, {azimuthHigh, latitudeLow},
                    {azimuthHigh, latitudeHigh}, {azimuthLow, latitudeHigh}};
                const std::uint32_t first = static_cast<std::uint32_t>(vertices.size());
                for (std::size_t corner = 0; corner < 4u; ++corner)
                {
                    const double cosLatitude = std::cos(cornerAngles[corner][1]);
                    const glm::dvec3 direction(std::cos(cornerAngles[corner][0]) * cosLatitude,
                        std::sin(cornerAngles[corner][1]), std::sin(cornerAngles[corner][0]) * cosLatitude);
                    const double weight = cornerWeights[corner];
                    planet::PlanetVertex vertex{};
                    vertex.position[0] = static_cast<float>(direction.x);
                    vertex.position[1] = static_cast<float>(direction.y);
                    vertex.position[2] = static_cast<float>(direction.z);
                    vertex.normal[0]   = static_cast<float>(direction.x);
                    vertex.normal[1]   = static_cast<float>(direction.y);
                    vertex.normal[2]   = static_cast<float>(direction.z);
                    vertex.color[0]    = static_cast<float>(kGlowTone[0] * weight);
                    vertex.color[1]    = static_cast<float>(kGlowTone[1] * weight);
                    vertex.color[2]    = static_cast<float>(kGlowTone[2] * weight);
                    vertices.push_back(vertex);
                }
                indices.insert(indices.end(), {first, first + 1u, first + 2u, first, first + 2u, first + 3u});
            }
        }
    }

    starDome_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (starDome_ == renderer::invalidMesh)
    {
        return;
    }
    renderer::MeshMaterial &material = starDomeMaterials_.emplace_back();
    material.unlit = true;
    // Backdrop: drawn before everything else, depth writes off (the opaque
    // terrain, cloud shell and limb overwrite it where they cover it).
    material.depthWrite = false;
}

void PlanetChunkRenderer::updateStarDome(const glm::dvec3 &eyePlanetLocal, double farZ)
{
    starDomeActive_ = starDome_ != renderer::invalidMesh && !starDomeMaterials_.empty();
    if (!starDomeActive_)
    {
        return;
    }
    // The stars live behind the atmosphere: once the eye is inside the air,
    // the sky dome owns every sky pixel (and would draw over the stars
    // anyway - hiding the dome keeps the draw cost off ground frames).
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    if (planet::atmosphereWeight(params_, eyeAltitude) > 0.0)
    {
        starDomeActive_ = false;
        return;
    }

    // A sphere around the eye, large enough to enclose the whole planet
    // (stars sit behind it from every space view) and well inside the far
    // plane. Half the far plane is the largest safe radius for the deepest
    // macro view (eye at 3.1 radii on the showcase).
    const double domeRadius = 0.5 * std::max(1.0, farZ);
    const glm::vec3 worldEye(static_cast<float>(params_.centerMeters[0] + eyePlanetLocal.x),
        static_cast<float>(params_.centerMeters[1] + eyePlanetLocal.y),
        static_cast<float>(params_.centerMeters[2] + eyePlanetLocal.z));
    starDomeTransform_ = glm::translate(glm::mat4(1.0f), worldEye)
        * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(domeRadius)));
}

void PlanetChunkRenderer::ensureSunDisc()
{
    if (renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer || sunDisc_ != renderer::invalidMesh)
    {
        return;
    }

    // Radial disc with a soft halo (see buildSunDiscMesh). In air the halo
    // spans ~4.3 degrees of aerial glare around a ~2.2 degree core: the
    // presentation bloom's bright pass averages 4x4 pixel blocks, so a
    // sun-sized core dilutes below its own threshold in the cell average
    // and never blooms - the disc carries enough saturated-white area to
    // drive the bloom halo itself, the way a photographed low sun bleeds
    // wide. In space it tightens to ~0.6 degrees (the sun is a hard point
    // with a tight glare against black - a wide halo reads as a blob).
    constexpr std::uint32_t kSegments = 48;
    constexpr std::uint32_t kBands = 4;
    sunDiscAngularRadiusRadians_ = 0.075; // in-air halo; space views shrink it below

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    buildSunDiscMesh(vertices, indices, kSegments, kBands);

    sunDisc_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (sunDisc_ == renderer::invalidMesh)
    {
        return;
    }
    renderer::MeshMaterial &material = sunDiscMaterials_.emplace_back();
    material.unlit = true;
    material.depthWrite = false; // backdrop layer; clouds and terrain overwrite it
    // Radial halo texture: the alpha channel carries the core-to-rim falloff
    // (the profile the vertex colors used to bake), the RGB stays white for
    // the per-frame baseColor tint. The draw rides the transparent route
    // (opacity just under 1): every pixel then blends at opacity x texel
    // alpha, so the disc FADES into the sky dome behind it instead of
    // painting a dark rim ring where the vertex colors used to reach zero.
    constexpr std::uint32_t kHaloSize = 128;
    std::vector<std::uint8_t> haloPixels(static_cast<std::size_t>(kHaloSize) * kHaloSize * 4u, 255u);
    for (std::uint32_t y = 0; y < kHaloSize; ++y)
    {
        for (std::uint32_t x = 0; x < kHaloSize; ++x)
        {
            const double dx = (static_cast<double>(x) + 0.5) / kHaloSize * 2.0 - 1.0;
            const double dy = (static_cast<double>(y) + 0.5) / kHaloSize * 2.0 - 1.0;
            const double r = std::min(1.0, std::sqrt(dx * dx + dy * dy));
            // Core-and-glow profile: full brightness through the inner ~30%
            // (the sun's own disc - enough saturated area to carry the
            // bloom), then a steep smooth fall to zero at the rim.
            const double falloff = r < 0.30 ? 1.0 : std::pow(1.0 - r, 3.0);
            const std::size_t index = (static_cast<std::size_t>(y) * kHaloSize + x) * 4u + 3u;
            haloPixels[index] = static_cast<std::uint8_t>(std::clamp(falloff, 0.0, 1.0) * 255.0);
        }
    }
    renderer::TextureDescription haloDescription;
    haloDescription.width = kHaloSize;
    haloDescription.height = kHaloSize;
    haloDescription.pixels = haloPixels.data();
    haloDescription.filter = renderer::TextureFilter::Linear;
    haloDescription.address = renderer::TextureAddress::ClampEdge;
    renderer::RenderError haloError = renderer::RenderError::None;
    sunHaloTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &haloDescription, &haloError);
    if (sunHaloTexture_ != renderer::invalidTexture)
    {
        material.baseColorTexture = sunHaloTexture_;
        material.opacity = 0.999f;
    }
}

void PlanetChunkRenderer::updateSunDisc(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up,
    const glm::dvec3 &sunDirection, const float sunColor[3], bool sunEnabled, double farZ)
{
    sunDiscActive_ = sunDisc_ != renderer::invalidMesh && !sunDiscMaterials_.empty();
    if (!sunDiscActive_)
    {
        return;
    }
    if (!sunEnabled || glm::length(sunDirection) < 1.0e-9)
    {
        sunDiscActive_ = false;
        return;
    }

    // The sun is a backdrop at infinity: a disc placed along the sun
    // direction at the backdrop distance (just inside the sky dome's radius,
    // or half the far plane for space views where no dome is active),
    // oriented facing the eye.
    const glm::dvec3 sun = glm::normalize(sunDirection);
    const glm::dvec3 upNormalized = glm::length(up) > 1.0e-9 ? glm::normalize(up) : glm::dvec3(0.0, 1.0, 0.0);
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    const bool inAir = planet::atmosphereWeight(params_, eyeAltitude) > 0.0;
    const double reliefTop = std::max(0.0, params_.derivedMaxReliefMeters);
    const double sightCeiling = std::max(1.0, eyeAltitude + reliefTop);
    const double horizonDistance = std::sqrt(std::max(0.0, sightCeiling * (2.0 * params_.radiusMeters + sightCeiling)));
    const double backdropRadius = inAir
        ? glm::clamp(1.25 * horizonDistance, 1.0e3, 0.45 * std::max(1.0, farZ))
        : 0.45 * std::max(1.0, farZ);

    const glm::vec3 worldEye(static_cast<float>(params_.centerMeters[0] + eyePlanetLocal.x),
        static_cast<float>(params_.centerMeters[1] + eyePlanetLocal.y),
        static_cast<float>(params_.centerMeters[2] + eyePlanetLocal.z));
    const glm::vec3 worldSunDirection(static_cast<float>(sun.x), static_cast<float>(sun.y), static_cast<float>(sun.z));
    const glm::vec3 place = worldEye + worldSunDirection * static_cast<float>(backdropRadius);
    // Orient the disc's +Z (its face) back at the eye: a basis whose third
    // axis is the negated sun direction.
    glm::vec3 right = glm::cross(worldSunDirection, glm::vec3(0.0f, 1.0f, 0.0f));
    if (glm::length(right) < 1.0e-5f)
    {
        right = glm::cross(worldSunDirection, glm::vec3(1.0f, 0.0f, 0.0f));
    }
    right = glm::normalize(right);
    const glm::vec3 down = glm::cross(right, worldSunDirection);
    const glm::mat3 rotation(glm::mat3(right, down, -worldSunDirection));
    const double haloRadiusRadians = inAir ? sunDiscAngularRadiusRadians_ : 0.011;
    const float discRadius = static_cast<float>(backdropRadius * std::tan(haloRadiusRadians));
    sunDiscTransform_ = glm::translate(glm::mat4(1.0f), place) * glm::mat4(rotation)
        * glm::scale(glm::mat4(1.0f), glm::vec3(discRadius));

    // Tint: the sun's own color lifted well above 1 so the core saturates to
    // white through the shared tone mapping, times the atmosphere's DIRECT
    // transmittance tint (see planet::sunDirectTint) - the disc is the sun
    // itself, so its color follows transmission (white -> warm -> deep red
    // toward the horizon) rather than the scattered-light sky tint. The
    // eye's atmosphere weight keeps a space view's disc unattenuated; the
    // in-air halo spreads the same energy.
    const double sunUp = glm::dot(sun, upNormalized);
    const double discAirWeight = planet::atmosphereWeight(params_, eyeAltitude);
    const planet::Vec3 directTint = planet::sunDirectTint(params_, sunUp, sunEnabled);
    const double discTint[3] = {
        1.0 + (directTint.x - 1.0) * discAirWeight,
        1.0 + (directTint.y - 1.0) * discAirWeight,
        1.0 + (directTint.z - 1.0) * discAirWeight};
    renderer::MeshMaterial &material = sunDiscMaterials_.front();
    // The in-air disc runs at the space view's brightness: the halo region
    // the sun-aware sky now paints behind the disc is already near-white,
    // and a dimmer core blends into that glare instead of reading as the
    // sun itself. Eight suns saturate through the tone curve to a hard
    // white core in both regimes.
    const float discBrightness = 8.0f;
    material.baseColor[0] = static_cast<float>(sunColor[0] * discTint[0]) * discBrightness;
    material.baseColor[1] = static_cast<float>(sunColor[1] * discTint[1]) * discBrightness;
    material.baseColor[2] = static_cast<float>(sunColor[2] * discTint[2]) * discBrightness;
}

void PlanetChunkRenderer::ensureMoon()
{
    if (!params_.moonEnabled || params_.moonAngularRadiusRadians <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || moon_ != renderer::invalidMesh)
    {
        return;
    }

    // 48 x 24 rings: the companion spans a dozen-odd pixels at the default
    // angular radius, and that tessellation keeps the silhouette round and
    // the terminator's gradient clean at capture resolutions (coarser grids
    // read as a faceted ball once the disc crosses ~10 px).
    constexpr std::uint32_t kSegments = 48;
    constexpr std::uint32_t kBands = 24;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    buildMoonMesh(params_, vertices, indices, kSegments, kBands);
    moon_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (moon_ == renderer::invalidMesh)
    {
        return;
    }
    renderer::MeshMaterial &material = moonMaterials_.emplace_back();
    // LIT draw (unlit stays false): the sun-facing hemisphere shades bright
    // and the trailing one dark - the phase is the shading's own output and
    // never desynchronizes from the scene's sun.
    material.roughnessFactor = 0.95f; // matte regolith
    material.metallicFactor = 0.0f;
    // The builder's grid winds the opposite way from the demo's other
    // outward shells (the star-disc fan convention), so the declared sign
    // here is negative; the wrong sign culls the sphere entirely, and
    // leaving it to the winding vote leaves the interior shading dim.
    material.frontFaceWinding = -1.0f;
    // depthWrite stays true (the default): the companion hides the stars
    // behind it and is hidden by whatever stands in front of it.
}

void PlanetChunkRenderer::updateMoon(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up,
    const glm::dvec3 &moonDirection, bool sunEnabled, double farZ)
{
    moonActive_ = moon_ != renderer::invalidMesh && !moonMaterials_.empty();
    if (!moonActive_)
    {
        return;
    }
    if (glm::length(moonDirection) < 1.0e-9)
    {
        moonActive_ = false;
        return;
    }
    const glm::dvec3 direction = glm::normalize(moonDirection);

    // Backdrop distance along the moon direction: in space it sits between
    // the sun disc (0.45 farZ) and the star dome (0.5 farZ), so the planet
    // and its weather hide it when they stand in the way and it hides the
    // stars behind it. In air it must land JUST INSIDE the sky dome's own
    // radius while staying BEYOND the cloud deck (0.9 dome): the depth
    // comparison is strict-less, so a companion inside the dome survives the
    // dome's later draw (the dome's equal-or-farther fragments fail against
    // the companion's nearer depth) while the deck's banks still occlude it.
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    const bool inAir = planet::atmosphereWeight(params_, eyeAltitude) > 0.0;
    const double reliefTop = std::max(0.0, params_.derivedMaxReliefMeters);
    const double sightCeiling = std::max(1.0, eyeAltitude + reliefTop);
    const double horizonDistance = std::sqrt(std::max(0.0, sightCeiling * (2.0 * params_.radiusMeters + sightCeiling)));
    const double sunBackdropRadius = glm::clamp(1.25 * horizonDistance, 1.0e3, 0.45 * std::max(1.0, farZ));
    const double backdropRadius = inAir
        ? std::min(0.98 * sunBackdropRadius, 0.97 * 0.40 * std::max(1.0, farZ))
        : 0.47 * std::max(1.0, farZ);

    const glm::vec3 worldEye(static_cast<float>(params_.centerMeters[0] + eyePlanetLocal.x),
        static_cast<float>(params_.centerMeters[1] + eyePlanetLocal.y),
        static_cast<float>(params_.centerMeters[2] + eyePlanetLocal.z));
    const glm::vec3 worldDirection(static_cast<float>(direction.x), static_cast<float>(direction.y),
        static_cast<float>(direction.z));
    const glm::vec3 place = worldEye + worldDirection * static_cast<float>(backdropRadius);
    // Tidally locked orientation: the sphere's local +Z points at the planet
    // (the same face turns planetward every orbit, the way a real companion
    // reads); the seed-fixed spin about that axis carries the maria around.
    glm::vec3 right = glm::cross(worldDirection, glm::vec3(0.0f, 1.0f, 0.0f));
    if (glm::length(right) < 1.0e-5f)
    {
        right = glm::cross(worldDirection, glm::vec3(1.0f, 0.0f, 0.0f));
    }
    right = glm::normalize(right);
    const glm::vec3 down = glm::cross(right, worldDirection);
    const glm::mat3 rotation(glm::mat3(right, down, -worldDirection));
    const float sphereRadius = static_cast<float>(backdropRadius * std::tan(params_.moonAngularRadiusRadians));
    moonTransform_ = glm::translate(glm::mat4(1.0f), place) * glm::mat4(rotation)
        * glm::scale(glm::mat4(1.0f), glm::vec3(sphereRadius));

    // Tint: the regolith's reflected light passes the same air column the
    // sun's direct light does, so the companion reddens and dims toward the
    // horizon exactly like the low sun (see planet::sunDirectTint, keyed on
    // the moon's own elevation here); the eye's atmosphere weight fades the
    // curve back to the bare regolith for space views.
    const glm::dvec3 upNormalized = glm::length(up) > 1.0e-9 ? glm::normalize(up) : glm::dvec3(0.0, 1.0, 0.0);
    const double moonUp = glm::dot(direction, upNormalized);
    const double moonAirWeight = planet::atmosphereWeight(params_, eyeAltitude);
    const planet::Vec3 directTint = planet::sunDirectTint(params_, moonUp, sunEnabled);
    const double albedo = std::max(0.0, params_.moonAlbedo);
    renderer::MeshMaterial &material = moonMaterials_.front();
    material.baseColor[0] = static_cast<float>(albedo * (1.0 + (directTint.x - 1.0) * moonAirWeight));
    material.baseColor[1] = static_cast<float>(albedo * (1.0 + (directTint.y - 1.0) * moonAirWeight));
    material.baseColor[2] = static_cast<float>(albedo * (1.0 + (directTint.z - 1.0) * moonAirWeight));
}

void PlanetChunkRenderer::ensureAurora(const glm::dvec3 &sunDirection)
{
    if (!params_.auroraEnabled || params_.auroraStrength <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || aurora_ != renderer::invalidMesh)
    {
        return;
    }

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<std::uint8_t> alphaPixels;
    buildAuroraMesh(params_, sunDirection, vertices, indices, alphaPixels);
    if (indices.empty())
    {
        return;
    }
    aurora_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (aurora_ == renderer::invalidMesh)
    {
        return;
    }
    // The curtain's intensity field (night gate, rays, altitude fade) rides
    // the alpha channel; the transparent route multiplies the material's
    // opacity by the texel alpha, so the day side composites to nothing
    // instead of alpha-darkening the sky behind it.
    renderer::TextureDescription alphaDescription;
    alphaDescription.width = 256;
    alphaDescription.height = 16;
    alphaDescription.pixels = alphaPixels.data();
    alphaDescription.filter = renderer::TextureFilter::Linear;
    alphaDescription.address = renderer::TextureAddress::ClampEdge;
    renderer::RenderError alphaError = renderer::RenderError::None;
    auroraTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &alphaDescription, &alphaError);
    // Planet-local ring vertices: the placement mirrors the cloud shell
    // (planet-local geometry shifted by the planet's center offset); the
    // mesh is static after this (the night gate is baked against the run's
    // fixed sun direction).
    auroraTransform_ = glm::translate(glm::mat4(1.0f),
        glm::vec3(static_cast<float>(params_.centerMeters[0]), static_cast<float>(params_.centerMeters[1]),
            static_cast<float>(params_.centerMeters[2])));
    renderer::MeshMaterial &material = auroraMaterials_.emplace_back();
    // Unlit emissive: the vertex colors carry the curtain's chroma (green
    // foot, violet crown) scaled by the configured strength; the intensity
    // field (night gate, rays, fade) rides the alpha texture. Opacity below
    // 1 routes the draw into the renderer's transparent pass: alpha-blended
    // over the opaque scene (limb, terrain, clouds, companion) with depth
    // writes off, depth-tested so the planet hides each ring's far side.
    material.unlit = true;
    material.opacity = 0.32f;
    material.doubleSided = true; // the crown reads from both sides of the ring
    material.baseColorTexture = auroraTexture_;
    material.baseColor[0] = static_cast<float>(params_.auroraStrength);
    material.baseColor[1] = static_cast<float>(params_.auroraStrength);
    material.baseColor[2] = static_cast<float>(params_.auroraStrength);
    auroraActive_ = true;
}

void PlanetChunkRenderer::updateAurora(const glm::dvec3 &eyePlanetLocal)
{
    auroraActive_ = aurora_ != renderer::invalidMesh && !auroraMaterials_.empty();
    if (!auroraActive_)
    {
        return;
    }
    // Space-view backdrop (see the header note): inside the atmosphere the
    // ring only blends a near-transparent wash over the horizon haze - the
    // green oval belongs to the dark limb an orbital camera reads.
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    if (planet::atmosphereWeight(params_, eyeAltitude) > 0.0)
    {
        auroraActive_ = false;
    }
}

void PlanetChunkRenderer::ensureCirrusLayer(const glm::dvec3 &sunDirection)
{
    if (!params_.cirrusEnabled || params_.cirrusStrength <= 0.0 || params_.cirrusCoverageAmount <= 0.0
        || renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || cirrusLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // Lat-long grid on the cirrus shell (datum radius + the deck's altitude).
    // The streak field band-limits at the grid's own spacing (the same
    // convention the cumulus shell's coverage sampling uses), so the veil
    // the mesh resolves keeps its wisps at the resolution the grid can
    // represent instead of aliasing them into speckle. The BAND count rules
    // the streaks' cross-streak scale: 160 latitude rows sit an angular
    // constant apart (row spacing scales with the radius - ~125 km at
    // Earth scale, under a tenth of the radius-scaled streak width), so
    // the filaments curve smoothly at capture resolutions - a coarser grid
    // photographed the veil as blocks a dozen pixels wide). The quads stay
    // cheap - the deck is a translucent veil, not a bank outline, so it
    // needs none of the cumulus shell's contour-walk machinery.
    constexpr std::uint32_t kSegments = 320;
    constexpr std::uint32_t kBands = 160;
    // Cirrus belt: the deck rides the mid-latitude westerlies and the
    // subtropical jets, so the mesh spans the extrapolar latitudes; the
    // poles stay clear (cirrus photographs there are rare, and the aurora
    // rings own the polar limb anyway).
    constexpr double kBeltLowRadians = -1.08;  // ~-62 deg
    constexpr double kBeltHighRadians = 1.08; // ~+62 deg
    // A quad renders only where one of its four corners carries this much
    // baked alpha (streak intensity times the day-side gate); quads below
    // the gate composite to nothing and are skipped whole.
    // The floor sits well above zero: the veil is a TRANSLUCENT draw whose
    // every covered supersample pays a texel fetch and a framebuffer blend,
    // so the faint penumbral wisps around the streak cores cost frame time
    // without ever reading as cloud - the strong streaks alone carry the
    // deck.
    constexpr double kQuadGateFloor = 0.06;
    // The intensity texture's own resolution is DECOUPLED from the mesh grid
    // (512 x 512): the veil's alpha interpolates through the texture per
    // sample, so the streak definition comes from the texture's texels - a
    // texture coupled to the mesh's rows quantized the wisps into
    // two-texel blocks at capture resolutions. The mesh's own rows carry
    // only the geometry (a smooth sheet), the texture carries the deck.
    constexpr std::uint32_t kAlphaWidth = 512;
    constexpr std::uint32_t kAlphaHeight = 512;

    const double shellRadius = params_.radiusMeters + params_.cirrusAltitudeMeters;
    const double vertexSpacing = glm::two_pi<double>() * shellRadius / static_cast<double>(kSegments);
    const double bandLimit = 2.0 * vertexSpacing;

    // The deck's intensity field, shared by the alpha texture and the quad
    // gate: streak weight x day-side gate. The gate keeps the veil over the
    // sunlit planet and lets it fall to nothing through the terminator into
    // the night - a night-side cirrus wisp would only alpha-darken the dark
    // limb behind it (the aurora's night gate works the same inequality in
    // reverse). The gate is baked against the run's fixed sun direction.
    const glm::dvec3 sun = glm::length(sunDirection) > 1.0e-9 ? glm::normalize(sunDirection) : glm::dvec3(0.0);
    const auto intensityAt = [&](double latitude, double azimuth) -> double
    {
        const double cosLatitude = std::cos(latitude);
        const planet::Vec3 direction{std::cos(azimuth) * cosLatitude, std::sin(latitude),
            std::sin(azimuth) * cosLatitude};
        const double streak = planet::cirrusStreakWeight(params_, direction, bandLimit);
        const double dayGate = glm::length(sun) > 1.0e-9
            ? planet::smoothstep(-0.08, 0.20, static_cast<double>(planet::dot(direction,
                planet::Vec3{sun.x, sun.y, sun.z})))
            : 1.0;
        return streak * dayGate;
    };

    // Two evaluations of the one intensity field, each at the resolution of
    // its own consumer: the mesh's corner grid gates the quads at the mesh's
    // own band/segment coordinates, and the alpha texture fills at its own
    // finer rows/columns (a row per 512th of the belt, a column per 512th
    // of the azimuth) so the sampled streaks interpolate smoothly instead
    // of quantizing to the mesh's coarser rows.
    std::vector<float> cornerIntensities(static_cast<std::size_t>(kSegments + 1u) * (kBands + 1u));
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = kBeltLowRadians
            + (kBeltHighRadians - kBeltLowRadians) * (static_cast<double>(band) / static_cast<double>(kBands));
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth
                = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            cornerIntensities[static_cast<std::size_t>(band) * (kSegments + 1u) + segment]
                = static_cast<float>(intensityAt(latitude, azimuth));
        }
    }
    std::vector<std::uint8_t> alphaPixels(
        static_cast<std::size_t>(kAlphaWidth) * kAlphaHeight * 4u, 255u);
    for (std::uint32_t row = 0; row < kAlphaHeight; ++row)
    {
        const double latitude = kBeltLowRadians
            + (kBeltHighRadians - kBeltLowRadians)
                * (static_cast<double>(row) / static_cast<double>(kAlphaHeight - 1u));
        for (std::uint32_t column = 0; column < kAlphaWidth; ++column)
        {
            const double azimuth
                = glm::two_pi<double>() * static_cast<double>(column) / static_cast<double>(kAlphaWidth);
            const double intensity = intensityAt(latitude, azimuth);
            const std::size_t index
                = (static_cast<std::size_t>(row) * kAlphaWidth + column) * 4u;
            alphaPixels[index + 3] = static_cast<std::uint8_t>(std::clamp(intensity, 0.0, 1.0) * 255.0);
        }
        // The last texture column mirrors the first: the mesh's seam vertex
        // (u = 1, azimuth 0) must not read a different intensity than u = 0
        // (the field is periodic in azimuth, so the wrap is exact).
        alphaPixels[(static_cast<std::size_t>(row) * kAlphaWidth + kAlphaWidth - 1u) * 4u + 3u]
            = alphaPixels[static_cast<std::size_t>(row) * kAlphaWidth * 4u + 3u];
    }

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(static_cast<std::size_t>(kSegments + 1u) * (kBands + 1u));
    indices.reserve(static_cast<std::size_t>(kSegments) * kBands * 6u);
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = kBeltLowRadians
            + (kBeltHighRadians - kBeltLowRadians) * (static_cast<double>(band) / static_cast<double>(kBands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth
                = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(kSegments);
            const planet::Vec3 direction{std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude};
            planet::PlanetVertex vertex{}; // value-init: untouched uv pair reads as finite zeros
            vertex.position[0] = static_cast<float>(direction.x * shellRadius);
            vertex.position[1] = static_cast<float>(direction.y * shellRadius);
            vertex.position[2] = static_cast<float>(direction.z * shellRadius);
            vertex.normal[0]   = static_cast<float>(direction.x);
            vertex.normal[1]   = static_cast<float>(direction.y);
            vertex.normal[2]   = static_cast<float>(direction.z);
            // Chroma only - the intensity field rides the alpha texture, so
            // the blend carries the veil's brightness (the composite
            // multiplies the same way: chroma x strength x opacity x
            // intensity). Ice-crystal decks read slightly cool white.
            vertex.color[0] = 1.0f;
            vertex.color[1] = 0.99f;
            vertex.color[2] = 0.97f;
            // UV spans the belt linearly: the texture's rows own the deck's
            // cross-streak resolution, the mesh's rows only place the sheet.
            vertex.uv[0]    = static_cast<float>(static_cast<double>(segment) / static_cast<double>(kSegments));
            vertex.uv[1]    = static_cast<float>(
                (latitude - kBeltLowRadians) / (kBeltHighRadians - kBeltLowRadians));
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            const auto cornerIntensity = [&](std::uint32_t row, std::uint32_t column)
            {
                const std::size_t index = static_cast<std::size_t>(row) * (kSegments + 1u) + column;
                return cornerIntensities[index];
            };
            const float maxIntensity = std::max(std::max(cornerIntensity(band, segment),
                                                 cornerIntensity(band, segment + 1u)),
                std::max(cornerIntensity(band + 1u, segment), cornerIntensity(band + 1u, segment + 1u)));
            if (maxIntensity <= kQuadGateFloor)
            {
                continue; // clear sky: nothing to composite
            }
            const std::uint32_t a = band * (kSegments + 1u) + segment;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (kSegments + 1u);
            const std::uint32_t d = c + 1u;
            // Outward winding for the +1 convention: (a, c, b) / (b, c, d)
            // (rows climb with latitude, columns advance with azimuth).
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
    if (indices.empty())
    {
        return; // the configured coverage keeps the whole belt clear
    }

    cirrusLayer_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (cirrusLayer_ == renderer::invalidMesh)
    {
        return;
    }
    // The veil's intensity field (streak weight x day gate) rides the alpha
    // channel; the transparent route multiplies the material's opacity by
    // the texel alpha, so the night side composites to nothing instead of
    // alpha-darkening the limb behind it.
    renderer::TextureDescription alphaDescription;
    alphaDescription.width = kAlphaWidth;
    alphaDescription.height = kAlphaHeight;
    alphaDescription.pixels = alphaPixels.data();
    alphaDescription.filter = renderer::TextureFilter::Linear;
    alphaDescription.address = renderer::TextureAddress::ClampEdge;
    renderer::RenderError alphaError = renderer::RenderError::None;
    cirrusTexture_ = renderer_->createTexture(renderer_->self, rendererHandle_, &alphaDescription, &alphaError);
    // Planet-local shell vertices: the placement mirrors the cloud shell
    // (planet-local geometry shifted by the planet's center offset); the
    // mesh is static after this (the day gate is baked against the run's
    // fixed sun direction).
    cirrusLayerTransform_ = glm::translate(glm::mat4(1.0f),
        glm::vec3(static_cast<float>(params_.centerMeters[0]), static_cast<float>(params_.centerMeters[1]),
            static_cast<float>(params_.centerMeters[2])));
    renderer::MeshMaterial &material = cirrusLayerMaterials_.emplace_back();
    // Unlit translucent veil: the vertex colors carry the deck's cool-white
    // chroma, the intensity field rides the alpha texture, and the material
    // opacity caps the composite's strongest filament. Opacity below 1
    // routes the draw into the renderer's transparent pass: alpha-blended
    // over the opaque scene (terrain, water, cumulus banks) with depth
    // writes off, depth-tested so the planet hides the far side of the
    // shell and the cumulus banks occlude the veil behind them.
    material.unlit = true;
    material.opacity = 0.70f;
    material.baseColorTexture = cirrusTexture_;
    material.baseColor[0] = static_cast<float>(params_.cirrusStrength);
    material.baseColor[1] = static_cast<float>(params_.cirrusStrength);
    material.baseColor[2] = static_cast<float>(params_.cirrusStrength);
    // Outward shell seen from outside at every view that draws it (globe and
    // orbit keep the eye above the veil): the declared winding culls the
    // far hemisphere per triangle. The lat-long grid's rows climb with
    // latitude and its columns advance with azimuth, so the outward-wound
    // triangle order is (a, c, b) / (b, c, d) - the reverse of the sky
    // deck's rows, which run pole-to-pole the other way.
    material.frontFaceWinding = macroShellFrontFaceWinding;
    cirrusLayerActive_ = true;
}

void PlanetChunkRenderer::updateCirrusLayer(const glm::dvec3 &eyePlanetLocal)
{
    cirrusLayerActive_ = cirrusLayer_ != renderer::invalidMesh && !cirrusLayerMaterials_.empty();
    if (!cirrusLayerActive_)
    {
        return;
    }
    // Space-view backdrop (see the header note): inside the air the sky deck
    // owns the whole sky, and the veil's own shell would smear through it.
    const double eyeAltitude = glm::length(eyePlanetLocal) - params_.radiusMeters;
    if (planet::atmosphereWeight(params_, eyeAltitude) > 0.0)
    {
        cirrusLayerActive_ = false;
    }
}

void PlanetChunkRenderer::ensureSkyCloudLayer(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection)
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || skyCloudLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // The dome's INITIAL frame first: the deck mesh, its UVs and its bake
    // all live in this frame (see ensureDeckTexture), so the frame basis
    // must exist before the bake runs.
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
    const glm::dmat3 domeToPlanet(tangent, upNormalized, third);

    ensureDeckTexture(sunDirection, domeToPlanet, eyePlanetLocal);
    if (deckTexture_ == renderer::invalidTexture || deckTexturePixels_.empty())
    {
        return;
    }

    // The deck is built on the unit ball in the sky dome's INITIAL frame
    // (its +Y on the eye's local up, so band latitude equals elevation
    // angle); the dome's per-frame update rotates it with the sky and
    // scales it inside the dome's radius. The UVs are the mesh's own
    // (segment, band) grid - the same grid the bake's texels follow - so
    // the per-pixel alpha, wisp fringes and cyclone arms of the bake land
    // on the sky at their own scale over well-conditioned UVs. A quad
    // guard above the horizon keeps the mesh off the rim band, where the
    // atmosphere's own haze swallows the detail anyway.
    //
    // Grid resolution: 512x128 over the hemisphere keeps a cell at ~0.70
    // degrees - under the half-degree a 1920-wide frame resolves at eye
    // level, so the geometry never limits the deck; the structure comes
    // from the texture per pixel, and the grid only places the sheet.
    constexpr std::uint32_t kSegments = 512;
    constexpr std::uint32_t kBands = 128;
    constexpr double kHorizonGuardRadians = 0.03490658503988659; // 2 deg above the horizon
    constexpr double kCornerGate = 0.03;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(static_cast<std::size_t>(kSegments + 1u) * (kBands + 1u));
    for (std::uint32_t band = 0; band <= kBands; ++band)
    {
        const double latitude = kHorizonGuardRadians
            + (glm::pi<double>() * 0.5 - kHorizonGuardRadians)
                * (static_cast<double>(band) / static_cast<double>(kBands));
        const double cosLatitude = std::cos(latitude);
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= kSegments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment)
                / static_cast<double>(kSegments);
            const glm::dvec3 domeDirection(std::cos(azimuth) * cosLatitude, sinLatitude,
                std::sin(azimuth) * cosLatitude);
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(domeDirection.x);
            vertex.position[1] = static_cast<float>(domeDirection.y);
            vertex.position[2] = static_cast<float>(domeDirection.z);
            vertex.normal[0] = static_cast<float>(domeDirection.x);
            vertex.normal[1] = static_cast<float>(domeDirection.y);
            vertex.normal[2] = static_cast<float>(domeDirection.z);
            // Neutral chroma: the bake's RGB carries the ground-sky lit
            // response, the dome's per-frame tint rides the material
            // baseColor, and the deck composites per texel over the dome
            // behind it (the space shell's own route).
            vertex.color[0] = 1.0f;
            vertex.color[1] = 1.0f;
            vertex.color[2] = 1.0f;
            // Grid-aligned UVs (see ensureDeckTexture): texel columns
            // follow the azimuth stations, rows the latitude span.
            vertex.uv[0] = static_cast<float>(static_cast<double>(segment) / static_cast<double>(kSegments));
            vertex.uv[1] = static_cast<float>(static_cast<double>(band) / static_cast<double>(kBands));
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t band = 0; band < kBands; ++band)
    {
        for (std::uint32_t segment = 0; segment < kSegments; ++segment)
        {
            const auto cornerAlpha = [&](std::uint32_t row, std::uint32_t column)
            {
                const double u = static_cast<double>(column) / static_cast<double>(kSegments);
                const double v = static_cast<double>(row) / static_cast<double>(kBands);
                return cloudBakeAlpha(deckTexturePixels_, deckTextureWidth_, deckTextureHeight_, u, v);
            };
            if (std::max(std::max(cornerAlpha(band, segment), cornerAlpha(band, segment + 1u)),
                    std::max(cornerAlpha(band + 1u, segment), cornerAlpha(band + 1u, segment + 1u)))
                < kCornerGate)
            {
                continue; // clear sky over this cell: nothing to composite
            }
            const std::uint32_t a = band * (kSegments + 1u) + segment;
            const std::uint32_t b = a + 1u;
            const std::uint32_t c = a + (kSegments + 1u);
            const std::uint32_t d = c + 1u;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
    if (indices.empty())
    {
        return; // the configured coverage keeps the whole sky clear
    }

    skyCloudLayer_ = uploadShellMesh(*renderer_, rendererHandle_, vertices, indices);
    if (skyCloudLayer_ == renderer::invalidMesh)
    {
        return;
    }
    skyCloudMaterials_.emplace_back();
    renderer::MeshMaterial &material = skyCloudMaterials_.front();
    // Unlit translucent deck (the space shell's route, ground-lit): the
    // bake's RGB carries the ground-sky lit response, its alpha the
    // per-texel coverage, and the dome's per-frame tint (sun elevation,
    // haze weight) shades the deck identically to the sky it floats in.
    // Opacity just under 1 routes the draw into the renderer's transparent
    // pass - alpha-blended per texel over the dome gradient behind it,
    // depth-tested against the terrain (banks hide behind ridgelines) with
    // depth writes off.
    material.unlit = true;
    material.depthWrite = false;
    material.opacity = 0.94f;
    material.baseColorTexture = deckTexture_;
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

void PlanetChunkRenderer::destroyMoon()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && moon_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, moon_);
    }
    moon_ = renderer::invalidMesh;
    moonMaterials_.clear();
    moonActive_ = false;
}

void PlanetChunkRenderer::destroyAurora()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && aurora_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, aurora_);
    }
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && auroraTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, auroraTexture_);
    }
    aurora_ = renderer::invalidMesh;
    auroraTexture_ = renderer::invalidTexture;
    auroraMaterials_.clear();
    auroraActive_ = false;
}

void PlanetChunkRenderer::destroyCirrusLayer()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && cirrusLayer_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, cirrusLayer_);
    }
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && cirrusTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, cirrusTexture_);
    }
    cirrusLayer_ = renderer::invalidMesh;
    cirrusTexture_ = renderer::invalidTexture;
    cirrusLayerMaterials_.clear();
    cirrusLayerActive_ = false;
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
    destroyStarDome();
    destroySunDisc();
    destroyMoon();
    destroyAurora();
    destroyCirrusLayer();
    destroyCloudTexture();
    destroyWaterDetailTextures();
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && deckTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, deckTexture_);
    }
    deckTexture_ = renderer::invalidTexture;
    cloudField_.reset();
    drawCommands_.clear();
    planet_ = nullptr;
    planetHandle_ = planet::invalidPlanet;
    renderer_ = nullptr;
    rendererHandle_ = renderer::invalidRenderer;
}

void PlanetChunkRenderer::destroyStarDome()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && starDome_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, starDome_);
    }
    starDome_ = renderer::invalidMesh;
    starDomeMaterials_.clear();
    starDomeActive_ = false;
}

void PlanetChunkRenderer::destroySunDisc()
{
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && sunDisc_ != renderer::invalidMesh)
    {
        renderer_->destroyMesh(renderer_->self, rendererHandle_, sunDisc_);
    }
    sunDisc_ = renderer::invalidMesh;
    if (renderer_ != nullptr && rendererHandle_ != renderer::invalidRenderer
        && sunHaloTexture_ != renderer::invalidTexture && renderer_->destroyTexture != nullptr)
    {
        renderer_->destroyTexture(renderer_->self, rendererHandle_, sunHaloTexture_);
    }
    sunHaloTexture_ = renderer::invalidTexture;
    sunDiscMaterials_.clear();
    sunDiscActive_ = false;
}

void PlanetChunkRenderer::ensureSkyDome(const glm::dvec3 &sunDirection, bool sunEnabled, const glm::dvec3 &up)
{
    if (!params_.atmosphereEnabled || renderer_ == nullptr || rendererHandle_ == renderer::invalidRenderer
        || skyDome_ != renderer::invalidMesh)
    {
        return;
    }

    // Lat-long sphere on the unit ball. The gradient is keyed to each
    // vertex's elevation above the local horizon (dot with the dome's +Y)
    // AND its angle off the dome's +X meridian: the bake places the
    // canonical sun on that meridian at the sun's own local elevation, so
    // the Mie glare wedge, the sun-side horizon whitening and the warm gold
    // band land where the per-frame rotation turns +X toward the real sun
    // (see updateSkyDome). Doubled segment count over the legacy dome: the
    // azimuth structure matters as much as the elevation ramp now, and the
    // dome is unlit geometry whose cost is fill, not shading.
    constexpr std::uint32_t kSegments = 96;
    constexpr std::uint32_t kBands = 32;
    const glm::dvec3 upNormalized = glm::length(up) > 1.0e-9 ? glm::normalize(up) : glm::dvec3(0.0, 1.0, 0.0);
    const double sunUp = sunEnabled && glm::length(sunDirection) > 1.0e-9
        ? glm::clamp(glm::dot(glm::normalize(sunDirection), upNormalized), -1.0, 1.0)
        : 2.0; // sentinel: no sun structure bakes (see rebakeSkyDomeColors)
    skyDomeBakedSunUp_ = sunUp;
    const double sunElevation = std::asin(glm::clamp(sunUp, -1.0, 1.0));
    // Canonical sun in dome coordinates: on the +X meridian at the sun's
    // own elevation (a disabled sun keeps the plain rotational ramp - the
    // sun-aware gradient's directional terms all vanish without it).
    const glm::dvec3 canonicalSun(std::cos(sunElevation), std::sin(sunElevation), 0.0);
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
            const planet::Vec3 color = planet::skyGradientColor(params_, direction.y,
                glm::dot(direction, canonicalSun), skyDomeBakedSunUp_);
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
            // Outward winding for the +1 convention: (a, c, b) / (b, c, d)
            // (rows climb with latitude, columns advance with azimuth).
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }

    const renderer::MeshDescription description = canonicalMeshDescription(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()), indices.data(),
        static_cast<std::uint32_t>(indices.size()));

    renderer::RenderError error = renderer::RenderError::None;
    skyDome_ = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
    if (skyDome_ == renderer::invalidMesh)
    {
        std::cerr << "demo_planet: sky dome upload failed (error " << static_cast<std::uint32_t>(error) << ")\n";
        return;
    }
    skyDomeMaterials_.emplace_back();
    skyDomeMaterials_.front().unlit = true;
    // Backdrop contract: the dome never writes depth (the terrain the eye
    // stands on pushed BEFORE it wins the shared pixels by depth test, and
    // the later sun disc - which sits at the same backdrop radius - must
    // not be depth-rejected against the dome's own surface).
    skyDomeMaterials_.front().depthWrite = false;
}

void PlanetChunkRenderer::rebakeSkyDomeColors(double sunUp)
{
    if (renderer_ == nullptr || renderer_->updateMeshVertices == nullptr || skyDome_ == renderer::invalidMesh)
    {
        return;
    }
    // Same grid as ensureSkyDome (the geometry never changes - only the
    // colors ride the sun's elevation); a drift beyond the updateSkyDome
    // threshold rewrites the vertex colors in place.
    constexpr std::uint32_t kSegments = 96;
    constexpr std::uint32_t kBands = 32;
    const double sunElevation = std::asin(glm::clamp(sunUp, -1.0, 1.0));
    const glm::dvec3 canonicalSun(std::cos(sunElevation), std::sin(sunElevation), 0.0);
    skyDomeBakedSunUp_ = sunUp;
    std::vector<planet::PlanetVertex> vertices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
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
            const planet::Vec3 color = planet::skyGradientColor(params_, direction.y,
                glm::dot(direction, canonicalSun), skyDomeBakedSunUp_);
            vertex.color[0] = static_cast<float>(color.x);
            vertex.color[1] = static_cast<float>(color.y);
            vertex.color[2] = static_cast<float>(color.z);
            vertices.push_back(vertex);
        }
    }
    renderer_->updateMeshVertices(renderer_->self, rendererHandle_, skyDome_, vertices.data(),
        static_cast<std::uint32_t>(vertices.size()), nullptr);
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

    // The sun's own local elevation this frame: a drifting sun (an eye
    // walking toward a pole) rebakes the dome colors when it strays from the
    // baked elevation, so the glare wedge and the warm band keep landing on
    // the sun the frame actually shows. Capture runs never drift far enough
    // to fire this twice.
    const double sunUp = sunEnabled && glm::length(sunDirection) > 1.0e-9
        ? glm::dot(glm::normalize(sunDirection), upNormalized)
        : 1.0;
    if (sunEnabled && glm::length(sunDirection) > 1.0e-9
        && std::abs(sunUp - skyDomeBakedSunUp_) > 0.03)
    {
        rebakeSkyDomeColors(glm::clamp(sunUp, -1.0, 1.0));
    }

    // Orientation - two bases this frame:
    // (1) the DOME's own: dome +Y onto the local up and dome +X onto the
    //     sun's horizon projection, so the baked canonical sun (on the +X
    //     meridian, see ensureSkyDome) lands on the real sun's azimuth -
    //     the glare wedge, the sun-side horizon whitening and the warm band
    //     rotate with the sun instead of with the eye. A sun at the zenith
    //     (or a disabled sun) falls back to any tangent: the gradient is
    //     symmetric about the up axis there.
    glm::dvec3 sunward{};
    bool sunwardValid = false;
    if (sunEnabled && glm::length(sunDirection) > 1.0e-9)
    {
        const glm::dvec3 sunNormalized = glm::normalize(sunDirection);
        sunward = sunNormalized - upNormalized * glm::dot(sunNormalized, upNormalized);
        if (glm::length(sunward) > 1.0e-4)
        {
            sunward = glm::normalize(sunward);
            sunwardValid = true;
        }
    }
    glm::dvec3 domeTangent = sunward;
    if (!sunwardValid)
    {
        domeTangent = glm::cross(upNormalized, glm::dvec3(0.0, 0.0, 1.0));
        if (glm::length(domeTangent) < 1.0e-6)
        {
            domeTangent = glm::cross(upNormalized, glm::dvec3(1.0, 0.0, 0.0));
        }
        domeTangent = glm::normalize(domeTangent);
    }
    const glm::dvec3 domeThird = glm::cross(domeTangent, upNormalized);
    const glm::mat3 domeRotation(domeTangent, upNormalized, domeThird);
    // (2) the DECK's basis: the legacy up-aligned tangent (cross with +Z).
    //     The deck mesh, its UVs and its bake all live in the eye's INITIAL
    //     local frame (see ensureSkyCloudLayer), so it must keep rotating by
    //     that frame's convention - rotating it by the sun-anchored basis
    //     would swing the cloud banks around the sun as the sun's azimuth
    //     moves instead of keeping them anchored to the planet.
    glm::dvec3 deckTangent = glm::cross(upNormalized, glm::dvec3(0.0, 0.0, 1.0));
    if (glm::length(deckTangent) < 1.0e-6)
    {
        deckTangent = glm::cross(upNormalized, glm::dvec3(1.0, 0.0, 0.0));
    }
    deckTangent = glm::normalize(deckTangent);
    const glm::dvec3 deckThird = glm::cross(deckTangent, upNormalized);
    const glm::mat3 deckRotation(deckTangent, upNormalized, deckThird);
    // The dome vertices are unit-ball offsets: the placement adds the
    // planet's world center to the planet-local eye (world = planet-local +
    // center), so the dome follows the world eye exactly.
    const glm::vec3 worldEye(static_cast<float>(params_.centerMeters[0] + eyePlanetLocal.x),
        static_cast<float>(params_.centerMeters[1] + eyePlanetLocal.y),
        static_cast<float>(params_.centerMeters[2] + eyePlanetLocal.z));
    skyDomeTransform_ = glm::translate(glm::mat4(1.0f), worldEye)
        * glm::mat4(domeRotation) * glm::scale(glm::mat4(1.0f), glm::vec3(domeRadius));

    const planet::Vec3 tint = planet::sunSkyTint(params_, sunUp, sunEnabled);
    renderer::MeshMaterial &material = skyDomeMaterials_.front();
    material.baseColor[0] = static_cast<float>(tint.x * weight);
    material.baseColor[1] = static_cast<float>(tint.y * weight);
    material.baseColor[2] = static_cast<float>(tint.z * weight);
    skyDomeActive_ = true;

    // Sky cloud deck rides the dome: same eye placement, the DECK's own
    // rotation (its bake's initial frame), a scale just inside the dome's
    // rim (the gradient keeps reading behind the deck's contoured gaps),
    // and the dome's tint so sun elevation and haze weight shade both
    // layers identically.
    if (skyCloudActive_ && skyCloudLayer_ != renderer::invalidMesh && !skyCloudMaterials_.empty())
    {
        skyCloudTransform_ = glm::translate(glm::mat4(1.0f), worldEye)
            * glm::mat4(deckRotation)
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
