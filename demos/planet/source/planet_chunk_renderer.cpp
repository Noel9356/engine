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
    // Horizon-guard fade: when set, vertices within this many radians above
    // minQuadLatitudeRadians blend their color toward the sky gradient behind
    // them, so the deck's lower boundary reads as thinning cloud instead of
    // the hard polygon cut the quad guard would otherwise leave.
    double guardFadeRadians = 0.0;
    // Inclusion-floor lift across the guard fade band (0 = off): quads inside
    // the band raise their coverage floor proportionally, so banks
    // DISINTEGRATE into fragments toward the horizon instead of staying bank
    // sized and merely paling out - the color fade alone leaves flattened
    // pale sheets hanging over the horizon, which reads as an artifact.
    double guardFloorLift = 0.0;
    // Edge softening: when set, quad corners with coverage near the floor
    // blend their vertex color toward the sky gradient behind them (sampled
    // at the corner's own elevation), so a deck's contoured boundary reads
    // as thinning cloud instead of a hard polygon cutout. The per-frame sun
    // tint applies to both the cloud and the sky colors, so the blend tracks
    // the dome behind it.
    bool hazeEdgeBlend = false;
    // Half-width of the haze edge blend above the coverage floor: wider
    // values stretch the thinning fringe (soft decks), narrow ones keep the
    // contour crisp (puffy cumulus against the sky).
    double hazeEdgeWidth = 0.45;
    // Number of 3x3 box-smoothing passes over the coverage grid before the
    // contour walk. Each pass rounds the quad inclusion boundary one grid
    // step further; two passes give contour lines whose curvature stays
    // above the grid spacing, so the marching-squares polygons read as
    // organic bank outlines instead of angular cutouts.
    std::uint32_t smoothingPasses = 1;
    // Greedy interior merge (see the merge pass): runs of solid interior
    // quads collapse into maximal grid-aligned rectangles. On shells whose
    // cells project to a handful of pixels or fewer (the macro shell at
    // orbit distance) this is pure savings, but on the sky deck - where one
    // cell spans a dozen pixels - the merged rectangles read as blocky
    // grid-aligned slabs, so the deck keeps its per-quad tessellation and
    // pays the extra triangles.
    bool mergeInterior = true;
    // Per-quad floor jitter (0 = off): every quad's inclusion threshold is
    // modulated by a deterministic hash noise at a few quads' scale, so the
    // cutout boundary wiggles between quads instead of following the grid's
    // stair pattern. Grid-aligned stair steps are the strongest tell of a
    // polygon cutout; the jitter replaces them with ragged, cloud-like
    // edges at zero extra geometry.
    double floorJitter = 0.0;
    std::uint64_t floorJitterSeed = 0;
    // Frame applied to each unit direction before coverage sampling: the sky
    // deck is built in the sky dome's initial frame (its +Y on the spawn
    // up), the macro shell samples directions directly (identity).
    glm::dmat3 frame{1.0};
    // Optional sun direction (world space) for the baked directional shading
    // above; null leaves the deck's shading flat.
    const planet::Vec3 *sunDirection = nullptr;
    // Threshold-band widening for the coverage sampling (see
    // CloudField::coverage softnessScale): the coverage function thresholds
    // the fBm field, and a band this narrow maps to a spatial transition a
    // couple of grid samples wide on a shell whose cells project a dozen
    // pixels - the sampled field then reads as near-binary speckle, and no
    // amount of post-smoothing turns it back into smooth banks. Widening the
    // band spreads the same threshold over several samples, so the sampled
    // field keeps the banks' own smooth gradients.
    double coverageSoftnessScale = 1.0;
    // Optional environment lighting for an EXACT unlit bake (see
    // BackdropLighting): when set, the vertex color carries the full
    // hemispheric ambient plus the sun's Lambertian term against the shell's
    // radial normals, and the shell draws unlit. Overridden when set together
    // with sunDirection (the deck keeps its sky-dome-tinted fake term).
    const BackdropLighting *bakedLighting = nullptr;
};

// Builds the contoured shell mesh: coverage is sampled on the lat-long grid,
// smoothed, then every quad whose corner coverage clears the floor
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
    // The coverage grid is sampled RAW here, smoothed below, and only then
    // colored: the contour walk, the vertex colors and the greedy merge all
    // read the SAME smoothed field. (Coloring from the pre-smoothing samples
    // painted a quad-aligned artifact band along every bank edge - the
    // smoothing moves the inclusion crossing by up to half the edge band, so
    // quads rendered from the smoothed field carried pre-smoothing colors of
    // nearly clear sky, and vice versa.)
    std::vector<float> coverageGrid;
    coverageGrid.reserve(static_cast<std::size_t>(stride) * (spec.bands + 1u));
    // Subdivision of near-boundary quads and the width of the near-boundary
    // band in coverage units: quads whose corners sit within the band of the
    // inclusion floor are marched at kSub x kSub sub-resolution, so every
    // visual boundary lives at 1/kSub of the grid spacing and the
    // marching-squares isolines curve at sub-quad precision instead of
    // snapping to the grid stair pattern. Interior quads keep their
    // single-pair cost. A quad whose corners clear its floor by the whole
    // band joins an interior rectangle (the bilinear coverage of a merged
    // rectangle stays above its corners, so the rectangle carries no
    // boundary inside it); quads inside the band still sub-divide at the
    // contour walk, because merging them would flatten their coverage-
    // shaded colors into rectangle steps the eye reads as stair edges.
    // Boundary subdivision: the fringe COLOR blend (hazeEdgeBlend) is what
    // actually softens the deck's outlines - it fades the vertex colors to
    // the sky behind over a coverage band wider than a cell, and the
    // geometry edge hides inside the fade. That alignment holds while the
    // colors and the isoline come from the SAME smoothed field (see the
    // smoothing note above); the widened threshold band keeps the sampled
    // field smooth, so the single-cell march's linear crossings already
    // follow smooth isolines and kSub stays at the cheap interior rate.
    // Saddle cells still resolve through the cell mean below (see the
    // saddle note in the walk): their bilinear isoline is a hyperbola whose
    // center-outside case must not fill.
    constexpr std::uint32_t kSub = 1;
    constexpr float kBoundaryBand = 0.30f;

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
                planet::Vec3{fieldDirection.x, fieldDirection.y, fieldDirection.z}, spec.lowPassCutoffMeters,
                spec.coverageSoftnessScale)));
            coverageGrid.push_back(coverage);

            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(shellDirection.x * spec.shellRadiusMeters);
            vertex.position[1] = static_cast<float>(shellDirection.y * spec.shellRadiusMeters);
            vertex.position[2] = static_cast<float>(shellDirection.z * spec.shellRadiusMeters);
            vertex.normal[0] = static_cast<float>(shellDirection.x);
            vertex.normal[1] = static_cast<float>(shellDirection.y);
            vertex.normal[2] = static_cast<float>(shellDirection.z);
            vertices.push_back(vertex);
        }
    }

    // One box pass per smoothing round over the coverage grid (wrap the
    // segments; clamp the bands): rounds the quad inclusion boundary the same
    // way an alpha edge would soften a textured deck. Two rounds give the
    // contour walk a field whose curvature stays above the grid spacing, so
    // the polygons read as bank outlines instead of angular cutouts.
    // Smoothing runs BEFORE the color pass below: the contour walk, the
    // vertex colors and the greedy merge must all read the same smoothed
    // field (see the sampling note above the grid loop). Coloring from the
    // pre-smoothing samples aligns the thinning fringe with the raw field's
    // band limit - which sits at the sampling grid's Nyquist frequency, so
    // the raw samples beat against the grid - and the rendered deck then
    // shows grid-pitch stair-stepped color blocks and row-coherent sawtooth
    // fringes wherever the raw and smoothed crossings disagree.
    // The kernel reaches round(sec(latitude)) segments per row: on a lat-long
    // grid the azimuth arc shrinks with cos(latitude), so a fixed 3x3 kernel
    // covers an ever narrower physical footprint toward a shell's pole and
    // the smoothed field terraces into latitude-row plateaus. The floor
    // cutout follows the terraces - banks read as stacked planks with
    // sawtooth fringe rows - and the read worsens with the bank rows'
    // elevation, which is why eye-level views whose banks ride high in the
    // dome frame showed it while low-horizon views did not. Widening the
    // segment reach per row restores an approximately circular physical
    // kernel (the cap keeps polar rows affordable; there the azimuth cells
    // are slivers a few pixels wide, below the terrace read).
    for (std::uint32_t pass = 0; pass < std::max(1u, spec.smoothingPasses); ++pass)
    {
        std::vector<float> smoothed(coverageGrid.size());
        for (std::uint32_t band = 0; band <= spec.bands; ++band)
        {
            const double kernelLatitude = spec.latitudeLowRadians
                + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                    * (static_cast<double>(band) / static_cast<double>(spec.bands));
            const std::uint32_t segmentSpan = std::clamp(
                static_cast<std::uint32_t>(std::lround(1.0 / std::max(std::cos(kernelLatitude), 0.08))), 1u, 6u);
            const std::uint32_t bandRows[3] = {band > 0 ? band - 1u : 0u, band,
                band < spec.bands ? band + 1u : spec.bands};
            const float kernelSize = static_cast<float>(3u * (2u * segmentSpan + 1u));
            for (std::uint32_t segment = 0; segment <= spec.segments; ++segment)
            {
                float sum = 0.0f;
                for (std::uint32_t row = 0; row < 3u; ++row)
                {
                    const std::size_t rowBase = static_cast<std::size_t>(bandRows[row]) * stride;
                    for (std::int32_t offset = -static_cast<std::int32_t>(segmentSpan);
                        offset <= static_cast<std::int32_t>(segmentSpan); ++offset)
                    {
                        const std::int32_t wrapped = (static_cast<std::int32_t>(segment) + offset
                            + static_cast<std::int32_t>(spec.segments)) % static_cast<std::int32_t>(spec.segments);
                        sum += coverageGrid[rowBase + static_cast<std::size_t>(wrapped)];
                    }
                }
                smoothed[static_cast<std::size_t>(band) * stride + segment] = sum / kernelSize;
            }
        }
        coverageGrid.swap(smoothed);
    }

    // ---- color pass over the smoothed field ---------------------------------------------
    // One color per grid vertex, sampled from the SMOOTHED grid the contour
    // walk follows (see the sampling note above): coverage shades the cloud
    // color (dense banks at full albedo, thin edges dimmer - a soft read for
    // the geometry edges at shell resolution), the sun term shades the deck
    // (the caller's sun direction against the shell-local vertex direction,
    // or the exact environment bake below - banks facing the sun read
    // bright, the anti-sun side cools off, and an unlit deck without one
    // reads as flat paper cutouts), and the haze and horizon-guard blends
    // thin the fringe into the sky behind it.
    for (std::uint32_t band = 0; band <= spec.bands; ++band)
    {
        const double latitude = spec.latitudeLowRadians
            + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                * (static_cast<double>(band) / static_cast<double>(spec.bands));
        const double sinLatitude = std::sin(latitude);
        for (std::uint32_t segment = 0; segment <= spec.segments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(spec.segments);
            const glm::dvec3 shellDirection(std::cos(azimuth) * std::cos(latitude), sinLatitude,
                std::sin(azimuth) * std::cos(latitude));
            const float coverage = coverageGrid[static_cast<std::size_t>(band) * stride + segment];
            const float shade = 0.88f + 0.12f * coverage;
            float sunFacing = spec.sunDirection != nullptr
                ? static_cast<float>(0.55 + 0.45 * planet::clamp01(0.5 + 0.5
                    * planet::dot(planet::Vec3{shellDirection.x, shellDirection.y, shellDirection.z},
                        *spec.sunDirection)))
                : 1.0f;
            glm::dvec3 color(params.cloudColor[0] * shade * sunFacing,
                params.cloudColor[1] * shade * sunFacing,
                params.cloudColor[2] * shade * sunFacing);
            if (spec.bakedLighting != nullptr && spec.bakedLighting->sunEnabled)
            {
                // Exact unlit bake (see BackdropLighting): the hemispheric
                // ambient against the shell's radial normal plus the sun's
                // Lambertian term - the matte deck's whole lit response,
                // evaluated per vertex instead of per pixel.
                const double upMix = shellDirection.y * 0.5 + 0.5;
                const glm::dvec3 bakedAmbient
                    = glm::mix(spec.bakedLighting->ambientGround, spec.bakedLighting->ambientSky, upMix);
                const double sunDot = std::max(0.0, planet::dot(
                    planet::Vec3{shellDirection.x, shellDirection.y, shellDirection.z},
                    planet::Vec3{spec.bakedLighting->sunDirection.x, spec.bakedLighting->sunDirection.y,
                        spec.bakedLighting->sunDirection.z}));
                const glm::dvec3 light = bakedAmbient
                    + glm::dvec3(spec.bakedLighting->sunRadianceOverPi.x, spec.bakedLighting->sunRadianceOverPi.y,
                        spec.bakedLighting->sunRadianceOverPi.z)
                        * sunDot;
                color = glm::dvec3(params.cloudColor[0] * shade * light.x,
                    params.cloudColor[1] * shade * light.y,
                    params.cloudColor[2] * shade * light.z);
            }
            const planet::Vec3 sky = planet::skyGradientColor(params, sinLatitude);
            // Guard position of this vertex (0 at the guard latitude, 1 above
            // the fade band) drives the guard's two thinning effects below.
            const double guardFadeT = spec.guardFadeRadians > 0.0 && spec.minQuadLatitudeRadians > -3.0
                ? planet::clamp01((latitude - spec.minQuadLatitudeRadians) / spec.guardFadeRadians)
                : 1.0;
            if (spec.hazeEdgeBlend)
            {
                // The thinning fringe runs over the configured haze width; a
                // width past the contour walk's boundary band would keep
                // blending where the merge operates, so it clamps to the
                // band. Decks with a guard-floor lift WIDEN the fringe
                // through the guard band: the lift leaves small dense
                // fragments there, and without the wider fringe they render
                // as hard-edged blades instead of haze-veiled wisps.
                double hazeWidth = std::min(spec.hazeEdgeWidth, static_cast<double>(kBoundaryBand));
                if (spec.guardFloorLift > 0.0)
                {
                    hazeWidth += (1.0 - guardFadeT) * 0.45;
                }
                const double blend = planet::smoothstep(spec.coverageFloor,
                    std::min(1.0, spec.coverageFloor + hazeWidth), static_cast<double>(coverage));
                color = glm::mix(glm::dvec3(sky.x, sky.y, sky.z), color, blend);
            }
            if (spec.guardFadeRadians > 0.0 && spec.minQuadLatitudeRadians > -3.0)
            {
                // Thin the deck out toward the horizon guard: the color fades
                // into the sky behind it over the guard band, matching the
                // haze blend's thinning read.
                const double guardBlend = planet::smoothstep(spec.minQuadLatitudeRadians,
                    spec.minQuadLatitudeRadians + spec.guardFadeRadians, latitude);
                color = glm::mix(glm::dvec3(sky.x, sky.y, sky.z), color, guardBlend);
            }
            planet::PlanetVertex &vertex = vertices[static_cast<std::size_t>(band) * stride + segment];
            vertex.color[0] = static_cast<float>(color.x);
            vertex.color[1] = static_cast<float>(color.y);
            vertex.color[2] = static_cast<float>(color.z);
        }
    }

    const auto coverageAt = [&coverageGrid, stride](std::uint32_t band, std::uint32_t segment)
    {
        return coverageGrid[static_cast<std::size_t>(band) * stride + segment];
    };

    // Per-quad threshold noise: a deterministic hash of the quad index mixed
    // into [0, 1) (see spec.floorJitter).
    const auto quadFloor = [&spec](std::uint32_t band, std::uint32_t segment)
    {
        if (spec.floorJitter <= 0.0)
        {
            return spec.coverageFloor;
        }
        std::uint64_t mixed = static_cast<std::uint64_t>(segment) * 0x9E3779B97F4A7C15ull
            ^ static_cast<std::uint64_t>(band) * 0xC2B2AE3D27D4EB4Full
            ^ spec.floorJitterSeed;
        mixed ^= mixed >> 33;
        mixed *= 0xFF51AFD7ED558CCDull;
        mixed ^= mixed >> 33;
        const double unit = static_cast<double>(mixed >> 11) / 9007199254740992.0;
        return spec.coverageFloor * (1.0 - spec.floorJitter) + spec.coverageFloor * 2.0 * spec.floorJitter * unit;
    };

    // Linear interpolation of the floor crossing along one grid edge (shared
    // by the coarse boundary walk and the sub-cell walk below).
    const auto crossingT = [](float coverageA, float coverageB, float floor) -> float
    {
        const float denominator = coverageA - coverageB;
        return std::abs(denominator) < 1.0e-6f
            ? 0.5f
            : std::clamp((coverageA - floor) / denominator, 0.0f, 1.0f);
    };

    // One interpolated floor-crossing vertex between two corner slots (the
    // saddle pockets and the ring walk below share it): position, normal and
    // color blend at the crossing parameter, so a crossing pushed per cell
    // still lands on the same point its neighbor computes for the shared
    // edge and the emitted shapes weld geometrically.
    const auto crossingVertex = [&vertices, &crossingT](std::uint32_t slotA, std::uint32_t slotB,
        float coverageA, float coverageB, float floor) -> std::uint32_t
    {
        const planet::PlanetVertex &vertexA = vertices[slotA];
        const planet::PlanetVertex &vertexB = vertices[slotB];
        const float t = crossingT(coverageA, coverageB, floor);
        planet::PlanetVertex vertex{};
        for (std::uint32_t k = 0; k < 3; ++k)
        {
            vertex.position[k] = vertexA.position[k] + (vertexB.position[k] - vertexA.position[k]) * t;
            vertex.normal[k] = vertexA.normal[k] + (vertexB.normal[k] - vertexA.normal[k]) * t;
            vertex.color[k] = vertexA.color[k] + (vertexB.color[k] - vertexA.color[k]) * t;
        }
        vertices.push_back(vertex);
        return static_cast<std::uint32_t>(vertices.size() - 1u);
    };

    // ---- per-quad inclusion floors ----------------------------------------------------
    // Every quad's inclusion threshold (the base floor plus the ragged-edge
    // wiggle, neighbor-gated) is computed once in a pre-pass: the greedy
    // interior merge below and the contour walk share the same values, so a
    // merged rectangle and the per-quad walk agree about what is inside.
    // Quad-indexed arrays stride by SPEC.SEGMENTS (quads per row); the
    // coverage grid itself strides by SEGMENTS+1 (grid corners per row).
    const std::size_t quadStride = spec.segments;
    const std::size_t quadSlotCount = static_cast<std::size_t>(spec.bands) * quadStride;
    std::vector<float> quadFloors(quadSlotCount, static_cast<float>(spec.coverageFloor));
    for (std::uint32_t band = 0; band < spec.bands; ++band)
    {
        // Guard-band floor lift (see CoverageShellSpec::guardFloorLift):
        // quads inside the fade band raise their threshold with the fade
        // position, thinning the banks into fragments near the horizon.
        double guardLift = 0.0;
        if (spec.guardFloorLift > 0.0 && spec.guardFadeRadians > 0.0 && spec.minQuadLatitudeRadians > -3.0)
        {
            const double quadLatitude = spec.latitudeLowRadians
                + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                    * (static_cast<double>(band) / static_cast<double>(spec.bands));
            const double fadeT = planet::clamp01(
                (quadLatitude - spec.minQuadLatitudeRadians) / spec.guardFadeRadians);
            guardLift = (1.0 - fadeT) * spec.guardFloorLift;
        }
        for (std::uint32_t segment = 0; segment < spec.segments; ++segment)
        {
            float floorThisQuad = static_cast<float>(quadFloor(band, segment) + guardLift);
            if (floorThisQuad < static_cast<float>(spec.coverageFloor))
            {
                // Neighbor gate on the threshold wiggle: a quad may clear a
                // LOWERED floor only while its neighborhood genuinely clears
                // the base floor. A sub-floor bump that keeps its threshold
                // lowered with no solid neighbor around renders as a lone
                // speck floating in clear sky; restoring the base floor there
                // keeps the ragged bank edges (their neighbors are solid) and
                // drops the isolated fragments.
                const float baseFloor = static_cast<float>(spec.coverageFloor);
                bool neighborGround = false;
                // Valid quad rows are [0, bands), valid columns [0, segments);
                // clamp the 3x3 neighborhood to them (the grid's own corner
                // rows/cols belong to quad corners, not quads).
                const std::uint32_t bandLow = band > 0u ? band - 1u : 0u;
                const std::uint32_t bandHigh = band + 2u < spec.bands ? band + 1u : spec.bands - 1u;
                const std::uint32_t segmentLow = segment > 0u ? segment - 1u : 0u;
                const std::uint32_t segmentHigh = segment + 2u < spec.segments ? segment + 1u : spec.segments - 1u;
                for (std::uint32_t neighborBand = bandLow; neighborBand <= bandHigh && !neighborGround; ++neighborBand)
                {
                    for (std::uint32_t neighborSegment = segmentLow;
                        neighborSegment <= segmentHigh && !neighborGround; ++neighborSegment)
                    {
                        if (neighborBand == band && neighborSegment == segment)
                        {
                            continue;
                        }
                        const std::uint32_t nl = neighborBand * stride + neighborSegment;
                        neighborGround = coverageGrid[nl] >= baseFloor || coverageGrid[nl + 1u] >= baseFloor
                            || coverageGrid[nl + stride] >= baseFloor || coverageGrid[nl + stride + 1u] >= baseFloor;
                    }
                }
                if (!neighborGround)
                {
                    floorThisQuad = baseFloor;
                }
            }
            quadFloors[static_cast<std::size_t>(band) * quadStride + segment] = floorThisQuad;
        }
    }

    // ---- greedy interior merge ---------------------------------------------------------
    // A quad is "solid interior" when all four corners clear its floor by
    // the whole boundary band above the BASE floor (the interior shade
    // saturates there, so the rectangle renders like the quads it absorbs).
    // Banks are mostly interior, so runs of
    // solid quads merge into maximal rectangles (a greedy row run extended
    // downward while the whole run stays solid): the rectangle emits the
    // same six indices its quads would have - as one quad over the
    // rectangle's grid corners - so a filled bank interior costs two
    // triangles per rectangle instead of a subdivided quad per quad, while
    // every boundary quad keeps the contour walk's sub-quad precision.
    // Bilinear coverage inside a rectangle is a convex combination of its
    // corners, so a rectangle whose corners are all above the floor carries
    // no boundary inside it. Rows below the
    // guard latitude never merge: the contour walk skips them outright.
    std::vector<std::uint8_t> rowAllowed(spec.bands, 0);
    for (std::uint32_t band = 0; band < spec.bands; ++band)
    {
        const double quadLatitude = spec.latitudeLowRadians
            + (spec.latitudeHighRadians - spec.latitudeLowRadians)
                * (static_cast<double>(band) / static_cast<double>(spec.bands));
        // Rows inside the horizon-guard fade never merge: the fade blends
        // per-vertex toward the sky behind the deck, and a rectangle
        // spanning it would render the fade as visible steps.
        rowAllowed[band] = quadLatitude >= spec.minQuadLatitudeRadians + spec.guardFadeRadians ? 1u : 0u;
    }
    std::vector<std::uint8_t> claimedQuad(quadSlotCount, 0);
    // Plain inclusion of one quad against its own (jittered) floor - the
    // per-quad walk's insideCount == 4 test.
    const auto quadFullyInside = [&](std::uint32_t band, std::uint32_t segment) -> bool
    {
        const float floor = quadFloors[static_cast<std::size_t>(band) * quadStride + segment];
        return coverageAt(band, segment) >= floor && coverageAt(band, segment + 1u) >= floor
            && coverageAt(band + 1u, segment + 1u) >= floor && coverageAt(band + 1u, segment) >= floor;
    };
    const auto quadSolidInterior = [&](std::uint32_t band, std::uint32_t segment) -> bool
    {
        // A rectangle may only replace quads STRICTLY INSIDE a bank, with
        // every 8-neighborhood quad also fully solid: the rectangle's edge
        // is one straight grid-aligned segment, and only the deep interior
        // has no visible boundary to straighten. The banks' ragged outer
        // boundary - each quad's own jittered floor wiggling the cutout -
        // then stays entirely with the per-quad contour walk, where it
        // belongs. (The raw-grid color bar is gone: the shade and the haze
        // blend are continuous functions of coverage, and the merge is now
        // confined to the deep interior where their gradient across a
        // rectangle is far below one encode step.)
        if (!quadFullyInside(band, segment))
        {
            return false;
        }
        const std::uint32_t bandLow = band > 0u ? band - 1u : 0u;
        const std::uint32_t bandHigh = band + 2u < spec.bands ? band + 1u : spec.bands - 1u;
        const std::uint32_t segmentLow = segment > 0u ? segment - 1u : 0u;
        const std::uint32_t segmentHigh = segment + 2u < spec.segments ? segment + 1u : spec.segments - 1u;
        for (std::uint32_t neighborBand = bandLow; neighborBand <= bandHigh; ++neighborBand)
        {
            for (std::uint32_t neighborSegment = segmentLow; neighborSegment <= segmentHigh; ++neighborSegment)
            {
                if (neighborBand == band && neighborSegment == segment)
                {
                    continue;
                }
                if (!quadFullyInside(neighborBand, neighborSegment))
                {
                    return false;
                }
            }
        }
        return true;
    };
    for (std::uint32_t band = 0; spec.mergeInterior && band < spec.bands; ++band)
    {
        if (!rowAllowed[band])
        {
            continue; // below the guard latitude (the horizon rim, for sky decks)
        }
        for (std::uint32_t segment = 0; segment < spec.segments; ++segment)
        {
            if (claimedQuad[static_cast<std::size_t>(band) * quadStride + segment]
                || !quadSolidInterior(band, segment))
            {
                continue;
            }
            // Extend the row run rightward, then grow the rectangle down
            // while every quad of the run in the next row is solid too.
            std::uint32_t runEnd = segment + 1u;
            while (runEnd < spec.segments && quadSolidInterior(band, runEnd))
            {
                ++runEnd;
            }
            std::uint32_t runBottom = band + 1u;
            const auto runSolid = [&quadSolidInterior, &rowAllowed, segment, runEnd](std::uint32_t row)
            {
                if (rowAllowed[row] == 0u)
                {
                    return false;
                }
                for (std::uint32_t column = segment; column < runEnd; ++column)
                {
                    if (!quadSolidInterior(row, column))
                    {
                        return false;
                    }
                }
                return true;
            };
            while (runBottom < spec.bands && runSolid(runBottom))
            {
                ++runBottom;
            }
            for (std::uint32_t row = band; row < runBottom; ++row)
            {
                for (std::uint32_t column = segment; column < runEnd; ++column)
                {
                    claimedQuad[static_cast<std::size_t>(row) * quadStride + column] = 1u;
                }
            }
            const std::uint32_t rectTopLeft = band * stride + segment;
            const std::uint32_t rectTopRight = band * stride + runEnd;
            const std::uint32_t rectBottomRight = runBottom * stride + runEnd;
            const std::uint32_t rectBottomLeft = runBottom * stride + segment;
            indices.insert(indices.end(), {rectTopLeft, rectTopRight, rectBottomRight, rectTopLeft, rectBottomRight,
                rectBottomLeft});
            segment = runEnd - 1u;
        }
    }

    // ---- contour walk over the remaining quads ------------------------------------------
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
            if (claimedQuad[static_cast<std::size_t>(band) * quadStride + segment])
            {
                continue; // already part of a merged interior rectangle
            }
            const std::uint32_t tl = band * stride + segment;
            const std::uint32_t tr = tl + 1u;
            const std::uint32_t br = tl + stride + 1u;
            const std::uint32_t bl = tl + stride;
            const float cornerCoverage[4] = {coverageAt(band, segment), coverageAt(band, segment + 1u),
                coverageAt(band + 1u, segment + 1u), coverageAt(band + 1u, segment)};
            const float floorThisQuad = quadFloors[static_cast<std::size_t>(band) * quadStride + segment];
            bool inside[4];
            std::uint32_t insideCount = 0;
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                inside[corner] = cornerCoverage[corner] >= floorThisQuad;
                insideCount += inside[corner] ? 1u : 0u;
            }
            if (insideCount == 0)
            {
                continue; // clear sky: no geometry, the surface shows through
            }
            if (insideCount == 4)
            {
                // Interior quads need one more test before the cheap full
                // quad: a quad whose corners all clear the floor can still
                // DIP below it inside (the bilinear saddle between its
                // corners), and an outer boundary that snaps to the quad
                // grid reads as stair steps against its cut-out neighbors.
                // Quads fully above the floor + boundary band take the fast
                // path; the rest are marched at kSub x kSub sub-resolution,
                // where the isoline curves at sub-quad precision.
                bool nearBoundary = false;
                for (std::uint32_t corner = 0; corner < 4 && !nearBoundary; ++corner)
                {
                    nearBoundary = cornerCoverage[corner] < floorThisQuad + kBoundaryBand;
                }
                if (!nearBoundary)
                {
                    indices.insert(indices.end(), {tl, tr, br, tl, br, bl});
                    continue;
                }
                // fall through to the sub-cell march below
            }
            else
            {
                // PARTIAL quad (the bank's outer boundary). The coarse
                // boundary walk interpolated its crossings along the grid
                // edges but still snapped the polygon to whole grid
                // vertices wherever a corner sat inside, so every bank
                // outline carried a grid-pitch staircase (one deck cell is
                // about ten pixels at eye level). The sub-cell march below
                // resolves partial coverage natively - the bilinear corners
                // feed (kSub+1)^2 sub-samples and each sub-cell walks its
                // own crossings - so boundary quads take it too, and the
                // isoline curves at sub-cell precision on every boundary.
                // Deep-interior saddles (all four corners well above the
                // floor, only the bilinear middle dipping) cannot reach
                // here: a partial quad has at least one corner below the
                // floor by construction.
            }

            // Boundary or fully-inside near-boundary quad: subdivide into kSub x kSub
            // cells (bilinear coverage + corner colors over the quad) and
            // march the cells. Sub-cell coverage values are shared arithmetic
            // between neighboring sub-cells (bilinear from the same corners),
            // so edges weld; the sub-resolution pushes the polygonal boundary
            // below one pixel of quantization at capture scales.
            float subCoverage[kSub + 1][kSub + 1];
            {
                for (std::uint32_t row = 0; row <= kSub; ++row)
                {
                    const float v = static_cast<float>(row) / static_cast<float>(kSub);
                    const float top = cornerCoverage[0] + (cornerCoverage[1] - cornerCoverage[0]) * v;
                    const float bottom = cornerCoverage[3] + (cornerCoverage[2] - cornerCoverage[3]) * v;
                    for (std::uint32_t column = 0; column <= kSub; ++column)
                    {
                        const float u = static_cast<float>(column) / static_cast<float>(kSub);
                        subCoverage[row][column] = top + (bottom - top) * u;
                    }
                }
            }
            // Sub-cell vertex slots: -2 = not needed yet, >= 0 = vertex slot
            // in the vertex buffer. The sub-grid's corner vertices reuse the
            // coarse quad's grid vertices when they coincide, and create
            // fresh (bilinear) vertices for the interior lattice points -
            // their colors come from the same bilinear blend as the coverage,
            // so shading stays continuous with the surrounding quads.
            const auto subVertex = [&vertices, &tl, &tr, &br, &bl](
                std::uint32_t row, std::uint32_t column) -> std::int32_t
            {
                if (row == 0 && column == 0) return static_cast<std::int32_t>(tl);
                if (row == 0 && column == kSub) return static_cast<std::int32_t>(tr);
                if (row == kSub && column == kSub) return static_cast<std::int32_t>(br);
                if (row == kSub && column == 0) return static_cast<std::int32_t>(bl);
                const float u = static_cast<float>(column) / static_cast<float>(kSub);
                const float v = static_cast<float>(row) / static_cast<float>(kSub);
                const planet::PlanetVertex &topLeft = vertices[tl];
                const planet::PlanetVertex &topRight = vertices[tr];
                const planet::PlanetVertex &bottomRight = vertices[br];
                const planet::PlanetVertex &bottomLeft = vertices[bl];
                planet::PlanetVertex vertex{};
                for (std::uint32_t k = 0; k < 3; ++k)
                {
                    const float top = topLeft.position[k] + (topRight.position[k] - topLeft.position[k]) * u;
                    const float bottom = bottomLeft.position[k] + (bottomRight.position[k] - bottomLeft.position[k]) * u;
                    vertex.position[k] = top + (bottom - top) * v;
                    const float normalTop = topLeft.normal[k] + (topRight.normal[k] - topLeft.normal[k]) * u;
                    const float normalBottom = bottomLeft.normal[k] + (bottomRight.normal[k] - bottomLeft.normal[k]) * u;
                    vertex.normal[k] = normalTop + (normalBottom - normalTop) * v;
                    const float colorTop = topLeft.color[k] + (topRight.color[k] - topLeft.color[k]) * u;
                    const float colorBottom = bottomLeft.color[k] + (bottomRight.color[k] - bottomLeft.color[k]) * u;
                    vertex.color[k] = colorTop + (colorBottom - colorTop) * v;
                }
                vertices.push_back(vertex);
                return static_cast<std::int32_t>(vertices.size() - 1u);
            };
            std::int32_t subSlots[kSub + 1][kSub + 1];
            for (std::uint32_t row = 0; row <= kSub; ++row)
            {
                for (std::uint32_t column = 0; column <= kSub; ++column)
                {
                    subSlots[row][column] = -2;
                }
            }
            for (std::uint32_t row = 0; row < kSub; ++row)
            {
                for (std::uint32_t column = 0; column < kSub; ++column)
                {
                    const float cellFloor = floorThisQuad;
                    const float cellCorner[4] = {subCoverage[row][column], subCoverage[row][column + 1u],
                        subCoverage[row + 1u][column + 1u], subCoverage[row + 1u][column]};
                    bool cellInside[4];
                    std::uint32_t cellInsideCount = 0;
                    for (std::uint32_t corner = 0; corner < 4; ++corner)
                    {
                        cellInside[corner] = cellCorner[corner] >= cellFloor;
                        cellInsideCount += cellInside[corner] ? 1u : 0u;
                    }
                    if (cellInsideCount == 0)
                    {
                        continue;
                    }
                    const std::uint32_t cornerRow[4] = {row, row, row + 1u, row + 1u};
                    const std::uint32_t cornerColumn[4] = {column, column + 1u, column + 1u, column};
                    std::int32_t cellSlot[4];
                    for (std::uint32_t corner = 0; corner < 4; ++corner)
                    {
                        if (subSlots[cornerRow[corner]][cornerColumn[corner]] == -2)
                        {
                            subSlots[cornerRow[corner]][cornerColumn[corner]]
                                = subVertex(cornerRow[corner], cornerColumn[corner]);
                        }
                        cellSlot[corner] = subSlots[cornerRow[corner]][cornerColumn[corner]];
                    }
                    if (cellInsideCount == 4)
                    {
                        indices.insert(indices.end(), {static_cast<std::uint32_t>(cellSlot[0]),
                            static_cast<std::uint32_t>(cellSlot[1]), static_cast<std::uint32_t>(cellSlot[2]),
                            static_cast<std::uint32_t>(cellSlot[0]), static_cast<std::uint32_t>(cellSlot[2]),
                            static_cast<std::uint32_t>(cellSlot[3])});
                        continue;
                    }
                    // Saddle resolution (see the builder's header comment):
                    // with two OPPOSITE corners inside, the ring walk below
                    // emits one hexagon fanned from the first corner. That
                    // hexagon is the correct region only when the bilinear
                    // cell center sits INSIDE the floor (the inside band
                    // connects through the middle and the hexagon stays
                    // convex). When the center sits OUTSIDE, the two inside
                    // corners own disconnected triangle pockets, and the
                    // hexagon additionally fills the clear-sky center: the
                    // fan paints cloud-colored polygons past the isoline
                    // into the sky while clipping the pockets' outer halves.
                    // Whether a quad saddles center-inside or center-outside
                    // follows the bank edges' phase against the sample grid,
                    // so the unresolved case surfaced as rows of sawtooth
                    // triangles along stretched, row-aligned bank boundaries.
                    // Resolve by the cell mean: center inside keeps the
                    // hexagon walk, center outside emits each inside
                    // corner's own pocket triangle.
                    const bool saddle = cellInsideCount == 2 && cellInside[0] == cellInside[2]
                        && cellInside[1] == cellInside[3];
                    const float cellCenterCoverage
                        = 0.25f * (cellCorner[0] + cellCorner[1] + cellCorner[2] + cellCorner[3]);
                    if (saddle && cellCenterCoverage < cellFloor)
                    {
                        for (std::uint32_t corner = 0; corner < 4; ++corner)
                        {
                            if (!cellInside[corner])
                            {
                                continue;
                            }
                            const std::uint32_t previous = (corner + 3u) % 4u;
                            const std::uint32_t next = (corner + 1u) % 4u;
                            std::uint32_t pocket[3];
                            pocket[0] = static_cast<std::uint32_t>(cellSlot[corner]);
                            pocket[1] = crossingVertex(cellSlot[previous], cellSlot[corner],
                                cellCorner[previous], cellCorner[corner], cellFloor);
                            pocket[2] = crossingVertex(cellSlot[corner], cellSlot[next],
                                cellCorner[corner], cellCorner[next], cellFloor);
                            indices.insert(indices.end(), {pocket[0], pocket[1], pocket[2]});
                        }
                        continue;
                    }
                    // Boundary walk over the sub-cell; interpolated crossings
                    // reuse the same arithmetic so adjacent sub-cells weld.
                    std::uint32_t polygon[8];
                    std::uint32_t polygonCount = 0;
                    for (std::uint32_t corner = 0; corner < 4; ++corner)
                    {
                        const std::uint32_t next = (corner + 1u) % 4u;
                        const bool straddles = cellInside[corner] != cellInside[next];
                        if (cellInside[corner])
                        {
                            polygon[polygonCount++] = static_cast<std::uint32_t>(cellSlot[corner]);
                        }
                        if (straddles)
                        {
                            polygon[polygonCount++] = crossingVertex(cellSlot[corner], cellSlot[next],
                                cellCorner[corner], cellCorner[next], cellFloor);
                        }
                    }
                    for (std::uint32_t fan = 1; fan + 1 < polygonCount; ++fan)
                    {
                        indices.insert(indices.end(), {polygon[0], polygon[fan], polygon[fan + 1]});
                    }
                }
            }
        }
    }
    return !indices.empty();
}

// Builds a radial disc mesh on the unit ball around +Z: a triangle fan of
// `bands` rings whose vertex colors fall from the bright core to zero at the
// rim. The material's per-frame baseColor tints the whole disc (the sun
// tint); the vertex colors carry the core-to-halo falloff.
void buildSunDiscMesh(std::vector<planet::PlanetVertex> &vertices, std::vector<std::uint32_t> &indices,
    std::uint32_t segments, std::uint32_t bands)
{
    vertices.reserve(1u + segments * bands);
    indices.reserve(segments * bands * 6u);
    // Ring radii and brightness: the core (inner ~55%) stays at full
    // brightness, the halo falls smoothly to zero at the rim.
    const auto ringBrightness = [bands](std::uint32_t band)
    {
        const double t = static_cast<double>(band) / static_cast<double>(bands);
        return std::pow(1.0 - t, 1.7);
    };
    auto pushVertex = [&vertices](const glm::dvec2 &direction, double radius, double brightness)
    {
        planet::PlanetVertex vertex{};
        vertex.position[0] = static_cast<float>(direction.x * radius);
        vertex.position[1] = static_cast<float>(direction.y * radius);
        vertex.position[2] = 0.0f;
        vertex.normal[0]   = 0.0f;
        vertex.normal[1]   = 0.0f;
        vertex.normal[2]   = 1.0f;
        vertex.color[0]    = static_cast<float>(brightness);
        vertex.color[1]    = static_cast<float>(brightness);
        vertex.color[2]    = static_cast<float>(brightness);
        vertices.push_back(vertex);
        return static_cast<std::uint32_t>(vertices.size() - 1u);
    };
    const std::uint32_t center = pushVertex(glm::dvec2{0.0, 0.0}, 0.0, 1.0);
    std::uint32_t previousRingStart = center;
    bool previousIsCenter = true;
    for (std::uint32_t band = 1; band <= bands; ++band)
    {
        const double radius = static_cast<double>(band) / static_cast<double>(bands);
        const double brightness = ringBrightness(band);
        std::uint32_t ringStart = 0;
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const double azimuth = glm::two_pi<double>() * static_cast<double>(segment) / static_cast<double>(segments);
            const std::uint32_t current
                = pushVertex(glm::dvec2{std::cos(azimuth), std::sin(azimuth)}, radius, brightness);
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
    double rockTextureMaxDistanceMeters)
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
    frameBackdropMatrices_.reserve(8);
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
            const bool groundTextured
                = !groundTextureVariants_.empty() && draw.distanceSquared <= groundTextureMaxDistanceMetersSquared_;
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
            // Subtle tangent-space relief: the headless backend rebuilds the
            // tangent frame per triangle from the tiled UVs, so the value
            // stays small to keep the frame continuous where the projection
            // switches axes.
            material.normalScale = 0.35f;
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
    if (sunDiscActive_ && sunDisc_ != renderer::invalidMesh && !sunDiscMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = sunDisc_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[4]);
        command.material = &sunDiscMaterials_.front();
        drawCommands_.push_back(command);
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
    if (skyDomeActive_ && skyDome_ != renderer::invalidMesh && !skyDomeMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = skyDome_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[6]);
        command.material = &skyDomeMaterials_.front();
        drawCommands_.push_back(command);
    }
    // The aurora is translucent (opacity < 1): the backend routes it into
    // the transparent pass regardless of its list position, alpha-blending
    // it over the finished opaque scene. Emitted last so the command list
    // reads far-to-near inside its own class (the rings sit behind
    // everything but the star dome).
    if (auroraActive_ && aurora_ != renderer::invalidMesh && !auroraMaterials_.empty())
    {
        renderer::RenderDrawCommand command{};
        command.mesh = aurora_;
        command.modelMatrix = glm::value_ptr(frameBackdropMatrices_[7]);
        command.material = &auroraMaterials_.front();
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

void PlanetChunkRenderer::ensureCloudLayer()
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0 || renderer_ == nullptr
        || rendererHandle_ == renderer::invalidRenderer || cloudLayer_ != renderer::invalidMesh)
    {
        return;
    }

    // Lat-long grid on the cloud shell (datum radius + cloud altitude).
    // 384 x 192 quads keep the vertex spacing (~1 km at the equator of the
    // 60 km showcase) well under the configured cloud wavelength, so the
    // shell resolves the banks while the coverage field's band limit (the
    // spacing is passed as the cutoff) keeps sub-spacing octaves out. The
    // greedy interior merge (see buildCoverageShellMesh) collapses the
    // banks' solid middles, so the triangle count stays near the contoured
    // boundary's own cost.
    constexpr std::uint32_t kSegments = 384;
    constexpr std::uint32_t kBands = 192;
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
    // The contour walk interpolates the grid linearly, so the field it
    // follows must not carry content the grid cannot represent: the band
    // limit sits at twice the vertex spacing (content at four spacings
    // renders at full strength, anything below two spacings fades out), and
    // three grid-smoothing rounds round the inclusion boundary. Together
    // they keep the bank outlines curved at the grid scale instead of
    // angular.
    spec.lowPassCutoffMeters = 2.0 * vertexSpacing;
    spec.coverageFloor = kCoverageFloor;
    spec.smoothingPasses = 3;
    // Ragged bank edges: the per-quad threshold wiggle breaks the grid stair
    // pattern of the cutout (see CoverageShellSpec::floorJitter).
    spec.floorJitter = 0.25;
    spec.floorJitterSeed = 0xC10D5C0Full;
    // Thinning fringes stay OFF on the macro shell: the edge blend fades
    // bank rims toward the SKY gradient, which over the globe reads as a
    // cyan bleed wherever ocean sits behind the bank (the blend cannot know
    // the backdrop). Orbit distance resolves the contour walk's smoothed,
    // jittered outlines on their own; the sky deck keeps the blend (see
    // ensureSkyCloudLayer), where the sky genuinely is what sits behind the
    // banks.
    spec.hazeEdgeBlend = false;
    // The shell bakes the environment lighting per vertex and draws unlit
    // (see BackdropLighting): a matte white deck's lit response has no
    // visible specular, so the per-pixel PBR evaluation over the disk-sized
    // shell only cost fill rate.
    if (backdropLightingValid_)
    {
        spec.bakedLighting = &backdropLighting_;
    }

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
    // The vertex colors carry the environment bake (see the spec above); the
    // material draws unlit with the coverage-shaded deck tones. Without a
    // bake (no pbr environment) the deck falls back to the lit draw.
    material.unlit = backdropLightingValid_ && backdropLighting_.sunEnabled;
    material.roughnessFactor = 0.9f; // matte decks; the sun does the shading
    // The shell is an outward sphere seen from outside at every view that
    // draws it (globe and orbit keep the eye above the shell): declaring the
    // winding culls the far hemisphere per triangle instead of letting its
    // bounding boxes scan the framebuffer against the depth buffer.
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
    constexpr std::uint32_t kSegments = 96;
    constexpr std::uint32_t kBands = 22;
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
    // Fade exponent: 1.25 keeps a dense glow past half the ring and eases
    // out to zero exactly at the annulus rim, so the ring's outer boundary
    // dissolves into space over the last stretch instead of ending in a
    // perceptible step (a steeper exponent steepens the final approach into
    // a hard edge).
    constexpr double kLimbFadeExponent = 1.25;
    // Day/night azimuth falloff: full glow toward the sun, a distinct faint
    // airglow residue on the night side. The floor is high enough that the
    // night limb still carries a visible thin ring - from space a real
    // atmosphere stays limb-bright all around the disc (airglow plus
    // scattered starlight) while glowing on the sun side; a near-zero floor
    // made the night limb's ring vanish, which read as a broken/asymmetric
    // halo instead of a lit planet.
    constexpr double kLimbNightFloor = 0.22;
    // Sunset warmth: the limb ring glows warm where the terminator crosses
    // it (the sun grazes the atmosphere there, so the light path reddens the
    // way sunrise/sunset light does). The warm band rides the azimuthal
    // gradient - azimuth 0 is the sunward limb (see updateAtmosphereShell),
    // so the warmth peaks where cos(azimuth) crosses zero and melts back to
    // the plain horizon hue well inside the day side. The strength stays a
    // minority share of the horizon hue: a full-strength warm ring reads as
    // a painted salmon halo from orbit, while a subtle one reads as sunset
    // light on the thin air the ring depicts.
    const planet::Vec3 warmTint{1.0, 0.52, 0.26};
    constexpr double kWarmStrength = 0.35;
    constexpr double kWarmBandWidth = 0.35;

    std::vector<planet::PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve((kSegments + 1u) * (kBands + 1u));
    indices.reserve(kSegments * kBands * 6u);
    const planet::Vec3 limbColor = planet::skyGradientColor(params_, 0.0); // the horizon band color
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
            const double terminatorBand = 1.0 - planet::smoothstep(0.0, kWarmBandWidth, std::abs(std::cos(azimuth)));
            const planet::Vec3 limbTint{
                planet::mix(limbColor.x, warmTint.x, kWarmStrength * terminatorBand),
                planet::mix(limbColor.y, warmTint.y, kWarmStrength * terminatorBand),
                planet::mix(limbColor.z, warmTint.z, kWarmStrength * terminatorBand)};
            planet::PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(direction.x);
            vertex.position[1] = static_cast<float>(direction.y);
            vertex.position[2] = static_cast<float>(direction.z);
            vertex.color[0] = static_cast<float>(limbTint.x * fade * day);
            vertex.color[1] = static_cast<float>(limbTint.y * fade * day);
            vertex.color[2] = static_cast<float>(limbTint.z * fade * day);
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
    // the limb glow's visual radius: a quarter of the configured air column
    // above the surface, clamped to 2-8% of the radius so the band hugs the
    // horizon the way atmospheric density does instead of inflating with a
    // deep showcase atmosphere. Translate to the planet's world center.
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
    const double glowHeight = glm::clamp(0.25 * std::max(1.0, params_.atmosphereHeightMeters),
        0.02 * params_.radiusMeters, 0.08 * params_.radiusMeters);
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
        // Linear RGB of the glow at full band weight: a dim warm wash that
        // displays in the high-30s after the shared tone mapping.
        constexpr double kGlowTone[3] = {3.0, 2.7, 2.1};
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
    // spans ~1.7 degrees (aerial glow around the disc); in space it tightens
    // to ~0.6 degrees (the sun is a hard point with a tight glare - a wide
    // halo reads as a giant blob on space views). The core itself stays near
    // the sun's true ~0.5 degree diameter either way.
    constexpr std::uint32_t kSegments = 48;
    constexpr std::uint32_t kBands = 4;
    sunDiscAngularRadiusRadians_ = 0.030; // in-air halo; space views shrink it below

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
    const float discBrightness = inAir ? 5.0f : 8.0f;
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

void PlanetChunkRenderer::ensureSkyCloudLayer(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection)
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
    // the haze band; the guard band itself FADES the deck into the sky
    // gradient (see buildCoverageShellMesh), so the lower boundary reads as
    // thinning cloud instead of a straight cut. The coverage field is
    // sampled along the frame-rotated directions (from the planet center,
    // which for a standing eye differs from the eye-local sky direction by
    // well under the field's feature size).
    // Grid resolution: fine enough that the deck's contoured edges stay
    // curved at eye level - the deck spans the whole upper hemisphere, and a
    // standing camera resolves roughly half a degree of azimuth per pixel
    // pair at 1920x1080; the 0.56-degree cells (with the contour walk's 6x6
    // subdivision of near-boundary quads) keep bank outlines smooth past
    // that scale. Coarser grids terrace the smoothed coverage into grid-row
    // plateaus, and the floor cutout follows the terraces: banks read as
    // stacked horizontal planks. The grid is ISOTROPIC in angle - segments
    // per degree of azimuth equal bands per degree of latitude
    // (640/360 ~ 160/90): an anisotropic grid smears the banks along the
    // azimuth rows through the coverage smoothing.
    constexpr std::uint32_t kSegments = 640;
    constexpr std::uint32_t kBands = 160;
    // The deck's coverage floor sits BELOW the visual cloud edge on purpose:
    // quads between the floor and the floor + haze width carry the thinning
    // fringe (their color fades from the sky gradient into the cloud tone),
    // so the deck dissolves into the sky instead of cutting off at a polygon
    // boundary. The opaque cutout then happens where quads are already
    // nearly sky-colored and its silhouette cannot read. With the widened
    // threshold band (see coverageSoftnessScale) the sampled field ramps
    // smoothly across this band, so the fringe's low end stays connected to
    // the bank bodies instead of shedding isolated flecks.
    constexpr double kCoverageFloor = 0.22;
    // The guard spans the low sky band where the deck's grid cells project
    // at extreme grazing angles (a 0.7-degree cell is a couple of pixels
    // tall but tens of pixels wide at 10 degrees of elevation), so contour
    // detail there reads as horizontal smears. The deck thins into the sky
    // gradient over the band instead - the same place the atmosphere's own
    // haze swallows distant detail, so the read stays natural.
    constexpr double kHorizonGuardRadians = 0.13962634015954636;  // 8 deg above the horizon
    constexpr double kGuardFadeRadians = 0.24434609527920614;     // 14 deg of thinning cloud above it

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
    // The sun direction for the baked bank shading: the deck's frame maps
    // the world sun into dome-local coordinates (the same frame the coverage
    // sampling uses below, so shading and coverage follow one geometry).
    const glm::dvec3 sunLocal = glm::length(sunDirection) > 1.0e-9
        ? glm::inverse(glm::dmat3(tangent, upNormalized, third)) * glm::normalize(sunDirection)
        : glm::dvec3(0.0);
    const planet::Vec3 deckSun{sunLocal.x, sunLocal.y, sunLocal.z};

    CoverageShellSpec spec;
    spec.segments = kSegments;
    spec.bands = kBands;
    spec.latitudeLowRadians = 0.0;
    spec.latitudeHighRadians = glm::pi<double>() * 0.5;
    spec.shellRadiusMeters = 1.0; // scaled per frame with the dome
    // Contour discipline for the deck: the field band-limits at twice the
    // vertex spacing (octaves the grid cannot represent fade out) and four
    // arc-isotropic smoothing rounds round the inclusion boundary, so the
    // deck's outline stays curved at the grid scale. The coverage threshold
    // band widens six-fold (see CoverageShellSpec::coverageSoftnessScale):
    // at the deck's viewing distance one grid cell spans a dozen pixels, and
    // the configured band maps to a spatial transition only a couple of
    // samples wide - without the widening the sampled field is near-binary
    // speckle and the contoured cutout terraces into stair steps and
    // sawtooth fringe rows no smoothing can repair.
    spec.lowPassCutoffMeters = 2.0 * vertexSpacing;
    spec.coverageSoftnessScale = 6.0;
    spec.coverageFloor = kCoverageFloor;
    spec.smoothingPasses = 4;
    // No per-quad floor jitter on the deck (unlike the macro shell): the
    // jittered threshold is per-QUAD, so two quads sharing a grid edge each
    // interpolate their own crossing position on that edge and the polygons
    // part by the threshold difference - hairline sky notches along every
    // bank boundary, glaring at ground-view quad sizes. The sub-quad contour
    // walk already curves the outlines (bilinear isolines at 4x4
    // sub-resolution), which is the stair-breaking the jitter was for.
    spec.floorJitter = 0.0;
    spec.floorJitterSeed = 0xC10D5C11ull;
    // The deck's cells span a dozen pixels at eye level, so merged interior
    // rectangles would read as blocky grid-aligned slabs (see
    // CoverageShellSpec::mergeInterior); the deck keeps per-quad
    // tessellation.
    spec.mergeInterior = false;
    spec.minQuadLatitudeRadians = kHorizonGuardRadians;
    spec.guardFadeRadians = kGuardFadeRadians;
    spec.guardFloorLift = 0.12; // banks thin into sparse fragments through the guard band
    spec.hazeEdgeBlend = true;
    spec.hazeEdgeWidth = 0.55;
    spec.frame = glm::dmat3(tangent, upNormalized, third);
    spec.sunDirection = glm::length(sunLocal) > 1.0e-9 ? &deckSun : nullptr;

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
    sunDiscMaterials_.clear();
    sunDiscActive_ = false;
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
