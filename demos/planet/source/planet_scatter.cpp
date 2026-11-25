// Surface scatter system for the planet demo (see planet_scatter.hpp).
//
// Design notes that keep this honest to the project's conventions:
//  - No hardcoded per-view constants: every rate and radius comes from the
//    planet module's configured surface-scatter keys (PlanetGenerationParams).
//  - Placement is a pure function of world space: the stationary elements
//    (vegetation, rocks) key their jitter and presence to a planet-centered
//    lattice of cells, so a cell always evaluates to the same plant no matter
//    how many times the eye passes through, while the annulus gate decides
//    only whether the cell is within range this rebuild.
//  - The transient elements (whitecaps, surf, mist, birds) are allowed to
//    re-roll: real breaking foam turns over every couple of seconds, so a
//    refresh-salted hash regenerating part of it reads as churn, not glitch.
//  - PlanetVertex (position/normal/color/uv, 44 bytes) is the only vertex
//    format; textures are procedurally generated once from the planet seed.
//  - The planet module owns the density rules (which biome carries which
//    element and how much); this file only meshes what those rules decide.
#include "planet_scatter.hpp"

#include "planet_demo_support.hpp"

#include <engine/planet/capabilities.hpp>
#include <engine/planet/climate.hpp>
#include <engine/planet/fbm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <unordered_set>
#include <vector>

namespace demo::planet_demo
{
namespace
{

using engine::planet::Biome;
using engine::planet::ClimateSample;
using engine::planet::PlanetVertex;
using engine::planet::Vec3;
namespace renderer = engine::renderer;

// One species of a vegetation atlas: the 2x2 tile index (0=TL, 1=TR, 2=BL,
// 3=BR), the card's aspect (width / height) and its height band in meters.
// See assets/textures/vegetation for what each tile photographs.
struct SpeciesCard
{
    std::uint32_t tile = 0;
    float aspect = 1.0f;
    double minHeightMeters = 1.0;
    double maxHeightMeters = 1.0;
};

// Tree species by biome (tile: 0 broadleaf, 1 acacia, 2 conifer, 3 dry
// broadleaf). The hash lane mixes the species where a biome grows more than
// one (real stands mix), stays fixed where it does not (taiga is conifer
// country). Height bands follow the biome's real stature: rainforest
// emergents, taiga spires, savanna flat-tops, arid stubs.
SpeciesCard treeSpeciesFor(Biome biome, std::uint64_t hash)
{
    const double mix = PlanetScatterSystem::hashUnit(hash, 8);
    switch (biome)
    {
        case Biome::TropicalRainforest:
        case Biome::TemperateRainforest:
            return {0, 0.82f, 11.0, 19.0};
        case Biome::TemperateForest:
            // Three-card deal: a stand where two silhouettes out of three
            // differ reads as a wood, one repeated silhouette reads as a
            // stamp row (the coast capture's sprinkle-grid tell).
            return mix < 0.45 ? SpeciesCard{0, 0.80f, 7.0, 17.0}
                : (mix < 0.80 ? SpeciesCard{1, 1.00f, 6.0, 15.0} : SpeciesCard{3, 0.78f, 5.0, 12.0});
        case Biome::SeasonalForest:
            return mix < 0.40 ? SpeciesCard{1, 1.00f, 6.0, 16.0}
                : (mix < 0.75 ? SpeciesCard{0, 0.80f, 7.0, 16.0} : SpeciesCard{3, 0.78f, 5.0, 13.0});
        case Biome::Taiga:
            return {2, 0.52f, 10.0, 17.0};
        case Biome::Savanna:
            return mix < 0.55 ? SpeciesCard{1, 1.05f, 5.0, 12.0}
                : (mix < 0.85 ? SpeciesCard{3, 0.78f, 4.0, 8.0} : SpeciesCard{0, 0.80f, 6.0, 11.0});
        case Biome::Grassland:
            return mix < 0.5 ? SpeciesCard{1, 1.00f, 4.0, 10.0} : SpeciesCard{0, 0.80f, 4.0, 10.0};
        case Biome::Shrubland:
            return {3, 0.78f, 4.0, 8.0};
        default:
            return {3, 0.78f, 4.0, 7.0};
    }
}

// Bush species by biome (tile: 0 green shrub, 1 dry shrub, 2 fern, 3 scrub).
SpeciesCard bushSpeciesFor(Biome biome, std::uint64_t hash)
{
    const double mix = PlanetScatterSystem::hashUnit(hash, 8);
    switch (biome)
    {
        case Biome::TropicalRainforest:
        case Biome::TemperateRainforest:
            return mix < 0.6 ? SpeciesCard{2, 0.70f, 0.8, 1.6} : SpeciesCard{0, 0.95f, 0.9, 1.8};
        case Biome::TemperateForest:
        case Biome::SeasonalForest:
            return mix < 0.5 ? SpeciesCard{0, 0.95f, 0.8, 1.7} : SpeciesCard{2, 0.70f, 0.7, 1.4};
        case Biome::Taiga:
            return {3, 0.85f, 0.7, 1.4};
        case Biome::Savanna:
            return mix < 0.6 ? SpeciesCard{1, 0.90f, 0.7, 1.5} : SpeciesCard{3, 0.85f, 0.6, 1.2};
        case Biome::Grassland:
            return mix < 0.6 ? SpeciesCard{0, 0.95f, 0.6, 1.3} : SpeciesCard{3, 0.85f, 0.5, 1.0};
        case Biome::Shrubland:
            return mix < 0.4 ? SpeciesCard{0, 0.95f, 0.7, 1.5}
                : (mix < 0.75 ? SpeciesCard{1, 0.90f, 0.7, 1.5} : SpeciesCard{3, 0.85f, 0.6, 1.3});
        case Biome::Tundra:
            return {3, 0.85f, 0.4, 0.8};
        default:
            return {3, 0.85f, 0.6, 1.2};
    }
}

// Grass card by biome (tile: 0 lush tuft, 1 dry tuft, 2 low sprouts,
// 3 sparse dry). Wet sites keep the lush card; arid country the dry one,
// the desert fringe and tundra the sparse ones.
SpeciesCard grassSpeciesFor(Biome biome, std::uint64_t hash)
{
    const double mix = PlanetScatterSystem::hashUnit(hash, 8);
    switch (biome)
    {
        case Biome::Grassland:
            // A mixed meadow, not a monoculture: the hash lane deals two
            // secondary cards into the dominant tile so neighbors differ in
            // silhouette as well as tint (a single-card biome planted the
            // whole sward with one repeated sprite).
            return mix < 0.55 ? SpeciesCard{0, 1.00f, 0.35, 0.75}
                : (mix < 0.85 ? SpeciesCard{2, 0.95f, 0.30, 0.68} : SpeciesCard{3, 1.10f, 0.28, 0.62});
        case Biome::Savanna:
            return mix < 0.65 ? SpeciesCard{1, 1.00f, 0.30, 0.65} : SpeciesCard{0, 1.00f, 0.30, 0.60};
        case Biome::Shrubland:
            return mix < 0.7 ? SpeciesCard{1, 1.00f, 0.28, 0.60} : SpeciesCard{3, 1.10f, 0.25, 0.55};
        case Biome::TemperateForest:
        case Biome::SeasonalForest:
            return mix < 0.7 ? SpeciesCard{0, 1.00f, 0.32, 0.70} : SpeciesCard{2, 0.90f, 0.25, 0.55};
        case Biome::TemperateRainforest:
        case Biome::TropicalRainforest:
            return mix < 0.7 ? SpeciesCard{0, 1.00f, 0.40, 0.85} : SpeciesCard{2, 0.90f, 0.35, 0.75};
        case Biome::Taiga:
            return mix < 0.6 ? SpeciesCard{3, 1.10f, 0.25, 0.55} : SpeciesCard{0, 1.00f, 0.28, 0.60};
        case Biome::Tundra:
            return mix < 0.75 ? SpeciesCard{3, 1.10f, 0.18, 0.40} : SpeciesCard{1, 1.00f, 0.20, 0.42};
        default:
            return {3, 1.10f, 0.22, 0.50};
    }
}

// UV rectangle of one atlas tile (the 2x2 species grid), with a small inset
// so bilinear filtering at the tile edge cannot bleed the neighboring
// species' pixels in. v0 is the tile's TOP row (the renderer reads the pixel
// buffer's first row at v=0), v1 its base.
struct SpeciesUvRect
{
    float u0, v0, u1, v1;
};
SpeciesUvRect speciesUv(std::uint32_t tile)
{
    const std::uint32_t count = 2;
    tile = tile % (count * count);
    const float tx = static_cast<float>(tile % count) / static_cast<float>(count);
    const float ty = static_cast<float>(tile / count) / static_cast<float>(count);
    const float step = 1.0f / static_cast<float>(count);
    const float inset = 0.01f;
    return {tx + inset * step, ty + inset * step, tx + (1.0f - inset) * step, ty + (1.0f - inset) * step};
}

inline constexpr std::uint32_t kVertexStride = sizeof(PlanetVertex);

// Salt lanes of the cell hash: every independent decision (presence, jitter,
// size, tint) draws from its own lane so no two decisions correlate.
inline constexpr std::uint64_t kSaltTree     = 0x51D2A1B7C0FFEE01ull;
inline constexpr std::uint64_t kSaltBush     = 0xA1B7C0FFEE515D21ull;
inline constexpr std::uint64_t kSaltGrass    = 0xC0FFEE515D2A1B70ull;
inline constexpr std::uint64_t kSaltRock     = 0x5D2A1B7C0FFEE1A1ull;
inline constexpr std::uint64_t kSaltWhitecap = 0x51A1B7C0FFEED2A1ull;
inline constexpr std::uint64_t kSaltSurf     = 0xB7C0FFEE51D2A1B0ull;
inline constexpr std::uint64_t kSaltMist     = 0xFFEE51D2A1B7C0F0ull;
inline constexpr std::uint64_t kSaltFlock    = 0x51D2F10C0FFEE100ull;

inline constexpr double kPi = 3.14159265358979323846;

// Species tint multipliers applied to the ground's biome color (the terrain
// albedo at the planting site) to read as foliage instead of painted ground.
inline constexpr float kTreeTint[3]  = {0.78f, 0.96f, 0.66f};
inline constexpr float kBushTint[3]  = {0.80f, 1.04f, 0.74f};

// Lattice cell sizes (meters). Chosen so each element's screen-area budget
// stays a small fraction of the frame at 1920x1080: distant trees are the
// cheapest per pixel, near-camera grass the most expensive, so grass keeps
// the tightest radius (see the configuration docs). Grass carries the finest
// lattice of all groups - real grass reads as a sward only when tufts sit
// closer than a blade's length apart; coarser cells plant visible rows.
inline constexpr double kTreeCellMeters  = 46.0;
inline constexpr double kBushCellMeters  = 24.0;
inline constexpr double kGrassCellMeters = 0.62;
inline constexpr double kGrassMidCellMeters = 1.4;
inline constexpr double kRockCellMeters  = 42.0;
inline constexpr double kFoamCellMeters  = 26.0;

// Vertical gates (meters above the datum sea radius).
inline constexpr double kMinLandElevationMeters      = 2.5; // keep plants off the intertidal sand
inline constexpr double kTreeMinElevationMeters      = 8.0; // trees start above the dune/swash belt, never on bare flats
inline constexpr double kFlockMinEyeDistanceMeters   = 80.0; // flocks never brush the camera
inline constexpr double kFlockMinEyeDistanceMarginMeters = 8.0;
inline constexpr double kMinWhitecapDepthMeters = 8.0;  // breaking crests live over open water
inline constexpr double kSeaSpriteLiftMeters    = 0.45; // foam quads ride above the wave-roughened sheet
// Contact shadows read only inside the near/mid ring - beyond it a tree's
// shadow subtends a couple of pixels at ground-level eyes and a grazing
// sliver at elevated ones, while its translucent fill still costs the frame.
inline constexpr double kShadowMaxDistanceMeters = 600.0;

// Sea-state churn cadence: the foam pool's staggered quarters re-roll on this
// wall-clock interval (real breaking foam turns over in a couple of seconds).
inline constexpr double kWhitecapRefreshSeconds = 1.1;
inline constexpr std::uint32_t kWhitecapEpochTicks = 4u;

inline float clamp01(float value)
{
    return std::clamp(value, 0.0f, 1.0f);
}

// Species tint for one planting site. The sprites are photographic cutouts
// that already carry their species' own hue, so the tint SHIFTS hue instead of
// repainting it: the site's ground color (wet sites cool-green, arid sites
// warm-straw) steers 30% of the multiply while neutral 1.0 carries the rest,
// scaled by the per-site variation. A full-saturation tint would wash the
// photo out into a painted billboard (and its minified mean into a neon chip).
// Grass tint: the photographic cards are khaki-neutral, so the multiply must
// carry the site's season - greenness keyed to the BIOME (a grassland sward
// is green even at moderate moisture; a savanna reads straw at the same
// moisture), then shifted by the site's moisture around that base. A neutral
// tint leaves the cards reading pale khaki against the terrain, which the
// eye parses as dead weeds rather than grass.
inline double grassGreenness(Biome biome)
{
    switch (biome)
    {
        case Biome::Grassland:
            return 0.78;
        case Biome::TropicalRainforest:
        case Biome::TemperateRainforest:
            return 0.88;
        case Biome::TemperateForest:
            return 0.72;
        case Biome::SeasonalForest:
            return 0.60;
        case Biome::Taiga:
            return 0.35;
        case Biome::Shrubland:
            return 0.30;
        case Biome::Savanna:
            return 0.22;
        case Biome::Tundra:
            return 0.12;
        default:
            return 0.25;
    }
}

inline std::array<float, 3> grassTint(Biome biome, double moisture01, double variation)
{
    const double m = std::clamp(moisture01, 0.0, 1.0);
    const double green = std::clamp(grassGreenness(biome) + (m - 0.5) * 0.5, 0.0, 1.0);
    double r = std::lerp(1.22, 0.76, green) * variation;
    double g = std::lerp(0.98, 1.38, green) * variation;
    double b = std::lerp(0.86, 0.68, green) * variation;
    // Preserve the hue: a per-channel clamp would flatten the green push back
    // to neutral (G is the tallest channel; clamping it alone while R stays
    // sub-1 kills exactly the G/R ratio the tint exists to create). Instead
    // the whole tint scales by its own tallest channel, so the ratio rides
    // intact and only the overall brightness yields.
    const double tallest = std::max({r, g, b});
    if (tallest > 1.0)
    {
        r /= tallest;
        g /= tallest;
        b /= tallest;
    }
    return {clamp01(static_cast<float>(r)), clamp01(static_cast<float>(g)), clamp01(static_cast<float>(b))};
}

// Sward tint for one ground sample: grassTint's biome/moisture hue pulled
// toward the local terrain albedo by a moisture-weighted share. The drier
// the ground, the more a tuft reads as the straw tone of the soil it stands
// in (a moisture-0.24 steppe sward carries most of the soil's ochre chroma;
// even a rain-soaked sward keeps a small ground pull, since real swards
// always carry some soil-intermixed color) - without the pull, a tuft's
// saturated blade green sits on olive ground and reads as decal plastic.
// hueJitter shifts the tuft's own moisture verdict per tuft, so neighbors
// on one soil patch differ in tone the way real swards do (a clump catches
// more drainage, its neighbor dries out first). The blend is hue-preserving
// like grassTint: it mixes the albedo's chroma (the luminance-normalized
// tint), never its brightness, so the sward's own shading and variation
// still set how bright a tuft is.
inline std::array<float, 3> grassSwardTint(const ClimateSample &sample, double variation, double hueJitter)
{
    ClimateSample tuftSample = sample;
    tuftSample.moisture01 = clamp01(static_cast<float>(sample.moisture01 + hueJitter));
    const std::array<float, 3> blade = grassTint(tuftSample.biome, tuftSample.moisture01, variation);
    const double ground[3] = {static_cast<double>(sample.color[0]),
        static_cast<double>(sample.color[1]), static_cast<double>(sample.color[2])};
    const double groundLuminance = 0.2126 * ground[0] + 0.7152 * ground[1] + 0.0722 * ground[2];
    if (groundLuminance <= 1.0e-4)
    {
        return blade;
    }
    const double dryness = 1.0 - std::clamp(sample.moisture01, 0.0, 1.0);
    std::array<float, 3> mixed{};
    for (int channel = 0; channel < 3; ++channel)
    {
        mixed[channel] = clamp01(static_cast<float>(
            std::lerp(static_cast<double>(blade[channel]), ground[channel] / groundLuminance,
                0.30 + 0.55 * dryness)));
    }
    // Same hue-preserving normalization as grassTint (see the note there):
    // rescale by the tallest channel instead of flattening per channel.
    const double tallest = std::max({mixed[0], mixed[1], mixed[2]});
    if (tallest > 1.0f)
    {
        for (float &channel : mixed)
        {
            channel = static_cast<float>(channel / tallest);
        }
    }
    return mixed;
}

inline std::array<float, 3> speciesTint(const ClimateSample &sample, const float tint[3], double variation)
{
    const double ground[3] = {static_cast<double>(sample.color[0]),
        static_cast<double>(sample.color[1]), static_cast<double>(sample.color[2])};
    const double groundLuminance = 0.2126 * ground[0] + 0.7152 * ground[1] + 0.0722 * ground[2];
    double hue[3] = {1.0, 1.0, 1.0};
    if (groundLuminance > 1.0e-4)
    {
        for (int channel = 0; channel < 3; ++channel)
        {
            hue[channel] = std::clamp(ground[channel] / groundLuminance * tint[channel], 0.35, 1.9);
        }
    }
    std::array<float, 3> result{};
    double mixed[3] = {std::lerp(1.0, hue[0], 0.30) * variation,
        std::lerp(1.0, hue[1], 0.30) * variation,
        std::lerp(1.0, hue[2], 0.30) * variation};
    // Same hue-preserving normalization as grassTint (see the note there).
    const double tallest = std::max({mixed[0], mixed[1], mixed[2]});
    if (tallest > 1.0)
    {
        for (double &channel : mixed)
        {
            channel /= tallest;
        }
    }
    for (int channel = 0; channel < 3; ++channel)
    {
        result[channel] = clamp01(static_cast<float>(mixed[channel]));
    }
    return result;
}

inline double clamp01(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

// Smooth Hermite step between edge0 and edge1, in EITHER edge order (the
// sprite generators step down as often as up: canopy falloffs, trunk tapers,
// foam rims). A degenerate zero-width band collapses to a hard threshold.
inline double smoothStepLocal(double edge0, double edge1, double value)
{
    const double span = edge1 - edge0;
    if (std::fabs(span) < 1.0e-9)
    {
        return value >= edge1 ? 1.0 : 0.0;
    }
    const double t = std::clamp((value - edge0) / span, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// One RGBA8 texel write into a generated sprite buffer.
inline void putPixel(std::vector<std::uint8_t> &pixels, std::uint32_t width, std::uint32_t x, std::uint32_t y,
    std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
    const std::size_t index = (static_cast<std::size_t>(y) * width + x) * 4u;
    pixels[index + 0] = r;
    pixels[index + 1] = g;
    pixels[index + 2] = b;
    pixels[index + 3] = a;
}

// Alpha bleed: every texel below full opacity inherits the RGB of its
// nearest fully opaque texel (breadth-first from the opaque cores). The
// sprite's alpha is untouched - the cutout masks exactly as before - but the
// RGB underneath stops reading as black. Without the bleed the mip chain's
// box filter (which averages RGB alone) drags a blade or canopy edge down
// with its unwritten black neighbors, and distant foliage collapses into
// dark silhouettes (see HeadlessTexture::sampleTrilinear).
void bleedSpriteRGB(std::vector<std::uint8_t> &pixels, std::uint32_t size)
{
    struct Seed
    {
        std::uint32_t x, y;
    };
    std::vector<Seed> frontier;
    frontier.reserve(static_cast<std::size_t>(size) * size / 4u);
    std::vector<std::uint8_t> filled(static_cast<std::size_t>(size) * size, 0u);
    for (std::uint32_t y = 0; y < size; ++y)
    {
        for (std::uint32_t x = 0; x < size; ++x)
        {
            const std::size_t index = (static_cast<std::size_t>(y) * size + x) * 4u;
            if (pixels[index + 3] == 255) // fully opaque cores seed the fill
            {
                filled[static_cast<std::size_t>(y) * size + x] = 1u;
                frontier.push_back(Seed{x, y});
            }
        }
    }
    std::size_t head = 0;
    while (head < frontier.size())
    {
        const Seed seed = frontier[head++];
        const std::size_t seedIndex = (static_cast<std::size_t>(seed.y) * size + seed.x) * 4u;
        const std::uint32_t neighbors[4][2] = {
            {seed.x - (seed.x > 0 ? 1u : 0u), seed.y},
            {seed.x + (seed.x + 1u < size ? 1u : 0u), seed.y},
            {seed.x, seed.y - (seed.y > 0 ? 1u : 0u)},
            {seed.x, seed.y + (seed.y + 1u < size ? 1u : 0u)},
        };
        for (const auto &neighbor : neighbors)
        {
            const std::size_t cell = static_cast<std::size_t>(neighbor[1]) * size + neighbor[0];
            if (filled[cell] != 0u)
            {
                continue;
            }
            const std::size_t index = cell * 4u;
            if (pixels[index + 3] == 255)
            {
                filled[cell] = 1u; // another core: mark, keep its own RGB
                frontier.push_back(Seed{neighbor[0], neighbor[1]});
                continue;
            }
            filled[cell] = 1u; // partial or empty: take the nearest core's RGB
            pixels[index + 0] = pixels[seedIndex + 0];
            pixels[index + 1] = pixels[seedIndex + 1];
            pixels[index + 2] = pixels[seedIndex + 2];
            frontier.push_back(Seed{neighbor[0], neighbor[1]});
        }
    }
}

// Deterministic value noise in [0, 1) over an integer lattice (the sprite
// generators' shared randomness; cheap, seed-pure, no external state).
inline double latticeNoise(std::uint64_t seed, std::int32_t x, std::int32_t y, std::uint64_t lane)
{
    std::uint64_t h = seed;
    h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) * 0x9E3779B97F4A7C15ull;
    h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) * 0xC2B2AE3D27D4EB4Full;
    h ^= lane * 0x165667B19E3779F9ull;
    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<double>(h >> 11) / 9007199254740992.0;
}

// Bilinear value noise over a normalized [0,1] sprite domain.
inline double spriteNoise(std::uint64_t seed, double u, double v, double frequency, std::uint64_t lane)
{
    const double fx = u * frequency;
    const double fy = v * frequency;
    const std::int32_t x0 = static_cast<std::int32_t>(std::floor(fx));
    const std::int32_t y0 = static_cast<std::int32_t>(std::floor(fy));
    const double tx = fx - static_cast<double>(x0);
    const double ty = fy - static_cast<double>(y0);
    const double sx = tx * tx * (3.0 - 2.0 * tx);
    const double sy = ty * ty * (3.0 - 2.0 * ty);
    const double n00 = latticeNoise(seed, x0, y0, lane);
    const double n10 = latticeNoise(seed, x0 + 1, y0, lane);
    const double n01 = latticeNoise(seed, x0, y0 + 1, lane);
    const double n11 = latticeNoise(seed, x0 + 1, y0 + 1, lane);
    return std::lerp(std::lerp(n00, n10, sx), std::lerp(n01, n11, sx), sy);
}

// Appends one shaded quad (2 triangles) into the canonical vertex layout.
// corners are the four planet-local positions (bilinear order: bottom-left,
// bottom-right, top-right, top-left).
struct QuadPush
{
    glm::dvec3 corners[4];
    glm::dvec3 normal;
    float color[3];
    float uv[4][2];
};

// Vertices are stored relative to the rebuild anchor (the eye at rebuild
// time): the corner math runs in f64 planet-local coordinates and only the
// final store subtracts the anchor, so the f32 pipeline sees small
// eye-relative coordinates exactly like the streamed chunk meshes do.
void pushQuad(std::vector<PlanetVertex> &vertices, std::vector<std::uint32_t> &indices, const QuadPush &quad,
    const glm::dvec3 &anchor)
{
    const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
    for (std::uint32_t corner = 0; corner < 4; ++corner)
    {
        PlanetVertex vertex;
        const glm::dvec3 relative = quad.corners[corner] - anchor;
        vertex.position[0] = static_cast<float>(relative.x);
        vertex.position[1] = static_cast<float>(relative.y);
        vertex.position[2] = static_cast<float>(relative.z);
        vertex.normal[0]   = static_cast<float>(quad.normal.x);
        vertex.normal[1]   = static_cast<float>(quad.normal.y);
        vertex.normal[2]   = static_cast<float>(quad.normal.z);
        vertex.color[0]    = quad.color[0];
        vertex.color[1]    = quad.color[1];
        vertex.color[2]    = quad.color[2];
        vertex.uv[0]       = quad.uv[corner][0];
        vertex.uv[1]       = quad.uv[corner][1];
        vertices.push_back(vertex);
    }
    // Bilinear corner order: (bl, br, tr), (bl, tr, tl). Billboards declare
    // doubleSided, so the winding only needs internal consistency.
    indices.push_back(base + 0u);
    indices.push_back(base + 1u);
    indices.push_back(base + 2u);
    indices.push_back(base + 0u);
    indices.push_back(base + 2u);
    indices.push_back(base + 3u);
}

// ---- the module-owned density rules ----------------------------------------
// Base presence of each element per biome, before the configured multipliers
// and the moisture scaling. These encode the real-world distributions the
// plausibility gap comes from: forests carry trees (~31% of Earth's land is
// forested), grasslands carry grass, shrubland carries shrubs, open ground
// carries scattered rocks, and nothing grows on ice or dunes.

double treeBaseDensity(Biome biome)
{
    switch (biome)
    {
        case Biome::TropicalRainforest:
        case Biome::TemperateRainforest:
            return 0.85;
        case Biome::TemperateForest:
            return 0.80;
        case Biome::SeasonalForest:
            return 0.65;
        case Biome::Taiga:
            return 0.88; // the boreal forest is a dense conifer stand, not open parkland
        case Biome::Savanna:
            return 0.16;
        case Biome::Grassland:
            return 0.05;
        case Biome::Shrubland:
            return 0.04;
        default:
            return 0.0;
    }
}

double bushBaseDensity(Biome biome)
{
    switch (biome)
    {
        case Biome::Shrubland:
            return 0.45;
        case Biome::Savanna:
            return 0.20;
        case Biome::Grassland:
            return 0.12;
        case Biome::TemperateForest:
        case Biome::SeasonalForest:
        case Biome::TemperateRainforest:
        case Biome::TropicalRainforest:
            return 0.14;
        case Biome::Taiga:
            return 0.10;
        case Biome::Tundra:
            return 0.05;
        default:
            return 0.0;
    }
}

double grassBaseDensity(Biome biome)
{
    switch (biome)
    {
        case Biome::Grassland:
            return 0.95;
        case Biome::Savanna:
            return 0.45;
        case Biome::Shrubland:
            return 0.38;
        case Biome::TemperateForest:
        case Biome::SeasonalForest:
        case Biome::TemperateRainforest:
        case Biome::TropicalRainforest:
            return 0.32;
        case Biome::Taiga:
            return 0.20;
        case Biome::Tundra:
            return 0.22;
        default:
            return 0.0;
    }
}

double rockBaseDensity(Biome biome)
{
    switch (biome)
    {
        case Biome::Rock:
        case Biome::Snow:
            return 0.50;
        case Biome::Desert:
        case Biome::Tundra:
            return 0.35;
        case Biome::Beach:
            return 0.14;
        default:
            return 0.15;
    }
}

} // namespace

PlanetScatterSystem::PlanetScatterSystem(engine::planet::PlanetCapability &planetCapability,
    engine::planet::PlanetHandle planet, const engine::planet::PlanetGenerationParams &params)
    : capability_(planetCapability)
    , planet_(planet)
    , params_(params)
    , shadowField_(params)
{
}

std::uint64_t PlanetScatterSystem::cellHash(std::uint64_t seed, std::uint64_t salt, std::int64_t x, std::int64_t y,
    std::int64_t z)
{
    std::uint64_t h = seed ^ salt;
    h ^= static_cast<std::uint64_t>(x) * 0x9E3779B97F4A7C15ull;
    h ^= static_cast<std::uint64_t>(y) * 0xC2B2AE3D27D4EB4Full;
    h ^= static_cast<std::uint64_t>(z) * 0x165667B19E3779F9ull;
    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return h;
}

double PlanetScatterSystem::hashUnit(std::uint64_t hash, std::uint64_t lane)
{
    std::uint64_t h = hash ^ (lane * 0x27D4EB2F165667C5ull);
    h ^= h >> 33;
    h *= 0xFF51AFD7ED558CCDull;
    h ^= h >> 33;
    return static_cast<double>(h >> 11) / 9007199254740992.0;
}

void PlanetScatterSystem::loadVegetationAtlases(const std::filesystem::path &assetDirectory)
{
    // The sea-state sprites (foam, mist, contact shadows) stay procedural in
    // every configuration; the foliage groups come from the photographic
    // atlases when they load and from the seed-generated fallback otherwise.
    buildSeaStateSprites();
    if (!rendererBound_ || renderCapability_ == nullptr || renderCapability_->createTextureFromFile == nullptr)
    {
        buildSpriteTextures();
        initialized_ = treeTexture_ != renderer::invalidTexture && bushTexture_ != renderer::invalidTexture
            && grassTexture_ != renderer::invalidTexture && foamTexture_ != renderer::invalidTexture
            && mistTexture_ != renderer::invalidTexture && shadowTexture_ != renderer::invalidTexture;
        return;
    }
    const std::filesystem::path vegetationDirectory = assetDirectory / "vegetation";
    renderer::TextureHandle handles[3] = {renderer::invalidTexture, renderer::invalidTexture, renderer::invalidTexture};
    renderer::TextureHandle *slots[3] = {&treeTexture_, &bushTexture_, &grassTexture_};
    const char *files[3] = {"tree_atlas.png", "bush_atlas.png", "grass_atlas.png"};
    bool allLoaded = true;
    for (std::size_t index = 0; index < 3; ++index)
    {
        const std::filesystem::path path = vegetationDirectory / files[index];
        renderer::RenderError error = renderer::RenderError::None;
        handles[index] = renderCapability_->createTextureFromFile(renderCapability_->self, rendererHandle_,
            path.string().c_str(), &error);
        if (handles[index] == renderer::invalidTexture)
        {
            allLoaded = false;
            break;
        }
    }
    if (!allLoaded)
    {
        // Degrade instead of vanish: free any partially loaded atlases and
        // fall back to the seed-generated procedural sprites.
        for (renderer::TextureHandle handle : handles)
        {
            if (handle != renderer::invalidTexture && renderCapability_->destroyTexture != nullptr)
            {
                renderCapability_->destroyTexture(renderCapability_->self, rendererHandle_, handle);
            }
        }
        std::fprintf(stderr, "demo_planet: vegetation atlases not found under '%s'; the scatter system draws its fallback procedural sprites\n",
            vegetationDirectory.string().c_str());
        buildSpriteTextures();
        return;
    }
    for (std::size_t index = 0; index < 3; ++index)
    {
        *slots[index] = handles[index];
    }
    initialized_ = treeTexture_ != renderer::invalidTexture && bushTexture_ != renderer::invalidTexture
        && grassTexture_ != renderer::invalidTexture && foamTexture_ != renderer::invalidTexture
        && mistTexture_ != renderer::invalidTexture && shadowTexture_ != renderer::invalidTexture;
}

renderer::TextureHandle PlanetScatterSystem::createSpriteAtlas(const std::vector<std::uint8_t> &sprite,
    std::uint32_t spriteSize)
{
    // Replicates one sprite into the 2x2 tile grid every vegetation group
    // addresses (the photographic atlases ship that layout; the fallback
    // keeps the same UV contract so the scatter code has one code path).
    const std::uint32_t atlasSize = spriteSize * 2u;
    std::vector<std::uint8_t> atlas(static_cast<std::size_t>(atlasSize) * atlasSize * 4u, 0u);
    for (std::uint32_t tileY = 0; tileY < 2u; ++tileY)
    {
        for (std::uint32_t tileX = 0; tileX < 2u; ++tileX)
        {
            for (std::uint32_t y = 0; y < spriteSize; ++y)
            {
                std::memcpy(&atlas[(static_cast<std::size_t>(tileY * spriteSize + y) * atlasSize
                     + static_cast<std::size_t>(tileX) * spriteSize) * 4u],
                    &sprite[static_cast<std::size_t>(y) * spriteSize * 4u],
                    static_cast<std::size_t>(spriteSize) * 4u);
            }
        }
    }
    renderer::TextureDescription description;
    description.width = atlasSize;
    description.height = atlasSize;
    description.pixels = atlas.data();
    description.filter = renderer::TextureFilter::Linear;
    description.address = renderer::TextureAddress::ClampEdge;
    renderer::RenderError error = renderer::RenderError::None;
    return renderCapability_->createTexture(renderCapability_->self, rendererHandle_, &description, &error);
}

void PlanetScatterSystem::buildSeaStateSprites()
{
    if (!rendererBound_ || renderCapability_ == nullptr || renderCapability_->createTexture == nullptr
        || foamTexture_ != renderer::invalidTexture)
    {
        return;
    }


    // Foam patch: an irregular white blob whose alpha edge breaks into the
    // clumpy holes real sea foam shows; RGB carries the bubble brightness.
    {
        const std::uint32_t size = 128;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        for (std::uint32_t y = 0; y < size; ++y)
        {
            const double v = static_cast<double>(y) / static_cast<double>(size - 1);
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                const double radial = std::sqrt((u - 0.5) * (u - 0.5) + (v - 0.5) * (v - 0.5)) * 2.0;
                const double clumps = spriteNoise(params_.seed, u * 9.0, v * 9.0, 9.0, 0x8192A3ull);
                const double fine = spriteNoise(params_.seed, u * 22.0, v * 22.0, 22.0, 0x92A3B4ull);
                double alpha = smoothStepLocal(0.88, 0.30, radial - 0.22 * (clumps - 0.5));
                alpha *= smoothStepLocal(0.30, 0.55, fine) * 0.55 + 0.45;
                alpha = std::clamp(alpha, 0.0, 1.0);
                const std::uint8_t bright = static_cast<std::uint8_t>(std::clamp(0.82 + 0.18 * fine, 0.0, 1.0) * 255.0);
                putPixel(pixels, size, x, y, bright, bright, bright, static_cast<std::uint8_t>(alpha * 255.0));
            }
        }
        renderer::TextureDescription description;
        description.width = size;
        description.height = size;
        description.pixels = pixels.data();
        description.filter = renderer::TextureFilter::Linear;
        description.address = renderer::TextureAddress::ClampEdge;
        renderer::RenderError error = renderer::RenderError::None;
        foamTexture_ = renderCapability_->createTexture(renderCapability_->self, rendererHandle_, &description, &error);
    }

    // Mist pool: a soft radial gradient with a noise-wobbled edge; RGB white
    // (the material tint colors it per rebuild). Symmetric in both axes - the
    // pool lies flat on the ground, so the fog reads equally soft in every
    // direction from above.
    {
        const std::uint32_t size = 128;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        for (std::uint32_t y = 0; y < size; ++y)
        {
            const double v = static_cast<double>(y) / static_cast<double>(size - 1);
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                const double radial = std::sqrt((u - 0.5) * (u - 0.5) + (v - 0.5) * (v - 0.5)) * 2.0;
                const double wobble = (spriteNoise(params_.seed, u * 5.0, v * 5.0, 5.0, 0xA3B4C5ull) - 0.5) * 0.30;
                double alpha = smoothStepLocal(1.05, 0.25, radial + wobble);
                putPixel(pixels, size, x, y, 255, 255, 255, static_cast<std::uint8_t>(std::clamp(alpha, 0.0, 1.0) * 255.0));
            }
        }
        renderer::TextureDescription description;
        description.width = size;
        description.height = size;
        description.pixels = pixels.data();
        description.filter = renderer::TextureFilter::Linear;
        description.address = renderer::TextureAddress::ClampEdge;
        renderer::RenderError error = renderer::RenderError::None;
        mistTexture_ = renderCapability_->createTexture(renderCapability_->self, rendererHandle_, &description, &error);
    }

    // Contact-shadow patch: BLACK RGB (the transparent route composites
    // src-over, so darkening the ground needs a black source - a grey source
    // would brighten dark ground) with a radial alpha falloff, soft-edged so
    // the penumbra reads on the ground. The vertex color stays black too; the
    // shadow's depth comes from the material's opacity times the texel alpha.
    {
        const std::uint32_t size = 64;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        for (std::uint32_t y = 0; y < size; ++y)
        {
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                const double v = static_cast<double>(y) / static_cast<double>(size - 1);
                const double radial = std::sqrt((u - 0.5) * (u - 0.5) + (v - 0.5) * (v - 0.5)) * 2.0;
                const double alpha = std::pow(smoothStepLocal(1.02, 0.18, radial), 1.25);
                putPixel(pixels, size, x, y, 0, 0, 0, static_cast<std::uint8_t>(std::clamp(alpha, 0.0, 1.0) * 255.0));
            }
        }
        renderer::TextureDescription description;
        description.width = size;
        description.height = size;
        description.pixels = pixels.data();
        description.filter = renderer::TextureFilter::Linear;
        description.address = renderer::TextureAddress::ClampEdge;
        renderer::RenderError error = renderer::RenderError::None;
        shadowTexture_ = renderCapability_->createTexture(renderCapability_->self, rendererHandle_, &description, &error);
    }
}

// Rescales a sprite's RGB so the OPAQUE (alpha^2-weighted) mean lands in the
// plant-albedo band: the rasterizer reads texture bytes as linear radiance,
// and a baked-bright sprite (canopy greens around 0.35 stored) renders as a
// bleached card four stops too bright against terrain whose own multiply
// carries ~0.1 linear albedo. The shading pattern is preserved - only the
// overall exposure moves.
void rescaleSpriteAlbedo(std::vector<std::uint8_t> &pixels, std::uint32_t size, double targetMean)
{
    double weightedSum = 0.0;
    double weightSum = 0.0;
    for (std::uint32_t y = 0; y < size; ++y)
    {
        for (std::uint32_t x = 0; x < size; ++x)
        {
            const std::size_t index = (static_cast<std::size_t>(y) * size + x) * 4u;
            const double alpha = static_cast<double>(pixels[index + 3]) / 255.0;
            const double weight = alpha * alpha;
            for (int channel = 0; channel < 3; ++channel)
            {
                weightedSum += weight * (static_cast<double>(pixels[index + channel]) / 255.0);
            }
            weightSum += weight * 3.0;
        }
    }
    const double mean = weightSum > 1.0e-6 ? weightedSum / weightSum : 0.0;
    const double scale = mean > 1.0e-4 ? targetMean / mean : 1.0;
    for (std::size_t index = 0; index + 3 < pixels.size(); index += 4)
    {
        for (int channel = 0; channel < 3; ++channel)
        {
            const double value = static_cast<double>(pixels[index + channel]) * scale;
            pixels[index + channel] = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
        }
    }
}

void PlanetScatterSystem::buildSpriteTextures()
{
    if (!rendererBound_ || renderCapability_ == nullptr || renderCapability_->createTexture == nullptr
        || treeTexture_ != renderer::invalidTexture)
    {
        return;
    }

    // Tree sprite: a tapering trunk plus five overlapping leaf clusters
    // with ragged, holed edges. The canopy's RGB bakes the shading a card
    // cannot light per-voxel: each cluster carries its own sun-side
    // brightness (upper clusters bright, the shaded understory clusters
    // dark), overlap regions multiply into deep shadow pockets between the
    // clumps, and every cluster's own top-to-bottom rim gradient rounds the
    // lobes. Per-instance vertex colors tint it per biome; alpha is the
    // cutout.
    {
        const std::uint32_t size = 256;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        const double trunkHalfWidth = 0.045;
        const double trunkTopY = 0.66;
        struct Cluster
        {
            double cx, cy, rx, ry;
            double brightness; // cluster's own light (top-lit lobes run bright)
            double shade;     // cluster's base value (understory darkens)
        };
        const Cluster clusters[] = {
            {0.50, 0.80, 0.30, 0.20, 1.00, 0.86},
            {0.31, 0.63, 0.21, 0.16, 0.74, 0.60},
            {0.69, 0.65, 0.22, 0.16, 0.82, 0.66},
            {0.50, 0.50, 0.27, 0.14, 0.62, 0.50},
            {0.42, 0.71, 0.15, 0.12, 0.55, 0.45},
        };
        for (std::uint32_t y = 0; y < size; ++y)
        {
            const double v = 1.0 - static_cast<double>(y) / static_cast<double>(size - 1); // 0 bottom, 1 top
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                double canopy = 0.0;
                double coverage = 0.0; // accumulated cluster weight (overlap = pockets)
                double litWeight = 0.0;
                for (const Cluster &cluster : clusters)
                {
                    const double dx = (u - cluster.cx) / cluster.rx;
                    const double dy = (v - cluster.cy) / cluster.ry;
                    const double wobble = (spriteNoise(params_.seed, u * 5.0, v * 5.0, 5.0, 0x1A2B3Cull) - 0.5) * 0.55;
                    const double d = dx * dx + dy * dy - 1.0 + wobble;
                    const double inside = smoothStepLocal(0.22, -0.14, d);
                    if (inside > 0.0)
                    {
                        // The cluster's own top-bright rim gradient: the lobe's
                        // sunlit cap shines, its underside falls into the shade.
                        const double localHeight = 0.5 + 0.5 * dy;
                        const double clusterShade = cluster.shade
                            + (cluster.brightness - cluster.shade) * smoothStepLocal(-0.4, 0.85, localHeight);
                        litWeight += inside * clusterShade;
                        coverage += inside;
                    }
                    canopy = std::max(canopy, inside);
                }
                // Interior leaf holes: high-frequency alpha nibbles keep the
                // canopy from reading as a solid card.
                const double holes = spriteNoise(params_.seed, u * 16.0, v * 16.0, 16.0, 0x2B3C4Dull);
                canopy *= smoothStepLocal(0.18, 0.34, holes) * 0.35 + 0.65;
                // Trunk: a slightly tapering column rising into the canopy.
                const double taper = 1.0 - 0.35 * v;
                const double trunk = smoothStepLocal(trunkHalfWidth * taper + 0.012, trunkHalfWidth * taper - 0.006,
                    std::fabs(u - 0.5)) * (v < trunkTopY ? 1.0 : smoothStepLocal(trunkTopY + 0.06, trunkTopY, v));
                const double alpha = std::clamp(canopy + trunk, 0.0, 1.0);
                // Canopy shading: the overlap-weighted cluster light (deep
                // pockets where lobes stack), the whole crown's vertical
                // self-shade, and a leaf-vein noise. The pocket darkening uses
                // the average (not the max) cluster shade, so two stacked
                // lobes multiply down past either one alone.
                const double overlapCount = std::max(1.0, coverage);
                const double clusterLight = coverage > 0.0 ? litWeight / overlapCount : 0.6;
                const double pocket = 1.0 - 0.32 * smoothStepLocal(0.55, 1.45, coverage);
                const double leafShade = clusterLight * pocket
                    * (0.90 + 0.10 * spriteNoise(params_.seed, u * 11.0, v * 11.0, 11.0, 0x3C4D5Eull))
                    * (0.72 + 0.28 * smoothStepLocal(0.15, 0.95, v));
                const std::uint8_t leafR = static_cast<std::uint8_t>(std::clamp(0.17 * leafShade * 2.4, 0.0, 1.0) * 255.0);
                const std::uint8_t leafG = static_cast<std::uint8_t>(std::clamp(0.44 * leafShade * 2.2, 0.0, 1.0) * 255.0);
                const std::uint8_t leafB = static_cast<std::uint8_t>(std::clamp(0.15 * leafShade * 2.6, 0.0, 1.0) * 255.0);
                // Bark: darker at the root, with a vertical bark-stripe noise.
                const double barkShade = 0.78 + 0.22 * (1.0 - v)
                    + 0.10 * (spriteNoise(params_.seed, u * 40.0, v * 22.0, 40.0, 0xBA2Bull) - 0.5);
                const std::uint8_t barkR = static_cast<std::uint8_t>(std::clamp(0.34 * barkShade, 0.0, 1.0) * 255.0);
                const std::uint8_t barkG = static_cast<std::uint8_t>(std::clamp(0.26 * barkShade * 0.9, 0.0, 1.0) * 255.0);
                const std::uint8_t barkB = static_cast<std::uint8_t>(std::clamp(0.19 * barkShade * 0.8, 0.0, 1.0) * 255.0);
                const double trunkMix = trunk / std::max(1.0e-6, alpha);
                const std::uint8_t r = static_cast<std::uint8_t>(std::lerp(static_cast<double>(leafR), static_cast<double>(barkR), trunkMix));
                const std::uint8_t g = static_cast<std::uint8_t>(std::lerp(static_cast<double>(leafG), static_cast<double>(barkG), trunkMix));
                const std::uint8_t b = static_cast<std::uint8_t>(std::lerp(static_cast<double>(leafB), static_cast<double>(barkB), trunkMix));
                putPixel(pixels, size, x, y, r, g, b, static_cast<std::uint8_t>(alpha * 255.0));
            }
        }
        bleedSpriteRGB(pixels, size); // trunks and canopy keep their color under the mask edge
        rescaleSpriteAlbedo(pixels, size, 0.10); // linear plant-albedo band (see rescaleSpriteAlbedo)
        treeTexture_ = createSpriteAtlas(pixels, size);
    }

    // Bush sprite: canopy-only blob (denser clusters, no trunk).
    {
        const std::uint32_t size = 128;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        for (std::uint32_t y = 0; y < size; ++y)
        {
            const double v = 1.0 - static_cast<double>(y) / static_cast<double>(size - 1);
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                const double dx = (u - 0.5) / 0.44;
                const double dy = (v - 0.5) / 0.40;
                const double wobble = (spriteNoise(params_.seed, u * 6.0, v * 6.0, 6.0, 0x4D5E6Full) - 0.5) * 0.6;
                const double d = dx * dx + dy * dy - 1.0 + wobble;
                double canopy = smoothStepLocal(0.20, -0.16, d);
                const double holes = spriteNoise(params_.seed, u * 14.0, v * 14.0, 14.0, 0x5E6F70ull);
                canopy *= smoothStepLocal(0.16, 0.32, holes) * 0.4 + 0.6;
                const double leafShade = 0.70
                    + 0.30 * spriteNoise(params_.seed, u * 8.0, v * 8.0, 8.0, 0x6F7081ull)
                    - 0.15 * (1.0 - v);
                const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(0.15 * leafShade * 2.2, 0.0, 1.0) * 255.0);
                const std::uint8_t g = static_cast<std::uint8_t>(std::clamp(0.38 * leafShade * 2.0, 0.0, 1.0) * 255.0);
                const std::uint8_t b = static_cast<std::uint8_t>(std::clamp(0.13 * leafShade * 2.4, 0.0, 1.0) * 255.0);
                putPixel(pixels, size, x, y, r, g, b, static_cast<std::uint8_t>(canopy * 255.0));
            }
        }
        bleedSpriteRGB(pixels, size);
        rescaleSpriteAlbedo(pixels, size, 0.11);
        bushTexture_ = createSpriteAtlas(pixels, size);
    }

    // Grass tuft sprite: seven tapered blades fanning from the base, each a
    // curved taper with a frayed tip.
    {
        const std::uint32_t size = 128;
        std::vector<std::uint8_t> pixels(size * size * 4u, 0u);
        for (std::uint32_t y = 0; y < size; ++y)
        {
            const double v = 1.0 - static_cast<double>(y) / static_cast<double>(size - 1); // 0 base, 1 tip
            for (std::uint32_t x = 0; x < size; ++x)
            {
                const double u = static_cast<double>(x) / static_cast<double>(size - 1);
                double blade = 0.0;
                for (std::uint32_t b = 0; b < 13; ++b)
                {
                    // Jittered bases cluster the blades into a tuft instead of
                    // a regular comb; the lean fans them outward with a
                    // per-blade random strength.
                    const double bx = std::clamp(0.16 + 0.68 * static_cast<double>(b) / 12.0
                        + (latticeNoise(params_.seed, static_cast<std::int32_t>(b), 3, 0xC3ull) - 0.5) * 0.10, 0.04, 0.96);
                    const double lean = (bx - 0.5) * (0.4 + 0.5 * latticeNoise(params_.seed, static_cast<std::int32_t>(b), 5, 0xC4ull));
                    const double bladeHeight = 0.66
                        + 0.30 * latticeNoise(params_.seed, static_cast<std::int32_t>(b), 7, 0xA1ull);
                    if (v > bladeHeight)
                    {
                        continue;
                    }
                    const double grow = v / bladeHeight;
                    const double cx = bx + lean * grow * grow;
                    const double halfWidth = 0.020 * (1.0 - 0.8 * grow);
                    const double edge = smoothStepLocal(halfWidth + 0.012, halfWidth - 0.003, std::fabs(u - cx));
                    const double fray = grow > 0.80
                        ? (latticeNoise(params_.seed, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y), 0xB2ull) > 0.30 ? 1.0 : 0.0)
                        : 1.0;
                    blade = std::max(blade, edge * fray);
                }
                // Root darkening: blade bases sit in their own and each
                // other's shadow, so the tuft's bottom third reads as
                // ground-shaded - the contact blend that keeps tufts planted
                // in the soil instead of floating on it.
                const double bladeShade = 0.52 + 0.48 * v
                    + 0.14 * (spriteNoise(params_.seed, u * 7.0, v * 7.0, 7.0, 0x708192ull) - 0.5);
                // Olive-leaning blade tone: the sward's ground carries the
                // module's dry-patch senescence, so a lime sprite clashes as
                // pasted-on; the blade chroma follows the grassland palette's
                // own red/green ratio instead.
                const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(0.36 * bladeShade * 1.7, 0.0, 1.0) * 255.0);
                const std::uint8_t g = static_cast<std::uint8_t>(std::clamp(0.42 * bladeShade * 1.5, 0.0, 1.0) * 255.0);
                const std::uint8_t b = static_cast<std::uint8_t>(std::clamp(0.20 * bladeShade * 1.7, 0.0, 1.0) * 255.0);
                putPixel(pixels, size, x, y, r, g, b, static_cast<std::uint8_t>(blade * 255.0));
            }
        }
        bleedSpriteRGB(pixels, size); // thin blades: distant tufts minify to blade color, not black
        rescaleSpriteAlbedo(pixels, size, 0.13);
        grassTexture_ = createSpriteAtlas(pixels, size);
    }
}

void PlanetScatterSystem::initialize(renderer::RenderCapability &renderCapability,
    renderer::RendererHandle rendererHandle, const std::filesystem::path &assetDirectory)
{
    if (rendererBound_)
    {
        return;
    }
    renderCapability_ = &renderCapability;
    rendererHandle_ = rendererHandle;
    rendererBound_ = renderCapability.createTexture != nullptr && renderCapability.createMesh != nullptr;
    loadVegetationAtlases(assetDirectory);
}

void PlanetScatterSystem::shutdown()
{
    if (!rendererBound_ || renderCapability_ == nullptr)
    {
        return;
    }
    ScatterMesh *slots[] = {&treeMesh_, &bushMesh_, &grassMesh_, &rockMesh_, &shadowMesh_, &whitecapMesh_, &surfMesh_, &mistMesh_, &birdMesh_};
    for (ScatterMesh *slot : slots)
    {
        if (slot->mesh != renderer::invalidMesh && renderCapability_->destroyMesh != nullptr)
        {
            renderCapability_->destroyMesh(renderCapability_->self, rendererHandle_, slot->mesh);
        }
    }
    const renderer::TextureHandle textures[] = {treeTexture_, bushTexture_, grassTexture_, foamTexture_, mistTexture_, shadowTexture_};
    for (const renderer::TextureHandle texture : textures)
    {
        if (texture != renderer::invalidTexture && renderCapability_->destroyTexture != nullptr)
        {
            renderCapability_->destroyTexture(renderCapability_->self, rendererHandle_, texture);
        }
    }
    treeMesh_ = {};
    bushMesh_ = {};
    grassMesh_ = {};
    rockMesh_ = {};
    shadowMesh_ = {};
    whitecapMesh_ = {};
    surfMesh_ = {};
    mistMesh_ = {};
    birdMesh_ = {};
    treeTexture_ = bushTexture_ = grassTexture_ = foamTexture_ = mistTexture_ = shadowTexture_ = renderer::invalidTexture;
    drawCommands_.clear();
    commandAnchors_.clear();
    flocks_.clear();
    whitecapSlots_.clear();
    shadowScratch_.clear();
    shadowIndexScratch_.clear();
    initialized_ = false;
    rendererBound_ = false;
    groundLevel_ = false;
}

void PlanetScatterSystem::uploadMesh(ScatterMesh &slot, std::vector<PlanetVertex> &vertices,
    std::vector<std::uint32_t> &indices, bool alphaCutout, bool unlit, bool transparent, float opacity,
    bool doubleSided)
{
    if (!initialized_ || vertices.empty() || indices.empty())
    {
        slot.active = false;
        return;
    }

    renderer::MeshDescription description;
    description.vertices = vertices.data();
    description.vertexCount = static_cast<std::uint32_t>(vertices.size());
    description.indices = indices.data();
    description.indexCount = static_cast<std::uint32_t>(indices.size());
    description.indexFormat = renderer::MeshIndexFormat::UInt32;
    description.vertexStrideBytes = kVertexStride;
    static const std::array<renderer::VertexAttribute, 4> attributes = canonicalAttributes();
    description.vertexAttributes = attributes.data();
    description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());

    if (slot.mesh == renderer::invalidMesh)
    {
        renderer::RenderError error = renderer::RenderError::None;
        slot.mesh = renderCapability_->createMesh(renderCapability_->self, rendererHandle_, &description, &error);
        if (slot.mesh == renderer::invalidMesh)
        {
            slot.active = false;
            return;
        }
        slot.vertexUpdated = false;
    }
    else
    {
        // Same vertex count, same stride: refresh in place (the whitecap
        // churn and bird paths; the backend recomputes bounds itself).
        renderCapability_->updateMeshVertices(renderCapability_->self, rendererHandle_, slot.mesh, vertices.data(),
            static_cast<std::uint32_t>(vertices.size()), nullptr);
    }

    slot.material = renderer::MeshMaterial{};
    slot.material.baseColor[0] = 1.0f;
    slot.material.baseColor[1] = 1.0f;
    slot.material.baseColor[2] = 1.0f;
    slot.material.metallicFactor = 0.0f;
    slot.material.roughnessFactor = 0.90f;
    slot.material.unlit = unlit;
    slot.material.doubleSided = doubleSided;
    slot.material.opacity = transparent ? opacity : 1.0f;
    slot.material.alphaCutoff = alphaCutout ? 0.5f : 0.0f;
    slot.active = true;
}

// ---- placement walk ---------------------------------------------------------

// Walks the planet-centered lattice cells inside a tangent-plane annulus
// around the eye. fn receives (cx, cy, cz, cellCenter, distance). Cell
// identity is a pure function of world space, so a cell evaluates to the same
// candidate on every rebuild; only the annulus gate changes with the eye.
template <typename Fn>
void walkLattice(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &east, const glm::dvec3 &north,
    double innerMeters, double outerMeters, double cellMeters, Fn &&fn)
{
    const std::int64_t span = static_cast<std::int64_t>(std::ceil(outerMeters / cellMeters)) + 1;
    std::unordered_set<std::uint64_t> visited;
    visited.reserve(static_cast<std::size_t>((2 * span + 1) * (2 * span + 1)));
    for (std::int64_t gy = -span; gy <= span; ++gy)
    {
        for (std::int64_t gx = -span; gx <= span; ++gx)
        {
            const double dx = (static_cast<double>(gx) + 0.5) * cellMeters;
            const double dy = (static_cast<double>(gy) + 0.5) * cellMeters;
            const glm::dvec3 candidate = eyePlanetLocal + east * dx + north * dy;
            // Snap to the planet-centered lattice (planet-local coordinates).
            const std::int64_t cx = static_cast<std::int64_t>(std::floor(candidate.x / cellMeters));
            const std::int64_t cy = static_cast<std::int64_t>(std::floor(candidate.y / cellMeters));
            const std::int64_t cz = static_cast<std::int64_t>(std::floor(candidate.z / cellMeters));
            const glm::dvec3 cellCenter((static_cast<double>(cx) + 0.5) * cellMeters,
                (static_cast<double>(cy) + 0.5) * cellMeters, (static_cast<double>(cz) + 0.5) * cellMeters);
            const glm::dvec3 delta = cellCenter - eyePlanetLocal;
            const double distance = glm::length(delta);
            if (distance < innerMeters || distance > outerMeters)
            {
                continue;
            }
            const std::uint64_t key = PlanetScatterSystem::cellHash(0, 0, cx, cy, cz);
            if (!visited.insert(key).second)
            {
                continue; // two tangent candidates snapped to one cell
            }
            fn(cx, cy, cz, cellCenter, distance);
        }
    }
}

void PlanetScatterSystem::rebuild(const glm::dvec3 &eyePlanetLocal)
{
    anchorEye_ = eyePlanetLocal;
    lastSunDirection_ = glm::length(lastSunDirection_) > 1.0e-9 ? lastSunDirection_ : glm::dvec3(0.0, 1.0, 0.0);
    anchorEyeRadius_ = glm::length(eyePlanetLocal);
    hasAnchor_ = true;
    const glm::dvec3 eyeDirection = eyePlanetLocal / std::max(1.0, anchorEyeRadius_);
    glm::dvec3 axis(0.0, 0.0, 1.0);
    if (std::abs(glm::dot(axis, eyeDirection)) > 0.9)
    {
        axis = glm::dvec3(0.0, 1.0, 0.0);
    }
    const glm::dvec3 east = glm::normalize(glm::cross(axis, eyeDirection));
    const glm::dvec3 north = glm::cross(eyeDirection, east);

    shadowScratch_.clear();
    shadowIndexScratch_.clear();
    scatterVegetation(eyePlanetLocal, eyeDirection, east, north);
    scatterRocks(eyePlanetLocal, eyeDirection, east, north);
    // The contact shadows collected under the vegetation and rocks upload as
    // one unlit translucent mesh (the transparent tail depth-tests them
    // against the terrain they lie on). A night run collects nothing and the
    // slot deactivates itself in uploadMesh.
    shadowMesh_.anchorPlanetLocal = anchorEye_;
    uploadMesh(shadowMesh_, shadowScratch_, shadowIndexScratch_, false, true, true,
        static_cast<float>(std::clamp(0.62 * params_.surfaceShadowStrength, 0.0, 1.0)), true);
    shadowMesh_.material.baseColorTexture = shadowTexture_;
    scatterWhitecaps(eyePlanetLocal, eyeDirection, east, north, false);
    scatterSurf(eyePlanetLocal, eyeDirection, east, north);
    const double sunUp = glm::dot(glm::normalize(lastSunDirection_), eyeDirection);
    scatterMist(eyePlanetLocal, eyeDirection, east, north, sunUp);
    scatterBirdFlocks(eyePlanetLocal, eyeDirection, east, north);
}


// ---- geometry helpers shared by the scatter groups ---------------------------

// One camera-facing billboard (foliage card). SPHERICAL billboarding: the
// card plane is perpendicular to the sight line from the plant to the eye,
// so an elevated camera reads canopies from above (a vertical card would
// stack into walls of green under every high view); the eye direction is
// taken at REBUILD time, and rebuilds re-orient the cards as the eye moves.
// baseCenter is sunk into the ground so no gap opens on uneven terrain.
// The lighting normal stays the ground's radial up - a billboard whose
// normal tracks the eye shades like a wall (dark the moment the sun steps
// behind it); foliage lit with the terrain's up shares the ground's sun
// response, the classic billboard-foliage trick.
// UV convention (the renderer's, not the texture generator's internal
// math): the sampler reads the pixel buffer's FIRST row at v=0, and image
// files decode top-row-first, so v=0 is the TOP of a sprite and v=1 its
// base. The two base corners therefore carry v=1 and the two top corners
// v=0 - assigning them the other way draws every plant upside-down
// (canopy skirt at the crown, blade tips stabbing the ground).
void pushBillboard(std::vector<PlanetVertex> &vertices, std::vector<std::uint32_t> &indices,
    const glm::dvec3 &baseCenter, const glm::dvec3 &up, glm::dvec3 toEye, double halfWidth, double height,
    const std::array<float, 3> &color, const glm::dvec3 &anchor, const SpeciesUvRect &uvRect,
    bool mirrorU = false, float rootDarken = 0.0f)
{
    glm::dvec3 right = glm::cross(up, toEye);
    if (glm::length(right) < 1.0e-6)
    {
        right = glm::dvec3(1.0, 0.0, 0.0);
    }
    right = glm::normalize(right);
    glm::dvec3 facing = glm::normalize(glm::cross(toEye, right)); // card's vertical, ⊥ sight line
    QuadPush quad;
    quad.normal = up;
    // Root shading: the card's two base corners darken toward the ground -
    // the occlusion a plant's own base sits in (no light reaches the bottom
    // of a tuft under its own blades). The top corners keep the full tint.
    // Horizontal mirror: half the instances flip the sprite's U axis, so a
    // stand of one species photographs two silhouettes instead of one.
    const float baseShade = 1.0f - rootDarken;
    for (std::uint32_t corner = 0; corner < 4; ++corner)
    {
        const float shade = corner < 2u ? baseShade : 1.0f;
        quad.color[0] = color[0] * shade;
        quad.color[1] = color[1] * shade;
        quad.color[2] = color[2] * shade;
    }
    const float uL = mirrorU ? uvRect.u1 : uvRect.u0;
    const float uR = mirrorU ? uvRect.u0 : uvRect.u1;
    const float uv[4][2] = {{uL, uvRect.v1}, {uR, uvRect.v1}, {uR, uvRect.v0}, {uL, uvRect.v0}};
    for (std::uint32_t corner = 0; corner < 4; ++corner)
    {
        quad.uv[corner][0] = uv[corner][0];
        quad.uv[corner][1] = uv[corner][1];
    }
    quad.corners[0] = baseCenter - right * halfWidth;
    quad.corners[1] = baseCenter + right * halfWidth;
    quad.corners[2] = baseCenter + right * halfWidth + facing * height;
    quad.corners[3] = baseCenter - right * halfWidth + facing * height;
    pushQuad(vertices, indices, quad, anchor);
}

// One sun-aligned contact shadow: a soft dark quad laid tangent to the
// ground under a scattered element, shifted and stretched along the
// away-from-sun direction by the site's sun elevation (a low sun drags the
// shadow long and pale, a high sun bunches it under the element). The
// darkness rides the quad's vertex color over a white-RGB alpha texture, so
// stretched shadows simply drop toward grey. Lifted slightly off the ground
// so the terrain's own depth wins without polygon-offset support.
void PlanetScatterSystem::appendShadow(const glm::dvec3 &groundPoint, const glm::dvec3 &up, double sizeMeters,
    double heightMeters)
{
    if (params_.surfaceShadowStrength <= 0.0 || !initialized_ || sizeMeters <= 0.0 || heightMeters <= 0.0)
    {
        return;
    }
    if (glm::length(groundPoint - anchorEye_) > kShadowMaxDistanceMeters)
    {
        return; // beyond the readable ring the fill cost buys nothing
    }
    const double sunLength = glm::length(lastSunDirection_);
    if (sunLength < 1.0e-9)
    {
        return;
    }
    const glm::dvec3 sunDir = lastSunDirection_ / sunLength;
    const double sunUp = glm::dot(sunDir, up);
    if (sunUp < 0.10)
    {
        return; // sun at or under the site horizon casts no readable shadow
    }
    const glm::dvec3 sunHorizontal = sunDir - up * sunUp;
    const double horizontal = glm::length(sunHorizontal);
    if (horizontal < 1.0e-6)
    {
        return; // sun at zenith: the element covers its own shadow footprint
    }
    // Shadow length ~ height / tan(elevation), clamped to the reach even a
    // sunset shadow keeps readable through the soft falloff. The quad spans
    // from half a footprint BEHIND the trunk to the stretched toe, so the
    // shaded patch always touches the element that casts it.
    const double stretchMeters = std::clamp(heightMeters * horizontal / std::max(0.12, sunUp),
        0.0, 2.5 * heightMeters);
    const double baseExtent = std::max(sizeMeters * 0.45, 0.5);
    const glm::dvec3 shadowDir = -(sunHorizontal / horizontal);
    const glm::dvec3 center = groundPoint + shadowDir * (0.5 * stretchMeters)
        + up * 0.12;
    const double halfLength = 0.5 * stretchMeters + 0.5 * baseExtent;
    const double halfWidth = sizeMeters;
    glm::dvec3 across = glm::cross(up, shadowDir);
    if (glm::length(across) < 1.0e-6)
    {
        across = glm::dvec3(1.0, 0.0, 0.0);
    }
    across = glm::normalize(across);
    // The source RGB is black either way (src-over darkening); the falloff
    // lives in the material's opacity alone (the pipeline has no per-vertex
    // alpha to carry a stretch-dependent paleness).
    QuadPush quad;
    quad.normal = up;
    quad.color[0] = 0.0f;
    quad.color[1] = 0.0f;
    quad.color[2] = 0.0f;
    const float uv[4][2] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
    for (std::uint32_t corner = 0; corner < 4; ++corner)
    {
        quad.uv[corner][0] = uv[corner][0];
        quad.uv[corner][1] = uv[corner][1];
    }
    quad.corners[0] = center - across * halfWidth - shadowDir * halfLength;
    quad.corners[1] = center + across * halfWidth - shadowDir * halfLength;
    quad.corners[2] = center + across * halfWidth + shadowDir * halfLength;
    quad.corners[3] = center - across * halfWidth + shadowDir * halfLength;
    pushQuad(shadowScratch_, shadowIndexScratch_, quad, anchorEye_);
}

void PlanetScatterSystem::scatterVegetation(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north)
{
    (void)eyeDirection;
    if (!params_.vegetationEnabled)
    {
        treeMesh_.active = false;
        bushMesh_.active = false;
        grassMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    const double grassRadius = std::max(2.0, params_.vegetationGrassRadiusMeters);
    const double treeRadius = std::max(grassRadius + 10.0, params_.vegetationTreeRadiusMeters);
    const double bushRadius = std::max(8.0, std::min(460.0, treeRadius * 0.33));

    // The grove clump field: one fBm stream shared by every tree candidate,
    // keyed off the planet seed (forests read as groves with clearings,
    // savanna trees as isolated stands).
    const engine::planet::FbmStreamSeeds clumpSeeds
        = engine::planet::makeFbmSeeds(engine::planet::hashCombine64(params_.seed, 0x7EE51D2ull), 3);
    const engine::planet::FbmParams clumpParams{3, 2.0, 0.5};

    std::uint32_t treeCount = 0;
    if (params_.vegetationTreeDensity > 0.0)
    {
        std::vector<PlanetVertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(1024);
        indices.reserve(1536);
        constexpr std::uint32_t kMaxTrees = 1200;
        // Dense forest biomes double up: a passing cell plants a companion
        // tree at its own hashed offset (the lattice cell stays the walk's
        // cost unit, so the forest floor thickens without doubling the
        // surface and climate samples). Companions draw from the same size
        // band with an independent hash lane, so stands read as mixed-age
        // growth instead of cloned pairs.
        const auto denseStand = [](Biome biome)
        {
            return biome == Biome::Taiga || biome == Biome::TropicalRainforest
                               || biome == Biome::TemperateRainforest
                               || biome == Biome::TemperateForest
                               || biome == Biome::SeasonalForest;
        };
        walkLattice(eyePlanetLocal, east, north, 12.0, treeRadius, kTreeCellMeters,
            [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double cellDistance)
            {
                if (treeCount >= kMaxTrees)
                {
                    return;
                }
                const std::uint64_t hash = cellHash(params_.seed, kSaltTree, cx, cy, cz);
                if (hashUnit(hash, 0) > 0.60)
                {
                    return; // cheap pre-gate: most cells hold no tree attempt
                }
                const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kTreeCellMeters * 0.7
                    + north * (hashUnit(hash, 2) - 0.5) * kTreeCellMeters * 0.7;
                const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
                double radius = 0.0;
                if (!sampleSurfaceRadius(direction, radius))
                {
                    return;
                }
                if (radius < seaRadius + kTreeMinElevationMeters)
                {
                    return; // water, intertidal sand and exposed dunes stay bare
                }
                ClimateSample sample;
                const double directionArray[3] = {direction.x, direction.y, direction.z};
                if (capability_.sampleClimate(capability_.self, planet_, directionArray, radius, &sample) == false)
                {
                    return;
                }
                const double base = treeBaseDensity(sample.biome);
                if (base <= 0.0)
                {
                    return;
                }
                const glm::dvec3 point = direction * radius;
                const double clump = engine::planet::fbm(clumpSeeds, Vec3{point.x, point.y, point.z}, 280.0, clumpParams);
                double clumpGate = smoothStepLocal(-0.55, 0.25, clump);
                // Dense forest biomes keep their canopy floor: the boreal and
                // rain forests are continuous stands with only small
                // clearings, so the grove field floors instead of opening
                // empty country (a taiga spot in a grove-field minimum
                // rendered two trees per frame).
                const bool denseForest = sample.biome == Biome::Taiga
                    || sample.biome == Biome::TropicalRainforest
                    || sample.biome == Biome::TemperateRainforest;
                if (denseForest)
                {
                    clumpGate = std::max(clumpGate, 0.62);
                }
                double presence
                    = base * (0.45 + 0.55 * sample.moisture01) * clumpGate * params_.vegetationTreeDensity;
                if (denseForest)
                {
                    // The continuous stands keep a presence floor: the moisture
                    // term collapses over cold boreal latitudes whose field
                    // values sit low even though the forest there is solid
                    // (the grove gate above already carries the clumping).
                    presence = std::max(presence, 0.78 * params_.vegetationTreeDensity);
                }
                if (hashUnit(hash, 3) > clamp01(presence))
                {
                    return;
                }
                // Slope gate: cliff faces carry no soil, so nothing grows on
                // them. The climate sample always carries a flat-ground slope
                // (it reads the flats at the cliff's feet), so without this
                // gate trees planted straight onto vertical rock - perched
                // trunks on overhangs and mid-wall ledges. The relief field's
                // cliff factor is the same terraced-country mask the mesher's
                // bare-rock row keys on; only actual cliff-grade steepness
                // opens it, so hillsides and terrace benches keep their trees.
                {
                    engine::planet::ReliefSample relief{};
                    if (capability_.sampleRelief(capability_.self, planet_, directionArray, &relief)
                        && relief.cliffFactor >= params_.cliffSteepnessLow)
                    {
                        return;
                    }
                }
                const SpeciesCard species = treeSpeciesFor(sample.biome, hash);
                const double height = species.minHeightMeters
                    + hashUnit(hash, 4) * (species.maxHeightMeters - species.minHeightMeters);
                const double halfWidth = 0.5 * height * species.aspect * (0.62 + 0.72 * hashUnit(hash, 5));
                const glm::dvec3 up = direction;
                // Trunk foot sunk a tenth of the height: the drawn terrain at
                // any LOD sits at or under the density surface on convex
                // ground, and a sprite anchored exactly at the surface can
                // float above the coarse mesh's ridge - the detached-trunk
                // look distant views show. A deeper burial reads as the
                // trunk's root flare disappearing into soil.
                const glm::dvec3 baseCenter = point - up * (0.10 * height);
                // The per-instance tint range spans half the multiplier's
                // dynamic range: at billboard distances the eye averages a
                // narrow band into one tone, so the stand's variation must
                // survive the minification.
                std::array<float, 3> color
                    = speciesTint(sample, kTreeTint, 0.70 + 0.60 * hashUnit(hash, 6));
                if (species.tile == 2)
                {
                    // Conifers carry their own spruce tone: cooler and deeper
                    // than the broadleaf multiply, but not near-black - the
                    // needle sprite is already dark, and the multiply must
                    // stay in the shaded-foliage band.
                    color[0] *= 0.86f;
                    color[1] *= 1.00f;
                    color[2] *= 0.90f;
                }
                // A distant canopy minifies to a handful of pixels and reads
                // against the muted ground mass; a fully saturated species
                // tint turns those pixels into neon chips (the classic
                // billboard tell). Real distant forests lose saturation into
                // the haze and read close to the ground tone they stand on,
                // so the tint carries the planting site's own albedo in with
                // distance - full species color at the eye, ground-faded
                // beyond the mid ring.
                const double groundFade = smoothStepLocal(160.0, 620.0, cellDistance);
                const float groundMix = static_cast<float>(0.55 * groundFade);
                // Cloud shade: the same field the terrain's vertex albedo
                // carries (see the mesher's macro sample). A bank's shadow
                // darkens the ground AND everything standing on it - without
                // the multiply, foliage glows like a sticker against its own
                // shaded ground.
                const float cloudShade = static_cast<float>(clamp01(
                    1.0 - shadowField_.cloudShadow(engine::planet::Vec3{direction.x, direction.y, direction.z},
                        params_.derivedVoxelMeters)));
                for (std::uint32_t channel = 0; channel < 3; ++channel)
                {
                    color[channel] = std::lerp(color[channel],
                        clamp01(static_cast<float>(sample.color[channel]) * 0.9f), groundMix) * cloudShade;
                }
                pushBillboard(vertices, indices, baseCenter, up, anchorEye_ - baseCenter, halfWidth,
                    height, color, anchorEye_, speciesUv(species.tile), hashUnit(hash, 10) < 0.5, 0.35f);
                appendShadow(point, up, halfWidth * 1.15, height * 0.82);
                ++treeCount;
                // The companion lane follows the configured tree density the
                // way the primary lane does: thinning the forests must thin
                // BOTH stands of a cell, not leave the companion rate at full
                // strength inside whatever cells still plant a primary. The
                // full-density fill stays 0.72 - a mixed-age companion in
                // roughly three of four forest cells reads as a stand, while
                // a 1.0 rate would butt canopies into hedges.
                if (treeCount < kMaxTrees && denseStand(sample.biome)
                    && hashUnit(hash, 9) < 0.72 * clamp01(params_.vegetationTreeDensity))
                {
                    const std::uint64_t companionHash = cellHash(params_.seed, kSaltTree ^ 0xA5A5A5A5A5A5A5A5ull,
                        cx * 2 + 1, cy * 2 + 1, cz * 2 + 1);
                    const glm::dvec3 companionJitter = east * (hashUnit(companionHash, 1) - 0.5) * kTreeCellMeters * 0.8
                        + north * (hashUnit(companionHash, 2) - 0.5) * kTreeCellMeters * 0.8;
                    const glm::dvec3 companionDirection = glm::normalize(cellCenter + companionJitter);
                    double companionRadius = 0.0;
                    if (sampleSurfaceRadius(companionDirection, companionRadius)
                        && companionRadius >= seaRadius + kTreeMinElevationMeters)
                    {
                        const SpeciesCard companionSpecies = treeSpeciesFor(sample.biome, companionHash);
                        const double companionHeight = companionSpecies.minHeightMeters
                            + hashUnit(companionHash, 4)
                                * (companionSpecies.maxHeightMeters - companionSpecies.minHeightMeters);
                        const double companionHalfWidth = 0.5 * companionHeight * companionSpecies.aspect
                            * (0.72 + 0.56 * hashUnit(companionHash, 5));
                        const glm::dvec3 companionPoint = companionDirection * companionRadius;
                        const glm::dvec3 companionBase = companionPoint - up * (0.10 * companionHeight);
                        std::array<float, 3> companionColor
                            = speciesTint(sample, kTreeTint, 0.70 + 0.60 * hashUnit(companionHash, 6));
                        if (companionSpecies.tile == 2)
                        {
                            companionColor[0] *= 0.86f;
                            companionColor[1] *= 1.00f;
                            companionColor[2] *= 0.90f;
                        }
                        const double companionGroundFade = smoothStepLocal(160.0, 620.0, cellDistance);
                        const float companionGroundMix = static_cast<float>(0.55 * companionGroundFade);
                        for (std::uint32_t channel = 0; channel < 3; ++channel)
                        {
                            companionColor[channel] = std::lerp(companionColor[channel],
                                clamp01(static_cast<float>(sample.color[channel]) * 0.9f), companionGroundMix)
                                * cloudShade;
                        }
                        pushBillboard(vertices, indices, companionBase, up,
                            anchorEye_ - companionBase, companionHalfWidth, companionHeight, companionColor,
                            anchorEye_, speciesUv(companionSpecies.tile), hashUnit(companionHash, 10) < 0.5, 0.35f);
                        appendShadow(companionPoint, up, companionHalfWidth * 1.15, companionHeight * 0.82);
                        ++treeCount;
                    }
                }
            });
        treeMesh_.anchorPlanetLocal = anchorEye_;
        uploadMesh(treeMesh_, vertices, indices, true, false, false, 1.0f, false); // lit foliage keeps its ground-up normal
        treeMesh_.material.baseColorTexture = treeTexture_;
    }
    else
    {
        treeMesh_.active = false;
    }

    std::uint32_t bushCount = 0;
    if (params_.vegetationBushDensity > 0.0)
    {
        std::vector<PlanetVertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(512);
        indices.reserve(768);
        constexpr std::uint32_t kMaxBushes = 220;
        walkLattice(eyePlanetLocal, east, north, 4.0, bushRadius, kBushCellMeters,
            [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double cellDistance)
            {
                if (bushCount >= kMaxBushes)
                {
                    return;
                }
                const std::uint64_t hash = cellHash(params_.seed, kSaltBush, cx, cy, cz);
                if (hashUnit(hash, 0) > 0.55)
                {
                    return;
                }
                const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kBushCellMeters * 0.75
                    + north * (hashUnit(hash, 2) - 0.5) * kBushCellMeters * 0.75;
                const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
                double radius = 0.0;
                if (!sampleSurfaceRadius(direction, radius) || radius < seaRadius + kMinLandElevationMeters)
                {
                    return;
                }
                ClimateSample sample;
                const double directionArray[3] = {direction.x, direction.y, direction.z};
                if (capability_.sampleClimate(capability_.self, planet_, directionArray, radius, &sample) == false)
                {
                    return;
                }
                const double base = bushBaseDensity(sample.biome);
                if (base <= 0.0)
                {
                    return;
                }
                const double presence = base * (0.5 + 0.5 * sample.moisture01) * params_.vegetationBushDensity;
                if (hashUnit(hash, 3) > clamp01(presence))
                {
                    return;
                }
                // Slope gate: see the tree walk - cliff faces carry no soil.
                {
                    engine::planet::ReliefSample relief{};
                    if (capability_.sampleRelief(capability_.self, planet_, directionArray, &relief)
                        && relief.cliffFactor >= params_.cliffSteepnessLow)
                    {
                        return;
                    }
                }
                const SpeciesCard species = bushSpeciesFor(sample.biome, hash);
                const double height = species.minHeightMeters
                    + hashUnit(hash, 4) * (species.maxHeightMeters - species.minHeightMeters);
                const double halfWidth = 0.5 * height * species.aspect * (1.05 + 0.35 * hashUnit(hash, 5));
                const glm::dvec3 up = direction;
                const glm::dvec3 baseCenter = direction * radius - up * (0.25 * height);
                std::array<float, 3> color
                    = speciesTint(sample, kBushTint, 0.88 + 0.26 * hashUnit(hash, 6));
                // Same distant-chip guard as the trees, weaker: bushes stay
                // inside the mid ring where the species tint still reads.
                const double groundFade = smoothStepLocal(200.0, 620.0, cellDistance);
                const float groundMix = static_cast<float>(0.40 * groundFade);
                // Cloud shade (see the tree walk): sprites darken with the
                // ground under the same bank.
                const float bushCloudShade = static_cast<float>(clamp01(
                    1.0 - shadowField_.cloudShadow(engine::planet::Vec3{direction.x, direction.y, direction.z},
                        params_.derivedVoxelMeters)));
                for (std::uint32_t channel = 0; channel < 3; ++channel)
                {
                    color[channel] = std::lerp(color[channel],
                        clamp01(static_cast<float>(sample.color[channel]) * 0.9f), groundMix) * bushCloudShade;
                }
                pushBillboard(vertices, indices, baseCenter, up, anchorEye_ - baseCenter, halfWidth,
                    height, color, anchorEye_, speciesUv(species.tile), hashUnit(hash, 7) < 0.5, 0.40f);
                appendShadow(direction * radius, up, halfWidth * 0.95, height * 0.9);
                ++bushCount;
            });
        bushMesh_.anchorPlanetLocal = anchorEye_;
        uploadMesh(bushMesh_, vertices, indices, true, false, false, 1.0f, false); // lit foliage keeps its ground-up normal
        bushMesh_.material.baseColorTexture = bushTexture_;
    }
    else
    {
        bushMesh_.active = false;
    }

    std::uint32_t grassCount = 0;
    if (params_.vegetationGrassDensity > 0.0)
    {
        std::vector<PlanetVertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(4096);
        indices.reserve(6144);
        // Two-scale sward: the near disc plants tufts on a 0.62 m lattice with
        // CROSSED cards (the sward reads as volume underfoot); the mid annulus
        // thins onto a 1.4 m lattice with single cards (crossing is invisible
        // past arm's reach). The split keeps the fill cost of the near disc
        // while the old one-scale 0.62 m ring across the full radius doubled
        // the frame for no visual gain beyond nine meters.
        constexpr std::uint32_t kMaxGrassNear = 1500;
        constexpr std::uint32_t kMaxGrassMid = 950;
        const double nearDiscMeters = std::min(grassRadius, 7.0);
        walkLattice(eyePlanetLocal, east, north, 1.8, nearDiscMeters, kGrassCellMeters,
            [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double cellDistance)
            {
                if (grassCount >= kMaxGrassNear)
                {
                    return;
                }
                const std::uint64_t hash = cellHash(params_.seed, kSaltGrass, cx, cy, cz);
                if (hashUnit(hash, 0) > 0.9)
                {
                    return;
                }
                const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kGrassCellMeters * 1.0
                    + north * (hashUnit(hash, 2) - 0.5) * kGrassCellMeters * 1.0;
                const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
                double radius = 0.0;
                if (!sampleSurfaceRadius(direction, radius) || radius < seaRadius + kMinLandElevationMeters * 0.6)
                {
                    return;
                }
                ClimateSample sample;
                const double directionArray[3] = {direction.x, direction.y, direction.z};
                if (capability_.sampleClimate(capability_.self, planet_, directionArray, radius, &sample) == false)
                {
                    return;
                }
                const double base = grassBaseDensity(sample.biome);
                if (base <= 0.0)
                {
                    return;
                }
                // Moisture drives the sward: wet sites fill in nearly solid,
                // dry sites thin to the sparse arid card. The presence gate
                // and the height band both key on it, so a savanna reads dry
                // and short while a rainforest floor reads lush and deep.
                const double grassRadiusMeters = std::max(2.0, params_.vegetationGrassRadiusMeters);
                const double ringFade = smoothStepLocal(grassRadiusMeters, grassRadiusMeters * 0.55, cellDistance);
                const double presence = base * (0.55 + 0.45 * sample.moisture01) * ringFade
                    * params_.vegetationGrassDensity;
                if (hashUnit(hash, 3) > clamp01(presence))
                {
                    return;
                }
                // Slope gate: see the tree walk - cliff faces carry no soil,
                // and a sward of tufts on vertical rock reads as noise.
                {
                    engine::planet::ReliefSample relief{};
                    if (capability_.sampleRelief(capability_.self, planet_, directionArray, &relief)
                        && relief.cliffFactor >= params_.cliffSteepnessLow)
                    {
                        return;
                    }
                }
                const SpeciesCard species = grassSpeciesFor(sample.biome, hash);
                const double moisture = clamp01(static_cast<float>(sample.moisture01));
                const double height = (species.minHeightMeters
                    + hashUnit(hash, 4) * (species.maxHeightMeters - species.minHeightMeters))
                    * (0.78 + 0.44 * moisture) * 1.30;
                const double halfWidth = 0.5 * height * species.aspect * (0.95 + 0.5 * hashUnit(hash, 5));
                const glm::dvec3 up = direction;
                const glm::dvec3 center = direction * radius - up * (0.08 * height);
                glm::dvec3 right1 = glm::cross(up, east);
                if (glm::length(right1) < 1.0e-6)
                {
                    right1 = north;
                }
                right1 = glm::normalize(right1);
                const double angle = hashUnit(hash, 6) * 2.0 * kPi;
                const glm::dvec3 rotated = right1 * std::cos(angle) + glm::cross(up, right1) * std::sin(angle);
                const glm::dvec3 right2 = glm::normalize(glm::cross(up, rotated));
                right1 = glm::normalize(rotated);
                const std::array<float, 3> color = grassSwardTint(sample,
                    0.72 + 0.62 * hashUnit(hash, 7), 0.16 * (hashUnit(hash, 8) - 0.5));
                // Cloud shade (see the tree walk): the sward darkens with the
                // ground it stands on.
                const float grassCloudShade = static_cast<float>(clamp01(
                    1.0 - shadowField_.cloudShadow(engine::planet::Vec3{direction.x, direction.y, direction.z},
                        params_.derivedVoxelMeters)));
                const std::array<float, 3> shadedColor{color[0] * grassCloudShade,
                    color[1] * grassCloudShade, color[2] * grassCloudShade};
                const SpeciesUvRect grassUv = speciesUv(species.tile);
                pushBillboard(vertices, indices, center, up, right1, halfWidth, height, shadedColor,
                    anchorEye_, grassUv, hashUnit(hash, 9) < 0.5, 0.45f);
                pushBillboard(vertices, indices, center, up, right2, halfWidth, height, shadedColor,
                    anchorEye_, grassUv, hashUnit(hash, 10) < 0.5, 0.45f);
                // Contact shadow: the tuft's own footprint darkens the soil
                // right under the blades. Without it the cutout sprite's hard
                // bottom edge sits on undarkened ground and the tuft reads as
                // a painted sticker floating on the soil - the decal the
                // trees wear, scaled to the tuft (a sward clump's shadow is
                // barely wider than the clump and a fraction of its height,
                // since the blades are mostly transparent to the sun).
                appendShadow(direction * (radius - 0.30 * height), up, halfWidth * 1.35, height * 0.28);
                ++grassCount;
            });
        // Mid annulus (near disc edge .. full radius): coarser lattice,
        // single card per tuft - the crossed pair is invisible beyond a few
        // meters and costs a full extra card of fill per tuft.
        if (grassRadius > nearDiscMeters)
        {
            walkLattice(eyePlanetLocal, east, north, nearDiscMeters, grassRadius, kGrassMidCellMeters,
                [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double cellDistance)
                {
                    if (grassCount >= kMaxGrassNear + kMaxGrassMid)
                    {
                        return;
                    }
                    const std::uint64_t hash = cellHash(params_.seed, kSaltGrass, cx, cy, cz);
                    if (hashUnit(hash, 0) > 0.9)
                    {
                        return;
                    }
                    const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kGrassMidCellMeters * 1.0
                        + north * (hashUnit(hash, 2) - 0.5) * kGrassMidCellMeters * 1.0;
                    const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
                    double radius = 0.0;
                    if (!sampleSurfaceRadius(direction, radius) || radius < seaRadius + kMinLandElevationMeters * 0.6)
                    {
                        return;
                    }
                    ClimateSample sample;
                    const double directionArray[3] = {direction.x, direction.y, direction.z};
                    if (capability_.sampleClimate(capability_.self, planet_, directionArray, radius, &sample) == false)
                    {
                        return;
                    }
                    const double base = grassBaseDensity(sample.biome);
                    if (base <= 0.0)
                    {
                        return;
                    }
                    const double ringFade = smoothStepLocal(grassRadius, grassRadius * 0.55, cellDistance);
                    const double presence = base * (0.55 + 0.45 * sample.moisture01) * ringFade
                        * params_.vegetationGrassDensity;
                    if (hashUnit(hash, 3) > clamp01(presence))
                    {
                        return;
                    }
                    // Slope gate: see the tree walk - cliff faces carry no soil.
                    {
                        engine::planet::ReliefSample relief{};
                        if (capability_.sampleRelief(capability_.self, planet_, directionArray, &relief)
                            && relief.cliffFactor >= params_.cliffSteepnessLow)
                        {
                            return;
                        }
                    }
                    const SpeciesCard species = grassSpeciesFor(sample.biome, hash);
                    const double moisture = clamp01(static_cast<float>(sample.moisture01));
                    const double height = (species.minHeightMeters
                        + hashUnit(hash, 4) * (species.maxHeightMeters - species.minHeightMeters))
                        * (0.78 + 0.44 * moisture);
                    const double halfWidth = 0.5 * height * species.aspect * (0.95 + 0.5 * hashUnit(hash, 5));
                    const glm::dvec3 up = direction;
                    // Mid-annulus tufts sink well into the ground: at this
                    // distance the rendered terrain can sit under the exact
                    // density surface by a voxel or more (a coarse LOD mesh
                    // cuts a convex ridge's top off), and a sprite anchored
                    // at the exact surface then FLOATS above the drawn
                    // ground - the sky-background specks distant views show.
                    // A quarter-height burial keeps every tuft's foot inside
                    // the drawn surface at any LOD while the blades still
                    // emerge from the soil.
                    const glm::dvec3 center = direction * radius - up * (0.30 * height);
                    glm::dvec3 right1 = glm::cross(up, east);
                    if (glm::length(right1) < 1.0e-6)
                    {
                        right1 = north;
                    }
                    right1 = glm::normalize(right1);
                    const double angle = hashUnit(hash, 6) * 2.0 * kPi;
                    const glm::dvec3 rotated = right1 * std::cos(angle) + glm::cross(up, right1) * std::sin(angle);
                    right1 = glm::normalize(rotated);
                    const std::array<float, 3> color = grassSwardTint(sample,
                        0.72 + 0.62 * hashUnit(hash, 7), 0.16 * (hashUnit(hash, 8) - 0.5));
                    // Cloud shade (see the near disc).
                    const float midCloudShade = static_cast<float>(clamp01(
                        1.0 - shadowField_.cloudShadow(engine::planet::Vec3{direction.x, direction.y, direction.z},
                            params_.derivedVoxelMeters)));
                    const std::array<float, 3> midShaded{color[0] * midCloudShade,
                        color[1] * midCloudShade, color[2] * midCloudShade};
                    pushBillboard(vertices, indices, center, up, right1, halfWidth, height, midShaded,
                        anchorEye_, speciesUv(species.tile), hashUnit(hash, 9) < 0.5, 0.45f);
                    ++grassCount;
                });
        }
        grassMesh_.anchorPlanetLocal = anchorEye_;
        uploadMesh(grassMesh_, vertices, indices, true, false, false, 1.0f, false); // cutout blades keep hard edges
        grassMesh_.material.baseColorTexture = grassTexture_;
    }
    else
    {
        grassMesh_.active = false;
    }
}

bool PlanetScatterSystem::sampleSurfaceRadius(const glm::dvec3 &direction, double &outRadius) const
{
    const double directionArray[3] = {direction.x, direction.y, direction.z};
    return capability_.sampleSurfaceRadius(capability_.self, planet_, directionArray, &outRadius);
}

void PlanetScatterSystem::scatterRocks(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north)
{
    (void)eyeDirection;
    if (params_.surfaceRockDensity <= 0.0)
    {
        rockMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    const double rockRadius = std::max(20.0, params_.surfaceRockRadiusMeters);
    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(2048);
    indices.reserve(2048);
    std::uint32_t rockCount = 0;
    constexpr std::uint32_t kMaxRocks = 48;

    walkLattice(eyePlanetLocal, east, north, 3.0, rockRadius, kRockCellMeters,
        [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double)
        {
            if (rockCount >= kMaxRocks)
            {
                return;
            }
            const std::uint64_t hash = cellHash(params_.seed, kSaltRock, cx, cy, cz);
            if (hashUnit(hash, 0) > 0.30)
            {
                return;
            }
            const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kRockCellMeters * 0.7
                + north * (hashUnit(hash, 2) - 0.5) * kRockCellMeters * 0.7;
            const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
            double radius = 0.0;
            if (!sampleSurfaceRadius(direction, radius) || radius < seaRadius + kMinLandElevationMeters * 0.6)
            {
                return;
            }
            ClimateSample sample;
            const double directionArray[3] = {direction.x, direction.y, direction.z};
            if (capability_.sampleClimate(capability_.self, planet_, directionArray, radius, &sample) == false)
            {
                return;
            }
            const double presence = rockBaseDensity(sample.biome) * params_.surfaceRockDensity;
            if (hashUnit(hash, 3) > clamp01(presence))
            {
                return;
            }
            // Low-poly displaced octahedron: eight faces with flat facets,
            // radially displaced so no two rocks share a silhouette, sunk a
            // third into the ground.
            const double rockSize = 0.3 + hashUnit(hash, 4) * 1.4;
            const double sink = rockSize * 0.34;
            const glm::dvec3 center = direction * (radius - sink);
            glm::dvec3 up = direction;
            glm::dvec3 axis = east;
            glm::dvec3 bit = north;
            // Octahedron corners in the local tangent frame.
            const glm::dvec3 cornerDirs[6] = {up, -up, axis, -axis, bit, -bit};
            glm::dvec3 corners[6];
            for (std::uint32_t corner = 0; corner < 6; ++corner)
            {
                const double displacement = 0.72 + 0.5 * hashUnit(hash, 10 + corner);
                corners[corner] = center + cornerDirs[corner] * (rockSize * displacement);
            }
            // The eight faces of the octahedron (corner index triplets),
            // wound double-sided-safe; rock material declares doubleSided.
            const std::uint32_t faces[8][3] = {
                {0, 2, 4}, {0, 4, 3}, {0, 3, 5}, {0, 5, 2},
                {1, 4, 2}, {1, 3, 4}, {1, 5, 3}, {1, 2, 5},
            };
            const float grey = 0.52f + 0.28f * static_cast<float>(hashUnit(hash, 5));
            std::array<float, 3> color = {clamp01(static_cast<float>(sample.color[0]) * 0.55f + grey * 0.45f),
                clamp01(static_cast<float>(sample.color[1]) * 0.55f + grey * 0.45f),
                clamp01(static_cast<float>(sample.color[2]) * 0.55f + grey * 0.45f)};
            for (const auto &face : faces)
            {
                const glm::dvec3 a = corners[face[0]];
                const glm::dvec3 b = corners[face[1]];
                const glm::dvec3 c = corners[face[2]];
                const glm::dvec3 faceNormal = glm::normalize(glm::cross(b - a, c - a));
                QuadPush quad;
                quad.normal = faceNormal;
                quad.color[0] = color[0];
                quad.color[1] = color[1];
                quad.color[2] = color[2];
                quad.corners[0] = a;
                quad.corners[1] = b;
                quad.corners[2] = c;
                quad.corners[3] = c; // unused by the first triangle's winding? pushQuad emits 2 tris; degenerate second
                const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
                for (std::uint32_t corner = 0; corner < 3; ++corner)
                {
                    PlanetVertex vertex;
                    const glm::dvec3 position = quad.corners[corner] - anchorEye_;
                    vertex.position[0] = static_cast<float>(position.x);
                    vertex.position[1] = static_cast<float>(position.y);
                    vertex.position[2] = static_cast<float>(position.z);
                    vertex.normal[0] = static_cast<float>(faceNormal.x);
                    vertex.normal[1] = static_cast<float>(faceNormal.y);
                    vertex.normal[2] = static_cast<float>(faceNormal.z);
                    vertex.color[0] = color[0];
                    vertex.color[1] = color[1];
                    vertex.color[2] = color[2];
                    vertex.uv[0] = 0.0f;
                    vertex.uv[1] = 0.0f;
                    vertices.push_back(vertex);
                }
                indices.push_back(base + 0u);
                indices.push_back(base + 1u);
                indices.push_back(base + 2u);
                (void)quad;
            }
            appendShadow(center, direction, rockSize * 1.2, rockSize * 0.9);
            ++rockCount;
        });

    rockMesh_.anchorPlanetLocal = anchorEye_;
    uploadMesh(rockMesh_, vertices, indices, false, false, false, 1.0f, false); // closed lit geometry, outward normals
}

void PlanetScatterSystem::scatterWhitecaps(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north, bool partial)
{
    (void)eyeDirection;
    if (!params_.whitecapsEnabled || params_.whitecapStrength <= 0.0 || !initialized_)
    {
        whitecapMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    // Breaking-crest coverage from the configured wind (whitecaps start near
    // 3 m/s; the fraction climbs roughly with the cube of the wind, clamped
    // to the share even a heavy sea never exceeds on film).
    const double wind = std::max(0.0, params_.whitecapWindSpeedMps - 2.5);
    const double coverage = std::min(0.045, 1.1e-5 * std::pow(wind, 3.1)) * params_.whitecapStrength;

    // Fixed-capacity slot pool: slots bind to lattice cells at the anchor
    // rebuild; the churn ticks re-roll a STAGGERED QUARTER of them in place
    // (slot i rolls over when (refreshCounter_ + i) % 4 == 0), so three
    // quarters of the sea's foam persists across any tick and the field
    // turns over the way real breaking foam does - no wholesale re-deal,
    // no vertex-count churn (the mesh refreshes via updateMeshVertices).
    constexpr std::uint32_t kMaxWhitecaps = 150;
    const std::uint64_t presenceSalt = kSaltWhitecap
        + (partial ? static_cast<std::uint64_t>(refreshCounter_ / kWhitecapEpochTicks) * 0x100000ull : 0ull);

    if (!partial || whitecapSlots_.empty())
    {
        whitecapSlots_.clear();
        const std::uint64_t fullSalt = kSaltWhitecap;
        walkLattice(eyePlanetLocal, east, north, 55.0, 520.0, kFoamCellMeters,
            [&](std::int64_t cx, std::int64_t cy, std::int64_t cz, const glm::dvec3 &cellCenter, double)
            {
                if (whitecapSlots_.size() >= kMaxWhitecaps)
                {
                    return;
                }
                const std::uint64_t hash = cellHash(params_.seed, fullSalt, cx, cy, cz);
                // Presence gate: the sprite's foam core (the texture's
                // alpha>=0.5 region, ~10 m2 of the bigger quads) over the
                // cell's 676 m2 realizes the configured coverage fraction,
                // so the per-cell probability is coverage * cellArea / coreArea.
                if (hashUnit(hash, 0) > coverage * 676.0 / 10.0)
                {
                    return;
                }
                const glm::dvec3 jitter = east * (hashUnit(hash, 1) - 0.5) * kFoamCellMeters * 0.9
                    + north * (hashUnit(hash, 2) - 0.5) * kFoamCellMeters * 0.9;
                const glm::dvec3 direction = glm::normalize(cellCenter + jitter);
                double terrainRadius = 0.0;
                if (!sampleSurfaceRadius(direction, terrainRadius))
                {
                    return;
                }
                if (terrainRadius > seaRadius - kMinWhitecapDepthMeters)
                {
                    return; // the shallows break through the surf band, not here
                }
                WhitecapSlot slot;
                slot.direction = direction;
                slot.halfLength = 2.6 + hashUnit(hash, 3) * 4.4;
                slot.halfWidth = 1.2 + hashUnit(hash, 4) * 1.6;
                slot.slotIndex = static_cast<std::uint32_t>(whitecapSlots_.size());
                slot.alive = true;
                whitecapSlots_.push_back(slot);
            });
        // The slots' presence lane rotates per slot so churn is staggered.
    }

    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(kMaxWhitecaps * 4u);
    indices.reserve(kMaxWhitecaps * 6u);

    // Wind direction: one seeded world angle, projected per sprite, so the
    // streaks all run the same way a real wind's fetch does.
    const double windAngle = hashUnit(cellHash(params_.seed, kSaltWhitecap, 0, 0, 0), 99) * 2.0 * kPi;
    const glm::dvec3 windWorld = east * std::cos(windAngle) + north * std::sin(windAngle);

    std::uint32_t aliveCount = 0;
    for (std::uint32_t slotIndex = 0; slotIndex < whitecapSlots_.size(); ++slotIndex)
    {
        WhitecapSlot &slot = whitecapSlots_[slotIndex];
        const bool rollsThisTick
            = partial && ((refreshCounter_ + slotIndex) % kWhitecapEpochTicks == 0u);
        if (rollsThisTick)
        {
            const std::uint64_t churnHash = cellHash(params_.seed, presenceSalt, slotIndex, 0, 0);
            slot.alive = hashUnit(churnHash, 0) < coverage * 676.0 / 10.0;
        }
        if (!slot.alive)
        {
            continue;
        }
        ++aliveCount;
        const glm::dvec3 &direction = slot.direction;
        const glm::dvec3 up = direction;
        const glm::dvec3 center = direction * (seaRadius + kSeaSpriteLiftMeters);
        glm::dvec3 windTangent = windWorld - up * glm::dot(windWorld, up);
        if (glm::length(windTangent) < 1.0e-6)
        {
            windTangent = east;
        }
        windTangent = glm::normalize(windTangent);
        const glm::dvec3 crosswise = glm::normalize(glm::cross(up, windTangent));
        QuadPush quad;
        quad.normal = up;
        const float bright = 0.9f;
        quad.color[0] = bright;
        quad.color[1] = bright;
        quad.color[2] = bright;
        const float uv[4][2] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            quad.uv[corner][0] = uv[corner][0];
            quad.uv[corner][1] = uv[corner][1];
        }
        quad.corners[0] = center - windTangent * slot.halfLength - crosswise * slot.halfWidth;
        quad.corners[1] = center + windTangent * slot.halfLength - crosswise * slot.halfWidth;
        quad.corners[2] = center + windTangent * slot.halfLength + crosswise * slot.halfWidth;
        quad.corners[3] = center - windTangent * slot.halfLength + crosswise * slot.halfWidth;
        pushQuad(vertices, indices, quad, anchorEye_);
    }

    // Pad to capacity with degenerate quads so the churn path can refresh the
    // mesh in place at a constant vertex count (a vanished sprite's slots
    // become zero-area triangles that never rasterize a pixel).
    const std::uint32_t totalQuads = static_cast<std::uint32_t>(whitecapSlots_.size());
    while (vertices.size() / 4u < kMaxWhitecaps)
    {
        QuadPush degenerate;
        degenerate.normal = eyeDirection;
        degenerate.color[0] = 0.0f;
        degenerate.color[1] = 0.0f;
        degenerate.color[2] = 0.0f;
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            degenerate.uv[corner][0] = 0.0f;
            degenerate.uv[corner][1] = 0.0f;
            degenerate.corners[corner] = anchorEye_ - eyeDirection * 4.0;
        }
        pushQuad(vertices, indices, degenerate, anchorEye_);
    }
    (void)totalQuads;
    (void)aliveCount;
    whitecapMesh_.anchorPlanetLocal = anchorEye_;
    uploadMesh(whitecapMesh_, vertices, indices, false, true, true, 0.80f);
    whitecapMesh_.material.baseColorTexture = foamTexture_;
}

void PlanetScatterSystem::scatterSurf(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north)
{
    (void)eyeDirection;
    if (!params_.surfFoamEnabled || params_.surfFoamStrength <= 0.0 || !initialized_)
    {
        surfMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    const double strength = clamp01(params_.surfFoamStrength);
    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(1024);
    indices.reserve(1536);
    std::uint32_t stripCount = 0;
    constexpr std::uint32_t kMaxStrips = 140;

    // Shore march: walk constant-azimuth curves out from the eye, bisecting
    // for the crossing where the terrain radius meets the sea. Each crossing
    // gets a breaker band (seaward, where the waves pitch and break) and a
    // swash band (at the waterline, where the run-up washes the sand).
    const std::uint32_t azimuthCount = 52;
    const double maxMarchMeters = 700.0;
    for (std::uint32_t azimuthIndex = 0; azimuthIndex < azimuthCount; ++azimuthIndex)
    {
        if (stripCount >= kMaxStrips)
        {
            break;
        }
        const double azimuth = (static_cast<double>(azimuthIndex) + 0.35 * hashUnit(cellHash(params_.seed, kSaltSurf, static_cast<std::int64_t>(azimuthIndex), 0, 0), 0))
            * (2.0 * kPi / static_cast<double>(azimuthCount));
        const glm::dvec3 walk = east * std::cos(azimuth) + north * std::sin(azimuth);
        const auto shoreGap = [&](double distance)
        {
            const glm::dvec3 probe = glm::normalize(eyePlanetLocal + walk * distance);
            double radius = 0.0;
            if (!sampleSurfaceRadius(probe, radius))
            {
                return 1.0e9;
            }
            return radius - seaRadius;
        };
        const double near = 6.0;
        const double far = maxMarchMeters;
        const double gapNear = shoreGap(near);
        const double gapFar = shoreGap(far);
        if ((gapNear > 0.0) == (gapFar > 0.0))
        {
            continue; // this azimuth never crosses the waterline in range
        }
        double lo = gapNear > 0.0 ? near : far; // outside the water (land side)
        double hi = gapNear > 0.0 ? far : near; // inside the water
        for (std::uint32_t iteration = 0; iteration < 11; ++iteration)
        {
            const double mid = 0.5 * (lo + hi);
            if ((shoreGap(mid) > 0.0) == (shoreGap(lo) > 0.0))
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        const double crossing = 0.5 * (lo + hi);
        const glm::dvec3 shoreDirection = glm::normalize(eyePlanetLocal + walk * crossing);
        // Shore tangent: horizontal and perpendicular to the radial walk.
        glm::dvec3 shoreTangent = glm::cross(shoreDirection, walk);
        if (glm::length(shoreTangent) < 1.0e-6)
        {
            shoreTangent = east;
        }
        shoreTangent = glm::normalize(shoreTangent);
        const auto pushStrip = [&](double radialOffset, double halfAlong, double halfAcross, double lift)
        {
            if (stripCount >= kMaxStrips)
            {
                return;
            }
            const glm::dvec3 up = shoreDirection;
            const glm::dvec3 center = shoreDirection * (seaRadius + radialOffset);
            QuadPush quad;
            quad.normal = up;
            const float bright = 0.92f;
            quad.color[0] = bright;
            quad.color[1] = bright;
            quad.color[2] = bright;
            const float uv[4][2] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                quad.uv[corner][0] = uv[corner][0];
                quad.uv[corner][1] = uv[corner][1];
            }
            quad.corners[0] = center - shoreTangent * halfAlong - walk * halfAcross;
            quad.corners[1] = center + shoreTangent * halfAlong - walk * halfAcross;
            quad.corners[2] = center + shoreTangent * halfAlong + walk * halfAcross;
            quad.corners[3] = center - shoreTangent * halfAlong + walk * halfAcross;
            pushQuad(vertices, indices, quad, anchorEye_);
            ++stripCount;
            (void)lift;
        };
        const std::uint64_t stripHash = cellHash(params_.seed, kSaltSurf, static_cast<std::int64_t>(azimuthIndex), 1, 0);
        // Breaker band: 4-9 m seaward of the waterline, wider where the hash
        // says the swell is stronger; swash band: hugging the waterline.
        // Both bands GATE by hash (a breaker only where this swell train is
        // actively pitching, a swash only where the last run-up reached):
        // bands at every azimuth march around the shore as continuous
        // concentric stripes - the contour-map tell - while gated bands
        // leave irregular gaps the way real surf patches do.
        if (hashUnit(stripHash, 6) < 0.58)
        {
            pushStrip(-3.0 - 6.0 * hashUnit(stripHash, 1), 5.0 + 9.0 * hashUnit(stripHash, 2), 2.2 + 1.6 * hashUnit(stripHash, 3), 0.3);
        }
        if (hashUnit(stripHash, 7) < 0.45)
        {
            pushStrip(0.8 + 1.2 * hashUnit(stripHash, 8), 4.0 + 7.0 * hashUnit(stripHash, 4), 1.6 + 1.2 * hashUnit(stripHash, 5), 0.2);
        }
    }

    surfMesh_.anchorPlanetLocal = anchorEye_;
    uploadMesh(surfMesh_, vertices, indices, false, true, true, static_cast<float>(0.55 + 0.30 * strength));
    surfMesh_.material.baseColorTexture = foamTexture_;
}

void PlanetScatterSystem::scatterMist(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north, double sunUp)
{
    (void)eyeDirection;
    if (!params_.valleyMistEnabled || params_.valleyMistStrength <= 0.0 || !initialized_)
    {
        mistMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    const double eyeElevation = anchorEyeRadius_ - seaRadius;
    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(64);
    indices.reserve(96);
    std::uint32_t bankCount = 0;
    constexpr std::uint32_t kMaxBanks = 5;

    // Ground fog pools in low ground: probe a coarse ring around the eye and
    // pool wherever the ground sits well below the eye (valley floors, and
    // open water near the shore), denser toward sunrise (radiation fog peaks
    // around dawn, per the cold-air-drainage picture). The pools are
    // ground-hugging horizontal quads lying a few meters over the floor -
    // a vertical card reads as an opaque wall with hard edges from the
    // sample cameras; a wide flat sheet with the radial-gradient texture
    // reads as fog lying IN the valley.
    const std::uint32_t probeCount = 36;
    for (std::uint32_t probeIndex = 0; probeIndex < probeCount && bankCount < kMaxBanks; ++probeIndex)
    {
        const double azimuth = static_cast<double>(probeIndex) * (2.0 * kPi / static_cast<double>(probeCount));
        const glm::dvec3 walk = east * std::cos(azimuth) + north * std::sin(azimuth);
        const double distance = 240.0 + 300.0 * hashUnit(cellHash(params_.seed, kSaltMist, static_cast<std::int64_t>(probeIndex), 0, 0), 0);
        const glm::dvec3 probe = glm::normalize(eyePlanetLocal + walk * distance);
        double radius = 0.0;
        if (!sampleSurfaceRadius(probe, radius))
        {
            continue;
        }
        const double elevation = radius - seaRadius;
        const bool valley = elevation < eyeElevation - 24.0;
        // The open-water trigger belongs to EYES ON LAND looking out at a
        // nearby shore: a coastal eye at 120-140 m sees soft banks floating
        // on the sea beyond the waterline. An eye standing OVER the open
        // sea (the water-biome sample cameras at ~25 m above the surface)
        // sees every probe direction as open water, so the full bank budget
        // surrounds and buries the camera itself - the sample then
        // photographs an unbroken pale fog wash instead of the biome's
        // water. Eyes within 60 m of the sea surface (over-water cameras
        // and low shorelines) get no sea banks at all.
        const bool openWater = elevation < 0.0 && eyeElevation > 60.0;
        if (!valley && !openWater)
        {
            continue;
        }
        const std::uint64_t hash = cellHash(params_.seed, kSaltMist, static_cast<std::int64_t>(probeIndex), 1, 0);
        const double bankHeight = 6.0 + 9.0 * hashUnit(hash, 1);
        const double halfWidth = 150.0 + 170.0 * hashUnit(hash, 2);
        const double halfDepth = 0.55 * halfWidth * (0.7 + 0.6 * hashUnit(hash, 3));
        const double floorElevation = valley ? elevation : 0.0;
        const glm::dvec3 up = probe;
        // The sheet lies flat over the floor: its own tangent frame, rotated
        // by a seeded angle so the pools never share an axis with the eye's.
        const double spin = 2.0 * kPi * hashUnit(hash, 4);
        glm::dvec3 side = glm::cross(up, east);
        if (glm::length(side) < 1.0e-6)
        {
            side = glm::cross(up, north);
        }
        side = glm::normalize(side);
        const glm::dvec3 forward = glm::normalize(glm::cross(side, up));
        const glm::dvec3 axisA = side * std::cos(spin) + forward * std::sin(spin);
        const glm::dvec3 axisB = side * std::sin(spin) - forward * std::cos(spin);
        const glm::dvec3 center = probe * (seaRadius + std::max(6.0, floorElevation) + bankHeight);
        QuadPush quad;
        quad.normal = up;
        quad.color[0] = 1.0f;
        quad.color[1] = 1.0f;
        quad.color[2] = 1.0f;
        const float uv[4][2] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            quad.uv[corner][0] = uv[corner][0];
            quad.uv[corner][1] = uv[corner][1];
        }
        quad.corners[0] = center - axisA * halfWidth - axisB * halfDepth;
        quad.corners[1] = center + axisA * halfWidth - axisB * halfDepth;
        quad.corners[2] = center + axisA * halfWidth + axisB * halfDepth;
        quad.corners[3] = center - axisA * halfWidth + axisB * halfDepth;
        pushQuad(vertices, indices, quad, anchorEye_);
        ++bankCount;
    }

    mistMesh_.anchorPlanetLocal = anchorEye_;
    // Fainter than the old vertical cards: the pools cover far more frame
    // area lying flat, and fog photographs as a light veil, not a slab.
    uploadMesh(mistMesh_, vertices, indices, false, true, true, static_cast<float>(0.08 * params_.valleyMistStrength));
    mistMesh_.material.baseColorTexture = mistTexture_;
    // The banks pick up the sky: bluish white at high sun, warm gold toward
    // sunrise/sunset (radiation fog at dawn catches the low sun's color).
    const double warmth = smoothStepLocal(0.45, 0.08, sunUp);
    mistMesh_.material.baseColor[0] = static_cast<float>(std::lerp(0.86, 0.96, warmth));
    mistMesh_.material.baseColor[1] = static_cast<float>(std::lerp(0.88, 0.86, warmth));
    mistMesh_.material.baseColor[2] = static_cast<float>(std::lerp(0.94, 0.74, warmth));
}

void PlanetScatterSystem::scatterBirdFlocks(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
    const glm::dvec3 &east, const glm::dvec3 &north)
{
    (void)eyeDirection;
    const std::uint32_t flockCount = static_cast<std::uint32_t>(std::max(0.0, std::min(16.0, std::round(params_.birdFlockCount))));
    flocks_.clear();
    if (flockCount == 0 || !params_.birdsEnabled)
    {
        birdMesh_.active = false;
        return;
    }
    const double seaRadius = params_.derivedSeaRadiusMeters;
    for (std::uint32_t flockIndex = 0; flockIndex < flockCount; ++flockIndex)
    {
        const std::uint64_t hash = cellHash(params_.seed, kSaltFlock, static_cast<std::int64_t>(flockIndex), 0, 0);
        BirdFlock flock;
        const double azimuth = hashUnit(hash, 0) * 2.0 * kPi;
        // Flight band: flocks stay close enough that a bird's wing span
        // still resolves a few pixels on screen at capture resolutions -
        // a flock beyond a couple hundred meters reads as a row of
        // unresolvable dark specks against the haze, which photographs as
        // debris rather than birds (the band tops out at 260 m; the path
        // keeps its own minimum eye distance below).
        const double distance = 80.0 + 180.0 * hashUnit(hash, 1);
        const glm::dvec3 walk = east * std::cos(azimuth) + north * std::sin(azimuth);
        const glm::dvec3 centerDirection = glm::normalize(eyePlanetLocal + walk * distance);
        double groundRadius = 0.0;
        if (!sampleSurfaceRadius(centerDirection, groundRadius))
        {
            continue;
        }
        groundRadius = std::max(groundRadius, seaRadius);
        flock.centerPlanetLocal = centerDirection * (groundRadius + 24.0 + 42.0 * hashUnit(hash, 2));
        // The path circle must keep its far rim away from the eye too: a
        // flock that swings through the camera's own neighborhood reads as a
        // dark flash instead of distant birds.
        flock.pathRadiusMeters = 55.0 + 110.0 * hashUnit(hash, 3);
        flock.pathRadiusMeters = std::min(flock.pathRadiusMeters,
            std::max(20.0, distance - kFlockMinEyeDistanceMeters));
        flock.angularPhase = hashUnit(hash, 4) * 2.0 * kPi;
        flock.angularSpeed = 0.10 + 0.09 * hashUnit(hash, 5);
        flock.flapHertz = 2.4 + 1.6 * hashUnit(hash, 6);
        flock.flapPhase = hashUnit(hash, 7) * 2.0 * kPi;
        flock.birdScale = 0.7 + 0.35 * hashUnit(hash, 8);
        flock.birdCount = static_cast<std::uint32_t>(std::max(1.0, std::round(params_.birdFlockSize)));
        flocks_.push_back(flock);
    }
    birdMesh_.anchorPlanetLocal = anchorEye_;

    // Initial geometry (animateBirds rewrites it every rendered frame).
    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    appendBirdGeometry(vertices, indices);
    uploadMesh(birdMesh_, vertices, indices, false, true, false, 1.0f);
    birdMesh_.material.baseColor[0] = 0.13f;
    birdMesh_.material.baseColor[1] = 0.14f;
    birdMesh_.material.baseColor[2] = 0.16f;
}

void PlanetScatterSystem::animateBirds(double deltaSeconds)
{
    (void)deltaSeconds; // the flight state advanced via the flap phase below; kept for cadence symmetry
    if (flocks_.empty() || birdMesh_.mesh == engine::renderer::invalidMesh
        || renderCapability_ == nullptr || renderCapability_->updateMeshVertices == nullptr)
    {
        return;
    }
    std::vector<PlanetVertex> vertices;
    std::vector<std::uint32_t> indices;
    appendBirdGeometry(vertices, indices);
    renderCapability_->updateMeshVertices(renderCapability_->self, rendererHandle_, birdMesh_.mesh, vertices.data(),
        static_cast<std::uint32_t>(vertices.size()), nullptr);
}

void PlanetScatterSystem::appendBirdGeometry(std::vector<PlanetVertex> &vertices, std::vector<std::uint32_t> &indices)
{
    vertices.clear();
    indices.clear();
    const double seaRadius = params_.derivedSeaRadiusMeters;
    for (const BirdFlock &flock : flocks_)
    {
        // Tangent frame at the flock center: the path plane is horizontal
        // (constant height above the ground under the center).
        const glm::dvec3 centerUp = glm::normalize(flock.centerPlanetLocal);
        glm::dvec3 axis(0.0, 0.0, 1.0);
        if (std::abs(glm::dot(axis, centerUp)) > 0.9)
        {
            axis = glm::dvec3(0.0, 1.0, 0.0);
        }
        const glm::dvec3 pathEast = glm::normalize(glm::cross(axis, centerUp));
        const glm::dvec3 pathNorth = glm::cross(centerUp, pathEast);
        const double pathAngle = flock.angularPhase;
        const glm::dvec3 pathCenter = flock.centerPlanetLocal
            + (pathEast * std::cos(pathAngle) + pathNorth * std::sin(pathAngle)) * flock.pathRadiusMeters;
        // Heading: the path tangent (direction of travel).
        const glm::dvec3 heading = glm::normalize(
            -pathEast * std::sin(pathAngle) + pathNorth * std::cos(pathAngle));
        const glm::dvec3 left = glm::normalize(glm::cross(centerUp, heading));
        for (std::uint32_t birdIndex = 0; birdIndex < flock.birdCount; ++birdIndex)
        {
            // Echelon: each bird trails the leader and steps alternately to
            // the side, the loose V a small flock shows at a distance.
            const double trail = static_cast<double>(birdIndex) * 4.4;
            const double sideSign = (birdIndex % 2) == 0 ? 1.0 : -1.0;
            const double side = sideSign * (0.8 + 0.55 * static_cast<double>(birdIndex));
            const double bob = std::sin(flock.flapPhase * 0.5 + static_cast<double>(birdIndex) * 0.9) * 0.6;
            const glm::dvec3 birdCenter = pathCenter - heading * trail + left * side + centerUp * bob;
            const double scale = flock.birdScale;
            // Flap: the wing dihedral swings ±0.55 rad around the body axis.
            const double flap = std::sin(flock.flapPhase + static_cast<double>(birdIndex) * 0.7) * 0.55;
            const double bodyHalfSpan = 0.34 * scale;
            const double bodyHalfChord = 0.16 * scale;
            const double wingSpan = 0.62 * scale;
            const double wingChord = 0.22 * scale;
            // Body: a small quad in the flight plane.
            QuadPush body;
            body.normal = centerUp;
            body.color[0] = 1.0f;
            body.color[1] = 1.0f;
            body.color[2] = 1.0f;
            for (std::uint32_t corner = 0; corner < 4; ++corner)
            {
                body.uv[corner][0] = 0.0f;
                body.uv[corner][1] = 0.0f;
            }
            body.corners[0] = birdCenter - heading * bodyHalfChord - left * bodyHalfSpan;
            body.corners[1] = birdCenter + heading * bodyHalfChord - left * bodyHalfSpan;
            body.corners[2] = birdCenter + heading * bodyHalfChord + left * bodyHalfSpan;
            body.corners[3] = birdCenter - heading * bodyHalfChord + left * bodyHalfSpan;
            pushQuad(vertices, indices, body, anchorEye_);
            // Wings: two quads hinged at the body sides, tips lifted by the
            // flap angle; up-stroke and down-stroke alternate per frame.
            for (std::uint32_t wing = 0; wing < 2; ++wing)
            {
                const double wingSign = wing == 0 ? 1.0 : -1.0;
                const glm::dvec3 rootInner = birdCenter + left * (wingSign * bodyHalfSpan * 0.7);
                const glm::dvec3 wingOut = left * (wingSign * wingSpan * std::cos(flap)) + centerUp * (wingSpan * std::sin(flap));
                QuadPush wingQuad;
                wingQuad.normal = centerUp;
                wingQuad.color[0] = 1.0f;
            wingQuad.color[1] = 1.0f;
            wingQuad.color[2] = 1.0f;
                for (std::uint32_t corner = 0; corner < 4; ++corner)
                {
                    wingQuad.uv[corner][0] = 0.0f;
                    wingQuad.uv[corner][1] = 0.0f;
                }
                wingQuad.corners[0] = rootInner - heading * wingChord;
                wingQuad.corners[1] = rootInner + heading * wingChord;
                wingQuad.corners[2] = rootInner + heading * wingChord * 0.4 + wingOut;
                wingQuad.corners[3] = rootInner - heading * wingChord * 0.4 + wingOut;
                pushQuad(vertices, indices, wingQuad, anchorEye_);
            }
        }
    }
    (void)seaRadius;
}

void PlanetScatterSystem::update(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection, bool sunEnabled,
    double deltaSeconds)
{
    if (!initialized_)
    {
        return;
    }
    const double eyeRadius = glm::length(eyePlanetLocal);
    const double eyeAltitude = eyeRadius - params_.radiusMeters;
    groundLevel_ = eyeAltitude < params_.atmosphereHeightMeters * 0.75;
    if (!groundLevel_)
    {
        treeMesh_.active = false;
        bushMesh_.active = false;
        grassMesh_.active = false;
        rockMesh_.active = false;
        shadowMesh_.active = false;
        whitecapMesh_.active = false;
        surfMesh_.active = false;
        mistMesh_.active = false;
        birdMesh_.active = false;
        drawCommands_.clear();
        commandAnchors_.clear();
        return;
    }

    lastSunDirection_ = sunDirection;
    const double moved = glm::length(eyePlanetLocal - anchorEye_);
    if (!hasAnchor_ || moved >= params_.scatterRebuildDistanceMeters)
    {
        rebuild(eyePlanetLocal);
    }
    else
    {
        // Keep the sea state and the birds alive between rebuilds: the foam
        // churns on its staggered quarters, the flocks fly their paths.
        const glm::dvec3 eyeDirection = eyePlanetLocal / std::max(1.0, eyeRadius);
        glm::dvec3 axis(0.0, 0.0, 1.0);
        if (std::abs(glm::dot(axis, eyeDirection)) > 0.9)
        {
            axis = glm::dvec3(0.0, 1.0, 0.0);
        }
        const glm::dvec3 east = glm::normalize(glm::cross(axis, eyeDirection));
        const glm::dvec3 north = glm::cross(eyeDirection, east);
        scatterWhitecaps(eyePlanetLocal, eyeDirection, east, north, true);
        animateBirds(deltaSeconds);
    }

    refreshAccumulator_ += deltaSeconds;
    if (refreshAccumulator_ >= kWhitecapRefreshSeconds)
    {
        refreshAccumulator_ = 0.0;
        ++refreshCounter_;
    }

    // Birds roost at night: the flock mesh only draws while the sun stands
    // over the horizon at the anchor (a sun below the horizon gates the
    // draw the way the aurora's day-gate does).
    if (!flocks_.empty())
    {
        const glm::dvec3 eyeDirection = eyePlanetLocal / std::max(1.0, eyeRadius);
        const double sunUp = glm::dot(glm::normalize(sunDirection), eyeDirection);
        birdMesh_.active = sunEnabled && sunUp > 0.06;
    }
    composeDrawCommands();
}

void PlanetScatterSystem::composeDrawCommands()
{
    drawCommands_.clear();
    commandAnchors_.clear();
    const auto push = [this](const ScatterMesh &slot)
    {
        if (!slot.active || slot.mesh == engine::renderer::invalidMesh)
        {
            return;
        }
        commandAnchors_.push_back(slot.anchorPlanetLocal);
        engine::renderer::RenderDrawCommand command;
        command.mesh = slot.mesh;
        command.material = &slot.material;
        command.modelMatrix = nullptr; // composed by the chunk renderer (f64 shift compose)
        drawCommands_.push_back(command);
    };
    push(treeMesh_);
    push(bushMesh_);
    push(grassMesh_);
    push(rockMesh_);
    push(shadowMesh_);
    push(birdMesh_);
    push(whitecapMesh_);
    push(surfMesh_);
    push(mistMesh_);
}

} // namespace demo::planet_demo
