// Shared value types for the planet module (engine::planet).
//
// Plain C++ vocabulary used by the generation core and the capability
// layer: a double-precision vector for planet-space math (planets are
// scaled to arbitrary radii, where float precision would break down),
// stable hashing for seeds, chunk identity, statistics, and the borrowed
// chunk-mesh views the module hands to consumers.

#ifndef ENGINE_PLANET_TYPES_HPP
#define ENGINE_PLANET_TYPES_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace planet
{

// Double-precision 3-vector. Planet-space positions can be millions of
// meters from the origin, so the generation core works in doubles and only
// converts to floats at the final chunk-local vertex stage.
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(const Vec3 &a, double scalar)
{
    return Vec3{a.x * scalar, a.y * scalar, a.z * scalar};
}

inline Vec3 operator*(double scalar, const Vec3 &a)
{
    return Vec3{a.x * scalar, a.y * scalar, a.z * scalar};
}

inline Vec3 &operator+=(Vec3 &a, const Vec3 &b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline double dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double lengthSquared(const Vec3 &a)
{
    return dot(a, a);
}

inline double length(const Vec3 &a)
{
    return std::sqrt(lengthSquared(a));
}

inline Vec3 normalized(const Vec3 &a)
{
    const double magnitude = length(a);
    if (magnitude <= 0.0)
    {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{a.x / magnitude, a.y / magnitude, a.z / magnitude};
}

// x = clamped linear interpolation.
inline double mix(double edge0, double edge1, double x)
{
    return edge0 + (edge1 - edge0) * x;
}

// Smooth Hermite interpolation between 0 and 1 for edge0 <= x <= edge1.
inline double smoothstep(double edge0, double edge1, double x)
{
    if (edge1 <= edge0)
    {
        return x < edge0 ? 0.0 : 1.0;
    }
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// ---- stable hashing ---------------------------------------------------------

// SplitMix64: the seed schedule for every noise stream derived from one
// planet seed. Pure integer math, identical on every platform.
inline std::uint64_t splitMix64(std::uint64_t &state)
{
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z               = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z               = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// FNV-1a 64: used to derive independent sub-seeds (and to hash chunk
// buffers in the determinism self-tests).
inline std::uint64_t fnv1a64(const void *data, std::size_t size, std::uint64_t seed = 0xCBF29CE484222325ull)
{
    const auto *bytes  = static_cast<const std::uint8_t *>(data);
    std::uint64_t hash = seed;
    for (std::size_t index = 0; index < size; ++index)
    {
        hash ^= bytes[index];
        hash *= 0x100000001B3ull;
    }
    return hash;
}

// Deterministic 64-bit mix of two integers (order-independent node keys,
// per-stream noise seeds).
inline std::uint64_t hashCombine64(std::uint64_t a, std::uint64_t b)
{
    std::uint64_t state = a ^ (b + 0x9E3779B97F4A7C15ull + (a << 6) + (a >> 2));
    return splitMix64(state);
}

// ---- chunk identity ---------------------------------------------------------

// Identifies one octree node: depth in the tree (0 = root cube) plus the
// node's integer coordinates on that depth's lattice. The key packs all
// four values into 64 bits; maxDepth is capped so the coordinates fit.
using ChunkKey = std::uint64_t;

inline constexpr std::uint32_t maxSupportedOctreeDepth = 20;

inline ChunkKey makeChunkKey(std::uint32_t depth, std::uint32_t x, std::uint32_t y, std::uint32_t z)
{
    return (static_cast<std::uint64_t>(depth & 0xFu) << 60) | (static_cast<std::uint64_t>(x & 0xFFFFFu) << 40)
         | (static_cast<std::uint64_t>(y & 0xFFFFFu) << 20) | static_cast<std::uint64_t>(z & 0xFFFFFu);
}

inline void unpackChunkKey(ChunkKey key, std::uint32_t &depth, std::uint32_t &x, std::uint32_t &y, std::uint32_t &z)
{
    depth = static_cast<std::uint32_t>((key >> 60) & 0xFu);
    x     = static_cast<std::uint32_t>((key >> 40) & 0xFFFFFu);
    y     = static_cast<std::uint32_t>((key >> 20) & 0xFFFFFu);
    z     = static_cast<std::uint32_t>(key & 0xFFFFFu);
}

// ---- biomes -----------------------------------------------------------------

// Terrestrial biomes plus the coastal/ocean bands. The numeric values are
// stable (saved games may persist them); the palette that maps each id to
// an RGB color lives in PlanetGenerationParams (config-overridable).
enum class Biome : std::uint32_t
{
    DeepOcean = 0,
    Ocean,
    ShallowOcean,
    Beach,
    Desert,
    Savanna,
    Grassland,
    Shrubland,
    TemperateForest,
    SeasonalForest,
    TropicalRainforest,
    TemperateRainforest,
    Taiga,
    Tundra,
    Snow,
    Rock,
    Count
};

inline constexpr std::uint32_t biomeCount = static_cast<std::uint32_t>(Biome::Count);

// ---- climate ----------------------------------------------------------------

// Climate state sampled at one surface point. temperature01 and
// moisture01 live in [0, 1]; temperature follows latitude minus an
// altitude lapse rate, moisture blends large-scale humidity noise with
// latitude band effects (ITCZ wet belt, subtropical dry belts).
struct ClimateSample
{
    double    temperature01 = 0.0;
    double    moisture01    = 0.0;
    Biome     biome         = Biome::Ocean;
    float     color[3]      = {1.0f, 1.0f, 1.0f}; // linear RGB, palette × local variation
};

// ---- statistics -------------------------------------------------------------

struct PlanetStatistics
{
    std::uint32_t chunksVisible    = 0;  // nodes the octree currently wants rendered
    std::uint32_t chunksReady      = 0;  // visible nodes with a finished mesh
    std::uint32_t chunksBuilding   = 0;  // queued or in-flight mesh builds
    std::uint32_t chunksReadyTotal = 0;  // every cached mesh (including hidden LOD rings)
    std::uint64_t triangleCount    = 0;  // triangles across ready chunks (terrain only)
    std::uint64_t verticesTotal    = 0;
    double        lastUpdateMilliseconds = 0.0; // cost of the last update() call
    std::uint32_t octreeDepthUsed  = 0;
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_TYPES_HPP
