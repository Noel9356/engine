// Shared math core of the CPU Lumen-style lighting modules (lumen_gi,
// lumen_reflection).
//
// The RNG, the tangent-frame and hemisphere sampling, the ray/bbox/ray/
// triangle intersection kernels, the luminance weight, and the quality
// preset table are the pieces both modules need with byte-identical
// behavior (a probe update and a reflection ray must draw from the same
// distributions and accept the same hits), so they live here once instead
// of being kept textually in sync per module.
//
// The ray/triangle kernel is a template over the caller's ray and packed
// triangle layouts: both modules carry `.origin`, `.direction`, `.tMax`,
// `.v0`, `.edge1`, `.edge2` members, and the kernel is pure arithmetic over
// them.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

namespace engine::lumen
{

inline constexpr float kPi = 3.14159265358979f;

// ---- deterministic RNG ------------------------------------------------------

// Splitmix64-style scramble of one seed value.
inline std::uint64_t hashSeed(std::uint64_t seed)
{
    std::uint64_t hash = seed * 0x9E3779B97F4A7C15ull;
    hash ^= hash >> 30;
    hash *= 0xBF58476D1CE4E5B9ull;
    hash ^= hash >> 27;
    hash *= 0x94D049BB133111EBull;
    hash ^= hash >> 31;
    return hash;
}

// Uniform [0, 1) float; advances the seed.
inline float randomFloat(std::uint64_t &seed)
{
    seed = hashSeed(seed + 0x632BE59BD9B4E019ull);
    return static_cast<float>(seed >> 40) * (1.0f / 16777216.0f);
}

// ---- sampling frames --------------------------------------------------------

// Orthonormal basis with `normal` as the third column (Duff et al. 2017;
// branchless and free of the discontinuity of the older cross-product axis
// switch).
inline glm::mat3 makeBasis(const glm::vec3 &normal)
{
    const float     sign = normal.z >= 0.0f ? 1.0f : -1.0f;
    const float     a    = -1.0f / (sign + normal.z);
    const float     b    = normal.x * normal.y * a;
    const glm::vec3 tangent(1.0f + sign * normal.x * normal.x * a, sign * b, -sign * normal.x);
    const glm::vec3 bitangent(b, sign + normal.y * normal.y * a, -normal.y);
    return glm::mat3(tangent, bitangent, normal);
}

// Malley cosine-hemisphere sample around +Z from two uniform numbers.
inline glm::vec3 cosineHemisphere(float u, float v)
{
    const float radius = std::sqrt(std::max(0.0f, u));
    const float phi    = v * 2.0f * kPi;
    return glm::vec3(radius * std::cos(phi), radius * std::sin(phi), std::sqrt(std::max(0.0f, 1.0f - u)));
}

// ---- intersection kernels ----------------------------------------------------

// Per-axis inverse direction with the degenerate-axis guard (1e8 stands in
// for infinity, so a zero component never divides by zero).
inline glm::vec3 safeInverseDirection(const glm::vec3 &direction)
{
    glm::vec3 inverse;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        inverse[axis] = std::fabs(direction[axis]) > 1.0e-8f ? 1.0f / direction[axis] : 1.0e8f;
    }
    return inverse;
}

// Slab test returning the entry distance (used to visit the nearer child
// first so tMax shrinks early and the far subtree is usually culled).
inline bool slabTest(const glm::vec3 &origin, const glm::vec3 &inverseDirection, const glm::vec3 &boundsMin,
    const glm::vec3 &boundsMax, float tMax, float &outNear)
{
    const float tx0 = (boundsMin.x - origin.x) * inverseDirection.x;
    const float tx1 = (boundsMax.x - origin.x) * inverseDirection.x;
    const float ty0 = (boundsMin.y - origin.y) * inverseDirection.y;
    const float ty1 = (boundsMax.y - origin.y) * inverseDirection.y;
    const float tz0 = (boundsMin.z - origin.z) * inverseDirection.z;
    const float tz1 = (boundsMax.z - origin.z) * inverseDirection.z;
    const float tNear = std::max(std::max(std::min(tx0, tx1), std::min(ty0, ty1)), std::max(std::min(tz0, tz1), 0.0f));
    const float tFar  = std::min(std::min(std::max(tx0, tx1), std::max(ty0, ty1)), std::min(std::max(tz0, tz1), tMax));
    outNear           = tNear;
    return tNear <= tFar;
}

// Moller-Trumbore with a hair of edge tolerance (shared-edge watertightness).
// Template over the module's ray and packed-triangle layouts; both carry
// .origin/.direction/.tMax and .v0/.edge1/.edge2.
template <typename Ray, typename PackedTriangle>
bool rayTriangle(const Ray &ray, const PackedTriangle &triangle, float &outT, glm::vec3 &outBarycentric)
{
    const glm::vec3 pVec        = glm::cross(ray.direction, triangle.edge2);
    const float     determinant = glm::dot(triangle.edge1, pVec);
    if (std::fabs(determinant) < 1.0e-12f)
    {
        return false;
    }
    const float     inverseDeterminant = 1.0f / determinant;
    const glm::vec3 tVec               = ray.origin - triangle.v0;
    constexpr float edgeTolerance      = 1.0e-6f;
    const float     u                  = glm::dot(tVec, pVec) * inverseDeterminant;
    if (u < -edgeTolerance || u > 1.0f + edgeTolerance)
    {
        return false;
    }
    const glm::vec3 qVec = glm::cross(tVec, triangle.edge1);
    const float     v    = glm::dot(ray.direction, qVec) * inverseDeterminant;
    if (v < -edgeTolerance || u + v > 1.0f + edgeTolerance)
    {
        return false;
    }
    const float t = glm::dot(triangle.edge2, qVec) * inverseDeterminant;
    if (t < 1.0e-4f || t > ray.tMax)
    {
        return false;
    }
    outT           = t;
    outBarycentric = glm::vec3(1.0f - u - v, u, v);
    return true;
}

// ---- shared weights ---------------------------------------------------------

// Rec. 709 luminance of a linear RGB value.
inline float luminance(const glm::vec3 &value)
{
    return 0.2126f * value.x + 0.7152f * value.y + 0.0722f * value.z;
}

// Quality preset table shared by the modules (Auto/Low/Medium/High/Ultra
// scale the budget the same way in each). Auto derives its scale from the
// host's logical processor count, which the caller queries once.
inline float qualityScale(std::uint32_t quality, std::uint32_t processorCount)
{
    switch (quality)
    {
    case 0: // Auto
        return processorCount >= 8u ? 1.0f : 0.5f;
    case 1: // Low
        return 0.25f;
    case 2: // Medium
        return 0.5f;
    case 4: // Ultra
        return 2.0f;
    case 3: // High
    default:
        return 1.0f;
    }
}

} // namespace engine::lumen
