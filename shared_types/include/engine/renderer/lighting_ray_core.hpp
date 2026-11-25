// Shared math core of the CPU Lumen-style lighting modules (radiance_cache,
// reflection_field).
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
#include <limits>

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

namespace engine::lighting
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

// ---- shared environment core -------------------------------------------------

// The pbr environment fields both lumen modules snapshot into their frame
// state on every evaluate() (the optional provider's values when present,
// otherwise the caller's defaults). One definition here keeps the two
// modules' sun, skylight, and tone mapping behavior identical, the way
// prepareLight keeps their direct terms identical.
struct EnvironmentCore
{
    bool     hasSun       = false;
    bool     sunEnabled   = false;
    glm::vec3 sunDirection{0.45f, 0.62f, 0.35f}; // toward the sun (unit)
    glm::vec3 sunColor{1.0f, 0.96f, 0.88f};
    float    sunIntensity = 3.0f;
    glm::vec3 ambientSky{0.55f, 0.65f, 0.80f};
    glm::vec3 ambientGround{0.35f, 0.30f, 0.25f};
    float    ambientIntensity = 0.5f;
    float    exposure      = 1.0f;
    pbr::ToneMapping toneMapping = pbr::ToneMapping::AcesFilmic;

    // Hemisphere skylight standing in for Lumen's Skylight (the environment is
    // the engine's only "infinitely far" lighting source).
    glm::vec3 skyRadiance(const glm::vec3 &direction) const
    {
        const float up = direction.y * 0.5f + 0.5f;
        return glm::mix(ambientGround, ambientSky, up) * ambientIntensity;
    }
    glm::vec3 sunRadiance() const
    {
        return (hasSun && sunEnabled) ? sunColor * sunIntensity : glm::vec3(0.0f);
    }
};

// Copies the pbr provider's environment over `out` and reports its version;
// returns false (leaving `out` untouched) when the provider is absent or its
// copy fails, so the caller's defaults stay in effect.
inline std::uint64_t copyPbrEnvironment(const pbr::PbrCapability *capability, EnvironmentCore &out)
{
    std::uint64_t version = 0;
    if (capability == nullptr || capability->copyEnvironment == nullptr)
    {
        return version;
    }
    pbr::PbrEnvironment environment{};
    if (!capability->copyEnvironment(capability->self, &environment, &version))
    {
        return 0;
    }
    out.hasSun           = true;
    out.sunEnabled       = environment.sun.enabled;
    const glm::vec3 sun(environment.sun.direction[0], environment.sun.direction[1], environment.sun.direction[2]);
    if (glm::dot(sun, sun) > 1.0e-12f)
    {
        out.sunDirection = glm::normalize(sun);
    }
    out.sunColor         = glm::vec3(environment.sun.color[0], environment.sun.color[1], environment.sun.color[2]);
    out.sunIntensity     = environment.sun.intensity;
    out.ambientSky       = glm::vec3(environment.ambient.skyColor[0], environment.ambient.skyColor[1],
        environment.ambient.skyColor[2]);
    out.ambientGround    = glm::vec3(environment.ambient.groundColor[0], environment.ambient.groundColor[1],
        environment.ambient.groundColor[2]);
    out.ambientIntensity = environment.ambient.intensity;
    out.exposure         = environment.exposure;
    out.toneMapping      = environment.toneMapping;
    return version;
}

// ---- shared direct-light preparation and evaluation --------------------------

// One direct_lighting registry light with the per-light constants the shading
// loops need (normalized direction or cone axis, cone cosines) prepared once
// per frame, so the per-texel and per-hit evaluation below is pure math.
// radiance_cache and reflection_field snapshot the same structure so their direct
// terms stay identical to each other's (and to the renderer's formulas).
struct PreparedLight
{
    direct_lighting::DirectLightType type = direct_lighting::DirectLightType::Directional;
    glm::vec3 color{0.0f};
    float     intensity   = 0.0f;
    // Directional: unit direction toward the light. Spot: unit cone axis.
    glm::vec3 direction{0.0f};
    glm::vec3 position{0.0f};
    float     range       = 0.0f;
    float     innerCosine = 0.0f;
    float     outerCosine = 0.0f;
    float     coneDivisor = 1.0e-5f;
};

// Prepares one registry light for the loops above; returns false for inactive
// lights (zero intensity or black color), which the snapshots drop.
inline bool prepareLight(const direct_lighting::DirectLight &light, PreparedLight &out)
{
    const direct_lighting::DirectLightDescription &description = light.description;
    const glm::vec3 color(description.color[0], description.color[1], description.color[2]);
    if (description.intensity <= 0.0f || (color.x <= 0.0f && color.y <= 0.0f && color.z <= 0.0f))
    {
        return false;
    }
    out.type      = description.type;
    out.color     = color;
    out.intensity = description.intensity;
    out.position  = glm::vec3(description.position[0], description.position[1], description.position[2]);
    out.range     = std::max(description.range, 1.0e-3f);
    if (description.type == direct_lighting::DirectLightType::Directional)
    {
        out.direction = -glm::normalize(glm::vec3(
            description.direction[0], description.direction[1], description.direction[2]));
    }
    else if (description.type == direct_lighting::DirectLightType::Spot)
    {
        out.direction = glm::normalize(glm::vec3(
            description.direction[0], description.direction[1], description.direction[2]));
        out.innerCosine = std::cos(description.innerConeRadians);
        out.outerCosine = std::cos(description.outerConeRadians);
        out.coneDivisor = std::max(out.innerCosine - out.outerCosine, 1.0e-5f);
    }
    else
    {
        out.direction = glm::vec3(0.0f);
    }
    return true;
}

// Unoccluded arrival of one prepared light at a shaded point: the irradiance
// contribution `color * intensity * attenuation * nDotL` together with the
// unit direction toward the light and the light's distance (the inputs a
// shadow ray needs; a directional light reports an infinite distance). The
// attenuation follows the renderer's shading formulas - range fade
// (1 - (d/r)^2)^2 / d^2 and the smoothstep cone - with no shadowing: the
// caller decides how, and whether, to occlude. Returns false when the light
// reaches nothing here (back-facing, out of range, outside the cone).
inline bool lightArrival(
    const PreparedLight &light, const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outDirection,
    float &outDistance, glm::vec3 &outArrival)
{
    if (light.type == direct_lighting::DirectLightType::Directional)
    {
        const glm::vec3 &direction = light.direction;
        const float      nDotL     = glm::dot(normal, direction);
        if (nDotL <= 0.0f)
        {
            return false;
        }
        outDirection = direction;
        outDistance  = std::numeric_limits<float>::max();
        outArrival   = light.color * light.intensity * nDotL;
        return true;
    }
    const glm::vec3 toLight         = light.position - position;
    const float     distanceSquared = std::max(glm::dot(toLight, toLight), 1.0e-4f);
    const float     rangeRatioSquared = distanceSquared / (light.range * light.range);
    if (rangeRatioSquared >= 1.0f)
    {
        return false;
    }
    const float     distance  = std::sqrt(distanceSquared);
    const glm::vec3 direction = toLight * (1.0f / distance);
    const float     rangeFade = 1.0f - rangeRatioSquared * rangeRatioSquared;
    float           attenuation = rangeFade * rangeFade / distanceSquared;
    if (light.type == direct_lighting::DirectLightType::Spot)
    {
        const float coneCosine = glm::dot(-direction, light.direction);
        const float coneT      = std::clamp((coneCosine - light.outerCosine) / light.coneDivisor, 0.0f, 1.0f);
        attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
    }
    const float nDotL = glm::dot(normal, direction);
    if (nDotL <= 0.0f || attenuation <= 0.0f)
    {
        return false;
    }
    outDirection = direction;
    outDistance  = distance;
    outArrival   = light.color * light.intensity * attenuation * nDotL;
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

// ---- shared SH2 evaluations ---------------------------------------------------

// The SH2 basis terms both lighting modules evaluate radiance/irradiance
// with. The stored coefficients are the standard SH coefficients
// pre-multiplied by their basis normalization constants (the radiance_cache
// probe convention), laid out [basis][channel]: the three channels of one
// basis term are contiguous, so each evaluation runs its three color
// channels as packed lanes of one vector - the exact scalar operation
// sequence per channel, a third of the instructions.
struct Sh2BasisTerms
{
    float x  = 0.0f;
    float y  = 0.0f;
    float z  = 0.0f;
    float xy = 0.0f;
    float yz = 0.0f;
    float zz3m1 = 0.0f; // 3zz - 1
    float xz = 0.0f;
    float xxmyy = 0.0f; // xx - yy
};

inline Sh2BasisTerms sh2BasisTermsFor(const glm::vec3 &direction)
{
    const float x  = direction.x;
    const float y  = direction.y;
    const float z  = direction.z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    Sh2BasisTerms terms;
    terms.x     = x;
    terms.y     = y;
    terms.z     = z;
    terms.xy    = x * y;
    terms.yz    = y * z;
    terms.zz3m1 = 3.0f * zz - 1.0f;
    terms.xz    = x * z;
    terms.xxmyy = xx - yy;
    return terms;
}

// Directional radiance of one probe's SH2 projection along `direction` (the
// plain polynomial over the [basis][channel] coefficients; a constant
// radiance L projects to exactly C00 = L). Grouped exactly as the scalar
// per-channel form grouped its terms, so each lane performs the same
// rounding the scalar form performed.
inline glm::vec3 evaluateSh2RadianceVec3(const float sh2[27], const Sh2BasisTerms &terms)
{
    const glm::vec3 band0(sh2[0], sh2[1], sh2[2]);
    const glm::vec3 band1Y(sh2[3], sh2[4], sh2[5]);
    const glm::vec3 band1Z(sh2[6], sh2[7], sh2[8]);
    const glm::vec3 band1X(sh2[9], sh2[10], sh2[11]);
    const glm::vec3 band2XY(sh2[12], sh2[13], sh2[14]);
    const glm::vec3 band2YZ(sh2[15], sh2[16], sh2[17]);
    const glm::vec3 band2ZZ(sh2[18], sh2[19], sh2[20]);
    const glm::vec3 band2XZ(sh2[21], sh2[22], sh2[23]);
    const glm::vec3 band2XXYY(sh2[24], sh2[25], sh2[26]);
    return band0 + band1Y * terms.y + band1Z * terms.z + band1X * terms.x + band2XY * terms.xy + band2YZ * terms.yz
        + band2ZZ * terms.zz3m1 + band2XZ * terms.xz + band2XXYY * terms.xxmyy;
}

// Cosine-lobed SH2 irradiance at one probe (the Ramamoorthi convolution the
// exported probe grid stores irradiance-ready coefficients for): the band
// weights pi, 2pi/3, and pi/4 scale the bands before the same grouped sum.
inline glm::vec3 evaluateSh2IrradianceVec3(const float sh2[27], const Sh2BasisTerms &terms)
{
    constexpr float k0 = 3.141592654f; // pi      (band 0)
    constexpr float k1 = 2.094395102f; // 2pi/3   (band 1)
    constexpr float k2 = 0.785398163f; // pi/4    (band 2)

    const glm::vec3 band0(sh2[0], sh2[1], sh2[2]);
    const glm::vec3 band1Y(sh2[3], sh2[4], sh2[5]);
    const glm::vec3 band1Z(sh2[6], sh2[7], sh2[8]);
    const glm::vec3 band1X(sh2[9], sh2[10], sh2[11]);
    const glm::vec3 band2XY(sh2[12], sh2[13], sh2[14]);
    const glm::vec3 band2YZ(sh2[15], sh2[16], sh2[17]);
    const glm::vec3 band2ZZ(sh2[18], sh2[19], sh2[20]);
    const glm::vec3 band2XZ(sh2[21], sh2[22], sh2[23]);
    const glm::vec3 band2XXYY(sh2[24], sh2[25], sh2[26]);
    return k0 * band0 + k1 * (band1Y * terms.y + band1Z * terms.z + band1X * terms.x)
        + k2 * (band2XY * terms.xy + band2YZ * terms.yz + band2ZZ * terms.zz3m1 + band2XZ * terms.xz
                 + band2XXYY * terms.xxmyy);
}

} // namespace engine::lighting
