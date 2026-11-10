// lumen_gi lighting side: the Lumen Scene lighting passes.
//
// - Direct lighting evaluates sun + direct lights per card texel (amortized
//   through a per-frame texel budget; untouched texels keep their cached
//   value, which is exactly Lumen's "Lumen Scene Lighting Update Speed"
//   behavior).
// - Indirect lighting runs the final gather in texture space: budgeted texels
//   trace a few cosine rays that read direct + PREVIOUS indirect radiance
//   from the surface cache, propagating one bounce per frame; Lumen's N+2
//   bounces fall out of the feedback loop.
// - The world-space radiance cache updates amortized octahedral probes whose
//   rays sample the same surface cache; misses fall back to the hemisphere
//   skylight. Screen-space probes (debug view) shorten their rays and
//   interpolate the world cache on miss, Lumen's integration rule.

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace lumen_gi
{



namespace
{

// Cosine-hemisphere sample around +Z (Malley's method) with per-index seeds;
// deterministic so frames are reproducible.
glm::vec3 cosineDirection(std::uint64_t seed)
{
    // xxhash-style avalanche for cheap stratification-free sequences.
    std::uint64_t hash = seed * 0x9E3779B97F4A7C15ull;
    hash ^= hash >> 30;
    hash *= 0xBF58476D1CE4E5B9ull;
    hash ^= hash >> 27;
    hash *= 0x94D049BB133111EBull;
    hash ^= hash >> 31;
    const double u = static_cast<double>(hash >> 40) / 16777216.0;
    const double v = static_cast<double>(hash & 0xFFFFFFull) / 16777216.0;
    const float  radius = std::sqrt(std::max(0.0f, static_cast<float>(u)));
    const float  phi    = static_cast<float>(v) * 2.0f * 3.141592654f;
    return glm::vec3(radius * std::cos(phi), radius * std::sin(phi),
        std::sqrt(std::max(0.0f, static_cast<float>(1.0 - u))));
}

glm::mat3 onb(const glm::vec3 &normal)
{
    const glm::vec3 axis = std::fabs(normal.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 tangent = glm::normalize(glm::cross(axis, normal));
    const glm::vec3 bitangent = glm::cross(normal, tangent);
    return glm::mat3(tangent, bitangent, normal);
}

// Point/spot/directional radiance arriving at position from the module's
// light snapshot; formulas mirror the renderer's shading (range fade
// (1 - (d/r)^2)^2 / d^2, smoothstep cone), so cached radiance matches what
// the renderer applies for direct lighting.
glm::vec3 directLightRadiance(const GiEnvironmentSnapshot &environment, const glm::vec3 &position,
    const glm::vec3 &normal)
{
    glm::vec3 sum(0.0f);
    if (!environment.hasDirectLights)
    {
        return sum;
    }
    for (const direct_lighting::DirectLight &light : environment.directLights)
    {
        const direct_lighting::DirectLightDescription &description = light.description;
        const glm::vec3 color(description.color[0], description.color[1], description.color[2]);
        const float     intensity = description.intensity;
        if (intensity <= 0.0f || (color.x <= 0.0f && color.y <= 0.0f && color.z <= 0.0f))
        {
            continue;
        }
        if (description.type == direct_lighting::DirectLightType::Directional)
        {
            const glm::vec3 direction = -glm::normalize(glm::vec3(description.direction[0], description.direction[1],
                description.direction[2]));
            const float nDotL = glm::dot(normal, direction);
            if (nDotL > 0.0f)
            {
                sum += color * intensity * nDotL;
            }
            continue;
        }
        const glm::vec3 lightPosition(description.position[0], description.position[1], description.position[2]);
        const glm::vec3 toLight          = lightPosition - position;
        const float     distanceSquared  = std::max(glm::dot(toLight, toLight), 1.0e-4f);
        const float     range            = std::max(description.range, 1.0e-3f);
        const float     rangeRatioSquared = distanceSquared / (range * range);
        if (rangeRatioSquared >= 1.0f)
        {
            continue;
        }
        const glm::vec3 direction  = toLight * (1.0f / std::sqrt(distanceSquared));
        const float     rangeFade  = 1.0f - rangeRatioSquared * rangeRatioSquared;
        float           attenuation = rangeFade * rangeFade / distanceSquared;
        if (description.type == direct_lighting::DirectLightType::Spot)
        {
            const glm::vec3 spotDirection(description.direction[0], description.direction[1], description.direction[2]);
            const float     coneCosine = glm::dot(-direction, glm::normalize(spotDirection));
            const float     inner      = std::cos(description.innerConeRadians);
            const float     outer      = std::cos(description.outerConeRadians);
            const float coneT =
                std::clamp((coneCosine - outer) / std::max(inner - outer, 1.0e-5f), 0.0f, 1.0f);
            attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
        }
        const float nDotL = glm::dot(normal, direction);
        if (nDotL <= 0.0f || attenuation <= 0.0f)
        {
            continue;
        }
        sum += color * intensity * attenuation * nDotL;
    }
    return sum;
}

} // namespace

// Shadow test toward the sun with Lumen's surface bias and backface rule:
// rays that start inside geometry or hit a surface from behind contribute
// nothing instead of leaking.
bool sunShadowed(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    std::uint32_t excludedSurface)
{
    const glm::vec3 sunDirection = state.environment.sunDirection;
    const float     bias         = 0.02f + 0.02f * std::fabs(glm::dot(normal, sunDirection));
    GiRay           ray{position + normal * bias, sunDirection, state.settings.maxTraceDistance};
    return traceSceneOccluded(state, ray, excludedSurface);
}

// ---- surface cache sampling -------------------------------------------------

// Projects a hit onto the hit surface's cards and blends their cached
// radiance: candidate cards are the face cards whose outward axis aligns with
// the hit normal; texels are weighted by normal alignment and depth
// agreement, mirroring Lumen's card sampling (three cards by normal, weights
// |texelDepth - rayHitT| and cardNormal . hitNormal).
void sampleSurfaceCache(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, bool includeIndirect, glm::vec3 &outRadiance)
{
    outRadiance = glm::vec3(0.0f);
    if (surfaceIndex >= state.surfaces.size())
    {
        return;
    }
    const GiSurface &surface = state.surfaces[surfaceIndex];
    glm::vec3        weightSum(0.0f);
    for (std::uint32_t cardIndex = surface.firstCard; cardIndex < surface.firstCard + surface.cardCount; ++cardIndex)
    {
        const GiCard &card = state.cards[cardIndex];
        // Project the hit point onto the card plane; texel coverage test.
        const glm::vec3 relative = position - card.origin;
        const float     u        = glm::dot(relative, card.axisU) / std::max(card.width, 1.0e-6f);
        const float     v        = glm::dot(relative, card.axisV) / std::max(card.height, 1.0e-6f);
        const float     depth    = glm::dot(relative, -card.axisW);
        // u, v are face-centered coordinates in [-0.5, 0.5], matching the
        // capture parameterization (texel centers at (i + 0.5) / texels).
        constexpr float halfTexelMargin = 0.5f + 1.0e-3f;
        if (depth < -0.05f || depth > card.depth + 0.05f || u < -halfTexelMargin || u > halfTexelMargin
            || v < -halfTexelMargin || v > halfTexelMargin)
        {
            continue;
        }
        // Weight by normal alignment: cards facing away from the surface
        // normal contribute nothing (Lumen's cardNormal . hitNormal weight).
        const float alignment = glm::dot(card.axisW, normal);
        if (alignment <= 0.15f)
        {
            continue;
        }
        const float texelX = (u + 0.5f) * static_cast<float>(card.texelWidth) - 0.5f;
        const float texelY = (v + 0.5f) * static_cast<float>(card.texelHeight) - 0.5f;
        const std::int32_t x0 = static_cast<std::int32_t>(std::floor(texelX));
        const std::int32_t y0 = static_cast<std::int32_t>(std::floor(texelY));
        for (std::uint32_t dy = 0; dy < 2; ++dy)
        {
            for (std::uint32_t dx = 0; dx < 2; ++dx)
            {
                const std::int32_t x = std::clamp(x0 + static_cast<std::int32_t>(dx), 0,
                    static_cast<std::int32_t>(card.texelWidth) - 1);
                const std::int32_t y = std::clamp(y0 + static_cast<std::int32_t>(dy), 0,
                    static_cast<std::int32_t>(card.texelHeight) - 1);
                const std::size_t texel = static_cast<std::size_t>(y) * card.texelWidth + x;
                if (card.depthTexels[texel] >= invalidCardDepth)
                {
                        continue; // invalid coverage: no contribution, no weight
                }
                // Reject texels whose stored depth disagrees with the hit
                // (occluded texels behind the hit point); Lumen's
                // |texelDepth - rayHitT| weight.
                const float depthDifference = std::fabs(card.depthTexels[texel] - depth);
                if (depthDifference > 0.25f + 0.1f * card.depthTexels[texel])
                {
                        continue;
                }
                const glm::vec3 radiance = card.directRadiance[texel]
                                         + (includeIndirect ? card.indirectRadiance[texel] : glm::vec3(0.0f));
                const float weight = alignment;
                outRadiance += radiance * weight;
                weightSum += glm::vec3(weight);
            }
        }
    }
    if (weightSum.x > 0.0f || weightSum.y > 0.0f || weightSum.z > 0.0f)
    {
        outRadiance /= glm::max(weightSum, glm::vec3(1.0e-6f));
    }
    // Unlit surfaces show their albedo as emission (renderer semantics).
    if (surface.unlit)
    {
        // The hit albedo is unknown here; unlit surfaces carry their radiance
        // in directRadiance (their capture skips lighting), so nothing extra.
    }
}

// Projects a radiance array onto the 9-coefficient SH2 basis (the GiProbe
// order) with uniform solid-angle weights; shared by the world probes and the
// screen-space probes. Constants verified against numerical integration.
void projectRadianceToSh2(const glm::vec3 *directions, const glm::vec3 *radiance, std::size_t count, float *outSh2)
{
    constexpr float kY00 = 0.282094792f;
    constexpr float kY1m = 0.488602512f;
    constexpr float kY2m = 1.092548431f;
    constexpr float kY20 = 0.315391565f;
    constexpr float kY22 = 0.546274215f;
    for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
    {
        outSh2[coefficient] = 0.0f;
    }
    const float solidAngle = count > 0 ? 4.0f * 3.141592654f / static_cast<float>(count) : 0.0f;
    for (std::size_t index = 0; index < count; ++index)
    {
        const glm::vec3 &dir = directions[index];
        const float      x = dir.x;
        const float      y = dir.y;
        const float      z = dir.z;
        const float      basis[9] = {kY00, kY1m * y, kY1m * z, kY1m * x, kY2m * x * y, kY2m * y * z,
            kY20 * (3.0f * z * z - 1.0f), kY2m * x * z, kY22 * (x * x - y * y)};
        for (std::uint32_t coefficient = 0; coefficient < 9; ++coefficient)
        {
            outSh2[coefficient * 3 + 0] += basis[coefficient] * radiance[index].r * solidAngle;
            outSh2[coefficient * 3 + 1] += basis[coefficient] * radiance[index].g * solidAngle;
            outSh2[coefficient * 3 + 2] += basis[coefficient] * radiance[index].b * solidAngle;
        }
    }
}

// Total radiance leaving a hit point: direct + indirect from the surface
// cache. This is what every bounce ray sees.
void radianceAtHit(const GiPipelineState &state, const GiHit &hit, bool includeIndirect, glm::vec3 &outRadiance)
{
    sampleSurfaceCache(state, hit.surfaceIndex, hit.position, hit.normal, includeIndirect, outRadiance);
}

// Card-captured albedo at a world-space point (SurfaceCache debug view).
void sampleSurfaceAlbedo(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outAlbedo)
{
    outAlbedo = glm::vec3(0.0f);
    if (surfaceIndex >= state.surfaces.size())
    {
        return;
    }
    const GiSurface &surface = state.surfaces[surfaceIndex];
    float            weightSum = 0.0f;
    for (std::uint32_t cardIndex = surface.firstCard; cardIndex < surface.firstCard + surface.cardCount; ++cardIndex)
    {
        const GiCard &card = state.cards[cardIndex];
        const glm::vec3 relative = position - card.origin;
        // u, v are face-centered coordinates in [-0.5, 0.5], matching the
        // capture parameterization (texel centers at (i + 0.5) / texels).
        const float     u        = glm::dot(relative, card.axisU) / std::max(card.width, 1.0e-6f);
        const float     v        = glm::dot(relative, card.axisV) / std::max(card.height, 1.0e-6f);
        const float     depth    = glm::dot(relative, -card.axisW);
        constexpr float halfTexelMargin = 0.5f + 1.0e-3f;
        if (depth < -0.05f || depth > card.depth + 0.05f || u < -halfTexelMargin || u > halfTexelMargin
            || v < -halfTexelMargin || v > halfTexelMargin)
        {
            continue;
        }
        const float alignment = glm::dot(card.axisW, normal);
        if (alignment <= 0.15f)
        {
            continue;
        }
        const float texelX = (u + 0.5f) * static_cast<float>(card.texelWidth) - 0.5f;
        const float texelY = (v + 0.5f) * static_cast<float>(card.texelHeight) - 0.5f;
        const std::int32_t x0 = static_cast<std::int32_t>(std::floor(texelX));
        const std::int32_t y0 = static_cast<std::int32_t>(std::floor(texelY));
        for (std::uint32_t dy = 0; dy < 2; ++dy)
        {
            for (std::uint32_t dx = 0; dx < 2; ++dx)
            {
                const std::int32_t x = std::clamp(x0 + static_cast<std::int32_t>(dx), 0,
                    static_cast<std::int32_t>(card.texelWidth) - 1);
                const std::int32_t y = std::clamp(y0 + static_cast<std::int32_t>(dy), 0,
                    static_cast<std::int32_t>(card.texelHeight) - 1);
                const std::size_t texel = static_cast<std::size_t>(y) * card.texelWidth + x;
                if (card.depthTexels[texel] >= invalidCardDepth)
                {
                    continue;
                }
                const float depthDifference = std::fabs(card.depthTexels[texel] - depth);
                if (depthDifference > 0.25f + 0.1f * card.depthTexels[texel])
                {
                    continue;
                }
                outAlbedo += card.albedo[texel] * alignment;
                weightSum += alignment;
            }
        }
    }
    if (weightSum > 1.0e-6f)
    {
        outAlbedo /= weightSum;
    }
}

// ---- direct lighting ---------------------------------------------------------

void gatherDirectLightingChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count)
{
    const GiEnvironmentSnapshot &environment = state.environment;
    const glm::vec3              sunRadiance = environment.sunRadiance();
    for (std::uint32_t item = 0; item < count; ++item)
    {
        const GiTexelWork &unit   = work[item];
        GiCard &          card    = state.cards[unit.cardIndex];
        const GiSurface & surface = state.surfaces[card.surfaceIndex];
        for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
        {
            if (card.depthTexels[texel] >= invalidCardDepth)
            {
                continue;
            }
            const glm::vec3 position = card.origin + card.axisU
                    * (((static_cast<float>(texel % card.texelWidth) + 0.5f) / static_cast<float>(card.texelWidth)
                           - 0.5f)
                              * card.width)
                + card.axisV
                    * (((static_cast<float>(texel / card.texelWidth) + 0.5f) / static_cast<float>(card.texelHeight)
                           - 0.5f)
                              * card.height);
            const glm::vec3 normal = card.normal[texel];
            glm::vec3       radiance(0.0f);
            if (surface.unlit)
            {
                // Unlit surfaces emit their albedo; no lighting is applied.
                radiance = card.albedo[texel];
            }
            else
            {
                // Irradiance from the shadowed sun plus the direct lights.
                glm::vec3 irradiance(0.0f);
                const float nDotL = glm::dot(normal, environment.sunDirection);
                if (nDotL > 0.0f && sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f)
                {
                    bool shadowed = false;
                    if (state.settings.sunShadows)
                    {
                        shadowed = sunShadowed(state, position, normal, card.surfaceIndex);
                    }
                    if (!shadowed)
                    {
                        irradiance += sunRadiance * nDotL;
                    }
                }
                irradiance += directLightRadiance(environment, position, normal);
                // Lambertian outgoing radiance: albedo / pi * E.
                radiance = card.albedo[texel] * (irradiance * (1.0f / 3.141592654f));
            }
            radiance += card.emissive[texel];
            card.directRadiance[texel] = radiance;
            card.lastDirectFrame = state.frameIndex;
        }
    }
    std::uint32_t processed = 0;
    for (std::uint32_t item = 0; item < count; ++item)
    {
        processed += work[item].texelEnd - work[item].texelBegin;
    }
    state.directTexelsThisFrame.fetch_add(processed, std::memory_order_relaxed);
}

// ---- indirect lighting (the feedback loop) ------------------------------------

void gatherIndirectLightingChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count)
{
    const std::uint32_t rayCount = std::max(state.settings.gatherRaysPerProbe, 1u);
    for (std::uint32_t item = 0; item < count; ++item)
    {
        const GiTexelWork &unit   = work[item];
        GiCard &          card    = state.cards[unit.cardIndex];
        const GiSurface & surface = state.surfaces[card.surfaceIndex];
        if (surface.unlit)
        {
            continue; // unlit surfaces do not receive indirect lighting
        }
        for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
        {
            if (card.depthTexels[texel] >= invalidCardDepth)
            {
                card.indirectRadiance[texel] = glm::vec3(0.0f);
                continue;
            }
            const glm::vec3 position = card.origin + card.axisU
                    * (((static_cast<float>(texel % card.texelWidth) + 0.5f) / static_cast<float>(card.texelWidth)
                           - 0.5f)
                              * card.width)
                + card.axisV
                    * (((static_cast<float>(texel / card.texelWidth) + 0.5f) / static_cast<float>(card.texelHeight)
                           - 0.5f)
                              * card.height);
            const glm::vec3 normal = card.normal[texel];
            const glm::mat3 basis = onb(normal);
            // Surface bias: start slightly off the surface; Lumen traces from
            // texels with a normal-dependent bias for the same reason.
            const glm::vec3 origin = position + normal * 0.02f;
            glm::vec3       gather(0.0f);
            for (std::uint32_t ray = 0; ray < rayCount; ++ray)
            {
                const std::uint64_t seed = state.frameIndex * 9781ull + texel * 131ull + ray;
                const glm::vec3     direction = glm::normalize(basis * cosineDirection(seed));
                GiRay               traceRay{origin, direction, state.settings.maxTraceDistance};
                const GiHit         hit = traceScene(state, traceRay);
                if (!hit.hit)
                {
                    gather += state.environment.skyRadiance(direction);
                    continue;
                }
                // Hits on geometric backfaces return zero radiance (Lumen's
                // anti-leak rule for texels inside geometry).
                if (glm::dot(hit.geometricNormal, direction) > 0.0f)
                {
                    continue;
                }
                glm::vec3 hitRadiance(0.0f);
                radianceAtHit(state, hit, true, hitRadiance);
                gather += hitRadiance;
            }
            // Cosine-weighted sampling: E = pi * mean(L_i), and the
            // Lambertian radiance is albedo / pi * E = albedo * mean(L_i).
            gather = card.albedo[texel] * (gather / static_cast<float>(rayCount));
            const glm::vec3 previous = card.indirectRadiance[texel];
            card.indirectRadiance[texel] =
                glm::mix(previous, gather, std::clamp(state.settings.indirectTemporalBlend, 0.0f, 1.0f));
            card.lastIndirectFrame = state.frameIndex;
        }
    }
    std::uint32_t processed = 0;
    for (std::uint32_t item = 0; item < count; ++item)
    {
        processed += work[item].texelEnd - work[item].texelBegin;
    }
    state.indirectTexelsThisFrame.fetch_add(processed, std::memory_order_relaxed);
}

// ---- world-space radiance cache ------------------------------------------------

namespace
{

// Evaluates one probe direction: trace, sample the surface cache, fall back
// to the skylight on miss. Shared by both clip levels.
glm::vec3 probeDirectionRadiance(GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &direction)
{
    GiRay       ray{position, direction, state.settings.maxTraceDistance};
    const GiHit hit = traceScene(state, ray);
    ++state.probeTracesThisFrame;
    if (!hit.hit)
    {
        return state.environment.skyRadiance(direction);
    }
    if (glm::dot(hit.geometricNormal, direction) > 0.0f)
    {
        return glm::vec3(0.0f); // backface hit: inside geometry, no light
    }
    glm::vec3 radiance(0.0f);
    radianceAtHit(state, hit, true, radiance);
    return radiance;
}

// Projects an octahedral radiance map to the 9-coefficient SH2 basis (the
// GiProbe order) using uniform solid-angle weights. Coefficients verified
// against numerical integration of smooth fields (see docs/overview.md).
// Kept for the probe-integration work that consumes it; [[maybe_unused]]
// silences the configurations where that path is compiled out.
[[maybe_unused]] void projectSh2(const std::vector<glm::vec3> &radiance, const std::vector<glm::vec3> &directions,
    const std::vector<float> &solidAngles, float *outSh2 /* 9 x 3 */)
{
    for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
    {
        outSh2[coefficient] = 0.0f;
    }
    static const float kBasis[9][3] = {
        // Per-direction SH2 basis values are evaluated on the fly below; this
        // table documents the order: C00, C1-1, C10, C11, C2-2, C2-1, C20,
        // C21, C22.
    };
    (void) kBasis;
    constexpr float kY00    = 0.282094792f;
    constexpr float kY1m    = 0.488602512f;
    constexpr float kY2m    = 1.092548431f;
    constexpr float kY20    = 0.315391565f;
    constexpr float kY22    = 0.546274215f;
    for (std::size_t direction = 0; direction < directions.size(); ++direction)
    {
        const glm::vec3 &dir    = directions[direction];
        const float      weight = solidAngles[direction];
        const float      x = dir.x;
        const float      y = dir.y;
        const float      z = dir.z;
        const float      basis[9] = {kY00,
            kY1m * y,
            kY1m * z,
            kY1m * x,
            kY2m * x * y,
            kY2m * y * z,
            kY20 * (3.0f * z * z - 1.0f),
            kY2m * x * z,
            kY22 * (x * x - y * y)};
        const glm::vec3 &value = radiance[direction];
        for (std::uint32_t coefficient = 0; coefficient < 9; ++coefficient)
        {
            outSh2[coefficient * 3 + 0] += basis[coefficient] * value.r * weight;
            outSh2[coefficient * 3 + 1] += basis[coefficient] * value.g * weight;
            outSh2[coefficient * 3 + 2] += basis[coefficient] * value.b * weight;
        }
    }
}

} // namespace

void updateProbeChunk(GiPipelineState &state, std::uint32_t levelIndex, std::uint32_t begin, std::uint32_t end)
{
    GiPipelineState::ProbeLevel &level = levelIndex == 0 ? state.level0 : state.level1;
    const std::uint32_t          dimension = level.dimensions[0];
    const float                  hysteresis = std::clamp(state.settings.probeHysteresis, 0.0f, 0.99f);
    for (std::uint32_t probeIndex = begin; probeIndex < end && probeIndex < level.probes.size(); ++probeIndex)
    {
        GiPipelineState::ProbeLevel::Probe &probe = level.probes[probeIndex];
        // Position from the lattice: consistent with snapProbeLevels.
        const std::uint32_t x = probeIndex % dimension;
        const std::uint32_t y = (probeIndex / dimension) % dimension;
        const std::uint32_t z = probeIndex / (dimension * dimension);
        probe.position = level.anchor
            + glm::vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)) * level.spacing;
        // Skip probes inside geometry: their radiance is meaningless (Lumen
        // builds its probes on surfaces; ours trace from lattice points and
        // mark enclosed ones invalid).
        {
            const glm::vec3 up(0.0f, 1.0f, 0.0f);
            if (traceSceneOccluded(state, GiRay{probe.position + up * 0.01f, up, 0.05f}, 0xffffffffu))
            {
                probe.valid   = false;
                probe.validity = 0.0f;
                continue;
            }
        }
        const std::size_t directionCount = state.probeDirections.size();
        probe.radiancePrevious.swap(probe.radiance);
        for (std::size_t direction = 0; direction < directionCount; ++direction)
        {
            probe.radiance[direction] = probeDirectionRadiance(state, probe.position, state.probeDirections[direction]);
        }
        if (hysteresis > 0.0f)
        {
            for (std::size_t direction = 0; direction < directionCount; ++direction)
            {
                probe.radiance[direction] = glm::mix(probe.radiance[direction], probe.radiancePrevious[direction],
                    hysteresis);
            }
        }
        projectRadianceToSh2(state.probeDirections.data(), probe.radiance.data(), probe.radiance.size(),
            probe.sh2.data());
        probe.valid     = true;
        probe.validity  = 1.0f;
        probe.lastUpdate = state.frameIndex;
    }
}

} // namespace lumen_gi
} // namespace engine
