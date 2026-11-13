// lumen_gi lighting side: the Lumen Scene lighting passes.
//
// - Direct lighting evaluates sun + direct lights per card texel (amortized
//   through a per-frame texel budget with per-card rotating cursors; untouched
//   texels keep their cached value, Lumen's "Lighting Update Speed" behavior).
// - Indirect lighting runs the final gather in texture space: budgeted texels
//   trace a few stratified cosine rays that read direct + the COMMITTED
//   indirect radiance from the surface cache. New values go to a staging
//   buffer and are committed after every task finished, so concurrent traces
//   never see a half-updated cache and untouched texels keep their value.
// - The world-space radiance cache updates amortized octahedral probes whose
//   rays sample the same surface cache; misses fall back to the hemisphere
//   skylight.

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace lumen_gi
{

// ---- direct light evaluation ---------------------------------------------------

// Shadow test toward the sun with a surface bias; rays that start inside
// geometry are caught by the surface/backface handling of the caller. The
// receiving surface is NOT excluded: a mesh must be able to shadow itself.
bool sunShadowed(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal)
{
    const glm::vec3 sunDirection = state.environment.sunDirection;
    const float     bias         = 0.02f + 0.02f * std::fabs(glm::dot(normal, sunDirection));
    GiRay           ray{position + normal * bias, sunDirection, state.settings.maxTraceDistance};
    return traceSceneOccluded(state, ray, noSurface);
}

// Directional/point/spot radiance arriving at position from the module's
// light snapshot; formulas mirror the renderer's shading (range fade
// (1 - (d/r)^2)^2 / d^2, smoothstep cone). Shadow rays follow
// lighting.point_light_shadows. Light directions and cone cosines come from
// the snapshot's prepared array (see GiEnvironmentSnapshot), so the loop
// body is pure per-texel math.
glm::vec3 directLightRadiance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal)
{
    const GiEnvironmentSnapshot &environment = state.environment;
    glm::vec3                    sum(0.0f);
    if (!environment.hasDirectLights)
    {
        return sum;
    }
    const bool shadows = state.settings.pointLightShadows;
    const glm::vec3 origin = position + normal * 0.03f;
    for (const GiEnvironmentSnapshot::PreparedDirectLight &light : environment.preparedLights)
    {
        const glm::vec3 &color     = light.color;
        const float      intensity = light.intensity;
        if (light.type == direct_lighting::DirectLightType::Directional)
        {
            const glm::vec3 &direction = light.direction;
            const float nDotL = glm::dot(normal, direction);
            if (nDotL > 0.0f)
            {
                if (shadows && traceSceneOccluded(state, GiRay{origin, direction, state.settings.maxTraceDistance}, noSurface))
                {
                    continue;
                }
                sum += color * intensity * nDotL;
            }
            continue;
        }
        const glm::vec3 toLight           = light.position - position;
        const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
        const float     rangeRatioSquared = distanceSquared / (light.range * light.range);
        if (rangeRatioSquared >= 1.0f)
        {
            continue;
        }
        const float     distance    = std::sqrt(distanceSquared);
        const glm::vec3 direction   = toLight * (1.0f / distance);
        const float     rangeFade   = 1.0f - rangeRatioSquared * rangeRatioSquared;
        float           attenuation = rangeFade * rangeFade / distanceSquared;
        if (light.type == direct_lighting::DirectLightType::Spot)
        {
            const float coneCosine = glm::dot(-direction, light.direction);
            const float coneT =
                std::clamp((coneCosine - light.outerCosine) / light.coneDivisor, 0.0f, 1.0f);
            attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
        }
        const float nDotL = glm::dot(normal, direction);
        if (nDotL <= 0.0f || attenuation <= 0.0f)
        {
            continue;
        }
        if (shadows && distance > 0.06f
            && traceSceneOccluded(state, GiRay{origin, direction, distance - 0.06f}, noSurface))
        {
            continue;
        }
        sum += color * intensity * attenuation * nDotL;
    }
    return sum;
}

// ---- surface cache sampling -------------------------------------------------

namespace
{

// Projects a local-space hit onto the surface's cards and blends their cached
// values: candidate cards are the faces whose outward axis aligns with the hit
// normal; the four bilinear texels of every depth layer are weighted by the
// bilinear footprint and the normal alignment, and rejected when their stored
// depth disagrees with the hit (Lumen's |texelDepth - rayHitT| rule). A second
// relaxed pass runs only when the strict pass found no coverage, so grazing
// silhouettes do not turn into black holes.
template <bool WantAlbedo>
bool sampleCardsPass(const GiSurface &surface, const glm::vec3 &localPosition, const glm::vec3 &localNormal,
    bool includeIndirect, bool relaxed, glm::vec3 &outRadiance, glm::vec3 &outAlbedo)
{
    glm::vec3 radianceSum(0.0f);
    glm::vec3 albedoSum(0.0f);
    float     weightSum = 0.0f;
    const float alignmentThreshold = relaxed ? 0.02f : 0.15f;
    const float toleranceScale     = relaxed ? 3.0f : 1.0f;
    for (const GiCard &card : surface.cards)
    {
        const float alignment = glm::dot(card.axisW, localNormal);
        if (alignment <= alignmentThreshold)
        {
            continue;
        }
        const glm::vec3 relative = localPosition - card.origin;
        const float u = glm::dot(relative, card.axisU) / std::max(card.width, 1.0e-9f);
        const float v = glm::dot(relative, card.axisV) / std::max(card.height, 1.0e-9f);
        constexpr float margin = 0.5f + 1.0e-3f;
        if (u < -margin || u > margin || v < -margin || v > margin)
        {
            continue;
        }
        const float queryDepth = glm::dot(relative, -card.axisW);
        if (queryDepth < -2.0f * card.nearBias || queryDepth > card.depth)
        {
            continue;
        }
        // Depth tolerance in world units converted back to the card's local
        // units through the instance scale along the card axis.
        const float worldScale = std::max(surface.axisScale[card.axisIndex], 1.0e-6f);
        const float texelX = (u + 0.5f) * static_cast<float>(card.texelWidth) - 0.5f;
        const float texelY = (v + 0.5f) * static_cast<float>(card.texelHeight) - 0.5f;
        const float fx0 = std::floor(texelX);
        const float fy0 = std::floor(texelY);
        const float fx  = texelX - fx0;
        const float fy  = texelY - fy0;
        const std::int32_t x0 = static_cast<std::int32_t>(fx0);
        const std::int32_t y0 = static_cast<std::int32_t>(fy0);
        const std::size_t  plane = card.planeTexels();
        for (std::uint32_t dy = 0; dy < 2; ++dy)
        {
            for (std::uint32_t dx = 0; dx < 2; ++dx)
            {
                const float bilinear = (dx != 0 ? fx : 1.0f - fx) * (dy != 0 ? fy : 1.0f - fy);
                if (bilinear <= 0.0f)
                {
                    continue;
                }
                const std::int32_t x = std::clamp(x0 + static_cast<std::int32_t>(dx), 0,
                    static_cast<std::int32_t>(card.texelWidth) - 1);
                const std::int32_t y = std::clamp(y0 + static_cast<std::int32_t>(dy), 0,
                    static_cast<std::int32_t>(card.texelHeight) - 1);
                const std::size_t xy = static_cast<std::size_t>(y) * card.texelWidth + x;
                for (std::uint32_t layer = 0; layer < card.layers; ++layer)
                {
                    const std::size_t texel = layer * plane + xy;
                    const float       stored = card.depthTexels[texel];
                    if (stored >= invalidCardDepth)
                    {
                        break; // layers are depth ordered: the rest are empty too
                    }
                    const float worldTolerance = toleranceScale * (0.25f + 0.1f * stored * worldScale);
                    if (std::fabs(stored - queryDepth) * worldScale > worldTolerance)
                    {
                        continue;
                    }
                    const float weight = alignment * bilinear;
                    radianceSum += (card.directRadiance[texel]
                                       + (includeIndirect ? card.indirectRadiance[texel] : glm::vec3(0.0f)))
                        * weight;
                    if constexpr (WantAlbedo)
                    {
                        albedoSum += card.albedo[texel] * weight;
                    }
                    weightSum += weight;
                }
            }
        }
    }
    if (weightSum <= 1.0e-6f)
    {
        return false;
    }
    outRadiance = radianceSum / weightSum;
    if constexpr (WantAlbedo)
    {
        outAlbedo = albedoSum / weightSum;
    }
    return true;
}

} // namespace

void sampleSurfaceCacheLocal(const GiPipelineState &state, const GiSurface &surface, const glm::vec3 &localPosition,
    const glm::vec3 &localNormal, bool includeIndirect, glm::vec3 &outRadiance)
{
    (void) state;
    outRadiance = glm::vec3(0.0f);
    glm::vec3 unused(0.0f);
    if (!sampleCardsPass<false>(surface, localPosition, localNormal, includeIndirect, false, outRadiance, unused))
    {
        sampleCardsPass<false>(surface, localPosition, localNormal, includeIndirect, true, outRadiance, unused);
    }
}

void sampleSurfaceAlbedoLocal(const GiPipelineState &state, const GiSurface &surface, const glm::vec3 &localPosition,
    const glm::vec3 &localNormal, glm::vec3 &outAlbedo)
{
    (void) state;
    outAlbedo = glm::vec3(0.0f);
    glm::vec3 unused(0.0f);
    if (!sampleCardsPass<true>(surface, localPosition, localNormal, false, false, unused, outAlbedo))
    {
        sampleCardsPass<true>(surface, localPosition, localNormal, false, true, unused, outAlbedo);
    }
}

void sampleSurfaceCache(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, bool includeIndirect, glm::vec3 &outRadiance)
{
    outRadiance = glm::vec3(0.0f);
    if (surfaceIndex >= state.surfaces.size())
    {
        return;
    }
    const GiSurface &surface = state.surfaces[surfaceIndex];
    const glm::vec3  local   = glm::vec3(surface.inverseTransform * glm::vec4(position, 1.0f));
    const glm::vec3  localNormal = glm::normalize(glm::transpose(surface.linear) * normal);
    sampleSurfaceCacheLocal(state, surface, local, localNormal, includeIndirect, outRadiance);
}

void sampleSurfaceAlbedo(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outAlbedo)
{
    outAlbedo = glm::vec3(0.0f);
    if (surfaceIndex >= state.surfaces.size())
    {
        return;
    }
    const GiSurface &surface = state.surfaces[surfaceIndex];
    const glm::vec3  local   = glm::vec3(surface.inverseTransform * glm::vec4(position, 1.0f));
    const glm::vec3  localNormal = glm::normalize(glm::transpose(surface.linear) * normal);
    sampleSurfaceAlbedoLocal(state, surface, local, localNormal, outAlbedo);
}

// Total radiance leaving a hit point: direct + indirect from the surface
// cache. Back-face hits return nothing (Lumen's anti-leak rule).
void radianceAtHit(const GiPipelineState &state, const GiHit &hit, bool includeIndirect, glm::vec3 &outRadiance)
{
    outRadiance = glm::vec3(0.0f);
    if (hit.backface)
    {
        return;
    }
    sampleSurfaceCacheLocal(state, state.surfaces[hit.surfaceIndex], hit.localPosition, hit.localNormal,
        includeIndirect, outRadiance);
}

// ---- SH2 -----------------------------------------------------------------------

// Projection onto the 9-coefficient basis in the GiProbe order. The stored
// coefficient is c_lm * K_lm (K = the basis normalization constant): then the
// cosine-lobe irradiance evaluation in capabilities.hpp / the renderer is the
// plain polynomial Sum A_l * s_lm * f_lm(n) and a constant radiance L yields
// exactly pi * L (the old storage was missing this factor, which made every
// probe-lit surface ~3.5x too bright).
void projectRadianceToSh2(
    const glm::vec3 *directions, const glm::vec3 *radiance, const float *weights, std::size_t count, float *outSh2)
{
    constexpr float kY00 = 0.282094792f;
    constexpr float kY1m = 0.488602512f;
    constexpr float kY2m = 1.092548431f;
    constexpr float kY20 = 0.315391565f;
    constexpr float kY22 = 0.546274215f;
    constexpr float kConstants[9] = {kY00, kY1m, kY1m, kY1m, kY2m, kY2m, kY20, kY2m, kY22};
    double accumulator[27] = {};
    for (std::size_t index = 0; index < count; ++index)
    {
        const glm::vec3 &dir = directions[index];
        const float      x = dir.x;
        const float      y = dir.y;
        const float      z = dir.z;
        // Polynomial parts f_lm; the true basis value is K_lm * f_lm.
        const float polynomial[9] = {1.0f, y, z, x, x * y, y * z, 3.0f * z * z - 1.0f, x * z, x * x - y * y};
        const glm::vec3 value = radiance[index] * weights[index];
        for (std::uint32_t coefficient = 0; coefficient < 9; ++coefficient)
        {
            const float basis = kConstants[coefficient] * polynomial[coefficient] * kConstants[coefficient];
            accumulator[coefficient * 3 + 0] += static_cast<double>(basis * value.r);
            accumulator[coefficient * 3 + 1] += static_cast<double>(basis * value.g);
            accumulator[coefficient * 3 + 2] += static_cast<double>(basis * value.b);
        }
    }
    for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
    {
        outSh2[coefficient] = static_cast<float>(accumulator[coefficient]);
    }

    // De-ringing: a band-limited fit of a high-contrast field (a small bright
    // emitter) rings below zero on the opposite side, which would subtract
    // light in the renderer. Shrink bands 1-2 just enough that the cosine
    // irradiance stays non-negative along 26 probe normals.
    static const glm::vec3 testNormals[26] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {0.577350f, 0.577350f, 0.577350f}, {-0.577350f, 0.577350f, 0.577350f},
        {0.577350f, -0.577350f, 0.577350f}, {-0.577350f, -0.577350f, 0.577350f},
        {0.577350f, 0.577350f, -0.577350f}, {-0.577350f, 0.577350f, -0.577350f},
        {0.577350f, -0.577350f, -0.577350f}, {-0.577350f, -0.577350f, -0.577350f},
        {0.707107f, 0.707107f, 0}, {-0.707107f, 0.707107f, 0}, {0.707107f, -0.707107f, 0},
        {-0.707107f, -0.707107f, 0}, {0.707107f, 0, 0.707107f}, {-0.707107f, 0, 0.707107f},
        {0.707107f, 0, -0.707107f}, {-0.707107f, 0, -0.707107f}, {0, 0.707107f, 0.707107f},
        {0, -0.707107f, 0.707107f}, {0, 0.707107f, -0.707107f}, {0, -0.707107f, -0.707107f}};
    float shrink = 1.0f;
    for (std::uint32_t channel = 0; channel < 3; ++channel)
    {
        const float base = kPi * outSh2[channel];
        for (const glm::vec3 &normal : testNormals)
        {
            const float n[3] = {normal.x, normal.y, normal.z};
            float       e[3];
            evaluateSh2Irradiance(outSh2, n, e);
            if (e[channel] < 0.0f && base > e[channel])
            {
                shrink = std::min(shrink, std::max(base / (base - e[channel]), 0.0f));
            }
        }
    }
    if (shrink < 1.0f)
    {
        for (std::uint32_t coefficient = 3; coefficient < 27; ++coefficient)
        {
            outSh2[coefficient] *= shrink;
        }
    }
}

// ---- direct lighting --------------------------------------------------------------

void gatherDirectLightingUnit(GiPipelineState &state, const GiTexelWork &unit)
{
    const GiEnvironmentSnapshot &environment = state.environment;
    const glm::vec3              sunRadiance = environment.sunRadiance();
    const bool                   hasSun      = sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f;
    GiCard &                     card        = state.card(unit.cardIndex);
    const GiSurface &            surface     = state.cardSurface(unit.cardIndex);
    for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
    {
        if (card.depthTexels[texel] >= invalidCardDepth)
        {
            card.directRadiance[texel] = glm::vec3(0.0f);
            continue;
        }
        glm::vec3 radiance(0.0f);
        if (surface.unlit)
        {
            // Unlit surfaces emit their albedo; no lighting is applied.
            radiance = card.albedo[texel];
        }
        else
        {
            const glm::vec3 position = glm::vec3(surface.transform * glm::vec4(cardTexelLocalPosition(card, texel), 1.0f));
            const glm::vec3 normal   = glm::normalize(surface.normalMatrix * card.normal[texel]);
            glm::vec3 irradiance(0.0f);
            const float nDotL = glm::dot(normal, environment.sunDirection);
            if (nDotL > 0.0f && hasSun)
            {
                if (!state.settings.sunShadows || !sunShadowed(state, position, normal))
                {
                    irradiance += sunRadiance * nDotL;
                }
            }
            irradiance += directLightRadiance(state, position, normal);
            // Lambertian outgoing radiance: albedo / pi * E.
            radiance = card.albedo[texel] * (irradiance * (1.0f / kPi));
        }
        radiance += surface.emissive;
        card.directRadiance[texel] = radiance;
    }
    state.directTexelsThisFrame.fetch_add(unit.texelEnd - unit.texelBegin, std::memory_order_relaxed);
}

// ---- indirect lighting (the feedback loop) --------------------------------------

void gatherIndirectLightingUnit(GiPipelineState &state, const GiTexelWork &unit, glm::vec3 *scratch)
{
    const std::uint32_t rayCount = std::max(state.settings.indirectRaysPerTexel, 1u);
    const float         history  = std::clamp(state.settings.indirectTemporalBlend, 0.0f, 0.999f);
    GiCard &            card     = state.card(unit.cardIndex);
    const GiSurface &   surface  = state.cardSurface(unit.cardIndex);
    std::uint32_t       rays     = 0;
    for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
    {
        glm::vec3 &out = scratch[texel - unit.texelBegin];
        out            = card.indirectRadiance[texel];
        if (surface.unlit)
        {
            continue; // unlit surfaces do not receive indirect lighting
        }
        if (card.depthTexels[texel] >= invalidCardDepth)
        {
            out                      = glm::vec3(0.0f);
            card.indirectSamples[texel] = 0;
            continue;
        }
        const glm::vec3 position = glm::vec3(surface.transform * glm::vec4(cardTexelLocalPosition(card, texel), 1.0f));
        const glm::vec3 normal   = glm::normalize(surface.normalMatrix * card.normal[texel]);
        const glm::mat3 basis    = makeBasis(normal);
        // Surface bias: start slightly off the surface (Lumen biases its
        // texel traces the same way).
        const glm::vec3 origin = position + normal * 0.02f;
        std::uint64_t   seed   = hashSeed(state.frameIndex * 0x9E3779B1ull + (static_cast<std::uint64_t>(unit.cardIndex) << 32)
                                       + texel);
        glm::vec3 gather(0.0f);
        for (std::uint32_t ray = 0; ray < rayCount; ++ray)
        {
            // Stratified over the cosine-weighted hemisphere: ray r owns the
            // r-th slice of the radial axis.
            const float u = (static_cast<float>(ray) + randomFloat(seed)) / static_cast<float>(rayCount);
            const float v = randomFloat(seed);
            const glm::vec3 direction = glm::normalize(basis * cosineHemisphere(u, v));
            GiRay           traceRay{origin, direction, state.settings.maxTraceDistance};
            const GiHit     hit = traceScene(state, traceRay);
            ++rays;
            if (!hit.hit)
            {
                gather += state.environment.skyRadiance(direction);
                continue;
            }
            glm::vec3 hitRadiance(0.0f);
            radianceAtHit(state, hit, true, hitRadiance);
            gather += hitRadiance;
        }
        // Cosine-weighted sampling: E = pi * mean(L_i), and the Lambertian
        // radiance is albedo / pi * E = albedo * mean(L_i).
        gather = card.albedo[texel] * (gather / static_cast<float>(rayCount));
        // Exponential history with a ramp-up: the first update takes the
        // fresh sample whole, then the weight settles at (1 - history).
        const std::uint32_t seen  = std::min<std::uint32_t>(card.indirectSamples[texel] + 1u, 255u);
        const float         alpha = std::max(1.0f / static_cast<float>(seen), 1.0f - history);
        out                       = glm::mix(card.indirectRadiance[texel], gather, alpha);
        card.indirectSamples[texel] = static_cast<std::uint8_t>(seen);
    }
    state.indirectTexelsThisFrame.fetch_add(unit.texelEnd - unit.texelBegin, std::memory_order_relaxed);
    state.indirectRaysThisFrame.fetch_add(rays, std::memory_order_relaxed);
}

void commitIndirectUnit(GiPipelineState &state, const GiTexelWork &unit, const glm::vec3 *scratch)
{
    GiCard &card = state.card(unit.cardIndex);
    std::copy(scratch, scratch + (unit.texelEnd - unit.texelBegin), card.indirectRadiance.begin() + unit.texelBegin);
}

// ---- world-space radiance cache ------------------------------------------------

void updateProbe(GiPipelineState &state, std::uint32_t levelIndex, std::uint32_t slot)
{
    GiProbeLevel &level = levelIndex == 0 ? state.level0 : state.level1;
    GiProbeLevel::Probe &probe = level.probes[slot];
    const std::size_t directionCount = state.probeDirections.size();
    if (probe.radiance.size() != directionCount)
    {
        return;
    }
    // A rotating stratum of the direction map is re-traced per update (every
    // K-th texel, K from probes.rays_per_update); a probe without data traces
    // everything once. Untouched texels keep their value.
    const std::uint32_t raysBudget = std::max(state.settings.probeRaysPerUpdate, 1u);
    std::uint32_t       stride     = static_cast<std::uint32_t>((directionCount + raysBudget - 1) / raysBudget);
    stride                         = probe.fresh ? 1u : std::max(stride, 1u);
    const std::uint32_t phase      = probe.phase % stride;
    const float hysteresis = probe.fresh ? 0.0f : std::clamp(state.settings.probeHysteresis, 0.0f, 0.99f);

    std::vector<std::uint32_t> backfaceTexels;
    glm::vec3     frontSum(0.0f);
    std::uint32_t frontCount = 0;
    std::uint32_t traced     = 0;
    for (std::size_t texel = phase; texel < directionCount; texel += stride)
    {
        const glm::vec3 &direction = state.probeDirections[texel];
        const GiHit      hit = traceScene(state, GiRay{probe.position, direction, state.settings.maxTraceDistance});
        ++traced;
        glm::vec3 value(0.0f);
        if (!hit.hit)
        {
            value = state.environment.skyRadiance(direction);
        }
        else if (hit.backface)
        {
            backfaceTexels.push_back(static_cast<std::uint32_t>(texel));
            continue;
        }
        else
        {
            radianceAtHit(state, hit, true, value);
        }
        frontSum += value;
        ++frontCount;
        probe.radiance[texel] = glm::mix(value, probe.radiance[texel], hysteresis);
    }
    // Rays that hit the back of a surface carry no information; fill them with
    // the mean of the rest so a probe hugging a wall keeps its energy instead
    // of losing the wall's share of the sphere.
    const glm::vec3 fill = frontCount > 0 ? frontSum / static_cast<float>(frontCount) : glm::vec3(0.0f);
    for (const std::uint32_t texel : backfaceTexels)
    {
        probe.radiance[texel] = glm::mix(fill, probe.radiance[texel], hysteresis);
    }
    state.probeTracesThisFrame.fetch_add(traced, std::memory_order_relaxed);

    // Probes inside geometry (most rays strike back faces) are excluded from
    // interpolation; a partial backface share lowers the validity smoothly.
    const float backfaceFraction = traced > 0 ? static_cast<float>(backfaceTexels.size()) / static_cast<float>(traced) : 0.0f;
    probe.enclosed = backfaceFraction > 0.5f;
    probe.validity = probe.enclosed ? 0.0f : std::clamp(1.0f - (backfaceFraction - 0.15f) / 0.35f, 0.0f, 1.0f);
    projectRadianceToSh2(state.probeDirections.data(), probe.radiance.data(), state.probeDirectionWeights.data(),
        directionCount, probe.sh2.data());
    probe.fresh      = false;
    probe.lastUpdate = state.frameIndex;
    ++probe.phase;
}

namespace
{

struct LatticeCorners
{
    std::uint32_t slots[8];
    float         weights[8];
};

// Trilinear corner slots / weights of a world position inside one level.
bool latticeCorners(const GiProbeLevel &level, const glm::vec3 &position, LatticeCorners &out)
{
    const std::int32_t dim = static_cast<std::int32_t>(level.dimension());
    if (dim < 2 || level.probes.empty())
    {
        return false;
    }
    const glm::vec3 logical = position / level.spacing - glm::vec3(level.originCell);
    const float     limit   = static_cast<float>(dim - 1);
    if (logical.x < 0.0f || logical.y < 0.0f || logical.z < 0.0f || logical.x > limit || logical.y > limit
        || logical.z > limit)
    {
        return false;
    }
    const glm::vec3 base = glm::min(glm::floor(logical), glm::vec3(static_cast<float>(dim - 2)));
    const glm::vec3 fraction = logical - base;
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const std::uint32_t cx = static_cast<std::uint32_t>(base.x) + (corner & 1u);
        const std::uint32_t cy = static_cast<std::uint32_t>(base.y) + ((corner >> 1) & 1u);
        const std::uint32_t cz = static_cast<std::uint32_t>(base.z) + ((corner >> 2) & 1u);
        const float wx = (corner & 1u) != 0u ? fraction.x : 1.0f - fraction.x;
        const float wy = ((corner >> 1) & 1u) != 0u ? fraction.y : 1.0f - fraction.y;
        const float wz = ((corner >> 2) & 1u) != 0u ? fraction.z : 1.0f - fraction.z;
        out.slots[corner]   = level.slotOfLogical(cx, cy, cz);
        out.weights[corner] = wx * wy * wz;
    }
    return true;
}

} // namespace

bool worldCacheLevelRadiance(const GiPipelineState &state, const GiProbeLevel &level, const glm::vec3 &position,
    const glm::vec3 &direction, glm::vec3 &outRadiance)
{
    LatticeCorners corners;
    if (!latticeCorners(level, position, corners))
    {
        return false;
    }
    const std::uint32_t resolution = state.settings.probeDirectionResolution;
    const std::uint32_t texel      = octahedralTexelIndex(direction, resolution);
    glm::vec3           sum(0.0f);
    float               weightSum = 0.0f;
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const GiProbeLevel::Probe &probe = level.probes[corners.slots[corner]];
        const float weight = corners.weights[corner] * probe.validity;
        if (weight <= 0.0f || probe.radiance.size() <= texel)
        {
            continue;
        }
        sum += probe.radiance[texel] * weight;
        weightSum += weight;
    }
    if (weightSum <= 1.0e-6f)
    {
        return false;
    }
    outRadiance = sum / weightSum;
    return true;
}

bool worldCacheIrradiance(
    const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance)
{
    const float normalArray[3] = {normal.x, normal.y, normal.z};
    for (const GiProbeLevel *level : {&state.level0, &state.level1})
    {
        LatticeCorners corners;
        if (!latticeCorners(*level, position, corners))
        {
            continue;
        }
        glm::vec3 sum(0.0f);
        float     weightSum = 0.0f;
        for (std::uint32_t corner = 0; corner < 8; ++corner)
        {
            const GiProbeLevel::Probe &probe = level->probes[corners.slots[corner]];
            const float weight = corners.weights[corner] * probe.validity;
            if (weight <= 0.0f)
            {
                continue;
            }
            float irradiance[3];
            evaluateSh2Irradiance(probe.sh2.data(), normalArray, irradiance);
            sum += glm::vec3(irradiance[0], irradiance[1], irradiance[2]) * weight;
            weightSum += weight;
        }
        if (weightSum > 1.0e-6f)
        {
            outIrradiance = sum / weightSum;
            return true;
        }
    }
    return false;
}

} // namespace lumen_gi
} // namespace engine
