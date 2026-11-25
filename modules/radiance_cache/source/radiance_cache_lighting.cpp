// radiance_cache lighting side: the Lumen Scene lighting passes.
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

#include "radiance_cache_internal.hpp"

#include <cstddef>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace engine
{
namespace radiance_cache
{

// ---- direct light evaluation ---------------------------------------------------

// Shadow transmittance toward the sun with a surface bias; rays that start
// inside geometry are caught by the surface/backface handling of the caller.
// The receiving surface is NOT excluded: a mesh must be able to shadow
// itself. Transparent surfaces soften the shadow by their (1 - opacity)
// transmittance instead of blocking it (traceSceneTransmittance).
float sunShadowTransmittance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal)
{
    const glm::vec3 sunDirection = state.environment.sunDirection;
    // The bias grows with the grazing angle so grazing-lit surfaces keep
    // their shadow rays off their own texels (trace.bias_m): a ray leaving a
    // surface head-on clears it immediately, while a grazing ray travels
    // nearly parallel to it and needs the full offset to clear the same
    // texels.
    const float bias = state.settings.traceBias
        + state.settings.traceBias * (1.0f - std::fabs(glm::dot(normal, sunDirection)));
    GiRay           ray{position + normal * bias, sunDirection, state.settings.maxTraceDistance};
    return traceSceneTransmittance(state, ray);
}

// Directional/point/spot radiance arriving at position from the module's
// light snapshot. The shared lighting::lightArrival evaluates the renderer's
// attenuation formulas (range fade (1 - (d/r)^2)^2 / d^2, smoothstep cone)
// and hands back the shadow ray's direction and distance; shadow rays follow
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
    const bool      shadows = state.settings.pointLightShadows;
    const glm::vec3 origin  = position + normal * state.settings.traceBias;
    for (const GiEnvironmentSnapshot::PreparedDirectLight &light : environment.preparedLights)
    {
        glm::vec3 direction(0.0f);
        float     distance = 0.0f;
        glm::vec3 arrival(0.0f);
        if (!lighting::lightArrival(light, position, normal, direction, distance, arrival))
        {
            continue;
        }
        if (!shadows)
        {
            sum += arrival;
            continue;
        }
        if (light.type == direct_lighting::DirectLightType::Directional)
        {
            const float transmittance
                = traceSceneTransmittance(state, GiRay{origin, direction, state.settings.maxTraceDistance});
            if (transmittance > 0.0f)
            {
                sum += arrival * transmittance;
            }
            continue;
        }
        // Point-light shadow rays start at the same surface bias as the
        // other trace sites and are skipped entirely for very close
        // lights, where a shadow ray would immediately re-hit the
        // receiver (trace.bias_m). Transparent surfaces soften the
        // shadow by their transmittance.
        const float shadowMinimum = state.settings.traceBias * 3.0f;
        if (distance <= shadowMinimum)
        {
            sum += arrival;
            continue;
        }
        const float transmittance
            = traceSceneTransmittance(state, GiRay{origin, direction, distance - shadowMinimum});
        if (transmittance > 0.0f)
        {
            sum += arrival * transmittance;
        }
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
    // Cards are axis-aligned by construction (buildCardCandidates places the
    // U/V/W axes on unit local axes), so the projections used here read the
    // matching components directly: dot(v, e_i) is that component exactly, and
    // dot(v, -w) with w = +/-e_axis is -v_axis times the card's outward sign.
    // The arithmetic is identical to the dot products it replaces.
    for (const GiCard &card : surface.cards)
    {
        const std::uint32_t axis = card.axisIndex;
        const float        wSign = card.axisW[axis];
        const float        alignment = localNormal[axis] * wSign;
        if (alignment <= alignmentThreshold)
        {
            continue;
        }
        const glm::vec3 relative = localPosition - card.origin;
        const float u = relative[(axis + 1u) % 3u] / std::max(card.width, 1.0e-9f);
        const float v = relative[(axis + 2u) % 3u] / std::max(card.height, 1.0e-9f);
        constexpr float margin = 0.5f + 1.0e-3f;
        if (u < -margin || u > margin || v < -margin || v > margin)
        {
            continue;
        }
        const float queryDepth = -relative[axis] * wSign;
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
                    radianceSum += (card.directRadiance[texel] + card.causticRadiance[texel]
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
void radianceAtHit(const GiPipelineState &state, const GiShadingHit &hit, bool includeIndirect, glm::vec3 &outRadiance)
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
// probe-lit surface ~3.5x too bright). The per-direction basis values
// (K_lm^2 * f_lm) come precomputed from the direction table, so the hot loop
// is a fused multiply-add per coefficient and channel.
void projectRadianceToSh2(
    const float *basis, const glm::vec3 *radiance, const float *weights, std::size_t count, float *outSh2)
{
    float accumulator[27] = {};
    for (std::size_t index = 0; index < count; ++index)
    {
        const float      *directionBasis = basis + index * 9;
        const glm::vec3 value = radiance[index] * weights[index];
        for (std::uint32_t coefficient = 0; coefficient < 9; ++coefficient)
        {
            const float projected = directionBasis[coefficient];
            accumulator[coefficient * 3 + 0] += projected * value.r;
            accumulator[coefficient * 3 + 1] += projected * value.g;
            accumulator[coefficient * 3 + 2] += projected * value.b;
        }
    }
    for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
    {
        outSh2[coefficient] = accumulator[coefficient];
    }

    // De-ringing: a band-limited fit of a high-contrast field (a small bright
    // emitter) rings below zero on the opposite side, which would subtract
    // light in the renderer. The bands are windowed, w_l = 1 / (1 + lambda *
    // l * (l + 1)), shrinking band 2 faster than band 1, with lambda solved
    // per channel by bisection so the cosine irradiance stays non-negative
    // along a fixed direction set (the 26 principal normals plus an
    // 11x11 octahedral grid). Windowing keeps more of the directional band
    // response than uniformly shrinking both bands: a small bright emitter
    // stays visible from the directions that face it while the ringing side
    // is suppressed, at the price of a slightly lifted floor elsewhere.
    // The 26 principal directions the non-negativity contract is checked
    // along (axis directions, cube diagonals, face diagonals), with each
    // direction's polynomial parts precomputed once.
    struct DeringTable
    {
        std::array<std::array<float, 9>, 26> polynomials{}; // f_lm per normal
        DeringTable()
        {
            const glm::vec3 normals[26] = {
                {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
                {0.577350f, 0.577350f, 0.577350f}, {-0.577350f, 0.577350f, 0.577350f},
                {0.577350f, -0.577350f, 0.577350f}, {-0.577350f, -0.577350f, 0.577350f},
                {0.577350f, 0.577350f, -0.577350f}, {-0.577350f, 0.577350f, -0.577350f},
                {0.577350f, -0.577350f, -0.577350f}, {-0.577350f, -0.577350f, -0.577350f},
                {0.707107f, 0.707107f, 0}, {-0.707107f, 0.707107f, 0}, {0.707107f, -0.707107f, 0},
                {-0.707107f, -0.707107f, 0}, {0.707107f, 0, 0.707107f}, {-0.707107f, 0, 0.707107f},
                {0.707107f, 0, -0.707107f}, {-0.707107f, 0, -0.707107f}, {0, 0.707107f, 0.707107f},
                {0, -0.707107f, 0.707107f}, {0, 0.707107f, -0.707107f}, {0, -0.707107f, -0.707107f}};
            for (std::size_t index = 0; index < 26; ++index)
            {
                const glm::vec3 &normal   = normals[index];
                const float     x = normal.x;
                const float     y = normal.y;
                const float     z = normal.z;
                polynomials[index] = {1.0f, y, z, x, x * y, y * z, 3.0f * z * z - 1.0f, x * z, x * x - y * y};
            }
        }
    };
    static const DeringTable deringTable;

    // Irradiance along one normal for window weights (w1, w2) on bands 1-2.
    // sh2 stores coefficient-major RGB triplets: coefficient c, channel ch at
    // sh2[c * 3 + ch], the GiProbe order.
    const auto windowedIrradiance = [&](std::size_t index, float w1, float w2, const float *sh2, std::uint32_t channel)
    {
        const std::array<float, 9> &poly = deringTable.polynomials[index];
        const float               c0 = sh2[channel];
        const float               c1 = sh2[3 + channel];
        const float               c2 = sh2[6 + channel];
        const float               c3 = sh2[9 + channel];
        const float               c4 = sh2[12 + channel];
        const float               c5 = sh2[15 + channel];
        const float               c6 = sh2[18 + channel];
        const float               c7 = sh2[21 + channel];
        const float               c8 = sh2[24 + channel];
        return kPi * c0
             + (2.0f * kPi / 3.0f) * w1 * (c1 * poly[1] + c2 * poly[2] + c3 * poly[3])
             + (kPi / 4.0f) * w2
                   * (c4 * poly[4] + c5 * poly[5] + c6 * poly[6] + c7 * poly[7] + c8 * poly[8]);
    };
    for (std::uint32_t channel = 0; channel < 3; ++channel)
    {
        // Un-windowed negative minimum along the table (early out).
        float worstNoWindow = 0.0f;
        for (std::size_t index = 0; index < 26; ++index)
        {
            worstNoWindow = std::min(worstNoWindow, windowedIrradiance(index, 1.0f, 1.0f, outSh2, channel));
        }
        if (worstNoWindow >= 0.0f)
        {
            continue;
        }
        // Bisect lambda so the windowed irradiance is non-negative along the
        // table; large lambda collapses toward the DC term, which is
        // non-negative whenever the radiance field carries net energy.
        float low = 0.0f;
        float high = 64.0f;
        for (std::uint32_t iteration = 0; iteration < 24; ++iteration)
        {
            const float middle = 0.5f * (low + high);
            const float w1     = 1.0f / (1.0f + middle * 2.0f);
            const float w2     = 1.0f / (1.0f + middle * 6.0f);
            float       worst  = 0.0f;
            for (std::size_t index = 0; index < 26; ++index)
            {
                worst = std::min(worst, windowedIrradiance(index, w1, w2, outSh2, channel));
            }
            if (worst < 0.0f)
            {
                low = middle;
            }
            else
            {
                high = middle;
            }
        }
        const float w1 = 1.0f / (1.0f + high * 2.0f);
        const float w2 = 1.0f / (1.0f + high * 6.0f);
        for (std::uint32_t band1 = 0; band1 < 3; ++band1)
        {
            outSh2[3 * (1 + band1) + channel] *= w1;
        }
        for (std::uint32_t band2 = 0; band2 < 5; ++band2)
        {
            outSh2[3 * (4 + band2) + channel] *= w2;
        }
    }
}

// ---- direct lighting --------------------------------------------------------------

// The scattered share a texel contributes to the caches: a transparent surface
// keeps its opacity share of the outgoing radiance (the reference estimator's
// rule - the (1 - opacity) share leaves through the transmission path the
// gather, probe, and reflection rays walk separately). Scaled once here, every
// consumer of the card (indirect gathers, probe rays, the reflection module's
// hit lighting through its own copy of the same rule) sees the correctly
// attenuated bounce without re-deriving it.
inline glm::vec3 scatteredShare(const GiSurface &surface, const glm::vec3 &radiance)
{
    return surface.opacity >= 1.0f ? radiance : radiance * surface.opacity;
}

void gatherDirectLightingUnit(GiPipelineState &state, const GiTexelWork &unit)
{
    const GiEnvironmentSnapshot &environment = state.environment;
    const glm::vec3              sunRadiance = environment.sunRadiance();
    const bool                   hasSun      = sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f;
    GiCard &                     card        = state.card(unit.cardIndex);
    const GiSurface &            surface     = state.cardSurface(unit.cardIndex);
    // Dark-environment fast path: with no sun and no direct lights the lit
    // radiance of every texel is exactly the emission term (the irradiance is
    // zero, so albedo / pi * E collapses to zero), and the world-space position
    // / normal the full evaluation transforms per texel would multiply zero.
    // Scenes lit purely through GI (the interior demo view) skip that work
    // every frame. The emission carries the same opacity share the lit terms
    // carry (see scatteredShare).
    if (!surface.unlit && !hasSun && !environment.hasDirectLights)
    {
        for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
        {
            card.directRadiance[texel] = card.depthTexels[texel] >= invalidCardDepth
                ? glm::vec3(0.0f)
                : scatteredShare(surface, surface.emissive);
        }
        state.directTexelsThisFrame.fetch_add(unit.texelEnd - unit.texelBegin, std::memory_order_relaxed);
        return;
    }
    // The unit walks flat layered texels; the (x, y) cell of the plane is
    // tracked incrementally so the loop never pays the flat-index divisions
    // (one integer division pair per unit instead of per texel).
    const std::uint32_t plane = static_cast<std::uint32_t>(card.planeTexels());
    std::uint32_t       xy    = unit.texelBegin % plane;
    std::uint32_t       x     = xy % card.texelWidth;
    std::uint32_t       y     = xy / card.texelWidth;
    for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
    {
        if (card.depthTexels[texel] >= invalidCardDepth)
        {
            card.directRadiance[texel] = glm::vec3(0.0f);
        }
        else
        {
            glm::vec3 radiance(0.0f);
            if (surface.unlit)
            {
                // Unlit surfaces emit their albedo; no lighting is applied.
                radiance = card.albedo[texel];
            }
            else
            {
                const glm::vec3 localPosition = cardTexelLocalPositionXY(card, x, y, texel);
                const glm::vec3 position = surface.localIsWorld
                    ? localPosition
                    : surface.linear * localPosition + surface.translation;
                // Captured card normals are unit length: an identity transform
                // passes them through untouched, anything else re-orthonormalizes
                // through the normal matrix (non-uniform scale skews them).
                const glm::vec3 normal = surface.localIsWorld
                    ? card.normal[texel]
                    : glm::normalize(surface.normalMatrix * card.normal[texel]);
                glm::vec3 irradiance(0.0f);
                const float nDotL = glm::dot(normal, environment.sunDirection);
                if (nDotL > 0.0f && hasSun)
                {
                    if (!state.settings.sunShadows)
                    {
                        irradiance += sunRadiance * nDotL;
                    }
                    else
                    {
                        const float transmittance = sunShadowTransmittance(state, position, normal);
                        if (transmittance > 0.0f)
                        {
                            irradiance += sunRadiance * nDotL * transmittance;
                        }
                    }
                }
                irradiance += directLightRadiance(state, position, normal);
                // Lambertian outgoing radiance: albedo / pi * E.
                radiance = card.albedo[texel] * (irradiance * (1.0f / kPi));
            }
            radiance += surface.emissive;
            card.directRadiance[texel] = scatteredShare(surface, radiance);
        }
        if (++x == card.texelWidth)
        {
            x = 0;
            ++y;
        }
        if (++xy == plane)
        {
            xy = 0;
            x  = 0;
            y  = 0;
        }
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
    // Incremental (x, y) cell like the direct pass: one division pair per
    // unit instead of per texel.
    const std::uint32_t plane = static_cast<std::uint32_t>(card.planeTexels());
    std::uint32_t       xy    = unit.texelBegin % plane;
    std::uint32_t       x     = xy % card.texelWidth;
    std::uint32_t       y     = xy / card.texelWidth;
    for (std::uint32_t texel = unit.texelBegin; texel < unit.texelEnd; ++texel)
    {
        glm::vec3 &out = scratch[texel - unit.texelBegin];
        out            = card.indirectRadiance[texel];
        if (surface.unlit)
        {
            // unlit surfaces do not receive indirect lighting
        }
        else if (card.depthTexels[texel] >= invalidCardDepth)
        {
            out                      = glm::vec3(0.0f);
            card.indirectSamples[texel] = 0;
        }
        else
        {
            const glm::vec3 localPosition = cardTexelLocalPositionXY(card, x, y, texel);
            const glm::vec3 position = surface.localIsWorld
                ? localPosition
                : surface.linear * localPosition + surface.translation;
            // Captured card normals are unit length (see the direct pass).
            const glm::vec3 normal = surface.localIsWorld
                ? card.normal[texel]
                : glm::normalize(surface.normalMatrix * card.normal[texel]);
            const glm::mat3 basis    = makeBasis(normal);
            // Surface bias: start slightly off the surface (Lumen biases its
            // texel traces the same way; trace.bias_m).
            const glm::vec3 origin = position + normal * state.settings.traceBias;
            std::uint64_t   seed   = hashSeed(state.frameIndex * 0x9E3779B1ull + (static_cast<std::uint64_t>(unit.cardIndex) << 32)
                                           + texel);
            glm::vec3 gather(0.0f);
            for (std::uint32_t ray = 0; ray < rayCount; ++ray)
            {
                // Stratified over the cosine-weighted hemisphere: ray r owns the
                // r-th slice of the radial axis.
                const float u = (static_cast<float>(ray) + randomFloat(seed)) / static_cast<float>(rayCount);
                const float v = randomFloat(seed);
                // The Duff basis is orthonormal and the Malley sample is unit
                // by construction, so the product is already unit length - no
                // normalize (a sqrt plus three divisions per ray) needed.
                const glm::vec3 direction = basis * cosineHemisphere(u, v);
                ++rays;
                // Transmission-aware gather: a transparent surface keeps its
                // opacity share of the response and passes the rest through,
                // tinted by its albedo, bent once by Snell's law with its
                // index ratio (the entering interface; exits straighten the
                // path back out). The split is deterministic, so the few-ray
                // gather stays low-variance.
                glm::vec3 weight(1.0f);
                glm::vec3 rayOrigin    = origin;
                glm::vec3 rayDirection = direction;
                for (std::uint32_t segment = 0; segment <= state.settings.maxTransmissionSegments; ++segment)
                {
                    const GiShadingHit hit = traceSceneShading(
                        state, GiRay{rayOrigin, rayDirection, state.settings.maxTraceDistance});
                    if (!hit.hit)
                    {
                        gather += weight * state.environment.skyRadiance(rayDirection);
                        break;
                    }
                    const GiSurface &surface = state.surfaces[hit.surfaceIndex];
                    const float      opacity = glm::clamp(surface.opacity, 0.0f, 1.0f);
                    if (opacity >= 1.0f)
                    {
                        glm::vec3 hitRadiance(0.0f);
                        radianceAtHit(state, hit, true, hitRadiance);
                        gather += weight * hitRadiance;
                        break;
                    }
                    glm::vec3 hitRadiance(0.0f);
                    radianceAtHit(state, hit, true, hitRadiance);
                    gather += weight * opacity * hitRadiance;
                    if (opacity <= 0.0f || surface.causticTransmitter
                        || segment == state.settings.maxTransmissionSegments)
                    {
                        break; // caustics pass owns the transmitted share
                    }
                    // The albedo absorption applies once per medium traversal,
                    // at the entering interface; the matching exit passes
                    // untinted.
                    weight *= hit.backface ? glm::vec3(1.0f) : surface.albedo * (1.0f - opacity);
                    // The gather works in world space; the shading hit carries
                    // local-space position/normal, so transform them once for
                    // the continuation.
                    const glm::vec3 worldPosition = surface.localIsWorld
                        ? hit.localPosition
                        : surface.linear * hit.localPosition + surface.translation;
                    const glm::vec3 worldNormal
                        = glm::normalize(surface.normalMatrix * hit.localNormal);
                    // The interface normal faces the ray: outward normals stay
                    // outward on a medium exit (a backface hit), and the
                    // refraction needs the normal on the ray's side to bend
                    // across the interface instead of mirroring back into the
                    // medium.
                    const glm::vec3 interfaceNormal = hit.backface ? -worldNormal : worldNormal;
                    const float     ior = glm::max(surface.indexOfRefraction, 1.0f);
                    glm::vec3 bent  = glm::refract(rayDirection, interfaceNormal, hit.backface ? ior : 1.0f / ior);
                    if (glm::dot(bent, bent) < 1.0e-8f)
                    {
                        bent = glm::reflect(rayDirection, interfaceNormal); // total internal reflection
                    }
                    rayOrigin    = worldPosition + bent * state.settings.traceBias;
                    rayDirection = bent;
                }
            }
            // Cosine-weighted sampling: E = pi * mean(L_i), and the Lambertian
            // radiance is albedo / pi * E = albedo * mean(L_i). The scattered
            // share carries the surface's opacity (see scatteredShare): the
            // transmitted share of a transparent texel lives in the ray chains
            // its transmission segments walk, never in its own bounce term.
            gather = scatteredShare(surface, card.albedo[texel] * (gather / static_cast<float>(rayCount)));
            // Exponential history with a ramp-up: the first update takes the
            // fresh sample whole, then the weight settles at (1 - history).
            const std::uint32_t seen  = std::min<std::uint32_t>(card.indirectSamples[texel] + 1u, 255u);
            const float         alpha = std::max(1.0f / static_cast<float>(seen), 1.0f - history);
            out                       = glm::mix(card.indirectRadiance[texel], gather, alpha);
            card.indirectSamples[texel] = static_cast<std::uint8_t>(seen);
        }
        if (++x == card.texelWidth)
        {
            x = 0;
            ++y;
        }
        if (++xy == plane)
        {
            xy = 0;
            x  = 0;
            y  = 0;
        }
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

    // Relocation constant (the DDGI near-surface rule): the margin a probe
    // keeps from any face it was pushed off, as a fraction of the spacing
    // (probes.relocation_margin).
    const float nearDistance
        = std::clamp(state.settings.probeRelocationMargin, 0.005f, 0.25f) * level.spacing;
    // The trace origin is the relocated position; the exported lattice
    // itself stays regular. The surface bias (trace.bias_m) is applied along
    // each ray's own direction: a probe sitting exactly on a surface plane
    // would otherwise self-intersect that plane at t ~ 0, and its
    // through-the-slab continuation would flag the hit a backface, marking
    // half the sphere enclosed (the "probe on a wall" known issue).
    const glm::vec3 traceOrigin = probe.position + probe.offset;
    const float     traceBias   = state.settings.traceBias;

    // Per-worker scratch for the backface texel indices (updateProbe runs on
    // scheduler workers): the thread_local vector is reserved once per worker
    // and reused every update instead of allocating per probe.
    thread_local std::vector<std::uint32_t> backfaceTexels;
    backfaceTexels.clear();
    glm::vec3     frontSum(0.0f);
    std::uint32_t frontCount = 0;
    std::uint32_t traced     = 0;
    // Hit statistics the relocation rules below read: the closest backface
    // (a ray that passed through a slab and struck its far side), the closest
    // front-facing hit (the nearest surface on the open side), and the mean
    // front-ray direction (the open-side evidence that validates a boundary
    // push).
    constexpr float kNoHit  = std::numeric_limits<float>::max();
    float     closestBackfaceT          = kNoHit;
    glm::vec3 closestBackfaceDirection{0.0f, 0.0f, 1.0f};
    bool      hasBackface               = false;
    float     closestFrontT             = kNoHit;
    glm::vec3 closestFrontDirection{0.0f, 0.0f, 1.0f};
    bool      hasFront                  = false;
    glm::vec3 frontDirectionSum(0.0f);
    for (std::size_t texel = phase; texel < directionCount; texel += stride)
    {
        const glm::vec3 &direction = state.probeDirections[texel];
        ++traced;
        glm::vec3 value(0.0f);
        bool      terminatedByOpaqueBackface = false;
        // Transmission-aware probe ray: transparent surfaces keep their
        // opacity share of the response and pass the rest through, tinted by
        // their albedo, bent once by Snell's law at the entering interface
        // (exits straighten the path back out; total internal reflection
        // mirrors). Transparent surfaces stay out of the relocation and
        // validity accounting entirely - a glass pane must not mark the
        // probe enclosed or push it off its plane - while the FIRST opaque
        // hit along the chain provides the backface/front evidence, measured
        // along the traveled path so a pane in front shifts the distances.
        {
            glm::vec3 weight(1.0f);
            glm::vec3 rayOrigin    = traceOrigin + direction * traceBias;
            glm::vec3 rayDirection = direction;
            float     rayTMax      = state.settings.maxTraceDistance - traceBias;
            float     traveled     = 0.0f;
            for (std::uint32_t segment = 0; segment <= state.settings.maxTransmissionSegments; ++segment)
            {
                const GiShadingHit hit = traceSceneShading(state, GiRay{rayOrigin, rayDirection, rayTMax});
                if (!hit.hit)
                {
                    value += weight * state.environment.skyRadiance(rayDirection);
                    break;
                }
                const GiSurface &surface = state.surfaces[hit.surfaceIndex];
                const float      opacity = glm::clamp(surface.opacity, 0.0f, 1.0f);
                if (opacity >= 1.0f)
                {
                    const float hitDistance = traveled + hit.t;
                    if (hit.backface)
                    {
                        terminatedByOpaqueBackface = true;
                        if (hitDistance < closestBackfaceT)
                        {
                            closestBackfaceT          = hitDistance;
                            closestBackfaceDirection = rayDirection;
                            hasBackface               = true;
                        }
                    }
                    else
                    {
                        frontDirectionSum += direction;
                        if (hitDistance < closestFrontT)
                        {
                            closestFrontT          = hitDistance;
                            closestFrontDirection = direction;
                            hasFront               = true;
                        }
                        glm::vec3 hitRadiance(0.0f);
                        radianceAtHit(state, hit, true, hitRadiance);
                        value += weight * hitRadiance;
                    }
                    break;
                }
                glm::vec3 hitRadiance(0.0f);
                radianceAtHit(state, hit, true, hitRadiance);
                value += weight * opacity * hitRadiance;
                if (opacity <= 0.0f || surface.causticTransmitter
                    || segment == state.settings.maxTransmissionSegments)
                {
                    break; // caustics pass owns the transmitted share
                }
                // The albedo absorption applies once per medium traversal, at
                // the entering interface; the matching exit passes untinted.
                weight *= hit.backface ? glm::vec3(1.0f) : surface.albedo * (1.0f - opacity);
                // The probe loop works in world space; the shading hit
                // carries local-space position/normal, so transform them once
                // for the continuation.
                const glm::vec3 worldPosition = surface.localIsWorld
                    ? hit.localPosition
                    : surface.linear * hit.localPosition + surface.translation;
                const glm::vec3 worldNormal
                    = glm::normalize(surface.normalMatrix * hit.localNormal);
                // The interface normal faces the ray (see the gather's
                // transmission walk): outward normals stay outward on a medium
                // exit, and the refraction needs the normal on the ray's side.
                const glm::vec3 interfaceNormal = hit.backface ? -worldNormal : worldNormal;
                const float     ior = glm::max(surface.indexOfRefraction, 1.0f);
                glm::vec3 bent  = glm::refract(rayDirection, interfaceNormal, hit.backface ? ior : 1.0f / ior);
                if (glm::dot(bent, bent) < 1.0e-8f)
                {
                    bent = glm::reflect(rayDirection, interfaceNormal);
                }
                traveled    += hit.t + state.settings.traceBias;
                rayOrigin    = worldPosition + bent * state.settings.traceBias;
                rayDirection = bent;
                rayTMax      = state.settings.maxTraceDistance;
            }
        }
        if (terminatedByOpaqueBackface)
        {
            backfaceTexels.push_back(static_cast<std::uint32_t>(texel));
            continue;
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
    projectRadianceToSh2(state.probeDirectionBasis.data(), probe.radiance.data(), state.probeDirectionWeights.data(),
        directionCount, probe.sh2.data());
    probe.fresh      = false;
    probe.lastUpdate = state.frameIndex;
    ++probe.phase;

    // ---- probe relocation (the DDGI rules) ------------------------------------
    // The rules move the trace origin off geometry so the next update's rays
    // see front faces and the validity recovers; the exported lattice itself
    // stays regular. A probe seeing backfaces sits on or inside a slab: the
    // closest backface ray traveled THROUGH that slab, so the open side is
    // the one it came from, and the probe steps away from that backface hit
    // by exactly enough to clear the slab (its hit distance) plus the
    // near-surface margin - but only when the front-face rays agree that the
    // opposite direction is open (their mean direction must lean toward it).
    // Without that agreement the probe sits just inside a slab where both
    // faces show up as backfaces and the nearest one is the face toward the
    // room, so it travels THROUGH the closest backface instead, out of the
    // solid the short way (the DDGI production rule; also the only rule for a
    // probe that sees no front face at all, deep inside a thick solid).
    // Probes in clear space relax onto the near-surface equilibrium: a probe
    // pushed off a plane settles at exactly the near-surface distance, and a
    // grazing probe is eased out to it, so a relocated probe never drifts
    // back across the surface it was moved off.
    // Per-axis wander limit: half the level spacing on every axis (the DDGI
    // grid-indexing bound; a probe never crosses a neighboring cell).
    const glm::vec3 offsetLimit(0.5f * level.spacing);
    const auto      clampOffsetAxis = [](glm::vec3 value, const glm::vec3 &limit)
    {
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            value[axis] = glm::clamp(value[axis], -limit[axis], limit[axis]);
        }
        return value;
    };
    const float frontMeanLength = glm::length(frontDirectionSum);
    const glm::vec3 frontMeanDirection
        = frontMeanLength > 1.0e-6f ? frontDirectionSum / frontMeanLength : glm::vec3(0.0f, 0.0f, 1.0f);
    constexpr float kOpenSideAgreement = 0.25f;
    glm::vec3 relocated = probe.offset;
    // Only the geometry-clearing moves invalidate the traced radiance; the
    // clear-space easing below never does.
    bool movedOffGeometry = false;
    if (hasBackface && !hasFront)
    {
        // Fully buried: out of the solid through the nearest face.
        relocated = probe.offset + closestBackfaceDirection * (closestBackfaceT + nearDistance);
        movedOffGeometry = true;
    }
    else if (hasBackface && backfaceFraction > 0.25f)
    {
        if (glm::dot(-closestBackfaceDirection, frontMeanDirection) > kOpenSideAgreement)
        {
            // On a surface boundary: away from the backface hit, clearing its
            // slab, toward the front-face evidence.
            relocated = probe.offset - closestBackfaceDirection * (closestBackfaceT + nearDistance);
        }
        else
        {
            // Ambiguous (both faces of a thin slab read as backfaces):
            // through the closest backface, out of the slab the short way.
            relocated = probe.offset + closestBackfaceDirection * (closestBackfaceT + nearDistance);
        }
        movedOffGeometry = true;
    }
    else
    {
        // Clear or grazing space: ease the offset toward the near-surface
        // equilibrium (the lattice position when nothing is near).
        const glm::vec3 relaxTarget = hasFront && closestFrontT < nearDistance
              ? -closestFrontDirection * (nearDistance - closestFrontT)
              : glm::vec3(0.0f);
        relocated = glm::mix(probe.offset, relaxTarget, 0.2f);
    }
    relocated = clampOffsetAxis(relocated, offsetLimit);
    // A geometry-clearing move invalidated this update's radiance (it was
    // traced from the old side of a slab): mark the probe fresh so the next
    // update re-traces the whole direction map with zero hysteresis, and the
    // planning pass serves it first. The clamp may have cancelled the move
    // entirely; only a real displacement counts.
    if (movedOffGeometry && glm::length(relocated - probe.offset) > 0.25f * nearDistance)
    {
        probe.fresh = true;
    }
    probe.offset = relocated;
}

namespace
{

struct LatticeCorners
{
    std::uint32_t slots[8];
    float         weights[8];
};

// Trilinear corner slots / weights of a world position inside one level.
// The lattice arithmetic runs in double precision: the level's origin cell
// grows with the distance the camera has traveled, and subtracting it in
// float32 would cancel the fractional position away (the clipmap tracks
// cells as exact integers for exactly this reason - the float remainder is
// the only place the cancellation can appear).
bool latticeCorners(const GiProbeLevel &level, const glm::vec3 &position, LatticeCorners &out)
{
    const std::int32_t dim = static_cast<std::int32_t>(level.dimension());
    if (dim < 2 || level.probes.empty())
    {
        return false;
    }
    const glm::dvec3 logical(double(position.x) / double(level.spacing) - double(level.originCell.x),
        double(position.y) / double(level.spacing) - double(level.originCell.y),
        double(position.z) / double(level.spacing) - double(level.originCell.z));
    const double limit   = static_cast<double>(dim - 1);
    if (logical.x < 0.0 || logical.y < 0.0 || logical.z < 0.0 || logical.x > limit || logical.y > limit
        || logical.z > limit)
    {
        return false;
    }
    const glm::dvec3 base = glm::min(glm::floor(logical), glm::dvec3(double(dim - 2)));
    const glm::vec3 fraction(logical - base);
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
        // Each valid corner evaluates against the shared normal and
        // accumulates into one vec3 accumulator (the register-friendly shape;
        // see the renderer twin in gi_probe_sampling.hpp); the tangent-plane
        // factor comes from the shared exported-grid convention
        // (radiance_cache::probeTangentPlaneFactor in capabilities.hpp).
        const float inverseSpacing = 1.0f / std::max(level->spacing, 1.0e-5f);
        glm::vec3 sum(0.0f);
        float     weightSum = 0.0f;
        for (std::uint32_t corner = 0; corner < 8; ++corner)
        {
            const GiProbeLevel::Probe &probe = level->probes[corners.slots[corner]];
            float weight = corners.weights[corner] * probe.validity;
            if (weight <= 0.0f)
            {
                continue;
            }
            const glm::vec3 toProbe = probe.position - position;
            weight *= probeTangentPlaneFactor(&normal.x, &toProbe.x, inverseSpacing);
            if (weight <= 1.0e-8f)
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

} // namespace radiance_cache
} // namespace engine
