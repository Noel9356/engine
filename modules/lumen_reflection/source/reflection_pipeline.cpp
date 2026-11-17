// lumen_reflection pipeline: the frame GBuffer, the shaded screen color, the
// stochastic GGX ray pass, and the hybrid tracing tiers.
//
// Stage order per evaluate() (Lumen's reflection solve):
//   1. rasterize the GBuffer at field resolution (depth, position, normal,
//      albedo, roughness/metallic, emission);
//   2. shade the whole GBuffer once into the screen color buffer (Lumen
//      Scene lighting - what screen traces sample);
//   3. per texel inside the traced roughness bands, importance-sample the
//      visible GGX lobe and resolve each ray: screen trace first (against
//      the shaded GBuffer), then a scene trace with hit lighting (sun +
//      direct lights + the lumen_gi probe-grid diffuse GI + emission), then
//      the skylight; the rough band resamples the lumen_gi probe radiance
//      instead of tracing rays;
//   4. the denoise chain (reflection_denoise.cpp) turns the stochastic field
//      into the published one.

#include "lumen_reflection_internal.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{
namespace lumen_reflection
{

namespace
{

constexpr float kMinimumRoughness = 0.045f; // matches the renderer's clamp floor
constexpr float kDielectricF0 = 0.04f;

// ---- GBuffer raster ------------------------------------------------------------

struct ClipVertex
{
    glm::vec4 clip{0.0f};
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct ScreenVertex
{
    glm::vec2 pixel{0.0f};
    float     inverseW = 0.0f;
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct ScreenTriangle
{
    ScreenVertex  vertex[3];
    std::uint32_t surface  = 0;
    std::uint32_t triangle = 0;
};

constexpr float nearW = 0.05f;
constexpr std::uint32_t bandRows = 16;

bool surfaceOutsideFrustum(const glm::mat4 &viewProjection, const RfSurface &surface)
{
    std::uint32_t outcodeAnd = 0x3fu;
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const glm::vec3 world((corner & 1u) ? surface.worldMax.x : surface.worldMin.x,
            (corner & 2u) ? surface.worldMax.y : surface.worldMin.y, (corner & 4u) ? surface.worldMax.z : surface.worldMin.z);
        const glm::vec4 clip = viewProjection * glm::vec4(world, 1.0f);
        std::uint32_t   code = 0;
        code |= clip.x > clip.w ? 1u : 0u;
        code |= clip.x < -clip.w ? 2u : 0u;
        code |= clip.y > clip.w ? 4u : 0u;
        code |= clip.y < -clip.w ? 8u : 0u;
        code |= clip.w < nearW ? 16u : 0u;
        outcodeAnd &= code;
    }
    return (outcodeAnd & 0x1fu) != 0u;
}

void setupSurfaceTriangles(const RfPipelineState &state, std::uint32_t surfaceIndex, std::uint32_t width,
    std::uint32_t height, std::vector<ScreenTriangle> &out)
{
    const RfSurface &surface = state.surfaces[surfaceIndex];
    if (surface.excluded || surfaceOutsideFrustum(state.viewProjection, surface))
    {
        return;
    }
    const RfSurfaceGeometry &geometry = surface.geometry;
    std::vector<glm::vec3> world(geometry.positions.size());
    std::vector<glm::vec4> clip(geometry.positions.size());
    for (std::size_t vertex = 0; vertex < geometry.positions.size(); ++vertex)
    {
        world[vertex] = glm::vec3(surface.transform * glm::vec4(geometry.positions[vertex], 1.0f));
        clip[vertex]  = state.viewProjection * glm::vec4(world[vertex], 1.0f);
    }
    const std::uint32_t triangleCount = static_cast<std::uint32_t>(geometry.indices.size() / 3);
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        ClipVertex polygon[4];
        std::uint32_t count = 0;
        ClipVertex input[3];
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            const std::uint32_t index = geometry.indices[triangle * 3 + corner];
            input[corner].clip  = clip[index];
            input[corner].world = world[index];
            input[corner].bary  = glm::vec3(corner == 0 ? 1.0f : 0.0f, corner == 1 ? 1.0f : 0.0f, corner == 2 ? 1.0f : 0.0f);
        }
        // Sutherland-Hodgman against the near plane w = nearW.
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            const ClipVertex &current = input[corner];
            const ClipVertex &next    = input[(corner + 1) % 3];
            const float       dCurrent = current.clip.w - nearW;
            const float       dNext    = next.clip.w - nearW;
            if (dCurrent >= 0.0f)
            {
                polygon[count++] = current;
            }
            if ((dCurrent >= 0.0f) != (dNext >= 0.0f))
            {
                const float t = dCurrent / (dCurrent - dNext);
                ClipVertex  edge;
                edge.clip  = glm::mix(current.clip, next.clip, t);
                edge.world = glm::mix(current.world, next.world, t);
                edge.bary  = glm::mix(current.bary, next.bary, t);
                polygon[count++] = edge;
            }
        }
        if (count < 3)
        {
            continue;
        }
        ScreenVertex projected[4];
        for (std::uint32_t corner = 0; corner < count; ++corner)
        {
            const glm::vec4 &c = polygon[corner].clip;
            projected[corner].pixel    = glm::vec2((c.x / c.w * 0.5f + 0.5f) * static_cast<float>(width),
                (0.5f - c.y / c.w * 0.5f) * static_cast<float>(height));
            projected[corner].inverseW = 1.0f / c.w;
            projected[corner].world    = polygon[corner].world;
            projected[corner].bary     = polygon[corner].bary;
        }
        for (std::uint32_t fan = 1; fan + 1 < count; ++fan)
        {
            ScreenTriangle screen;
            screen.vertex[0] = projected[0];
            screen.vertex[1] = projected[fan];
            screen.vertex[2] = projected[fan + 1];
            screen.surface   = surfaceIndex;
            screen.triangle  = triangle;
            out.push_back(screen);
        }
    }
}

void rasterizeBand(RfPipelineState &state, const std::vector<ScreenTriangle> &triangles,
    const std::vector<std::uint32_t> &binned, std::uint32_t rowBegin, std::uint32_t rowEnd)
{
    const std::uint32_t width = state.fieldWidth;
    for (const std::uint32_t triangleIndex : binned)
    {
        const ScreenTriangle &screen = triangles[triangleIndex];
        const glm::vec2 a = screen.vertex[0].pixel;
        const glm::vec2 b = screen.vertex[1].pixel;
        const glm::vec2 c = screen.vertex[2].pixel;
        const glm::vec2 boundsMin = glm::min(a, glm::min(b, c));
        const glm::vec2 boundsMax = glm::max(a, glm::max(b, c));
        const std::int32_t xBegin = std::max(static_cast<std::int32_t>(std::floor(boundsMin.x)), 0);
        const std::int32_t xEnd   = std::min(static_cast<std::int32_t>(std::ceil(boundsMax.x)) + 1,
            static_cast<std::int32_t>(width));
        const std::int32_t yBegin = std::max(static_cast<std::int32_t>(std::floor(boundsMin.y)), static_cast<std::int32_t>(rowBegin));
        const std::int32_t yEnd   = std::min(static_cast<std::int32_t>(std::ceil(boundsMax.y)) + 1,
            static_cast<std::int32_t>(rowEnd));
        if (xBegin >= xEnd || yBegin >= yEnd)
        {
            continue;
        }
        const glm::vec2 edge1 = b - a;
        const glm::vec2 edge2 = c - a;
        const float     denominator = edge1.x * edge2.y - edge2.x * edge1.y;
        if (std::fabs(denominator) < 1.0e-12f)
        {
            continue;
        }
        const RfSurface         &surface  = state.surfaces[screen.surface];
        const RfSurfaceGeometry &geometry = surface.geometry;
        const std::uint32_t      i0 = geometry.indices[screen.triangle * 3 + 0];
        const std::uint32_t      i1 = geometry.indices[screen.triangle * 3 + 1];
        const std::uint32_t      i2 = geometry.indices[screen.triangle * 3 + 2];
        for (std::int32_t y = yBegin; y < yEnd; ++y)
        {
            for (std::int32_t x = xBegin; x < xEnd; ++x)
            {
                const glm::vec2 toPixel(static_cast<float>(x) + 0.5f - a.x, static_cast<float>(y) + 0.5f - a.y);
                const float     u = (toPixel.x * edge2.y - edge2.x * toPixel.y) / denominator;
                const float     v = (edge1.x * toPixel.y - toPixel.x * edge1.y) / denominator;
                if (u < 0.0f || v < 0.0f || u + v > 1.0f)
                {
                    continue;
                }
                // Perspective-correct weights.
                const float w0 = (1.0f - u - v) * screen.vertex[0].inverseW;
                const float w1 = u * screen.vertex[1].inverseW;
                const float w2 = v * screen.vertex[2].inverseW;
                const float sum = w0 + w1 + w2;
                if (sum <= 0.0f)
                {
                    continue;
                }
                const float mu0 = w0 / sum;
                const float mu1 = w1 / sum;
                const float mu2 = w2 / sum;
                const glm::vec3 worldPosition = screen.vertex[0].world * mu0 + screen.vertex[1].world * mu1
                                              + screen.vertex[2].world * mu2;
                const float       distance   = glm::length(worldPosition - state.cameraPosition);
                const std::size_t pixelIndex = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
                if (distance >= state.gbufferDepth[pixelIndex])
                {
                    continue;
                }
                const glm::vec3 bary = screen.vertex[0].bary * mu0 + screen.vertex[1].bary * mu1
                                     + screen.vertex[2].bary * mu2;
                glm::vec3 localNormal = geometry.normals[i0] * bary.x + geometry.normals[i1] * bary.y
                                      + geometry.normals[i2] * bary.z;
                const float normalLength = glm::length(localNormal);
                localNormal = normalLength > 1.0e-8f ? localNormal / normalLength : glm::vec3(0.0f, 1.0f, 0.0f);
                state.gbufferDepth[pixelIndex]         = distance;
                state.gbufferPosition[pixelIndex]       = worldPosition;
                state.gbufferSurface[pixelIndex]        = screen.surface;
                state.gbufferNormal[pixelIndex]         = glm::normalize(surface.transform * glm::vec4(localNormal, 0.0f));
                state.gbufferAlbedo[pixelIndex] = surface.albedo
                    * (geometry.colors[i0] * bary.x + geometry.colors[i1] * bary.y + geometry.colors[i2] * bary.z);
                state.gbufferRoughness[pixelIndex]      = surface.roughness;
                state.gbufferMetallic[pixelIndex]       = surface.metallic;
                state.gbufferEmissive[pixelIndex]       = surface.emissive;
                state.gbufferUnlit[pixelIndex]          = static_cast<std::uint8_t>(surface.unlit ? 1u : 0u);
            }
        }
    }
}

} // namespace

void rfRasterizeGBuffer(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::uint32_t scale = std::max(state.settings.screenScale, 1u);
    const std::uint32_t width  = std::max(state.viewportWidth / scale, 1u);
    const std::uint32_t height = std::max(state.viewportHeight / scale, 1u);
    state.fieldWidth  = width;
    state.fieldHeight = height;
    const std::size_t pixelCount = static_cast<std::size_t>(width) * height;
    state.gbufferDepth.assign(pixelCount, rfBackgroundDepth);
    state.gbufferPosition.assign(pixelCount, glm::vec3(0.0f));
    state.gbufferNormal.assign(pixelCount, glm::vec3(0.0f, 1.0f, 0.0f));
    state.gbufferAlbedo.assign(pixelCount, glm::vec3(0.0f));
    state.gbufferRoughness.assign(pixelCount, 1.0f);
    state.gbufferMetallic.assign(pixelCount, 0.0f);
    state.gbufferEmissive.assign(pixelCount, glm::vec3(0.0f));
    state.gbufferUnlit.assign(pixelCount, 0u);
    state.gbufferSurface.assign(pixelCount, noSurface);

    std::vector<std::vector<ScreenTriangle>> perSurface(state.surfaces.size());
    rfParallelRange(scheduler, state.surfaces.size(), 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t surface = begin; surface < end; ++surface)
            {
                setupSurfaceTriangles(state, static_cast<std::uint32_t>(surface), width, height, perSurface[surface]);
            }
        });
    std::vector<ScreenTriangle> triangles;
    for (const std::vector<ScreenTriangle> &list : perSurface)
    {
        triangles.insert(triangles.end(), list.begin(), list.end());
    }
    const std::uint32_t bandCount = (height + bandRows - 1) / bandRows;
    std::vector<std::vector<std::uint32_t>> bins(bandCount);
    for (std::uint32_t index = 0; index < triangles.size(); ++index)
    {
        const ScreenTriangle &screen = triangles[index];
        const float yMin = std::min({screen.vertex[0].pixel.y, screen.vertex[1].pixel.y, screen.vertex[2].pixel.y});
        const float yMax = std::max({screen.vertex[0].pixel.y, screen.vertex[1].pixel.y, screen.vertex[2].pixel.y});
        if (yMax < 0.0f || yMin > static_cast<float>(height))
        {
            continue;
        }
        const std::int32_t firstBand = std::max(static_cast<std::int32_t>(std::floor(yMin)) / static_cast<std::int32_t>(bandRows), 0);
        const std::int32_t lastBand  = std::min(static_cast<std::int32_t>(std::ceil(yMax)) / static_cast<std::int32_t>(bandRows),
            static_cast<std::int32_t>(bandCount) - 1);
        for (std::int32_t band = firstBand; band <= lastBand; ++band)
        {
            bins[band].push_back(index);
        }
    }
    rfParallelRange(scheduler, bandCount, 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t band = begin; band < end; ++band)
            {
                const std::uint32_t rowBegin = static_cast<std::uint32_t>(band) * bandRows;
                rasterizeBand(state, triangles, bins[band], rowBegin, std::min(rowBegin + bandRows, height));
            }
        });
}

// ---- hit lighting -------------------------------------------------------------------

// Radiance a reflection ray sees when it lands on a surface (Lumen's hit
// lighting): direct sun + direct lights (same formulas as the renderer's
// shading), the lumen_gi probe-grid diffuse GI, emission, and the hemispheric
// ambient fallback while the probe grid carries no data. View-dependent
// specular at the hit is deliberately not evaluated - Lumen's cached scene
// lighting carries diffuse lighting and emission only.
glm::vec3 rfShadeHitPoint(const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    const glm::vec3 &albedo, const glm::vec3 &emissive)
{
    glm::vec3 result = emissive;
    const glm::vec3 diffuse = albedo * (1.0f / kPi);

    const RfEnvironmentSnapshot &environment = state.environment;
    // Sun (shadow ray with the grazing-angle bias lumen_gi uses).
    if (environment.hasSun && environment.sunEnabled)
    {
        const glm::vec3 sunDirection = environment.sunDirection;
        const float nDotL = glm::dot(normal, sunDirection);
        if (nDotL > 0.0f)
        {
            const float bias = state.settings.traceBias
                + state.settings.traceBias * std::fabs(glm::dot(normal, sunDirection));
            const RfRay shadowRay{position + normal * bias, sunDirection, state.settings.maxTraceDistance};
            const RfHit blocked = rfTraceScene(state, shadowRay);
            if (!blocked.hit)
            {
                result += diffuse * environment.sunRadiance() * nDotL;
            }
        }
    }

    // Direct lights (point/spot/directional of the direct_lighting module):
    // the renderer's attenuation formulas.
    const glm::vec3 origin = position + normal * state.settings.traceBias;
    for (const RfEnvironmentSnapshot::PreparedDirectLight &light : environment.preparedLights)
    {
        glm::vec3  localLightDirection;
        float      attenuation = 1.0f;
        if (light.type == direct_lighting::DirectLightType::Directional)
        {
            localLightDirection = -light.direction;
        }
        else
        {
            const glm::vec3 toLight           = light.position - position;
            const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
            const float     rangeRatioSquared = distanceSquared / (light.range * light.range);
            if (rangeRatioSquared >= 1.0f)
            {
                continue;
            }
            const float inverseDistance = 1.0f / std::sqrt(distanceSquared);
            localLightDirection         = toLight * inverseDistance;
            const float rangeFade       = 1.0f - rangeRatioSquared * rangeRatioSquared;
            attenuation                 = rangeFade * rangeFade / distanceSquared;
            if (light.type == direct_lighting::DirectLightType::Spot)
            {
                const float coneCosine = glm::dot(-localLightDirection, light.direction);
                const float coneT = std::clamp((coneCosine - light.outerCosine) / light.coneDivisor, 0.0f, 1.0f);
                attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
            }
        }
        const float localNDotL = glm::dot(normal, localLightDirection);
        if (localNDotL <= 0.0f || attenuation <= 0.0f)
        {
            continue;
        }
        // Point-light shadows on reflection hits are off by default (the
        // module carries no dedicated pointLightShadows setting; the sun
        // shadow above carries the occlusion the eye reads).
        result += diffuse * (light.color * light.intensity) * attenuation * localNDotL;
    }

    // Diffuse GI from the probe grid; the hemispheric ambient fallback while
    // no probe around the position holds data.
    glm::vec3 indirect(0.0f);
    if (rfProbeIrradiance(state, position, normal, indirect))
    {
        result += diffuse * indirect;
    }
    else
    {
        const glm::vec3 ambient = glm::mix(environment.ambientGround, environment.ambientSky, normal.y * 0.5f + 0.5f)
            * environment.ambientIntensity;
        result += diffuse * ambient;
    }
    return result;
}

// Radiance of a scene-traced hit (anti-leak: backface hits return nothing,
// lumen_gi's rule). Unlit hits emit their albedo.
glm::vec3 rfRadianceAtHit(const RfPipelineState &state, const RfHit &hit)
{
    if (!hit.hit || hit.backface)
    {
        return glm::vec3(0.0f);
    }
    const RfSurface &surface = state.surfaces[hit.surfaceIndex];
    if (surface.unlit)
    {
        return surface.albedo;
    }
    return rfShadeHitPoint(state, hit.position, hit.normal, hit.albedo, surface.emissive);
}

// ---- screen color + screen traces ----------------------------------------------------

// Shaded color of every GBuffer texel (Lumen Scene lighting): what a screen
// trace samples when it hits. Background texels read as black; rays that
// march into background texels resolve the skylight at the trace site
// instead (resolveSkyFallback).
void rfShadeScreenColor(const RfPipelineState &state, TaskScheduler *scheduler, std::vector<glm::vec3> &outColor)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    outColor.assign(pixelCount, glm::vec3(0.0f));
    rfParallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    if (rfIsBackground(state.gbufferDepth[index]))
                    {
                        continue;
                    }
                    if (state.gbufferUnlit[index] != 0u)
                    {
                        outColor[index] = state.gbufferAlbedo[index];
                        continue;
                    }
                    outColor[index] = rfShadeHitPoint(state,
                        state.gbufferPosition[index],
                        state.gbufferNormal[index],
                        state.gbufferAlbedo[index],
                        state.gbufferEmissive[index]);
                }
            }
        });
}

// Screen-space march of one reflection ray (Lumen's screen traces): steps
// through the field's depth buffer in equal world-space increments (the ray
// stays linear in world distance), tests each visited texel's stored camera
// distance against the ray's, and accepts the hit when the ray passes behind
// the stored surface by at most `thickness` (thin-occluder heuristic). Rays
// leaving the screen or exceeding the trace distance miss, handing the rest
// to the scene trace. Background texels never intersect (the sky sits behind
// everything).
struct ScreenTraceResult
{
    bool          hit = false;
    std::uint32_t x = 0;
    std::uint32_t y = 0;
};

ScreenTraceResult screenTrace(const RfPipelineState &state, const glm::vec3 &origin, const glm::vec3 &direction,
    const std::vector<glm::vec3> &screenColor, float maxDistance, glm::vec3 &outRadiance)
{
    ScreenTraceResult result;
    outRadiance = glm::vec3(0.0f);
    // Clip-space linearization: clip(t) = clipOrigin + clipDir * t lets the
    // march project each step exactly without per-step matrix products.
    const glm::vec4 clipOrigin = state.viewProjection * glm::vec4(origin, 1.0f);
    const glm::vec4 clipDir    = state.viewProjection * glm::vec4(direction, 0.0f);
    const float      width     = static_cast<float>(state.fieldWidth);
    const float      height     = static_cast<float>(state.fieldHeight);

    constexpr std::uint32_t maxSteps = 48;
    constexpr float         thickness = 0.5f;
    // The world step is derived from the on-screen ray extent so near-surface
    // reflections get dense sampling and long grazes stay cheap.
    const glm::vec2 ndcStart(clipOrigin.x / clipOrigin.w, clipOrigin.y / clipOrigin.w);
    const float      projectedExtent = glm::length(glm::vec2(
        (clipOrigin.x + clipDir.x * maxDistance) / (clipOrigin.w + clipDir.w * maxDistance) - ndcStart.x,
        (clipOrigin.y + clipDir.y * maxDistance) / (clipOrigin.w + clipDir.w * maxDistance) - ndcStart.y));
    const float screenSteps = std::max(projectedExtent * 0.5f * (width + height), 4.0f);
    const std::uint32_t steps = std::min(maxSteps, static_cast<std::uint32_t>(screenSteps));
    const float stepLength = maxDistance / static_cast<float>(steps);

    for (std::uint32_t step = 1; step <= steps; ++step)
    {
        const float t = step * stepLength;
        const float w = clipOrigin.w + clipDir.w * t;
        if (w <= nearW)
        {
            break; // behind the eye: no more screen content
        }
        const float u = (clipOrigin.x + clipDir.x * t) / w * 0.5f + 0.5f;
        const float v = 0.5f - (clipOrigin.y + clipDir.y * t) / w * 0.5f;
        if (u < 0.0f || u >= 1.0f || v < 0.0f || v >= 1.0f)
        {
            // Off-screen: Lumen hands the remainder to the scene trace.
            return result;
        }
        const std::uint32_t x = std::min(static_cast<std::uint32_t>(u * width), state.fieldWidth - 1u);
        const std::uint32_t y = std::min(static_cast<std::uint32_t>(v * height), state.fieldHeight - 1u);
        const std::size_t index = static_cast<std::size_t>(y) * state.fieldWidth + x;
        const float storedDepth = state.gbufferDepth[index];
        if (rfIsBackground(storedDepth))
        {
            continue;
        }
        const float rayDepth = glm::length((origin + direction * t) - state.cameraPosition);
        if (rayDepth > storedDepth && rayDepth - storedDepth <= thickness)
        {
            result.hit = true;
            result.x   = x;
            result.y   = y;
            outRadiance = screenColor[index];
            return result;
        }
        // A ray well behind the stored surface here means it slipped past
        // thin geometry between steps; keep marching.
    }
    return result;
}

// ---- the stochastic ray pass ---------------------------------------------------------

glm::vec3 rfSampleGgxVndf(const glm::vec3 &view, const glm::vec3 &normal, float alpha, std::uint64_t &seed)
{
    // Heitz 2018, "Sampling the GGX Distribution of Visible Normals",
    // isotropic case, evaluated in the tangent frame (+Z = normal): the
    // view is stretched into the hemisphere configuration, the projected
    // area is sampled with a (r, phi) disk parameterization, and the sampled
    // microfacet normal is transformed back to the unstretched space before
    // reflecting the view. The lobe pdf cancels the lobe half of the BRDF, so
    // the traced radiance IS the estimator (the caller applies the Fresnel
    // weight; the masking term is left to the denoising filters - the
    // standard real-time stochastic-SSR approximation).
    const glm::mat3 basis = makeBasis(normal);
    const glm::vec3 localView = glm::normalize(glm::transpose(basis) * view);
    const float     alphaUsed = std::max(alpha, 1.0e-3f);

    // Section 3.2: transform the view direction to the hemisphere
    // configuration.
    const glm::vec3 hemisphereView
        = glm::normalize(glm::vec3(alphaUsed * localView.x, alphaUsed * localView.y, localView.z));
    const float lensq = hemisphereView.x * hemisphereView.x + hemisphereView.y * hemisphereView.y;
    glm::vec3 tangent;
    if (lensq > 1.0e-12f)
    {
        tangent = glm::normalize(glm::vec3(-hemisphereView.y, hemisphereView.x, 0.0f));
    }
    else
    {
        tangent = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    const glm::vec3 bitangent = glm::cross(hemisphereView, tangent);

    // Section 3.2: parameterization of the projected area.
    const float r   = std::sqrt(std::clamp(randomFloat(seed), 0.0f, 1.0f - 1.0e-6f));
    const float phi = randomFloat(seed) * 2.0f * kPi;
    float       t1  = r * std::cos(phi);
    float       t2  = r * std::sin(phi);
    const float s   = 0.5f * (1.0f + hemisphereView.z);
    t2              = (1.0f - s) * std::sqrt(std::max(0.0f, 1.0f - t1 * t1)) + s * t2;

    // Section 3.3: the sampled microfacet normal, back in the unstretched
    // tangent frame.
    const glm::vec3 halfVector = glm::normalize(
        glm::vec3(alphaUsed * (t1 * tangent.x + t2 * bitangent.x),
            alphaUsed * (t1 * tangent.y + t2 * bitangent.y),
            std::max(hemisphereView.z, 0.0f)));
    // Section 3.4: reflect the local view on the sampled half-vector.
    glm::vec3 localDirection = glm::reflect(-localView, halfVector);
    if (glm::dot(localDirection, glm::vec3(0.0f, 0.0f, 1.0f)) <= 0.0f)
    {
        // Sampled microfacet below the horizon: fall back to the mirror
        // direction so the sample still carries energy.
        localDirection = glm::reflect(-localView, glm::vec3(0.0f, 0.0f, 1.0f));
    }
    return glm::normalize(basis * localDirection);
}

// Fresnel weight of one sample (Schlick on the view/half-vector cosine),
// applied at trace time so dielectrics and metals both get the right share.
glm::vec3 rfFresnelWeight(const glm::vec3 &view, const glm::vec3 &sampledDirection, const glm::vec3 &albedo, float metallic)
{
    const glm::vec3 halfVector = glm::normalize(view + sampledDirection);
    const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);
    const glm::vec3 f0        = glm::mix(glm::vec3(kDielectricF0), albedo, metallic);
    const glm::vec3 specular  = f0 + (1.0f - f0) * std::pow(1.0f - vDotH, 5.0f);
    return specular;
}

// One full reflection ray: screen trace tier, scene trace tier, skylight.
glm::vec3 resolveDedicatedRay(const RfPipelineState &state, const std::vector<glm::vec3> &screenColor,
    const glm::vec3 &position, const glm::vec3 &normal, const glm::vec3 &view, const glm::vec3 &direction,
    const glm::vec3 &fresnel, RfResolve &outResolve)
{
    const float screenDistance = std::max(state.settings.screenRayDistance, 0.05f);
    glm::vec3   radiance(0.0f);
    ScreenTraceResult screen = screenTrace(state, position + normal * state.settings.traceBias, direction,
        screenColor, screenDistance, radiance);
    if (screen.hit)
    {
        outResolve = RfResolve::ScreenTrace;
        return radiance * fresnel;
    }
    const RfRay ray{position + normal * state.settings.traceBias, direction, state.settings.maxTraceDistance};
    const RfHit hit = rfTraceScene(state, ray);
    if (hit.hit)
    {
        outResolve = RfResolve::SceneTrace;
        return rfRadianceAtHit(state, hit) * fresnel;
    }
    outResolve = RfResolve::Sky;
    return state.environment.skyRadiance(direction) * fresnel;
}

void rfTraceReflections(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.fieldRaw.assign(pixelCount, glm::vec3(0.0f));
    state.fieldResolve.assign(pixelCount, RfResolve::None);
    state.fieldValid.assign(pixelCount, 0u);
    state.fieldRayCount.assign(pixelCount, 0.0f);
    state.pixelsTracedThisFrame.store(0, std::memory_order_relaxed);
    state.raysTracedThisFrame.store(0, std::memory_order_relaxed);
    state.raysReusedCacheThisFrame.store(0, std::memory_order_relaxed);
    state.screenTraceHitsThisFrame.store(0, std::memory_order_relaxed);
    state.sceneTraceHitsThisFrame.store(0, std::memory_order_relaxed);

    std::vector<glm::vec3> screenColor;
    rfShadeScreenColor(state, scheduler, screenColor);

    const std::uint32_t raysPerPixel = std::max(state.settings.raysPerPixel, 1u);
    const float         rayBandMax  = std::min(state.settings.roughRayBandMax, state.settings.roughMaxTrace);
    const std::size_t   width       = state.fieldWidth;

    rfParallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            std::uint32_t pixelsDone = 0;
            std::uint32_t raysDone   = 0;
            std::uint32_t reusedDone = 0;
            std::uint32_t screenHits = 0;
            std::uint32_t sceneHits  = 0;
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * width + x;
                    if (rfIsBackground(state.gbufferDepth[index]) || state.gbufferUnlit[index] != 0u)
                    {
                        continue;
                    }
                    const float roughness = state.gbufferRoughness[index];
                    if (roughness > state.settings.roughMaxTrace)
                    {
                        continue; // above Max Roughness To Trace: no specular term
                    }
                    const glm::vec3 position = state.gbufferPosition[index];
                    const glm::vec3 normal   = state.gbufferNormal[index];
                    const glm::vec3 albedo   = state.gbufferAlbedo[index];
                    const float     metallic = state.gbufferMetallic[index];
                    glm::vec3       view     = state.cameraPosition - position;
                    const float     viewLength = glm::length(view);
                    if (viewLength < 1.0e-4f)
                    {
                        continue;
                    }
                    view /= viewLength;
                    const float nDotV = glm::dot(normal, view);
                    if (nDotV <= 0.0f)
                    {
                        continue;
                    }
                    const float clampedRoughness = std::max(roughness, kMinimumRoughness);
                    const float alpha            = clampedRoughness * clampedRoughness;

                    std::uint64_t seed = hashSeed(state.frameIndex * 7919ull + index * 104729ull);
                    glm::vec3    sum(0.0f);
                    RfResolve   resolve = RfResolve::None;
                    for (std::uint32_t ray = 0; ray < raysPerPixel; ++ray)
                    {
                        const glm::vec3 direction = rfSampleGgxVndf(view, normal, alpha, seed);
                        const glm::vec3 fresnel  = rfFresnelWeight(view, direction, albedo, metallic);
                        if (roughness < rayBandMax)
                        {
                            // Dedicated reflection ray through the tracing
                            // tiers.
                            RfResolve rayResolve = RfResolve::None;
                            const glm::vec3 radiance = resolveDedicatedRay(
                                state, screenColor, position, normal, view, direction, fresnel, rayResolve);
                            sum += radiance;
                            ++raysDone;
                            resolve = rayResolve;
                            if (rayResolve == RfResolve::ScreenTrace)
                            {
                                ++screenHits;
                            }
                            else if (rayResolve == RfResolve::SceneTrace)
                            {
                                ++sceneHits;
                            }
                        }
                        else
                        {
                            // Rough band: resample the world radiance cache
                            // (Lumen's radiance-cache reuse); the analytic sky
                            // when no probe covers the position.
                            glm::vec3 radiance(0.0f);
                            if (!rfProbeRadiance(state,
                                    position + normal * state.settings.traceBias, direction, radiance))
                            {
                                radiance = state.environment.skyRadiance(direction);
                            }
                            sum += radiance * fresnel;
                            ++reusedDone;
                            resolve = RfResolve::Cache;
                        }
                    }
                    state.fieldRaw[index]    = sum / static_cast<float>(raysPerPixel);
                    state.fieldValid[index]  = 1u;
                    state.fieldRayCount[index] = static_cast<float>(raysPerPixel);
                    state.fieldResolve[index] = resolve;
                    ++pixelsDone;
                }
            }
            state.pixelsTracedThisFrame.fetch_add(pixelsDone, std::memory_order_relaxed);
            state.raysTracedThisFrame.fetch_add(raysDone, std::memory_order_relaxed);
            state.raysReusedCacheThisFrame.fetch_add(reusedDone, std::memory_order_relaxed);
            state.screenTraceHitsThisFrame.fetch_add(screenHits, std::memory_order_relaxed);
            state.sceneTraceHitsThisFrame.fetch_add(sceneHits, std::memory_order_relaxed);
        });
    state.totalRaysTraced.fetch_add(
        state.raysTracedThisFrame.load(std::memory_order_relaxed)
            + state.raysReusedCacheThisFrame.load(std::memory_order_relaxed),
        std::memory_order_relaxed);
}

// ---- frame orchestration --------------------------------------------------------------

bool rfRunReflectionFrame(RfPipelineState &state, TaskScheduler *scheduler)
{
    try
    {
        rfRasterizeGBuffer(state, scheduler);
        rfTraceReflections(state, scheduler);
        rfSpatialReuse(state, scheduler);
        rfTemporalAccumulate(state, scheduler);
        rfbilateralFilter(state, scheduler);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace lumen_reflection
} // namespace engine
