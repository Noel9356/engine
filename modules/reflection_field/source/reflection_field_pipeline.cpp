// reflection_field pipeline: the frame GBuffer, the shaded screen color, the
// stochastic GGX ray pass, and the hybrid tracing tiers.
//
// Stage order per evaluate() (Lumen's reflection solve):
//   1. rasterize the GBuffer at field resolution (depth, position, normal,
//      albedo, roughness/metallic, emission);
//   2. per texel inside the traced roughness bands, importance-sample the
//      visible GGX lobe and resolve each ray: screen trace first (against the
//      frame GBuffer), then a scene trace with hit lighting (sun + direct
//      lights + the radiance_cache probe-grid diffuse GI + emission), then the
//      skylight; the rough band resamples the radiance_cache probe radiance
//      instead of tracing rays. Screen-trace hits land on GBuffer texels
//      whose shaded color is resolved by a lazy shading pass that runs over
//      exactly the texels the rays actually hit (Lumen Scene lighting on
//      demand) and is applied afterwards;
//   3. the denoise chain (reflection_denoise.cpp) turns the stochastic field
//      into the published one.

#include "reflection_field_internal.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{
namespace reflection_field
{

namespace
{

constexpr float nearW = 0.05f;         // near-plane clip distance of the raster
constexpr std::uint32_t bandRows = 16;  // rasterized rows per parallel band

// ---- GBuffer raster ------------------------------------------------------------

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

// Projects one surface's triangles to the field, caching them per surface:
// the world-space vertices depend only on the surface transform, the clip
// vertices on the view, and the projected triangles on both, so the work is
// only redone when the surface's recorded raster serial no longer matches the
// pipeline's (view / registry / field-size change, or a transform update).
void setupSurfaceTriangles(RfPipelineState &state, std::uint32_t surfaceIndex, std::uint32_t width,
    std::uint32_t height, std::vector<RfScreenTriangle> &out)
{
    RfSurface &surface = state.surfaces[surfaceIndex];
    if (surface.rasterBuiltSerial == state.rasterSerial)
    {
        return; // the projected triangles are still current
    }
    out.clear();
    surface.rasterBuiltSerial = state.rasterSerial;
    if (surface.excluded || surfaceOutsideFrustum(state.viewProjection, surface))
    {
        return;
    }
    const RfSurfaceGeometry &geometry = surface.geometry;
    const std::size_t         vertexCount = geometry.positions.size();
    if (surface.rasterWorldPositions.size() != vertexCount)
    {
        surface.rasterWorldPositions.resize(vertexCount);
        surface.rasterWorldDirty = true;
    }
    if (surface.rasterWorldDirty)
    {
        for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
        {
            surface.rasterWorldPositions[vertex]
                = glm::vec3(surface.transform * glm::vec4(geometry.positions[vertex], 1.0f));
        }
        surface.rasterWorldDirty = false;
    }
    surface.rasterClipPositions.resize(vertexCount);
    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
    {
        surface.rasterClipPositions[vertex] = state.viewProjection * glm::vec4(surface.rasterWorldPositions[vertex], 1.0f);
    }
    const std::uint32_t triangleCount = static_cast<std::uint32_t>(geometry.indices.size() / 3);
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        RfClipVertex polygon[4];
        std::uint32_t count = 0;
        RfClipVertex input[3];
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            const std::uint32_t index = geometry.indices[triangle * 3 + corner];
            input[corner].clip  = surface.rasterClipPositions[index];
            input[corner].world = surface.rasterWorldPositions[index];
            input[corner].bary  = glm::vec3(corner == 0 ? 1.0f : 0.0f, corner == 1 ? 1.0f : 0.0f, corner == 2 ? 1.0f : 0.0f);
        }
        // Sutherland-Hodgman against the near plane w = nearW.
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            const RfClipVertex &current = input[corner];
            const RfClipVertex &next    = input[(corner + 1) % 3];
            const float       dCurrent = current.clip.w - nearW;
            const float       dNext    = next.clip.w - nearW;
            if (dCurrent >= 0.0f)
            {
                polygon[count++] = current;
            }
            if ((dCurrent >= 0.0f) != (dNext >= 0.0f))
            {
                const float t = dCurrent / (dCurrent - dNext);
                RfClipVertex edge;
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
        RfScreenVertex projected[4];
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
            RfScreenTriangle screen;
            screen.vertex[0] = projected[0];
            screen.vertex[1] = projected[fan];
            screen.vertex[2] = projected[fan + 1];
            screen.surface   = surfaceIndex;
            screen.triangle  = triangle;
            out.push_back(screen);
        }
    }
}

void rasterizeBand(RfPipelineState &state, const std::vector<RfScreenTriangle> &triangles,
    const std::vector<std::uint32_t> &binned, std::uint32_t rowBegin, std::uint32_t rowEnd)
{
    const std::uint32_t width = state.fieldWidth;
    for (const std::uint32_t triangleIndex : binned)
    {
        const RfScreenTriangle &screen = triangles[triangleIndex];
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
        const float invDenominator = 1.0f / denominator;
        const RfSurface         &surface  = state.surfaces[screen.surface];
        const RfSurfaceGeometry &geometry = surface.geometry;
        const std::uint32_t      i0 = geometry.indices[screen.triangle * 3 + 0];
        const std::uint32_t      i1 = geometry.indices[screen.triangle * 3 + 1];
        const std::uint32_t      i2 = geometry.indices[screen.triangle * 3 + 2];
        // World-space vertex normals through the surface's normal matrix (the
        // inverse transpose): the plain linear matrix skews them under
        // non-uniform scale. The transform is linear, so shifting it before the
        // barycentric interpolation instead of after gives the same
        // interpolated direction (and the length change the interpolation
        // applies is erased by the final normalize).
        const glm::vec3 worldNormal0 = surface.normalMatrix * geometry.normals[i0];
        const glm::vec3 worldNormal1 = surface.normalMatrix * geometry.normals[i1];
        const glm::vec3 worldNormal2 = surface.normalMatrix * geometry.normals[i2];
        for (std::int32_t y = yBegin; y < yEnd; ++y)
        {
            for (std::int32_t x = xBegin; x < xEnd; ++x)
            {
                const glm::vec2 toPixel(static_cast<float>(x) + 0.5f - a.x, static_cast<float>(y) + 0.5f - a.y);
                const float     u = (toPixel.x * edge2.y - edge2.x * toPixel.y) * invDenominator;
                const float     v = (edge1.x * toPixel.y - toPixel.x * edge1.y) * invDenominator;
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
                const float invSum = 1.0f / sum;
                const float mu0 = w0 * invSum;
                const float mu1 = w1 * invSum;
                const float mu2 = w2 * invSum;
                const glm::vec3 worldPosition = screen.vertex[0].world * mu0 + screen.vertex[1].world * mu1
                                              + screen.vertex[2].world * mu2;
                const glm::vec3   relative = worldPosition - state.cameraPosition;
                const float       distanceSquared = glm::dot(relative, relative);
                const std::size_t pixelIndex = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
                // Squared comparison against the stored camera distance (the
                // square root is only spent on the texels that win the depth
                // test; the background sentinel squares to infinity).
                if (distanceSquared >= state.gbufferDepth[pixelIndex] * state.gbufferDepth[pixelIndex])
                {
                    continue;
                }
                const glm::vec3 bary = screen.vertex[0].bary * mu0 + screen.vertex[1].bary * mu1
                                     + screen.vertex[2].bary * mu2;
                glm::vec3 interpolatedNormal = worldNormal0 * bary.x + worldNormal1 * bary.y
                                              + worldNormal2 * bary.z;
                const float normalLength = glm::length(interpolatedNormal);
                const glm::vec3 normal = normalLength > 1.0e-8f ? interpolatedNormal / normalLength
                                                               : glm::vec3(0.0f, 1.0f, 0.0f);
                state.gbufferDepth[pixelIndex]         = std::sqrt(distanceSquared);
                state.gbufferPosition[pixelIndex]       = worldPosition;
                state.gbufferSurface[pixelIndex]        = screen.surface;
                state.gbufferNormal[pixelIndex]         = normal;
                state.gbufferAlbedo[pixelIndex] = surface.albedo
                    * (geometry.colors[i0] * bary.x + geometry.colors[i1] * bary.y + geometry.colors[i2] * bary.z);
                state.gbufferRoughness[pixelIndex]      = surface.roughness;
                state.gbufferMetallic[pixelIndex]       = surface.metallic;
                state.gbufferOpacity[pixelIndex]        = surface.opacity;
                state.gbufferIor[pixelIndex]            = surface.indexOfRefraction;
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
    state.gbufferOpacity.assign(pixelCount, 1.0f);
    state.gbufferIor.assign(pixelCount, 1.0f);
    state.gbufferEmissive.assign(pixelCount, glm::vec3(0.0f));
    state.gbufferUnlit.assign(pixelCount, 0u);
    state.gbufferSurface.assign(pixelCount, noSurface);

    // A field-size change retires every projected-triangle cache (the pixel
    // coordinates depend on the resolution).
    if (state.rasterFieldWidth != width || state.rasterFieldHeight != height)
    {
        state.rasterFieldWidth  = width;
        state.rasterFieldHeight = height;
        ++state.rasterSerial;
    }
    if (state.rasterTriangles.size() < state.surfaces.size())
    {
        state.rasterTriangles.resize(state.surfaces.size());
    }
    parallelRange(scheduler, state.surfaces.size(), 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t surface = begin; surface < end; ++surface)
            {
                setupSurfaceTriangles(state, static_cast<std::uint32_t>(surface), width, height,
                    state.rasterTriangles[surface]);
            }
        });
    state.rasterConcat.clear();
    const std::size_t cachedSurfaces = std::min(state.surfaces.size(), state.rasterTriangles.size());
    for (std::size_t surface = 0; surface < cachedSurfaces; ++surface)
    {
        state.rasterConcat.insert(state.rasterConcat.end(), state.rasterTriangles[surface].begin(),
            state.rasterTriangles[surface].end());
    }
    const std::uint32_t bandCount = (height + bandRows - 1) / bandRows;
    if (state.rasterBins.size() != bandCount)
    {
        state.rasterBins.assign(bandCount, std::vector<std::uint32_t>());
    }
    else
    {
        for (std::vector<std::uint32_t> &bin : state.rasterBins)
        {
            bin.clear();
        }
    }
    for (std::uint32_t index = 0; index < state.rasterConcat.size(); ++index)
    {
        const RfScreenTriangle &screen = state.rasterConcat[index];
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
            state.rasterBins[band].push_back(index);
        }
    }
    parallelRange(scheduler, bandCount, 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t band = begin; band < end; ++band)
            {
                const std::uint32_t rowBegin = static_cast<std::uint32_t>(band) * bandRows;
                rasterizeBand(state, state.rasterConcat, state.rasterBins[band], rowBegin, std::min(rowBegin + bandRows, height));
            }
        });
}

// ---- hit lighting -------------------------------------------------------------------

// Sky radiance for reflection rays (contract in reflection_field_internal.hpp):
// the environment's hemispheric skylight plus the analytic sun disc.
glm::vec3 rfSkyRadiance(const RfPipelineState &state, const glm::vec3 &direction)
{
    glm::vec3 radiance = state.environment.skyRadiance(direction);
    if (state.environment.hasSun && state.environment.sunEnabled)
    {
        const glm::vec3 sunRadiance = state.environment.sunRadiance();
        if (glm::dot(sunRadiance, sunRadiance) > 0.0f)
        {
            const float radians = glm::clamp(state.settings.skySunDiscDegrees, 0.05f, 15.0f) * (3.141592654f / 180.0f);
            const glm::vec3 normalized = glm::normalize(direction);
            if (glm::dot(normalized, state.environment.sunDirection) >= std::cos(radians))
            {
                radiance = glm::max(radiance, sunRadiance);
            }
        }
    }
    return radiance;
}

// Radiance a reflection ray sees when it lands on a surface (Lumen's hit
// lighting): direct sun + direct lights (same formulas as the renderer's
// shading), the radiance_cache probe-grid diffuse GI, emission, and the hemispheric
// ambient fallback while the probe grid carries no data. The optional
// specular block evaluates the surface's own view-dependent response from the
// world radiance cache: the cache answers along the mirrored view direction,
// Schlick-weighted by the surface's base reflectivity, for roughness inside
// the traced band (Lumen's cached scene response for reflective hits; a
// reflection ray landing on a mirror must see the mirror's room image, not
// the near-black diffuse of a metal).
glm::vec3 rfShadeHitPoint(const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    const glm::vec3 &albedo, const glm::vec3 &emissive, const glm::vec3 &viewDirection, float roughness,
    float metallic)
{
    glm::vec3 result = emissive;
    const glm::vec3 diffuse = albedo * (1.0f / kPi);

    const RfEnvironmentSnapshot &environment = state.environment;
    // Photon caustics from the radiance_cache (when coupled): a hit on a
    // caustic receiver sits behind a transmitter, and the photon estimate IS
    // the sun's transported energy there - the interface Fresnel, the
    // medium's absorption, and the focusing net included. The unoccluded sun
    // term below is REPLACED by it for such hits (adding both would count
    // the sun twice and wash the caustic net out into a flat bright floor);
    // the direct point lights keep their own terms (their photon transport
    // is a second-order share the cones only carry while a light hangs near
    // the transmitter).
    glm::vec3 causticIrradiance(0.0f);
    bool      causticsAnswered = false;
    if (state.radianceCacheCapability != nullptr && state.radianceCacheCapability->sampleCausticsIrradiance != nullptr)
    {
        causticsAnswered = state.radianceCacheCapability->sampleCausticsIrradiance(
            state.radianceCacheCapability->self, &position.x, &normal.x, &causticIrradiance.x);
    }
    // Sun, unoccluded exactly like the renderer's own direct term: the field
    // composites onto the renderer's shaded pixels, so reflection hit lighting
    // must match the presented image's occlusion model (occlusion carried only
    // by reflections reads as dark objects the scene does not contain). A
    // renderer with a shadow system is the place to reintroduce it.
    const glm::vec3 sunRadiance = environment.sunRadiance();
    if (!causticsAnswered && environment.hasSun && environment.sunEnabled
        && glm::dot(sunRadiance, sunRadiance) > 0.0f)
    {
        const float nDotL = glm::dot(normal, environment.sunDirection);
        if (nDotL > 0.0f)
        {
            result += diffuse * sunRadiance * nDotL;
        }
    }

    // Direct lights (point/spot/directional of the direct_lighting module):
    // the shared lighting::lightArrival evaluates the renderer's attenuation
    // formulas and hands back the unoccluded arrival; point-light shadows on
    // reflection hits are off by default (the module carries no dedicated
    // pointLightShadows setting; the unoccluded sun above follows the
    // renderer's occlusion model).
    for (const RfEnvironmentSnapshot::PreparedDirectLight &light : environment.preparedLights)
    {
        glm::vec3 lightDirection(0.0f);
        float     lightDistance = 0.0f;
        glm::vec3 arrival(0.0f);
        if (!lighting::lightArrival(light, position, normal, lightDirection, lightDistance, arrival))
        {
            continue;
        }
        result += diffuse * arrival;
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
    // The caustic receiver's transported sun (sampled at the top of the
    // function) adds where the photon estimate answered; the sun block above
    // skipped its direct term for exactly those hits.
    if (causticsAnswered)
    {
        result += diffuse * causticIrradiance;
    }

    // Specular response of the hit's own material, answered by the world
    // radiance cache along the mirrored view direction (the cache-band rule:
    // roughness above Max Roughness To Trace carries no traced specular, and
    // the zero view direction marks callers that did not ask for it).
    if (roughness <= state.settings.roughMaxTrace && glm::dot(viewDirection, viewDirection) > 0.5f)
    {
        const glm::vec3 view = glm::normalize(viewDirection);
        const glm::vec3 mirrorDirection = glm::reflect(-view, normal);
        glm::vec3 radiance(0.0f);
        if (rfProbeRadiance(state, position, mirrorDirection, radiance))
        {
            const float nDotV = glm::max(glm::dot(normal, view), 0.0f);
            const glm::vec3 f0 = glm::mix(glm::vec3(kDielectricF0), albedo, metallic);
            result += (f0 + (glm::vec3(1.0f) - f0) * rfFresnelSchlickScalar(nDotV, 0.0f)) * radiance;
        }
    }
    return result;
}

// Radiance of a scene-traced hit (anti-leak: backface hits return nothing,
// radiance_cache's rule). Unlit hits emit their albedo. `incomingDirection` is the
// ray's travel direction: the hit's specular samples the cache along its
// mirror (the exact view the chain arrives with).
glm::vec3 rfRadianceAtHit(const RfPipelineState &state, const RfHit &hit, const glm::vec3 &incomingDirection)
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
    return rfShadeHitPoint(state, hit.position, hit.normal, hit.albedo, surface.emissive, -incomingDirection,
        surface.roughness, surface.metallic);
}

// ---- screen traces -------------------------------------------------------------------

// Screen-space march of one reflection ray (Lumen's screen traces): steps
// through the field's depth buffer in equal world-space increments (the ray
// stays linear in world distance), tests each visited texel's stored camera
// distance against the ray's, and accepts the hit when the ray passes behind
// the stored surface by at most `thickness` (thin-occluder heuristic). Rays
// leaving the screen or exceeding the trace distance miss, handing the rest
// to the scene trace. Background texels never intersect (the sky sits behind
// everything). The hit's radiance is not read here: the texel is shaded by the
// lazy pass that follows the ray pass, so only texels rays actually land on
// are ever shaded.
struct ScreenTraceResult
{
    bool        hit = false;
    std::size_t texel = 0;
    // Transmission tint accumulated while the march passed through
    // transparent GBuffer texels (white = none). Meaningful only when hit is
    // set: a march that ends without an opaque hit discards its tint, because
    // the fall-back scene trace re-hits those surfaces and applies their
    // refraction (bending included) itself - tinting both tiers would
    // attenuate the same pane twice.
    glm::vec3   tint{1.0f};
    // How many transparent texels the march passed through (the transmission
    // segment count the statistics report for screen traces).
    std::uint32_t passThroughs = 0;
};

ScreenTraceResult screenTrace(const RfPipelineState &state, const glm::vec4 &clipOrigin, const glm::vec3 &origin,
    const glm::vec3 &direction, float maxDistance)
{
    ScreenTraceResult result;
    // Clip-space linearization: clip(t) = clipOrigin + clipDir * t lets the
    // march project each step exactly without per-step matrix products. The
    // origin's clip position is shared by every ray of a texel, so the caller
    // passes it in.
    const glm::vec4 clipDir = state.viewProjection * glm::vec4(direction, 0.0f);
    const float     width   = static_cast<float>(state.fieldWidth);
    const float     height  = static_cast<float>(state.fieldHeight);

    const std::uint32_t maxSteps  = std::max(state.settings.screenTraceMaxSteps, 1u);
    const float         thickness = std::max(state.settings.screenTraceThickness, 0.01f);
    // The world step is derived from the on-screen ray extent so near-surface
    // reflections get dense sampling and long grazes stay cheap.
    const glm::vec2 ndcStart(clipOrigin.x / clipOrigin.w, clipOrigin.y / clipOrigin.w);
    const float      projectedExtent = glm::length(glm::vec2(
        (clipOrigin.x + clipDir.x * maxDistance) / (clipOrigin.w + clipDir.w * maxDistance) - ndcStart.x,
        (clipOrigin.y + clipDir.y * maxDistance) / (clipOrigin.w + clipDir.w * maxDistance) - ndcStart.y));
    const float screenSteps = std::max(projectedExtent * 0.5f * (width + height), 4.0f);
    const std::uint32_t steps = std::min(maxSteps, static_cast<std::uint32_t>(screenSteps));
    const float stepLength = maxDistance / static_cast<float>(steps);

    // Camera distance of the marched point as a quadratic in t (the direction
    // is unit length): d(t)^2 = t^2 + linear * t + constant, so each step
    // needs two multiplies and an add instead of a vector rebuild. The
    // comparison runs entirely on squared distances - both the hit test
    // (ray behind the stored surface) and the thin-occluder thickness test
    // (rayDepth - storedDepth <= thickness, with both distances non-negative,
    // is rayDepth <= storedDepth + thickness) are order-preserving under
    // squaring, so the per-step square root disappears.
    const glm::vec3 relative = origin - state.cameraPosition;
    const float     linear   = 2.0f * glm::dot(direction, relative);
    const float     constant = glm::dot(relative, relative);

    // The transparent surface the march last passed through (and its texel):
    // a step smaller than a texel can revisit the same texel across
    // consecutive steps, and one glass body covers many texels - a ray
    // crossing its own medium must attenuate once, not once per texel.
    std::uint32_t lastTransparentSurface = noSurface;
    std::size_t   lastTransparentTexel = std::numeric_limits<std::size_t>::max();
    for (std::uint32_t step = 1; step <= steps; ++step)
    {
        const float t = step * stepLength;
        const float w = clipOrigin.w + clipDir.w * t;
        if (w <= nearW)
        {
            break; // behind the eye: no more screen content
        }
        const float inverseW = 1.0f / w;
        const float u = (clipOrigin.x + clipDir.x * t) * inverseW * 0.5f + 0.5f;
        const float v = 0.5f - (clipOrigin.y + clipDir.y * t) * inverseW * 0.5f;
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
        const float rayDepthSquared = t * t + linear * t + constant;
        const float hitDepthLimit   = storedDepth + thickness;
        if (rayDepthSquared > storedDepth * storedDepth && rayDepthSquared <= hitDepthLimit * hitDepthLimit)
        {
            // Grazing rejection (screen_trace.grazing_limit): a hit whose
            // stored normal meets the ray at a shallower angle than the
            // configured cosine is a marching artifact, not an intersection
            // the depth test can justify - the classic case is a reflected
            // ray running nearly parallel over a floor, where every step
            // passes behind the floor by microns and the thin-occluder slack
            // accepts whichever texel the step grid happens to sample (the
            // banded reflections a raw march shows at grazing view angles).
            // Refusing keeps the march going; a chain that finds no steeper
            // hit falls through to the scene trace, which resolves the true
            // reflector exactly.
            if (state.settings.screenTraceGrazingLimit > 0.0f)
            {
                const float hitCosine = -glm::dot(direction, state.gbufferNormal[index]);
                if (hitCosine < state.settings.screenTraceGrazingLimit)
                {
                    continue;
                }
            }
            // A transparent texel transmits the ray instead of stopping it:
            // the surface's tint (albedo x (1 - opacity)) attenuates the
            // march, and it keeps searching for the opaque responder behind.
            // The reflected share of the glass is not resolved here - the
            // glass surface's own field texels carry it, and the renderer
            // samples them at the glass's pixels.
            if (state.gbufferOpacity[index] < 1.0f)
            {
                const std::uint32_t surface = state.gbufferSurface[index];
                if (index != lastTransparentTexel && surface != lastTransparentSurface)
                {
                    lastTransparentTexel   = index;
                    lastTransparentSurface = surface;
                    result.tint *= state.gbufferAlbedo[index] * (1.0f - state.gbufferOpacity[index]);
                    ++result.passThroughs;
                }
                continue;
            }
            result.hit  = true;
            result.texel = index;
            return result;
        }
        // A ray well behind the stored surface here means it slipped past
        // thin geometry between steps; keep marching.
    }
    return result;
}

// ---- the stochastic ray pass ---------------------------------------------------------

glm::vec3 rfSampleGgxVndf(const glm::mat3 &basis, const glm::vec3 &localView, float alpha, std::uint64_t &seed)
{
    // Heitz 2018, "Sampling the GGX Distribution of Visible Normals",
    // isotropic case, evaluated in the tangent frame (+Z = normal): the
    // view is stretched into the hemisphere configuration, the projected
    // area is sampled with a (r, phi) disk parameterization, and the sampled
    // microfacet normal is transformed back to the unstretched space before
    // reflecting the view. The lobe pdf cancels the lobe half of the BRDF, so
    // the traced radiance IS the estimator (the caller applies the Fresnel
    // weight; the masking term is left to the denoising filters - the
    // standard real-time stochastic-SSR approximation). The tangent frame is
    // provided by the caller because every ray of a texel shares it.
    const float alphaUsed = std::max(alpha, 1.0e-3f);

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
// `f0` is the texel's base reflectivity, hoisted by the caller.
glm::vec3 rfFresnelWeight(const glm::vec3 &view, const glm::vec3 &sampledDirection, const glm::vec3 &f0)
{
    const glm::vec3 halfVector = glm::normalize(view + sampledDirection);
    const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);
    const float     complement = 1.0f - vDotH;
    const float     complementSquared = complement * complement;
    const float     powered = complementSquared * complementSquared * complement; // (1-v)^5
    return f0 + (glm::vec3(1.0f) - f0) * powered;
}

// One ray chain through the tracing tiers with transmission (contract in
// reflection_field_internal.hpp): screen traces
// against the frame GBuffer first (transparent texels pass through with the
// surface's attenuation), then scene traces against the BVHs where
// transparent surfaces split the ray Fresnel-style - the stochastic pick
// either reflects off the interface (resolving the reflection radiance
// through a nested chain) or transmits through it (tinted by the surface's
// albedo share, bent by Snell's law with the interface's index ratio, total
// internal reflection respected, and rough lobes blurred for translucent
// surfaces). The chain weight multiplies every terminal radiance, so the
// stochastic picks are unbiased; a chain whose transmission budget runs out
// stops tracing and the world radiance cache answers its transmittance share
// along the would-be continuation. The caller's `weight` carries the
// source-interface share (the Fresnel weight for a reflected lobe, the
// transmitted share for a refraction lobe); `remainingSegments` bounds how
// many more interfaces the chain may cross.
RfChainResolve resolveRayChain(const RfPipelineState &state, const glm::vec4 &clipOrigin, const glm::vec3 &origin,
    const glm::vec3 &direction, const glm::vec3 &weight, std::uint32_t remainingSegments, bool allowScreenTrace,
    std::uint64_t &seed)
{
    RfChainResolve result;
    if (allowScreenTrace && state.settings.screenTracesEnabled)
    {
        const float screenDistance = std::max(state.settings.screenRayDistance, 0.05f);
        ScreenTraceResult screen = screenTrace(state, clipOrigin, origin, direction, screenDistance);
        if (screen.hit)
        {
            result.deferred[0].hitTexel = static_cast<std::uint32_t>(screen.texel);
            result.deferred[0].tint     = weight * screen.tint;
            result.deferredCount        = 1u;
            result.resolve              = RfResolve::ScreenTrace;
            result.segments             = screen.passThroughs;
            return result;
        }
    }
    const RfRay ray{origin, direction, state.settings.maxTraceDistance};
    const RfHit hit = rfTraceScene(state, ray);
    if (!hit.hit)
    {
        result.radiance = rfSkyRadiance(state, direction) * weight;
        result.resolve  = RfResolve::Sky;
        return result;
    }
    const RfSurface &surface = state.surfaces[hit.surfaceIndex];
    if (surface.opacity >= 1.0f)
    {
        result.radiance = rfRadianceAtHit(state, hit, direction) * weight;
        result.resolve  = RfResolve::SceneTrace;
        return result;
    }

    // Transparent interface: one transmission segment, then the Fresnel
    // split. `entering` distinguishes the outside -> medium crossing from the
    // medium -> outside one and picks the index ratio accordingly. The
    // interface works on the ray-facing normal: rfTraceScene keeps the
    // outward normal on a true backface (its flip handles only the winding
    // disagreement), and the refraction needs the normal on the ray's side to
    // bend across the interface instead of mirroring back into the medium.
    // The refraction is computed before the budget check so an exhausted chain
    // can still fall back to the cache along the direction the ray would
    // continue with (refracted, or mirrored under total internal reflection)
    // instead of reading black.
    ++result.segments;
    const bool    entering = !hit.backface;
    const float   ior      = glm::max(surface.indexOfRefraction, 1.0f);
    const float   n1       = entering ? 1.0f : ior;
    const float   n2       = entering ? ior : 1.0f;
    const glm::vec3 interfaceNormal = glm::dot(hit.normal, direction) < 0.0f ? hit.normal : -hit.normal;
    const float   cosI     = glm::clamp(glm::dot(-direction, interfaceNormal), 0.0f, 1.0f);
    const float   fresnel  = rfFresnelSchlickScalar(cosI, rfDielectricF0FromIor(n1, n2));
    const glm::vec3 transmitted = glm::refract(direction, interfaceNormal, n1 / n2);
    if (remainingSegments == 0)
    {
        // Budget exhausted: the chain stops tracing, but the pixel keeps its
        // transmittance share - the world radiance cache answers along the
        // would-be continuation (the skylight when nothing covers the
        // position), so deep glass chains fade toward the room instead of
        // absorbing to black.
        const glm::vec3 continuation = glm::dot(transmitted, transmitted) > 1.0e-8f
            ? glm::normalize(transmitted)
            : glm::reflect(direction, interfaceNormal);
        glm::vec3 cached(0.0f);
        if (!rfProbeRadiance(state, hit.position, continuation, cached))
        {
            cached = rfSkyRadiance(state, continuation);
        }
        result.radiance = cached * weight * (1.0f - fresnel)
            * (entering ? surface.albedo : glm::vec3(1.0f));
        result.resolve  = RfResolve::Cache;
        return result;
    }
    // Every transparent-interface branch consumes one segment of the
    // remaining budget (opaque hits end the chain without consuming any), so
    // mutually facing transparent surfaces cannot recurse without bound.
    --remainingSegments;
    // The interface's split resolves BOTH branches deterministically (the
    // same split estimator the first segment uses): the reflected branch at
    // its Fresnel weight, the transmitted branch at its `(1 - Fresnel)`
    // weight. A one-pick split would trace either branch at full weight -
    // the rare draw that catches a bright emitter through the interface
    // would swing the texel's running mean far past the signal - while the
    // deterministic split's per-frame variance is only the lobes' jitter,
    // which the temporal accumulation averages into a smooth response.
    // Reflected: off the interface; the reflected path leaves the medium (or
    // stays inside under total internal reflection below), so its nested
    // chain may use the screen tier again.
    {
        const glm::vec3 reflectDirection = glm::reflect(direction, interfaceNormal);
        const glm::vec4 reflectClip = state.viewProjection
            * glm::vec4(hit.position + interfaceNormal * state.settings.traceBias, 1.0f);
        result.add(resolveRayChain(state, reflectClip, hit.position + interfaceNormal * state.settings.traceBias,
            reflectDirection, weight * fresnel, remainingSegments, true, seed));
    }
    // Transmitted: bent by Snell's law; total internal reflection folds the
    // whole `(1 - Fresnel)` share into a deterministic full reflect (the path
    // stays inside the medium on scene traces). The albedo absorption applies
    // once per medium traversal, at the ENTERING interface - the matching
    // exit passes untinted, so a solid pane attenuates exactly one tint
    // however many interfaces the bent path crosses inside it.
    const bool exitCrossing = hit.backface;
    if (glm::dot(transmitted, transmitted) < 1.0e-8f)
    {
        // Total internal reflection: the continuation runs on scene traces
        // (the screen march cannot walk the bending interfaces) and offsets
        // along the mirrored direction.
        const glm::vec3 reflectDirection = glm::reflect(direction, interfaceNormal);
        const glm::vec4 reflectClip = state.viewProjection
            * glm::vec4(hit.position + reflectDirection * state.settings.traceBias, 1.0f);
        result.add(resolveRayChain(state, reflectClip, hit.position + reflectDirection * state.settings.traceBias,
            reflectDirection, weight * (1.0f - fresnel), remainingSegments, false, seed));
        return result;
    }
    glm::vec3 transmittedDirection = glm::normalize(transmitted);
    {
        // The transmitted lobe blurs by the surface's roughness (the
        // frosted-glass approximation) with the transmission antialiasing
        // floor underneath: the temporal accumulation averages the jittered
        // lobe into a smooth, alias-free refraction at field resolution.
        const float alpha = std::max(surface.roughness * surface.roughness, state.settings.transmissionLobeAlphaFloor);
        transmittedDirection = rfSampleGgxVndf(makeBasis(transmittedDirection), glm::vec3(0.0f, 0.0f, 1.0f), alpha, seed);
    }
    const glm::vec3 transmittedWeight = (exitCrossing ? weight : weight * surface.albedo) * (1.0f - fresnel);
    const glm::vec3 transmittedOrigin = hit.position + transmittedDirection * state.settings.traceBias;
    const glm::vec4 transmittedClip = state.viewProjection * glm::vec4(transmittedOrigin, 1.0f);
    // The transmitted path travels inside a medium: the screen march cannot
    // reproduce the exit interface's bending, so the rest of the chain runs
    // on scene traces only.
    result.add(resolveRayChain(state, transmittedClip, transmittedOrigin, transmittedDirection, transmittedWeight,
        remainingSegments, false, seed));
    return result;
}

void rfTraceReflections(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.fieldRaw.assign(pixelCount, glm::vec3(0.0f));
    state.fieldResolve.assign(pixelCount, RfResolve::None);
    state.fieldValid.assign(pixelCount, 0u);
    state.fieldRayCount.assign(pixelCount, 0.0f);
    state.fieldReused.assign(pixelCount, 0u);
    // The interleave phase advances every frame, so a stride above 1 rotates
    // WHICH texels trace: every texel gets its turn within `stride` frames
    // while the temporal accumulation (the denoise pass's reuse path) holds
    // the skipped ones at their reprojected history.
    const std::uint32_t traceStride = std::clamp(state.traceStride, 1u, 8u);
    const std::size_t tracePhase = state.tracePhase;
    if (state.screenColor.size() != pixelCount)
    {
        state.screenColor.resize(pixelCount);
        state.screenNeedShade.resize(pixelCount);
    }
    state.screenNeedShade.assign(pixelCount, 0u);
    if (state.screenDeferredRows.size() != state.fieldHeight)
    {
        state.screenDeferredRows.resize(state.fieldHeight);
    }
    for (std::vector<RfScreenDeferred> &row : state.screenDeferredRows)
    {
        row.clear();
    }
    // The emissive highlight capture only pays where the scene carries
    // emissive surfaces: the scan is over the registered surface list once
    // per frame (bounded by scene.max_surfaces) and skips two screen marches
    // per rough-band texel in scenes lit entirely by the sun/sky.
    bool sceneHasEmissive = false;
    for (const RfSurface &surface : state.surfaces)
    {
        if (!surface.excluded && luminance(surface.emissive) > 0.0f)
        {
            sceneHasEmissive = true;
            break;
        }
    }
    state.pixelsTracedThisFrame.store(0, std::memory_order_relaxed);
    state.raysTracedThisFrame.store(0, std::memory_order_relaxed);
    state.raysReusedCacheThisFrame.store(0, std::memory_order_relaxed);
    state.screenTraceHitsThisFrame.store(0, std::memory_order_relaxed);
    state.sceneTraceHitsThisFrame.store(0, std::memory_order_relaxed);
    state.transmissionSegmentsThisFrame.store(0, std::memory_order_relaxed);

    const std::uint32_t raysPerPixel = std::max(state.settings.raysPerPixel, 1u);
    const float         rayBandMax  = std::min(state.settings.roughRayBandMax, state.settings.roughMaxTrace);
    const std::size_t   width       = state.fieldWidth;

    // Pass 1 - the stochastic ray pass. Screen-trace hits defer their
    // radiance: the ray records the texel it landed on (and the texel's row
    // list keeps the Fresnel-weighted contribution to apply afterwards).
    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            std::uint32_t pixelsDone = 0;
            std::uint32_t raysDone   = 0;
            std::uint32_t reusedDone = 0;
            std::uint32_t screenHits = 0;
            std::uint32_t sceneHits  = 0;
            std::uint32_t segments   = 0;
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                std::vector<RfScreenDeferred> &rowDeferred = state.screenDeferredRows[y];
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
                    // Governor interleave (update_budget_milliseconds): this
                    // frame's turn belongs to another texel - mark the reuse
                    // and skip the trace; the denoise pass's temporal path
                    // carries the reprojected history instead. The phase
                    // rotation guarantees every texel traces within `stride`
                    // frames. DYNAMIC surfaces (the animated water, flagged
                    // through the registry) are exempt: their content changes
                    // with the surface, not the camera, so reprojected
                    // history is a stale average of past wave phases - they
                    // trace every frame and the stride paces the static field
                    // alone.
                    const bool texelDynamic = state.gbufferSurface[index] != noSurface
                        && static_cast<std::size_t>(state.gbufferSurface[index]) < state.surfaces.size()
                        && state.surfaces[state.gbufferSurface[index]].dynamic;
                    if (traceStride > 1 && ((index + tracePhase) % traceStride) != 0 && !texelDynamic)
                    {
                        state.fieldReused[index] = 1u;
                        continue;
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
                    // The lobe antialiasing floor underneath: a near-mirror
                    // texel's traced direction would otherwise be near-
                    // deterministic, so the high-contrast scene it reflects
                    // aliases into single-texel outliers at field resolution.
                    const float alpha            = std::max(clampedRoughness * clampedRoughness, state.settings.lobeAlphaFloor);

                    // Per-texel constants shared by every ray of the texel:
                    // the tangent frame, the Fresnel base reflectivity, the
                    // screen-trace origin's clip position, and the texel's
                    // transmission pair (a transparent surface splits every
                    // ray into its reflected and transmitted shares).
                    const glm::mat3 basis     = makeBasis(normal);
                    const float     texelOpacity = glm::clamp(state.gbufferOpacity[index], 0.0f, 1.0f);
                    const float     texelIor     = glm::max(state.gbufferIor[index], 1.0f);
                    const glm::vec3 f0        = texelOpacity < 1.0f
                        ? glm::vec3(rfDielectricF0FromIor(1.0f, texelIor))
                        : glm::mix(glm::vec3(kDielectricF0), albedo, metallic);
                    const glm::vec3 rayOrigin = position + normal * state.settings.traceBias;
                    const glm::vec4 clipOrigin = state.viewProjection * glm::vec4(rayOrigin, 1.0f);
                    // The transmitted share leaves through the refracted
                    // direction (Snell with the air -> medium ratio); rough
                    // surfaces blur it, frosted glass reads blurry instead of
                    // clear. The refracted lobe's tangent frame is shared by
                    // every ray of the texel, so it is built once here (the
                    // split call only consumes it).
                    const glm::vec3 refractedView = texelOpacity < 1.0f
                        ? glm::normalize(glm::refract(-view, normal, 1.0f / texelIor))
                        : glm::vec3(0.0f);
                    const bool transmits = texelOpacity < 1.0f
                        && glm::dot(refractedView, refractedView) > 0.5f;
                    const glm::mat3 refractedBasis = transmits ? makeBasis(refractedView) : basis;

                    const bool mirrorBand     = roughness < rayBandMax;
                    // The near-mirror band supersamples: mirror and
                    // transmission content aliases at field resolution, so
                    // each texel integrates several sub-texel-jittered taps
                    // (see RfSettings::mirrorBandTaps) and the temporal
                    // accumulation converges the rejittered draws.
                    const std::uint32_t tapCount = mirrorBand
                        ? std::max(std::max(state.settings.mirrorBandTaps, 1u), raysPerPixel)
                        : raysPerPixel;
                    // The texel's world footprint from the right/below
                    // GBuffer neighbors, gated to the same surface plane (the
                    // spatial reuse's plane gate): the jitter spreads the taps
                    // across it. A neighbor that left the plane (silhouette,
                    // depth break) zeroes that axis, and a footprint of zero
                    // leaves the taps at the texel center where the VNDF lobe
                    // jitter alone spreads them.
                    glm::vec3 footprintU(0.0f);
                    glm::vec3 footprintV(0.0f);
                    if (tapCount > 1)
                    {
                        const auto sameSurfaceDelta = [&](std::size_t neighborIndex) -> glm::vec3
                        {
                            if (rfIsBackground(state.gbufferDepth[neighborIndex])
                                || state.gbufferUnlit[neighborIndex] != 0u)
                            {
                                return glm::vec3(0.0f);
                            }
                            const glm::vec3 delta = state.gbufferPosition[neighborIndex] - position;
                            const float     deltaLength = glm::length(delta);
                            if (deltaLength < 1.0e-6f
                                || std::fabs(glm::dot(normal, delta)) > 0.1f * deltaLength)
                            {
                                return glm::vec3(0.0f);
                            }
                            return delta;
                        };
                        if (x + 1u < state.fieldWidth)
                        {
                            footprintU = sameSurfaceDelta(index + 1u);
                        }
                        if (y + 1u < state.fieldHeight)
                        {
                            footprintV = sameSurfaceDelta(index + width);
                        }
                    }
                    const bool footprintUsable = glm::dot(footprintU, footprintU) > 0.0f
                        || glm::dot(footprintV, footprintV) > 0.0f;

                    std::uint64_t seed = hashSeed(state.frameIndex * 7919ull + index * 104729ull);
                    glm::vec3    sum(0.0f);
                    RfResolve   resolve = RfResolve::None;
                    std::uint32_t segmentsDone = 0;
                    for (std::uint32_t ray = 0; ray < tapCount; ++ray)
                    {
                        glm::vec3 tapPosition = position;
                        if (footprintUsable)
                        {
                            const float jitterX = randomFloat(seed) - 0.5f;
                            const float jitterY = randomFloat(seed) - 0.5f;
                            tapPosition = position + footprintU * jitterX + footprintV * jitterY;
                        }
                        glm::vec3 tapView = view;
                        if (footprintUsable)
                        {
                            const glm::vec3 tapDelta = state.cameraPosition - tapPosition;
                            const float     tapViewLength = glm::length(tapDelta);
                            if (tapViewLength > 1.0e-4f)
                            {
                                tapView = tapDelta / tapViewLength;
                            }
                        }
                        const glm::vec3 tapLocalView = glm::normalize(glm::transpose(basis) * tapView);
                        const glm::vec3 direction = rfSampleGgxVndf(basis, tapLocalView, alpha, seed);
                        const glm::vec3 fresnel  = rfFresnelWeight(tapView, direction, f0);
                        const glm::vec3 tapRayOrigin = tapPosition + normal * state.settings.traceBias;
                        const glm::vec4 tapClipOrigin = state.viewProjection * glm::vec4(tapRayOrigin, 1.0f);
                        if (mirrorBand)
                        {
                            // Dedicated reflection ray through the tracing
                            // tiers. A transparent texel splits the ray into
                            // its reflected share (deterministic, at its
                            // Fresnel weight) and a stochastic transmitted
                            // pick (rfSplitFirstSegment: the split estimator
                            // keeps the rare-bright-draw variance out of the
                            // running mean over nested media).
                            glm::vec3 chainWeight(0.0f);
                            glm::vec3 chainDirection(0.0f);
                            glm::vec3 chainOrigin(0.0f);
                            bool      transmitRuns = false;
                            glm::vec3 transmitWeight(0.0f);
                            glm::vec3 transmitDirection(0.0f);
                            glm::vec3 transmitOrigin(0.0f);
                            rfSplitFirstSegment(transmits, fresnel, texelOpacity, albedo,
                                direction, alpha, refractedBasis, tapPosition, tapRayOrigin,
                                state.settings.traceBias, state.settings.transmissionLobeAlphaFloor,
                                chainDirection, chainOrigin, chainWeight,
                                transmitRuns, transmitDirection, transmitOrigin, transmitWeight, seed);
                            // The reflected share resolves through the tracing
                            // tiers (the screen march starts from the
                            // normal-offset origin). A transparent texel's
                            // reflected share runs on scene traces directly:
                            // its near-mirror lobe resolves at content the
                            // 2 m screen march rarely reaches, so the march
                            // would spend its full step budget walking to a
                            // miss the scene trace answers exactly.
                            RfChainResolve chain = resolveRayChain(
                                state, tapClipOrigin, chainOrigin, chainDirection, chainWeight,
                                state.settings.maxTransmissionSegments, !transmits, seed);
                            segmentsDone += chain.segments;
                            for (std::uint32_t entry = 0; entry < chain.deferredCount; ++entry)
                            {
                                rowDeferred.push_back(RfScreenDeferred{static_cast<std::uint32_t>(index),
                                    chain.deferred[entry].hitTexel, chain.deferred[entry].tint});
                            }
                            sum += chain.radiance;
                            if (resolve == RfResolve::None)
                            {
                                resolve = chain.resolve;
                            }
                            if (chain.resolve == RfResolve::ScreenTrace)
                            {
                                ++screenHits;
                            }
                            else if (chain.resolve == RfResolve::SceneTrace)
                            {
                                ++sceneHits;
                            }
                            // The transmitted share starts inside the medium:
                            // scene traces only from there on.
                            if (transmitRuns)
                            {
                                const glm::vec4 transmitClip
                                    = state.viewProjection * glm::vec4(transmitOrigin, 1.0f);
                                RfChainResolve transmitChain = resolveRayChain(
                                    state, transmitClip, transmitOrigin, transmitDirection, transmitWeight,
                                    state.settings.maxTransmissionSegments, false, seed);
                                segmentsDone += transmitChain.segments;
                                for (std::uint32_t entry = 0; entry < transmitChain.deferredCount; ++entry)
                                {
                                    rowDeferred.push_back(RfScreenDeferred{static_cast<std::uint32_t>(index),
                                        transmitChain.deferred[entry].hitTexel, transmitChain.deferred[entry].tint});
                                }
                                sum += transmitChain.radiance;
                            }
                            ++raysDone;
                        }
                        else
                        {
                            // Rough band: resample the world radiance cache
                            // (Lumen's radiance-cache reuse); the analytic sky
                            // when no probe covers the position. The cache
                            // answer reads the UNJITTERED mirror direction:
                            // the SH2 evaluation is already the directional
                            // average the rough lobe needs, and a jittered
                            // lookup would freeze one draw's answer per texel
                            // when the static skip publishes the field - the
                            // per-texel draws read as speckle on exactly the
                            // surfaces (brushed metal, frosted glass) whose
                            // reflections should be smoothest. A transparent
                            // texel evaluates the reflected share off the
                            // mirror direction plus the transmitted share
                            // along the refracted direction, tinted by its
                            // albedo.
                            const glm::vec3 mirrorDirection = glm::normalize(glm::reflect(-view, normal));
                            const glm::vec3 roughFresnel = rfFresnelWeight(view, mirrorDirection, f0);
                            glm::vec3 radiance(0.0f);
                            if (!rfProbeRadiance(state, rayOrigin, mirrorDirection, radiance))
                            {
                                radiance = rfSkyRadiance(state, mirrorDirection);
                            }
                            // The emissive highlight the SH2 cache cannot
                            // carry: the probes smooth a small, bright emitter
                            // into their cell's average, so a rough lobe aimed
                            // at a lamp reads dim. The march along the jittered
                            // lobe sample captures the hit texel's emission
                            // (Lumen's screen traces run for every roughness
                            // band with the lobe footprint; the march-only
                            // capture is the CPU counterpart for the reuse
                            // band). The jitter spreads the capture across the
                            // lobe and the temporal accumulation converges it
                            // into the soft highlight a rough lobe shows.
                            if (sceneHasEmissive)
                            {
                                const ScreenTraceResult march = screenTrace(
                                    state, clipOrigin, rayOrigin, direction, state.settings.screenRayDistance);
                                if (march.hit && luminance(state.gbufferEmissive[march.texel]) > 0.0f)
                                {
                                    radiance += state.gbufferEmissive[march.texel];
                                }
                            }
                            if (transmits)
                            {
                                glm::vec3 transmittedRadiance(0.0f);
                                const glm::vec3 transmittedOrigin = position + refractedView * state.settings.traceBias;
                                if (!rfProbeRadiance(state, transmittedOrigin, refractedView, transmittedRadiance))
                                {
                                    transmittedRadiance = rfSkyRadiance(state, refractedView);
                                }
                                // The transmitted share's highlight: a lamp
                                // behind a frosted pane glows through it (the
                                // refracted lobe jittered by the same floor
                                // rule the smooth band's transmitted lobe
                                // uses).
                                if (sceneHasEmissive)
                                {
                                    const glm::vec3 transmittedDirection = rfSampleGgxVndf(refractedBasis,
                                        glm::vec3(0.0f, 0.0f, 1.0f), std::max(alpha, state.settings.transmissionLobeAlphaFloor), seed);
                                    const ScreenTraceResult transmittedMarch = screenTrace(state, clipOrigin,
                                        transmittedOrigin, transmittedDirection, state.settings.screenRayDistance);
                                    if (transmittedMarch.hit
                                        && luminance(state.gbufferEmissive[transmittedMarch.texel]) > 0.0f)
                                    {
                                        transmittedRadiance += state.gbufferEmissive[transmittedMarch.texel];
                                    }
                                }
                                const glm::vec3 transmittedShare
                                    = (glm::vec3(1.0f) - roughFresnel) * (1.0f - texelOpacity) * albedo;
                                sum += radiance * roughFresnel + transmittedRadiance * transmittedShare;
                                ++reusedDone;
                                ++segmentsDone;
                            }
                            else
                            {
                                sum += radiance * roughFresnel;
                                ++reusedDone;
                            }
                            resolve = RfResolve::Cache;
                        }
                    }
                    state.fieldRaw[index]    = sum / static_cast<float>(tapCount);
                    state.fieldValid[index]  = 1u;
                    state.fieldRayCount[index] = static_cast<float>(tapCount);
                    state.fieldResolve[index] = resolve;
                    segments += segmentsDone;
                    ++pixelsDone;
                }
            }
            state.pixelsTracedThisFrame.fetch_add(pixelsDone, std::memory_order_relaxed);
            state.raysTracedThisFrame.fetch_add(raysDone, std::memory_order_relaxed);
            state.raysReusedCacheThisFrame.fetch_add(reusedDone, std::memory_order_relaxed);
            state.screenTraceHitsThisFrame.fetch_add(screenHits, std::memory_order_relaxed);
            state.sceneTraceHitsThisFrame.fetch_add(sceneHits, std::memory_order_relaxed);
            state.transmissionSegmentsThisFrame.fetch_add(segments, std::memory_order_relaxed);
        });

    // Mark the texels the screen rays landed on (a single-threaded walk over
    // the deferred lists; several rays can land on the same texel).
    for (const std::vector<RfScreenDeferred> &row : state.screenDeferredRows)
    {
        for (const RfScreenDeferred &deferred : row)
        {
            state.screenNeedShade[deferred.hitTexel] = 1u;
        }
    }

    // Pass 2 - lazy screen shading: shade exactly the GBuffer texels the
    // screen traces hit (Lumen Scene lighting on demand). Unlit texels read
    // as their albedo, as the full-field shading did.
    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    if (state.screenNeedShade[index] == 0u)
                    {
                        continue;
                    }
                    if (state.gbufferUnlit[index] != 0u)
                    {
                        state.screenColor[index] = state.gbufferAlbedo[index];
                        continue;
                    }
                    // The lazy shade answers with the texel's own response as
                    // seen from the camera: the specular block samples the
                    // cache along the camera's mirror (the landing rays come
                    // from elsewhere, but the cache's directional blur makes
                    // the camera mirror the right neighborhood).
                    state.screenColor[index] = rfShadeHitPoint(state,
                        state.gbufferPosition[index],
                        state.gbufferNormal[index],
                        state.gbufferAlbedo[index],
                        state.gbufferEmissive[index],
                        glm::normalize(state.cameraPosition - state.gbufferPosition[index]),
                        state.gbufferRoughness[index],
                        state.gbufferMetallic[index]);
                }
            }
        });

    // Pass 3 - apply the deferred screen-hit contributions. The deferred
    // tint carries the chain's full weight (source-interface Fresnel or
    // transmitted share, plus every transmission tint along the way). The
    // normalization reads the per-texel tap count (fieldRayCount: the
    // near-mirror band traces more taps than raysPerPixel).
    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (const RfScreenDeferred &deferred : state.screenDeferredRows[y])
                {
                    const float rayCount = state.fieldRayCount[deferred.sourceTexel];
                    const float inverseRayCount = rayCount > 0.0f ? 1.0f / rayCount : 1.0f;
                    state.fieldRaw[deferred.sourceTexel]
                        += state.screenColor[deferred.hitTexel] * deferred.tint * inverseRayCount;
                }
            }
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
        rfBilateralFilter(state, scheduler);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace reflection_field
} // namespace engine
