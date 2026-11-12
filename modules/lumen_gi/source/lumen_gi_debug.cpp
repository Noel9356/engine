// lumen_gi debug side: the module's GBuffer rasterizer and the screen-space
// final gather (Lumen's opaque Final Gather scaled to a CPU view).
//
// The final gather follows Lumen's three tiers: screen probes on a jittered
// sub-resolution grid trace a few rays each (stratified over the upper
// hemisphere of an 8x8 octahedral map, importance-selected by the probe's
// previous radiance), rays shorter than gather.screen_ray_distance_m resolve
// the rest from the world-space radiance cache on miss, and the
// full-resolution pass interpolates probe SH2 within the depth plane, then
// applies temporal accumulation and a bilateral spatial filter.
//
// None of this feeds the renderer (that consumes the world probe grid), so the
// whole path runs on demand - when a debug view is exported, or every frame
// only with debug.continuous_gather - instead of costing CPU on every
// evaluate().

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace engine
{
namespace lumen_gi
{

namespace
{

constexpr float backgroundDepth = std::numeric_limits<float>::max();

bool isBackground(float depth)
{
    return depth >= backgroundDepth * 0.5f;
}

// ---- GBuffer raster -------------------------------------------------------------

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

// Clip distance (clip-space w). Clipping closer than this projects vertices to
// enormous pixel coordinates and the edge functions lose all precision.
constexpr float nearW = 0.05f;
constexpr std::uint32_t bandRows = 16;

bool surfaceOutsideFrustum(const glm::mat4 &viewProjection, const GiSurface &surface)
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

void setupSurfaceTriangles(const GiPipelineState &state, std::uint32_t surfaceIndex, std::uint32_t width,
    std::uint32_t height, std::vector<ScreenTriangle> &out)
{
    const GiSurface &surface = state.surfaces[surfaceIndex];
    if (surface.excluded || surfaceOutsideFrustum(state.viewProjection, surface))
    {
        return;
    }
    const GiSurfaceGeometry &geometry = surface.geometry;
    // Transform every vertex once (not once per triangle corner).
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
        // Sutherland-Hodgman against the near plane w = nearW: triangles that
        // cross it are clipped instead of discarded.
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

void rasterizeBand(GiPipelineState &state, const std::vector<ScreenTriangle> &triangles,
    const std::vector<std::uint32_t> &binned, std::uint32_t rowBegin, std::uint32_t rowEnd)
{
    const std::uint32_t width = state.viewWidth;
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
        const GiSurface         &surface  = state.surfaces[screen.surface];
        const GiSurfaceGeometry &geometry = surface.geometry;
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
                // Perspective-correct weights: screen-space barycentrics scaled
                // by 1/w and renormalized (interpolating world position or
                // normals linearly in screen space, as before, bends them).
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
                if (distance >= state.viewDepth[pixelIndex])
                {
                    continue;
                }
                const glm::vec3 bary = screen.vertex[0].bary * mu0 + screen.vertex[1].bary * mu1
                                     + screen.vertex[2].bary * mu2;
                glm::vec3 localNormal = geometry.normals[i0] * bary.x + geometry.normals[i1] * bary.y
                                      + geometry.normals[i2] * bary.z;
                const float normalLength = glm::length(localNormal);
                localNormal = normalLength > 1.0e-8f ? localNormal / normalLength : glm::vec3(0.0f, 1.0f, 0.0f);
                state.viewDepth[pixelIndex]         = distance;
                state.viewWorldPosition[pixelIndex] = worldPosition;
                state.viewSurfaceIndex[pixelIndex]  = screen.surface;
                state.viewNormal[pixelIndex]        = glm::normalize(surface.normalMatrix * localNormal);
                state.viewAlbedo[pixelIndex] = surface.albedo
                    * (geometry.colors[i0] * bary.x + geometry.colors[i1] * bary.y + geometry.colors[i2] * bary.z);
                state.viewEmissive[pixelIndex] = surface.emissive;
                state.viewFlags[pixelIndex]    = static_cast<std::uint8_t>(surface.unlit ? 1u : 0u);
            }
        }
    }
}

void rasterizeViewGBuffer(GiPipelineState &state, TaskScheduler *scheduler)
{
    const std::uint32_t width  = std::max(state.settings.debugInternalWidth, 16u);
    const std::uint32_t height = std::max(state.settings.debugInternalHeight, 16u);
    state.viewWidth  = width;
    state.viewHeight = height;
    const std::size_t pixelCount = static_cast<std::size_t>(width) * height;
    state.viewDepth.assign(pixelCount, backgroundDepth);
    state.viewNormal.assign(pixelCount, glm::vec3(0.0f));
    state.viewWorldPosition.assign(pixelCount, glm::vec3(0.0f));
    state.viewAlbedo.assign(pixelCount, glm::vec3(0.0f));
    state.viewEmissive.assign(pixelCount, glm::vec3(0.0f));
    state.viewFlags.assign(pixelCount, 0u);
    state.viewSurfaceIndex.assign(pixelCount, noSurface);

    // Phase 1: per-surface triangle setup (parallel over surfaces).
    std::vector<std::vector<ScreenTriangle>> perSurface(state.surfaces.size());
    parallelRange(scheduler, state.surfaces.size(), 1,
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
    // Phase 2: bin triangles into row bands (kept in submission order).
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
    // Phase 3: bands are disjoint rows, so they rasterize in parallel.
    parallelRange(scheduler, bandCount, 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t band = begin; band < end; ++band)
            {
                const std::uint32_t rowBegin = static_cast<std::uint32_t>(band) * bandRows;
                rasterizeBand(state, triangles, bins[band], rowBegin, std::min(rowBegin + bandRows, height));
            }
        });
}

// ---- screen-space final gather ---------------------------------------------------

// Resolves one gather ray: short surface-cache trace first, then the world
// cache (level 0, then the coarser level 1), then the skylight.
glm::vec3 gatherRayRadiance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &direction)
{
    const GiHit hit = traceScene(state, GiRay{position, direction, std::max(state.settings.screenRayDistance, 0.05f)});
    if (!hit.hit)
    {
        glm::vec3 value(0.0f);
        if (worldCacheLevelRadiance(state, state.level0, position, direction, value)
            || worldCacheLevelRadiance(state, state.level1, position, direction, value))
        {
            return value;
        }
        // No usable cache here (outside both levels, or every surrounding
        // probe invalid): continue the ray through the scene instead of
        // dropping its energy, then fall back to the skylight.
        const float shortDistance = std::max(state.settings.screenRayDistance, 0.05f);
        const GiHit far = traceScene(state, GiRay{position + direction * shortDistance, direction,
            std::max(state.settings.maxTraceDistance - shortDistance, 0.0f)});
        if (!far.hit)
        {
            return state.environment.skyRadiance(direction);
        }
        glm::vec3 farValue(0.0f);
        radianceAtHit(state, far, true, farValue);
        return farValue;
    }
    glm::vec3 value(0.0f);
    radianceAtHit(state, hit, true, value); // back-face hits return nothing
    return value;
}

void traceScreenProbes(GiPipelineState &state, TaskScheduler *scheduler)
{
    const std::uint32_t width  = state.viewWidth;
    const std::uint32_t height = state.viewHeight;
    const std::uint32_t scale  = std::max(state.settings.gatherScreenScale, 1u);
    const std::uint32_t probeWidth  = std::max((width + scale - 1u) / scale, 1u);
    const std::uint32_t probeHeight = std::max((height + scale - 1u) / scale, 1u);
    const std::size_t   directionCount = state.screenProbeDirections.size();
    const std::size_t   probeCount  = static_cast<std::size_t>(probeWidth) * probeHeight;
    if (state.screenProbeRadiance.size() != probeCount * directionCount)
    {
        state.screenProbeRadiance.assign(probeCount * directionCount, glm::vec3(0.0f));
        state.screenProbePosition.assign(probeCount, glm::vec3(1.0e30f));
    }
    state.screenProbePixel.assign(probeCount, 0u);
    state.screenProbeSh2.assign(probeCount * 27, 0.0f);
    state.screenProbeValid.assign(probeCount, 0u);
    state.screenProbeWidth  = probeWidth;
    state.screenProbeHeight = probeHeight;

    const std::uint32_t rayCount = std::max(state.settings.gatherRaysPerProbe, 1u);
    parallelRange(scheduler, probeHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            std::uint32_t probesDone = 0;
            std::uint32_t raysDone   = 0;
            std::vector<std::uint32_t> candidates;
            std::vector<float>         luminance;
            for (std::size_t probeY = rowBegin; probeY < rowEnd; ++probeY)
            {
                for (std::uint32_t probeX = 0; probeX < probeWidth; ++probeX)
                {
                    const std::size_t probeIndex = probeY * probeWidth + probeX;
                    // Jittered placement inside the scale x scale cell.
                    std::uint64_t seed = hashSeed(state.gatherFrame * 7919ull + probeIndex);
                    const float   jitterX = randomFloat(seed);
                    const float   jitterY = randomFloat(seed);
                    const std::uint32_t pixelX = std::min(probeX * scale + static_cast<std::uint32_t>(jitterX * scale), width - 1u);
                    const std::uint32_t pixelY = std::min(static_cast<std::uint32_t>(probeY) * scale
                            + static_cast<std::uint32_t>(jitterY * scale), height - 1u);
                    const std::size_t pixelIndex = static_cast<std::size_t>(pixelY) * width + pixelX;
                    if (isBackground(state.viewDepth[pixelIndex]))
                    {
                        continue; // background: no probe
                    }
                    const glm::vec3 position = state.viewWorldPosition[pixelIndex];
                    const glm::vec3 normal   = state.viewNormal[pixelIndex];
                    glm::vec3 *     radiance = &state.screenProbeRadiance[probeIndex * directionCount];
                    state.screenProbeValid[probeIndex] = 1u;
                    state.screenProbePixel[probeIndex] = static_cast<std::uint32_t>(pixelIndex);
                    ++probesDone;

                    // The persistent map describes the place the probe last
                    // traced from; once the placement moved on (camera motion,
                    // different surface) the old radiance is a stale importance
                    // hint and a wrong accumulation seed, so start over.
                    const float resetDistance = std::max(0.5f, 0.08f * state.viewDepth[pixelIndex]);
                    if (glm::distance(position, state.screenProbePosition[probeIndex]) > resetDistance)
                    {
                        std::fill(radiance, radiance + directionCount, glm::vec3(0.0f));
                    }
                    state.screenProbePosition[probeIndex] = position;

                    // Candidate strata: the texels of the 8x8 map whose center
                    // lies above the surface (the lower half sees only the
                    // surface itself).
                    candidates.clear();
                    luminance.clear();
                    float luminanceTotal = 0.0f;
                    for (std::uint32_t texel = 0; texel < directionCount; ++texel)
                    {
                        if (glm::dot(state.screenProbeDirections[texel], normal) > 0.0f)
                        {
                            candidates.push_back(texel);
                            const glm::vec3 &value = radiance[texel];
                            const float l = 0.2126f * value.r + 0.7152f * value.g + 0.0722f * value.b;
                            luminance.push_back(l);
                            luminanceTotal += l;
                        }
                    }
                    if (candidates.empty())
                    {
                        continue;
                    }
                    const bool useImportance = state.settings.gatherImportanceSampling && luminanceTotal > 1.0e-5f;
                    for (std::uint32_t ray = 0; ray < rayCount; ++ray)
                    {
                        std::size_t selected = candidates[(state.gatherFrame * rayCount + ray) % candidates.size()];
                        if (useImportance && randomFloat(seed) < 0.5f)
                        {
                            // Inverse-transform sampling over previous radiance.
                            const float pick = randomFloat(seed) * luminanceTotal;
                            float       accumulated = 0.0f;
                            for (std::size_t candidate = 0; candidate < candidates.size(); ++candidate)
                            {
                                accumulated += luminance[candidate];
                                if (accumulated >= pick)
                                {
                                    selected = candidates[candidate];
                                    break;
                                }
                            }
                        }
                        // A direction jittered inside the selected texel, so the
                        // traced direction and the texel it is stored into agree.
                        const float u = (static_cast<float>(selected % 8) + randomFloat(seed)) / 8.0f * 2.0f - 1.0f;
                        const float v = (static_cast<float>(selected / 8) + randomFloat(seed)) / 8.0f * 2.0f - 1.0f;
                        glm::vec3 direction = octahedralDecode(u, v);
                        const float facing = glm::dot(direction, normal);
                        if (facing < 0.02f)
                        {
                            direction = glm::normalize(direction - normal * (2.0f * facing - 0.05f));
                        }
                        ++raysDone;
                        const glm::vec3 value = gatherRayRadiance(state, position + normal * 0.02f, direction);
                        glm::vec3 &target = radiance[octahedralTexelIndex(direction, 8)];
                        // An untouched texel takes the fresh sample whole;
                        // touched ones blend (accumulation without first-frame bias).
                        target = glm::dot(target, target) > 0.0f ? glm::mix(value, target, 0.35f) : value;
                    }
                    projectRadianceToSh2(state.screenProbeDirections.data(), radiance,
                        state.screenProbeDirectionWeights.data(), directionCount, &state.screenProbeSh2[probeIndex * 27]);
                }
            }
            state.screenProbesThisFrame.fetch_add(probesDone, std::memory_order_relaxed);
            state.gatherRaysThisFrame.fetch_add(raysDone, std::memory_order_relaxed);
        });
}

// Full-resolution pass: interpolate probe SH2 within the depth plane (Lumen
// interpolates "to pixels within the same plane"), then temporal accumulation
// against the reprojection of the previous frame and a bilateral spatial
// filter. Writes gatherIrradiance per pixel (indirect irradiance E).
void resolvePixelIrradiance(GiPipelineState &state, TaskScheduler *scheduler)
{
    const std::uint32_t width  = state.viewWidth;
    const std::uint32_t height = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    state.gatherIrradiancePrevious.swap(state.gatherIrradiance);
    if (state.gatherIrradiance.size() != pixelCount)
    {
        state.gatherIrradiance.assign(pixelCount, glm::vec3(0.0f));
    }
    if (state.gatherIrradiancePrevious.size() != pixelCount)
    {
        state.gatherIrradiancePrevious.assign(pixelCount, glm::vec3(0.0f));
    }
    const std::uint32_t scale = std::max(state.settings.gatherScreenScale, 1u);
    const bool      temporalEnabled = state.gatherHasHistory && state.gatherPreviousWidth == width
                                   && state.gatherPreviousHeight == height
                                   && state.gatherPreviousDepth.size() == pixelCount;
    if (!temporalEnabled)
    {
        state.gatherHistoryLength = 0;
    }
    // History ramp-up: with n accumulated passes the history weight is at most
    // n / (n + 1), so the first passes average instead of fading in from black.
    const float historyRamp = static_cast<float>(state.gatherHistoryLength) / static_cast<float>(state.gatherHistoryLength + 1);
    const float temporalBlend = std::min(std::clamp(state.settings.gatherTemporalBlend, 0.0f, 0.99f), historyRamp);
    const glm::mat4 previousViewProjection = state.gatherPreviousViewProjection;
    parallelRange(scheduler, height, 4,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < width; ++x)
                {
                    const std::size_t pixelIndex = y * width + x;
                    const float       depth      = state.viewDepth[pixelIndex];
                    if (isBackground(depth))
                    {
                        state.gatherIrradiance[pixelIndex] = glm::vec3(0.0f);
                        continue;
                    }
                    const glm::vec3 pixelNormal = state.viewNormal[pixelIndex];
                    const float     normalArray[3] = {pixelNormal.x, pixelNormal.y, pixelNormal.z};
                    const std::int32_t cellX = static_cast<std::int32_t>(x / scale);
                    const std::int32_t cellY = static_cast<std::int32_t>(y / scale);
                    glm::vec3 interpolated(0.0f);
                    float     weightSum = 0.0f;
                    for (std::int32_t dy = -1; dy <= 1; ++dy)
                    {
                        for (std::int32_t dx = -1; dx <= 1; ++dx)
                        {
                            const std::int32_t px = cellX + dx;
                            const std::int32_t py = cellY + dy;
                            if (px < 0 || py < 0 || px >= static_cast<std::int32_t>(state.screenProbeWidth)
                                || py >= static_cast<std::int32_t>(state.screenProbeHeight))
                            {
                                continue;
                            }
                            const std::size_t probeIndex = static_cast<std::size_t>(py) * state.screenProbeWidth + px;
                            if (state.screenProbeValid[probeIndex] == 0u)
                            {
                                continue;
                            }
                            // Plane-aware weight against the probe's ACTUAL
                            // (jittered) placement.
                            const std::uint32_t probePixel = state.screenProbePixel[probeIndex];
                            const float probeDepth = state.viewDepth[probePixel];
                            if (std::fabs(probeDepth - depth) > state.settings.gatherPlaneDepthTolerance * (depth + 1.0f)
                                || glm::dot(state.viewNormal[probePixel], pixelNormal) < 0.5f)
                            {
                                continue;
                            }
                            const float probeX = static_cast<float>(probePixel % width);
                            const float probeY = static_cast<float>(probePixel / width);
                            const float screenDistance = std::sqrt((probeX - static_cast<float>(x)) * (probeX - static_cast<float>(x))
                                + (probeY - static_cast<float>(y)) * (probeY - static_cast<float>(y)))
                                / static_cast<float>(scale);
                            const float weight = 1.0f / (1.0f + screenDistance * screenDistance * 1.5f
                                + std::fabs(probeDepth - depth) / (depth + 1.0f));
                            float irradiance[3];
                            evaluateSh2Irradiance(&state.screenProbeSh2[probeIndex * 27], normalArray, irradiance);
                            interpolated += glm::vec3(irradiance[0], irradiance[1], irradiance[2]) * weight;
                            weightSum += weight;
                        }
                    }
                    glm::vec3 irradiance = weightSum > 1.0e-6f ? interpolated / weightSum : glm::vec3(0.0f);
                    // Temporal accumulation: reproject into the previous frame.
                    if (temporalEnabled)
                    {
                        const glm::vec4 previousClip = previousViewProjection
                            * glm::vec4(state.viewWorldPosition[pixelIndex], 1.0f);
                        if (previousClip.w > 0.0f)
                        {
                            const float previousX = (previousClip.x / previousClip.w * 0.5f + 0.5f) * static_cast<float>(width) - 0.5f;
                            const float previousY = (0.5f - previousClip.y / previousClip.w * 0.5f) * static_cast<float>(height) - 0.5f;
                            if (previousX >= 0.0f && previousY >= 0.0f && previousX < static_cast<float>(width - 1)
                                && previousY < static_cast<float>(height - 1))
                            {
                                const std::size_t previousIndex = static_cast<std::size_t>(previousY) * width
                                    + static_cast<std::size_t>(previousX);
                                const float previousDepth = state.gatherPreviousDepth[previousIndex];
                                const float depthChange   = std::fabs(previousDepth - depth) / (depth + 1.0f);
                                if (!isBackground(previousDepth) && depthChange < 0.2f)
                                {
                                    irradiance = glm::mix(irradiance, state.gatherIrradiancePrevious[previousIndex], temporalBlend);
                                }
                            }
                        }
                    }
                    state.gatherIrradiance[pixelIndex] = irradiance;
                }
            }
        });
    // Bilateral spatial filter guided by depth and normal.
    if (state.settings.gatherSpatialFilter > 0)
    {
        std::vector<glm::vec3> filtered(pixelCount, glm::vec3(0.0f));
        const std::int32_t radius = static_cast<std::int32_t>(std::min(state.settings.gatherSpatialFilter / 2u, 2u));
        parallelRange(scheduler, height, 4,
            [&](std::size_t rowBegin, std::size_t rowEnd)
            {
                for (std::size_t y = rowBegin; y < rowEnd; ++y)
                {
                    for (std::uint32_t x = 0; x < width; ++x)
                    {
                        const std::size_t pixelIndex = y * width + x;
                        if (isBackground(state.viewDepth[pixelIndex]))
                        {
                            continue;
                        }
                        glm::vec3 sum(0.0f);
                        float     weightSum = 0.0f;
                        const glm::vec3 centerNormal = state.viewNormal[pixelIndex];
                        const float     centerDepth  = state.viewDepth[pixelIndex];
                        for (std::int32_t dy = -radius; dy <= radius; ++dy)
                        {
                            for (std::int32_t dx = -radius; dx <= radius; ++dx)
                            {
                                const std::int32_t sx = static_cast<std::int32_t>(x) + dx;
                                const std::int32_t sy = static_cast<std::int32_t>(y) + dy;
                                if (sx < 0 || sy < 0 || sx >= static_cast<std::int32_t>(width)
                                    || sy >= static_cast<std::int32_t>(height))
                                {
                                    continue;
                                }
                                const std::size_t sampleIndex = static_cast<std::size_t>(sy) * width + sx;
                                if (isBackground(state.viewDepth[sampleIndex]))
                                {
                                    continue;
                                }
                                const float depthWeight  = std::exp(-std::fabs(state.viewDepth[sampleIndex] - centerDepth)
                                    / (0.05f * centerDepth + 0.05f));
                                const float normalWeight = std::max(glm::dot(state.viewNormal[sampleIndex], centerNormal), 0.0f);
                                const float weight       = depthWeight * normalWeight;
                                sum += state.gatherIrradiance[sampleIndex] * weight;
                                weightSum += weight;
                            }
                        }
                        filtered[pixelIndex] = weightSum > 1.0e-6f ? sum / weightSum : state.gatherIrradiance[pixelIndex];
                    }
                }
            });
        state.gatherIrradiance.swap(filtered);
    }
}

} // namespace

void prepareDebugBuffers(GiPipelineState &state, TaskScheduler *scheduler, std::uint32_t gatherPasses)
{
    rasterizeViewGBuffer(state, scheduler);
    for (std::uint32_t pass = 0; pass < gatherPasses; ++pass)
    {
        traceScreenProbes(state, scheduler);
        resolvePixelIrradiance(state, scheduler);
        state.gatherPreviousViewProjection = state.viewProjection;
        state.gatherPreviousWidth          = state.viewWidth;
        state.gatherPreviousHeight         = state.viewHeight;
        state.gatherPreviousDepth          = state.viewDepth;
        state.gatherHasHistory             = true;
        ++state.gatherHistoryLength;
        ++state.gatherFrame;
    }
}

} // namespace lumen_gi
} // namespace engine
