// lumen_gi debug side: the screen-space final gather (Lumen's opaque Final
// Gather scaled to a CPU view), the debug view compositor, the reference
// path tracer used to validate the pipeline, and the PPM writer.
//
// The final gather follows Lumen's three tiers: screen probes placed on a
// jittered sub-resolution grid trace a few rays each (stratified over the
// octahedral map, importance-selected by the previous frame's radiance - the
// Screen Space Radiance Cache trick), rays shorter than
// gather.screen_ray_distance_m resolve the rest from the world-space radiance
// cache on miss, and the full-resolution pass interpolates probe SH2 within
// the depth plane, then applies temporal accumulation and a bilateral spatial
// filter. The gather feeds the module's own debug images; the renderer-facing
// result of the pipeline is the world probe grid.

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cstdio>

namespace engine
{
namespace lumen_gi
{

namespace
{

constexpr float kPi = 3.141592654f;

std::uint64_t hashSeed(std::uint64_t seed)
{
    std::uint64_t hash = seed * 0x9E3779B97F4A7C15ull;
    hash ^= hash >> 30;
    hash *= 0xBF58476D1CE4E5B9ull;
    hash ^= hash >> 27;
    hash *= 0x94D049BB133111EBull;
    hash ^= hash >> 31;
    return hash;
}

float randomFloat(std::uint64_t &seed)
{
    seed = hashSeed(seed);
    return static_cast<float>(seed & 0xFFFFFFull) / 16777216.0f;
}

glm::vec3 cosineDirectionSeeded(std::uint64_t &seed)
{
    const float u      = randomFloat(seed);
    const float v      = randomFloat(seed);
    const float radius = std::sqrt(std::max(0.0f, u));
    const float phi    = v * 2.0f * kPi;
    return glm::vec3(radius * std::cos(phi), radius * std::sin(phi), std::sqrt(std::max(0.0f, 1.0f - u)));
}

glm::mat3 onb(const glm::vec3 &normal)
{
    const glm::vec3 axis      = std::fabs(normal.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 tangent   = glm::normalize(glm::cross(axis, normal));
    const glm::vec3 bitangent = glm::cross(normal, tangent);
    return glm::mat3(tangent, bitangent, normal);
}

[[maybe_unused]] glm::vec3 toneMapColor(glm::vec3 color, float exposure, pbr::ToneMapping mapper)
{
    color *= exposure;
    switch (mapper)
    {
    case pbr::ToneMapping::None:
        break;
    case pbr::ToneMapping::Reinhard:
        color = color / (1.0f + color);
        break;
    case pbr::ToneMapping::AcesFilmic:
    default:
        color = glm::clamp((color * (2.51f * color + 0.03f)) / (color * (2.43f * color + 0.59f) + 0.14f), 0.0f, 1.0f);
        break;
    }
    return color;
}

[[maybe_unused]] std::uint8_t toByte(float value)
{
    return static_cast<std::uint8_t>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
}

[[maybe_unused]] glm::vec3 gammaEncode(glm::vec3 color)
{
    return glm::pow(glm::max(color, glm::vec3(0.0f)), glm::vec3(1.0f / 2.2f));
}

[[maybe_unused]] bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgb)
{
    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        return false;
    }
    std::fprintf(file, "P6\n%u %u\n255\n", width, height);
    const std::size_t byteCount = static_cast<std::size_t>(width) * height * 3;
    if (std::fwrite(rgb, 1, byteCount, file) != byteCount)
    {
        std::fclose(file);
        return false;
    }
    std::fclose(file);
    return true;
}

// Interpolates the world-space radiance cache at a direction: trilinear
// blend of the surrounding probes' octahedral texel values weighted by their
// validity (Lumen shortens screen probe rays and interpolates the world cache
// on miss; empty probes fall through to the caller's skylight).
glm::vec3 worldCacheRadiance(const GiPipelineState &state, const GiPipelineState::ProbeLevel &level,
    const glm::vec3 &position, const glm::vec3 &direction, bool &outAnyValid)
{
    outAnyValid = false;
    const std::uint32_t dimension = level.dimensions[0];
    const glm::vec3     cell      = (position - level.anchor) / level.spacing - 0.5f;
    const glm::vec3     base      = glm::clamp(glm::floor(cell), glm::vec3(0.0f),
        glm::vec3(static_cast<float>(dimension - 2)));
    const glm::vec3     fraction  = glm::clamp(cell - base, glm::vec3(0.0f), glm::vec3(1.0f));
    // Octahedral texel of the direction (same mapping as the direction table).
    const glm::vec3     absolute  = glm::abs(direction);
    glm::vec2           octahedral;
    if (absolute.z >= absolute.x && absolute.z >= absolute.y)
    {
        octahedral = glm::vec2(direction.x, direction.y) / std::max(absolute.z, 1.0e-6f);
    }
    else if (absolute.x >= absolute.y)
    {
        octahedral = glm::vec2(direction.z, direction.y) / std::max(absolute.x, 1.0e-6f);
    }
    else
    {
        octahedral = glm::vec2(direction.x, direction.z) / std::max(absolute.y, 1.0e-6f);
    }
    const std::uint32_t resolution = state.settings.probeDirectionResolution;
    const std::int32_t tx = std::clamp(
        static_cast<std::int32_t>(std::lround(0.5f * (octahedral.x + 1.0f) * static_cast<float>(resolution - 1))), 0,
        static_cast<std::int32_t>(resolution) - 1);
    const std::int32_t ty = std::clamp(
        static_cast<std::int32_t>(std::lround(0.5f * (octahedral.y + 1.0f) * static_cast<float>(resolution - 1))), 0,
        static_cast<std::int32_t>(resolution) - 1);
    glm::vec3 sum(0.0f);
    float     weightSum = 0.0f;
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const std::uint32_t cx = static_cast<std::uint32_t>(base.x) + (corner & 1u);
        const std::uint32_t cy = static_cast<std::uint32_t>(base.y) + ((corner >> 1) & 1u);
        const std::uint32_t cz = static_cast<std::uint32_t>(base.z) + ((corner >> 2) & 1u);
        const float         wx = (corner & 1u) != 0u ? fraction.x : 1.0f - fraction.x;
        const float         wy = ((corner >> 1) & 1u) != 0u ? fraction.y : 1.0f - fraction.y;
        const float         wz = ((corner >> 2) & 1u) != 0u ? fraction.z : 1.0f - fraction.z;
        const float         weight = wx * wy * wz;
        if (weight <= 0.0f)
        {
            continue;
        }
        const GiPipelineState::ProbeLevel::Probe &probe =
            level.probes[(static_cast<std::size_t>(cz) * dimension + cy) * dimension + cx];
        if (!probe.valid || probe.validity <= 0.0f)
        {
            continue;
        }
        outAnyValid = true;
        sum += probe.radiance[static_cast<std::size_t>(ty) * resolution + tx] * weight * probe.validity;
        weightSum += weight * probe.validity;
    }
    return weightSum > 1.0e-6f ? sum / weightSum : glm::vec3(0.0f);
}

// Resolves one gather ray: short surface-cache trace first, then the world
// cache (level 0, then the coarser level 1), then the skylight.
glm::vec3 gatherRayRadiance(GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &direction)
{
    GiRay       traceRay{position, direction, std::max(state.settings.screenRayDistance, 0.05f)};
    const GiHit hit = traceScene(state, traceRay);
    if (!hit.hit)
    {
        bool anyValid = false;
        glm::vec3 value = worldCacheRadiance(state, state.level0, position, direction, anyValid);
        if (!anyValid)
        {
            value = worldCacheRadiance(state, state.level1, position, direction, anyValid);
        }
        return anyValid ? value : state.environment.skyRadiance(direction);
    }
    if (glm::dot(hit.geometricNormal, direction) > 0.0f)
    {
        return glm::vec3(0.0f); // backface hit: started inside geometry
    }
    glm::vec3 value(0.0f);
    radianceAtHit(state, hit, true, value);
    return value;
}

} // namespace

// ---- GBuffer raster (module debug view) --------------------------------------

void rasterizeViewGBuffer(GiPipelineState &state)
{
    const std::uint32_t width  = std::max(state.settings.debugInternalWidth, 16u);
    const std::uint32_t height = std::max(state.settings.debugInternalHeight, 16u);
    state.viewWidth  = width;
    state.viewHeight = height;
    const std::size_t pixelCount = static_cast<std::size_t>(width) * height;
    state.viewDepth.assign(pixelCount, std::numeric_limits<float>::max());
    state.viewNormal.assign(pixelCount, glm::vec3(0.0f));
    state.viewWorldPosition.assign(pixelCount, glm::vec3(0.0f));
    state.viewAlbedo.assign(pixelCount, glm::vec3(0.0f));
    state.viewEmissive.assign(pixelCount, glm::vec3(0.0f));
    state.viewFlags.assign(pixelCount, 0u);
    state.viewSurfaceIndex.assign(pixelCount, 0xffffffffu);

    const glm::mat4 viewProjection = state.viewProjection;
    for (std::uint32_t surfaceIndex = 0; surfaceIndex < state.surfaces.size(); ++surfaceIndex)
    {
        const GiSurface &      surface = state.surfaces[surfaceIndex];
        if (surface.excluded)
        {
            continue;
        }
        const GiSurfaceGeometry &geometry      = surface.geometry;
        const std::uint32_t      triangleCount = static_cast<std::uint32_t>(geometry.indices.size() / 3);
        for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const glm::vec3 p0 = glm::vec3(
                surface.transform * glm::vec4(geometry.positions[geometry.indices[triangle * 3 + 0]], 1.0f));
            const glm::vec3 p1 = glm::vec3(
                surface.transform * glm::vec4(geometry.positions[geometry.indices[triangle * 3 + 1]], 1.0f));
            const glm::vec3 p2 = glm::vec3(
                surface.transform * glm::vec4(geometry.positions[geometry.indices[triangle * 3 + 2]], 1.0f));
            const glm::vec4 c0 = viewProjection * glm::vec4(p0, 1.0f);
            const glm::vec4 c1 = viewProjection * glm::vec4(p1, 1.0f);
            const glm::vec4 c2 = viewProjection * glm::vec4(p2, 1.0f);
            if (c0.w <= 0.0f || c1.w <= 0.0f || c2.w <= 0.0f)
            {
                continue;
            }
            const glm::vec2 pixelA((c0.x / c0.w * 0.5f + 0.5f) * static_cast<float>(width),
                (0.5f - c0.y / c0.w * 0.5f) * static_cast<float>(height));
            const glm::vec2 pixelB((c1.x / c1.w * 0.5f + 0.5f) * static_cast<float>(width),
                (0.5f - c1.y / c1.w * 0.5f) * static_cast<float>(height));
            const glm::vec2 pixelC((c2.x / c2.w * 0.5f + 0.5f) * static_cast<float>(width),
                (0.5f - c2.y / c2.w * 0.5f) * static_cast<float>(height));
            const glm::vec2 boundsMin = glm::min(pixelA, glm::min(pixelB, pixelC));
            const glm::vec2 boundsMax = glm::max(pixelA, glm::max(pixelB, pixelC));
            const std::int32_t xBegin = std::max(static_cast<std::int32_t>(std::floor(boundsMin.x)), 0);
            const std::int32_t yBegin = std::max(static_cast<std::int32_t>(std::floor(boundsMin.y)), 0);
            const std::int32_t xEnd   = std::min(static_cast<std::int32_t>(std::ceil(boundsMax.x)) + 1,
                static_cast<std::int32_t>(width));
            const std::int32_t yEnd   = std::min(static_cast<std::int32_t>(std::ceil(boundsMax.y)) + 1,
                static_cast<std::int32_t>(height));
            if (xBegin >= xEnd || yBegin >= yEnd)
            {
                continue;
            }
            const glm::vec3 &n0 = geometry.normals[geometry.indices[triangle * 3 + 0]];
            const glm::vec3 &n1 = geometry.normals[geometry.indices[triangle * 3 + 1]];
            const glm::vec3 &n2 = geometry.normals[geometry.indices[triangle * 3 + 2]];
            const glm::vec3 &k0 = geometry.colors[geometry.indices[triangle * 3 + 0]];
            const glm::vec3 &k1 = geometry.colors[geometry.indices[triangle * 3 + 1]];
            const glm::vec3 &k2 = geometry.colors[geometry.indices[triangle * 3 + 2]];
            const glm::vec2 edge1 = pixelB - pixelA;
            const glm::vec2 edge2 = pixelC - pixelA;
            const float     denominator = edge1.x * edge2.y - edge2.x * edge1.y;
            if (std::fabs(denominator) < 1.0e-12f)
            {
                continue;
            }
            for (std::int32_t y = yBegin; y < yEnd; ++y)
            {
                for (std::int32_t x = xBegin; x < xEnd; ++x)
                {
                    const glm::vec2 pixel(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);
                    const glm::vec2 toPixel = pixel - pixelA;
                    const float     u = (toPixel.x * edge2.y - edge2.x * toPixel.y) / denominator;
                    const float     v = (edge1.x * toPixel.y - toPixel.x * edge1.y) / denominator;
                    if (u < 0.0f || v < 0.0f || u + v > 1.0f)
                    {
                        continue;
                    }
                    const float     interpolatedW = (1.0f - u - v) * c0.w + u * c1.w + v * c2.w;
                    if (interpolatedW <= 0.0f)
                    {
                        continue;
                    }
                    const glm::vec3   worldPosition = p0 * (1.0f - u - v) + p1 * u + p2 * v;
                    const float       distance     = glm::length(worldPosition - state.cameraPosition);
                    const std::size_t pixelIndex  = static_cast<std::size_t>(y) * width + x;
                    if (distance >= state.viewDepth[pixelIndex])
                    {
                        continue;
                    }
                    state.viewDepth[pixelIndex]         = distance;
                    state.viewWorldPosition[pixelIndex] = worldPosition;
                    state.viewSurfaceIndex[pixelIndex]  = surfaceIndex;
                    state.viewNormal[pixelIndex]        = glm::normalize(
                        surface.normalMatrix * glm::normalize(n0 * (1.0f - u - v) + n1 * u + n2 * v));
                    state.viewAlbedo[pixelIndex]   = surface.albedo * (k0 * (1.0f - u - v) + k1 * u + k2 * v);
                    state.viewEmissive[pixelIndex] = surface.emissive;
                    state.viewFlags[pixelIndex]    = static_cast<std::uint8_t>(surface.unlit ? 1u : 0u);
                }
            }
        }
    }
}

// ---- screen-space final gather ------------------------------------------------

void traceScreenProbes(GiPipelineState &state)
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
        state.screenProbeRadiancePrevious.assign(probeCount * directionCount, glm::vec3(0.0f));
    }
    state.screenProbeSh2.assign(probeCount * 27, 0.0f);
    state.screenProbeIrradiance.assign(probeCount * 3, 0.0f);
    state.screenProbeValid.assign(probeCount, 0u);
    state.screenProbesThisFrame.store(0, std::memory_order_relaxed);
    state.gatherRaysThisFrame.store(0, std::memory_order_relaxed);
    state.screenProbeWidth  = probeWidth;
    state.screenProbeHeight = probeHeight;

    const std::uint32_t rayCount  = std::max(state.settings.gatherRaysPerProbe, 1u);
    const std::size_t   mapTexels = directionCount;
    for (std::uint32_t probeY = 0; probeY < probeHeight; ++probeY)
    {
        for (std::uint32_t probeX = 0; probeX < probeWidth; ++probeX)
        {
            const std::size_t   probeIndex = static_cast<std::size_t>(probeY) * probeWidth + probeX;
            // Jittered probe placement inside the scale x scale cell (Lumen
            // jitters the placement grid and accumulates over frames).
            std::uint64_t seed = hashSeed(state.frameIndex * 7919ull + probeIndex);
            const float   jitterX = randomFloat(seed);
            const float   jitterY = randomFloat(seed);
            const std::uint32_t pixelX =
                std::min(probeX * scale + static_cast<std::uint32_t>(jitterX * scale), width - 1u);
            const std::uint32_t pixelY =
                std::min(probeY * scale + static_cast<std::uint32_t>(jitterY * scale), height - 1u);
            const std::size_t   pixelIndex = static_cast<std::size_t>(pixelY) * width + pixelX;
            if (state.viewDepth[pixelIndex] >= std::numeric_limits<float>::max() * 0.5f)
            {
                continue; // background: no probe
            }
            const glm::vec3 position = state.viewWorldPosition[pixelIndex];
            const glm::vec3 normal   = state.viewNormal[pixelIndex];
            glm::vec3 *     radiance = &state.screenProbeRadiance[probeIndex * directionCount];
            state.screenProbeValid[probeIndex] = 1u;
            state.screenProbesThisFrame.fetch_add(1, std::memory_order_relaxed);

            // Importance estimate: the probe's PREVIOUS radiance (last
            // frame's Screen Space Radiance Cache sample of the same area).
            std::vector<float> previousLuminance(directionCount, 0.0f);
            float              luminanceTotal = 0.0f;
            if (state.settings.gatherImportanceSampling)
            {
                for (std::size_t direction = 0; direction < directionCount; ++direction)
                {
                    const glm::vec3 &value     = radiance[direction];
                    const float      luminance = 0.2126f * value.r + 0.7152f * value.g + 0.0722f * value.b;
                    previousLuminance[direction] = luminance;
                    luminanceTotal += luminance;
                }
            }
            const bool  useImportance = luminanceTotal > 1.0e-5f;
            const glm::mat3 basis     = onb(normal);
            // Stratified over the octahedral map: ray r owns texel stratum r,
            // importance-selected by the previous radiance when enabled.
            for (std::uint32_t ray = 0; ray < rayCount; ++ray)
            {
                const std::size_t stratum = (static_cast<std::size_t>(state.frameIndex) * rayCount + ray) % mapTexels;
                std::size_t       selected = stratum;
                if (useImportance && randomFloat(seed) < 0.5f)
                {
                    // Inverse-transform sampling over the previous radiance.
                    const float pick       = randomFloat(seed) * luminanceTotal;
                    float       accumulated = 0.0f;
                    for (std::size_t candidate = 0; candidate < directionCount; ++candidate)
                    {
                        accumulated += previousLuminance[candidate];
                        if (accumulated >= pick)
                        {
                            selected = candidate;
                            break;
                        }
                    }
                }
                const glm::vec3 stratumCenter = state.probeDirections[selected];
                glm::vec3       direction;
                if (glm::dot(stratumCenter, normal) > 0.05f)
                {
                    // Jitter within the stratum's cone (octahedral texel
                    // solid angle ~ 4pi / texelCount).
                    const float cone = std::sqrt(4.0f * kPi / static_cast<float>(mapTexels)) * 0.5f;
                    direction = glm::normalize(stratumCenter
                        + (cosineDirectionSeeded(seed) - glm::vec3(0.0f, 0.0f, 1.0f)) * cone);
                }
                else
                {
                    // Off-hemisphere strata sample the BRDF lobe instead so
                    // the ray budget stays on visible directions.
                    direction = glm::normalize(basis * cosineDirectionSeeded(seed));
                }
                state.gatherRaysThisFrame.fetch_add(1, std::memory_order_relaxed);
                const glm::vec3 origin = position + normal * 0.02f;
                const glm::vec3 value  = gatherRayRadiance(state, origin, direction);
                // Blend the fresh stratum sample into the probe's persistent
                // octahedral map: strata rotate over frames, so the map fills
                // in and temporally accumulates (Lumen jitters the probe grid
                // and accumulates for the same reason).
                glm::vec3 &target = radiance[selected];
                // An untouched texel takes the fresh sample whole; touched
                // ones blend (temporal accumulation without first-frame bias).
                target = glm::length(target) > 0.0f ? glm::mix(value, target, 0.35f) : value;
            }
            projectRadianceToSh2(state.screenProbeDirections.data(), radiance, directionCount,
                &state.screenProbeSh2[probeIndex * 27]);
            float irradiance[3];
            float probeNormal[3] = {normal.x, normal.y, normal.z};
            GiProbe temporary;
            std::memcpy(temporary.sh2, &state.screenProbeSh2[probeIndex * 27], sizeof(temporary.sh2));
            evaluateProbeIrradiance(temporary, probeNormal, irradiance);
            state.screenProbeIrradiance[probeIndex * 3 + 0] = irradiance[0];
            state.screenProbeIrradiance[probeIndex * 3 + 1] = irradiance[1];
            state.screenProbeIrradiance[probeIndex * 3 + 2] = irradiance[2];
        }
    }
}

// Full-resolution pass: interpolate probe SH2 within the depth plane (Lumen
// interpolates "to pixels within the same plane"), then temporal accumulation
// against the reprojection of the previous frame and a bilateral spatial
// filter. Writes gatherIrradiance per pixel (indirect irradiance E).
void resolvePixelIrradiance(GiPipelineState &state)
{
    const std::uint32_t width  = state.viewWidth;
    const std::uint32_t height = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    state.gatherIrradiancePrevious.swap(state.gatherIrradiance);
    if (state.gatherIrradiance.size() != pixelCount)
    {
        state.gatherIrradiance.assign(pixelCount, glm::vec3(0.0f));
        state.gatherIrradiancePrevious.assign(pixelCount, glm::vec3(0.0f));
    }
    const glm::mat4 previousViewProjection = state.previousViewProjection;
    const float     temporalBlend          = std::clamp(state.settings.gatherTemporalBlend, 0.0f, 0.99f);
    const bool      temporalEnabled        = state.hasPreviousView && state.previousViewWidth == width
                                          && state.previousViewHeight == height;
    for (std::uint32_t y = 0; y < height; ++y)
    {
        for (std::uint32_t x = 0; x < width; ++x)
        {
            const std::size_t pixelIndex = static_cast<std::size_t>(y) * width + x;
            const float       depth      = state.viewDepth[pixelIndex];
            if (depth >= std::numeric_limits<float>::max() * 0.5f)
            {
                state.gatherIrradiance[pixelIndex] = glm::vec3(0.0f);
                continue;
            }
            // Interpolate the 2x2 screen-probe neighborhood.
            const std::uint32_t scale = std::max(state.settings.gatherScreenScale, 1u);
            const std::uint32_t probeWidth = state.screenProbeWidth;
            const std::int32_t  probeX = static_cast<std::int32_t>(x / scale);
            const std::int32_t  probeY = static_cast<std::int32_t>(y / scale);
            glm::vec3           interpolated(0.0f);
            float               weightSum = 0.0f;
            for (std::int32_t dy = -1; dy <= 1; ++dy)
            {
                for (std::int32_t dx = -1; dx <= 1; ++dx)
                {
                    const std::int32_t px = probeX + dx;
                    const std::int32_t py = probeY + dy;
                    if (px < 0 || py < 0 || px >= static_cast<std::int32_t>(probeWidth)
                        || py >= static_cast<std::int32_t>(state.screenProbeHeight))
                    {
                        continue;
                    }
                    const std::size_t probeIndex = static_cast<std::size_t>(py) * probeWidth + px;
                    if (state.screenProbeValid[probeIndex] == 0u)
                    {
                        continue;
                    }
                    // Plane-aware weight: reject probes far from this pixel's
                    // depth plane (Lumen's same-plane interpolation).
                    const std::uint32_t probePixelX = std::min(
                        (static_cast<std::uint32_t>(px) + 1u) * scale, width - 1u);
                    const std::uint32_t probePixelY = std::min(
                        (static_cast<std::uint32_t>(py) + 1u) * scale, height - 1u);
                    const float probeDepth =
                        state.viewDepth[static_cast<std::size_t>(probePixelY) * width + probePixelX];
                    if (std::fabs(probeDepth - depth) > state.settings.gatherPlaneDepthTolerance * (depth + 1.0f))
                    {
                        continue;
                    }
                    const float screenDistance = std::sqrt(static_cast<float>(dx * dx + dy * dy));
                    const float weight = 1.0f / (1.0f + screenDistance * 0.75f
                        + std::fabs(probeDepth - depth) / (depth + 1.0f));
                    // Evaluate the probe's SH2 with this pixel's normal.
                    GiProbe temporary;
                    std::memcpy(temporary.sh2, &state.screenProbeSh2[probeIndex * 27], sizeof(temporary.sh2));
                    const glm::vec3 pixelNormal = state.viewNormal[pixelIndex];
                    const float     normalArray[3] = {pixelNormal.x, pixelNormal.y, pixelNormal.z};
                    float           irradiance[3];
                    evaluateProbeIrradiance(temporary, normalArray, irradiance);
                    interpolated += glm::vec3(irradiance[0], irradiance[1], irradiance[2]) * weight;
                    weightSum += weight;
                }
            }
            glm::vec3 irradiance = weightSum > 1.0e-6f ? interpolated / weightSum : glm::vec3(0.0f);
            // Temporal accumulation: reproject the pixel's world position
            // into the previous frame and blend the previous irradiance.
            if (temporalEnabled)
            {
                const glm::vec4 previousClip = previousViewProjection
                    * glm::vec4(state.viewWorldPosition[pixelIndex], 1.0f);
                if (previousClip.w > 0.0f)
                {
                    const glm::vec2 previousNdc(previousClip.x / previousClip.w, previousClip.y / previousClip.w);
                    const float     previousX = (previousNdc.x * 0.5f + 0.5f) * static_cast<float>(width) - 0.5f;
                    const float     previousY = (0.5f - previousNdc.y * 0.5f) * static_cast<float>(height) - 0.5f;
                    if (previousX >= 0.0f && previousY >= 0.0f && previousX < static_cast<float>(width - 1)
                        && previousY < static_cast<float>(height - 1))
                    {
                        const std::uint32_t ix = static_cast<std::uint32_t>(previousX);
                        const std::uint32_t iy = static_cast<std::uint32_t>(previousY);
                        const std::size_t   previousIndex = static_cast<std::size_t>(iy) * width + ix;
                        const float         previousDepth = state.previousViewDepth[previousIndex];
                        const float         depthChange = std::fabs(previousDepth - depth) / (depth + 1.0f);
                        if (previousDepth < std::numeric_limits<float>::max() * 0.5f && depthChange < 0.2f)
                        {
                            irradiance = glm::mix(irradiance, state.gatherIrradiancePrevious[previousIndex],
                                temporalBlend);
                        }
                    }
                }
            }
            state.gatherIrradiance[pixelIndex] = irradiance;
        }
    }
    // Bilateral spatial filter guided by depth and normal (Lumen's full
    // resolution spatial filtering); applied in place from a scratch copy.
    if (state.settings.gatherSpatialFilter > 0)
    {
        static thread_local std::vector<glm::vec3> filtered;
        filtered.assign(pixelCount, glm::vec3(0.0f));
        const std::int32_t radius = static_cast<std::int32_t>(std::min(state.settings.gatherSpatialFilter / 2u, 2u));
        for (std::uint32_t y = 0; y < height; ++y)
        {
            for (std::uint32_t x = 0; x < width; ++x)
            {
                const std::size_t pixelIndex = static_cast<std::size_t>(y) * width + x;
                if (state.viewDepth[pixelIndex] >= std::numeric_limits<float>::max() * 0.5f)
                {
                    filtered[pixelIndex] = glm::vec3(0.0f);
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
                        if (state.viewDepth[sampleIndex] >= std::numeric_limits<float>::max() * 0.5f)
                        {
                            continue;
                        }
                        const float depthWeight  = std::exp(-std::fabs(state.viewDepth[sampleIndex] - centerDepth)
                            / (0.05f * centerDepth + 0.05f));
                        const float normalWeight =
                            std::max(glm::dot(state.viewNormal[sampleIndex], centerNormal), 0.0f);
                        const float weight       = depthWeight * normalWeight;
                        sum += state.gatherIrradiance[sampleIndex] * weight;
                        weightSum += weight;
                    }
                }
                filtered[pixelIndex] = weightSum > 1.0e-6f ? sum / weightSum : state.gatherIrradiance[pixelIndex];
            }
        }
        state.gatherIrradiance.swap(filtered);
    }
}

} // namespace lumen_gi
} // namespace engine
