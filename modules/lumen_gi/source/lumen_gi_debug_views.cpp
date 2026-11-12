// lumen_gi debug views: composition of the debug images from the gathered
// buffers, the brute-force path tracer used as ground truth for validation,
// and the PPM export entry point.
//
// The composed image applies the same lighting model as the renderer (sun +
// direct lights + emissive through the environment's exposure and tone
// mapping) with the GI term replacing the hemispheric ambient; the reference
// path tracer shares the environment snapshot but traces real paths so
// images can be compared numerically.

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace engine
{
namespace lumen_gi
{

namespace
{

glm::vec3 toneMapColor(glm::vec3 color, float exposure, pbr::ToneMapping mapper)
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

std::uint8_t toByte(float value)
{
    return static_cast<std::uint8_t>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
}

glm::vec3 gammaEncode(glm::vec3 color)
{
    return glm::pow(glm::max(color, glm::vec3(0.0f)), glm::vec3(1.0f / 2.2f));
}

void storePixel(std::vector<std::uint8_t> &rgb, std::size_t pixelIndex, const glm::vec3 &linearColor,
    const GiEnvironmentSnapshot &environment)
{
    const glm::vec3 encoded = gammaEncode(toneMapColor(linearColor, environment.exposure, environment.toneMapping));
    rgb[pixelIndex * 3 + 0]  = toByte(encoded.r);
    rgb[pixelIndex * 3 + 1]  = toByte(encoded.g);
    rgb[pixelIndex * 3 + 2]  = toByte(encoded.b);
}

bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgb)
{
    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        return false;
    }
    std::fprintf(file, "P6\n%u %u\n255\n", width, height);
    const std::size_t byteCount = static_cast<std::size_t>(width) * height * 3;
    const bool        ok        = std::fwrite(rgb, 1, byteCount, file) == byteCount;
    return std::fclose(file) == 0 && ok;
}

// Camera ray through the pixel (u, v in pixels).
glm::vec3 viewRayDirection(const GiPipelineState &state, const glm::mat4 &inverseViewProjection, float px, float py)
{
    const glm::vec2 ndc(px / static_cast<float>(state.viewWidth) * 2.0f - 1.0f,
        1.0f - py / static_cast<float>(state.viewHeight) * 2.0f);
    const glm::vec4 target = inverseViewProjection * glm::vec4(ndc, 1.0f, 1.0f);
    return glm::normalize(glm::vec3(target) / (std::fabs(target.w) > 1.0e-6f ? target.w : 1.0e-6f)
        - state.cameraPosition);
}

} // namespace

// ---- composition -------------------------------------------------------------

void composeDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, std::vector<std::uint8_t> &outRgb)
{
    const std::uint32_t width      = state.viewWidth;
    const std::uint32_t height     = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    outRgb.assign(pixelCount * 3, 0u);
    const GiEnvironmentSnapshot &snapshot = state.environment;
    const glm::mat4 inverseViewProjection = glm::inverse(state.viewProjection); // once, not per pixel
    const glm::vec3 sunRadiance           = snapshot.sunRadiance();
    const bool      hasSun                = sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f;
    const GiPipelineState &constState     = state;

    parallelRange(scheduler, height, 4,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < width; ++x)
                {
                    const std::size_t pixelIndex = y * width + x;
                    if (constState.viewDepth[pixelIndex] >= std::numeric_limits<float>::max() * 0.5f)
                    {
                        const glm::vec3 ray = viewRayDirection(constState, inverseViewProjection,
                            static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);
                        storePixel(outRgb, pixelIndex, snapshot.skyRadiance(ray), snapshot);
                        continue;
                    }
                    const glm::vec3 albedo   = constState.viewAlbedo[pixelIndex];
                    const glm::vec3 emissive = constState.viewEmissive[pixelIndex];
                    const bool      unlit    = (constState.viewFlags[pixelIndex] & 1u) != 0u;
                    const glm::vec3 position = constState.viewWorldPosition[pixelIndex];
                    const glm::vec3 normal   = constState.viewNormal[pixelIndex];
                    const std::uint32_t surfaceIndex = constState.viewSurfaceIndex[pixelIndex];

                    glm::vec3 color(0.0f);
                    switch (view)
                    {
                    case GiDebugView::SurfaceCache:
                        sampleSurfaceAlbedo(constState, surfaceIndex, position, normal, color);
                        break;
                    case GiDebugView::LumenScene:
                        sampleSurfaceCache(constState, surfaceIndex, position, normal, true, color);
                        break;
                    case GiDebugView::ProbeIrradiance:
                    {
                        glm::vec3 irradiance(0.0f);
                        color = worldCacheIrradiance(constState, position, normal, irradiance)
                            ? albedo / kPi * irradiance + emissive
                            : emissive;
                        break;
                    }
                    case GiDebugView::GiOnly:
                        color = albedo / kPi * constState.gatherIrradiance[pixelIndex] + emissive;
                        break;
                    case GiDebugView::FinalImage:
                    default:
                    {
                        color = emissive;
                        if (unlit)
                        {
                            color += albedo;
                            break;
                        }
                        // Sun and direct lights are separate terms: a shadowed
                        // sun must not switch the other lights off.
                        glm::vec3 irradiance(0.0f);
                        const float nDotL = glm::dot(normal, snapshot.sunDirection);
                        if (hasSun && nDotL > 0.0f
                            && (!constState.settings.sunShadows || !sunShadowed(constState, position, normal)))
                        {
                            irradiance += sunRadiance * nDotL;
                        }
                        irradiance += directLightRadiance(constState, position, normal);
                        color += albedo * (irradiance * (1.0f / kPi));
                        if (constState.settings.debugIncludeAmbient)
                        {
                            color += albedo * snapshot.skyRadiance(normal);
                        }
                        color += albedo / kPi * constState.gatherIrradiance[pixelIndex];
                        break;
                    }
                    }
                    storePixel(outRgb, pixelIndex, color, snapshot);
                }
            }
        });
}

// ---- reference path tracer -----------------------------------------------------

void referencePathTrace(GiPipelineState &state, TaskScheduler *scheduler, std::vector<std::uint8_t> &outRgb)
{
    const std::uint32_t width      = state.viewWidth;
    const std::uint32_t height     = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    outRgb.assign(pixelCount * 3, 0u);
    const GiEnvironmentSnapshot &snapshot = state.environment;
    const std::uint32_t samples = std::max(state.settings.referenceSamples, 1u);
    const std::uint32_t bounces = std::max(state.settings.referenceBounces, 1u);
    const glm::mat4     inverseViewProjection = glm::inverse(state.viewProjection);
    const glm::vec3     sunRadiance = snapshot.sunRadiance();
    const bool          hasSun      = sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f;
    const GiPipelineState &constState = state;

    parallelRange(scheduler, height, 2,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < width; ++x)
                {
                    std::uint64_t seed = hashSeed(0x5DEECE66Dull * (static_cast<std::uint64_t>(y) * width + x + 1)
                        + constState.frameIndex);
                    glm::vec3 sum(0.0f);
                    for (std::uint32_t sample = 0; sample < samples; ++sample)
                    {
                        glm::vec3 direction = viewRayDirection(constState, inverseViewProjection,
                            static_cast<float>(x) + randomFloat(seed), static_cast<float>(y) + randomFloat(seed));
                        glm::vec3 pathRadiance(0.0f);
                        glm::vec3 throughput(1.0f);
                        glm::vec3 origin = constState.cameraPosition;
                        for (std::uint32_t bounce = 0; bounce <= bounces; ++bounce)
                        {
                            const GiHit hit = traceScene(constState, GiRay{origin, direction, constState.settings.maxTraceDistance});
                            if (!hit.hit)
                            {
                                pathRadiance += throughput * snapshot.skyRadiance(direction);
                                break;
                            }
                            if (hit.backface)
                            {
                                break; // inside geometry
                            }
                            const GiSurface &surface = constState.surfaces[hit.surfaceIndex];
                            glm::vec3 hitAlbedo(0.0f);
                            sampleSurfaceAlbedoLocal(constState, surface, hit.localPosition, hit.localNormal, hitAlbedo);
                            if (glm::dot(hitAlbedo, hitAlbedo) <= 0.0f)
                            {
                                hitAlbedo = surface.albedo;
                            }
                            // Emission only: unlit surfaces show their albedo,
                            // lit ones their emissive term (never the lit
                            // radiance the cache stores).
                            pathRadiance += throughput * (surface.unlit ? hitAlbedo : surface.emissive);
                            if (surface.unlit)
                            {
                                break;
                            }
                            // Next-event estimation: shadowed sun + direct lights.
                            glm::vec3 irradiance(0.0f);
                            const float nDotL = glm::dot(hit.normal, snapshot.sunDirection);
                            if (hasSun && nDotL > 0.0f && !sunShadowed(constState, hit.position, hit.normal))
                            {
                                irradiance += sunRadiance * nDotL;
                            }
                            irradiance += directLightRadiance(constState, hit.position, hit.normal);
                            pathRadiance += throughput * hitAlbedo * (irradiance * (1.0f / kPi));
                            // Indirect: cosine-sampled bounce.
                            throughput *= hitAlbedo;
                            direction = glm::normalize(makeBasis(hit.normal) * cosineHemisphere(randomFloat(seed), randomFloat(seed)));
                            origin    = hit.position + hit.normal * 0.02f;
                        }
                        sum += pathRadiance;
                    }
                    storePixel(outRgb, static_cast<std::size_t>(y) * width + x, sum / static_cast<float>(samples), snapshot);
                }
            }
        });
}

// ---- export entry point ---------------------------------------------------------

bool exportDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, const char *filePath, GiError *error)
{
    std::vector<std::uint8_t> rgb;
    if (view == GiDebugView::Reference)
    {
        referencePathTrace(state, scheduler, rgb);
    }
    else
    {
        composeDebugView(state, scheduler, view, rgb);
    }
    if (!writePpm(filePath, state.viewWidth, state.viewHeight, rgb.data()))
    {
        if (error != nullptr)
        {
            *error = GiError::ExportFailed;
        }
        return false;
    }
    if (error != nullptr)
    {
        *error = GiError::None;
    }
    return true;
}

} // namespace lumen_gi
} // namespace engine
