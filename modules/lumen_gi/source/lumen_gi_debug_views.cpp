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

#include <engine/util/ppm.hpp>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

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

// Direct lighting for visible surfaces, mirroring the renderer's shading
// formulas (the GI term replaces the renderer's hemispheric ambient).
glm::vec3 visibleDirectLighting(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal)
{
    const GiEnvironmentSnapshot &environment = state.environment;
    glm::vec3                    sum(0.0f);
    const glm::vec3              sunRadiance = environment.sunRadiance();
    if (sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f)
    {
        const float nDotL = glm::dot(normal, environment.sunDirection);
        if (nDotL > 0.0f)
        {
            sum += sunRadiance * nDotL;
        }
    }
    if (environment.hasDirectLights)
    {
        for (const direct_lighting::DirectLight &light : environment.directLights)
        {
            const direct_lighting::DirectLightDescription &description = light.description;
            const glm::vec3 color(description.color[0], description.color[1], description.color[2]);
            if (description.intensity <= 0.0f)
            {
                continue;
            }
            if (description.type == direct_lighting::DirectLightType::Directional)
            {
                const glm::vec3 direction = -glm::normalize(
                    glm::vec3(description.direction[0], description.direction[1], description.direction[2]));
                const float nDotL = glm::dot(normal, direction);
                if (nDotL > 0.0f)
                {
                    sum += color * description.intensity * nDotL;
                }
                continue;
            }
            const glm::vec3 lightPosition(description.position[0], description.position[1], description.position[2]);
            const glm::vec3 toLight           = lightPosition - position;
            const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
            const float     range             = std::max(description.range, 1.0e-3f);
            const float     rangeRatioSquared = distanceSquared / (range * range);
            if (rangeRatioSquared >= 1.0f)
            {
                continue;
            }
            const glm::vec3 direction   = toLight * (1.0f / std::sqrt(distanceSquared));
            const float     rangeFade   = 1.0f - rangeRatioSquared * rangeRatioSquared;
            float           attenuation = rangeFade * rangeFade / distanceSquared;
            if (description.type == direct_lighting::DirectLightType::Spot)
            {
                const glm::vec3 spotDirection =
                    glm::vec3(description.direction[0], description.direction[1], description.direction[2]);
                const float coneCosine = glm::dot(-direction, glm::normalize(spotDirection));
                const float     inner      = std::cos(description.innerConeRadians);
                const float     outer      = std::cos(description.outerConeRadians);
                const float coneT =
                    std::clamp((coneCosine - outer) / std::max(inner - outer, 1.0e-5f), 0.0f, 1.0f);
                attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
            }
            const float nDotL = glm::dot(normal, direction);
            if (nDotL > 0.0f && attenuation > 0.0f)
            {
                sum += color * description.intensity * attenuation * nDotL;
            }
        }
    }
    return sum;
}

glm::vec3 backgroundSky(const GiPipelineState &state, std::uint32_t x, std::uint32_t y)
{
    const glm::vec2 ndc((static_cast<float>(x) + 0.5f) / static_cast<float>(state.viewWidth) * 2.0f - 1.0f,
        1.0f - (static_cast<float>(y) + 0.5f) / static_cast<float>(state.viewHeight) * 2.0f);
    const glm::vec4 target  = glm::inverse(state.viewProjection) * glm::vec4(ndc, 1.0f, 1.0f);
    const glm::vec3 viewRay = glm::normalize(
        glm::vec3(target) / std::max(target.w, 1.0e-6f) - state.cameraPosition);
    return state.environment.skyRadiance(viewRay);
}

} // namespace

// ---- composition -------------------------------------------------------------

void composeDebugView(GiPipelineState &state, GiDebugView view, std::vector<std::uint8_t> &outRgb)
{
    const std::uint32_t width      = state.viewWidth;
    const std::uint32_t height     = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    outRgb.assign(pixelCount * 3, 0u);
    const GiEnvironmentSnapshot &snapshot  = state.environment;
    const float                  exposure   = snapshot.exposure;
    const pbr::ToneMapping       toneMapper = snapshot.toneMapping;

    for (std::uint32_t y = 0; y < height; ++y)
    {
        for (std::uint32_t x = 0; x < width; ++x)
        {
            const std::size_t pixelIndex = static_cast<std::size_t>(y) * width + x;
            const std::size_t byteIndex  = pixelIndex * 3;
            if (state.viewDepth[pixelIndex] >= std::numeric_limits<float>::max() * 0.5f)
            {
                const glm::vec3 color = gammaEncode(toneMapColor(backgroundSky(state, x, y), exposure, toneMapper));
                outRgb[byteIndex + 0] = toByte(color.r);
                outRgb[byteIndex + 1] = toByte(color.g);
                outRgb[byteIndex + 2] = toByte(color.b);
                continue;
            }
            const glm::vec3 albedo   = state.viewAlbedo[pixelIndex];
            const glm::vec3 emissive = state.viewEmissive[pixelIndex];
            const bool      unlit    = (state.viewFlags[pixelIndex] & 1u) != 0u;
            const glm::vec3 position = state.viewWorldPosition[pixelIndex];
            const glm::vec3 normal   = state.viewNormal[pixelIndex];
            const std::uint32_t surfaceIndex = state.viewSurfaceIndex[pixelIndex];

            glm::vec3 color(0.0f);
            switch (view)
            {
            case GiDebugView::SurfaceCache:
            {
                glm::vec3 cachedAlbedo(0.0f);
                sampleSurfaceAlbedo(state, surfaceIndex, position, normal, cachedAlbedo);
                color = cachedAlbedo;
                break;
            }
            case GiDebugView::LumenScene:
            {
                glm::vec3 radiance(0.0f);
                sampleSurfaceCache(state, surfaceIndex, position, normal, true, radiance);
                color = radiance;
                break;
            }
            case GiDebugView::ProbeIrradiance:
            {
                // World cache trilinear SH2 evaluation with the pixel normal.
                glm::vec3 accumulated(0.0f);
                float     weightSum = 0.0f;
                const float normalArray[3] = {normal.x, normal.y, normal.z};
                for (std::uint32_t levelIndex = 0; levelIndex < 2; ++levelIndex)
                {
                    const GiPipelineState::ProbeLevel &level     = levelIndex == 0 ? state.level0 : state.level1;
                    const std::uint32_t                dimension = level.dimensions[0];
                    const glm::vec3 cell     = (position - level.anchor) / level.spacing - 0.5f;
                    const glm::vec3 base     = glm::clamp(glm::floor(cell), glm::vec3(0.0f),
                        glm::vec3(static_cast<float>(dimension - 2)));
                    const glm::vec3 fraction = glm::clamp(cell - base, glm::vec3(0.0f), glm::vec3(1.0f));
                    bool anyValid = false;
                    for (std::uint32_t corner = 0; corner < 8; ++corner)
                    {
                        const std::uint32_t cx = static_cast<std::uint32_t>(base.x) + (corner & 1u);
                        const std::uint32_t cy = static_cast<std::uint32_t>(base.y) + ((corner >> 1) & 1u);
                        const std::uint32_t cz = static_cast<std::uint32_t>(base.z) + ((corner >> 2) & 1u);
                        const float wx = (corner & 1u) != 0u ? fraction.x : 1.0f - fraction.x;
                        const float wy = ((corner >> 1) & 1u) != 0u ? fraction.y : 1.0f - fraction.y;
                        const float wz = ((corner >> 2) & 1u) != 0u ? fraction.z : 1.0f - fraction.z;
                        const float weight = wx * wy * wz;
                        const GiPipelineState::ProbeLevel::Probe &probe =
                            level.probes[(static_cast<std::size_t>(cz) * dimension + cy) * dimension + cx];
                        if (!probe.valid || probe.validity <= 0.0f || weight <= 0.0f)
                        {
                            continue;
                        }
                        anyValid = true;
                        float irradiance[3];
                        evaluateSh2Irradiance(probe.sh2.data(), normalArray, irradiance);
                        accumulated += glm::vec3(irradiance[0], irradiance[1], irradiance[2]) * weight * probe.validity;
                        weightSum += weight * probe.validity;
                    }
                    if (anyValid && weightSum > 1.0e-6f)
                    {
                        break; // nearest level resolved the point
                    }
                    weightSum = 0.0f;
                    accumulated = glm::vec3(0.0f);
                }
                color = weightSum > 1.0e-6f ? albedo / detail::kPi * (accumulated / weightSum) + emissive
                                            : emissive;
                break;
            }
            case GiDebugView::GiOnly:
            {
                color = albedo / detail::kPi * state.gatherIrradiance[pixelIndex] + emissive;
                break;
            }
            case GiDebugView::FinalImage:
            default:
            {
                color = emissive;
                if (unlit)
                {
                    color += albedo;
                }
                else
                {
                    const bool shadowed = state.settings.sunShadows
                                              ? sunShadowed(state, position, normal, 0xffffffffu)
                                              : false;
                    glm::vec3 direct(0.0f);
                    if (!shadowed)
                    {
                        direct = visibleDirectLighting(state, position, normal);
                    }
                    color += direct * albedo;
                    if (state.settings.debugIncludeAmbient)
                    {
                        const glm::vec3 hemisphere = glm::mix(snapshot.ambientGround, snapshot.ambientSky,
                            normal.y * 0.5f + 0.5f) * snapshot.ambientIntensity;
                        color += albedo * hemisphere;
                    }
                    color += albedo / detail::kPi * state.gatherIrradiance[pixelIndex];
                }
                break;
            }
            }
            const glm::vec3 encoded = gammaEncode(toneMapColor(color, exposure, toneMapper));
            outRgb[byteIndex + 0] = toByte(encoded.r);
            outRgb[byteIndex + 1] = toByte(encoded.g);
            outRgb[byteIndex + 2] = toByte(encoded.b);
        }
    }
}

// ---- reference path tracer -----------------------------------------------------

void referencePathTrace(GiPipelineState &state, std::vector<std::uint8_t> &outRgb)
{
    const std::uint32_t width      = state.viewWidth;
    const std::uint32_t height     = state.viewHeight;
    const std::size_t   pixelCount = static_cast<std::size_t>(width) * height;
    outRgb.assign(pixelCount * 3, 0u);
    std::vector<glm::vec3> accumulation(pixelCount, glm::vec3(0.0f));
    const GiEnvironmentSnapshot &snapshot  = state.environment;
    const float                  exposure   = snapshot.exposure;
    const pbr::ToneMapping       toneMapper = snapshot.toneMapping;
    const std::uint32_t          samples    = std::max(state.settings.referenceSamples, 1u);
    const std::uint32_t          bounces    = std::max(state.settings.referenceBounces, 1u);
    const glm::mat4              inverseViewProjection = glm::inverse(state.viewProjection);

    for (std::uint32_t y = 0; y < height; ++y)
    {
        for (std::uint32_t x = 0; x < width; ++x)
        {
            std::uint64_t seed = detail::hashSeed(0x5DEECE66Dull * (static_cast<std::uint64_t>(y) * width + x + 1)
                + state.frameIndex);
            glm::vec3     sum(0.0f);
            for (std::uint32_t sample = 0; sample < samples; ++sample)
            {
                const glm::vec2 jitter(detail::randomFloat(seed), detail::randomFloat(seed));
                const glm::vec2 ndc(((static_cast<float>(x) + jitter.x) / static_cast<float>(width)) * 2.0f - 1.0f,
                    1.0f - ((static_cast<float>(y) + jitter.y) / static_cast<float>(height)) * 2.0f);
                const glm::vec4 target  = inverseViewProjection * glm::vec4(ndc, 1.0f, 1.0f);
                glm::vec3       direction = glm::normalize(
                    glm::vec3(target) / std::max(target.w, 1.0e-6f) - state.cameraPosition);
                glm::vec3       pathRadiance(0.0f);
                glm::vec3       throughput(1.0f);
                glm::vec3       origin = state.cameraPosition;
                bool            terminated = false;
                for (std::uint32_t bounce = 0; bounce <= bounces && !terminated; ++bounce)
                {
                    GiRay       ray{origin, direction, state.settings.maxTraceDistance};
                    const GiHit hit = traceScene(state, ray);
                    if (!hit.hit)
                    {
                        pathRadiance += throughput * snapshot.skyRadiance(direction);
                        break;
                    }
                    if (glm::dot(hit.geometricNormal, direction) > 0.0f)
                    {
                        break; // backface: inside geometry
                    }
                    const GiSurface &surface = state.surfaces[hit.surfaceIndex];
                    glm::vec3        hitAlbedo(1.0f);
                    {
                        glm::vec3 cachedAlbedo(0.0f);
                        sampleSurfaceAlbedo(state, hit.surfaceIndex, hit.position, hit.normal, cachedAlbedo);
                        hitAlbedo = glm::length(cachedAlbedo) > 0.0f ? cachedAlbedo : surface.albedo;
                    }
                    glm::vec3 emission(0.0f);
                    {
                        glm::vec3 cachedRadiance(0.0f);
                        sampleSurfaceCache(state, hit.surfaceIndex, hit.position, hit.normal, false, cachedRadiance);
                        // Emissive + unlit carry no lighting response: take the
                        // direct term as emission for those surfaces.
                        if (surface.unlit || glm::length(surface.emissive) > 0.0f)
                        {
                            emission = cachedRadiance;
                        }
                    }
                    pathRadiance += throughput * emission;
                    // Next-event estimation: shadowed sun.
                    const float nDotL = glm::dot(hit.normal, snapshot.sunDirection);
                    if (nDotL > 0.0f && snapshot.sunEnabled)
                    {
                        if (!sunShadowed(state, hit.position, hit.normal, hit.surfaceIndex))
                        {
                            pathRadiance += throughput * hitAlbedo / detail::kPi * snapshot.sunRadiance() * nDotL;
                        }
                    }
                    // Indirect: cosine-sampled bounce.
                    throughput *= hitAlbedo;
                    const glm::mat3 basis    = detail::onb(hit.normal);
                    direction               = glm::normalize(basis * detail::cosineDirectionSeeded(seed));
                    origin                  = hit.position + hit.normal * 0.02f;
                }
                sum += pathRadiance;
            }
            const glm::vec3 mean    = sum / static_cast<float>(samples);
            const glm::vec3 encoded = gammaEncode(toneMapColor(mean, exposure, toneMapper));
            const std::size_t byteIndex = (static_cast<std::size_t>(y) * width + x) * 3;
            outRgb[byteIndex + 0] = toByte(encoded.r);
            outRgb[byteIndex + 1] = toByte(encoded.g);
            outRgb[byteIndex + 2] = toByte(encoded.b);
        }
    }
}

// ---- export entry point ---------------------------------------------------------

bool exportDebugView(GiPipelineState &state, GiDebugView view, const char *filePath, GiError *error)
{
    std::vector<std::uint8_t> rgb;
    if (view == GiDebugView::Reference)
    {
        referencePathTrace(state, rgb);
    }
    else
    {
        composeDebugView(state, view, rgb);
    }
    if (!util::writePpmRgb(filePath, state.viewWidth, state.viewHeight, rgb.data(),
            static_cast<std::size_t>(state.viewWidth) * 3))
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
