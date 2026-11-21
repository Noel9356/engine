// lumen_gi debug views: composition of the debug images from the gathered
// buffers, the brute-force path tracer used as ground truth for validation,
// and the PPM export entry point.
//
// The composed image applies the same lighting model as the renderer (sun +
// direct lights + emissive through the environment's exposure and tone
// mapping) with the GI term replacing the hemispheric ambient; the reference
// path tracer shares the environment snapshot but traces real paths so
// images can be compared numerically.

#include <engine/kernel/task_scheduler.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/util/ppm.hpp>
#include "lumen_gi_internal.hpp"

#include <cstdint>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace engine
{
namespace lumen_gi
{

namespace
{

bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgb)
{
    // The shared util writer (engine::util::writePpmRgb) carries the overflow
    // guards this local copy never had.
    return util::writePpmRgb(filePath, width, height, rgb, static_cast<std::size_t>(width) * 3);
}

// Camera ray through the pixel (u, v in pixels). Unprojects two NDC depths
// and takes their difference, so perspective and orthographic projections
// both give the exact view ray.
glm::vec3 viewRayDirection(const GiPipelineState &state, const glm::mat4 &inverseViewProjection, float px, float py)
{
    const glm::vec2 ndc(px / static_cast<float>(state.viewWidth) * 2.0f - 1.0f,
        1.0f - py / static_cast<float>(state.viewHeight) * 2.0f);
    const auto unproject = [&](float depthNdc)
    {
        const glm::vec4 target = inverseViewProjection * glm::vec4(ndc, depthNdc, 1.0f);
        return glm::vec3(target) / (std::fabs(target.w) > 1.0e-6f ? target.w : 1.0e-6f);
    };
    return glm::normalize(unproject(1.0f) - unproject(-1.0f));
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
                        storePixel(outRgb, pixelIndex, snapshot.skyRadiance(ray), snapshot.exposure, snapshot.toneMapping);
                        continue;
                    }
                    const glm::vec3 albedo   = constState.viewAlbedo[pixelIndex];
                    const glm::vec3 emissive = constState.viewEmissive[pixelIndex];
                    const bool      unlit    = (constState.viewFlags[pixelIndex] & 1u) != 0u;
                    const glm::vec3 position = constState.viewWorldPosition[pixelIndex];
                    const glm::vec3 normal   = constState.viewNormal[pixelIndex];
                    const std::uint32_t surfaceIndex = constState.viewSurfaceIndex[pixelIndex];
                    // The share of its own response a transparent surface
                    // scatters (scatteredShare; the transmitted share leaves
                    // through the transmission paths the gather, probe, and
                    // reference rays walk separately). The image views compose
                    // the scattered response, so every lit term carries it -
                    // the card caches and the renderer's shading apply the
                    // same share.
                    const float opacityShare = glm::clamp(constState.surfaces[surfaceIndex].opacity, 0.0f, 1.0f);

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
                        // The indirect term on top of emission, so lamp pixels
                        // stay identifiable in the A/B against FinalImage. Both
                        // terms scatter at the surface's opacity share.
                        color = albedo / kPi * constState.gatherFiltered[pixelIndex] * opacityShare
                            + emissive * opacityShare;
                        break;
                    case GiDebugView::FinalImage:
                    default:
                    {
                        color = emissive * opacityShare;
                        if (unlit)
                        {
                            color += albedo;
                            break;
                        }
                        // Sun and direct lights are separate terms: a shadowed
                        // sun must not switch the other lights off. Transparent
                        // surfaces soften the shadow by their transmittance.
                        glm::vec3 irradiance(0.0f);
                        const float nDotL = glm::dot(normal, snapshot.sunDirection);
                        if (hasSun && nDotL > 0.0f)
                        {
                            if (!constState.settings.sunShadows)
                            {
                                irradiance += sunRadiance * nDotL;
                            }
                            else
                            {
                                irradiance += sunRadiance * nDotL
                                    * sunShadowTransmittance(constState, position, normal);
                            }
                        }
                        irradiance += directLightRadiance(constState, position, normal);
                        color += albedo * (irradiance * (1.0f / kPi)) * opacityShare;
                        if (constState.settings.debugIncludeAmbient)
                        {
                            color += albedo * snapshot.skyRadiance(normal) * opacityShare;
                        }
                        color += albedo / kPi * constState.gatherFiltered[pixelIndex] * opacityShare;
                        break;
                    }
                    }
                    storePixel(outRgb, pixelIndex, color, snapshot.exposure, snapshot.toneMapping);
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

    // One pending transmitted continuation of a reference path: a transparent
    // interface split the path into its scattered share (walked now) and its
    // transmitted share (walked when the current branch ends). The stack
    // bounds the nested transparent interfaces a path walks; splits deeper
    // than the ceiling truncate their transmitted share deterministically.
    struct PendingReferencePath
    {
        glm::vec3     origin;
        glm::vec3     direction;
        glm::vec3     throughput;
        std::uint32_t bouncesLeft;
    };
    constexpr std::uint32_t kReferencePendingCapacity = 8u;
    // One pending entry per transparent interface crossing, bounded by the
    // module's transmission budget.
    const std::uint32_t pendingCapacity
        = std::min(state.settings.maxTransmissionSegments + 1u, kReferencePendingCapacity);

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
                        // Sequenced draws: the jitter offsets are consumed in a
                        // fixed order so every toolchain sees the same path.
                        const float jitterX = randomFloat(seed);
                        const float jitterY = randomFloat(seed);
                        glm::vec3 direction = viewRayDirection(constState, inverseViewProjection,
                            static_cast<float>(x) + jitterX, static_cast<float>(y) + jitterY);
                        glm::vec3 pathRadiance(0.0f);
                        // A transparent interface splits the path into its
                        // scattered share (cosine bounce) and its transmitted
                        // share (refracted continuation). Both shares are
                        // walked - the deterministic split the cache's gather
                        // and the probe rays mirror - so the reference
                        // estimates a pane's own bounce and the scene behind
                        // it; the stack resumes the transmitted continuations
                        // depth-first when the current branch ends.
                        PendingReferencePath pending[kReferencePendingCapacity];
                        std::uint32_t pendingCount  = 0;
                        glm::vec3     throughput(1.0f);
                        glm::vec3     origin      = constState.cameraPosition;
                        std::uint32_t bouncesLeft = bounces;
                        // Pops the next transmitted continuation; false ends
                        // the path (no share left to walk).
                        const auto resumePending = [&]() -> bool
                        {
                            if (pendingCount == 0u)
                            {
                                return false;
                            }
                            const PendingReferencePath &resumed = pending[--pendingCount];
                            origin      = resumed.origin;
                            direction   = resumed.direction;
                            throughput  = resumed.throughput;
                            bouncesLeft = resumed.bouncesLeft;
                            return true;
                        };
                        for (;;)
                        {
                            const GiHit hit = traceScene(constState, GiRay{origin, direction, constState.settings.maxTraceDistance});
                            if (!hit.hit)
                            {
                                pathRadiance += throughput * snapshot.skyRadiance(direction);
                                if (!resumePending())
                                {
                                    break;
                                }
                                continue;
                            }
                            const GiSurface &surface = constState.surfaces[hit.surfaceIndex];
                            const float     opacity  = glm::clamp(surface.opacity, 0.0f, 1.0f);
                            if (hit.backface && opacity >= 1.0f)
                            {
                                // Inside opaque geometry: the branch ends (the
                                // module's anti-leak rule); the pending shares
                                // still walk.
                                if (!resumePending())
                                {
                                    break;
                                }
                                continue;
                            }
                            // The interface normal faces the ray: outward
                            // normals stay outward on a medium exit (a
                            // backface hit), and the refraction needs the
                            // normal on the ray's side to bend across the
                            // interface.
                            const glm::vec3 interfaceNormal
                                = glm::dot(hit.normal, direction) < 0.0f ? hit.normal : -hit.normal;
                            if (opacity < 1.0f)
                            {
                                if (hit.backface)
                                {
                                    // Medium exit: the anti-leak rule keeps the
                                    // exit face dark (inside hits contribute
                                    // nothing); the share leaves untinted (the
                                    // albedo absorption applied once, at the
                                    // entering interface) and bent by the
                                    // inverse Snell ratio - total internal
                                    // reflection mirrors the path back inside.
                                    if (bouncesLeft-- == 0u)
                                    {
                                        if (!resumePending())
                                        {
                                            break;
                                        }
                                        continue;
                                    }
                                    glm::vec3 bent = glm::refract(direction, interfaceNormal,
                                        glm::max(surface.indexOfRefraction, 1.0f));
                                    if (glm::dot(bent, bent) < 1.0e-8f)
                                    {
                                        bent = glm::reflect(direction, interfaceNormal);
                                    }
                                    direction = glm::normalize(bent);
                                    origin    = hit.position + direction * constState.settings.traceBias;
                                    continue;
                                }
                                // Transparent surface, entering the medium: the
                                // opacity share scatters (emission + the
                                // albedo-lit direct response next-event
                                // estimated here + the indirect share the
                                // cosine bounce below collects), the rest
                                // transmits - tinted by the albedo and bent by
                                // Snell's law with the index ratio (the
                                // reference tracer mirrors the cache's
                                // deterministic split).
                                glm::vec3 hitAlbedo(0.0f);
                                sampleSurfaceAlbedoLocal(constState, surface, hit.localPosition, hit.localNormal, hitAlbedo);
                                if (glm::dot(hitAlbedo, hitAlbedo) <= 0.0f)
                                {
                                    hitAlbedo = surface.albedo;
                                }
                                glm::vec3 scattered = surface.unlit ? hitAlbedo : surface.emissive;
                                if (!surface.unlit)
                                {
                                    glm::vec3 irradiance(0.0f);
                                    const float nDotL = glm::dot(interfaceNormal, snapshot.sunDirection);
                                    if (hasSun && nDotL > 0.0f)
                                    {
                                        irradiance += sunRadiance * nDotL
                                            * sunShadowTransmittance(constState, hit.position, interfaceNormal);
                                    }
                                    irradiance += directLightRadiance(constState, hit.position, interfaceNormal);
                                    scattered += hitAlbedo * (irradiance * (1.0f / kPi));
                                }
                                pathRadiance += throughput * opacity * scattered;
                                if (bouncesLeft-- == 0u)
                                {
                                    if (!resumePending())
                                    {
                                        break;
                                    }
                                    continue;
                                }
                                // Transmitted share: pushed and resumed when
                                // the scattered branch ends.
                                if (opacity > 0.0f && pendingCount < pendingCapacity)
                                {
                                    glm::vec3 bent = glm::refract(direction, interfaceNormal,
                                        1.0f / glm::max(surface.indexOfRefraction, 1.0f));
                                    if (glm::dot(bent, bent) < 1.0e-8f)
                                    {
                                        bent = glm::reflect(direction, interfaceNormal);
                                    }
                                    bent = glm::normalize(bent);
                                    pending[pendingCount++] = PendingReferencePath{
                                        hit.position + bent * constState.settings.traceBias, bent,
                                        throughput * surface.albedo * (1.0f - opacity), bouncesLeft};
                                }
                                if (surface.unlit)
                                {
                                    if (!resumePending())
                                    {
                                        break;
                                    }
                                    continue;
                                }
                                // The scattered share's indirect: the cosine
                                // bounce collects albedo * E_i of the opacity
                                // share.
                                throughput *= surface.albedo * opacity;
                                direction   = glm::normalize(makeBasis(interfaceNormal)
                                    * cosineHemisphere(randomFloat(seed), randomFloat(seed)));
                                origin      = hit.position + interfaceNormal * constState.settings.traceBias;
                                continue;
                            }
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
                                if (!resumePending())
                                {
                                    break;
                                }
                                continue;
                            }
                            // Next-event estimation: shadowed sun + direct
                            // lights; transparent surfaces soften the shadow.
                            glm::vec3 irradiance(0.0f);
                            const float nDotL = glm::dot(interfaceNormal, snapshot.sunDirection);
                            if (hasSun && nDotL > 0.0f)
                            {
                                const float transmittance = sunShadowTransmittance(constState, hit.position, interfaceNormal);
                                irradiance += sunRadiance * nDotL * transmittance;
                            }
                            irradiance += directLightRadiance(constState, hit.position, interfaceNormal);
                            pathRadiance += throughput * hitAlbedo * (irradiance * (1.0f / kPi));
                            // Indirect: cosine-sampled bounce.
                            if (bouncesLeft-- == 0u)
                            {
                                if (!resumePending())
                                {
                                    break;
                                }
                                continue;
                            }
                            throughput *= hitAlbedo;
                            direction   = glm::normalize(makeBasis(interfaceNormal)
                                * cosineHemisphere(randomFloat(seed), randomFloat(seed)));
                            origin      = hit.position + interfaceNormal * constState.settings.traceBias;
                        }
                        sum += pathRadiance;
                    }
                    storePixel(outRgb, static_cast<std::size_t>(y) * width + x, sum / static_cast<float>(samples), snapshot.exposure,
        snapshot.toneMapping);
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
