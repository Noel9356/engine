// lumen_reflection debug views: PPM exports for A/B validation of the
// reflection field, mirroring lumen_gi's debug-view contract.
//
// All views render at the field resolution (the module's working
// resolution): Composite is the module's own shaded image (direct + diffuse
// GI + emission + the reflection field, through the pbr tone mapping);
// ReflectionsOnly isolates the field; ResolveMask colors texels by where
// their radiance resolved (screen trace / scene trace / radiance cache /
// sky); Reference brute-forces reference_samples rays per texel with the
// same estimator as the per-frame pass, so the filtered field can be
// compared against a converged ground truth.

#include "lumen_reflection_internal.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

namespace engine
{
namespace lumen_reflection
{

bool rfComposeDebugView(RfPipelineState &state, TaskScheduler *scheduler, ReflectionDebugView view,
    std::vector<std::uint8_t> &outRgb)
{
    if (state.fieldWidth == 0 || state.fieldHeight == 0 || state.fieldFiltered.empty())
    {
        return false;
    }
    // Views render at the field resolution: the module's working resolution,
    // exactly what the renderer composites, so no resampling can blur an A/B
    // comparison against the Reference view.
    const std::uint32_t viewWidth  = state.fieldWidth;
    const std::uint32_t viewHeight = state.fieldHeight;
    const std::size_t pixelCount = static_cast<std::size_t>(viewWidth) * viewHeight;
    outRgb.assign(pixelCount * 3, 0);

    switch (view)
    {
    case ReflectionDebugView::Composite:
    {
        // Background texels resolve the skylight along their view ray (the
        // un-projection of the pixel center through the inverse camera).
        const glm::mat4 inverseViewProjection = glm::inverse(state.viewProjection);
        parallelRange(scheduler, viewHeight, 1,
            [&](std::size_t rowBegin, std::size_t rowEnd)
            {
                for (std::size_t y = rowBegin; y < rowEnd; ++y)
                {
                    for (std::uint32_t x = 0; x < viewWidth; ++x)
                    {
                        const std::size_t index = y * viewWidth + x;
                        glm::vec3 color(0.0f);
                        if (!rfIsBackground(state.gbufferDepth[index]))
                        {
                            if (state.gbufferUnlit[index] != 0u)
                            {
                                color = state.gbufferAlbedo[index];
                            }
                            else
                            {
                                color = rfShadeHitPoint(state,
                                    state.gbufferPosition[index],
                                    state.gbufferNormal[index],
                                    state.gbufferAlbedo[index],
                                    state.gbufferEmissive[index]);
                            }
                            color += state.fieldFiltered[index];
                        }
                        else
                        {
                            const float ndcX = (static_cast<float>(x) + 0.5f) / static_cast<float>(viewWidth) * 2.0f - 1.0f;
                            const float ndcY = 1.0f - (static_cast<float>(y) + 0.5f) / static_cast<float>(viewHeight) * 2.0f;
                            const glm::vec4 nearPlane = inverseViewProjection * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
                            const glm::vec3 direction = glm::normalize(glm::vec3(nearPlane) / nearPlane.w - state.cameraPosition);
                            color = state.environment.skyRadiance(direction);
                        }
                        storePixel(outRgb, index, color, state.environment.exposure, state.environment.toneMapping);
                    }
                }
            });
        return true;
    }
    case ReflectionDebugView::ReflectionsOnly:
    {
        parallelRange(scheduler, viewHeight, 1,
            [&](std::size_t rowBegin, std::size_t rowEnd)
            {
                for (std::size_t y = rowBegin; y < rowEnd; ++y)
                {
                    for (std::uint32_t x = 0; x < viewWidth; ++x)
                    {
                        const std::size_t index = y * viewWidth + x;
                        storePixel(outRgb, index, state.fieldFiltered[index], state.environment.exposure, state.environment.toneMapping);
                    }
                }
            });
        return true;
    }
    case ReflectionDebugView::ResolveMask:
    {
        const glm::vec3 colorFor[5] = {
            glm::vec3(0.05f),          // None: near black
            glm::vec3(0.1f, 0.8f, 0.2f), // ScreenTrace: green
            glm::vec3(0.2f, 0.4f, 0.9f), // SceneTrace: blue
            glm::vec3(0.9f, 0.8f, 0.1f), // Cache: yellow
            glm::vec3(0.1f, 0.8f, 0.9f), // Sky: cyan
        };
        parallelRange(scheduler, viewHeight, 1,
            [&](std::size_t rowBegin, std::size_t rowEnd)
            {
                for (std::size_t y = rowBegin; y < rowEnd; ++y)
                {
                    for (std::uint32_t x = 0; x < viewWidth; ++x)
                    {
                        const std::size_t index = y * viewWidth + x;
                        // Raw resolve mask colors bypass the tone mapping:
                        // the mask is data, not an image.
                        const glm::vec3 color = colorFor[static_cast<std::uint32_t>(state.fieldResolve[index]) % 5u];
                        outRgb[index * 3 + 0] = toByte(color.x);
                        outRgb[index * 3 + 1] = toByte(color.y);
                        outRgb[index * 3 + 2] = toByte(color.z);
                    }
                }
            });
        return true;
    }
    case ReflectionDebugView::Reference:
    {
        const std::uint32_t samples = std::max(state.settings.referenceSamples, 1u);
        parallelRange(scheduler, viewHeight, 1,
            [&](std::size_t rowBegin, std::size_t rowEnd)
            {
                for (std::size_t y = rowBegin; y < rowEnd; ++y)
                {
                    for (std::uint32_t x = 0; x < viewWidth; ++x)
                    {
                        const std::size_t index = y * viewWidth + x;
                        glm::vec3 color(0.0f);
                        if (!rfIsBackground(state.gbufferDepth[index]) && state.gbufferUnlit[index] == 0u
                            && state.gbufferRoughness[index] <= state.settings.roughMaxTrace)
                        {
                            // Brute-force the same estimator the per-frame
                            // pass uses: many samples per texel, no filters.
                            const glm::vec3 position = state.gbufferPosition[index];
                            const glm::vec3 normal   = state.gbufferNormal[index];
                            const glm::vec3 albedo   = state.gbufferAlbedo[index];
                            const float     metallic = state.gbufferMetallic[index];
                            glm::vec3       view     = state.cameraPosition - position;
                            const float     viewLength = glm::length(view);
                            view /= viewLength;
                            const float clampedRoughness
                                = std::max(state.gbufferRoughness[index], kMinimumRoughness);
                            const float alpha = clampedRoughness * clampedRoughness;
                            // The tangent frame and the Fresnel base reflectivity
                            // are shared by every sample of the texel, exactly
                            // as the per-frame pass hoists them.
                            const glm::mat3 basis     = makeBasis(normal);
                            const glm::vec3 localView = glm::normalize(glm::transpose(basis) * view);
                            const float     texelOpacity = glm::clamp(state.gbufferOpacity[index], 0.0f, 1.0f);
                            const float     texelIor     = glm::max(state.gbufferIor[index], 1.0f);
                            const glm::vec3 f0        = texelOpacity < 1.0f
                                ? glm::vec3(rfDielectricF0FromIor(1.0f, texelIor))
                                : glm::mix(glm::vec3(kDielectricF0), albedo, metallic);
                            const glm::vec3 rayOrigin = position + normal * state.settings.traceBias;
                            const glm::vec4 clipOrigin = state.viewProjection * glm::vec4(rayOrigin, 1.0f);
                            const glm::vec3 refractedView = texelOpacity < 1.0f
                                ? glm::normalize(glm::refract(-view, normal, 1.0f / texelIor))
                                : glm::vec3(0.0f);
                            const bool transmits = texelOpacity < 1.0f
                                && glm::dot(refractedView, refractedView) > 0.5f;
                            const glm::mat3 refractedBasis = transmits ? makeBasis(refractedView) : basis;
                            std::uint64_t seed = hashSeed(0x5EEDBEEFull + index * 2654435761ull);
                            glm::vec3    sum(0.0f);
                            for (std::uint32_t sample = 0; sample < samples; ++sample)
                            {
                                const glm::vec3 direction = rfSampleGgxVndf(basis, localView, alpha, seed);
                                const glm::vec3 fresnel  = rfFresnelWeight(view, direction, f0);
                                glm::vec3 chainWeight(1.0f);
                                glm::vec3 chainDirection(0.0f);
                                glm::vec3 chainOrigin(0.0f);
                                bool      chainReflected = true;
                                if (!rfSplitFirstSegment(transmits, fresnel, texelOpacity, albedo, refractedView,
                                    direction, clampedRoughness, alpha, refractedBasis, position, rayOrigin,
                                    state.settings.traceBias, chainDirection, chainOrigin, chainWeight,
                                    chainReflected, seed))
                                {
                                    continue; // absorbed share
                                }
                                const glm::vec4 chainClipOrigin = chainReflected
                                    ? clipOrigin
                                    : state.viewProjection * glm::vec4(chainOrigin, 1.0f);
                                RfChainResolve chain = resolveRayChain(
                                    state, chainClipOrigin, chainOrigin, chainDirection, chainWeight,
                                    state.settings.maxTransmissionSegments, chainReflected, seed);
                                // The reference brute-forces scene traces only
                                // (no screen tier), so a deferred chain answer is
                                // shaded right here (the texel's own response as
                                // seen from the camera, specular included).
                                if (chain.deferred)
                                {
                                    chain.radiance = rfShadeHitPoint(state,
                                        state.gbufferPosition[chain.hitTexel],
                                        state.gbufferNormal[chain.hitTexel],
                                        state.gbufferAlbedo[chain.hitTexel],
                                        state.gbufferEmissive[chain.hitTexel],
                                        glm::normalize(state.cameraPosition
                                            - state.gbufferPosition[chain.hitTexel]),
                                        state.gbufferRoughness[chain.hitTexel],
                                        state.gbufferMetallic[chain.hitTexel]) * chain.tint;
                                }
                                sum += chain.radiance;
                            }
                            color = sum / static_cast<float>(samples);
                        }
                        storePixel(outRgb, index, color, state.environment.exposure, state.environment.toneMapping);
                    }
                }
            });
        return true;
    }
    }
    return false;
}

} // namespace lumen_reflection
} // namespace engine
