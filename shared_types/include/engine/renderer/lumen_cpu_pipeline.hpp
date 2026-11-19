// Shared CPU pipeline helpers for the engine's software lighting systems
// (lumen_gi and lumen_reflection).
//
// The two lighting modules run the same shape of frame: plan an amortized
// workload, fan the per-texel or per-band stages out over the kernel's task
// scheduler, and compose debug images from the HDR results. The helpers here
// keep that machinery in one place so both modules agree on the parallel
// chunking behavior and on how linear HDR colors reach the debug bytes.
//
// Header-only and dependency-free beyond the kernel scheduler interface, the
// shared math vocabulary, and the pbr tone-mapping enum.

#pragma once

#include <engine/kernel/task_scheduler.hpp>
#include <engine/pbr/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace engine::lumen
{

// Runs `body(begin, end)` over [0, itemCount) in parallel, splitting the
// range into chunks of at least `minItemsPerTask` items. A null scheduler
// (or a scheduler that is not running) falls back to one inline call over
// the whole range, so callers do not branch on the execution environment.
template <typename Body>
void parallelRange(TaskScheduler *scheduler, std::size_t itemCount, std::size_t minItemsPerTask, Body &&body)
{
    if (itemCount == 0)
    {
        return;
    }
    if (scheduler == nullptr)
    {
        body(std::size_t{0}, itemCount);
        return;
    }
    scheduler->parallelFor(itemCount, minItemsPerTask, body);
}

// Clamps a linear channel into [0, 1] and quantizes it to a display byte.
inline std::uint8_t toByte(float channel)
{
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<std::uint8_t>(std::lround(clamped * 255.0f));
}

// sRGB-ish display gamma: the debug views show tone-mapped colors through a
// standard 2.2 encode so mid-tones read the way the interactive renders do.
inline float gammaEncode(float channel)
{
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return std::pow(clamped, 1.0f / 2.2f);
}

// Tone maps a linear HDR color with the pbr module's operators, mirroring
// mesh.frag's toneMap(): exposure, then none / Reinhard / ACES filmic.
inline glm::vec3 toneMapColor(const glm::vec3 &color, float exposure, pbr::ToneMapping mapper)
{
    glm::vec3 mapped = color * exposure;
    switch (mapper)
    {
    case pbr::ToneMapping::Reinhard:
        mapped = mapped / (1.0f + mapped);
        break;
    case pbr::ToneMapping::AcesFilmic:
        mapped = glm::clamp((mapped * (2.51f * mapped + 0.03f)) / (mapped * (2.43f * mapped + 0.59f) + 0.14f), 0.0f, 1.0f);
        break;
    case pbr::ToneMapping::None:
    default:
        break;
    }
    return mapped;
}

// Composes one debug pixel: tone map the linear color, gamma encode, and
// store three bytes at pixelIndex * 3 of the RGB8 output.
inline void storePixel(std::vector<std::uint8_t> &outRgb, std::size_t pixelIndex, const glm::vec3 &linearColor,
    float exposure, pbr::ToneMapping mapper)
{
    const glm::vec3 mapped = toneMapColor(linearColor, exposure, mapper);
    const std::size_t offset = pixelIndex * 3u;
    outRgb[offset + 0u] = toByte(gammaEncode(mapped.x));
    outRgb[offset + 1u] = toByte(gammaEncode(mapped.y));
    outRgb[offset + 2u] = toByte(gammaEncode(mapped.z));
}

} // namespace engine::lumen
