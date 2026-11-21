// Shared math/data helpers used by both backends and the module core. The
// shading model is split into focused headers and re-exported here:
// direct_light_shading.hpp (prepared lights + per-point resolution),
// gi_probe_sampling.hpp and reflection_field_sampling.hpp (the lumen_gi /
// lumen_reflection snapshot types and their samplers), pbr_shading.hpp (the
// CPU twin of shaders/mesh.frag) and fur_shading.hpp (the twin of
// shaders/fur.frag). This header carries what is genuinely common: the
// backend-neutral enums, the default PBR environment, frustum rejection, and
// the output encode/tone-map/error/PPM utilities.

#pragma once

#include "direct_light_shading.hpp"
#include "fur_shading.hpp"
#include "gi_probe_sampling.hpp"
#include "pbr_shading.hpp"
#include "reflection_field_sampling.hpp"
#include "render_errors.hpp"
#include "shading_inline.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/util/png.hpp>
#include <engine/util/ppm.hpp>

#include <glm/common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace renderer
{

// Device selection bias used when a renderer is created.
enum class DevicePreference : std::uint32_t
{
    Auto, // any device, discrete preferred
    Discrete,
    Integrated
};

// Presentation-queue policy for the swapchain. Auto keeps the historical
// behaviour; the explicit modes exist so frame-latency behaviour is a
// configuration decision instead of a driver default. The enum lives here
// (backend-neutral) because [renderer] present_mode is parsed by the module
// core regardless of which backends are compiled in; only the Vulkan
// backend's swapchain consumes it.
enum class SwapchainPresentMode
{
    Auto,      // FIFO when vsync, else mailbox, else immediate
    Fifo,      // always tear-free, one image on screen, blocks on the compositor
    Mailbox,   // triple buffered, latency-bounded, never blocks the CPU for long
    Immediate, // no presentation queue at all: lowest latency, may tear
};

// Gribb-Hartmann frustum rejection for a world-space bounding sphere: true
// when the sphere lies fully behind any one of the four side planes, so the
// draw cannot touch the viewport. Both backends cull with this (the planes
// are extracted once per camera), so the rejection rule is defined once.
inline bool sphereOutsideFrustum(const glm::vec4 frustumPlanes[4], const glm::vec4 &worldCenter, float worldRadius)
{
    for (std::uint32_t plane = 0; plane < 4; ++plane)
    {
        const float signedDistance = glm::dot(frustumPlanes[plane], worldCenter);
        if (std::isfinite(signedDistance) && signedDistance < -worldRadius)
        {
            return true;
        }
    }
    return false;
}

// Sun + hemisphere ambient + ACES tone mapping, used whenever the pbr
// capability is absent.
inline pbr::PbrEnvironment defaultEnvironment()
{
    const glm::vec3 direction = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
    pbr::PbrEnvironment environment{
        .sun = {
            .direction = {direction.x, direction.y, direction.z},
            .color     = {1.0f, 0.96f, 0.88f},
            .intensity = 3.0f,
            .enabled   = true
        },
        .ambient = {
            .skyColor    = {0.55f, 0.65f, 0.80f},
            .groundColor = {0.35f, 0.30f, 0.25f},
            .intensity   = 0.5f,
        },
        .exposure            = 1.0f,
        .toneMapping         = pbr::ToneMapping::AcesFilmic};
    return environment;
}

// Unsigned-byte quantization with round-to-nearest; NaN-safe through the
// clamps (toByte(NaN) would otherwise be undefined through the float->int
// conversion).
inline std::uint8_t toByte(float value)
{
    return static_cast<std::uint8_t>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
}

// Fast sRGB encode for the CPU rasterizer. glm::pow(c, 1/2.2) costs three
// powf calls per pixel - by far the most expensive op in the shading loop -
// while the output lands in an 8-bit target anyway. A 256-entry nearest
// lookup keeps the error below half an output quantization step.
//
// The tables are C++17 inline variables (dynamically initialized before the
// first use of anything defined alongside them), so the hot pixel path reads
// them without a thread-safe-initialization guard check.
namespace srgb_tables
{
struct FloatEncodeTable
{
    float values[256];
    FloatEncodeTable()
    {
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            values[index] = std::pow(static_cast<float>(index) / 255.0f, 1.0f / 2.2f);
        }
    }
};

inline const FloatEncodeTable floatEncode;
} // namespace srgb_tables

inline float fastSrgbEncode(float channel)
{
    const float clamped = channel < 0.0f ? 0.0f : (channel > 1.0f ? 1.0f : channel);
    return srgb_tables::floatEncode.values[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
}

inline glm::vec3 fastSrgbEncode(const glm::vec3 &color)
{
    return glm::vec3(fastSrgbEncode(color.r), fastSrgbEncode(color.g), fastSrgbEncode(color.b));
}

// The composed output transform for opaque pixels: fastSrgbEncode followed
// by toByte. Both stages quantize through the same 256-step grid, so the
// composition is itself a function of the grid index alone; folding the two
// tables into one byte-per-index table reproduces the exact sequence of
// operations the two-stage form performs (the intermediate float is the float
// table's entry in both cases), bit for bit, while replacing three clamps,
// three multiplies and three conversions per pixel with one lookup per
// channel.
namespace srgb_tables
{
struct ByteEncodeTable
{
    std::uint8_t values[256];
    ByteEncodeTable()
    {
        // Built from the same float encode the two-stage form uses, so every
        // entry equals toByte(fastSrgbEncode(index / 255)).
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            values[index] = toByte(std::pow(static_cast<float>(index) / 255.0f, 1.0f / 2.2f));
        }
    }
};

inline const ByteEncodeTable byteEncode;
} // namespace srgb_tables

// Encodes a tone-mapped linear color as the packed little-endian RGBA word
// the rasterizer commits (R at the lowest address, alpha 255). Channels at or
// outside [0, 1] clamp exactly like the two-stage encode does.
ENGINE_SHADING_INLINE std::uint32_t encodeColorRgba(const glm::vec3 &color)
{
    const std::uint8_t *values = srgb_tables::byteEncode.values;
    const auto channelByte = [values](float channel)
    {
        const float clamped = channel < 0.0f ? 0.0f : (channel > 1.0f ? 1.0f : channel);
        return values[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
    };
    const std::uint32_t red   = channelByte(color.r);
    const std::uint32_t green = channelByte(color.g);
    const std::uint32_t blue  = channelByte(color.b);
    return red | (green << 8) | (blue << 16) | (0xFFu << 24);
}

// Tone mappers mirroring mesh.frag's toneMap(): none / Reinhard / ACES filmic.
ENGINE_SHADING_INLINE glm::vec3 toneMapColor(glm::vec3 color, float exposure, pbr::ToneMapping mapper)
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
        color = glm::clamp((color * (2.51f * color + 0.03f)) / (color * (2.43f * color + 0.59f) + 0.14f), 0.0f, 1.0f);
        break;
    }
    return color;
}

// Writes a binary P6 PPM from an RGBA8 raster. flipVertically and
// swapRedBlue adapt the source row and channel order.
inline bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
    std::size_t sourceStrideBytes, bool flipVertically, bool swapRedBlue, RenderError *error)
{
    if (!util::writePpmRgba(filePath, width, height, rgba, sourceStrideBytes, flipVertically, swapRedBlue))
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    reportError(error, RenderError::None);
    return true;
}

// Writes a truecolor PNG from the same RGBA8 raster (see writePpm).
inline bool writePng(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
    std::size_t sourceStrideBytes, bool flipVertically, bool swapRedBlue, RenderError *error)
{
    if (!util::writePngRgba(filePath, width, height, rgba, sourceStrideBytes, flipVertically, swapRedBlue))
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    reportError(error, RenderError::None);
    return true;
}

} // namespace renderer
} // namespace engine
