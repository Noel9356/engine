// lumen_reflection screen-space field sampling shared by the module core and
// the headless pixel path: the borrowed snapshot types copied from the
// lumen_reflection capability and the bilinear, roughness-band-gated sample
// that turns the field into per-pixel indirect specular radiance.

#pragma once

#include <engine/lumen_reflection/capabilities.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace renderer
{

// Snapshot of the lumen_reflection screen-space field borrowed by the
// renderer: the indirect specular radiance buffer produced for one camera.
// Backends copy the data synchronously inside setReflectionField(), so the
// borrowed view only needs to stay valid for that call. A null/empty field
// means "no reflections": the shading model keeps its pre-hook behavior
// bit-identical.
struct ReflectionField
{
    lumen_reflection::ReflectionFieldHeader header{};
    // Borrowed; width*height*reflectionFieldTexelFloats linear HDR texels
    // (radiance RGB interleaved with the texel roughness, per the
    // lumen_reflection ABI).
    const float *radiance = nullptr;

    bool valid() const
    {
        return radiance != nullptr && header.width != 0 && header.height != 0;
    }
};

// The reflection field flattened for the pixel path: the field's own
// view-projection built once per snapshot instead of per shaded sample, plus
// the dimensions and radiance array the bilinear walk reads.
struct PreparedReflectionField
{
    glm::mat4     viewProjection{0.0f};
    std::uint32_t width  = 0;
    std::uint32_t height = 0;
    // The module's roughness band edge (roughness.max_trace): pixels
    // rougher than this receive no specular environment response, so the
    // field sample early-outs for them.
    float         maxRoughness = 0.4f;
    // Borrowed from the owning backend copy (valid for the frame; see
    // ReflectionField's borrowed-view contract).
    const float *radiance = nullptr;
    bool         valid     = false;
};

inline PreparedReflectionField prepareReflectionField(const ReflectionField &field)
{
    PreparedReflectionField prepared;
    if (field.valid())
    {
        prepared.viewProjection = glm::make_mat4(field.header.viewProjection);
        prepared.width          = field.header.width;
        prepared.height         = field.header.height;
        prepared.maxRoughness   = field.header.maxRoughness;
        prepared.radiance       = field.radiance;
        prepared.valid          = true;
    }
    return prepared;
}

// Bilinear sample of the reflection field at a shaded point: the point is
// projected through the field's own camera, and the four surrounding texels
// are blended (the field is lower-resolution than the framebuffer by
// design; the screen traces inside the module already accounted for the
// geometry the pixel sees). `pixelRoughness` is the shaded pixel's clamped
// roughness and gates the sample twice: a pixel rougher than the field's
// maxRoughness band receives no specular environment response at all (its
// surface never traced one, exactly like the module's own band rule), and
// the four surrounding texels are refused when their surface roughness
// leaves the pixel's band (lumen_reflection::reflectionFieldRoughnessBand)
// or carries the neutral value (no traced response: background, unlit,
// above the traced roughness band), so the bilinear weights renormalize
// over the surviving texels and a fully-refused neighborhood reads black -
// the documented sample-gate contract of the field's texel layout. The
// band gate keeps one material's traced response from bleeding across a
// silhouette onto a rougher neighbor's pixels (the field is
// lower-resolution, so a reflective surface's texels unavoidably touch the
// border of whatever sits behind it). Returns black when the point falls
// outside the field (behind the camera or off its frustum).
// outTraced reports whether any texel of the footprint survived the gates
// (the pixel's material actually has a traced response there). Transmissive
// draws use it to composite the traced response instead of alpha blending:
// the field's transmitted share already carries what the surface shows of
// the scene behind it, so blending the raw background in as well would
// double it.
inline glm::vec3 sampleReflectionField(const PreparedReflectionField &field, const glm::vec3 &worldPosition,
    float pixelRoughness, bool *outTraced = nullptr)
{
    if (!field.valid)
    {
        return glm::vec3(0.0f);
    }
    if (pixelRoughness > field.maxRoughness)
    {
        return glm::vec3(0.0f);
    }
    const glm::mat4 &viewProjection = field.viewProjection;
    const glm::vec4 clip = viewProjection * glm::vec4(worldPosition, 1.0f);
    if (clip.w <= 0.05f)
    {
        return glm::vec3(0.0f);
    }
    const float u = (clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(field.width) - 0.5f;
    const float v = (0.5f - clip.y / clip.w * 0.5f) * static_cast<float>(field.height) - 0.5f;
    if (u < 0.0f || v < 0.0f || u > static_cast<float>(field.width) - 1.0f
        || v > static_cast<float>(field.height) - 1.0f)
    {
        return glm::vec3(0.0f);
    }
    const std::uint32_t x0 = static_cast<std::uint32_t>(std::floor(u));
    const std::uint32_t y0 = static_cast<std::uint32_t>(std::floor(v));
    const std::uint32_t x1 = std::min(x0 + 1u, field.width - 1u);
    const std::uint32_t y1 = std::min(y0 + 1u, field.height - 1u);
    const float       fx = u - static_cast<float>(x0);
    const float       fy = v - static_cast<float>(y0);
    float roughness00 = 1.0f, roughness10 = 1.0f, roughness01 = 1.0f, roughness11 = 1.0f;
    const auto texelAt = [&field](std::uint32_t x, std::uint32_t y, bool &traced, float &texelRoughness)
    {
        const float *texel
            = field.radiance
              + (static_cast<std::size_t>(y) * field.width + x) * lumen_reflection::reflectionFieldTexelFloats;
        // The trailing roughness is the sample gate: the module reports the
        // neutral value for every texel it never traced a response into, and
        // blending those in would smear background radiance across the
        // neighborhood (or resurrect a stale frame's field when the module
        // skipped an evaluate).
        texelRoughness = texel[3];
        traced = texel[3] < lumen_reflection::reflectionFieldNeutralRoughness;
        return glm::vec3(texel[0], texel[1], texel[2]);
    };
    bool traced00 = false, traced10 = false, traced01 = false, traced11 = false;
    const glm::vec3 t00 = texelAt(x0, y0, traced00, roughness00);
    const glm::vec3 t10 = texelAt(x1, y0, traced10, roughness10);
    const glm::vec3 t01 = texelAt(x0, y1, traced01, roughness01);
    const glm::vec3 t11 = texelAt(x1, y1, traced11, roughness11);
    // Roughness-band gate on top of the traced/neutral gate: a texel traced
    // for a different lobe than the pixel's belongs to another material
    // (the module traced every texel in its own band, so the difference can
    // only come from a silhouette neighbor).
    const float band      = lumen_reflection::reflectionFieldRoughnessBand;
    traced00 = traced00 && std::fabs(roughness00 - pixelRoughness) <= band;
    traced10 = traced10 && std::fabs(roughness10 - pixelRoughness) <= band;
    traced01 = traced01 && std::fabs(roughness01 - pixelRoughness) <= band;
    traced11 = traced11 && std::fabs(roughness11 - pixelRoughness) <= band;
    // Refused texels drop out of the blend; the surviving weights
    // renormalize, and an empty neighborhood (all four refused) reads black.
    const float w00 = traced00 ? (1.0f - fx) * (1.0f - fy) : 0.0f;
    const float w10 = traced10 ? fx * (1.0f - fy) : 0.0f;
    const float w01 = traced01 ? (1.0f - fx) * fy : 0.0f;
    const float w11 = traced11 ? fx * fy : 0.0f;
    const float weightSum = w00 + w10 + w01 + w11;
    if (weightSum <= 0.0f)
    {
        if (outTraced != nullptr)
        {
            *outTraced = false;
        }
        return glm::vec3(0.0f);
    }
    if (outTraced != nullptr)
    {
        *outTraced = true;
    }
    return (t00 * w00 + t10 * w10 + t01 * w01 + t11 * w11) / weightSum;
}

} // namespace renderer
} // namespace engine
