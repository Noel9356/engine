// radiance_cache screen-space gather field sampling shared by the module core
// and the headless pixel path: the borrowed snapshot types copied from the
// radiance_cache.gather capability and the validity-gated bilinear sample
// that turns the field into per-pixel indirect irradiance. The field is the
// module's final-gather result (screen probes, short surface-cache rays,
// depth-plane interpolation, temporal accumulation, bilateral filter), so it
// carries the contact-scale structure the coarse probe lattice cannot
// resolve - short-range occlusion at surface meet lines and the caustic
// irradiance the photon pass deposited on the card texels.

#pragma once

#include "finite_math.hpp"
#include "shading_inline.hpp"

#include <engine/radiance_cache/capabilities.hpp>

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

// Snapshot of the radiance_cache gather field borrowed by the renderer: the
// per-pixel indirect irradiance produced for one camera. Backends copy the
// data synchronously inside setGatherField(), so the borrowed view only
// needs to stay valid for that call. A null/empty field means "no gather":
// the shading model keeps its probe-walk ambient bit-identical.
struct GatherField
{
    radiance_cache::RadianceGatherHeader header{};
    // Borrowed; width*height*radianceGatherTexelFloats linear floats (the
    // irradiance RGB interleaved with the texel validity, per the
    // radiance_cache.gather ABI).
    const float *irradiance = nullptr;

    bool valid() const
    {
        return irradiance != nullptr && header.width != 0 && header.height != 0;
    }
};

// The gather field flattened for the pixel path: the field's own
// view-projection built once per snapshot instead of per shaded sample, plus
// the dimensions the bilinear walk reads.
struct PreparedGatherField
{
    glm::mat4     viewProjection{0.0f};
    std::uint32_t width  = 0;
    std::uint32_t height = 0;
    // Borrowed from the owning backend copy (valid for the frame; see
    // GatherField's borrowed-view contract).
    const float *irradiance = nullptr;
    // Multiplier on the texels' contact-occlusion channel
    // (renderer.gather_occlusion_strength): 1 keeps the module's reported
    // occlusion, above 1 deepens the contact bands.
    float occlusionStrength = 1.0f;
    bool         valid      = false;
};

inline PreparedGatherField prepareGatherField(const GatherField &field)
{
    PreparedGatherField prepared;
    if (field.valid())
    {
        // The provider-owned view-projection must be finite before the pixel
        // path projects through it: a NaN/Inf entry (a malformed snapshot)
        // would otherwise survive the bounds test below as NaN screen
        // coordinates - every NaN comparison is false, so the rejection
        // chain never fires - and reach the uint32 texel cast as undefined
        // behaviour with a wild index. The bit-level test cannot be folded
        // away by the rasterizer unit's math flags (see finite_math.hpp);
        // a non-finite matrix keeps the field inactive for this snapshot,
        // which is the documented "no gather" fallback (probe-walk
        // ambient, bit-identical to a missing capability).
        if (!isFiniteRange(field.header.viewProjection, 16u))
        {
            return prepared;
        }
        prepared.viewProjection = glm::make_mat4(field.header.viewProjection);
        prepared.width          = field.header.width;
        prepared.height         = field.header.height;
        prepared.irradiance     = field.irradiance;
        prepared.valid          = true;
    }
    return prepared;
}

// Bilinear sample of the gather field at a shaded point: the point is
// projected through the field's own camera and the four surrounding texels
// are blended. The trailing validity gates every footprint texel: background
// texels and texels whose neighborhood never produced a sample are refused
// (the renormalizing blend of the reflection field sample), so a surface at
// the screen edge keeps its probe-walk ambient instead of reading a darkened
// mix. Returns false (outCoverage 0) when the point falls outside the field
// (behind the camera or off its frustum) or every footprint texel was
// refused; outCoverage otherwise reports the surviving weight fraction the
// caller blends against its fallback ambient.
ENGINE_SHADING_INLINE bool sampleGatherField(const PreparedGatherField &field, const glm::vec3 &worldPosition,
    glm::vec3 &outIrradiance, float &outCoverage, float &outOcclusion)
{
    outIrradiance = glm::vec3(0.0f);
    outCoverage   = 0.0f;
    outOcclusion  = 1.0f;
    if (!field.valid)
    {
        return false;
    }
    const glm::mat4 &viewProjection = field.viewProjection;
    const glm::vec4 clip = viewProjection * glm::vec4(worldPosition, 1.0f);
    if (clip.w <= 0.05f)
    {
        return false;
    }
    const float u = (clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(field.width) - 0.5f;
    const float v = (0.5f - clip.y / clip.w * 0.5f) * static_cast<float>(field.height) - 0.5f;
    // Per-point stochastic sample offset (up to half a texel, world-anchored
    // hash): the field is a texel lattice, and every pixel reading it at the
    // exact projected texel center keeps whatever correlation the field's own
    // pipeline (screen probes at a stride, the bilateral filter's kernel
    // pitch, validity renormalization) bakes into that lattice - a faint
    // screen-pitch stipple on broad dark flats even through the smoothstep
    // blend. Offsetting every sample by an independent sub-texel amount
    // decorrelates neighboring pixels' reads: the lattice becomes noise the
    // eye averages instead of a pattern it traces. Deterministic in the
    // world position, so a converged frame stays bit-stable and nothing
    // shimmers; the offset never moves a sample more than half a texel, so
    // contact-scale structure (the meet-line darkening the field resolves)
    // keeps its position within one field texel.
    const float gatherHash = std::fmod(
        std::sin(worldPosition.x * 41.631f + worldPosition.y * 23.137f + worldPosition.z * 7.543f) * 17341.2589f,
        1.0f);
    const float gatherHash01 = gatherHash - std::floor(gatherHash);
    const float jitterU      = gatherHash01 - 0.5f;
    const float secondHash   = gatherHash01 * 7.61f + 0.37f;
    const float jitterV      = (secondHash - std::floor(secondHash)) - 0.5f;
    const float uj = u + jitterU;
    const float vj = v + jitterV;
    // Conjunction form (all four comparisons must HOLD): a NaN coordinate
    // fails `u >= 0.0f` and is rejected, while the disjunction form it
    // replaces (`u < 0.0f || ...`) lets NaN slip through every comparison
    // and continue into the uint32 floor cast. NaN can still arise here with
    // a finite matrix when extreme world positions overflow the projection
    // and the terms cancel (Inf - Inf), so the belt stays even though
    // prepareGatherField gates the matrix itself.
    if (!(uj >= 0.0f && vj >= 0.0f && uj <= static_cast<float>(field.width) - 1.0f
            && vj <= static_cast<float>(field.height) - 1.0f))
    {
        return false;
    }
    const std::uint32_t x0 = static_cast<std::uint32_t>(std::floor(uj));
    const std::uint32_t y0 = static_cast<std::uint32_t>(std::floor(vj));
    const std::uint32_t x1 = std::min(x0 + 1u, field.width - 1u);
    const std::uint32_t y1 = std::min(y0 + 1u, field.height - 1u);
    // Smoothstep per axis: the field spans field_scale screen pixels per
    // texel, and plain bilinear weights carry the texel's diamond as a
    // screen-aligned lattice at exactly that pitch (the grid read every
    // broad flat shows). The curve keeps the blend's endpoints while making
    // the joins C1, hiding the tile without blurring the content.
    const float       fxRaw = uj - static_cast<float>(x0);
    const float       fyRaw = vj - static_cast<float>(y0);
    const float       fx = fxRaw * fxRaw * (3.0f - 2.0f * fxRaw);
    const float       fy = fyRaw * fyRaw * (3.0f - 2.0f * fyRaw);
    const auto texelAt = [&field](std::uint32_t x, std::uint32_t y, bool &valid, float &occlusion)
    {
        const float *texel = field.irradiance
            + (static_cast<std::size_t>(y) * field.width + x) * radiance_cache::radianceGatherTexelFloats;
        // The trailing validity is the sample gate: the module reports zero
        // for texels with no resolved sample, and blending those in would
        // smear the refusal across the neighborhood. The contact occlusion
        // rides alongside (1 = open hemisphere).
        valid = texel[3] > 0.0f;
        occlusion = texel[4];
        return glm::vec3(texel[0], texel[1], texel[2]);
    };
    bool valid00 = false, valid10 = false, valid01 = false, valid11 = false;
    float oc00 = 1.0f, oc10 = 1.0f, oc01 = 1.0f, oc11 = 1.0f;
    const glm::vec3 t00 = texelAt(x0, y0, valid00, oc00);
    const glm::vec3 t10 = texelAt(x1, y0, valid10, oc10);
    const glm::vec3 t01 = texelAt(x0, y1, valid01, oc01);
    const glm::vec3 t11 = texelAt(x1, y1, valid11, oc11);
    // Refused texels drop out of the blend; the surviving weights
    // renormalize, and an empty neighborhood reports no coverage so the
    // caller keeps its fallback ambient.
    const float w00 = valid00 ? (1.0f - fx) * (1.0f - fy) : 0.0f;
    const float w10 = valid10 ? fx * (1.0f - fy) : 0.0f;
    const float w01 = valid01 ? (1.0f - fx) * fy : 0.0f;
    const float w11 = valid11 ? fx * fy : 0.0f;
    const float weightSum = w00 + w10 + w01 + w11;
    if (weightSum <= 0.0f)
    {
        return false;
    }
    outIrradiance = (t00 * w00 + t10 * w10 + t01 * w01 + t11 * w11) / weightSum;
    outCoverage   = weightSum;
    // The occlusion channel is 1 = open hemisphere, so the strength knob
    // scales the DARKENING (1 - occlusion), not the channel itself; above 1
    // deepens the contact bands, 1 keeps the module's reported occlusion.
    outOcclusion  = glm::clamp(1.0f - (1.0f - (oc00 * w00 + oc10 * w10 + oc01 * w01 + oc11 * w11) / weightSum)
            * field.occlusionStrength,
        0.0f,
        1.0f);
    return true;
}

} // namespace renderer
} // namespace engine
