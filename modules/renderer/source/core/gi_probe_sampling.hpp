// lumen_gi probe-grid sampling shared by both backends: the borrowed snapshot
// types the renderer module copies from the lumen_gi capability, the prepared
// per-snapshot level descriptors, and the trilinear validity-weighted SH2
// irradiance walk the headless pixel path runs (the GLSL twin lives in
// shaders/mesh.frag). The SH2 convolution is the same as
// lumen_gi::evaluateProbeIrradiance; kept textually in sync because both
// sides must agree on the exported probe layout.

#pragma once

#include "shading_inline.hpp"

#include <engine/lumen_gi/capabilities.hpp>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace renderer
{

// Probes a GiProbeGrid holds at most (two 9^3 clip levels); the Vulkan probe
// storage buffer is sized for this bound.
inline constexpr std::uint32_t maxGiProbes = 2u * 9u * 9u * 9u;
// Floats per probe in the packed storage-buffer/std430 record: 1x vec4
// (position.xyz + validity) + 7x vec4 (the 27 SH2 floats plus one pad).
inline constexpr std::uint32_t giProbeRecordVec4s = 8u;

// One level of the exported probe cache: a regular dims^3 lattice anchored at
// anchor with uniform spacing, index = x + dims.x * (y + dims.y * z), the
// ordering copyProbes documents (z-major, level 0 first then level 1).
struct GiProbeLevel
{
    glm::vec3     anchor{0.0f};
    glm::ivec3    dims{0};
    float         spacing = 1.0f;
    // Reciprocal of spacing, precomputed per snapshot: the per-pixel probe
    // walk's tangent-plane factor multiplies by it once per corner, and a
    // per-corner division would dominate the walk's cost.
    float         inverseSpacing = 1.0f;
    std::uint32_t offset  = 0; // first probe index inside GiProbeGrid::probes
};

// Snapshot of the lumen_gi world-space radiance cache borrowed by the
// renderer: a GiProbeHeader plus the probe array behind it. Backends copy the
// data synchronously inside setProbeGrid(), so the borrowed view only needs
// to stay valid for that call. A null/empty snapshot means "no GI": the
// shading model keeps its legacy hemispheric ambient (bit-identical to the
// pre-hook behavior).
struct GiProbeGrid
{
    lumen_gi::GiProbeHeader  header{};
    const lumen_gi::GiProbe *probes     = nullptr; // borrowed; header.probeCount entries
    std::uint32_t            probeCount = 0;

    // Precomputed level descriptors, filled once per snapshot by
    // buildProbeLevels(); giProbeLevelFor reads them instead of rebuilding
    // both structs for every shaded pixel.
    GiProbeLevel level0{};
    GiProbeLevel level1{};
    bool         levelsBuilt = false;

    bool valid() const
    {
        return probes != nullptr && probeCount != 0;
    }
};

// Builds both level descriptors from the snapshot's header and probe array;
// backends call this once per setProbeGrid so per-pixel shading can pick a
// level without re-reading probe positions. levelsBuilt reports whether the
// snapshot carries a usable pair of levels.
inline bool buildProbeLevels(GiProbeGrid &grid)
{
    const lumen_gi::GiProbeHeader &header = grid.header;
    grid.levelsBuilt                      = false;
    if (!grid.valid() || header.level0Dimensions[0] < 2u || header.level1Dimensions[0] < 2u)
    {
        return false;
    }
    const std::uint32_t level0Count = header.level1ProbeOffset - header.level0ProbeOffset;
    if (header.level0ProbeOffset >= grid.probeCount || level0Count == 0
        || header.level1ProbeOffset >= grid.probeCount)
    {
        return false;
    }

    grid.level0.anchor  = glm::make_vec3(grid.probes[header.level0ProbeOffset].position);
    grid.level0.dims    = glm::ivec3(static_cast<int>(header.level0Dimensions[0]),
        static_cast<int>(header.level0Dimensions[1]), static_cast<int>(header.level0Dimensions[2]));
    grid.level0.spacing = header.level0Spacing;
    grid.level0.inverseSpacing = 1.0f / std::max(header.level0Spacing, 1.0e-5f);
    grid.level0.offset  = header.level0ProbeOffset;

    grid.level1.anchor  = glm::make_vec3(grid.probes[header.level1ProbeOffset].position);
    grid.level1.dims    = glm::ivec3(static_cast<int>(header.level1Dimensions[0]),
        static_cast<int>(header.level1Dimensions[1]), static_cast<int>(header.level1Dimensions[2]));
    grid.level1.spacing = header.level1Spacing;
    grid.level1.inverseSpacing = 1.0f / std::max(header.level1Spacing, 1.0e-5f);
    grid.level1.offset  = header.level1ProbeOffset;

    grid.levelsBuilt = true;
    return true;
}

// The probe grid flattened for the pixel path: both level descriptors plus
// the level-0 bounds-test constants (the level pick's only per-snapshot
// data) and the probe array the trilinear walk reads. Filled once per
// snapshot (setProbeGrid time) so the pixel path spends its math on the
// actual probe walk; the values are identical to the per-pixel derivations
// they replaced.
struct PreparedGiProbeGrid
{
    GiProbeLevel level0{};
    GiProbeLevel level1{};
    glm::vec3    anchor0{0.0f};
    glm::vec3    extent0{0.0f};
    float        halfCell0 = 0.0f;
    // Borrowed from the owning backend copy (valid for the frame; see
    // GiProbeGrid's borrowed-view contract).
    const lumen_gi::GiProbe *probes     = nullptr;
    std::uint32_t            probeCount = 0;
    bool                     valid      = false;
};

inline PreparedGiProbeGrid prepareGiProbeGrid(GiProbeGrid &grid)
{
    PreparedGiProbeGrid prepared;
    prepared.probes     = grid.probes;
    prepared.probeCount = grid.probeCount;
    if (buildProbeLevels(grid))
    {
        prepared.level0    = grid.level0;
        prepared.level1    = grid.level1;
        prepared.anchor0   = grid.level0.anchor;
        prepared.extent0   = glm::vec3(grid.level0.dims - 1) * grid.level0.spacing;
        prepared.halfCell0 = grid.level0.spacing * 0.5f;
        prepared.valid     = true;
    }
    return prepared;
}

// Picks the lattice level for a world position: level 0 inside its
// half-cell-extended bounds, level 1 everywhere else (its trilinear lookup
// clamps to the boundary probes, so distant geometry still receives the
// coarse ambient). Returns false when the grid holds no usable levels.
inline bool giProbeLevelFor(const PreparedGiProbeGrid &grid, const glm::vec3 &position, GiProbeLevel &outLevel)
{
    if (!grid.valid)
    {
        return false;
    }
    const glm::vec3 local        = position - grid.anchor0;
    const bool      insideLevel0 = glm::all(glm::bvec4(local.x >= -grid.halfCell0, local.y >= -grid.halfCell0,
        local.z >= -grid.halfCell0, true))
                                && glm::all(glm::bvec4(local.x <= grid.extent0.x + grid.halfCell0,
                                     local.y <= grid.extent0.y + grid.halfCell0,
                                     local.z <= grid.extent0.z + grid.halfCell0, true));
    outLevel = insideLevel0 ? grid.level0 : grid.level1;
    return true;
}

// The SH2 basis evaluated at one direction. The band-1 terms are the
// direction's components and the band-2 terms their products; a probe
// footprint evaluation (giProbeGridIrradiance) shades several probes against
// the SAME normal, so the products are computed once per lookup and shared by
// every probe instead of being re-derived per probe.
struct Sh2Basis
{
    float x  = 0.0f;
    float y  = 0.0f;
    float z  = 0.0f;
    float xy = 0.0f;
    float yz = 0.0f;
    float zz3m1 = 0.0f; // 3zz - 1
    float xz = 0.0f;
    float xxmyy = 0.0f; // xx - yy
};

inline Sh2Basis sh2BasisFor(const glm::vec3 &normal)
{
    const float x  = normal.x;
    const float y  = normal.y;
    const float z  = normal.z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    Sh2Basis basis;
    basis.x     = x;
    basis.y     = y;
    basis.z     = z;
    basis.xy    = x * y;
    basis.yz    = y * z;
    basis.zz3m1 = 3.0f * zz - 1.0f;
    basis.xz    = x * z;
    basis.xxmyy = xx - yy;
    return basis;
}

// Cosine-lobed SH2 irradiance at one probe (the same convolution as
// lumen_gi::evaluateProbeIrradiance; kept textually in sync because the GLSL
// mirror in shaders/mesh.frag must match this CPU path like the rest of the
// shading model). sh2 is 27 floats in the GiProbe [basis][channel] order; the
// basis comes from sh2BasisFor so a shared normal evaluates its products once.
//
// The three color channels run as packed lanes of one vector each (the SH
// layout is [basis][channel], so each basis term's three channel coefficients
// are contiguous): every lane performs the exact scalar operation sequence the
// per-channel form performed - same products, same addition grouping, hence
// the same rounding - while issuing a third of the instructions.
ENGINE_SHADING_INLINE glm::vec3 giEvaluateSh2Irradiance(const float sh2[27], const Sh2Basis &basis)
{
    constexpr float k0 = 3.141592654f; // pi      (band 0)
    constexpr float k1 = 2.094395102f; // 2pi/3   (band 1)
    constexpr float k2 = 0.785398163f; // pi/4    (band 2)

    const glm::vec3 band0(sh2[0], sh2[1], sh2[2]);
    const glm::vec3 band1Y(sh2[3], sh2[4], sh2[5]);
    const glm::vec3 band1Z(sh2[6], sh2[7], sh2[8]);
    const glm::vec3 band1X(sh2[9], sh2[10], sh2[11]);
    const glm::vec3 band2XY(sh2[12], sh2[13], sh2[14]);
    const glm::vec3 band2YZ(sh2[15], sh2[16], sh2[17]);
    const glm::vec3 band2ZZ(sh2[18], sh2[19], sh2[20]);
    const glm::vec3 band2XZ(sh2[21], sh2[22], sh2[23]);
    const glm::vec3 band2XXYY(sh2[24], sh2[25], sh2[26]);

    // Grouped exactly as the scalar per-channel form grouped its terms:
    // (k0*band0 + k1*((Y*y + Z*z) + X*x)) + k2*(((XY*xy + YZ*yz) + ZZ*zz3m1) + XZ*xz) + XXYY*xxmyy).
    return (k0 * band0 + k1 * ((band1Y * basis.y + band1Z * basis.z) + band1X * basis.x))
           + (k2
                 * ((((band2XY * basis.xy + band2YZ * basis.yz) + band2ZZ * basis.zz3m1) + band2XZ * basis.xz)
                     + band2XXYY * basis.xxmyy));
}

// Trilinear probe-grid irradiance for one shaded point, validity- and
// tangent-plane-weighted. Returns false when the grid holds no valid probes
// along this footprint (the caller keeps its legacy ambient); outWeight is the
// validity coverage of the footprint in 0..1, used to blend GI against the
// hemispheric ambient. The tangent-plane term comes from the shared
// lumen_gi::probeTangentPlaneFactor, the exported-grid convention the
// provider's world cache and lumen_reflection's hit lighting also apply.
ENGINE_SHADING_INLINE bool giProbeGridIrradiance(const PreparedGiProbeGrid &grid, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outIrradiance, float &outWeight)
{
    outIrradiance = glm::vec3(0.0f);
    outWeight     = 0.0f;
    GiProbeLevel  level;
    if (!giProbeLevelFor(grid, position, level))
    {
        return false;
    }

    // The SH2 basis is shared by the single blended evaluation (same normal),
    // so its products are computed once here.
    const Sh2Basis basis = sh2BasisFor(normal);

    const glm::vec3 cell     = (position - level.anchor) / level.spacing;
    const glm::ivec3 base    = glm::clamp(glm::ivec3(glm::floor(cell)), glm::ivec3(0), level.dims - glm::ivec3(2));
    const glm::vec3 fraction = glm::clamp(cell - glm::vec3(base), 0.0f, 1.0f);

    // The irradiance sum evaluates each valid probe corner against the shared
    // basis and accumulates its weighted result into ONE vec3 accumulator
    // (three live floats, everything stays in registers; blending the corners'
    // coefficients first would hold all 27 coefficient floats live and spill);
    // the loop keeps its early-outs (probe index range, zero-weight corners)
    // so invalid probes never touch the arithmetic at all. The tangent-plane
    // term runs in its factored form (see lumen_gi::probeTangentPlaneFactor):
    // the pixel-level base alignment (measured against the cell's base corner,
    // anchor + base * spacing) plus the corner's unit offsets dotted with the
    // normal, so no corner position is rebuilt per probe.
    const float baseAlignment
        = glm::dot(normal, level.anchor + glm::vec3(base) * level.spacing - position) * level.inverseSpacing;
    float       weightSum     = 0.0f;
    glm::vec3   irradianceSum(0.0f);
    for (int cornerZ = 0; cornerZ < 2; ++cornerZ)
    {
        for (int cornerY = 0; cornerY < 2; ++cornerY)
        {
            for (int cornerX = 0; cornerX < 2; ++cornerX)
            {
                const glm::ivec3 corner = base + glm::ivec3(cornerX, cornerY, cornerZ);
                const std::size_t probeIndex = static_cast<std::size_t>(level.offset)
                    + static_cast<std::size_t>(corner.x + level.dims.x * (corner.y + level.dims.y * corner.z));
                if (probeIndex >= grid.probeCount)
                {
                    continue;
                }
                const lumen_gi::GiProbe &probe = grid.probes[probeIndex];
                const float latticeWeight = (cornerX != 0 ? fraction.x : 1.0f - fraction.x)
                                          * (cornerY != 0 ? fraction.y : 1.0f - fraction.y)
                                          * (cornerZ != 0 ? fraction.z : 1.0f - fraction.z);
                float weight = latticeWeight * probe.validity;
                if (weight <= 0.0f)
                {
                    continue;
                }
                float alignment = baseAlignment;
                if (cornerX != 0)
                {
                    alignment += normal.x;
                }
                if (cornerY != 0)
                {
                    alignment += normal.y;
                }
                if (cornerZ != 0)
                {
                    alignment += normal.z;
                }
                weight *= lumen_gi::probeTangentPlaneAlignmentFactor(alignment);
                if (weight <= 1.0e-8f)
                {
                    continue;
                }
                irradianceSum += weight * giEvaluateSh2Irradiance(&probe.sh2[0][0], basis);
                weightSum += weight;
            }
        }
    }
    if (weightSum <= 0.0f)
    {
        return false;
    }
    outIrradiance = irradianceSum * (1.0f / weightSum);
    outWeight     = glm::clamp(weightSum, 0.0f, 1.0f);
    return true;
}

} // namespace renderer
} // namespace engine
