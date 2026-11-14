// Public capability surface of the fur module.
//
// Fur content and policy for the renderer module's shell-fur draw path
// (see engine/renderer/capabilities.hpp FurLayerParams): deterministic
// procedural fur strand textures, fur parameter presets, and the
// screen-size LOD evaluator that decides how many shell layers a fur
// surface should draw at a given distance. The module owns no GPU state -
// consumers upload the generated pixels through the renderer's
// createTexture and submit draws through the renderer's drawFur with the
// parameters this capability produces.
//
// Everything here is consumed via ParticipantContext::acquireCapability<T>().

#ifndef ENGINE_FUR_CAPABILITIES_HPP
#define ENGINE_FUR_CAPABILITIES_HPP

#include <cstddef>
#include <cstdint>

namespace engine
{
namespace fur
{

// ABI version of the FurCapability struct below. Bump on any layout change.
// History: 1 - initial surface (presets, texture synthesis, LOD evaluator).
inline constexpr std::uint32_t furCapabilityAbi = 1u;

// Named fur looks. The demo maps its per-species controls onto these.
enum class FurPreset : std::uint32_t
{
    Plush = 0, // short dense coat, soft edges - pet / chibi look
    Shaggy,    // long uneven strands, strong length variance
    Velvety,   // very short, high density, low variance - sleek coat
    count
};

// Fur look parameters. One instance describes one fur look:
// strandLength    - strand height in object/model units; rides
//                   FurLayerParams::furHeight unchanged.
// density         - strand coverage in 0..1: what fraction of the texture's
//                   strand cells actually carry a strand (plus an undercoat).
// lengthVariance  - per-strand length spread in 0..1 (0 = uniform lengths).
// colorVariation  - per-texel albedo jitter amplitude in 0..1 (0 = flat).
// patternScale    - strand pattern repeats per UV unit (higher = finer fur).
// gravityStrength - tip droop toward world -Y; rides
//                   FurLayerParams::gravityStrength unchanged.
// densityBoost    - strand alpha edge sharpness (1 = soft wisps, ~8 = crisp
//                   thin strands); rides FurLayerParams::densityBoost.
// seed            - hash seed; the same parameters + seed always produce the
//                   same texture (the demo's regression captures rely on it).
struct FurParameters
{
    float         strandLength    = 0.035f;
    float         density         = 0.72f;
    float         lengthVariance  = 0.45f;
    float         colorVariation  = 0.18f;
    float         patternScale    = 6.0f;
    float         gravityStrength = 0.0f;
    float         densityBoost    = 6.0f;
    std::uint32_t seed            = 1u;
};

// One procedural fur texture request: RGBA8, width*height*4 bytes written
// into the caller's buffer (tightly packed rows, top-left first, Repeat
// addressing assumed - the pattern is tileable in both axes). 16..2048
// pixels per side.
struct FurTextureRequest
{
    FurParameters parameters{};
    std::uint32_t width  = 256;
    std::uint32_t height = 256;
};

// The screen-size LOD decision for one fur surface this frame. The consumer
// maps it onto renderer::FurLayerParams: layerCount -> layerCount (the
// renderer clamps it to [renderer] fur_max_layers), furHeight * furHeightScale
// -> furHeight, outerLayerFade -> outerLayerFade.
struct FurLodDecision
{
    std::uint32_t layerCount     = 8;
    float         outerLayerFade = 1.0f;
    float         furHeightScale = 1.0f;
    std::uint32_t lodLevel       = 0;
};

// The "fur" capability: pure functions over configuration + arguments, no
// GPU state, safe to call at asset-build time and per frame. All functions
// are main-thread only (like every engine capability).
struct FurCapability
{
    static constexpr const char *capabilityName = "fur";

    std::uint32_t abi = furCapabilityAbi;

    void *self = nullptr;

    // Parameters of a named preset; out-of-range preset ids fall back to
    // Plush.
    FurParameters (*presetParameters)(void *self, FurPreset preset);

    // Generates one tiled RGBA8 fur strand texture (rgb = strand albedo
    // variation multiplying the material's vertex colors and base color
    // factor, alpha = the per-texel strand height field the shell layers
    // crop against). Deterministic for identical parameters + seed. Returns
    // false and logs when the request is malformed or the buffer is too
    // small (capacity must be >= width * height * 4).
    bool (*generateFurTexture)(
        void *self, const FurTextureRequest *request, std::uint8_t *outPixelsRgba, std::size_t outCapacityBytes);

    // Screen-size LOD for one fur surface: angularSize is the surface's
    // world bounding radius divided by its distance to the camera (the same
    // metric the headless backend's mesh LOD uses; roughly the fraction of
    // the view the surface spans). The decision scales the shell layer count
    // down with distance and fades the outermost layer across each band
    // boundary so the reduced layer count dissolves the strand tips instead
    // of clipping them.
    FurLodDecision (*evaluateFurLod)(void *self, float angularSize);
};

} // namespace fur
} // namespace engine

#endif
