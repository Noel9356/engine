// Public capability surface of the pbr module.
//
// Declares the "pbr" capability: the shared PBR environment (sun, ambient
// hemisphere, exposure, tone mapping) and legacy-model-material to PBR
// conversion. Consumers bind it via ParticipantContext::acquireCapability<>.

#ifndef ENGINE_PBR_CAPABILITIES_HPP
#define ENGINE_PBR_CAPABILITIES_HPP

#include <engine/models/material.hpp>
#include <engine/render/material.hpp>

#include <cstdint>

namespace engine
{
namespace pbr
{

// ABI version of PbrCapability; bump when any struct in this header changes.
inline constexpr std::uint32_t pbrCapabilityAbi = 5u;

// Tone mapping operator applied to the HDR color before display: linear
// passthrough, Reinhard (x / (1 + x)), or the ACES filmic curve.
enum class ToneMapping : std::uint32_t
{
    None = 0,
    Reinhard,
    AcesFilmic
};

// Directional sun light. direction is stored normalized (the module
// normalizes on assignment); color is linear RGB; a disabled sun is skipped.
struct SunLight
{
    float direction[3] = {0.4f, 0.8f, 0.3f};
    float color[3]     = {1.0f, 0.96f, 0.88f};
    float intensity    = 3.0f;
    bool  enabled      = true;
};

// Ambient light for the two supported modes: hemisphere shading between
// skyColor and groundColor, or a uniform color (both fields set equal).
struct AmbientLight
{
    float skyColor[3]    = {0.55f, 0.65f, 0.80f};
    float groundColor[3] = {0.35f, 0.30f, 0.25f};
    float intensity      = 0.5f;
};

// The module-owned environment state. Stored normalized (unit sun direction,
// non-negative colors, positive exposure) and versioned so consumers can
// detect changes by polling environmentVersion().
struct PbrEnvironment
{
    SunLight     sun;
    AmbientLight ambient;
    float        exposure    = 1.0f;
    ToneMapping  toneMapping = ToneMapping::AcesFilmic;
};

// Knobs for converting legacy Phong-style models::ModelMaterial data into
// PBR form; the scales multiply the corresponding source values.
struct ModelMaterialConversionOptions
{
    float baseColorScale[3]     = {1.0f, 1.0f, 1.0f};
    float emissiveScale         = 1.0f;
    float metallicScale         = 1.0f;
    float roughnessScale        = 1.0f;
    float minimumRoughness      = 0.045f; // floor for the converted roughness
    bool  useSpecularAsMetallic = false;  // metallic := max(specular) if source lacks PBR factors and metallic is 0
    bool  decodeBaseColorSrgb   = false;  // decode baseColor from sRGB before scaling
    bool  decodeEmissiveSrgb    = false;  // decode emissive from sRGB before scaling
};

// Result of one conversion: the render::MeshMaterial the renderer consumes
// plus the source's non-PBR scalar properties (opacity, sheen, clearcoat,
// anisotropy, ...) preserved for consumers that still need them.
struct ConvertedModelMaterial
{
    render::MeshMaterial material;
    float                ambientColor[3]    = {0.0f, 0.0f, 0.0f};
    float                transmittance[3]   = {0.0f, 0.0f, 0.0f};
    float                opacity            = 1.0f;
    float                indexOfRefraction  = 1.5f;
    float                shininess          = 0.0f;
    float                sheen              = 0.0f;
    float                clearcoat          = 0.0f;
    float                clearcoatRoughness = 0.0f;
    float                anisotropy         = 0.0f;
    float                anisotropyRotation = 0.0f;
};

// The "pbr" capability: function pointers bound to the module singleton
// (self is the first argument of every entry).
struct PbrCapability
{
    static constexpr const char *capabilityName = "pbr";

    std::uint32_t abi  = pbrCapabilityAbi;
    void         *self = nullptr;

    // Current environment snapshot and its version; the version increments
    // only when stored values actually change.
    const PbrEnvironment *(*environment)(void *self);
    std::uint64_t (*environmentVersion)(void *self);
    // setEnvironment validates and normalizes before storing, returning
    // false (old state kept) for invalid input. resetEnvironment restores
    // the snapshot built from [pbr] configuration. copyEnvironment writes
    // either output; at least one must be non-null.
    bool (*setEnvironment)(void *self, const PbrEnvironment *environment);
    void (*resetEnvironment)(void *self);
    bool (*copyEnvironment)(void *self, PbrEnvironment *outEnvironment, std::uint64_t *outVersion);

    // Stateless blend of from -> to at t in [0, 1]; exposure blends in log
    // space. Returns false for null pointers or out-of-range t.
    bool (*blendEnvironments)(
        void *self, const PbrEnvironment *from, const PbrEnvironment *to, float t, PbrEnvironment *outEnvironment);

    // Convert one (or a batch of) legacy material(s). options == nullptr
    // selects the module-configured options; the batch stops at the first
    // invalid material (earlier outputs remain written).
    bool (*convertModelMaterial)(void *self, const models::ModelMaterial *source,
        const ModelMaterialConversionOptions *options, ConvertedModelMaterial *outMaterial);

    bool (*convertModelMaterials)(void *self, const models::ModelMaterial *sources, std::uint32_t count,
        const ModelMaterialConversionOptions *options, ConvertedModelMaterial *outMaterials);

    // set/reset/copy for the module-configured conversion options; the copy
    // reports the options version so callers can re-convert on change.
    bool (*setModelMaterialConversionOptions)(void *self, const ModelMaterialConversionOptions *options);
    void (*resetModelMaterialConversionOptions)(void *self);
    bool (*copyModelMaterialConversionOptions)(
        void *self, ModelMaterialConversionOptions *outOptions, std::uint64_t *outVersion);
};

} // namespace pbr
} // namespace engine

#endif
