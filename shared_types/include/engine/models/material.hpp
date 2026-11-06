// Shared model-side material structure (engine::models).
//
// Asset-pipeline-style material (Phong legacy fields plus PBR/extension
// factors) exchanged between asset import and mesh conversion.

#ifndef ENGINE_SHARED_MODELS_MATERIAL_HPP
#define ENGINE_SHARED_MODELS_MATERIAL_HPP

#include <cstdint>

namespace engine::models
{

// Color space a texture's values are interpreted in.
enum class ModelTextureColorSpace : std::uint32_t
{
    Unspecified = 0,
    Linear,
    Srgb
};

// Fixed texture slots a model material can reference; Count = number of slots.
enum class ModelMaterialTextureSlot : std::uint32_t
{
    Ambient = 0,
    BaseColor,
    Specular,
    SpecularHighlight,
    Normal,
    Roughness,
    Metallic,
    Emissive,
    Opacity,
    Displacement,
    Reflection,
    Sheen,
    Clearcoat,
    ClearcoatRoughness,
    ClearcoatNormal,
    Transmission,
    SpecularColor,
    SheenColor,
    SheenRoughness,
    Iridescence,
    IridescenceThickness,
    Thickness,
    DiffuseTransmission,
    DiffuseTransmissionColor,
    Anisotropy,
    Count
};

// Complete material description combining Phong legacy fields and glTF-style
// PBR/extension factors.
struct ModelMaterial
{
    float baseColor[3]       = {1.0f, 1.0f, 1.0f};
    float ambient[3]         = {0.0f, 0.0f, 0.0f};
    float emissive[3]        = {0.0f, 0.0f, 0.0f};
    float specular[3]        = {0.0f, 0.0f, 0.0f};
    float transmittance[3]   = {0.0f, 0.0f, 0.0f};
    float metallic           = 0.0f;
    float roughness          = 1.0f;
    float opacity            = 1.0f;
    float indexOfRefraction  = 1.5f;
    float shininess          = 0.0f;
    float sheen              = 0.0f;
    float clearcoat          = 0.0f;
    float clearcoatRoughness = 0.0f;
    float anisotropy         = 0.0f;
    float anisotropyRotation = 0.0f;
    // Legacy MTL illumination model index.
    std::uint32_t illuminationModel = 2;
    // True when the PBR fields carry real values (vs. legacy Phong defaults).
    bool  hasPbrFactors       = false;
    float transmission        = 0.0f;
    float emissiveStrength    = 1.0f;
    float specularFactor      = 1.0f;
    float iridescence         = 0.0f;
    float alphaCutoff         = 0.5f;
    float attenuationColor[3] = {1.0f, 1.0f, 1.0f};
    float attenuationDistance = 0.0f;
    // Alpha mode (0 = opaque).
    std::uint32_t alphaMode                   = 0;
    bool          doubleSided                 = false;
    bool          unlit                       = false;
    float         sheenColor[3]               = {0.0f, 0.0f, 0.0f};
    float         sheenRoughness              = 0.0f;
    float         thickness                   = 0.0f;
    float         diffuseTransmission         = 0.0f;
    float         diffuseTransmissionColor[3] = {1.0f, 1.0f, 1.0f};
    float         dispersion                  = 0.0f;
    float         iridescenceIor              = 1.3f;
    float         iridescenceThicknessMinimum = 100.0f;
    float         iridescenceThicknessMaximum = 400.0f;
};

// Per-texture sampling and adjustment options carried with a slot assignment
// (MTL-style options).
struct ModelMaterialTextureOptions
{
    float offset[3]      = {0.0f, 0.0f, 0.0f};
    float scale[3]       = {1.0f, 1.0f, 1.0f};
    float turbulence[3]  = {0.0f, 0.0f, 0.0f};
    float sharpness      = 1.0f;
    float brightness     = 0.0f;
    float contrast       = 1.0f;
    float bumpMultiplier = 1.0f;
    // -1 = no forced resolution.
    std::int32_t           textureResolution = -1;
    std::uint32_t          textureType       = 0;
    char                   imageChannel      = 'l';
    ModelTextureColorSpace colorSpace        = ModelTextureColorSpace::Unspecified;
    bool                   clamp             = false;
    bool                   blendU            = true;
    bool                   blendV            = true;
    float                  rotation          = 0.0f;
    std::uint32_t          texCoord          = 0;
    std::uint32_t          samplerMinFilter  = 0;
    std::uint32_t          samplerMagFilter  = 0;
    std::uint32_t          wrapU             = 0;
    std::uint32_t          wrapV             = 0;
};

} // namespace engine::models

#endif
