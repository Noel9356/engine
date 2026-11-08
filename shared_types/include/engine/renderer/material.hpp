// Runtime mesh material (engine::renderer).
//
// GPU-facing PBR subset: base color, metallic-roughness, normal, occlusion
// and emissive textures by TextureHandle plus their scalar factors.

#ifndef ENGINE_SHARED_RENDER_MATERIAL_HPP
#define ENGINE_SHARED_RENDER_MATERIAL_HPP

#include <cstdint>

namespace engine::renderer
{

// Opaque renderer-side texture id; invalidTexture (0) means "not set".
using TextureHandle                           = std::uint32_t;
inline constexpr TextureHandle invalidTexture = 0;

struct MeshMaterial
{
    float baseColor[3] = {1.0f, 1.0f, 1.0f};

    TextureHandle baseColorTexture = invalidTexture;

    float         metallicFactor           = 0.0f;
    float         roughnessFactor          = 1.0f;
    TextureHandle metallicRoughnessTexture = invalidTexture;

    TextureHandle normalTexture = invalidTexture;
    float         normalScale   = 1.0f;

    TextureHandle occlusionTexture  = invalidTexture;
    float         occlusionStrength = 1.0f;

    float         emissiveColor[3] = {0.0f, 0.0f, 0.0f};
    TextureHandle emissiveTexture  = invalidTexture;

    bool unlit = false;
};

} // namespace engine::renderer

#endif
