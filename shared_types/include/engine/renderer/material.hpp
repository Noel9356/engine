// Runtime mesh material (engine::renderer).
//
// GPU-facing PBR subset: base color, metallic-roughness, normal, occlusion
// and emissive textures by TextureHandle plus their scalar factors, the
// transmission pair (opacity, index of refraction) the transparent-draw path
// and the lumen_gi / lumen_reflection surfaces evaluate with, and the fabric
// shading pair (sheen strength, double-sided rendering) the cloth path
// evaluates with. Field order is append-only: modules embed this struct by
// value in ABI surfaces (the pbr module's ConvertedModelMaterial), so new
// fields go at the end.

#pragma once

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

    // Whether this draw writes to the depth buffer (depth testing always
    // applies). Backdrop geometry drawn before the scene it surrounds - an
    // atmosphere shell, a sky dome - renders with depth writes off so the
    // later opaque passes overwrite it inside the silhouette while the shell
    // survives around it. Transparent draws (opacity below 1) also keep
    // depth writes off so co-transmitted surfaces do not occlude each other.
    bool depthWrite = true;

    // Fraction of light the surface keeps off the transmitted path (1 =
    // opaque, 0 = fully transparent). Values below 1 route the draw into the
    // transparent pass (alpha blended over the opaque scene, depth writes
    // off) and register the surface as transmissive with the lumen_gi and
    // lumen_reflection modules, whose rays pass through with this
    // attenuation. The transmitted light is tinted by baseColor.
    float opacity = 1.0f;

    // Index of refraction of the surface's medium (glass 1.5, water 1.33);
    // refractive rays bend by Snell's law with this ratio. 1 keeps the
    // transmitted path unbent.
    float indexOfRefraction = 1.5f;

    // Fabric sheen strength in [0, 1] (0 = plain metallic-roughness PBR).
    // Positive values blend the GGX specular lobe toward the Charlie sheen
    // lobe (Estevez & Kulla 2017 with the Neubelt visibility term), soften
    // the direct-light terminator with the energy-conserving wrap-diffuse
    // factor, and tint the sheen highlight with the base color - the
    // production cloth look for simulated fabric.
    float sheenFactor = 0.0f;

    // Two-sided rendering: back-face culling is skipped and the shading
    // normal flips toward the viewer on the back side, so thin sheets
    // (cloth, flags, curtains) light correctly from both sides without a
    // duplicated shell mesh.
    bool doubleSided = false;

    // Winding convention of this mesh's triangles, known by the generator:
    // +1.0f marks triangles wound so cross(b - a, c - a) points outward in
    // world space, -1.0f the mirrored convention, 0.0f (the default) leaves
    // the convention to the renderer (its configured culling plus, on the
    // CPU backend, its winding vote). A known sign enables exact
    // per-triangle back-face culling for meshes whose winding the vote
    // cannot classify (rugged procedural terrain: crease triangles whose
    // smooth normals lean across edges flip the majority vote). Culling
    // applies only where the renderer culls at all; a negative sign is
    // ignored on double-sided draws. Field order is append-only.
    float frontFaceWinding = 0.0f;

    // Alpha-cutout threshold for opaque textured draws (the mesh.frag
    // uCloth.z discard mirror): texels whose sampled alpha falls below the
    // threshold are discarded before shading and depth commits, so foliage
    // canopies and sprite cards read as cut-out silhouettes rather than
    // textured quads. 0.0f (the default) disables the test; a value in
    // (0, 1) on a transparent draw is ignored (transparent draws blend by
    // texel alpha instead). Field order is append-only.
    float alphaCutoff = 0.0f;
};

} // namespace engine::renderer

