// Shared material access for the geometry pass and the SDF tracing passes.
//
// One texture array per material map kind: layer i belongs to texture set i
// (the renderer uploads sets in the order their names first appear in the
// scene). Per-primitive material indices live in the material table
// (binding 3), one entry per scene primitive in the same order - the
// primitives buffer (scene_common.glsl, binding 1) and this table are the
// same length. A negative layer index means "no texture": the pass falls
// back to the flat values the primitive itself carries.
//
// Data map packing (one RGBA8 layer per set, assembled by the renderer):
//   R roughness, G metallic, B/A tangent-space normal XY (OpenGL +X/+Y).
// The normal Z is reconstructed in the shader.

#ifndef ENGINE_MATERIALS_COMMON_GLSL
#define ENGINE_MATERIALS_COMMON_GLSL

#include "scene_common.glsl"

struct GpuMaterial
{
    ivec4 layers;       // x albedo array layer, y data array layer (-1 = none)
    vec4 scaleAndFlags; // x uv tiling scale, y unused, z unused, w unused
};

layout(std430, set = 0, binding = 3) readonly buffer MaterialBuffer
{
    GpuMaterial materials[];
};

layout(set = 0, binding = 4) uniform sampler2DArray albedoTextureArray; // sRGB decoded on sample
layout(set = 0, binding = 5) uniform sampler2DArray dataTextureArray;  // R rough, G metal, BA normal xy

struct SurfaceMaterial
{
    vec3 albedo;
    float roughness;
    float metallic;
    bool hasNormalMap;
    vec2 normalXY;
};

// Material of one primitive at one point. primitiveIndex indexes BOTH the
// primitives and the material table; localPoint is the primitive-local
// position (already rotated by the primitive's rotationY).
SurfaceMaterial materialAt(uint primitiveIndex, vec3 localPoint)
{
    const GpuPrimitive primitive = primitives[primitiveIndex];
    const GpuMaterial material = materials[primitiveIndex];
    const float uvScale = max(material.scaleAndFlags.x, 1e-4);

    vec2 uv;
    if (primitive.positionType.w < 0.5)
    {
        uv = boxSurfaceUv(localPoint, primitive.sizeRotation.xyz, uvScale);
    }
    else
    {
        uv = sphereSurfaceUv(localPoint, primitive.sizeRotation.x, uvScale);
    }

    SurfaceMaterial result;
    result.albedo = primitive.albedoRoughness.rgb;
    result.roughness = primitive.albedoRoughness.a;
    result.metallic = primitive.metallicEmissiveStrength.x;
    result.hasNormalMap = false;
    result.normalXY = vec2(0.5);

    if (material.layers.x >= 0)
    {
        result.albedo = textureLod(albedoTextureArray, vec3(uv, float(material.layers.x)), 0.0).rgb;
    }
    if (material.layers.y >= 0)
    {
        const vec4 data = textureLod(dataTextureArray, vec3(uv, float(material.layers.y)), 0.0);
        result.roughness = data.r;
        result.metallic = data.g;
        result.hasNormalMap = true;
        result.normalXY = data.ba;
    }
    return result;
}

// World normal perturbed by the material's normal map. Geometric fallback:
// the interpolated/geometric normal itself when the set has no normal map.
vec3 materialWorldNormal(const SurfaceMaterial material, vec3 geometricNormal, vec3 tangent, vec3 bitangent)
{
    if (!material.hasNormalMap)
    {
        return geometricNormal;
    }
    const vec2 xy = material.normalXY * 2.0 - vec2(1.0);
    const float z = sqrt(max(1.0 - dot(xy, xy), 0.0));
    return normalize(tangent * xy.x + bitangent * xy.y + geometricNormal * z);
}

#endif // ENGINE_MATERIALS_COMMON_GLSL
