#version 450
#extension GL_GOOGLE_include_directive : require
// Geometry pass fragment shader: writes the G-buffer. Three attachments keep
// the formats minimal (memory rule): albedo+roughness in RGBA8, world normal
// + metallic in RGBA16F, emissive radiance in RGBA16F (HDR values would clip
// in the 8-bit attachment). The depth attachment is bound by the pipeline.
//
// Materials: when the object references a texture set, albedo/roughness/
// metallic/normal come from the texture arrays (materials_common.glsl);
// otherwise the flat configured values pass through. UVs and the tangent
// frame come from the shared parameterization, so a GI bounce or a
// reflection ray landing on the same point samples identical texels.


#include "scene_common.glsl"
#include "materials_common.glsl"

layout(location = 0) in vec3 inWorldPosition;
layout(location = 1) in vec3 inWorldNormal;
layout(location = 2) in vec3 inLocalPosition;
layout(location = 3) in vec3 inLocalNormal;

layout(location = 0) out vec4 outEmissive;     // RGBA16F
layout(location = 1) out vec4 outAlbedo;       // RGBA8_UNORM
layout(location = 2) out vec4 outNormal;       // RGBA16F

layout(push_constant) uniform ObjectPush
{
    layout(offset = 0) mat4 model;
    layout(offset = 64) vec4 albedoRoughness;          // rgb albedo, a roughness
    layout(offset = 80) vec4 metallicEmissiveStrength; // x metallic, y emissive strength
    layout(offset = 96) vec4 materialLayers;           // x albedo layer, y data layer (-1 none), z uv scale, w primitive type
    layout(offset = 112) vec4 meshBox;                 // mesh objects: the registered mesh's bbox half extents
} object;

void main()
{
    // The model's column lengths are the axis scales = the primitive's half
    // extents (and the sphere radius) - the exact numbers materialAt() reads
    // from the primitives buffer on the tracing side. Registered meshes arrive
    // with an identity-scale model (vertices are authored in meters): their
    // half extents ride in meshBox, the same numbers the tracing proxy box
    // carries, so both paths parameterize the surface identically.
    const uint primitiveType = uint(object.materialLayers.w + 0.5);
    const vec3 halfExtents = primitiveType == primitiveTypeMesh
                                 ? object.meshBox.xyz
                                 : vec3(length(object.model[0].xyz), length(object.model[1].xyz),
                                        length(object.model[2].xyz));
    const float uvScale = max(object.materialLayers.z, 1e-4);
    // floor(x + 0.5) rounds HALF AWAY FROM ZERO for negatives too: int()
    // truncates toward zero, which would turn the -1 sentinel into 0.
    const int albedoLayer = int(floor(object.materialLayers.x + 0.5));
    const int dataLayer = int(floor(object.materialLayers.y + 0.5));

    SurfaceMaterial material;
    material.albedo = object.albedoRoughness.rgb;
    material.roughness = object.albedoRoughness.a;
    material.metallic = object.metallicEmissiveStrength.x;
    material.hasNormalMap = false;
    material.normalXY = vec2(0.5);

    if (albedoLayer >= 0 || dataLayer >= 0)
    {
        // Mesh UVs are the box parameterization over the bbox (mesh-local
        // meters): the relight path reconstructs the SAME local point through
        // worldToLocalY and the proxy's half extents, so both sample the
        // same texels on every GI bounce and reflection hit.
        const vec2 uv = primitiveType == primitiveTypeSphere
                            ? sphereSurfaceUv(inLocalPosition, halfExtents.x, uvScale)
                            : boxSurfaceUv(inLocalPosition, halfExtents, uvScale);
        if (albedoLayer >= 0)
        {
            material.albedo = texture(albedoTextureArray, vec3(uv, float(albedoLayer))).rgb;
        }
        if (dataLayer >= 0)
        {
            const vec4 data = texture(dataTextureArray, vec3(uv, float(dataLayer)));
            material.roughness = data.r;
            material.metallic = data.g;
            material.hasNormalMap = true;
            material.normalXY = data.ba;
        }
    }

    vec3 worldNormal = normalize(inWorldNormal);
    if (material.hasNormalMap)
    {
        vec3 tangent;
        vec3 bitangent;
        surfaceTbn(worldNormal, inLocalNormal, mat3(object.model), primitiveType, tangent, bitangent);
        worldNormal = materialWorldNormal(material, worldNormal, tangent, bitangent);
    }

    outEmissive = vec4(material.albedo * object.metallicEmissiveStrength.y, 1.0);
    outAlbedo = vec4(material.albedo, material.roughness);
    outNormal = vec4(worldNormal, material.metallic);
}
