// Shared surface relight for every SDF tracing consumer (the cascade march's
// hit points, the gather's starved-pixel fallback, reflection ray hits):
// outgoing diffuse radiance of a surface point under the direct lights, with
// the point's TEXTURED material. One function, one truth - all consumers
// sample the same texture sets the G-buffer pass rasterizes with, so GI
// bounces and reflections carry the same albedo/roughness the direct pass
// sees.
//
// Deliberate limits (documented in docs/architecture.md):
// - normal maps do not perturb the relight (the SDF gives the geometric
//   normal; the bounce difference is second order);
// - the relight is the FIRST bounce only - light arriving at the hit point
//   from other surfaces comes from the cascade field, not from here.

#ifndef ENGINE_RELIGHT_COMMON_GLSL
#define ENGINE_RELIGHT_COMMON_GLSL

#include "lighting_common.glsl"
#include "materials_common.glsl"

// Direct irradiance (sun + point lights, shadowed) at an arbitrary world
// point, divided by pi. The /pi makes this the radiance a Lambertian surface
// with albedo 1 would emit, which is the unit the cascade FIELD stores:
//   march hit:   L = albedo * directIrradianceOverPi + emissive
//   gather:      gi = sum(cos * solidAngle * L) / pi   (irradiance / pi)
//   composite:   albedo * gi
// The chain then reproduces the physically correct Lambertian transport
// exactly once per stage. An earlier version skipped the /pi here, which
// made every GI bounce pi times brighter than the direct pass it sits next
// to (glowing walls, wrong bounce colors).
vec3 directIrradianceOverPi(vec3 point, vec3 normal, uint primitiveCount)
{
    return (sunDiffuse(point, normal, primitiveCount) + pointDiffuse(point, normal, primitiveCount)) *
           0.3183098861837907;
}

// Index of the primitive that owns a world point (nearest signed distance -
// the same ownership rule every SDF consumer applies). surfaceOutgoingRadiance
// and the reflection pass's convex self-hit test share this.
uint primitiveOwnerAt(vec3 point, uint primitiveCount)
{
    float best = 1e30;
    uint owner = 0u;
    for (uint index = 0u; index < primitiveCount; ++index)
    {
        const GpuPrimitive primitive = primitives[index];
        const vec3 local = worldToLocalY(point - primitive.positionType.xyz, primitive.sizeRotation.w);
        float distance;
        if (primitive.positionType.w < 0.5)
        {
            distance = signedDistanceBox(local, primitive.sizeRotation.xyz);
        }
        else
        {
            distance = signedDistanceSphere(local, primitive.sizeRotation.x);
        }
        if (distance < best)
        {
            best = distance;
            owner = index;
        }
    }
    return owner;
}

vec3 surfaceOutgoingRadiance(vec3 point, vec3 normal, uint primitiveCount, out vec3 materialAlbedo)
{
    const uint owner = primitiveOwnerAt(point, primitiveCount);

    const GpuPrimitive primitive = primitives[owner];
    // worldToLocalY (NOT rotateY): this must be the inverse of the model
    // matrix, or the local point - and with it the sampled texel - drifts
    // across the surface as the rotation grows (see scene_common.glsl).
    const vec3 localPoint = worldToLocalY(point - primitive.positionType.xyz, primitive.sizeRotation.w);
    const SurfaceMaterial material = materialAt(owner, localPoint);
    // Lambertian outgoing radiance: albedo/pi * E, with the 1/pi folded into
    // directIrradianceOverPi (see the comment above for the unit chain).
    vec3 radiance = material.albedo * directIrradianceOverPi(point, normal, primitiveCount);
    radiance += material.albedo * primitive.metallicEmissiveStrength.y;
    materialAlbedo = material.albedo;
    return radiance;
}

#endif // ENGINE_RELIGHT_COMMON_GLSL
