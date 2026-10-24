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
// - the function itself is the DIRECT term only - light arriving at the hit
//   point from other surfaces comes from the cascade field. The radiance
//   cascades' multi-bounce mode ([lighting.screen_space_radiance_cascades].bounces > 1)
//   adds the previous bounce's gathered transport at the hit point in the
//   march shader, on top of what this function returns.

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
// Traced hits should NOT call this: the trace's arg-min owner is available for
// free through sceneSDFOwned/traceIntervalOwned (scene_common.glsl) - this scan
// is for owners that were not produced by a trace (the reflection launch point).
uint primitiveOwnerAt(vec3 point, uint primitiveCount)
{
    uint owner;
    sceneSDFOwned(point, primitiveCount, owner);
    return owner;
}

// Relight of a traced hit point. Callers pass the owner the trace already
// produced (sceneSDFOwned/traceIntervalOwned in scene_common.glsl): the
// former primitiveOwnerAt re-scan at the hit point re-evaluated the whole
// field the trace had just walked.
vec3 surfaceOutgoingRadianceOwned(vec3 point, vec3 normal, uint owner, uint primitiveCount,
                                  out vec3 materialAlbedo)
{
    const GpuPrimitive primitive = primitives[owner];
    // worldToLocalY (NOT the forward rotation): this must be the inverse of the model
    // matrix, or the local point - and with it the sampled texel - drifts
    // across the surface as the rotation grows (see scene_common.glsl).
    const vec3 localPoint = worldToLocalY(point - primitive.positionType.xyz, primitive.metallicEmissiveStrength.zw);
    const SurfaceMaterial material = materialAt(owner, localPoint);
    // Lambertian outgoing radiance: albedo/pi * E, with the 1/pi folded into
    // directIrradianceOverPi (see the comment above for the unit chain).
    vec3 radiance = material.albedo * directIrradianceOverPi(point, normal, primitiveCount);
    radiance += material.albedo * primitive.metallicEmissiveStrength.y;
    materialAlbedo = material.albedo;
    return radiance;
}

#endif // ENGINE_RELIGHT_COMMON_GLSL
