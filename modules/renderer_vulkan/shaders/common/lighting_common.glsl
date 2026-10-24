// Shared lighting math for the direct lighting pass and the GI relight.
//
// The light uniform block is uploaded by the lighting modules from the
// LightingCapability state (configuration seeds it, C++ mods mutate it at
// runtime). Both sides must keep the std140 layout in sync.

#ifndef ENGINE_LIGHTING_COMMON_GLSL
#define ENGINE_LIGHTING_COMMON_GLSL

#include "scene_common.glsl"

layout(std140, set = 0, binding = 2) uniform LightUniform
{
    vec4 sunDirectionIntensity;  // xyz direction (towards the scene), w intensity
    vec4 sunColorEnabled;        // rgb color, w enabled (0/1)
    ivec4 lightCountsAndFlags;   // x point light count, y shadows enabled, z shadow steps, w unused
    vec4 pointPositionRadius[maxPointLights];    // xyz position, w radius cutoff (0 = none)
    vec4 pointColorIntensity[maxPointLights];    // rgb color, w intensity
};

// Soft shadow along a ray using the analytic SDF: classic penumbra estimate
// (min of k * d / t). Returns 1.0 fully lit, 0.0 fully shadowed.
//
// Step policy: sphere tracing advances by the FULL SDF distance (floored at
// 0.005 so grazing rays cannot stall). An upper clamp on the step was
// removed deliberately: with a 0.35 ceiling, 24 steps reach only ~8.4 world
// units against a 60 unit bound, so long rays stopped mid-flight with a
// partially darkened estimate even though nothing blocked them any more -
// dark smears that depended on the step budget instead of the geometry. Full
// steps let a clear ray escape within a handful of iterations; the step
// count bounds only the grazing cases it is meant to bound.
float softShadow(vec3 origin, vec3 direction, float maxDistance, uint primitiveCount, uint steps, float penumbra)
{
    float t = 0.02;
    float shadow = 1.0;
    for (uint step = 0u; step < steps; ++step)
    {
        const float distance = sceneSDF(origin + direction * t, primitiveCount);
        if (distance < 0.0015)
        {
            return 0.0; // hit: hard terminator, sharper than the penumbra floor
        }
        shadow = min(shadow, penumbra * distance / t);
        t += max(distance, 0.005);
        if (shadow < 0.005 || t > maxDistance)
        {
            break;
        }
    }
    return clamp(shadow, 0.0, 1.0);
}

// Surface normal from the analytic SDF (used for relighting hit points inside
// the GI march, where no rasterized normal exists). Tetrahedron technique:
// four SDF evaluations instead of six.
vec3 sdfNormal(vec3 point, uint primitiveCount)
{
    const float epsilon = 0.0008;
    const vec2 k = vec2(1.0, -1.0);
    return normalize(k.xyy * sceneSDF(point + k.xyy * epsilon, primitiveCount) +
                     k.yyx * sceneSDF(point + k.yyx * epsilon, primitiveCount) +
                     k.yxy * sceneSDF(point + k.yxy * epsilon, primitiveCount) +
                     k.xxx * sceneSDF(point + k.xxx * epsilon, primitiveCount));
}

// Diffuse sun contribution at an arbitrary world point. sunDirection points
// FROM the light TOWARDS the scene, so the lit side faces -sunDirection.
//
// The uniform's xyz is normalized once at upload (fillSunAndPointLights,
// light_uniform.hpp - the single definition every lighting module fills its
// block from), so this per-pixel-per-evaluation normalize is gone: sunDiffuse
// runs inside every shadowed relight and shadow ray, and normalize() was a
// reciprocal-square-root per call buying nothing.
vec3 sunDiffuse(vec3 point, vec3 normal, uint primitiveCount)
{
    if (sunColorEnabled.w < 0.5 || sunDirectionIntensity.w <= 0.0)
    {
        return vec3(0.0);
    }
    const vec3 direction = -sunDirectionIntensity.xyz;
    const float cosine = max(dot(normal, direction), 0.0);
    if (cosine <= 0.0)
    {
        return vec3(0.0);
    }
    float visibility = 1.0;
    if (lightCountsAndFlags.y != 0)
    {
        const uint steps = uint(max(lightCountsAndFlags.z, 4));
        visibility = softShadow(point + normal * 0.004, direction, 60.0, primitiveCount, steps, 12.0);
        if (visibility <= 0.0)
        {
            return vec3(0.0);
        }
    }
    return sunColorEnabled.rgb * sunDirectionIntensity.w * cosine * visibility;
}

// Diffuse point light contribution at an arbitrary world point.
vec3 pointDiffuse(vec3 point, vec3 normal, uint primitiveCount)
{
    vec3 sum = vec3(0.0);
    const bool shadows = lightCountsAndFlags.y != 0;
    const uint steps = uint(max(lightCountsAndFlags.z, 4));
    for (uint index = 0u; index < uint(lightCountsAndFlags.x); ++index)
    {
        const vec4 positionRadius = pointPositionRadius[index];
        const vec4 colorIntensity = pointColorIntensity[index];
        const vec3 delta = positionRadius.xyz - point;
        const float distance = length(delta);
        if (distance < 1e-4 || colorIntensity.w <= 0.0)
        {
            continue;
        }
        if (positionRadius.w > 0.0 && distance > positionRadius.w)
        {
            continue;
        }
        const vec3 direction = delta / distance;
        const float cosine = max(dot(normal, direction), 0.0);
        if (cosine <= 0.0)
        {
            continue;
        }
        const float attenuation = 1.0 / (distance * distance);
        float visibility = 1.0;
        if (shadows)
        {
            // Stop the shadow ray AT the light (never at the radius cutoff):
            // a ray that overshoots the light hits whatever sits beyond it -
            // a ceiling centimetres behind a lamp, say - and carves phantom
            // shadow bands around every nearby light.
            visibility = softShadow(point + normal * 0.004, direction, max(distance - 0.05, 0.05),
                                    primitiveCount, steps, 12.0);
        }
        sum += colorIntensity.rgb * colorIntensity.w * cosine * attenuation * visibility;
    }
    return sum;
}

#endif // ENGINE_LIGHTING_COMMON_GLSL
