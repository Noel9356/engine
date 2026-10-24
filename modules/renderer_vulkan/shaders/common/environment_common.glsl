// Shared sky model for the composite pass, the cascade compose base and the
// reflection fallback. A vertical gradient (zenith / horizon / ground) plus
// an optional physically scaled sun disk.
//
// Two entry points on purpose:
//   skyBase(direction)     gradient only - the cascade compose uses this, so
//                          the GI field stays smooth (a sub-pixel disk would
//                          alias into the coarse angular texels);
//   skyRadiance(direction) gradient + disk - direct view (composite sky
//                          pixels) and reflection rays see the sun itself.
//
// The sun disk radiance is sunColor * sunIntensity / solidAngle: integrating
// the disk over its solid angle reproduces exactly the sun's configured
// irradiance, so the disk is bright enough for ACES to clamp it to white but
// adds no extra energy to the scene.

#ifndef ENGINE_ENVIRONMENT_COMMON_GLSL
#define ENGINE_ENVIRONMENT_COMMON_GLSL

// Descriptor binding of the environment block. Consumers that use the
// technique shared-set layout get the default (6); the composite pass binds
// it at 7 (its bindings 0-6 are the image inputs) and defines the macro
// before the include.
#ifndef ENGINE_ENVIRONMENT_BINDING
#define ENGINE_ENVIRONMENT_BINDING 6
#endif

// std140 mirror of renderer.cpp EnvironmentData (keep both sides in sync).
layout(std140, set = 0, binding = ENGINE_ENVIRONMENT_BINDING) uniform EnvironmentUniform
{
    vec4 skyZenith;        // rgb zenith radiance, w horizon exponent
    vec4 skyHorizon;       // rgb horizon radiance, w ground exponent
    vec4 skyGround;        // rgb ground radiance, w sun disk enabled (0/1)
    vec4 sunDiskDirection; // xyz direction towards the scene, w cos(inner angle)
    vec4 sunDiskRadiance;  // rgb disk radiance, w cos(outer angle)
} environment;

vec3 skyBase(vec3 direction)
{
    const float height = direction.y;
    const vec3 ground = environment.skyGround.rgb;
    const vec3 horizon = environment.skyHorizon.rgb;
    const vec3 zenith = environment.skyZenith.rgb;
    if (height >= 0.0)
    {
        const float t = pow(clamp(height, 0.0, 1.0), max(environment.skyZenith.w, 0.001));
        return mix(horizon, zenith, t);
    }
    const float t = pow(clamp(-height, 0.0, 1.0), max(environment.skyHorizon.w, 0.001));
    return mix(horizon, ground, t);
}

vec3 skyRadiance(vec3 direction)
{
    vec3 result = skyBase(direction);
    if (environment.skyGround.w > 0.5)
    {
        const float cosine = dot(normalize(direction), normalize(environment.sunDiskDirection.xyz));
        // Smoothstep between the outer and inner cosine angles gives a small
        // antialiased limb instead of a hard circle.
        const float disk = smoothstep(environment.sunDiskRadiance.w, environment.sunDiskDirection.w, cosine);
        result += environment.sunDiskRadiance.rgb * disk;
    }
    return result;
}

#endif // ENGINE_ENVIRONMENT_COMMON_GLSL
