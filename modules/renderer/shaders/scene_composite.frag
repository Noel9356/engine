// Composite pass fragment shader: final HDR assembly and tonemap.
//
//   final = direct                                    (Cook-Torrance radiance)
//         + albedo * (1 - metallic) * gi              (diffuse GI irradiance;
//                                                      metals receive none)
//         + indirectSpecular                          (ray traced reflections,
//                                                      Fresnel-weighted by the
//                                                      reflections technique)
//         + emissive                                  (depth < 1: geometry)
//   final = skyRadiance(primary ray direction)        (depth >= 1: sky)
//
// The sky is the shared directional model (environment_common.glsl):
// gradient plus sun disk, the same function reflection rays see, so a
// reflective floor and the sky agree where they meet. Tonemapping and
// exposure are configurable ([renderer] tonemap / exposure).

#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D albedoSampler;   // rgb albedo (linear), a roughness
layout(set = 0, binding = 1) uniform sampler2D emissiveSampler; // rgb emissive radiance
layout(set = 0, binding = 2) uniform sampler2D directSampler;   // rgb direct radiance
layout(set = 0, binding = 3) uniform sampler2D giSampler;       // rgb diffuse GI irradiance / pi
layout(set = 0, binding = 4) uniform sampler2D depthSampler;
layout(set = 0, binding = 5) uniform sampler2D normalSampler;   // rgb world normal, a metallic
layout(set = 0, binding = 6) uniform sampler2D specularSampler; // rgb Fresnel-weighted reflection radiance

// The shared sky model (gradient + sun disk). The composite binds the
// environment block at 7: bindings 0-6 are the image inputs above.
#define ENGINE_ENVIRONMENT_BINDING 7
#include "environment_common.glsl"

// std140 mirror of render_interface.hpp CameraData.
layout(std140, set = 0, binding = 8) uniform CameraUniform
{
    mat4 viewProjection;
    mat4 inverseViewProjection;
    vec4 cameraPosition;
} camera;

layout(push_constant) uniform CompositePush
{
    layout(offset = 0) vec4 flagsExposure; // x unused, w exposure
    layout(offset = 16) ivec4 modeFlags;   // x tonemap (0 none, 1 aces, 2 reinhard), y directEnabled, z giEnabled, w specularEnabled
} composite;

vec3 acesTonemap(vec3 color)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((color * (a * color + b)) / (color * (c * color + d) + e), 0.0, 1.0);
}

void main()
{
    const float depth = texture(depthSampler, inUv).r;
    vec3 color;
    if (depth >= 1.0)
    {
        const vec4 target = camera.inverseViewProjection * vec4(inUv * 2.0 - vec2(1.0), 1.0, 1.0);
        // The shared skyRadiance reads the raw direction's y for the
        // gradient height, so normalize here (the ray is reconstructed).
        color = skyRadiance(normalize(target.xyz / target.w - camera.cameraPosition.xyz));
    }
    else
    {
        const vec4 albedoRoughness = texture(albedoSampler, inUv);
        const vec3 albedo = albedoRoughness.rgb;
        const vec3 emissive = texture(emissiveSampler, inUv).rgb;
        const float metallic = texture(normalSampler, inUv).a;

        vec3 direct = vec3(0.0);
        vec3 gi = vec3(0.0);
        vec3 specular = vec3(0.0);
        if (composite.modeFlags.y != 0)
        {
            direct = texture(directSampler, inUv).rgb;
        }
        if (composite.modeFlags.z != 0)
        {
            gi = texture(giSampler, inUv).rgb;
        }
        if (composite.modeFlags.w != 0)
        {
            specular = texture(specularSampler, inUv).rgb;
        }
        color = direct + albedo * (1.0 - metallic) * gi + specular + emissive;
    }

    color *= composite.flagsExposure.w;
    if (composite.modeFlags.x == 1)
    {
        color = acesTonemap(color);
    }
    else if (composite.modeFlags.x == 2)
    {
        color = color / (1.0 + color);
    }
    outColor = vec4(color, 1.0);
}
