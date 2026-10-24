// Final composite (graphics): tone-maps the linear HDR scene and adds the
// blurred bloom.
//
// The bloom path's last stage: the composite pass wrote the untonemapped HDR
// scene into the HDR intermediate (tonemap mode none), the bloom chain
// extracted and blurred its bright energy, and THIS pass combines
//     final = tonemap(hdr + bloomStrength * bloom)
// into the swapchain. When bloom is disabled the regular composite pass
// tonemaps straight into the swapchain and this shader never runs.
//
// The tonemap operators are the composite's, kept textually identical so a
// bloom run and a non-bloom run of the same scene match pixel for pixel
// outside the glow.

#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D hdrSampler;    // linear HDR scene
layout(set = 0, binding = 1) uniform sampler2D bloomSampler;  // blurred bright-pass

layout(push_constant) uniform FinalPush
{
    layout(offset = 0) vec4 flagsExposure; // x bloomStrength, w exposure
    layout(offset = 16) ivec4 modeFlags;   // x tonemap (0 none, 1 aces, 2 reinhard)
} final_push;

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
    vec3 hdr    = texture(hdrSampler, inUv).rgb;
    vec3 bloom  = texture(bloomSampler, inUv).rgb;
    vec3 color  = hdr + bloom * final_push.flagsExposure.x;

    if (final_push.modeFlags.x == 1)
    {
        color = acesTonemap(color * final_push.flagsExposure.w);
    }
    else if (final_push.modeFlags.x == 2)
    {
        color = color * final_push.flagsExposure.w;
        color = color / (1.0 + color); // reinhard
    }
    else
    {
        color = clamp(color * final_push.flagsExposure.w, 0.0, 1.0);
    }
    outColor = vec4(color, 1.0);
}
