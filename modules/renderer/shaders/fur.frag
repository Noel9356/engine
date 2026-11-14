#version 450

layout(early_fragment_tests) in;

layout(constant_id = 0) const uint MAX_LIGHTS = 32u;
layout(constant_id = 6) const int SRGB_OUTPUT = 0;
// Fragment stage of the renderer module's shell-fur pipeline.
//
// Technique: shells + strand-texture alpha (Lengyel et al. 2001). Each shell
// layer crops the strand mask (the fur texture's ALPHA channel) against the
// layer ratio carried in vShellRatio, so a stack of alpha-blended layers
// builds up the strand volume. Silhouette "fins" from the original paper are
// replaced by two cheaper artifacts of the same field: the root-side layers
// stay almost fully opaque (an undercoat that fills the silhouette gaps) and
// the per-vertex height jitter in fur.vert keeps the layer edges ragged. This
// is the trade-off shipped by most real-time shell fur (the fin geometry pass
// costs another full silhouette extraction per frame).
//
// Lighting: half-Lambert diffuse (fur must not go black when back-lit - the
// hemispheric volume scatters light like the classic game-fur lighting) plus
// a Kajiya-Kay anisotropic specular lobe (Kajiya & Kay 1989) with the strand
// tangent = the shell normal, since strands grow along it:
//   specular = specColor * pow(sin(T,H), power), sin(T,H) = sqrt(1-(T.H)^2)
// Parity contract: this file mirrors the CPU path furShade() in
// source/core/render_shared.hpp EXACTLY - same formulas, same constants,
// same clamps - like mesh.frag mirrors pbrShade. Changing either side
// requires updating the rasterizer in headless_backend.cpp.
//
// Perf notes: this shader is deliberately cheap because fur cost is
// fill-rate-bound (the shell stack blends every layer over every covered
// pixel): one texture fetch, no normal map, no occlusion/emissive maps, and
// NO lumen_gi probe-grid fetch (the probe walk costs 8 probes x 27 floats
// per pixel and multiplies by the layer count; fur keeps the hemispheric
// ambient term instead - documented in modules/fur/docs/overview.md).
// layout(early_fragment_tests) is safe: no discard, no gl_FragDepth write;
// fragments hidden behind the character's opaque base surface skip the
// shading entirely.

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vColor;
layout(location = 2) in vec2 vUv;
layout(location = 3) in vec3 vWorldPosition;
layout(location = 4) in float vShellRatio;

layout(push_constant) uniform PushData
{
    mat4 uModel;
    vec4 uBaseColorFactor;
    vec4 uMetallicRoughness; // on the fur pipeline: x/y unused, z = densityBoost (strand alpha sharpness), w = outerLayerFade
    vec4 uEmissive;          // rgb emissiveColor, w unlit flag
    vec4 uFur;               // x = layerIndex, y = layerCount, z = furHeight, w = gravityStrength
} pushData;

struct LightData
{
    vec4 positionType;   // xyz position, w = DirectLightType
    vec4 directionRange; // xyz ray direction / spot direction, w = range
    vec4 colorIntensity; // rgb linear color, a = intensity
    vec4 spotParameters; // x = cos(inner cone), y = cos(outer cone)
};

layout(std140, set = 0, binding = 0) uniform FrameData
{
    mat4 uViewProj;
    vec4 uCameraPos;
    vec4 uSunDirection;
    vec4 uSunColor;
    vec4 uAmbientSky;
    vec4 uAmbientGround;
    vec4 uToneMapping;
    vec4 uDirectLightMeta;
    LightData uDirectLights[MAX_LIGHTS];
    vec4 uProbeGridA;
    vec4 uProbeGridB;
    vec4 uProbeGridC;
    vec4 uProbeGridD;
} frameData;

// The fur material set is the SAME five-sampler layout as the mesh pipeline
// (bindable across both pipelines because the layout objects are identical);
// fur shading reads only the base-color map: rgb = strand albedo variation,
// alpha = the strand height field (1 at the root of a strand, 0 at its tip).
layout(set = 1, binding = 0) uniform sampler2D uBaseColorTexture;

layout(location = 0) out vec4 oColor;

const float PI = 3.141592653589793;

// Kajiya-Kay constants. specColor/specPower are fixed (not material-driven)
// for this first iteration: shell fur reads best with a modest broadband
// sheen, and every material knob here would multiply across the layer stack.
const vec3  furSpecularColor = vec3(0.26);
const float furSpecularPower = 24.0;
// Root-to-tip shade ramp: strand roots sit inside the coat (darker, ambient
// blocked by the layers above), tips catch the light.
const float rootShade  = 0.60;
const float tipShade   = 1.05;

// Kajiya-Kay anisotropic specular with the strand tangent (mirror of the
// furShade() term in render_shared.hpp).
float kajiyaKaySpecular(vec3 tangent, vec3 view, vec3 lightDirection)
{
    const vec3  halfVector = normalize(view + lightDirection);
    const float tDotH      = clamp(dot(tangent, halfVector), -1.0, 1.0);
    const float sinTH      = sqrt(max(1.0 - tDotH * tDotH, 0.0));
    return pow(sinTH, furSpecularPower);
}

// Per-layer strand-field jitter (mirror of furFieldJitter in
// render_shared.hpp): a small uv-hash offset unique to each shell layer.
// Without it the consecutive shell layers crop the SAME field, and viewing
// the stack anywhere near parallel to the layers shows the classic shell
// banding - horizontal streaks tracing each layer's silhouette. The jitter
// decorrelates consecutive layers just enough to break the sheets up into
// noise without destroying strand coherence (amplitude 0.12 = +-0.06 on a
// 0..1 field).
float furFieldJitter(vec2 uv, float layerIndex)
{
    return fract(sin(dot(uv * 43.758 + vec2(layerIndex * 7.31, layerIndex * 3.17), vec2(12.9898, 78.233)))
                 * 43758.5453);
}

vec3 toneMap(vec3 color, float mapperId)
{
    if (mapperId < 0.5)
    {
        return color; // none
    }
    if (mapperId < 1.5)
    {
        // Reinhard.
        return color / (1.0 + color);
    }
    // ACES filmic approximation (Narkowicz).
    const vec3 acesInput = color;
    return clamp((acesInput * (2.51 * acesInput + 0.03)) / (acesInput * (2.43 * acesInput + 0.59) + 0.14), 0.0, 1.0);
}

void main()
{
    const vec3  geometryNormal = normalize(vNormal);
    const float t              = clamp(vShellRatio, 0.0, 1.0);
    const float densityBoost   = max(pushData.uMetallicRoughness.z, 0.0);

    // ---- strand field + per-layer alpha ---------------------------------
    const vec4  furSample = texture(uBaseColorTexture, vUv);
    // Layer-jittered strand field: see furFieldJitter above (the shell
    // banding break-up).
    const float field = clamp(furSample.a + (furFieldJitter(vUv, pushData.uFur.x) - 0.5) * 0.12, 0.0, 1.0);
    // Alpha crop: this layer shows the part of the strand whose height
    // exceeds the layer ratio. densityBoost (push constants) sharpens the
    // strand edges: 1 = soft wide wisps, ~8 = thin crisp strands.
    float alpha = clamp((field - t) * densityBoost, 0.0, 1.0);
    // LOD fade: the outermost drawn layer multiplies its alpha by
    // outerLayerFade (0..1) so a decreasing layer count dissolves the tips
    // instead of clipping them abruptly.
    if (t >= 0.999)
    {
        alpha *= clamp(pushData.uMetallicRoughness.w, 0.0, 1.0);
    }
    if (alpha <= 0.0039) // below 1/255: the blend would be invisible
    {
        discard;
    }

    // ---- material assembly ----------------------------------------------
    // Strand albedo: vertex color x base color factor x per-texel variation
    // (same chain as mesh.frag, minus the metallic-roughness map - fur is
    // dielectric by construction).
    const vec3 baseColor = vColor * pushData.uBaseColorFactor.rgb * furSample.rgb;
    const float strandShade = mix(rootShade, tipShade, t);

    const vec3 view           = normalize(frameData.uCameraPos.xyz - vWorldPosition);
    const vec3 lightDirection = normalize(frameData.uSunDirection.xyz);
    const vec3 sunRadiance    = frameData.uSunColor.rgb * frameData.uSunColor.a * frameData.uSunDirection.w;

    // Half-Lambert diffuse (Valve's formulation): nDotL remapped 0..1 with a
    // squared falloff, so fur keeps its fill when the light grazes the coat.
    const float nDotL       = dot(geometryNormal, lightDirection);
    const float halfLambert = nDotL * 0.5 + 0.5;
    const vec3  diffuse     = baseColor * (halfLambert * halfLambert) * strandShade;

    // Kajiya-Kay specular against the sun (tangent = strand direction =
    // shell normal; strands grow along the surface normal).
    const float sunSpecular = kajiyaKaySpecular(geometryNormal, view, lightDirection) * strandShade;

    // Hemispheric ambient, same colors as mesh.frag. Strand shade applies:
    // deep coat layers receive less sky light.
    const vec3 upColor   = frameData.uAmbientSky.rgb * frameData.uAmbientSky.a;
    const vec3 downColor = frameData.uAmbientGround.rgb * frameData.uAmbientSky.a;
    const vec3 ambient   = mix(downColor, upColor, geometryNormal.y * 0.5 + 0.5) * baseColor * strandShade;

    vec3 color = ambient + (diffuse + furSpecularColor * sunSpecular) * sunRadiance;

    // Dynamic direct lights: identical attenuation/cone logic to mesh.frag,
    // with the fur diffuse/specular terms in place of the Cook-Torrance pair.
    const uint directLightCount = uint(clamp(frameData.uDirectLightMeta.x, 0.0, float(MAX_LIGHTS)));
    for (uint lightIndex = 0u; lightIndex < MAX_LIGHTS; ++lightIndex)
    {
        if (lightIndex >= directLightCount) break;
        const LightData light = frameData.uDirectLights[lightIndex];
        const int lightType = int(light.positionType.w + 0.5);
        vec3 lightDirection;
        float attenuation = 1.0;
        if (lightType == 0)
        {
            lightDirection = -light.directionRange.xyz;
        }
        else
        {
            const vec3 toLight = light.positionType.xyz - vWorldPosition;
            const float distanceSquared = max(dot(toLight, toLight), 1e-4);
            const float inverseDistance = inversesqrt(distanceSquared);
            const float range = max(light.directionRange.w, 1e-3);
            const float rangeRatioSquared = distanceSquared / (range * range);
            if (rangeRatioSquared >= 1.0)
            {
                continue;
            }
            lightDirection = toLight * inverseDistance;
            const float rangeFade = 1.0 - rangeRatioSquared * rangeRatioSquared;
            attenuation = rangeFade * rangeFade / distanceSquared;
            if (lightType == 2)
            {
                const float coneCosine = dot(-lightDirection, light.directionRange.xyz);
                attenuation *= smoothstep(light.spotParameters.y, light.spotParameters.x, coneCosine);
            }
        }
        if (attenuation <= 0.0)
        {
            continue;
        }
        const float lightHalfLambert = dot(geometryNormal, lightDirection) * 0.5 + 0.5;
        const float lightSpecular    = kajiyaKaySpecular(geometryNormal, view, lightDirection) * strandShade;
        const vec3  radiance         = light.colorIntensity.rgb * light.colorIntensity.a * attenuation;
        color += (baseColor * (lightHalfLambert * lightHalfLambert) * strandShade
                  + furSpecularColor * lightSpecular)
               * radiance;
    }

    if (pushData.uEmissive.a > 0.5)
    {
        color = baseColor;
    }

    color = toneMap(color * frameData.uToneMapping.x, frameData.uToneMapping.y);
    if (SRGB_OUTPUT == 0)
    {
        color = pow(max(color, vec3(0.0)), vec3(1.0 / 2.2));
    }

    oColor = vec4(color, alpha);
}
