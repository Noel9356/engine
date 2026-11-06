#version 450

layout(early_fragment_tests) in;

layout(constant_id = 0) const uint MAX_LIGHTS = 32u;
layout(constant_id = 6) const int SRGB_OUTPUT = 0;
// Fragment stage of the render module's Vulkan backend: the engine's PBR
// shading model, metallic-roughness workflow.
//
// The lighting model must agree with the software rasterizer in
// headless_backend.cpp - same formulas, same constants, same clamps - so
// both backends keep producing the same image. Changing either side
// requires updating the rasterizer; the build recompiles this file to
// SPIR-V automatically (cmake/engine_module_compile_shaders.cmake).
//
// SAFETY: layout(early_fragment_tests) declares the hardware should run the
// depth/stencil test BEFORE the fragment shader. This shader does not modify
// gl_FragDepth or discard, so the early test is safe and free; fragments
// behind existing geometry skip the expensive PBR math entirely. Zero
// regression risk — the shader produces identical output, just faster.

layout(early_fragment_tests) in;

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vColor;
layout(location = 2) in vec2 vUv;
layout(location = 3) in vec3 vWorldPosition;

// S-2: MAX_LIGHTS specialization constant. Default 32 = original behavior.
// The renderer can specialize to a smaller bucket (1, 4, 8, 16) to unroll
// the light loop and reduce register pressure. Default 32 = no change.

// G-9: SRGB_OUTPUT specialization constant. Default 0 = manual gamma
// (original behavior). When 1, the shader skips pow(1/2.2) because the
// render target is _SRGB (hardware does the conversion). Default 0 = safe.

layout(push_constant) uniform PushData
{
    mat4 uModel;
    vec4 uBaseColorFactor;
    vec4 uMetallicRoughness; // x metallicFactor, y roughnessFactor, z normalScale, w occlusionStrength
    vec4 uEmissive;          // rgb emissiveColor, a unlit flag
} pushData;

struct LightData
{
    vec4 positionType; // xyz position, w = DirectLightType
    vec4 directionRange; // xyz ray direction / spot direction, w = range
    vec4 colorIntensity; // rgb linear color, a = intensity
    vec4 spotParameters; // x = cos(inner cone), y = cos(outer cone)
};

layout(std140, set = 0, binding = 0) uniform FrameData
{
    mat4 uViewProj;
    vec4 uCameraPos;
    vec4 uSunDirection;   // xyz = direction toward the sun, w = sun enabled
    vec4 uSunColor;       // rgb = sun color, a = sun intensity
    vec4 uAmbientSky;     // rgb = sky color, a = ambient intensity
    vec4 uAmbientGround;  // rgb = ground bounce color
    vec4 uToneMapping;    // x = exposure, y = tone mapper id
    vec4 uDirectLightMeta; // x = active direct-light count
    LightData uDirectLights[MAX_LIGHTS];
} frameData;

// One material descriptor set (the layout's set 1): five combined image
// samplers, one binding per map. The renderer fills every slot - a material
// without a map samples the neutral stand-in - so the declarations must keep
// these exact set/binding numbers (they must match vulkan_pipeline.cpp's
// materialSetLayout and VulkanResources' descriptor writes).
layout(set = 1, binding = 0) uniform sampler2D uBaseColorTexture;
layout(set = 1, binding = 1) uniform sampler2D uNormalTexture;
layout(set = 1, binding = 2) uniform sampler2D uMetallicRoughnessTexture;
layout(set = 1, binding = 3) uniform sampler2D uOcclusionTexture;
layout(set = 1, binding = 4) uniform sampler2D uEmissiveTexture;

layout(location = 0) out vec4 oColor;

const float PI = 3.141592653589793;

// Roughness below this would make the specular highlight a point light of
// its own (and risks dividing by zero); both backends clamp identically.
const float minimumRoughness = 0.045;

// Schlick-GGX normal distribution.
float distributionGGX(float nDotH, float roughness)
{
    const float a  = roughness * roughness;
    const float a2 = a * a;
    const float d  = nDotH * nDotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Smith geometry term (Schlick-GGX with the direct-lighting k).
float geometrySmith(
    float nDotV,
    float nDotL,
    float roughness
)
{
    const float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    const float gView    = nDotV / (nDotV * (1.0 - k) + k);
    const float gLight   = nDotL / (nDotL * (1.0 - k) + k);
    return gView * gLight;
}

vec3 fresnelSchlick(
    float cosTheta,
    vec3  f0
)
{
    return f0 + (1.0 - f0) * pow(1.0 - cosTheta, 5.0);
}

// The tangent frame for the normal map, derived from screen-space
// derivatives of the position and uv varyings - the standard fallback for
// meshes without vertex tangents (MikkTSpace-style). The software
// rasterizer builds the same frame analytically from the triangle's world-
// space edges and uv deltas; the two agree because a triangle is planar.
mat3 tangentFrame(
    vec3 normal,
    vec3 worldPosition,
    vec2 uv
)
{
    const vec3 dp1 = dFdx(worldPosition);
    const vec3 dp2 = dFdy(worldPosition);
    const vec2 duv1 = dFdx(uv);
    const vec2 duv2 = dFdy(uv);

    const vec3 dp2Perp = cross(dp2, normal);
    const vec3 dp1Perp = cross(normal, dp1);
    const vec3 tangent   = dp2Perp * duv1.x + dp1Perp * duv2.x;
    const vec3 bitangent = dp2Perp * duv1.y + dp1Perp * duv2.y;

    const float invMax = inversesqrt(max(dot(tangent, tangent), dot(bitangent, bitangent)) + 1e-12);
    return mat3(tangent * invMax, bitangent * invMax, normal);
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
    const vec3 geometryNormal = normalize(vNormal);

    // ---- material assembly: the five maps times their factors
    const vec3  baseColor = vColor * pushData.uBaseColorFactor.rgb
                          * texture(uBaseColorTexture, vUv).rgb;

    // Perturbed normal: decode the tangent-space map, scale the tangential
    // part by normalScale, and rotate into world space. The neutral flat
    // normal (0.5, 0.5, 1.0) decodes to (0, 0, 1) and leaves the geometry
    // normal standing.
    const vec3  normalMapSample = texture(uNormalTexture, vUv).xyz * 2.0 - 1.0;
    const vec3  tangentNormal   = normalize(vec3(normalMapSample.xy * pushData.uMetallicRoughness.z, normalMapSample.z));
    const mat3  tbn             = tangentFrame(geometryNormal, vWorldPosition, vUv);
    const vec3  normal          = normalize(tbn * tangentNormal);

    // Metallic-roughness: map channels times factors (glTF convention).
    const vec2  metallicRoughnessSample = texture(uMetallicRoughnessTexture, vUv).gb;
    const float metallic  = clamp(metallicRoughnessSample.y * pushData.uMetallicRoughness.x, 0.0, 1.0);
    const float roughness = clamp(metallicRoughnessSample.x * pushData.uMetallicRoughness.y, minimumRoughness, 1.0);

    // Ambient occlusion: 1 - strength * (1 - sampled) darkens the ambient
    // light only; the neutral white map (sampled = 1) is a no-op.
    const float occlusion = 1.0 - pushData.uMetallicRoughness.w * (1.0 - texture(uOcclusionTexture, vUv).r);

    // Direct light: one directional sun.
    const vec3  view           = normalize(frameData.uCameraPos.xyz - vWorldPosition);
    const vec3  lightDirection = normalize(frameData.uSunDirection.xyz);
    const vec3  halfVector     = normalize(view + lightDirection);
    const float nDotL = max(dot(normal, lightDirection), 0.0);
    const float nDotV = max(dot(normal, view), 1e-4);
    const float nDotH = max(dot(normal, halfVector), 0.0);
    const float vDotH = max(dot(view, halfVector), 0.0);

    const vec3  f0           = mix(vec3(0.04), baseColor, metallic);
    const vec3  sunRadiance  = frameData.uSunColor.rgb * frameData.uSunColor.a * frameData.uSunDirection.w;

    const float dTerm = distributionGGX(nDotH, roughness);
    const float gTerm = geometrySmith(nDotV, nDotL, roughness);
    const vec3  fTerm = fresnelSchlick(vDotH, f0);

    const vec3 specular = (dTerm * gTerm * fTerm) / vec3(4.0 * nDotV * nDotL + 1e-5);
    const vec3 diffuse  = baseColor * (1.0 - metallic) / PI;

    // Hemispheric ambient: sky color for up-facing normals, ground bounce
    // for down-facing ones, blended by the world-space normal's y, scaled
    // by the occlusion map.
    const vec3 upColor   = frameData.uAmbientSky.rgb * frameData.uAmbientSky.a;
    const vec3 downColor = frameData.uAmbientGround.rgb * frameData.uAmbientSky.a;
    const vec3 ambient   = mix(downColor, upColor, normal.y * 0.5 + 0.5) * baseColor * occlusion;

    vec3 color = (diffuse + specular) * sunRadiance * nDotL + ambient;

    // Dynamic direct lights. The loop bound uses MAX_LIGHTS (specialization
    // constant, default 32 = original). The active count breaks out early.
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
        const float lightNDotL = max(dot(normal, lightDirection), 0.0);
        if (lightNDotL <= 0.0 || attenuation <= 0.0)
        {
            continue;
        }
        const vec3 lightHalfVector = normalize(view + lightDirection);
        const float lightNDotH = max(dot(normal, lightHalfVector), 0.0);
        const float lightVDotH = max(dot(view, lightHalfVector), 0.0);
        const float lightD = distributionGGX(lightNDotH, roughness);
        const float lightG = geometrySmith(nDotV, lightNDotL, roughness);
        const vec3 lightF = fresnelSchlick(lightVDotH, f0);
        const vec3 lightSpecular = (lightD * lightG * lightF) / vec3(4.0 * nDotV * lightNDotL + 1e-5);
        const vec3 radiance = light.colorIntensity.rgb * light.colorIntensity.a * attenuation;
        color += (diffuse + lightSpecular) * radiance * lightNDotL;
    }

    // Unlit surfaces (sky dome, sun disc) skip shading and show their
    // albedo; the flag rides on the push constants, so the branch is
    // uniform across the draw.
    if (pushData.uEmissive.a > 0.5)
    {
        color = baseColor;
    }

    // The surface's own light, added on top of the lit (or unlit) result.
    color += pushData.uEmissive.rgb * texture(uEmissiveTexture, vUv).rgb;

    // Exposure + tone mapping. The linear-to-sRGB output transform is
    // conditional on SRGB_OUTPUT (specialization constant, default 0):
    //   0 = UNORM target, manual pow(1/2.2) gamma (original behavior).
    //   1 = _SRGB target, hardware does the conversion (skip manual gamma).
    // Default 0 = safe = original behavior.
    color = toneMap(color * frameData.uToneMapping.x, frameData.uToneMapping.y);
    if (SRGB_OUTPUT == 0)
    {
        if (SRGB_OUTPUT == 0) { color = pow(max(color, vec3(0.0)), vec3(1.0 / 2.2)); }
    }

    oColor = vec4(color, 1.0);
}
