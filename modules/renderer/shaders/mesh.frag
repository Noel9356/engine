#version 450

layout(early_fragment_tests) in;

layout(constant_id = 0) const uint MAX_LIGHTS = 32u;
layout(constant_id = 6) const int SRGB_OUTPUT = 0;
// Fragment stage of the renderer module's Vulkan backend: the engine's PBR
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
    // lumen_gi probe-grid header (the indirect-lighting hook; the SH2 probe
    // records ride the set-0 binding-1 storage buffer below):
    vec4 uProbeGridA; // xyz = grid center, w = level0 spacing
    vec4 uProbeGridB; // xyz = level0 dimensions, w = level1 spacing
    vec4 uProbeGridC; // xyz = level1 dimensions, w = grid active (1.0 = probes bound)
    vec4 uProbeGridD; // x = probe count, y = direction resolution, z = level0 offset, w = level1 offset
} frameData;

// lumen_gi probe records, 8 vec4 per probe: (position.xyz, validity) then
// the 27 SH2 radiance floats (GiProbe [basis][channel] order) plus one pad.
// Written by the renderer module from the lumen_gi copyProbes snapshot.
layout(std430, set = 0, binding = 1) readonly buffer ProbeData
{
    vec4 probeRecords[];
};

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

// ---- lumen_gi probe-grid indirect lighting ----------------------------------
// Mirrors the CPU path in source/render_shared.hpp exactly (same lattice
// walk, same SH2 convolution, same validity weighting), like every other
// formula this shader shares with the software rasterizer.

// Cosine-lobed SH2 irradiance at the probe whose record starts at vec4 index
// `base` (= 8 * probe index): Ramamoorthi & Hanrahan convolution, band
// constants A0 = pi, A1 = 2pi/3, A2 = pi/4.
vec3 giProbeIrradiance(uint base, vec3 n)
{
    float sh2[27];
    for (int i = 0; i < 27; ++i)
    {
        sh2[i] = probeRecords[base + 1u + uint(i / 4)][i % 4];
    }
    const float k0 = 3.141592654;  // pi    (band 0)
    const float k1 = 2.094395102;  // 2pi/3 (band 1)
    const float k2 = 0.785398163;  // pi/4  (band 2)
    const float xx = n.x * n.x;
    const float yy = n.y * n.y;
    const float zz = n.z * n.z;
    vec3 irradiance;
    irradiance[0] = k0 * sh2[0]  + k1 * (sh2[3]  * n.y + sh2[6]  * n.z + sh2[9]  * n.x)
                  + k2 * (sh2[12] * n.x * n.y + sh2[15] * n.y * n.z + sh2[18] * (3.0 * zz - 1.0)
                        + sh2[21] * n.x * n.z + sh2[24] * (xx - yy));
    irradiance[1] = k0 * sh2[1]  + k1 * (sh2[4]  * n.y + sh2[7]  * n.z + sh2[10] * n.x)
                  + k2 * (sh2[13] * n.x * n.y + sh2[16] * n.y * n.z + sh2[19] * (3.0 * zz - 1.0)
                        + sh2[22] * n.x * n.z + sh2[25] * (xx - yy));
    irradiance[2] = k0 * sh2[2]  + k1 * (sh2[5]  * n.y + sh2[8]  * n.z + sh2[11] * n.x)
                  + k2 * (sh2[14] * n.x * n.y + sh2[17] * n.y * n.z + sh2[20] * (3.0 * zz - 1.0)
                        + sh2[23] * n.x * n.z + sh2[26] * (xx - yy));
    return irradiance;
}

// Trilinear SH2 irradiance over one level of the probe lattice (regular
// dims^3 grid, index = x + dimX * (y + dimY * z), clamped to the boundary
// probes), validity-weighted. Returns the irradiance and the validity
// coverage of the footprint in giWeight.
vec3 giLevelIrradiance(uint levelOffset, vec3 levelAnchor, vec3 levelDims, float levelSpacing, vec3 position,
    vec3 normal, out float giWeight)
{
    const vec3 cell       = (position - levelAnchor) / levelSpacing;
    const ivec3 base      = clamp(ivec3(floor(cell)), ivec3(0.0), ivec3(levelDims) - ivec3(2.0));
    const vec3 fraction   = clamp(cell - vec3(base), vec3(0.0), vec3(1.0));
    vec3 irradianceSum    = vec3(0.0);
    float weightSum       = 0.0;
    for (int cornerZ = 0; cornerZ < 2; ++cornerZ)
    {
        for (int cornerY = 0; cornerY < 2; ++cornerY)
        {
            for (int cornerX = 0; cornerX < 2; ++cornerX)
            {
                const ivec3 corner = base + ivec3(cornerX, cornerY, cornerZ);
                const uint probeIndex = levelOffset + uint(corner.x + uint(levelDims.x) * (corner.y + uint(levelDims.y) * corner.z));
                if (probeIndex >= uint(frameData.uProbeGridD.x))
                {
                    continue;
                }
                const uint recordBase = probeIndex * 8u;
                const float latticeWeight = (cornerX != 0 ? fraction.x : 1.0 - fraction.x)
                                          * (cornerY != 0 ? fraction.y : 1.0 - fraction.y)
                                          * (cornerZ != 0 ? fraction.z : 1.0 - fraction.z);
                const float weight = latticeWeight * probeRecords[recordBase].w;
                if (weight <= 0.0)
                {
                    continue;
                }
                irradianceSum += weight * giProbeIrradiance(recordBase, normal);
                weightSum += weight;
            }
        }
    }
    giWeight = clamp(weightSum, 0.0, 1.0);
    return weightSum > 0.0 ? irradianceSum / weightSum : vec3(0.0);
}

// Validity-weighted probe-grid irradiance at a world point: level 0 inside
// its half-cell-extended bounds, level 1 elsewhere (boundary clamped).
bool giGridIrradiance(vec3 position, vec3 normal, out vec3 giIrradiance, out float giWeight)
{
    giIrradiance = vec3(0.0);
    giWeight     = 0.0;
    if (frameData.uProbeGridC.w < 0.5 || frameData.uProbeGridD.x < 1.0)
    {
        return false;
    }
    const vec3  anchor0      = probeRecords[uint(frameData.uProbeGridD.z) * 8u].xyz;
    const vec3  local0       = position - anchor0;
    const vec3  extent0      = (vec3(frameData.uProbeGridB.xyz) - vec3(1.0)) * frameData.uProbeGridA.w;
    const float halfCell0    = frameData.uProbeGridA.w * 0.5;
    const bool  insideLevel0 = all(greaterThanEqual(local0, vec3(-halfCell0)))
                            && all(lessThanEqual(local0, extent0 + vec3(halfCell0)));
    if (insideLevel0)
    {
        giIrradiance = giLevelIrradiance(uint(frameData.uProbeGridD.z), anchor0, frameData.uProbeGridB.xyz,
            frameData.uProbeGridA.w, position, normal, giWeight);
    }
    else
    {
        const vec3 anchor1 = probeRecords[uint(frameData.uProbeGridD.w) * 8u].xyz;
        giIrradiance = giLevelIrradiance(uint(frameData.uProbeGridD.w), anchor1, frameData.uProbeGridC.xyz,
            frameData.uProbeGridB.w, position, normal, giWeight);
    }
    return giWeight > 0.0;
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
    // by the occlusion map. When the lumen_gi probe grid holds valid data
    // at this point, its validity-weighted SH2 irradiance replaces the
    // hemispheric term (blended back by coverage so partially populated
    // grids degrade gracefully instead of going black).
    const vec3 upColor   = frameData.uAmbientSky.rgb * frameData.uAmbientSky.a;
    const vec3 downColor = frameData.uAmbientGround.rgb * frameData.uAmbientSky.a;
    const vec3 hemispheric = mix(downColor, upColor, normal.y * 0.5 + 0.5) * baseColor * occlusion;
    vec3 ambient = hemispheric;
    vec3 giIrradiance;
    float giWeight;
    if (giGridIrradiance(vWorldPosition, normal, giIrradiance, giWeight))
    {
        const vec3 giAmbient = baseColor * (giIrradiance * (1.0 / PI)) * occlusion;
        ambient = mix(hemispheric, giAmbient, giWeight);
    }

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
