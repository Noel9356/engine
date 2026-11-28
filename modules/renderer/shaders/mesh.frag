#version 450

layout(constant_id = 0) const uint MAX_LIGHTS = 32u;
layout(constant_id = 6) const int SRGB_OUTPUT = 0;
// Fragment stage of the renderer module's Vulkan backend: the engine's PBR
// shading model, metallic-roughness workflow, with the fabric (cloth) shading
// path driven by the material's sheenFactor/doubleSided pair - the Charlie
// sheen lobe (Estevez & Kulla 2017) with the Neubelt visibility term, the
// energy-conserving wrap-diffuse terminator, and the two-sided normal flip.
//
// The lighting model must agree with the software rasterizer in
// headless_backend.cpp - same formulas, same constants, same clamps - so
// both backends keep producing the same image. Changing either side
// requires updating the rasterizer; the build recompiles this file to
// SPIR-V automatically (cmake/engine_module_compile_shaders.cmake).
//
// NOTE on early fragment tests: this shader DOES discard (the material's
// alphaCutoff masking, uCloth.z), so layout(early_fragment_tests) is
// deliberately absent - a discard would invalidate the early test anyway
// and the depth pre-test's benefit is lost exactly on masked draws.

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
    vec4 uCloth;             // x sheenFactor, y doubleSided flag (1 = two-sided draw), z alphaCutoff, w opacity
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
    // radiance_cache probe-grid header (the indirect-lighting hook; the SH2 probe
    // records ride the set-0 binding-1 storage buffer below):
    vec4 uProbeGridA; // xyz = grid center, w = level0 spacing
    vec4 uProbeGridB; // xyz = level0 dimensions, w = level1 spacing
    vec4 uProbeGridC; // xyz = level1 dimensions, w = grid active (1.0 = probes bound)
    vec4 uProbeGridD; // x = probe count, y = direction resolution, z = level0 offset, w = level1 offset
    // reflection_field screen-space field header (the indirect-specular hook;
    // the RGBA32F texels ride the set-0 binding-2 sampler below): the field's
    // own GL-convention camera plus (width, height, max traced roughness,
    // active). mesh.vert reads only the leading members, so its shorter
    // block declaration stays valid.
    mat4 uFieldViewProj;
    vec4 uFieldMeta;      // x = width, y = height, z = max roughness, w = active
} frameData;

// radiance_cache probe records, 8 vec4 per probe: (position.xyz, validity) then
// the 27 SH2 radiance floats (GiProbe [basis][channel] order) plus one pad.
// Written by the renderer module from the radiance_cache copyProbes snapshot.
layout(std430, set = 0, binding = 1) readonly buffer ProbeData
{
    vec4 probeRecords[];
};

// reflection_field screen-space field: RGBA32F, one texel per field cell
// (radiance.rgb + the texel's surface roughness in alpha; alpha at the
// neutral 1.0 marks an untraced texel). Read through texelFetch - the manual
// bilinear below needs no filtering, so float-filter device support never
// gates the hook.
layout(set = 0, binding = 2) uniform sampler2D uReflectionField;

// One material descriptor set (the layout's set 1): six combined image
// samplers, one binding per map. The renderer fills every slot - a material
// without a map samples the neutral stand-in - so the declarations must keep
// these exact set/binding numbers (they must match vulkan_pipeline.cpp's
// materialSetLayout and VulkanResources' descriptor writes).
layout(set = 1, binding = 0) uniform sampler2D uBaseColorTexture;
layout(set = 1, binding = 1) uniform sampler2D uNormalTexture;
layout(set = 1, binding = 2) uniform sampler2D uMetallicRoughnessTexture;
layout(set = 1, binding = 3) uniform sampler2D uOcclusionTexture;
layout(set = 1, binding = 4) uniform sampler2D uEmissiveTexture;
layout(set = 1, binding = 5) uniform sampler2D uDetailNormalTexture;

// The material extension block (the layout's set 2, a dynamic-offset UBO):
// the per-material fields that do not fit the 128-byte push-constant budget
// - the distance-fog triple, the procedural mottle pair and the near-field
// relief overlay's triple (see MeshMaterial's appended fields; the CPU
// rasterizer carries the same values through RasterSetup). The renderer
// binds one block per distinct tuple with the draw's dynamic offset.
layout(std140, set = 2, binding = 0) uniform MaterialExtension
{
    vec4 uFog;        // rgb fog color, a fog asymptote (0 disables)
    vec4 uFogMottle;  // x fog scale meters, y noise scale meters, z noise strength, w unused
    vec4 uDetail;     // x detail uv scale, y detail strength, z detail fade start meters, w detail fade end meters
} materialExtension;

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

// Charlie sheen distribution (Estevez & Kulla 2017, "Production Friendly
// Microfacet Sheen BRDF"): an exponentiated-sinusoid fabric lobe that keeps
// the grazing-angle rim of velvet-like materials without the harsh
// near-ndotl-1 transitions of the Ashikhmin velvet distribution.
float distributionCharlie(float roughness, float nDotH)
{
    const float invAlpha = 1.0 / roughness;
    const float cos2h    = nDotH * nDotH;
    const float sin2h    = max(1.0 - cos2h, 0.0078125); // 2^(-14/2), so sin2h^(1/alpha) stays finite in fp16
    return (2.0 + invAlpha) * pow(sin2h, invAlpha * 0.5) / (2.0 * PI);
}

// Neubelt visibility (the Ready-at-Dawn cloth visibility term): the smooth
// Ashikhmin approximation of the Smith geometry term with the microfacet
// denominator folded in - the standard partner of the Charlie lobe.
float visibilityNeubelt(float nDotV, float nDotL)
{
    return 1.0 / (4.0 * (nDotL + nDotV - nDotL * nDotV));
}

// One direct light's contribution: the standard GGX specular blended toward
// the Charlie/Neubelt fabric lobe by sheenFactor, with the energy-conserving
// wrap-diffuse weight ((nDotL + w) / (1 + w)^2, w = 0.5 x sheenFactor)
// replacing the plain nDotL diffuse weight when cloth shading is active.
// The sheen highlight is tinted toward the fabric's base color, so colored
// cloth shows a colored rim instead of the dielectric-gray GGX highlight.
vec3 shadeDirectLight(vec3 normal, vec3 view, vec3 lightDirection, vec3 radiance, vec3 diffuse, vec3 f0,
    vec3 baseColor, float roughness, float sheenFactor, float nDotV)
{
    const float nDotL = max(dot(normal, lightDirection), 0.0);
    if (nDotL <= 0.0)
    {
        return vec3(0.0);
    }
    const vec3  halfVector = normalize(view + lightDirection);
    const float nDotH      = max(dot(normal, halfVector), 0.0);
    const float vDotH      = max(dot(view, halfVector), 0.0);

    const float dTerm = distributionGGX(nDotH, roughness);
    const float gTerm = geometrySmith(nDotV, nDotL, roughness);
    const vec3  fTerm = fresnelSchlick(vDotH, f0);
    vec3  specular    = (dTerm * gTerm * fTerm) / vec3(4.0 * nDotV * nDotL + 1e-5);
    float diffuseWeight = nDotL;
    if (sheenFactor > 0.0)
    {
        const float sheenD      = distributionCharlie(roughness, nDotH);
        const float sheenV      = visibilityNeubelt(nDotV, nDotL);
        const vec3  sheenColor  = mix(vec3(1.0), baseColor, 0.6);
        const vec3  sheenSpec   = vec3(sheenD * sheenV) * sheenColor;
        specular                = mix(specular, sheenSpec, sheenFactor);
        const float wrap        = 0.5 * sheenFactor;
        diffuseWeight           = max((nDotL + wrap) / ((1.0 + wrap) * (1.0 + wrap)), 0.0);
    }
    return (diffuse + specular) * radiance * diffuseWeight;
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

// ---- radiance_cache probe-grid indirect lighting ----------------------------------
// Mirrors the CPU path in source/core/gi_probe_sampling.hpp exactly (same lattice
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
    // Smoothstep per axis (the DDGI probe-blend convention; the CPU twin in
    // gi_probe_sampling.hpp carries the full rationale): C1 lattice joins so
    // the probe grid stops reading as a world-aligned cell pattern.
    const vec3 raw        = clamp(cell - vec3(base), vec3(0.0), vec3(1.0));
    const vec3 fraction   = raw * raw * (3.0 - 2.0 * raw);
    // Tangent-plane anti-leak weight (the CPU twin's
    // probeTangentPlaneAlignmentFactor, in its factored form): the base
    // alignment measured against the cell's base corner plus the corner's
    // unit offsets dotted with the normal, so a probe on the far side of
    // the shaded point's tangent plane drops out instead of lighting the
    // surface through walls. Probes lying in the plane keep half weight.
    const float inverseSpacing = 1.0 / max(levelSpacing, 1.0e-5);
    const float baseAlignment  = dot(normal, levelAnchor + vec3(base) * levelSpacing - position) * inverseSpacing;
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
                float weight = latticeWeight * probeRecords[recordBase].w;
                if (weight <= 0.0)
                {
                    continue;
                }
                float alignment = baseAlignment;
                if (cornerX != 0)
                {
                    alignment += normal.x;
                }
                if (cornerY != 0)
                {
                    alignment += normal.y;
                }
                if (cornerZ != 0)
                {
                    alignment += normal.z;
                }
                weight *= clamp(alignment * 0.5 + 0.5, 0.0, 1.0);
                if (weight <= 1.0e-8)
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

// ---- reflection_field screen-space indirect specular -------------------------
// Mirrors the CPU path in source/core/reflection_field_sampling.hpp exactly
// (same projection, same gates, same renormalizing bilinear): the shaded
// point projects through the field's own camera, the four surrounding
// texels blend, and a texel is refused when it carries the neutral roughness
// (no traced response) or sits outside the pixel's roughness band. An empty
// neighborhood reports traced = false. The neutral roughness and band width
// are the reflection_field ABI constants.
const float kFieldNeutralRoughness = 1.0;
const float kFieldRoughnessBand    = 0.15;

vec3 sampleReflectionField(vec3 worldPosition, float pixelRoughness, out bool traced)
{
    traced = false;
    if (frameData.uFieldMeta.w < 0.5 || pixelRoughness > frameData.uFieldMeta.z)
    {
        return vec3(0.0);
    }
    const vec4 clip = frameData.uFieldViewProj * vec4(worldPosition, 1.0);
    if (clip.w <= 0.05)
    {
        return vec3(0.0);
    }
    const float width  = frameData.uFieldMeta.x;
    const float height = frameData.uFieldMeta.y;
    const float u = (clip.x / clip.w * 0.5 + 0.5) * width - 0.5;
    const float v = (0.5 - clip.y / clip.w * 0.5) * height - 0.5;
    if (u < 0.0 || v < 0.0 || u > width - 1.0 || v > height - 1.0)
    {
        return vec3(0.0);
    }
    const ivec2 base = ivec2(floor(vec2(u, v)));
    const ivec2 far  = min(base + ivec2(1), ivec2(width, height) - ivec2(1));
    const vec2  fraction = vec2(u, v) - vec2(base);

    const vec4 t00 = texelFetch(uReflectionField, base, 0);
    const vec4 t10 = texelFetch(uReflectionField, ivec2(far.x, base.y), 0);
    const vec4 t01 = texelFetch(uReflectionField, ivec2(base.x, far.y), 0);
    const vec4 t11 = texelFetch(uReflectionField, far, 0);

    const bool traced00 = t00.a < kFieldNeutralRoughness && abs(t00.a - pixelRoughness) <= kFieldRoughnessBand;
    const bool traced10 = t10.a < kFieldNeutralRoughness && abs(t10.a - pixelRoughness) <= kFieldRoughnessBand;
    const bool traced01 = t01.a < kFieldNeutralRoughness && abs(t01.a - pixelRoughness) <= kFieldRoughnessBand;
    const bool traced11 = t11.a < kFieldNeutralRoughness && abs(t11.a - pixelRoughness) <= kFieldRoughnessBand;

    const float w00 = traced00 ? (1.0 - fraction.x) * (1.0 - fraction.y) : 0.0;
    const float w10 = traced10 ? fraction.x * (1.0 - fraction.y) : 0.0;
    const float w01 = traced01 ? (1.0 - fraction.x) * fraction.y : 0.0;
    const float w11 = traced11 ? fraction.x * fraction.y : 0.0;
    const float weightSum = w00 + w10 + w01 + w11;
    if (weightSum <= 0.0)
    {
        return vec3(0.0);
    }
    traced = true;
    return (t00.rgb * w00 + t10.rgb * w10 + t01.rgb * w01 + t11.rgb * w11) / weightSum;
}

// Procedural surface mottle (the CPU rasterizer's mottle_noise.hpp twin):
// a footprint-adaptive two-octave 3D value noise that perturbs the shaded
// albedo. The GPU evaluates the same avalanche-hash lattice; the footprint
// floor uses the pixel's world-position derivatives (dFdx/dFdy), the CPU
// twin's per-record estimate.
float mottleLattice(ivec3 c)
{
    uint state = uint(c.x) * 0x8DA6B343u ^ uint(c.y) * 0xD816394Cu ^ uint(c.z) * 0xCB1AB31Fu;
    state ^= state >> 15;
    state *= 0x2C1B3C6Du;
    state ^= state >> 12;
    state *= 0x97A2D39Bu;
    state ^= state >> 15;
    return float(state & 0x00FFFFFFu) * (1.0 / 16777216.0);
}

float mottleValueNoise(vec3 p)
{
    const ivec3 base = ivec3(floor(p));
    const vec3 f = p - vec3(base);
    const vec3 fade = f * f * (3.0 - 2.0 * f);
    const float c000 = mottleLattice(base);
    const float c100 = mottleLattice(base + ivec3(1, 0, 0));
    const float c010 = mottleLattice(base + ivec3(0, 1, 0));
    const float c110 = mottleLattice(base + ivec3(1, 1, 0));
    const float c001 = mottleLattice(base + ivec3(0, 0, 1));
    const float c101 = mottleLattice(base + ivec3(1, 0, 1));
    const float c011 = mottleLattice(base + ivec3(0, 1, 1));
    const float c111 = mottleLattice(base + ivec3(1, 1, 1));
    const float x00 = c000 + (c100 - c000) * fade.x;
    const float x10 = c010 + (c110 - c010) * fade.x;
    const float x01 = c001 + (c101 - c001) * fade.x;
    const float x11 = c011 + (c111 - c011) * fade.x;
    const float y0 = x00 + (x10 - x00) * fade.y;
    const float y1 = x01 + (x11 - x01) * fade.y;
    return y0 + (y1 - y0) * fade.z;
}

float mottleAlbedoModulation(vec3 worldPosition, float baseScaleMeters, float footprintMeters, float strength)
{
    const float scaleLow = max(baseScaleMeters, footprintMeters);
    const float scaleHigh = max(baseScaleMeters * 0.5, footprintMeters);
    const float n = 0.6 * mottleValueNoise(worldPosition * (1.0 / scaleLow))
        + 0.4 * mottleValueNoise(worldPosition * (1.0 / scaleHigh));
    return 1.0 + strength * (2.0 * n - 1.0);
}

// Analytic environment specular (pbrEnvSpecular's GLSL twin in
// source/core/pbr_shading.hpp): the hemispheric ambient pair doubles as a
// two-color environment radiance estimate, so every surface carries the
// grazing-angle sky sheen that separates a rough dielectric from a
// Lambertian shade. Formulas and constants must stay in sync with the CPU
// twin.
vec3 envSpecular(vec3 normal, vec3 view, float roughness, vec3 f0, float nDotV,
    vec3 ambientSky, vec3 ambientGround, float ambientOcclusion)
{
    const vec3 reflected = reflect(-view, normal);
    const float blur = roughness * roughness;
    const float lobeUp = mix(reflected.y * 0.5 + 0.5, 0.5, blur);
    const vec3 radiance = mix(ambientGround, ambientSky, lobeUp);
    const vec4 fitX = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 fitY = vec4(1.0, 0.0425, 1.04, -0.04);
    const vec4 r = vec4(roughness) * fitX + fitY;
    const float a004 = min(r.x * r.x, exp(-9.28 * nDotV)) * r.x + r.y;
    const vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
    return radiance * (f0 * ab.x + vec3(ab.y)) * ambientOcclusion;
}

void main()
{
    const vec3 geometryNormal = normalize(vNormal);

    // Two-sided draws (cloth, flags, thin sheets) skip back-face culling, so
    // back-facing fragments arrive here with normals pointing away from the
    // camera. Flipping the geometry normal toward the viewer before the
    // tangent frame is built keeps both sides of the sheet lit identically
    // (the glTF doubleSided contract: back-face normals are reversed before
    // the lighting equation is evaluated).
    const bool twoSided = pushData.uCloth.y > 0.5;
    const vec3 view     = normalize(frameData.uCameraPos.xyz - vWorldPosition);
    vec3 orientedNormal = geometryNormal;
    if (twoSided && dot(orientedNormal, view) < 0.0)
    {
        orientedNormal = -orientedNormal;
    }

    // ---- material assembly: the five maps times their factors
    // The base-color map samples ONCE: its RGB shades the surface and its
    // alpha feeds both the cutoff mask (uCloth.z, 0 = disabled - the
    // rasterizer's per-texel skip twin) and the transparent commit's
    // effective opacity (see oColor below).
    const vec4 baseColorSample = texture(uBaseColorTexture, vUv);
    if (pushData.uCloth.z > 0.0 && baseColorSample.a < pushData.uCloth.z)
    {
        discard;
    }
    vec3 baseColor = vColor * pushData.uBaseColorFactor.rgb
                   * baseColorSample.rgb;

    // Procedural surface mottle (the rasterizer's mottle_noise.hpp twin):
    // the footprint-adaptive two-octave value noise at the fragment's world
    // position multiplies the albedo. The footprint floor rides the
    // world-position derivatives (the CPU twin's per-record estimate).
    if (materialExtension.uFogMottle.y > 0.0 && materialExtension.uFogMottle.z > 0.0)
    {
        const vec3 footprintVector = max(abs(dFdx(vWorldPosition)), abs(dFdy(vWorldPosition)));
        const float footprint = max(max(footprintVector.x, footprintVector.y), footprintVector.z);
        baseColor *= mottleAlbedoModulation(vWorldPosition, materialExtension.uFogMottle.y, footprint,
            materialExtension.uFogMottle.z);
    }

    // Perturbed normal: decode the tangent-space map, scale the tangential
    // part by normalScale, and rotate into world space. The neutral flat
    // normal (0.5, 0.5, 1.0) decodes to (0, 0, 1) and leaves the geometry
    // normal standing. The near-field relief overlay (the rasterizer's
    // MeshMaterial detail triple) perturbs the tangent-space normal first:
    // full at the fade's start distance, gone at its end, so the ground
    // gains real grain while the eye descends (the slope-add blend keeps
    // the base map's larger relief standing).
    const vec3  normalMapSample = texture(uNormalTexture, vUv).xyz * 2.0 - 1.0;
    vec3        tangentNormal   = normalize(vec3(normalMapSample.xy * pushData.uMetallicRoughness.z, normalMapSample.z));
    if (materialExtension.uDetail.x > 0.0)
    {
        const float pixelDistance = distance(vWorldPosition, frameData.uCameraPos.xyz);
        const float fade = clamp((materialExtension.uDetail.w - pixelDistance)
            / max(materialExtension.uDetail.w - materialExtension.uDetail.z, 1e-3), 0.0, 1.0);
        if (fade > 0.0)
        {
            const vec2 detailUv = vUv * materialExtension.uDetail.x;
            const vec3 detailSample = texture(uDetailNormalTexture, detailUv).xyz * 2.0 - 1.0;
            tangentNormal = normalize(tangentNormal
                + fade * materialExtension.uDetail.y * vec3(detailSample.xy, 0.0));
        }
    }
    const mat3  tbn             = tangentFrame(orientedNormal, vWorldPosition, vUv);
    const vec3  normal          = normalize(tbn * tangentNormal);

    // Metallic-roughness: map channels times factors (glTF convention).
    const vec2  metallicRoughnessSample = texture(uMetallicRoughnessTexture, vUv).gb;
    const float metallic  = clamp(metallicRoughnessSample.y * pushData.uMetallicRoughness.x, 0.0, 1.0);
    const float roughness = clamp(metallicRoughnessSample.x * pushData.uMetallicRoughness.y, minimumRoughness, 1.0);

    // Ambient occlusion: 1 - strength * (1 - sampled) darkens the ambient
    // light only; the neutral white map (sampled = 1) is a no-op.
    const float occlusion = 1.0 - pushData.uMetallicRoughness.w * (1.0 - texture(uOcclusionTexture, vUv).r);

    // Direct light: one directional sun.
    const float sheenFactor = clamp(pushData.uCloth.x, 0.0, 1.0);
    const vec3  lightDirection = normalize(frameData.uSunDirection.xyz);
    const float nDotL = max(dot(normal, lightDirection), 0.0);
    const float nDotV = max(dot(normal, view), 1e-4);

    const vec3  f0           = mix(vec3(0.04), baseColor, metallic);
    const vec3  sunRadiance  = frameData.uSunColor.rgb * frameData.uSunColor.a * frameData.uSunDirection.w;
    // Transparent surfaces scatter only the share they keep off the
    // transmitted path, so the diffuse terms scale by the opacity (the
    // pbrShade twin in source/core/pbr_shading.hpp; the transmitted share
    // lives in the reflection field and the alpha blend, never in the
    // surface's own diffuse response).
    const float opacityFactor = clamp(pushData.uCloth.w, 0.0, 1.0);
    const vec3  diffuse      = baseColor * (1.0 - metallic) / PI * opacityFactor;

    const vec3 sunContribution = shadeDirectLight(
        normal, view, lightDirection, sunRadiance, diffuse, f0, baseColor, roughness, sheenFactor, nDotV);

    // Hemispheric ambient: sky color for up-facing normals, ground bounce
    // for down-facing ones, blended by the world-space normal's y, scaled
    // by the occlusion map. When the radiance_cache probe grid holds valid data
    // at this point, its validity-weighted SH2 irradiance replaces the
    // hemispheric term (blended back by coverage so partially populated
    // grids degrade gracefully instead of going black).
    // Metallic surfaces carry almost no diffuse response. The hemispheric
    // ambient and the probe-grid GI are diffuse integrals, so they scale by
    // (1 - metallic) exactly like the direct-lighting diffuse above
    // (pbrShade keeps the twin of this factor).
    const vec3 diffuseAlbedo = baseColor * (1.0 - metallic) * opacityFactor;
    const vec3 upColor   = frameData.uAmbientSky.rgb * frameData.uAmbientSky.a;
    const vec3 downColor = frameData.uAmbientGround.rgb * frameData.uAmbientSky.a;
    const vec3 hemispheric = mix(downColor, upColor, normal.y * 0.5 + 0.5) * diffuseAlbedo * occlusion;
    vec3 ambient = hemispheric;
    vec3 giIrradiance;
    float giWeight;
    if (giGridIrradiance(vWorldPosition, normal, giIrradiance, giWeight))
    {
        // The irradiance is already renormalized over the valid probes of the
        // footprint, so a quarter of valid coverage is enough to trust it
        // (probe lattices routinely lose the corners that fall inside nearby
        // geometry); the ambient remains the fallback only while almost no
        // probe is valid.
        const float giTrust   = smoothstep(0.0, 0.25, giWeight);
        const vec3 giAmbient = diffuseAlbedo * (giIrradiance * (1.0 / PI)) * occlusion;
        ambient = mix(hemispheric, giAmbient, giTrust);
    }

    vec3 color = sunContribution + ambient;
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
        const vec3 radiance = light.colorIntensity.rgb * light.colorIntensity.a * attenuation;
        if (attenuation <= 0.0)
        {
            continue;
        }
        color += shadeDirectLight(
            normal, view, lightDirection, radiance, diffuse, f0, baseColor, roughness, sheenFactor, nDotV);
    }

    // Indirect specular (reflection_field field hook): the field texel this
    // fragment projects into carries the expected GGX-lobe response of its
    // material, so the add is the whole composite - the rasterizer's
    // sampleReflectionField twin, in linear space before the tone map. Lit
    // draws only (the software rasterizer's unlit path skips the hook).
    // The analytic environment specular (the rasterizer's envSpecularScale
    // handoff twin) applies wherever the field did NOT answer - the field's
    // traced response already contains the surface's specular-environment
    // share, and the two terms must not double.
    bool fieldTraced = false;
    if (pushData.uEmissive.a <= 0.5)
    {
        color += sampleReflectionField(vWorldPosition, roughness, fieldTraced);
        if (!fieldTraced)
        {
            color += envSpecular(normal, view, roughness, f0, nDotV, upColor, downColor, occlusion);
        }
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

    // Distance fog (MeshMaterial's fog triple, the rasterizer's twin block):
    // per-pixel, in linear space, after every lighting and emissive term,
    // before the output transform. The fragment's own position carries the
    // weight, so the fog stays continuous across mesh and chunk boundaries.
    // Unlit draws fog too (a distant emissive card fades like the terrain
    // behind it).
    if (materialExtension.uFog.a > 0.0)
    {
        const float fogWeight = materialExtension.uFog.a
            * (1.0 - exp(-distance(vWorldPosition, frameData.uCameraPos.xyz) / materialExtension.uFogMottle.x));
        color = mix(color, materialExtension.uFog.rgb, fogWeight);
    }

    // Exposure + tone mapping. The linear-to-sRGB output transform is
    // conditional on SRGB_OUTPUT (specialization constant, default 0):
    //   0 = UNORM target, manual pow(1/2.2) gamma (original behavior).
    //   1 = _SRGB target, hardware does the conversion (skip manual gamma).
    // Default 0 = safe = original behavior.
    color = toneMap(color * frameData.uToneMapping.x, frameData.uToneMapping.y);
    if (SRGB_OUTPUT == 0)
    {
        color = pow(max(color, vec3(0.0)), vec3(1.0 / 2.2));
    }

    // The alpha channel carries the material's opacity times the base-color
    // map's texel alpha (1 for the neutral stand-in), so the transparent
    // pipeline's SRC_ALPHA blend matches the headless rasterizer's
    // opacity-times-texel-alpha commit: foam patches and mist banks with
    // soft alpha textures fade per texel, opaque draws ignore the channel.
    // A transmissive surface with a traced field response commits opaque
    // instead: the field's response already carries the interface's
    // reflected share and the refracted transmitted share, so blending the
    // raw background in as well would double it (the rasterizer's
    // transmissive-commit twin).
    float outputAlpha = pushData.uCloth.w * baseColorSample.a;
    if (pushData.uCloth.w < 1.0 && fieldTraced)
    {
        outputAlpha = 1.0;
    }
    oColor = vec4(color, outputAlpha);
}
