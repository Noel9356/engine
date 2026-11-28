// The CPU PBR shading pass (the reference twin of shaders/mesh.frag):
// Cook-Torrance BRDF terms, the hemispheric-ambient-or-probe-GI ambient, the
// directional sun, and the prepared direct lights. Formulas, clamps and
// operation order must stay in sync with mesh.frag - changing one side
// requires updating the other (the headless rasterizer consumes these).

#pragma once

#include "direct_light_shading.hpp"
#include "gi_probe_sampling.hpp"
#include "shading_inline.hpp"

#include <engine/pbr/capabilities.hpp>

#include <glm/common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace renderer
{

// BRDF constants shared with shaders/mesh.frag so both backends shade alike.
inline constexpr float     minimumRoughness = 0.045f;
inline constexpr glm::vec3 dielectricF0     = glm::vec3(0.04f);

// Cook-Torrance BRDF terms; formulas must stay in sync with mesh.frag.
inline float pbrDistributionGGX(float nDotH, float roughness)
{
    const float a     = roughness * roughness;
    const float a2    = a * a;
    const float d     = nDotH * nDotH * (a2 - 1.0f) + 1.0f;
    const float invD2 = 1.0f / (d * d);
    return a2 * (1.0f / glm::pi<float>()) * invD2;
}

inline float pbrGeometrySmith(float nDotV, float nDotL, float k)
{
    const float gView  = nDotV / (nDotV * (1.0f - k) + k);
    const float gLight = nDotL / (nDotL * (1.0f - k) + k);
    return gView * gLight;
}

inline glm::vec3 pbrFresnelSchlick(float cosTheta, const glm::vec3 &f0)
{
    const float oneMinusCosine = 1.0f - cosTheta;
    const float squared        = oneMinusCosine * oneMinusCosine;
    const float fifthPower     = squared * squared * oneMinusCosine;
    return f0 + (1.0f - f0) * fifthPower;
}

// Charlie sheen distribution (Estevez & Kulla 2017): the fabric lobe that
// keeps a grazing-angle rim on cloth-like materials. The twin of
// distributionCharlie in shaders/mesh.frag.
inline float pbrDistributionCharlie(float nDotH, float roughness)
{
    const float invAlpha = 1.0f / roughness;
    const float cos2h    = nDotH * nDotH;
    const float sin2h    = glm::max(1.0f - cos2h, 0.0078125f); // 2^(-14/2), so sin2h^(1/alpha) stays finite
    return (2.0f + invAlpha) * std::pow(sin2h, invAlpha * 0.5f) * (1.0f / (2.0f * glm::pi<float>()));
}

// Neubelt visibility (the Ready-at-Dawn cloth visibility term): the smooth
// Ashikhmin approximation of the Smith geometry term with the microfacet
// denominator folded in - the standard partner of the Charlie lobe.
inline float pbrVisibilityNeubelt(float nDotV, float nDotL)
{
    return 1.0f / (4.0f * (nDotL + nDotV - nDotL * nDotV));
}

// A pre-evaluated probe-grid ambient sample: the irradiance walk's result for
// one (position, normal) pair plus its validity coverage. The tile rasterizer
// computes one sample per 2x2 pixel quad when the quad's surface response is
// smooth (same plane, compatible normal) and hands it to pbrShade, so the
// per-pixel probe walk runs a quarter as often with no visible change - the
// term the sample carries is exactly what the walk would return for any
// point of the quad that passes the same gates.
struct GiAmbientSample
{
    glm::vec3 irradiance{0.0f};
    float     coverage = 0.0f;
    // Contact occlusion of the sample (0..1, 1 = open): the gather field's
    // near-contact darkening channel. The indirect term multiplies by it, so
    // the tight dark bands where surfaces meet survive the field's
    // resolution; samples without one (the probe-grid walk) stay 1.
    float     occlusion = 1.0f;
    bool      valid    = false;
};

// Analytic environment specular (the split-sum approximation of the GGX
// environment integral, Karis, "Real Shading in Unreal Engine 4"): the
// hemispheric ambient pair doubles as a two-color environment radiance
// estimate, so every surface - not just the ones a traced reflection field
// answered for - carries the grazing-angle sky sheen that separates a
// rough dielectric from a Lambertian shade. The reflection direction
// picks between ground and sky radiance; roughness blurs the choice toward
// the hemispheric mean (a mirror samples its reflection, a fully rough
// surface sees the direction average); the analytic environment BRDF
// weighs the surface's f0 and the white-tail share the same way the
// split-sum lookup table does. ambientOcclusion darkens it exactly like
// the diffuse ambient it rides with. mesh.frag keeps the twin of this
// function - changing one side requires updating the other.
inline glm::vec3 pbrEnvSpecular(const glm::vec3 &normal, const glm::vec3 &view, float roughness, const glm::vec3 &f0,
    float nDotV, const glm::vec3 &ambientSky, const glm::vec3 &ambientGround, float ambientOcclusion)
{
    const glm::vec3 reflected = glm::reflect(-view, normal);
    const float     blur      = roughness * roughness;
    const float     lobeUp    = glm::mix(reflected.y * 0.5f + 0.5f, 0.5f, blur);
    const glm::vec3 radiance  = glm::mix(ambientGround, ambientSky, lobeUp);
    // The environment BRDF fit: r = roughness * x + y, a004 = ... (the
    // constants are the published split-sum fit; the clamped exponential
    // keeps the grazing tail finite exactly like the reference).
    const glm::vec4 fitX(-1.0f, -0.0275f, -0.572f, 0.022f);
    const glm::vec4 fitY(1.0f, 0.0425f, 1.04f, -0.04f);
    const glm::vec4 r        = glm::vec4(roughness) * fitX + fitY;
    const float     a004     = std::min(r.x * r.x, std::exp(-9.28f * nDotV)) * r.x + r.y;
    const glm::vec2 ab(-1.04f * a004 + r.z, 1.04f * a004 + r.w);
    return radiance * (f0 * ab.x + glm::vec3(ab.y)) * ambientOcclusion;
}

// Full CPU shading pass: hemispheric ambient (or the radiance_cache probe-grid
// indirect term blended over it by probe validity), one directional sun, then
// the prepared direct lights (range-faded point/spot attenuation with
// smoothstep cones). Roughness/metallic are clamped here exactly like the
// shader. giGrid is the borrowed probe-grid snapshot; nullptr keeps the
// legacy ambient path bit-identical. giSample, when non-null, replaces the
// per-pixel probe walk with a caller-evaluated sample (see GiAmbientSample).
// envSpecularScale weighs the analytic environment specular (see
// pbrEnvSpecular): callers whose traced reflection field already answered for
// this sample pass 0 so the two specular-environment terms never double.
ENGINE_SHADING_INLINE glm::vec3 pbrShade(const glm::vec3 &normal, const glm::vec3 &worldPosition,
    const glm::vec3 &cameraPosition, const glm::vec3 &lightDirection, const glm::vec3 &sunRadiance,
    const glm::vec3 &ambientSky, const glm::vec3 &ambientGround, const glm::vec3 &albedo, float metallic,
    float roughness, float ambientOcclusion, const PreparedDirectLight *directLights,
    std::uint32_t directLightCount, const PreparedGiProbeGrid *giGrid = nullptr, float opacity = 1.0f,
    float sheenFactor = 0.0f, const GiAmbientSample *giSample = nullptr, float envSpecularScale = 1.0f)
{
    // Metallic surfaces carry almost no diffuse response. The hemispheric
    // ambient and the probe-grid GI are diffuse integrals, so they scale by
    // (1 - metallic) exactly like the direct-lighting diffuse below
    // (mesh.frag keeps the twin of this factor).
    metallic              = std::clamp(metallic, 0.0f, 1.0f);
    const glm::vec3 diffuseAlbedo = albedo * (1.0f - metallic);
    const glm::vec3 hemispheric = glm::mix(ambientGround, ambientSky, normal.y * 0.5f + 0.5f) * diffuseAlbedo
                                  * ambientOcclusion;
    glm::vec3       ambient     = hemispheric;
    // The SH2 convolution returns incident irradiance; a Lambertian BRDF
    // turns it into outgoing radiance as albedo / pi * E. The occlusion map
    // darkens the indirect term like the ambient it replaces. The irradiance
    // is already renormalized over the valid probes of the footprint, so a
    // quarter of valid coverage is enough to trust it (probe lattices
    // routinely lose the corners that fall inside nearby geometry); the
    // ambient remains the fallback only while almost no probe is valid.
    const auto blendGiAmbient = [&](const glm::vec3 &giIrradiance, float giWeight, float giOcclusion)
    {
        const float     giTrust   = glm::smoothstep(0.0f, 0.25f, giWeight);
        const glm::vec3 giAmbient = diffuseAlbedo * (giIrradiance * (1.0f / glm::pi<float>())) * ambientOcclusion
            * giOcclusion;
        ambient                   = glm::mix(hemispheric, giAmbient, giTrust);
    };
    // The hemispheric ambient and the probe-grid GI are diffuse integrals, so
    // a fully metallic or fully transparent surface carries no diffuse
    // response at all - the probe walk would multiply zero (mesh.frag's twin
    // of this early-out).
    if (giSample != nullptr)
    {
        if (giSample->valid)
        {
            blendGiAmbient(giSample->irradiance, giSample->coverage, giSample->occlusion);
        }
    }
    else if (giGrid != nullptr && giGrid->valid
        && diffuseAlbedo.x + diffuseAlbedo.y + diffuseAlbedo.z > 1.0e-6f)
    {
        glm::vec3 giIrradiance(0.0f);
        float     giWeight         = 0.0f;
        if (giProbeGridIrradiance(*giGrid, worldPosition, normal, giIrradiance, giWeight))
        {
            blendGiAmbient(giIrradiance, giWeight, 1.0f);
        }
    }
    glm::vec3 result = ambient;

    roughness = std::clamp(roughness, minimumRoughness, 1.0f);

    const glm::vec3 view    = safeNormalize(cameraPosition - worldPosition);
    const float     nDotV   = std::max(glm::dot(normal, view), 1.0e-4f);
    const glm::vec3 f0      = glm::mix(dielectricF0, albedo, metallic);
    // Analytic environment specular (see pbrEnvSpecular): the ambient pair
    // is all the environment this pass knows about, so the term rides every
    // lit sample - scaled to zero when the caller's traced reflection field
    // already carries the surface's specular-environment response.
    if (envSpecularScale > 0.0f)
    {
        result += envSpecularScale
            * pbrEnvSpecular(normal, view, roughness, f0, nDotV, ambientSky, ambientGround, ambientOcclusion);
    }
    const float     invPi   = 1.0f / glm::pi<float>();
    const glm::vec3 diffuse = albedo * ((1.0f - metallic) * invPi);
    // Fabric (cloth) shading, the twin of shaders/mesh.frag's shadeDirectLight:
    // sheenFactor > 0 blends the GGX specular toward the Charlie/Neubelt
    // fabric lobe (sheen highlight tinted toward the base color) and replaces
    // the plain nDotL diffuse weight with the energy-conserving wrap factor.
    sheenFactor            = glm::clamp(sheenFactor, 0.0f, 1.0f);
    const bool  clothShading = sheenFactor > 0.0f;
    const float wrapWeight   = 0.5f * sheenFactor;
    const float wrapDivisor  = (1.0f + wrapWeight) * (1.0f + wrapWeight);
    const glm::vec3 sheenColor = glm::mix(glm::vec3(1.0f), albedo, 0.6f);

    const float geomK = (roughness + 1.0f) * (roughness + 1.0f) * 0.125f;

    const float sunNDotL = std::max(glm::dot(normal, lightDirection), 0.0f);
    if (sunNDotL > 0.0f && (sunRadiance.x > 0.0f || sunRadiance.y > 0.0f || sunRadiance.z > 0.0f))
    {
        const glm::vec3 halfVector = safeNormalize(view + lightDirection);
        const float     nDotH      = std::max(glm::dot(normal, halfVector), 0.0f);
        const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);
        const float     dTerm      = pbrDistributionGGX(nDotH, roughness);
        const float     gTerm      = pbrGeometrySmith(nDotV, sunNDotL, geomK);
        const glm::vec3 fTerm      = pbrFresnelSchlick(vDotH, f0);

        const float     invSpec  = 1.0f / (4.0f * nDotV * sunNDotL + 1.0e-5f);
        glm::vec3       specular = (dTerm * gTerm * invSpec) * fTerm;
        float sunDiffuseWeight = sunNDotL;
        if (clothShading)
        {
            const glm::vec3 sheenSpecular
                = glm::vec3(pbrDistributionCharlie(nDotH, roughness) * pbrVisibilityNeubelt(nDotV, sunNDotL))
                  * sheenColor;
            specular         = glm::mix(specular, sheenSpecular, sheenFactor);
            sunDiffuseWeight = glm::max((sunNDotL + wrapWeight) / wrapDivisor, 0.0f);
        }
        result += (diffuse + specular) * sunRadiance * sunDiffuseWeight;
    }

    const std::uint32_t lightCount = std::min(directLightCount, direct_lighting::maxDirectLights);
    for (std::uint32_t index = 0; index < lightCount; ++index)
    {
        const PreparedDirectLight &light = directLights[index];
        glm::vec3                  localLightDirection;
        float                      attenuation = 1.0f;
        if (!resolveDirectLight(light, worldPosition, localLightDirection, attenuation))
        {
            continue;
        }
        const float localNDotL = std::max(glm::dot(normal, localLightDirection), 0.0f);
        if (localNDotL <= 0.0f)
        {
            continue;
        }
        const glm::vec3 halfVector = safeNormalize(view + localLightDirection);
        const float     nDotH      = std::max(glm::dot(normal, halfVector), 0.0f);
        const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);

        const float     dTerm    = pbrDistributionGGX(nDotH, roughness);
        const float     gTerm    = pbrGeometrySmith(nDotV, localNDotL, geomK);
        const glm::vec3 fTerm    = pbrFresnelSchlick(vDotH, f0);
        const float     invSpec  = 1.0f / (4.0f * nDotV * localNDotL + 1.0e-5f);
        glm::vec3       specular = (dTerm * gTerm * invSpec) * fTerm;
        float lightDiffuseWeight = localNDotL;
        if (clothShading)
        {
            const glm::vec3 sheenSpecular
                = glm::vec3(pbrDistributionCharlie(nDotH, roughness) * pbrVisibilityNeubelt(nDotV, localNDotL))
                  * sheenColor;
            specular          = glm::mix(specular, sheenSpecular, sheenFactor);
            lightDiffuseWeight = glm::max((localNDotL + wrapWeight) / wrapDivisor, 0.0f);
        }
        result += (diffuse + specular) * light.radiance * attenuation * lightDiffuseWeight;
    }
    return result;
}

} // namespace renderer
} // namespace engine
