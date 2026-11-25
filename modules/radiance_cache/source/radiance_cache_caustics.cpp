// radiance_cache caustics: photon-differential splatting into the card surface cache.
//
// Light that transmissive surfaces focus - the caustic pattern a glass sphere
// throws onto the floor, or sunlit water throws onto its bed - is rendered by
// tracing photons from the light sources to the transmissive surfaces,
// refracting them with the same Snell / total-internal-reflection rules every
// other ray of this module uses, and depositing the focused flux as caustic
// irradiance on the receivers' card texels. Deposits land in the Lumen Scene
// (the surface cache), so every consumer of the cache picks them up exactly
// like any other direct lighting: the amortized indirect gathers bounce off
// caustic-lit surfaces, the world radiance cache probes carry the caustic
// radiance (the SH2 band limit softens it there, the same limit every small
// bright emitter faces), and the screen-space final gather resolves it at the
// captured resolution.
//
// Photon differentials give each photon a footprint without closed-form
// differential calculus: every photon is traced as three rays (the center
// plus two auxiliary rays offset on the emission plane), and the auxiliary
// rays refract independently, so the triangle the three landings span is the
// emission cell's image under the actual transport - the focus factor falls
// out of its area. The deposited irradiance is the photon flux divided by
// that footprint area, splatted over the covered texels with a normalized
// kernel, so a focused photon concentrates its energy exactly as the optics
// dictate. Clamped footprint radii keep the estimator bounded (a texel-sized
// floor stops single-texel fireflies, a scene-scale ceiling stops a
// degenerate differential from smearing the whole card).
//
// Sources: the sun and the registry's directional lights (parallel beams
// through the transmitter's projected area), the registry's point and spot
// lights (targeted cones onto the transmitter's bounding sphere, so no photon
// is spent on empty space), and emissive card texels (a rotating stratum;
// aimed at the transmitters within the texel's emitting hemisphere). Photons
// that never cross a transparent interface deposit nothing - that light is
// plain direct lighting the direct pass already evaluates, and the caustic
// term must not double-count it.
//
// Temporal behavior: every frame re-samples the emission with fresh jitter,
// so the per-frame deposit sum is a fresh Monte Carlo estimate of the caustic
// irradiance field (a frame in which no photon lands on a texel is a
// legitimate zero draw); the estimate blends into the card's caustic state
// with the configurable temporal blend, and the whole field decays toward
// the current estimate each frame the pass runs - a caustic whose source
// moved away fades out instead of ghosting. A receiver that moves drops its
// deposits outright (updateSurfaceTransform clears the arrays: a moved
// card's texels describe new world positions). The static-frame skip freezes
// the converged state with everything else.

#include "radiance_cache_internal.hpp"

#include <cstddef>
#include <cstdio>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace engine
{
namespace radiance_cache
{

namespace
{

// One staged deposit: the flat texel (across every card) the photon landed
// on and the irradiance the photon contributed to this frame's estimate.
struct CausticDeposit
{
    std::uint64_t texel = 0;
    glm::vec3     irradiance{0.0f};
};

// Landing of one traced photon chain: the opaque receiver's surface and
// local shading pair plus the RGB flux share the chain carried to it.
struct PhotonLanding
{
    bool          hit = false;
    std::uint32_t surfaceIndex = noSurface;
    glm::vec3     localPosition{0.0f};
    glm::vec3     localNormal{0.0f, 1.0f, 0.0f};
};

// Uniform sample inside a cone of half-angle cosMax around `axis` (the
// targeted-emission sampler for cone sources).
glm::vec3 sampleCone(const glm::vec3 &axis, float cosMax, float u, float v)
{
    const float     cosTheta = 1.0f - u * (1.0f - cosMax);
    const float     sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
    const float     phi      = v * 2.0f * kPi;
    const glm::mat3 basis    = makeBasis(axis);
    return basis * glm::vec3(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

// Rodrigues rotation of `direction` around the unit axis `axis` by `angle`
// radians (the angular differential of cone sources).
glm::vec3 rotateAbout(const glm::vec3 &direction, const glm::vec3 &axis, float angle)
{
    const float cosine = std::cos(angle);
    const float sine   = std::sin(angle);
    return direction * cosine + glm::cross(axis, direction) * sine
        + axis * (glm::dot(axis, direction) * (1.0f - cosine));
}

// Perpendicular emission-plane pair of one direction (any stable pair works;
// the differential offsets need no particular orientation).
void emissionAxesOf(const glm::vec3 &direction, glm::vec3 &axisU, glm::vec3 &axisV)
{
    const glm::vec3 helper = std::fabs(direction.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                           : glm::vec3(1.0f, 0.0f, 0.0f);
    axisU = glm::normalize(glm::cross(helper, direction));
    axisV = glm::cross(direction, axisU);
}

// Traces one photon chain from `origin` along `direction` carrying `flux`
// (the source's transmitted share): transparent interfaces bend it (Snell,
// total internal reflection mirrors with the full remaining share), the
// albedo absorption applies once per medium traversal at the entering
// interface, and the first opaque hit is the landing. A chain that crosses
// no transparent interface returns a miss - that light is plain direct
// lighting the direct pass already evaluates, and the caustic term must not
// double-count it. Backface opaque hits are the anti-leak rule: no deposit.
PhotonLanding tracePhotonChain(const GiPipelineState &state, const glm::vec3 &origin, const glm::vec3 &direction,
    const glm::vec3 &flux, std::uint32_t maxSegments)
{
    PhotonLanding landing;
    glm::vec3     rayOrigin     = origin;
    glm::vec3     rayDirection  = direction;
    glm::vec3     weight        = flux;
    bool          crossedInterface = false;
    for (std::uint32_t segment = 0; segment <= maxSegments; ++segment)
    {
        const GiShadingHit hit = traceSceneShading(state, GiRay{rayOrigin, rayDirection, state.settings.maxTraceDistance});
        if (!hit.hit)
        {
            return landing; // escaped to the sky: no deposit
        }
        const GiSurface &surface = state.surfaces[hit.surfaceIndex];
        if (surface.opacity >= 1.0f)
        {
            if (!crossedInterface || hit.backface)
            {
                return landing;
            }
            landing.hit           = true;
            landing.surfaceIndex  = hit.surfaceIndex;
            landing.localPosition = hit.localPosition;
            landing.localNormal   = hit.localNormal;
            return landing;
        }
        // Transparent interface: keep the transmitted share (the Fresnel
        // reflection bounces off as specular light the reflection_field
        // module resolves; the absorbed share stays in the medium). The
        // interface works on the ray-facing normal, exactly like the gather
        // chains, and the albedo absorption applies once per traversal at
        // the entering interface (the exit passes untinted).
        const float opacity     = glm::clamp(surface.opacity, 0.0f, 1.0f);
        const bool  entering    = !hit.backface;
        const glm::vec3 worldNormal = surface.localIsWorld
            ? hit.localNormal
            : glm::normalize(surface.normalMatrix * hit.localNormal);
        const glm::vec3 interfaceNormal = entering ? worldNormal : -worldNormal;
        const float     ior = glm::max(surface.indexOfRefraction, 1.0f);
        const float     n1 = entering ? 1.0f : ior;
        const float     n2 = entering ? ior : 1.0f;
        const glm::vec3 bent = glm::refract(rayDirection, interfaceNormal, n1 / n2);
        if (glm::dot(bent, bent) < 1.0e-8f)
        {
            // Total internal reflection: the full remaining share mirrors.
            const glm::vec3 mirrored = glm::reflect(rayDirection, interfaceNormal);
            const glm::vec3 worldPosition = surface.localIsWorld
                ? hit.localPosition
                : surface.linear * hit.localPosition + surface.translation;
            rayOrigin     = worldPosition + mirrored * state.settings.traceBias;
            rayDirection  = glm::normalize(mirrored);
            crossedInterface = true;
            continue;
        }
        const float cosI = glm::clamp(glm::dot(-rayDirection, interfaceNormal), 0.0f, 1.0f);
        // Dielectric Fresnel of the crossing (Schlick on the exact
        // index-ratio base): its reflected share leaves the photon.
        const float ratio       = (n1 - n2) / (n1 + n2);
        const float fresnelBase = glm::clamp(ratio * ratio, 0.0f, 1.0f);
        const float complement  = 1.0f - cosI;
        const float complementSquared = complement * complement;
        const float fresnel = fresnelBase + (1.0f - fresnelBase)
            * complementSquared * complementSquared * complement;
        weight *= (1.0f - fresnel) * (1.0f - opacity);
        weight *= entering ? surface.albedo : glm::vec3(1.0f);
        if (glm::dot(weight, weight) <= 1.0e-12f)
        {
            return landing;
        }
        crossedInterface = true;
        const glm::vec3 worldPosition = surface.localIsWorld
            ? hit.localPosition
            : surface.linear * hit.localPosition + surface.translation;
        const glm::vec3 continuation = glm::normalize(bent);
        rayOrigin    = worldPosition + continuation * state.settings.traceBias;
        rayDirection = continuation;
    }
    return landing; // budget exhausted before an opaque receiver
}

// Deposits one photon's flux on the receiver's card texels: the cards whose
// outward axis matches the hit normal and whose layered depth agrees with the
// hit (the read-side localization rules, applied write-side), splatted over
// the texel centers inside the footprint radius with the normalized photon-
// mapping density kernel 3(1 - d/R) / (pi R^2) (integrating to one over the
// disc, so each photon's flux is conserved whatever the radius). The first
// card that accepts the hit takes the deposit; a footprint clipped at a card
// edge simply loses its outside share.
void depositPhoton(const GiPipelineState &state, const PhotonLanding &landing, float footprintRadius,
    const glm::vec3 &flux, std::vector<CausticDeposit> &out)
{
    const GiSurface &surface = state.surfaces[landing.surfaceIndex];
    // Localization runs in the receiver's local space (cards are local).
    const glm::vec3 local = surface.localIsWorld
        ? landing.localPosition
        : glm::vec3(surface.inverseTransform * glm::vec4(landing.localPosition, 1.0f));
    const glm::vec3 localNormal = surface.localIsWorld
        ? landing.localNormal
        : glm::normalize(glm::transpose(surface.linear) * landing.localNormal);
    for (std::uint32_t flatCard = 0; flatCard < state.cardRefs.size(); ++flatCard)
    {
        const GiCardRef &reference = state.cardRefs[flatCard];
        if (reference.surface != landing.surfaceIndex)
        {
            continue;
        }
        const GiCard &card = surface.cards[reference.card];
        const std::uint32_t axis = card.axisIndex;
        const float        wSign = card.axisW[axis];
        if (localNormal[axis] * wSign <= 0.15f)
        {
            continue;
        }
        const glm::vec3 relative = local - card.origin;
        const float u = relative[(axis + 1u) % 3u] / std::max(card.width, 1.0e-9f);
        const float v = relative[(axis + 2u) % 3u] / std::max(card.height, 1.0e-9f);
        if (u < -0.5f || u > 0.5f || v < -0.5f || v > 0.5f)
        {
            continue;
        }
        const float queryDepth = -relative[axis] * wSign;
        // Texel pitch along the card axes, in local units; the world pitch
        // divides by the instance scale of the matching axis.
        const float pitchU = card.width / std::max(static_cast<float>(card.texelWidth), 1.0f);
        const float pitchV = card.height / std::max(static_cast<float>(card.texelHeight), 1.0f);
        const float axisScaleU = std::max(surface.axisScale[(axis + 1u) % 3u], 1.0e-6f);
        const float axisScaleV = std::max(surface.axisScale[(axis + 2u) % 3u], 1.0e-6f);
        const float axisScaleW = std::max(surface.axisScale[axis], 1.0e-6f);
        // Normalized density kernel of the footprint radius.
        const float kernelScale = 3.0f / (kPi * footprintRadius * footprintRadius);
        const float texelX = (u + 0.5f) * static_cast<float>(card.texelWidth) - 0.5f;
        const float texelY = (v + 0.5f) * static_cast<float>(card.texelHeight) - 0.5f;
        const std::int32_t centerX = static_cast<std::int32_t>(std::lround(texelX));
        const std::int32_t centerY = static_cast<std::int32_t>(std::lround(texelY));
        // Footprint radius in texels per axis (world radius converted
        // through the pitch and the instance scale).
        const float radiusU = footprintRadius / (pitchU * axisScaleU);
        const float radiusV = footprintRadius / (pitchV * axisScaleV);
        const std::int32_t spanU = static_cast<std::int32_t>(std::ceil(radiusU));
        const std::int32_t spanV = static_cast<std::int32_t>(std::ceil(radiusV));
        const std::size_t plane = card.planeTexels();
        const std::uint64_t texelBase = state.cardTexelBase(flatCard);
        for (std::int32_t dy = -spanV; dy <= spanV; ++dy)
        {
            const std::int32_t y = centerY + dy;
            if (y < 0 || y >= static_cast<std::int32_t>(card.texelHeight))
            {
                continue;
            }
            for (std::int32_t dx = -spanU; dx <= spanU; ++dx)
            {
                const std::int32_t x = centerX + dx;
                if (x < 0 || x >= static_cast<std::int32_t>(card.texelWidth))
                {
                    continue;
                }
                const float du = (texelX - static_cast<float>(x)) * pitchU * axisScaleU;
                const float dv = (texelY - static_cast<float>(y)) * pitchV * axisScaleV;
                const float distanceSquared = du * du + dv * dv;
                if (distanceSquared > footprintRadius * footprintRadius)
                {
                    continue;
                }
                const float kernel = (1.0f - std::sqrt(distanceSquared) / footprintRadius) * kernelScale;
                if (kernel <= 0.0f)
                {
                    continue;
                }
                // Front-most layer whose stored depth agrees with the hit
                // (layers are depth ordered; the read-side tolerance rule).
                for (std::uint32_t layer = 0; layer < card.layers; ++layer)
                {
                    const std::size_t texel = layer * plane + static_cast<std::size_t>(y) * card.texelWidth
                        + static_cast<std::uint32_t>(x);
                    const float stored = card.depthTexels[texel];
                    if (stored >= invalidCardDepth)
                    {
                        break; // layers are depth ordered: the rest are empty
                    }
                    const float worldTolerance = 0.25f + 0.1f * stored * axisScaleW;
                    if (std::fabs(stored - queryDepth) * axisScaleW > worldTolerance)
                    {
                        continue;
                    }
                    out.push_back(CausticDeposit{texelBase + texel, flux * kernel});
                    break;
                }
            }
        }
        return; // one card accepted the deposit
    }
}

// One emission source of the frame: a parallel beam (the sun or a registry
// directional light) or a cone (a registry point or spot light aimed at a
// transmitter), each with its per-photon flux and the emission-plane axes the
// differential offsets use.
struct EmissionSource
{
    enum class Kind : std::uint8_t
    {
        Beam,
        Cone
    };
    Kind          kind = Kind::Beam;
    glm::vec3     origin{0.0f};       // Beam: emission-plane center; Cone: light position
    glm::vec3     direction{0.0f, 0.0f, 1.0f}; // Beam: beam direction; Cone: transmitter center
    glm::vec3     axisU{1.0f, 0.0f, 0.0f};     // Beam only: emission-plane axes
    glm::vec3     axisV{0.0f, 1.0f, 0.0f};
    float         halfExtent = 0.0f;          // Beam only: emission square half extent
    float         cosMax = 1.0f;              // Cone only: cone half-angle cosine
    glm::vec3     fluxPerPhoton{0.0f};
};

// Builds the beam frame of one transmitter for one parallel light: the plane
// through the transmitter's center perpendicular to the light direction,
// sized to the projected extents of its world bounds (padded to a square so
// the per-photon cell areas stay uniform).
EmissionSource beamSourceFor(const GiSurface &transmitter, const glm::vec3 &lightDirection)
{
    EmissionSource source;
    source.kind      = EmissionSource::Kind::Beam;
    source.direction = lightDirection;
    emissionAxesOf(lightDirection, source.axisU, source.axisV);
    const glm::vec3 center = 0.5f * (transmitter.worldMin + transmitter.worldMax);
    float halfExtent = 1.0e-3f;
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const glm::vec3 local((corner & 1u) ? transmitter.worldMax.x : transmitter.worldMin.x,
            (corner & 2u) ? transmitter.worldMax.y : transmitter.worldMin.y,
            (corner & 4u) ? transmitter.worldMax.z : transmitter.worldMin.z);
        const glm::vec3 relative = local - center;
        halfExtent = std::max(halfExtent, std::fabs(glm::dot(relative, source.axisU)));
        halfExtent = std::max(halfExtent, std::fabs(glm::dot(relative, source.axisV)));
    }
    source.halfExtent = halfExtent;
    // The emission plane sits the transmitter's diagonal outside, toward the
    // light, so every photon starts clear of the transmitter's bounds.
    const float offset = 0.5f * glm::length(transmitter.worldMax - transmitter.worldMin) + halfExtent;
    source.origin = center - lightDirection * offset;
    return source;
}

// Flux share one photon of a beam carries: the beam's irradiance times the
// emission square's area over the photon count (each photon is one cell of
// the beam's cross-section).
glm::vec3 beamFluxPerPhoton(const EmissionSource &source, std::uint32_t photons)
{
    const float area = 4.0f * source.halfExtent * source.halfExtent;
    return source.kind == EmissionSource::Kind::Beam && photons > 0
        ? source.fluxPerPhoton * (area / static_cast<float>(photons))
        : glm::vec3(0.0f);
}

// Flux share one photon of a cone carries: the light's radiant intensity into
// the cone's solid angle over the photon count.
glm::vec3 coneFluxPerPhoton(const EmissionSource &source, std::uint32_t photons)
{
    if (source.kind != EmissionSource::Kind::Cone || photons <= 0)
    {
        return glm::vec3(0.0f);
    }
    const float solidAngle = 2.0f * kPi * (1.0f - source.cosMax);
    return source.fluxPerPhoton * (solidAngle / static_cast<float>(photons));
}

// Whether the geometry focuses the light it transmits. Two shapes pass a
// beam through without converging it, so the gather's deterministic Snell
// path already resolves their transmitted share exactly and photons aimed
// at them add variance without focus: a single plane (all vertex normals
// align up to sign - double-sided planes fold onto one direction), and a
// thin slab (one local-space extent far below the other two: two large
// parallel faces carry the transmission while the rim stays minor).
// Curved surfaces - spheres - and boxes with comparable extents bend rays
// non-uniformly at their faces and edges and keep the caustic transmitter
// role. Both tests read local geometry (vertex normals, local bounds), so
// they are transform-invariant.
bool nonFocusingGeometry(const GiSurfaceGeometry &geometry)
{
    if (!geometry.normals.empty())
    {
        glm::vec3 foldedSum(0.0f);
        for (const glm::vec3 &normal : geometry.normals)
        {
            foldedSum += glm::abs(normal);
        }
        const float sumLength = glm::length(foldedSum);
        if (sumLength > 1.0e-9f)
        {
            const glm::vec3 dominantDirection = foldedSum / sumLength;
            // A 2.5-degree normal agreement budget: sphere belts and box
            // corner normals deviate far beyond it, while a flat pane's
            // smoothing noise stays inside it.
            constexpr float kPlanarNormalAlignment = 0.999f;
            bool aligned = true;
            for (const glm::vec3 &normal : geometry.normals)
            {
                if (glm::abs(glm::dot(normal, dominantDirection)) < kPlanarNormalAlignment)
                {
                    aligned = false;
                    break;
                }
            }
            if (aligned)
            {
                return true;
            }
        }
    }
    // Thin-slab test on the local-space extent: the smallest axis far below
    // the middle one marks a pane or water sheet whose two large parallel
    // faces dominate the transmission.
    const glm::vec3 extent = geometry.boundsMax - geometry.boundsMin;
    std::array<float, 3> sortedExtent = {extent.x, extent.y, extent.z};
    std::sort(sortedExtent.begin(), sortedExtent.end());
    constexpr float kSlabThinness = 0.2f;
    return sortedExtent[0] < kSlabThinness * sortedExtent[1];
}

} // namespace

void rebuildCausticsTransmitters(GiPipelineState &state)
{
    state.scratchCausticTransmitters.clear();
    for (std::uint32_t index = 0; index < state.surfaces.size(); ++index)
    {
        GiSurface &surface = state.surfaces[index];
        // Transmitter = a transmissive surface that focuses the light it
        // passes (see nonFocusingGeometry): photons aim at it and the photon
        // pass owns its transmitted share. Planes and thin slabs stay on the
        // gather's deterministic refraction path instead.
        surface.causticTransmitter = !state.settings.causticsEnabled ? false
            : (!surface.excluded && surface.opacity < 1.0f && surface.opacity > 0.0f
                && !surface.geometry.bvh.empty() && !nonFocusingGeometry(surface.geometry));
        if (surface.causticTransmitter)
        {
            state.scratchCausticTransmitters.push_back(index);
        }
    }
    state.causticTransmittersDirty = false;
}

void runCausticsPass(GiPipelineState &state, TaskScheduler *scheduler)
{
    if (!state.settings.causticsEnabled)
    {
        return;
    }
    // Transmitters: the transmissive surfaces the sources aim at (and the
    // surfaces whose transmitted light this pass owns - see
    // GiSurface::causticTransmitter). The list rebuild happens at frame-plan
    // time; here it only reads it.
    const std::vector<std::uint32_t> &transmitters = state.scratchCausticTransmitters;
    if (transmitters.empty() && state.causticsDecayFrames == 0)
    {
        return;
    }

    // ---- plan the frame's emission (single threaded) -------------------------
    const GiSettings &settings = state.settings;
    const std::uint32_t photonBudget = std::max(settings.causticsPhotonsPerFrame, 1u);
    const glm::vec3 sunRadiance = state.environment.sunRadiance();
    const bool sunSource = glm::dot(sunRadiance, sunRadiance) > 1.0e-9f && !transmitters.empty();

    // Emission sources: the environment sun, every registry directional
    // light, and the registry's point and spot lights (one cone each, aimed
    // at the transmitter nearest the light). Emissive texels are handled as
    // their own source class below (they need the captured card data).
    std::vector<EmissionSource> sources;
    std::vector<std::uint32_t> sourcePhotons;
    {
        std::uint32_t beamCount = 0;
        if (sunSource)
        {
            ++beamCount;
        }
        for (const GiEnvironmentSnapshot::PreparedDirectLight &light : state.environment.preparedLights)
        {
            if (light.type == direct_lighting::DirectLightType::Directional)
            {
                ++beamCount;
            }
        }
        // Cone sources: point and spot lights with a transmitter in range.
        std::vector<std::uint32_t> coneLights;
        for (std::uint32_t lightIndex = 0; lightIndex < state.environment.preparedLights.size(); ++lightIndex)
        {
            const GiEnvironmentSnapshot::PreparedDirectLight &light = state.environment.preparedLights[lightIndex];
            if (light.type == direct_lighting::DirectLightType::Directional)
            {
                continue;
            }
            for (std::uint32_t transmitter = 0; transmitter < transmitters.size(); ++transmitter)
            {
                const GiSurface &target = state.surfaces[transmitters[transmitter]];
                const glm::vec3 center = 0.5f * (target.worldMin + target.worldMax);
                const float distance = glm::length(center - light.position);
                if (distance <= light.range + 0.5f * glm::length(target.worldMax - target.worldMin))
                {
                    coneLights.push_back(lightIndex);
                    break;
                }
            }
        }
        // Budget: beams (the dominant caustic driver outdoors) take half the
        // photons when present; the rest splits between the cones and the
        // emissive stratum.
        const bool emissiveOn = settings.causticsEmissiveSources;
        std::uint32_t beamPhotons = 0;
        std::uint32_t conePhotons = 0;
        if (beamCount > 0)
        {
            beamPhotons = photonBudget / 2;
        }
        std::uint32_t remainder = photonBudget - beamPhotons;
        if (!coneLights.empty() && emissiveOn)
        {
            conePhotons = remainder / 2;
        }
        else if (!coneLights.empty())
        {
            conePhotons = remainder;
        }
        // Per-beam budget split across the beam count (the sun included).
        const std::uint32_t perBeam = beamCount > 0 ? beamPhotons / beamCount : 0;
        // Build the beams (rotating the transmitter assignment per frame so
        // multi-transmitter scenes share the beam budget fairly).
        if (sunSource)
        {
            const std::uint32_t transmitter = static_cast<std::uint32_t>(state.frameIndex) % transmitters.size();
            EmissionSource source = beamSourceFor(state.surfaces[transmitters[transmitter]],
                state.environment.sunDirection);
            source.fluxPerPhoton = sunRadiance;
            sources.push_back(source);
            sourcePhotons.push_back(perBeam);
        }
        for (const GiEnvironmentSnapshot::PreparedDirectLight &light : state.environment.preparedLights)
        {
            if (light.type != direct_lighting::DirectLightType::Directional || perBeam == 0)
            {
                continue;
            }
            const std::uint32_t transmitter = static_cast<std::uint32_t>(state.frameIndex) % transmitters.size();
            EmissionSource source = beamSourceFor(state.surfaces[transmitters[transmitter]],
                -light.direction);
            source.fluxPerPhoton = light.color * light.intensity;
            sources.push_back(source);
            sourcePhotons.push_back(perBeam);
        }
        const std::uint32_t perCone = coneLights.empty() ? 0u : conePhotons / static_cast<std::uint32_t>(coneLights.size());
        for (std::uint32_t index = 0; index < coneLights.size(); ++index)
        {
            const GiEnvironmentSnapshot::PreparedDirectLight &light = state.environment.preparedLights[coneLights[index]];
            // Aim at the transmitter nearest the light (the range check
            // above found the first in range; that is the one).
            std::uint32_t bestTransmitter = 0;
            float bestDistance = std::numeric_limits<float>::max();
            for (std::uint32_t transmitter = 0; transmitter < transmitters.size(); ++transmitter)
            {
                const GiSurface &target = state.surfaces[transmitters[transmitter]];
                const glm::vec3 center = 0.5f * (target.worldMax + target.worldMin);
                const float distance = glm::length(center - light.position);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestTransmitter = transmitter;
                }
            }
            const GiSurface &target = state.surfaces[transmitters[bestTransmitter]];
            const glm::vec3 center = 0.5f * (target.worldMax + target.worldMin);
            const float radius = std::max(0.5f * glm::length(target.worldMax - target.worldMin), 1.0e-3f);
            const float centerDistance = std::max(glm::length(center - light.position), radius);
            EmissionSource source;
            source.kind = EmissionSource::Kind::Cone;
            source.origin = light.position;
            source.direction = center; // the cone axis target
            // The cone that circumscribes the transmitter: half-angle
            // atan(radius / centerDistance), so cosMax = centerDistance /
            // hypot. Every sampled direction can reach the transmitter (the
            // tight cone the "no photon spent on empty space" aim requires)
            // while the silhouette still samples fully.
            source.cosMax = centerDistance / std::sqrt(centerDistance * centerDistance + radius * radius);
            // Spot lights clip their cone to the spot's outer cone: a
            // transmitter outside the spot contributes nothing.
            if (light.type == direct_lighting::DirectLightType::Spot)
            {
                const glm::vec3 axis = light.direction;
                const glm::vec3 toTransmitter = glm::normalize(center - light.position);
                const float axisAngle = glm::dot(axis, toTransmitter);
                const float transmitterAngle = radius / std::max(glm::length(center - light.position), 1.0e-3f);
                // Approximate the cone intersection: require the transmitter
                // cone to overlap the spot cone at all, then clamp the
                // sampled cone to the tighter of the two.
                if (axisAngle < light.outerCosine - transmitterAngle)
                {
                    continue; // transmitter outside the spot: no caustics
                }
                source.cosMax = std::max(source.cosMax, std::min(light.outerCosine, 1.0f));
            }
            source.fluxPerPhoton = light.color * light.intensity;
            sources.push_back(source);
            sourcePhotons.push_back(perCone);
        }
    }

    // ---- run the photon batches over the scheduler ---------------------------
    // Worker-local staging: photons of one chunk append into the worker's
    // list, and the list is published into the pass's chunk vectors under a
    // tiny mutex (one lock per chunk; the sequential commit below runs after
    // every worker finished).
    std::mutex publishMutex;
    std::vector<std::vector<CausticDeposit>> chunkDeposits;
    std::atomic<std::uint32_t> depositsThisFrame{0};

    const std::uint32_t maxSegments = settings.maxTransmissionSegments;
    const float footprintMin = std::max(settings.causticsFootprintMinM, 1.0e-3f);
    const float footprintMax = std::max(settings.causticsFootprintMaxM, footprintMin + 1.0e-3f);

    parallelRange(scheduler, sources.size(), 1,
        [&](std::size_t begin, std::size_t end)
        {
            for (std::size_t sourceIndex = begin; sourceIndex < end; ++sourceIndex)
            {
                const EmissionSource &source = sources[sourceIndex];
                const std::uint32_t photons = sourcePhotons[sourceIndex];
                glm::vec3 fluxPerPhoton(0.0f);
                // The emission cell one photon covers: its side drives the
                // differential offset (the auxiliary rays trace the cell's
                // corners, so their landing parallelogram IS the cell's image
                // under the transport) and the flux normalization.
                float cellArea = 0.0f;
                if (source.kind == EmissionSource::Kind::Beam)
                {
                    fluxPerPhoton = beamFluxPerPhoton(source, photons);
                    cellArea     = 4.0f * source.halfExtent * source.halfExtent / std::max(photons, 1u);
                }
                else
                {
                    fluxPerPhoton = coneFluxPerPhoton(source, photons);
                    cellArea     = 2.0f * kPi * (1.0f - source.cosMax) / std::max(photons, 1u); // solid-angle cell
                }
                if (photons == 0 || glm::dot(fluxPerPhoton, fluxPerPhoton) <= 1.0e-12f)
                {
                    continue;
                }
                const float differential = std::max(std::sqrt(cellArea), 1.0e-4f);
                thread_local std::vector<CausticDeposit> staging;
                staging.clear();
                for (std::uint32_t photon = 0; photon < photons; ++photon)
                {
                    std::uint64_t seed = hashSeed(state.frameIndex * 0x9E3779B97F4A7C15ull
                        + (static_cast<std::uint64_t>(sourceIndex) << 32) + photon);
                    glm::vec3 origin(0.0f);
                    glm::vec3 direction(0.0f, 0.0f, 1.0f);
                    glm::vec3 axisU(1.0f, 0.0f, 0.0f);
                    glm::vec3 axisV(0.0f, 1.0f, 0.0f);
                    if (source.kind == EmissionSource::Kind::Beam)
                    {
                        const float u = randomFloat(seed);
                        const float v = randomFloat(seed);
                        origin    = source.origin + source.axisU * ((u - 0.5f) * 2.0f * source.halfExtent)
                            + source.axisV * ((v - 0.5f) * 2.0f * source.halfExtent);
                        direction = source.direction;
                        axisU     = source.axisU;
                        axisV     = source.axisV;
                    }
                    else
                    {
                        direction = sampleCone(glm::normalize(source.direction - source.origin), source.cosMax,
                            randomFloat(seed), randomFloat(seed));
                        origin = source.origin + direction * state.settings.traceBias;
                        emissionAxesOf(direction, axisU, axisV);
                    }
                    // The center chain and its two differentials: the
                    // auxiliary rays refract independently, so their landing
                    // parallelogram is the emission cell's image under the
                    // transport. Beams and emissive texels offset the
                    // emission POSITION along their surface axes; cone
                    // sources have a point origin and offset the emission
                    // DIRECTION (the angular cell) instead.
                    const PhotonLanding center = tracePhotonChain(state, origin, direction, fluxPerPhoton, maxSegments);
                    if (!center.hit)
                    {
                        continue;
                    }
                    PhotonLanding landingU;
                    PhotonLanding landingV;
                    if (source.kind == EmissionSource::Kind::Cone)
                    {
                        landingU = tracePhotonChain(state, origin,
                            rotateAbout(direction, axisU, differential), fluxPerPhoton, maxSegments);
                        landingV = tracePhotonChain(state, origin,
                            rotateAbout(direction, axisV, differential), fluxPerPhoton, maxSegments);
                    }
                    else
                    {
                        landingU = tracePhotonChain(
                            state, origin + axisU * differential, direction, fluxPerPhoton, maxSegments);
                        landingV = tracePhotonChain(
                            state, origin + axisV * differential, direction, fluxPerPhoton, maxSegments);
                    }
                    // Footprint: the parallelogram the differential landings
                    // span - the emission cell's image, so the focus factor
                    // falls out of its area. Divergent auxiliary chains
                    // (edges, total-internal-reflection disagreement) fall
                    // back to the ceiling radius: a conservative blur, never
                    // an energy spike.
                    float footprintArea = std::numeric_limits<float>::max();
                    if (landingU.hit && landingV.hit && landingU.surfaceIndex == center.surfaceIndex
                        && landingV.surfaceIndex == center.surfaceIndex)
                    {
                        const glm::vec3 deltaU = landingU.localPosition - center.localPosition;
                        const glm::vec3 deltaV = landingV.localPosition - center.localPosition;
                        const float     area = glm::length(glm::cross(deltaU, deltaV));
                        if (std::isfinite(area) && area > 1.0e-12f)
                        {
                            footprintArea = area;
                        }
                    }
                    const float footprintRadius = glm::clamp(
                        std::sqrt(footprintArea * (1.0f / kPi)), footprintMin, footprintMax);
                    depositPhoton(state, center, footprintRadius, fluxPerPhoton, staging);
                }
                if (!staging.empty())
                {
                    depositsThisFrame.fetch_add(static_cast<std::uint32_t>(staging.size()),
                        std::memory_order_relaxed);
                    std::lock_guard<std::mutex> lock(publishMutex);
                    chunkDeposits.emplace_back();
                    chunkDeposits.back().assign(staging.begin(), staging.end());
                }
            }
        });

    // ---- emissive-texel photons (a rotating stratum over the emitters) -------
    // Emission lives on the emissive surfaces' card texels: each photon of
    // the stratum leaves a sampled texel toward a transmitter's bounds
    // (within the texel's emitting hemisphere), carrying the texel's
    // Lambertian emission into the transmitter's cone, scaled by the share
    // of the emitter area the frame's stratum covers (an unbiased estimate
    // of the full emission).
    if (settings.causticsEmissiveSources && !transmitters.empty())
    {
        // Emissive cards (surfaces with emission and captured cards).
        if (state.scratchCausticEmissiveCards.empty() || state.causticEmissiveCardsDirty)
        {
            state.scratchCausticEmissiveCards.clear();
            for (std::uint32_t flatCard = 0; flatCard < state.cardRefs.size(); ++flatCard)
            {
                const GiSurface &surface = state.cardSurface(flatCard);
                if (!surface.excluded && glm::dot(surface.emissive, surface.emissive) > 1.0e-12f)
                {
                    state.scratchCausticEmissiveCards.push_back(flatCard);
                }
            }
            state.causticEmissiveCardsDirty = false;
        }
        // The emissive share of the budget is what the beam/cone split left.
        std::uint32_t spentPhotons = 0;
        for (std::size_t index = 0; index < sources.size(); ++index)
        {
            spentPhotons += sourcePhotons[index];
        }
        const std::uint32_t emissivePhotons = photonBudget - std::min(spentPhotons, photonBudget);
        if (emissivePhotons > 0 && !state.scratchCausticEmissiveCards.empty())
        {
            // Total emissive texel count across the emissive cards.
            std::uint64_t emissiveTexels = 0;
            for (const std::uint32_t flatCard : state.scratchCausticEmissiveCards)
            {
                emissiveTexels += state.card(flatCard).planeTexels();
            }
            if (emissiveTexels > 0)
            {
                parallelRange(scheduler, emissivePhotons, 64,
                    [&](std::size_t begin, std::size_t end)
                    {
                        thread_local std::vector<CausticDeposit> staging;
                        staging.clear();
                        for (std::size_t photon = begin; photon < end; ++photon)
                        {
                            // Rotating stratum: photon ordinal + the frame's
                            // stride covers the emitter area over frames.
                            const std::uint64_t ordinal = state.frameIndex * emissivePhotons + photon;
                            std::uint64_t texelCursor = ordinal % emissiveTexels;
                            const GiCard *card = nullptr;
                            const GiSurface *emitter = nullptr;
                            std::uint32_t texelX = 0;
                            std::uint32_t texelY = 0;
                            for (const std::uint32_t candidate : state.scratchCausticEmissiveCards)
                            {
                                const GiCard &candidateCard = state.card(candidate);
                                const std::uint64_t plane = candidateCard.planeTexels();
                                if (texelCursor < plane)
                                {
                                    card = &candidateCard;
                                    emitter = &state.cardSurface(candidate);
                                    texelX = static_cast<std::uint32_t>(texelCursor % candidateCard.texelWidth);
                                    texelY = static_cast<std::uint32_t>(texelCursor / candidateCard.texelWidth);
                                    break;
                                }
                                texelCursor -= plane;
                            }
                            if (card == nullptr || emitter == nullptr)
                            {
                                continue;
                            }
                            std::uint64_t seed = hashSeed(0xD1B54A32D192ED03ull + ordinal * 0x9E3779B1ull);
                            // Only the card's front-most layer emits (a
                            // captured back layer belongs to geometry the
                            // card also sees through).
                            const std::size_t texel = static_cast<std::size_t>(texelY) * card->texelWidth + texelX;
                            if (card->depthTexels[texel] >= invalidCardDepth)
                            {
                                continue; // not captured (or empty) texel
                            }
                            // The transmitter this photon aims at (rotating).
                            const std::uint32_t transmitterIndex
                                = static_cast<std::uint32_t>(ordinal % transmitters.size());
                            const GiSurface &target = state.surfaces[transmitters[transmitterIndex]];
                            const glm::vec3 targetCenter = 0.5f * (target.worldMax + target.worldMin);
                            const float targetRadius = 0.5f * glm::length(target.worldMax - target.worldMin);
                            // Emitter texel in world space.
                            const glm::vec3 localPosition = cardTexelLocalPositionXY(*card, texelX, texelY, texel);
                            const glm::vec3 position = emitter->localIsWorld
                                ? localPosition
                                : emitter->linear * localPosition + emitter->translation;
                            const glm::vec3 normal = emitter->localIsWorld
                                ? card->normal[texel]
                                : glm::normalize(emitter->normalMatrix * card->normal[texel]);
                            const float distance = std::max(glm::length(targetCenter - position), 1.0e-3f);
                            // Circumscribing cone (half-angle atan(r/d), so
                            // cosMax = distance / hypot): the emission the
                            // stratum carries into this solid angle reaches
                            // the transmitter instead of spending the budget
                            // on the empty space around it.
                            const float cosMax = distance / std::sqrt(distance * distance + targetRadius * targetRadius);
                            const glm::vec3 targetAxis = glm::normalize(targetCenter - position);
                            const glm::vec3 direction = sampleCone(targetAxis, cosMax, randomFloat(seed), randomFloat(seed));
                            const float cosine = glm::dot(direction, normal);
                            if (cosine <= 0.0f)
                            {
                                continue; // below the emitter's horizon: no emission
                            }
                            // Texel world area (Lambertian emission scaling).
                            const std::uint32_t cardAxis = card->axisIndex;
                            const float pitchU = card->width / std::max(static_cast<float>(card->texelWidth), 1.0f);
                            const float pitchV = card->height / std::max(static_cast<float>(card->texelHeight), 1.0f);
                            const float scaleU = std::max(emitter->axisScale[(cardAxis + 1u) % 3u], 1.0e-6f);
                            const float scaleV = std::max(emitter->axisScale[(cardAxis + 2u) % 3u], 1.0e-6f);
                            const float texelArea = pitchU * scaleU * pitchV * scaleV;
                            const float solidAngle = 2.0f * kPi * (1.0f - cosMax);
                            // The photon represents emissiveTexels /
                            // emissivePhotons texels of the emitter area (the
                            // stratum's share), so its flux is one texel's
                            // emission scaled by that share; the emission cell
                            // it covers (texel area x share) drives the
                            // differential offset below.
                            const float share = static_cast<float>(emissiveTexels) / static_cast<float>(emissivePhotons);
                            const glm::vec3 flux = emitter->emissive * (texelArea * cosine * solidAngle) * share;
                            const float cellArea = texelArea * share;
                            const float differential = std::max(std::sqrt(cellArea), 1.0e-4f);
                            const PhotonLanding center
                                = tracePhotonChain(state, position + normal * state.settings.traceBias, direction,
                                    flux, maxSegments);
                            if (!center.hit)
                            {
                                continue;
                            }
                            // The differential offsets live on the emitter
                            // texel's card axes (the emission plane), in world
                            // space through the emitter's transform.
                            const glm::vec3 axisU = emitter->localIsWorld
                                ? card->axisU
                                : emitter->linear * card->axisU;
                            const glm::vec3 axisV = emitter->localIsWorld
                                ? card->axisV
                                : emitter->linear * card->axisV;
                            const PhotonLanding landingU = tracePhotonChain(
                                state, position + axisU * differential, direction, flux, maxSegments);
                            const PhotonLanding landingV = tracePhotonChain(
                                state, position + axisV * differential, direction, flux, maxSegments);
                            float footprintArea = std::numeric_limits<float>::max();
                            if (landingU.hit && landingV.hit && landingU.surfaceIndex == center.surfaceIndex
                                && landingV.surfaceIndex == center.surfaceIndex)
                            {
                                const glm::vec3 deltaU = landingU.localPosition - center.localPosition;
                                const glm::vec3 deltaV = landingV.localPosition - center.localPosition;
                                const float     area = glm::length(glm::cross(deltaU, deltaV));
                                if (std::isfinite(area) && area > 1.0e-12f)
                                {
                                    footprintArea = area;
                                }
                            }
                            const float footprintRadius = glm::clamp(
                                std::sqrt(footprintArea * (1.0f / kPi)), footprintMin, footprintMax);
                            depositPhoton(state, center, footprintRadius, flux, staging);
                        }
                        if (!staging.empty())
                        {
                            depositsThisFrame.fetch_add(static_cast<std::uint32_t>(staging.size()),
                                std::memory_order_relaxed);
                            std::lock_guard<std::mutex> lock(publishMutex);
                            chunkDeposits.emplace_back();
                            chunkDeposits.back().assign(staging.begin(), staging.end());
                        }
                    });
            }
        }
    }

    // ---- commit: blend the frame estimate into the caustic state ------------
    // The frame estimate of a touched texel is the staged deposit sum (an
    // unbiased draw of that texel's caustic irradiance; a frame with no
    // deposit is a zero draw). The blend carries the temporal history with
    // a first-deposit ramp; the decay pass then pulls the whole field toward
    // the current estimate (zero where nothing landed) so a caustic whose
    // source moved away fades instead of ghosting.
    // The staging buffer is zero outside the commit section (every touched
    // entry is cleared again below), and rebuilt card sets re-zero it.
    if (state.scratchCausticFrame.size() != state.totalCardTexels)
    {
        state.scratchCausticFrame.assign(state.totalCardTexels, glm::vec3(0.0f));
    }
    const std::uint32_t deposits = depositsThisFrame.load(std::memory_order_relaxed);
    const float history = std::clamp(settings.causticsTemporalBlend, 0.0f, 0.99f);
    if (deposits > 0)
    {
        state.causticsDecayFrames = 256; // keep the decay pass alive
    }
    if (state.causticsDecayFrames == 0)
    {
        return; // converged to nothing and no new deposits: fully idle
    }
    --state.causticsDecayFrames;

    // Stage the deposits per flat texel (the chunk lists overlap).
    state.scratchCausticTouched.clear();
    for (const std::vector<CausticDeposit> &chunk : chunkDeposits)
    {
        for (const CausticDeposit &deposit : chunk)
        {
            if (deposit.texel < state.totalCardTexels)
            {
                state.scratchCausticFrame[deposit.texel] += deposit.irradiance;
                state.scratchCausticTouched.push_back(deposit.texel);
            }
        }
    }
    std::sort(state.scratchCausticTouched.begin(), state.scratchCausticTouched.end());
    state.scratchCausticTouched.erase(
        std::unique(state.scratchCausticTouched.begin(), state.scratchCausticTouched.end()),
        state.scratchCausticTouched.end());

    // Blend the touched texels, then decay + re-derive the cached radiance of
    // every texel in one pass over the cards (the arrays are small compared
    // with the photon budget; the pass also clears the staging). The touched
    // list is sorted, and the flat texel walk is sorted too, so a single
    // cursor replaces the per-texel search.
    const float inversePi = 1.0f / kPi;
    std::size_t touchedCursor = 0;
    // Receiver flags recomputed by the walk below: a surface whose cards
    // hold any caustic state this frame is a receiver the gather samples.
    for (GiSurface &surface : state.surfaces)
    {
        surface.causticReceiver = false;
    }
    for (std::uint32_t flatCard = 0; flatCard < state.cardRefs.size(); ++flatCard)
    {
        const GiCardRef reference = state.cardRefs[flatCard];
        GiCard &card = state.surfaces[reference.surface].cards[reference.card];
        const std::uint64_t texelBase = state.cardTexelBase(flatCard);
        const std::size_t texelCount = card.totalTexels();
        for (std::size_t texel = 0; texel < texelCount; ++texel)
        {
            const std::uint64_t flatTexel = texelBase + texel;
            while (touchedCursor < state.scratchCausticTouched.size()
                && state.scratchCausticTouched[touchedCursor] < flatTexel)
            {
                ++touchedCursor;
            }
            glm::vec3 estimate = card.causticIrradiance[texel];
            const bool touched = touchedCursor < state.scratchCausticTouched.size()
                && state.scratchCausticTouched[touchedCursor] == flatTexel;
            if (touched)
            {
                const std::uint32_t seen = std::min<std::uint32_t>(card.causticSamples[texel] + 1u, 255u);
                card.causticSamples[texel] = static_cast<std::uint8_t>(seen);
                estimate = glm::mix(state.scratchCausticFrame[flatTexel], estimate,
                    std::max(1.0f / static_cast<float>(seen), history));
            }
            else
            {
                estimate *= history;
            }
            card.causticIrradiance[texel] = estimate;
            card.causticRadiance[texel] = card.albedo[texel] * (inversePi * estimate);
            if (glm::dot(card.causticRadiance[texel], card.causticRadiance[texel]) > 1.0e-12f)
            {
                state.surfaces[reference.surface].causticReceiver = true;
            }
        }
    }
    // Clear the staging (the touched list carries the entries; the frame
    // buffer is zeroed by the next frame's writes only where it deposited).
    for (const std::uint64_t flatTexel : state.scratchCausticTouched)
    {
        state.scratchCausticFrame[flatTexel] = glm::vec3(0.0f);
    }
}

bool sampleSurfaceCausticsIrradiance(
    const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position, const glm::vec3 &normal,
    glm::vec3 &outIrradiance)
{
    outIrradiance = glm::vec3(0.0f);
    if (surfaceIndex >= state.surfaces.size())
    {
        return false;
    }
    const GiSurface &surface = state.surfaces[surfaceIndex];
    if (surface.cards.empty())
    {
        return false;
    }
    const glm::vec3 local = glm::vec3(surface.inverseTransform * glm::vec4(position, 1.0f));
    const glm::vec3 localNormal = glm::normalize(glm::transpose(surface.linear) * normal);
    float weightSum = 0.0f;
    glm::vec3 sum(0.0f);
    for (const GiCard &card : surface.cards)
    {
        const std::uint32_t axis = card.axisIndex;
        const float        wSign = card.axisW[axis];
        const float alignment = localNormal[axis] * wSign;
        if (alignment <= 0.15f)
        {
            continue;
        }
        const glm::vec3 relative = local - card.origin;
        const float u = relative[(axis + 1u) % 3u] / std::max(card.width, 1.0e-9f);
        const float v = relative[(axis + 2u) % 3u] / std::max(card.height, 1.0e-9f);
        constexpr float margin = 0.5f + 1.0e-3f;
        if (u < -margin || u > margin || v < -margin || v > margin)
        {
            continue;
        }
        const float queryDepth = -relative[axis] * wSign;
        if (queryDepth < -2.0f * card.nearBias || queryDepth > card.depth)
        {
            continue;
        }
        const float worldScale = std::max(surface.axisScale[card.axisIndex], 1.0e-6f);
        const float texelX = (u + 0.5f) * static_cast<float>(card.texelWidth) - 0.5f;
        const float texelY = (v + 0.5f) * static_cast<float>(card.texelHeight) - 0.5f;
        const float fx0 = std::floor(texelX);
        const float fy0 = std::floor(texelY);
        const float fx  = texelX - fx0;
        const float fy  = texelY - fy0;
        const std::int32_t x0 = static_cast<std::int32_t>(fx0);
        const std::int32_t y0 = static_cast<std::int32_t>(fy0);
        const std::size_t plane = card.planeTexels();
        for (std::uint32_t dy = 0; dy < 2; ++dy)
        {
            for (std::uint32_t dx = 0; dx < 2; ++dx)
            {
                const float bilinear = (dx != 0 ? fx : 1.0f - fx) * (dy != 0 ? fy : 1.0f - fy);
                if (bilinear <= 0.0f)
                {
                    continue;
                }
                const std::int32_t x = std::clamp(x0 + static_cast<std::int32_t>(dx), 0,
                    static_cast<std::int32_t>(card.texelWidth) - 1);
                const std::int32_t y = std::clamp(y0 + static_cast<std::int32_t>(dy), 0,
                    static_cast<std::int32_t>(card.texelHeight) - 1);
                const std::size_t xy = static_cast<std::size_t>(y) * card.texelWidth + x;
                for (std::uint32_t layer = 0; layer < card.layers; ++layer)
                {
                    const std::size_t texel = layer * plane + xy;
                    const float stored = card.depthTexels[texel];
                    if (stored >= invalidCardDepth)
                    {
                        break;
                    }
                    const float worldTolerance = 0.25f + 0.1f * stored * worldScale;
                    if (std::fabs(stored - queryDepth) * worldScale > worldTolerance)
                    {
                        continue;
                    }
                    const float weight = alignment * bilinear;
                    sum += card.causticIrradiance[texel] * weight;
                    weightSum += weight;
                }
            }
        }
    }
    if (weightSum <= 1.0e-6f)
    {
        return false;
    }
    outIrradiance = sum / weightSum;
    return true;
}

} // namespace radiance_cache
} // namespace engine
