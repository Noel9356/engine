// Terrain density field implementation (see density.hpp).

#include <engine/planet/density.hpp>

#include <engine/planet/noise.hpp>

#include <algorithm>
#include <cmath>

namespace engine
{
namespace planet
{
namespace
{

double clamp01(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

// Per-octave fade of the band-limited field: a term of the given wavelength
// contributes fully once its wavelength reaches twice the cutoff, and nothing
// at or below the cutoff. cutoff <= 0 disables the fade (the raw field).
double lowPassFade(double cutoffMeters, double termWavelengthMeters)
{
    if (cutoffMeters <= 0.0)
    {
        return 1.0;
    }
    return smoothstep(cutoffMeters, 2.0 * cutoffMeters, termWavelengthMeters);
}

// fBm over planet-local positions with a metric wavelength; returns [-1, 1].
double fbm(std::int64_t seed, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters)
{
    const double frequency = 1.0 / std::max(1.0, wavelengthMeters);
    double       x         = point.x * frequency;
    double       y         = point.y * frequency;
    double       z         = point.z * frequency;
    double       amplitude = 1.0;
    double       sum       = 0.0;
    double       norm      = 0.0;
    double       octaveWavelength = wavelengthMeters;
    for (std::uint32_t octave = 0; octave < fbmParams.octaves; ++octave)
    {
        const double fade = lowPassFade(lowPassCutoffMeters, octaveWavelength);
        if (fade <= 0.0)
        {
            break; // octaves shorten monotonically: every later fade is also zero
        }
        sum += amplitude * noise::simplex3ImproveXY(hashCombine64(seed, octave), x, y, z);
        norm += amplitude;
        x *= fbmParams.lacunarity;
        y *= fbmParams.lacunarity;
        z *= fbmParams.lacunarity;
        amplitude *= fbmParams.gain;
        octaveWavelength /= fbmParams.lacunarity;
    }
    return sum / std::max(norm, 1.0e-9);
}

// Ridged fBm in [0, 1]: mountain crests from |noise| inverted per octave.
double ridgedFbm(std::int64_t seed, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters)
{
    const double frequency = 1.0 / std::max(1.0, wavelengthMeters);
    double       x         = point.x * frequency;
    double       y         = point.y * frequency;
    double       z         = point.z * frequency;
    double       amplitude = 1.0;
    double       sum       = 0.0;
    double       norm      = 0.0;
    double       octaveWavelength = wavelengthMeters;
    for (std::uint32_t octave = 0; octave < fbmParams.octaves; ++octave)
    {
        const double fade = lowPassFade(lowPassCutoffMeters, octaveWavelength);
        if (fade <= 0.0)
        {
            break; // octaves shorten monotonically: every later fade is also zero
        }
        const double n = noise::simplex3ImproveXY(hashCombine64(seed, octave), x, y, z);
        sum += amplitude * (1.0 - std::abs(n));
        norm += amplitude;
        x *= fbmParams.lacunarity;
        y *= fbmParams.lacunarity;
        z *= fbmParams.lacunarity;
        amplitude *= fbmParams.gain;
        octaveWavelength /= fbmParams.lacunarity;
    }
    return clamp01(sum / std::max(norm, 1.0e-9));
}

} // namespace

DensityField::DensityField(const PlanetGenerationParams &params)
    : params_(params)
    , climate_(params)
    , seedContinents_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xC0718E51ULL)))
    , seedWarp_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x3A27F0ULL)))
    , seedMountains_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x40E1D5ULL)))
    , seedHills_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x4B1115ULL)))
    , seedDetail_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xD37A11ULL)))
    , seedCliffMask_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xC11F41ULL)))
    , seedOverhang_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x09E23A7ULL)))
    , seedStrata_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x572A7AULL)))
    , seedCaveA_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xCAE1AULL)))
    , seedCaveB_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xCAE2BULL)))
    , seedMountainMass_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x4A55ULL)))
    , seedRangeCluster_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x51A7E5ULL)))
    , seedRegional_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x6E10A7ULL)))
{
    const double reliefRange = params.continentAmplitudeMeters + params.mountainAmplitudeMeters + params.hillAmplitudeMeters;
    terraceStepMeters_ = std::max(1.0, reliefRange / std::max(1.0, params.terraceSteps));
}

ReliefSample DensityField::reliefAt(const Vec3 &unitDirection) const
{
    return reliefImpl(unitDirection, 0.0);
}

ReliefSample DensityField::reliefImpl(const Vec3 &unitDirection, double lowPassCutoffMeters) const
{
    ReliefSample relief;

    // All continent-scale fields sample the noise at PHYSICAL positions
    // (meters) divided by their wavelength, so feature sizes stay metric at
    // any planet radius: a 9 km continent wavelength produces 9 km
    // continents whether the radius is 6 km or 6000 km.
    const double datumRadius = params_.radiusMeters;

    // Domain warp: bend the sample position with a low-frequency vector
    // noise for swirling coastlines and bays.
    const Vec3 warpDirection = normalized(unitDirection
        + params_.domainWarpAmplitude
            * Vec3{fbm(hashCombine64(seedWarp_, 0), unitDirection * datumRadius, params_.domainWarpWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters),
                fbm(hashCombine64(seedWarp_, 1), unitDirection * datumRadius, params_.domainWarpWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters),
                fbm(hashCombine64(seedWarp_, 2), unitDirection * datumRadius, params_.domainWarpWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters)});
    const Vec3 warpedPosition = warpDirection * datumRadius;

    // Continent base: signed low-frequency field; positive rises above the
    // datum (land), negative dips below (ocean basins).
    const double continents = fbm(seedContinents_, warpedPosition, params_.continentWavelengthMeters, params_.continentsFbm, lowPassCutoffMeters);
    double       height     = params_.continentAmplitudeMeters * continents;

    // Mountains grow where the continent field is strongly positive. A broad
    // "range cluster" field gates the mask a second time so uplift groups
    // into distinct ranges separated by lower foothill country (a uniform
    // mask carpets every landmass with peaks - the globe reads spiky).
    const double baseMountainMask = smoothstep(params_.mountainMaskLow, params_.mountainMaskHigh, continents);
    const double clusterField     = fbm(seedRangeCluster_, warpedPosition, params_.rangeClusterWavelengthMeters,
                                  FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters);
    const double clusterGate      = smoothstep(-0.35, 0.45, clusterField);
    const double mountainMask     = clamp01(baseMountainMask * mix(1.0 - params_.rangeClusterStrength, 1.0, clusterGate));

    // Mountain profile: split the amplitude between a broad, smooth mass
    // field (wide uplifted massifs with real silhouette) and the ridged
    // crest field (ridge lines riding on top of the mass). A pure ridge sum
    // concentrates all height in narrow crest lines - needles from orbit,
    // all-rib slopes up close. The crest is weighted toward the mass so the
    // highest ridges crown the biggest massifs.
    const double ridges     = ridgedFbm(seedMountains_, warpedPosition, params_.mountainWavelengthMeters, params_.mountainsFbm, lowPassCutoffMeters);
    const double mass       = 0.5 + 0.5 * fbm(seedMountainMass_, warpedPosition, params_.derivedMountainMassWavelengthMeters,
                              FbmParams{3, 2.0, 0.5}, lowPassCutoffMeters);
    const double crest      = std::pow(ridges, params_.mountainSharpness);
    const double mountainProfile = params_.mountainMassFraction * mass
        + (1.0 - params_.mountainMassFraction) * crest * (0.35 + 0.65 * mass);
    height += params_.mountainAmplitudeMeters * mountainMask * mountainProfile;

    // Submarine fade: ocean basins are smooth sediment plains. Detail fields
    // (hills, overhangs, strata) fade out with depth below the datum, so the
    // sea floor stays clean. The band is HARD and narrow (it closes just
    // above the datum): a generous fade band let the medium-frequency layers
    // raise shallow shoals into archipelago speckle at globe scale - shoals
    // deeper than a few meters must carry NOTHING but the continent field,
    // whose own fine octaves (tens of meters of amplitude) then cannot cross
    // the waterline either.
    const double detailFade = smoothstep(-0.02 * params_.continentAmplitudeMeters, 0.06 * params_.continentAmplitudeMeters, height);

    // Coastal flattening: waves and sediment plan off the small-scale relief
    // right at the waterline. Detail terms (hills, terraces, overhangs,
    // strata) are damped inside a band around the datum, so coastlines slope
    // into the sea smoothly instead of zig-zagging across it - the zig-zag
    // reads as a web of sub-sea ponds, each with its own beach rim. The band
    // follows the regional field, so regional basins that dip below the datum
    // flatten into lagoons exactly where they should.
    // The curve is STEEPENED: full damping until nearly half the band width,
    // full strength only past the band edge. A plain smoothstep from zero
    // already reaches ~0.7 at half width, which let hill octaves raise
    // shallow shoals into archipelago speckle at globe scale.
    const double flattenWidth = std::max(1.0, params_.coastalFlattenMeters);
    const auto coastalTaperOf = [this, &flattenWidth](double surfaceHeight)
    {
        return mix(params_.coastalFlattenFloor, 1.0,
            smoothstep(0.45 * flattenWidth, 1.6 * flattenWidth,
                std::abs(surfaceHeight - params_.seaLevelOffsetMeters)));
    };

    // Regional relief: broad plateaus, basins and valley systems between the
    // continent and hill bands. This is the layer that carries landform
    // structure at globe scale (where anything finer than a rendered voxel
    // is invisible) and gives mid-altitude views large, readable shapes.
    // Damped halfway toward the coast band so regional basins cannot shred
    // marginal land into islands (basins inland keep their full depth).
    const double regionalTaper = mix(1.0, coastalTaperOf(height), 0.5);
    height += params_.regionalAmplitudeMeters * detailFade
        * regionalTaper
        * fbm(seedRegional_, unitDirection * datumRadius, params_.regionalWavelengthMeters, params_.regionalFbm, lowPassCutoffMeters);

    const double coastalTaper = coastalTaperOf(height);

    // Rolling medium-frequency hills everywhere on land.
    height += params_.hillAmplitudeMeters * detailFade * coastalTaper
        * fbm(seedHills_, unitDirection * datumRadius, params_.hillWavelengthMeters, params_.hillsFbm, lowPassCutoffMeters);

    // Terraced-country mask drives both the terrace quantization below and
    // the detail/overhang weighting; compute it before the fine layers.
    const double cliffMask = clamp01(0.5
        + 0.9 * fbm(seedCliffMask_, unitDirection * datumRadius, params_.cliffWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters));

    // Fine detail layer: rock ribs, soil ripples, scree - the roughness that
    // keeps close-range views from reading as smooth plastic. Carried fully
    // by mountains and cliffs, partially by plains (a large plains fraction
    // turns open ground into bump fields), faded under water and along the
    // flattened shoreline band.
    const double detailMaskLevel = clamp01(params_.detailPlainsFraction
        + (1.0 - params_.detailPlainsFraction) * (0.8 * mountainMask + 0.6 * cliffMask));
    height += params_.detailAmplitudeMeters * detailFade * coastalTaper * detailMaskLevel
        * fbm(seedDetail_, unitDirection * datumRadius, params_.detailWavelengthMeters, params_.detailFbm, lowPassCutoffMeters);

    // Terraces: quantize the height profile inside a noise-masked band,
    // producing flat plateaus joined by steep cliff steps (mesa country).
    const double u    = height / terraceStepMeters_;
    const double k    = std::floor(u);
    const double f    = u - k;
    const double edge = smoothstep(0.30, 0.70, f);
    height            = mix(height, (k + edge) * terraceStepMeters_, params_.terraceStrength * cliffMask * detailFade * coastalTaper);

    relief.heightMeters   = height;
    relief.mountainFactor = mountainMask;
    relief.cliffFactor    = cliffMask;
    relief.detailFade     = detailFade;
    relief.coastalTaper   = coastalTaper;
    return relief;
}

double DensityField::densityAt(const Vec3 &point, double lowPassCutoffMeters) const
{
    const double radius = length(point);
    if (radius < 1.0e-9)
    {
        // The planet center is deep inside the body.
        return params_.radiusMeters;
    }
    const Vec3 direction = Vec3{point.x / radius, point.y / radius, point.z / radius};

    const ReliefSample relief = reliefImpl(direction, lowPassCutoffMeters);
    double             d      = (params_.radiusMeters + relief.heightMeters) - radius;

    // Overhang displacement: true 3D noise added to the signed density.
    // Concentrated on mountainous/terraced ground - only a small configured
    // fraction survives on plains (a large plains fraction boils the whole
    // surface into lumps), it pushes the iso-surface sideways as well as up:
    // the term that makes overhangs, ledges and cave mouths possible.
    const double overhangMask = clamp01(params_.overhangPlainsFraction
        + (1.0 - params_.overhangPlainsFraction) * (0.8 * relief.mountainFactor + 0.6 * relief.cliffFactor))
        * relief.detailFade * relief.coastalTaper;
    d += noise::simplex3ImproveXY(seedOverhang_,
             point.x / params_.overhangWavelengthMeters,
             point.y / params_.overhangWavelengthMeters,
             point.z / params_.overhangWavelengthMeters)
        * params_.overhangAmplitudeMeters * overhangMask
        * lowPassFade(lowPassCutoffMeters, params_.overhangWavelengthMeters);

    // Rock strata: sedimentary banding that follows the body's shell, bent by
    // a noise field so the bands are not perfect rings. Gated to GENUINELY
    // terraced country (cliff mask well above its 0.5 baseline): the raw mask
    // averages 0.5 on open slopes, where a sinusoid at this amplitude turns
    // whole hillsides into lit/shadow corduroy under grazing sun. The warp
    // strength must stay small: a strong phase warp folds the sinusoid around
    // noise extrema and the cliff face dissolves into nested ripple rings.
    const double strataGate  = smoothstep(0.45, 0.85, relief.cliffFactor);
    const double bandPhase   = radius / std::max(1.0, params_.strataBandMeters);
    const double bandWarp    = noise::simplex3ImproveXY(seedStrata_,
                              point.x / std::max(1.0, params_.strataBandMeters),
                              point.y / std::max(1.0, params_.strataBandMeters),
                              point.z / std::max(1.0, params_.strataBandMeters));
    d += params_.strataAmplitudeMeters * std::sin(6.283185307179586 * bandPhase + params_.strataWarp * bandWarp)
        * strataGate * relief.detailFade * relief.coastalTaper
        * lowPassFade(lowPassCutoffMeters, params_.strataBandMeters);

    // Cave worms: two noise fields; where both are near zero their iso-
    // surfaces intersect in tunnels, and the density is carved out. The carve
    // fades with the worm wavelength under the band limit like every other
    // term.
    if (params_.cavesEnabled)
    {
        const double wormA = fbm(seedCaveA_, point, params_.caveWormWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters);
        const double wormB = fbm(seedCaveB_, point, params_.caveWormWavelengthMeters, FbmParams{2, 2.0, 0.5}, lowPassCutoffMeters);
        const double wormSquared = wormA * wormA + wormB * wormB;
        const double threshold   = params_.caveThreshold;
        if (wormSquared < threshold * threshold)
        {
            d -= params_.caveCarveMeters * (1.0 - std::sqrt(wormSquared) / threshold)
                * lowPassFade(lowPassCutoffMeters, params_.caveWormWavelengthMeters);
        }
    }

    return d;
}

double DensityField::surfaceRadius(const Vec3 &unitDirection) const
{
    const double band   = params_.derivedMaxReliefMeters * 2.0 + params_.overhangAmplitudeMeters * 2.0 + params_.caveCarveMeters;
    double       lowRadius  = std::max(1.0, params_.radiusMeters - band);
    double       highRadius = params_.radiusMeters + band;

    if (densityAt(Vec3{unitDirection.x * lowRadius, unitDirection.y * lowRadius, unitDirection.z * lowRadius}) < 0.0)
    {
        // No solid below the band on this ray (degenerate parameters); give
        // the datum so callers still get a usable answer.
        return params_.radiusMeters;
    }

    for (int iteration = 0; iteration < 48; ++iteration)
    {
        const double midRadius = 0.5 * (lowRadius + highRadius);
        const Vec3   midPoint{unitDirection.x * midRadius, unitDirection.y * midRadius, unitDirection.z * midRadius};
        if (densityAt(midPoint) > 0.0)
        {
            lowRadius = midRadius;
        }
        else
        {
            highRadius = midRadius;
        }
    }
    return 0.5 * (lowRadius + highRadius);
}

} // namespace planet
} // namespace engine
