// Terrain density field implementation (see density.hpp).

#include <cstdint>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <engine/planet/fbm.hpp>
#include <engine/planet/noise.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace engine
{
namespace planet
{
namespace
{

// Fixed-shape fBm layers (warp, cluster, mass, cliff mask, caves, provinces)
// share this profile; configured layers (continents, mountains, ...) carry
// their own.
constexpr FbmParams kFixedFbm{2, 2.0, 0.5};
constexpr FbmParams kMassFbm{3, 2.0, 0.5};
constexpr FbmParams kProvinceFbm{2, 2.0, 0.5};

// Bisection stops once the surface bracket is tighter than this (sub-
// millimeter at any supported radius) - further halvings cannot move the
// reported crossing measurably, and the saved evaluations add up across the
// scans the viewpoint picker and the walker issue per frame.
constexpr double kSurfaceBisectResolutionMeters = 1.0e-3;

} // namespace

DensityField::DensityField(const PlanetGenerationParams &params)
    : params_(params)
    , climate_(params)
    , clouds_(params)
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
    , seedErosion_(static_cast<std::int64_t>(hashCombine64(params.seed, 0xE9051AULL)))
    , erosion_(params, seedErosion_)
    , fbmSeeds_()
{
    const double reliefRange = params.continentAmplitudeMeters + params.mountainAmplitudeMeters + params.hillAmplitudeMeters;
    terraceStepMeters_ = std::max(1.0, reliefRange / std::max(1.0, params.terraceSteps));

    // Per-octave fBm seeds, derived once per field (see FbmStreamSeeds).
    const auto streamSeed = [](std::int64_t base, std::uint64_t tag)
    { return static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(base), tag)); };
    fbmSeeds_[streamWarpX]        = makeFbmSeeds(streamSeed(seedWarp_, 0), 2);
    fbmSeeds_[streamWarpY]        = makeFbmSeeds(streamSeed(seedWarp_, 1), 2);
    fbmSeeds_[streamWarpZ]        = makeFbmSeeds(streamSeed(seedWarp_, 2), 2);
    fbmSeeds_[streamContinents]   = makeFbmSeeds(seedContinents_, params.continentsFbm.octaves);
    fbmSeeds_[streamRangeCluster] = makeFbmSeeds(seedRangeCluster_, 2);
    fbmSeeds_[streamMountainMass] = makeFbmSeeds(seedMountainMass_, 3);
    fbmSeeds_[streamMountains]    = makeFbmSeeds(seedMountains_, params.mountainsFbm.octaves);
    fbmSeeds_[streamRegional]     = makeFbmSeeds(seedRegional_, params.regionalFbm.octaves);
    fbmSeeds_[streamHills]        = makeFbmSeeds(seedHills_, params.hillsFbm.octaves);
    fbmSeeds_[streamCliffMask]    = makeFbmSeeds(seedCliffMask_, 2);
    fbmSeeds_[streamDetail]       = makeFbmSeeds(seedDetail_, params.detailFbm.octaves);
    fbmSeeds_[streamCaveA]        = makeFbmSeeds(seedCaveA_, 2);
    fbmSeeds_[streamCaveB]        = makeFbmSeeds(seedCaveB_, 2);
    // Geologic provinces: the rock-character stream and the carbonate stream
    // share the fixed two-octave profile at the derived province wavelength
    // (the carbonate patch rides a smaller multiple so limestone country
    // patches in at a finer scale than the sediment/shield split).
    fbmSeeds_[streamProvince]          = makeFbmSeeds(streamSeed(seedContinents_, 0x9B0B1EULL), kProvinceFbm.octaves);
    fbmSeeds_[streamProvinceCarbonate] = makeFbmSeeds(streamSeed(seedContinents_, 0x9B0B2FULL), kProvinceFbm.octaves);
}

ReliefSample DensityField::reliefAt(const Vec3 &unitDirection) const
{
    return reliefImpl(unitDirection, 0.0, false);
}

Vec3 DensityField::warpedSamplePosition(const Vec3 &unitDirection, double lowPassCutoffMeters) const
{
    // All continent-scale fields sample the noise at PHYSICAL positions
    // (meters) divided by their wavelength, so feature sizes stay metric at
    // any planet radius: a 9 km continent wavelength produces 9 km
    // continents whether the radius is 6 km or 6000 km.
    const double datumRadius = params_.radiusMeters;
    const Vec3   radialPosition = unitDirection * datumRadius;

    // Domain warp: bend the sample position with a low-frequency vector
    // noise for swirling coastlines and bays. Skipped entirely when the
    // amplitude is configured to zero (six noise evaluations per sample).
    if (params_.domainWarpAmplitude <= 0.0)
    {
        return radialPosition;
    }
    const Vec3 warpDirection = normalized(unitDirection
        + params_.domainWarpAmplitude
            * Vec3{fbm(fbmSeeds_[streamWarpX], radialPosition, params_.domainWarpWavelengthMeters, kFixedFbm, lowPassCutoffMeters),
                fbm(fbmSeeds_[streamWarpY], radialPosition, params_.domainWarpWavelengthMeters, kFixedFbm, lowPassCutoffMeters),
                fbm(fbmSeeds_[streamWarpZ], radialPosition, params_.domainWarpWavelengthMeters, kFixedFbm, lowPassCutoffMeters)});
    return warpDirection * datumRadius;
}

double DensityField::continentFieldAt(const Vec3 &warpedPosition, double lowPassCutoffMeters) const
{
    // Continent base: signed low-frequency field; positive rises above the
    // datum (land), negative dips below (ocean basins).
    return fbm(fbmSeeds_[streamContinents], warpedPosition, params_.continentWavelengthMeters, params_.continentsFbm,
        lowPassCutoffMeters);
}

double DensityField::mountainMassAt(const Vec3 &warpedPosition, double lowPassCutoffMeters) const
{
    // Broad, smooth mountain-mass envelope in [0, 1] (see the mountain
    // profile below); shared by the relief composition and the climate's
    // rain shadow so both agree about where mountain country sits.
    return 0.5 + 0.5 * fbm(fbmSeeds_[streamMountainMass], warpedPosition,
        params_.derivedMountainMassWavelengthMeters, kMassFbm, lowPassCutoffMeters);
}

Vec3 DensityField::prevailingWindDownwind(const Vec3 &unitDirection) const
{
    // Unit tangent pointing DOWNWIND at this direction (the way the wind
    // blows, not where it comes from). The frame derives from the spin axis:
    // east is the direction the body's rotation carries the surface. With
    // the latitude zones enabled the direction follows the circulation cells
    // every rotating planet shows - easterly trades inside the tropics
    // (blowing toward west), westerlies over the temperate band, easterly
    // polar winds - with the zones blended over a band of latitude so a
    // walking camera never sees the wind direction flip.
    const Vec3 axis{params_.axis[0], params_.axis[1], params_.axis[2]};
    Vec3 east = cross(unitDirection, axis);
    const double eastLength = length(east);
    if (eastLength < 1.0e-9)
    {
        return Vec3{0.0, 0.0, 0.0}; // at the pole: no meaningful prevailing wind
    }
    east = east * (1.0 / eastLength);
    if (!params_.climateWindZonesEnabled)
    {
        return east; // uniform westerly flow
    }
    const double absSinLatitude = clamp01(std::abs(dot(unitDirection, axis)));
    const double latitudeDegrees = std::asin(absSinLatitude) * (180.0 / 3.14159265358979323846);
    const double tradeWesterly = 1.0 - smoothstep(24.0, 36.0, latitudeDegrees); // 1 = blowing toward west
    const double polarEasterly = smoothstep(54.0, 66.0, latitudeDegrees);
    const double westward = std::max(tradeWesterly, polarEasterly);
    return east * (1.0 - 2.0 * westward);
}

void DensityField::macroSampleAt(const Vec3 &surfacePoint, double lowPassCutoffMeters, ClimateMacroSample &outMacro) const
{
    const double radius = std::max(1.0e-9, length(surfacePoint));
    const Vec3  direction = Vec3{surfacePoint.x / radius, surfacePoint.y / radius, surfacePoint.z / radius};

    // Sea-level temperature and moisture (latitude gradient, perturbation
    // noise, warp, latitude belts).
    const ClimateSample sea = climate_.sampleSeaLevel(direction);
    double moisture = sea.moisture01;

    // Continental interiors dry out away from the ocean: the same continent
    // field that shapes the landmasses rises toward its interior cores, so
    // the interior factor climbs with it. The coast keeps its onshore
    // humidity boost (applied in the per-vertex climate sample); this term
    // pulls the deep interiors toward steppe and desert.
    if (params_.climateContinentalityStrength > 0.0)
    {
        const Vec3 position = warpedSamplePosition(direction, lowPassCutoffMeters);
        const double continent = continentFieldAt(position, lowPassCutoffMeters);
        moisture -= params_.climateContinentalityStrength * smoothstep(0.0, 0.85, continent);
    }

    // Rain shadow: mountain country upwind wrings the moisture out of the
    // air, so the lee side carries a dry belt. The mountain MASS envelope is
    // sampled upwind along the prevailing wind at roughly the mountain
    // wavelength's reach - the distance orographic precipitation plausibly
    // extends.
    if (params_.climateRainShadowStrength > 0.0)
    {
        const Vec3 wind = prevailingWindDownwind(direction);
        if (lengthSquared(wind) > 0.5)
        {
            const double reach = std::max(1.0, params_.mountainWavelengthMeters * 1.25);
            const Vec3 upwindDirection = normalized(direction + wind * (reach / radius));
            const double mass = mountainMassAt(upwindDirection * params_.radiusMeters, lowPassCutoffMeters);
            moisture -= params_.climateRainShadowStrength * mass;
        }
    }

    outMacro.temperature01 = sea.temperature01;
    outMacro.moisture01    = clamp01(moisture);
    outMacro.cloudShadow   = cloudShadow(direction, lowPassCutoffMeters);

    // Geologic province character (see the provinces keys in params.hpp).
    outMacro.provinceChemistry = 0.0;
    outMacro.provinceCarbonate = 0.0;
    if (params_.provincesEnabled)
    {
        const Vec3 radial = direction * params_.radiusMeters;
        const double provinceField = fbm(fbmSeeds_[streamProvince], radial,
            params_.derivedProvinceWavelengthMeters, kProvinceFbm, lowPassCutoffMeters);
        outMacro.provinceChemistry = smoothstep(-0.30, 0.45, provinceField);
        const double carbonateField = fbm(fbmSeeds_[streamProvinceCarbonate], radial,
            params_.derivedProvinceWavelengthMeters * 0.43, kProvinceFbm, lowPassCutoffMeters);
        // Carbonate provinces follow the climate the way real limestone
        // country does: platforms and their exposed reef tracts sit in warm
        // lowlands (tropical shelves and karst plains), while cold or alpine
        // country keeps the chemistry's shield/volcanic split whatever the
        // raw field says. The gate rides the sea-level temperature this macro
        // sample already carries, and an elevation band that runs from the
        // shelf up to the mid slopes (scaled by the water and snow-line keys
        // the climate already uses), so the pale limestone tint lands where
        // geology puts it instead of painting random polar massifs white.
        const double elevationMeters = radius - params_.derivedSeaRadiusMeters;
        const double warmLowland = smoothstep(0.35, 0.65, sea.temperature01)
            * (1.0 - smoothstep(params_.deepWaterMeters, 0.5 * params_.snowLineMeters, elevationMeters));
        outMacro.provinceCarbonate = smoothstep(0.10, 0.72, carbonateField) * warmLowland;
    }
}

ClimateSample DensityField::sampleClimate(const Vec3 &unitDirection, double radiusMeters, double slope01,
    double lowPassCutoffMeters) const
{
    ClimateMacroSample macro;
    macroSampleAt(unitDirection * radiusMeters, lowPassCutoffMeters, macro);
    return climate_.sample(macro, unitDirection, radiusMeters, slope01, lowPassCutoffMeters);
}

ReliefSample DensityField::reliefImpl(const Vec3 &unitDirection, double lowPassCutoffMeters, bool denseSampling) const
{
    ReliefSample relief;

    const Vec3 warpedPosition = warpedSamplePosition(unitDirection, lowPassCutoffMeters);
    const double continents = continentFieldAt(warpedPosition, lowPassCutoffMeters);
    double       height     = params_.continentAmplitudeMeters * continents;
    // Close-range layers (regional, hills, cliff mask, detail) sample the
    // UNWARPED radial position: the warp bends the continent-scale geography
    // only, and mixing it into the near-field layers would smear terrain
    // texture along the warp's own contours.
    const Vec3 radialPosition = unitDirection * params_.radiusMeters;

    // Abyssal deepening: the sea floor sinks from the shoreline into the
    // basins, carving a real ocean profile - a shallow continental shelf at
    // the coast, a steep slope, dark abyssal plains offshore. The mask rides
    // the (band-limited) continent field, so the shelf is as wide and smooth
    // as the continent geography itself; the deepened floor also keeps every
    // detail layer faded out below the slope. 0 disables.
    const double oceanDeepMask = 1.0 - smoothstep(params_.abyssalMaskLow, params_.abyssalMaskHigh, continents);
    height -= params_.abyssalDepthMeters * oceanDeepMask;

    // Mountains grow where the continent field is strongly positive. A broad
    // "range cluster" field gates the mask a second time so uplift groups
    // into distinct ranges separated by lower foothill country (a uniform
    // mask carpets every landmass with peaks - the globe reads spiky).
    const double baseMountainMask = smoothstep(params_.mountainMaskLow, params_.mountainMaskHigh, continents);
    double       mountainMask     = 0.0;
    if (baseMountainMask > 0.0)
    {
        const double clusterField = fbm(fbmSeeds_[streamRangeCluster], warpedPosition, params_.rangeClusterWavelengthMeters,
                                  kFixedFbm, lowPassCutoffMeters);
        const double clusterGate  = smoothstep(params_.rangeClusterGateLow, params_.rangeClusterGateHigh, clusterField);
        mountainMask = clamp01(baseMountainMask * mix(1.0 - params_.rangeClusterStrength, 1.0, clusterGate));
    }

    // Mountain profile: split the amplitude between a broad, smooth mass
    // field (wide uplifted massifs with real silhouette) and the ridged
    // crest field (ridge lines riding on top of the mass). A pure ridge sum
    // concentrates all height in narrow crest lines - needles from orbit,
    // all-rib slopes up close. The crest is weighted toward the mass so the
    // highest ridges crown the biggest massifs. Evaluated only where the
    // mask lets mountains contribute at all: over the oceans (about two
    // thirds of a typical globe) the crest and mass fields would be pure
    // waste, and they are the two most expensive layers of the field.
    // With the erosion filter weighted in, the belt relief instead comes
    // from the analytic eroded-mountain field (see erosion.hpp): the
    // filter's height remaps onto the same [0, 1] profile span, the ridge
    // map takes over the crest-line role (cliff energy below keys on it),
    // and the classic mass/crest streams are skipped entirely at full
    // weight - the erosion sample costs more than they do, but only where
    // mountains actually rise, and its interpolation lattice amortizes it
    // across every voxel of a chunk.
    double       ridges        = 0.0; // crest measure feeding the cliff mask
    if (mountainMask > 0.0)
    {
        const double erosionWeight = params_.mountainErosionWeight;
        double fbmProfile    = 0.0;
        double erosionProfile = 0.0;
        if (erosionWeight < 1.0)
        {
            ridges = ridgedFbm(fbmSeeds_[streamMountains], warpedPosition, params_.mountainWavelengthMeters,
                params_.mountainsFbm, lowPassCutoffMeters);
            const double mass  = mountainMassAt(warpedPosition, lowPassCutoffMeters);
            const double crest = std::pow(ridges, params_.mountainSharpness);
            fbmProfile = params_.mountainMassFraction * mass
                + (1.0 - params_.mountainMassFraction) * crest
                    * (params_.mountainCrestMassCoupling + (1.0 - params_.mountainCrestMassCoupling) * mass);
        }
        if (erosionWeight > 0.0)
        {
            const MountainErosionSample eroded =
                erosion_.sample(unitDirection * params_.radiusMeters, lowPassCutoffMeters, denseSampling);
            erosionProfile = clamp01((eroded.height - params_.mountainErosionProfileLow)
                / std::max(0.01, params_.mountainErosionProfileHigh - params_.mountainErosionProfileLow));
            // The ridge map (-1 crease .. +1 ridge) is the eroded profile's
            // own crest measure; the blend keeps the cliff-mask energy
            // continuous across the weight range.
            ridges = mix(ridges, clamp01(0.5 + 0.5 * eroded.ridgeMap), erosionWeight);
        }
        const double mountainProfile = mix(fbmProfile, erosionProfile, erosionWeight);
        height += params_.mountainAmplitudeMeters * mountainMask * mountainProfile;
    }

    // Submarine fade: ocean basins are smooth sediment plains. Detail fields
    // (regional relief, hills, overhangs, strata) fade out with depth below
    // the datum, so the sea floor stays clean. The band is HARD and narrow
    // (it closes just above the datum): a generous fade band let the
    // medium-frequency layers raise shallow shoals into archipelago speckle
    // at globe scale - shoals deeper than a few meters must carry NOTHING
    // but the continent field, whose own fine octaves (tens of meters of
    // amplitude) then cannot cross the waterline either.
    const double detailFade = smoothstep(-params_.submarineFadeLowFraction * params_.continentAmplitudeMeters,
        params_.submarineFadeHighFraction * params_.continentAmplitudeMeters, height);

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
            smoothstep(params_.coastalFlattenFullFraction * flattenWidth, params_.coastalFlattenReachFraction * flattenWidth,
                std::abs(surfaceHeight - params_.seaLevelOffsetMeters)));
    };

    // Regional relief: broad plateaus, basins and valley systems between the
    // continent and hill bands. This is the layer that carries landform
    // structure at globe scale (where anything finer than a rendered voxel
    // is invisible) and gives mid-altitude views large, readable shapes.
    // Damped halfway toward the coast band so regional basins cannot shred
    // marginal land into islands (basins inland keep their full depth).
    double cliffMask   = 0.0;
    double coastalTaper = coastalTaperOf(height);
    // Rock character of this point (0 = shield/volcanic, 1 = sedimentary);
    // evaluated with the land detail fields below and carried out on the
    // relief sample so the strata banding in densityAt() can bias with it.
    double provinceChemistry = 0.0;
    if (detailFade > 0.0)
    {
        const double regionalTaper = mix(1.0, coastalTaper, 0.5);
        height += params_.regionalAmplitudeMeters * detailFade
            * regionalTaper
            * fbm(fbmSeeds_[streamRegional], radialPosition, params_.regionalWavelengthMeters, params_.regionalFbm, lowPassCutoffMeters);

        coastalTaper = coastalTaperOf(height);

        // Rolling medium-frequency hills everywhere on land. The field's
        // VALUE is kept: its flanks (near its zero crossings) are the steep
        // hill country the cliff mask below reads.
        const double hillField = fbm(fbmSeeds_[streamHills], radialPosition, params_.hillWavelengthMeters, params_.hillsFbm, lowPassCutoffMeters);
        height += params_.hillAmplitudeMeters * detailFade * coastalTaper * hillField;

        // Terraced-country mask, keyed to the terrain's OWN steepness: hill
        // flanks ride the hill field's zero crossings (where that field
        // carries its gradient), massif crest lines ride the ridged crest
        // field, and the cliff-noise stream only MODULATES the bands for
        // organic variety at cliffWavelengthMeters scale. The mask must sit
        // on genuinely steep ground because terracing, strata banding and
        // overhang weighting all follow it: a decorrelated gate scatters
        // mesas over open ground, where quantization reads as corduroy, not
        // cliffs. The hill term is capped at half strength: hill country spans
        // about one terrace step of height, so quantization there can only
        // flatten (benches without risers) - full-strength steps belong to
        // massif flanks, whose relief crosses several steps.
        // cliffSharpness concentrates the hill-flank term; the steepness
        // gate (cliffSteepnessLow/High) trims how much of the flank country
        // qualifies.
        const double hillFlank = std::pow(clamp01(1.0 - std::abs(hillField)), params_.cliffSharpness);
        const double cliffVariety = 0.82 + 0.18 * fbm(fbmSeeds_[streamCliffMask], radialPosition, params_.cliffWavelengthMeters, kFixedFbm, lowPassCutoffMeters);
        const double crestRise = mountainMask * (0.30 + 0.70 * ridges);
        const double cliffEnergy = std::max(0.50 * hillFlank * cliffVariety, crestRise);
        cliffMask = smoothstep(params_.cliffSteepnessLow, params_.cliffSteepnessHigh, cliffEnergy);

        // Geologic province character of THIS point (see the provinces keys):
        // layered sedimentary country terraces and bands more readily than
        // shield/volcanic country, so the terrace strength biases with the
        // rock character. The same stream feeds the albedo tint (macro
        // sample), so what the geometry builds and what the surface paints
        // agree about the ground.
        if (params_.provincesEnabled)
        {
            const double provinceField = fbm(fbmSeeds_[streamProvince], radialPosition,
                params_.derivedProvinceWavelengthMeters, kProvinceFbm, lowPassCutoffMeters);
            provinceChemistry = smoothstep(-0.30, 0.45, provinceField);
        }
        const double terraceBias = mix(1.0 - params_.provinceTerraceBias, 1.0 + params_.provinceTerraceBias, provinceChemistry);

        // Fine detail layer: rock ribs, soil ripples, scree - the roughness
        // that keeps close-range views from reading as smooth plastic.
        // Carried fully by mountains and cliffs, partially by plains (a
        // large plains fraction turns open ground into bump fields), faded
        // under water and along the flattened shoreline band.
        const double detailMaskLevel = clamp01(params_.detailPlainsFraction
            + (1.0 - params_.detailPlainsFraction) * (0.8 * mountainMask + 0.6 * cliffMask));
        height += params_.detailAmplitudeMeters * detailFade * coastalTaper * detailMaskLevel
            * fbm(fbmSeeds_[streamDetail], radialPosition, params_.detailWavelengthMeters, params_.detailFbm, lowPassCutoffMeters);

        // Terraces: quantize the height profile inside the steepness-masked
        // band, producing flat plateaus joined by steep cliff steps (mesa
        // country). The riser spans terraceRiserFraction of each step - a
        // narrow riser keeps the step's wall genuinely cliff-like instead of
        // a long shoulder (the riser grade is the flank slope divided by the
        // fraction; see params.hpp). The quantization weight runs through a
        // SHARPER, lower gate than the cliff mask itself: the mask ramps
        // over the whole steepness band, and weighting the quantization by
        // the raw mask turns every mesa flank into a ramp that only snaps
        // into steps right at the crest lines. The terrace gate reaches
        // full strength a quarter of the way into the band, so genuinely
        // steep flanks quantize fully (crisp benches and risers) while open
        // ground below the band stays unquantized - the mask shape stays
        // untouched for the strata, overhang and detail weighting, which
        // grade with the terrain's true steepness.
        const double terraceGate = smoothstep(params_.cliffSteepnessLow,
            params_.cliffSteepnessLow + 0.25 * (params_.cliffSteepnessHigh - params_.cliffSteepnessLow), cliffEnergy);
        const double u    = height / terraceStepMeters_;
        const double k    = std::floor(u);
        const double f    = u - k;
        const double halfRiser = 0.5 * params_.terraceRiserFraction;
        const double edge = smoothstep(0.5 - halfRiser, 0.5 + halfRiser, f);
        height            = mix(height, (k + edge) * terraceStepMeters_,
            params_.terraceStrength * terraceBias * terraceGate * detailFade * coastalTaper);
    }

    relief.heightMeters   = height;
    relief.mountainFactor = mountainMask;
    relief.cliffFactor    = cliffMask;
    relief.detailFade     = detailFade;
    relief.coastalTaper   = coastalTaper;
    relief.provinceChemistry = provinceChemistry;
    return relief;
}

double DensityField::densityAt(const Vec3 &point, double lowPassCutoffMeters, bool denseSampling) const
{
    const double radius = length(point);
    if (radius < 1.0e-9)
    {
        // The planet center is deep inside the body.
        return params_.radiusMeters;
    }
    const Vec3 direction = Vec3{point.x / radius, point.y / radius, point.z / radius};

    const ReliefSample relief = reliefImpl(direction, lowPassCutoffMeters, denseSampling);
    double             d      = (params_.radiusMeters + relief.heightMeters) - radius;

    // Overhang displacement: true 3D noise added to the signed density.
    // Concentrated on mountainous/terraced ground - only a small configured
    // fraction survives on plains (a large plains fraction boils the whole
    // surface into lumps), it pushes the iso-surface sideways as well as up:
    // the term that makes overhangs, ledges and cave mouths possible.
    // The band-limit fade is tested BEFORE the noise evaluation: on chunks
    // whose cutoff sits at or above the overhang wavelength the term is
    // exactly zero, and the noise call is pure waste.
    const double overhangMask = clamp01(params_.overhangPlainsFraction
        + (1.0 - params_.overhangPlainsFraction) * (0.8 * relief.mountainFactor + 0.6 * relief.cliffFactor))
        * relief.detailFade * relief.coastalTaper;
    const double overhangFade = lowPassFade(lowPassCutoffMeters, params_.overhangWavelengthMeters);
    if (overhangMask > 0.0 && overhangFade > 0.0)
    {
        d += noise::simplex3ImproveXY(seedOverhang_,
                 point.x / params_.overhangWavelengthMeters,
                 point.y / params_.overhangWavelengthMeters,
                 point.z / params_.overhangWavelengthMeters)
            * params_.overhangAmplitudeMeters * overhangMask * overhangFade;
    }

    // Rock strata: sedimentary banding that follows the body's shell, bent by
    // a noise field so the bands are not perfect rings. Gated to GENUINELY
    // terraced country (cliff mask well above its 0.5 baseline): the raw mask
    // averages 0.5 on open slopes, where a sinusoid at this amplitude turns
    // whole hillsides into lit/shadow corduroy under grazing sun. The warp
    // strength must stay small: a strong phase warp folds the sinusoid around
    // noise extrema and the cliff face dissolves into nested ripple rings.
    // The band-limit fade gates the evaluation like the overhang above.
    if (relief.detailFade > 0.0 && relief.coastalTaper > 0.0)
    {
        const double strataFade = lowPassFade(lowPassCutoffMeters, params_.strataBandMeters);
        const double strataGate  = smoothstep(params_.strataGateLow, params_.strataGateHigh, relief.cliffFactor);
        if (strataGate > 0.0 && strataFade > 0.0)
        {
            // Sedimentary country (provinceChemistry -> 1) bands more
            // strongly than shield/volcanic country - layered beds vs
            // massive igneous rock - so the band amplitude biases with the
            // rock character the relief sample already carries.
            const double strataBias = mix(1.0 - params_.provinceStrataBias, 1.0 + params_.provinceStrataBias,
                params_.provincesEnabled ? relief.provinceChemistry : 0.0);
            const double bandWidth = std::max(1.0, params_.strataBandMeters);
            const double bandPhase = radius / bandWidth;
            // Two warps bend the bands: a spatial warp at the band scale
            // (strataWarp, keeping the bands from being perfect rings) and a
            // low-frequency THICKNESS warp that stretches and squeezes the
            // band spacing the way real sedimentary beds vary - a constant
            // period reads as a brick wall. The thickness warp shares its
            // seed and scale with the climate sampler's rock stripes, so the
            // color bands track these geometric ridges.
            const double bandWarp = noise::simplex3ImproveXY(seedStrata_,
                point.x / bandWidth, point.y / bandWidth, point.z / bandWidth);
            const double thicknessWarp = noise::simplex3ImproveXY(seedStrata_,
                point.x / (bandWidth * 5.0), point.y / (bandWidth * 5.0), point.z / (bandWidth * 5.0));
            // Bedding strength: real cliff faces alternate between strongly
            // banded and almost featureless rock; the thickness-warp field
            // (already evaluated) gates the band amplitude across each face
            // so the stripes come and go between outcrops instead of painting
            // every cliff with the same contrast.
            const double bedding = 0.30 + 0.70 * smoothstep(-0.55, 0.55, thicknessWarp);
            d += params_.strataAmplitudeMeters * strataBias * bedding
                * std::sin(6.283185307179586 * bandPhase + params_.strataWarp * bandWarp
                    + params_.strataWarp * 2.6 * thicknessWarp)
                * strataGate * relief.detailFade * relief.coastalTaper * strataFade;
        }
    }

    // Cave worms: two noise fields; where both are near zero their iso-
    // surfaces intersect in tunnels, and the density is carved out. The carve
    // fades with the worm wavelength under the band limit like every other
    // term. Like the strata, the carve lives inside the detail band - deep
    // under the sea floor the term is zero anyway.
    if (params_.cavesEnabled && relief.detailFade > 0.0)
    {
        // Two worm streams: their iso-surfaces intersect where BOTH are near
        // zero, so the second evaluation is skipped whenever the first alone
        // already sits outside the carve band (wormSquared >= threshold^2
        // needs |wormA| >= threshold).
        const double wormA = fbm(fbmSeeds_[streamCaveA], point, params_.caveWormWavelengthMeters, kFixedFbm, lowPassCutoffMeters);
        if (std::abs(wormA) < params_.caveThreshold)
        {
            const double wormB = fbm(fbmSeeds_[streamCaveB], point, params_.caveWormWavelengthMeters, kFixedFbm, lowPassCutoffMeters);
            const double wormSquared = wormA * wormA + wormB * wormB;
            const double threshold   = params_.caveThreshold;
            if (wormSquared < threshold * threshold)
            {
                d -= params_.caveCarveMeters * (1.0 - std::sqrt(wormSquared) / threshold)
                    * lowPassFade(lowPassCutoffMeters, params_.caveWormWavelengthMeters);
            }
        }
    }

    return d;
}

double DensityField::surfaceRadius(const Vec3 &unitDirection) const
{
    // The scan starts below the deepest possible valley and walks outward;
    // the first solid-to-air bracket holds the surface a camera or probe
    // would hit first (overhangs and caves may create further crossings
    // above it that this function deliberately ignores).
    //
    // The march bracket comes from the terrain's own height profile: along a
    // radial ray the heightfield part of the density is EXACTLY linear
    // (radius + height(direction) - r), so every zero crossing of the
    // composed field must sit within the 3D terms' total amplitude of the
    // heightfield surface - a bracket of a couple of hundred meters instead
    // of the whole relief band. The wide band remains as the fallback for
    // degenerate parameters where the profile guess fails to bracket.
    const double band3d = params_.overhangAmplitudeMeters + params_.strataAmplitudeMeters
        + (params_.cavesEnabled ? params_.caveCarveMeters : 0.0);
    const double wideBand = params_.derivedMaxReliefMeters * 2.0 + params_.overhangAmplitudeMeters * 2.0
        + params_.caveCarveMeters;
    const double wideLowRadius = std::max(1.0, params_.radiusMeters - wideBand);
    const double wideHighRadius = params_.radiusMeters + wideBand;

    double lowRadius = wideLowRadius;
    double highRadius = wideHighRadius;
    double lowDensity = 0.0;
    const ReliefSample profile = reliefImpl(unitDirection, 0.0, false);
    if (std::isfinite(profile.heightMeters))
    {
        const double guessLow = params_.radiusMeters + profile.heightMeters - band3d - 1.0;
        const double guessHigh = params_.radiusMeters + profile.heightMeters + band3d + 1.0;
        if (guessLow > 1.0)
        {
            const Vec3 guessLowPoint{unitDirection.x * guessLow, unitDirection.y * guessLow, unitDirection.z * guessLow};
            lowDensity = densityAt(guessLowPoint);
            if (lowDensity > 0.0)
            {
                const Vec3 guessHighPoint{
                    unitDirection.x * guessHigh, unitDirection.y * guessHigh, unitDirection.z * guessHigh};
                if (densityAt(guessHighPoint) <= 0.0)
                {
                    lowRadius = guessLow;
                    highRadius = guessHigh;
                }
            }
        }
    }
    if (lowRadius == wideLowRadius)
    {
        lowDensity = densityAt(Vec3{unitDirection.x * lowRadius, unitDirection.y * lowRadius, unitDirection.z * lowRadius});
    }

    if (lowDensity < 0.0)
    {
        // No solid below the bracket on this ray (degenerate parameters); give
        // the datum so callers still get a usable answer.
        return params_.radiusMeters;
    }

    // March outward in fixed steps until the density drops below the
    // isolevel, then bisect inside that bracket. The step count balances
    // the march cost against the bisection depth: with the bracket at most
    // one step wide, 32 bisection iterations resolve well below a centimeter
    // at any supported radius.
    constexpr std::uint32_t kMarchSteps   = 24;
    constexpr std::uint32_t kBisectSteps  = 32;
    const double stepMeters = (highRadius - lowRadius) / static_cast<double>(kMarchSteps);
    double       belowRadius = lowRadius;
    double       aboveRadius = highRadius;
    bool         bracketed   = false;
    for (std::uint32_t step = 1; step <= kMarchSteps; ++step)
    {
        const double testRadius = lowRadius + stepMeters * static_cast<double>(step);
        const Vec3   testPoint{unitDirection.x * testRadius, unitDirection.y * testRadius, unitDirection.z * testRadius};
        if (densityAt(testPoint) <= 0.0)
        {
            aboveRadius = testRadius;
            belowRadius = testRadius - stepMeters;
            bracketed   = true;
            break;
        }
        belowRadius = testRadius;
    }
    if (!bracketed)
    {
        // The field stays solid across the whole band (degenerate
        // parameters); give the band top so callers still get a usable
        // answer outside the body.
        return highRadius;
    }
    for (std::uint32_t iteration = 0; iteration < kBisectSteps; ++iteration)
    {
        if (aboveRadius - belowRadius <= kSurfaceBisectResolutionMeters)
        {
            break;
        }
        const double midRadius = 0.5 * (belowRadius + aboveRadius);
        const Vec3   midPoint{unitDirection.x * midRadius, unitDirection.y * midRadius, unitDirection.z * midRadius};
        if (densityAt(midPoint) > 0.0)
        {
            belowRadius = midRadius;
        }
        else
        {
            aboveRadius = midRadius;
        }
    }
    return 0.5 * (belowRadius + aboveRadius);
}

double DensityField::cloudShadow(const Vec3 &unitDirection, double lowPassCutoffMeters) const
{
    if (params_.cloudShadowStrength <= 0.0 || !params_.cloudsEnabled)
    {
        return 0.0;
    }
    // The bank that blocks THIS point's sunlight sits up-sun along the ray
    // through the point: from the surface, the ray toward the sun climbs to
    // the cloud shell over t = shellHeight / (sun . up). Sampling the
    // coverage field THERE makes each shadow fall away from its bank (the
    // way real cloud shadows do at anything but noon) instead of hiding
    // directly beneath the cloud that casts it. Night points (sun at or
    // below the local horizon) get no direct sun to block; the shift is
    // clamped so a grazing sun cannot fling the sample across the planet.
    const Vec3 sun{params_.cloudShadowSunDirection[0], params_.cloudShadowSunDirection[1],
        params_.cloudShadowSunDirection[2]};
    const double sunUp = dot(sun, unitDirection);
    if (sunUp <= 0.02)
    {
        return 0.0;
    }
    const double shellHeight = std::max(1.0, params_.cloudAltitudeMeters);
    const double rise = std::min(shellHeight / sunUp, 8.0 * shellHeight);
    const Vec3 samplePoint = unitDirection * params_.radiusMeters + sun * rise;
    // The cast shade samples the coverage field with a WIDER threshold band
    // than the painted banks (cloud_shadow_softness scales the edge width):
    // sunlight around a cloud comes through a penumbra, so the shadow's
    // edges blur beyond the outline that casts them instead of repeating it
    // as a hard copy - the harsh dark blotches a 1:1 shade produces read as
    // stencil cuts, not weather.
    const double shadowSoftness = 1.0 + 2.0 * params_.cloudShadowSoftness;
    return params_.cloudShadowStrength * clouds_.coverage(normalized(samplePoint), lowPassCutoffMeters, shadowSoftness);
}

} // namespace planet
} // namespace engine
