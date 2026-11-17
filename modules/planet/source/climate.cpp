// Climate and biome classification (see climate.hpp).

#include <cstddef>
#include <cstdint>
#include <engine/planet/climate.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <engine/planet/fbm.hpp>
#include <engine/planet/noise.hpp>

#include <algorithm>
#include <cmath>

namespace engine
{
namespace planet
{
namespace
{

// Gaussian-style latitude belt: 1 at centerDegrees, fading over widthDegrees.
double latitudeBelt(double latitudeDegrees, double centerDegrees, double widthDegrees)
{
    const double offset = (latitudeDegrees - centerDegrees) / widthDegrees;
    return std::exp(-offset * offset);
}

// Copy one palette row into a double buffer.
void rowOf(const float *row, double out[3])
{
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        out[channel] = row[channel];
    }
}

// Fade for a SHARP SPATIAL BAND (a shoreline strip, a snow-line crossing)
// whose width sits below the spacing the color is sampled at: when the band
// is much narrower than the spacing, at most one sample in a strip lands
// inside it and the blended result aliases into isolated off-color speckle
// (dark coastal fringes, cloud-like snow patches on a globe view). The
// weight fades out once the band drops under half the spacing and reaches
// full strength once it spans 1.5 spacings. spacingMeters <= 0 (raw
// sampling, no spacing knowledge) keeps full detail.
double bandBelowSpacingFade(double bandWidthMeters, double spacingMeters)
{
    if (spacingMeters <= 0.0)
    {
        return 1.0;
    }
    return smoothstep(0.5 * spacingMeters, 1.5 * spacingMeters, bandWidthMeters);
}

// Linear blend of two double RGB arrays into `out` (component-wise, weight in
// [0, 1]; safe in-place - each channel is computed independently).
void blendColors(const double *rowA, const double *rowB, double weight, double out[3])
{
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        out[channel] = rowA[channel] + (rowB[channel] - rowA[channel]) * weight;
    }
}

// Amplitude and wet-belt geometry of the latitude moisture bands (ITCZ wet
// belt, subtropical dry belts, temperate westerlies, polar highs). Central
// values follow the classic atmospheric circulation cells; the amplitudes
// tune how strongly each belt shifts the humidity field.
struct MoistureBelt
{
    double amplitude;
    double centerDegrees;
    double widthDegrees;
};

constexpr MoistureBelt kMoistureBelts[] = {
    {+0.20, 0.0, 11.0},   // ITCZ
    {-0.26, 26.0, 10.0},  // subtropical highs
    {+0.10, 54.0, 13.0},  // temperate westerlies
    {-0.06, 88.0, 10.0},  // polar highs
};

constexpr double kPi = 3.14159265358979323846;

} // namespace

ClimateSampler::ClimateSampler(const PlanetGenerationParams &params)
    : params_(params)
    , seedTemperature_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x51A7E0)))
    , seedMoisture_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x40157)))
    , seedMoistureWarp_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x3A9E51)))
    , seedRockStripes_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x572A7A)))
{
    const auto streamSeed = [](std::int64_t base, std::uint64_t tag)
    { return static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(base), tag)); };
    seedTemperatureOctaves_  = makeFbmSeeds(seedTemperature_, 3);
    seedMoistureOctaves_     = makeFbmSeeds(seedMoisture_, 4);
    seedJitterOctaves_       = makeFbmSeeds(streamSeed(seedMoisture_, 0xB10B), 2);
    seedDetailJitterOctaves_ = makeFbmSeeds(streamSeed(seedMoisture_, 0xB10C), 2);
    seedGroundJitterOctaves_ = makeFbmSeeds(streamSeed(seedMoisture_, 0xB10D), 2);
    for (std::uint32_t octave = 0; octave < 3; ++octave)
    {
        seedMoistureWarpOctaves_[octave] = streamSeed(seedMoistureWarp_, octave);
    }
}

double ClimateSampler::seaLevelTemperature(const Vec3 &direction) const
{
    // Latitude from the configured spin axis: 0 at the equator, pi/2 at the
    // poles (absolute value - climate is hemispherically symmetric).
    const double absSinLatitude = clamp01(std::abs(dot(direction, Vec3{params_.axis[0], params_.axis[1], params_.axis[2]})));

    // Temperature: equator-to-pole gradient plus a wandering noise field so
    // climate bands are not perfect circles.
    double temperature = params_.equatorTemperature
                       + (params_.poleTemperature - params_.equatorTemperature) * absSinLatitude;
    const Vec3 surfacePosition = direction * params_.radiusMeters;
    temperature += params_.temperaturePerturbation
                 * fbm(seedTemperatureOctaves_, surfacePosition, params_.temperatureWavelengthMeters,
                     FbmParams{3, 2.0, 0.5});
    return clamp01(temperature);
}

ClimateSample ClimateSampler::sampleSeaLevel(const Vec3 &direction) const
{
    ClimateSample sample;

    sample.temperature01 = seaLevelTemperature(direction);

    // Moisture: base humidity noise plus the latitude belts (ITCZ wet belt,
    // subtropical dry belts, mild temperate wet belt). The humidity field is
    // domain-warped - sampling a bent position turns the round fbm blobs into
    // the elongated, organic shapes real rainfall systems follow (unwarped,
    // biome patches read as camouflage spots on the terrain).
    const Vec3 surfacePosition = direction * params_.radiusMeters;
    Vec3 moisturePosition = surfacePosition;
    if (params_.moistureWarpAmplitude > 0.0)
    {
        const double warpScale = 1.0 / std::max(1.0, params_.moistureWavelengthMeters * 1.6);
        const double warpX = noise::simplex3ImproveXY(seedMoistureWarpOctaves_[0],
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        const double warpY = noise::simplex3ImproveXY(seedMoistureWarpOctaves_[1],
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        const double warpZ = noise::simplex3ImproveXY(seedMoistureWarpOctaves_[2],
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        moisturePosition = surfacePosition
            + params_.moistureWarpAmplitude * params_.moistureWavelengthMeters * Vec3{warpX, warpY, warpZ};
    }
    double moisture = 0.5 + params_.moisturePerturbation * fbm(seedMoistureOctaves_, moisturePosition, params_.moistureWavelengthMeters,
        FbmParams{4, 2.0, 0.5});
    if (params_.moistureLatitudeBands)
    {
        const double latitudeDegrees = std::asin(clamp01(std::abs(dot(direction, Vec3{params_.axis[0], params_.axis[1], params_.axis[2]})))) * (180.0 / kPi);
        for (const MoistureBelt &belt : kMoistureBelts)
        {
            moisture += belt.amplitude * latitudeBelt(latitudeDegrees, belt.centerDegrees, belt.widthDegrees);
        }
    }
    sample.moisture01 = clamp01(moisture);
    return sample;
}

ClimateSample ClimateSampler::sample(const Vec3 &direction, double radiusMeters, double slope01, double lowPassCutoffMeters) const
{
    const double  seaRadius = params_.derivedSeaRadiusMeters;
    const double  elevation = radiusMeters - seaRadius;

    // Underwater samples (the sea floor and the water surface below the
    // wave band) classify and color by DEPTH alone: the temperature and
    // moisture fields only shape the sample through the air above the
    // waterline, so the expensive sea-level climate evaluation is skipped
    // and the returned fields carry the latitude gradient without the
    // perturbation noise. Air samples (elevation >= 0) keep the full field.
    ClimateSample sample;
    if (elevation < 0.0)
    {
        const double absSinLatitude = clamp01(std::abs(dot(direction, Vec3{params_.axis[0], params_.axis[1], params_.axis[2]})));
        sample.temperature01 = clamp01(params_.equatorTemperature
            + (params_.poleTemperature - params_.equatorTemperature) * absSinLatitude);
        sample.moisture01 = 1.0;
    }
    else
    {
        sample = sampleSeaLevel(direction);
        // Altitude lapse rate cools the air above sea level.
        sample.temperature01 = clamp01(sample.temperature01 - elevation / std::max(1.0, params_.temperatureLapseMeters));

        // Coastal humidity: onshore winds keep low land near the sea humid, so
        // the flat tapered shoreline belt stays green instead of painting a beige
        // desert fringe around every landmass. Fades out over the band width.
        if (params_.coastalMoistureBoost > 0.0)
        {
            const double band = std::max(1.0, params_.coastalMoistureBandMeters);
            sample.moisture01 = clamp01(sample.moisture01
                + params_.coastalMoistureBoost * (1.0 - smoothstep(0.0, band, elevation)));
        }
    }

    const auto paletteRow = [this](Biome biome) -> const float *
    { return params_.biomePalette + static_cast<std::size_t>(biome) * 3u; };

    // Water biomes by depth; the floor keeps a faint depth tint in the
    // palette rows so shallow shelves read sandy through the water tint.
    if (elevation < 0.0)
    {
        const double depth = -elevation;
        if (depth > params_.deepWaterMeters)
        {
            sample.biome = Biome::DeepOcean;
        }
        else if (depth > params_.shallowWaterMeters)
        {
            sample.biome = Biome::Ocean;
        }
        else
        {
            sample.biome = Biome::ShallowOcean;
        }

        // The floor color blends continuously through the depth bands (the
        // band ids above stay discrete for consumers); hard switches would
        // paint stepped rings onto every shelf and basin.
        const double shallowWidth = std::max(1.0, params_.shallowWaterMeters * params_.elevationBlendFraction);
        const double deepWidth    = std::max(1.0, params_.deepWaterMeters * params_.elevationBlendFraction);
        double shallowRow[3];
        double oceanRow[3];
        double deepRow[3];
        double blended[3];
        rowOf(paletteRow(Biome::ShallowOcean), shallowRow);
        rowOf(paletteRow(Biome::Ocean), oceanRow);
        rowOf(paletteRow(Biome::DeepOcean), deepRow);
        blendColors(shallowRow, oceanRow,
            smoothstep(params_.shallowWaterMeters - shallowWidth, params_.shallowWaterMeters + shallowWidth, depth), blended);
        blendColors(blended, deepRow,
            smoothstep(params_.deepWaterMeters - deepWidth, params_.deepWaterMeters + deepWidth, depth), blended);
        for (std::size_t channel = 0; channel < 3; ++channel)
        {
            sample.color[channel] = static_cast<float>(blended[channel]);
        }
        return sample;
    }

    const double t = sample.temperature01;
    const double m = sample.moisture01;

    // ---- biome id (discrete classification, unchanged) ------------------------
    // Snow: altitude snow line drops as the climate cools toward the
    // poles; hard freeze line overrides the altitude rule.
    const double snowLine = params_.snowLineMeters * (0.25 + 0.75 * t);
    const bool   snowy    = elevation > snowLine || t < 0.14;

    if (snowy && slope01 < params_.rockSlopeThreshold + 0.2)
    {
        sample.biome = Biome::Snow;
    }
    else if (slope01 > params_.rockSlopeThreshold)
    {
        // Steep faces scrape off soil and snow: bare rock.
        sample.biome = Biome::Rock;
    }
    else if (elevation < params_.beachWidthMeters && t > 0.25)
    {
        sample.biome = Biome::Beach;
    }
    else if (t > 0.75)
    {
        sample.biome = m < 0.30 ? Biome::Desert
                     : m < 0.58 ? Biome::Savanna
                                : Biome::TropicalRainforest;
    }
    else if (t > 0.50)
    {
        sample.biome = m < 0.24 ? Biome::Desert
                     : m < 0.44 ? Biome::Savanna
                     : m < 0.62 ? Biome::Grassland
                     : m < 0.80 ? Biome::SeasonalForest
                                : Biome::TemperateRainforest;
    }
    else if (t > 0.30)
    {
        sample.biome = m < 0.22 ? Biome::Shrubland
                     : m < 0.52 ? Biome::Grassland
                     : m < 0.78 ? Biome::TemperateForest
                                : Biome::TemperateRainforest;
    }
    else if (t > 0.14)
    {
        sample.biome = m < 0.32 ? Biome::Tundra : Biome::Taiga;
    }
    else
    {
        sample.biome = Biome::Tundra;
    }

    // ---- albedo (continuous blend between the same palette rows) --------------
    // Biome colors interpolate instead of hard-switching: per-vertex switches
    // smear across triangles into pale stains. The thresholds are exactly the
    // classification thresholds above; only the switching became blending.
    const double tBlend = params_.biomeBlendWidth;
    const double mBlend = params_.biomeBlendWidth;

    // Moisture blend inside one temperature band: start from the first row,
    // then fold in the next row at each threshold (smoothed by mBlend).
    struct Segment
    {
        double threshold;
        Biome  biome;
    };
    const auto moistureBlend = [&](const Segment *segments, std::size_t count, double out[3])
    {
        rowOf(paletteRow(segments[0].biome), out);
        for (std::size_t index = 1; index < count; ++index)
        {
            double row[3];
            rowOf(paletteRow(segments[index].biome), row);
            blendColors(out, row,
                smoothstep(segments[index].threshold - mBlend, segments[index].threshold + mBlend, m), out);
        }
    };

    const Segment coldSegments[] = {{0.0, Biome::Tundra}};
    const Segment coolSegments[] = {{0.0, Biome::Tundra}, {0.32, Biome::Taiga}};
    const Segment mildSegments[] = {{0.0, Biome::Shrubland}, {0.22, Biome::Grassland}, {0.52, Biome::TemperateForest},
        {0.78, Biome::TemperateRainforest}};
    const Segment warmSegments[] = {{0.0, Biome::Desert}, {0.24, Biome::Savanna}, {0.44, Biome::Grassland},
        {0.62, Biome::SeasonalForest}, {0.80, Biome::TemperateRainforest}};
    const Segment hotSegments[]  = {{0.0, Biome::Desert}, {0.30, Biome::Savanna}, {0.58, Biome::TropicalRainforest}};

    double cold[3];
    double cool[3];
    double mild[3];
    double warm[3];
    double hot[3];
    moistureBlend(coldSegments, 1, cold);
    moistureBlend(coolSegments, 2, cool);
    moistureBlend(mildSegments, 4, mild);
    moistureBlend(warmSegments, 5, warm);
    moistureBlend(hotSegments, 3, hot);

    // Temperature blend across the bands (cold at the bottom, hot on top).
    // blendColors is safe in-place: each channel is computed independently.
    double color[3];
    blendColors(cold, cool, smoothstep(0.14 - tBlend, 0.14 + tBlend, t), color);
    blendColors(color, mild, smoothstep(0.30 - tBlend, 0.30 + tBlend, t), color);
    blendColors(color, warm, smoothstep(0.50 - tBlend, 0.50 + tBlend, t), color);
    blendColors(color, hot, smoothstep(0.75 - tBlend, 0.75 + tBlend, t), color);

    // Layered overrides, each fading over a blend width:
    // beach (warm, flat, within the shoreline band) under snow, snow under rock.
    const double sBlend     = params_.slopeBlendWidth;
    const double beachBand  = std::max(1.0, params_.beachWidthMeters * params_.elevationBlendFraction);
    // Sub-spacing fades: the beach strip and the wet swash rim are sharp
    // bands a few meters wide, and the altitude snow crossing is one over
    // the blend width. When a band is much narrower than the color's sample
    // spacing, keeping it would alias into speckle (dark coastal fringes,
    // cloud-like snow patches at globe scale); it fades out instead. The
    // freeze-line snow (a smooth temperature field) is not a narrow band and
    // stays.
    const double beachFade = bandBelowSpacingFade(params_.beachWidthMeters + beachBand, lowPassCutoffMeters);
    const double beachW    = beachFade
        * (1.0 - smoothstep(params_.beachWidthMeters - beachBand, params_.beachWidthMeters + beachBand, elevation))
        * smoothstep(0.25 - tBlend, 0.25 + tBlend, t)
        * (1.0 - smoothstep(params_.rockSlopeThreshold * 0.5, params_.rockSlopeThreshold, slope01));
    const double snowBand   = std::max(1.0, params_.snowLineMeters * params_.elevationBlendFraction);
    // Snow where the elevation climbs past the climate snow line (weight 0
    // below the band, 1 above it) or where the air passes the freeze line.
    const double snowW      = std::max(
        bandBelowSpacingFade(2.0 * snowBand, lowPassCutoffMeters)
            * smoothstep(snowLine - snowBand, snowLine + snowBand, elevation),
        1.0 - smoothstep(0.14 - tBlend, 0.14 + tBlend, t));
    const double rockW      = smoothstep(params_.rockSlopeThreshold - sBlend, params_.rockSlopeThreshold + sBlend, slope01);

    double beachRow[3];
    double snowRow[3];
    double rockRow[3];
    rowOf(paletteRow(Biome::Beach), beachRow);
    rowOf(paletteRow(Biome::Snow), snowRow);
    rowOf(paletteRow(Biome::Rock), rockRow);
    // Wet sand: the swash band right above the waterline darkens and
    // saturates (real beaches show a sharp wet rim where the last wave
    // reached); fades over the configured width and never overrides snow or
    // rock. Applied to the beach palette row BEFORE it blends into the base
    // color, so the rim sits inside the sand band. The rim fades out with
    // the beach band below the sample spacing (a wet rim is a sub-meter
    // feature at coarse spacings, where it would alias into dark speckle).
    if (params_.beachWetDarkening > 0.0 && params_.beachWetWidthMeters > 0.0)
    {
        const double wetW = bandBelowSpacingFade(params_.beachWetWidthMeters, lowPassCutoffMeters)
            * (1.0 - smoothstep(0.0, params_.beachWetWidthMeters, elevation));
        if (wetW > 0.0)
        {
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                // Darken toward a saturated wet tone: multiply toward zero
                // but keep a little of the hue (sand darkens, not vanishes).
                beachRow[channel] *= 1.0 - params_.beachWetDarkening * wetW;
            }
        }
    }
    if (rockW > 0.0 && params_.rockStrataContrast > 0.0)
    {
        // Sediment stripes on rock faces: a luminance band keyed to the strata
        // band spacing (the same spacing that folds the geometric strata
        // terms), so cliff walls read as layered sediment instead of flat
        // gray sheets. The band phase follows the body's shell like the
        // geometric strata do, and a low-frequency phase warp varies the
        // band THICKNESS the way real sedimentary beds vary (a perfectly
        // periodic band reads as a brick wall); the warp shares its seed and
        // scale with the geometric term's thickness warp, so the color
        // stripes track the geometric ridges. The same field gates the
        // stripe AMPLITUDE (bedding strength): outcrops alternate between
        // strongly banded and almost plain rock the way the geometric beds
        // do.
        const double bandWidth = std::max(1.0, params_.strataBandMeters);
        const Vec3   stripePosition = direction * params_.radiusMeters;
        const double thicknessWarp = noise::simplex3ImproveXY(seedRockStripes_,
            stripePosition.x / (bandWidth * 5.0), stripePosition.y / (bandWidth * 5.0),
            stripePosition.z / (bandWidth * 5.0));
        const double strataPhase = 6.283185307179586 * (radiusMeters / bandWidth)
            + params_.strataWarp * 2.6 * thicknessWarp;
        const double bedding = 0.30 + 0.70 * smoothstep(-0.55, 0.55, thicknessWarp);
        const double stripe = 1.0 + params_.rockStrataContrast * bedding * std::sin(strataPhase);
        for (std::size_t channel = 0; channel < 3; ++channel)
        {
            rockRow[channel] = std::clamp(rockRow[channel] * stripe, 0.0, 1.0);
        }
    }
    blendColors(color, beachRow, beachW * (1.0 - snowW), color);
    blendColors(color, snowRow, snowW, color);
    blendColors(color, rockRow, rockW, color);

    // Albedo: jittered by a per-point variation field so large biomes do not
    // read as flat plastic sheets. The field carries THREE scales - the base
    // wavelength mottles biome sheets at globe scale, an octave at an eighth
    // of that wavelength covers the mid-range, and an octave at the ground
    // wavelength keeps ground-level views (a walking camera sees metres)
    // from reading as one unbroken color. Each scale is a pure function of
    // direction (order independent), and the channel weighting turns them
    // into subtle hue mottling (warm/cool patches) instead of pure luminance
    // banding. Each scale fades under the optional band-limit cutoff like
    // every other noise term a rendered chunk samples, so coarse LOD chunks
    // neither spend the evaluations nor alias sub-voxel mottle as speckle.
    const Vec3 surfacePosition = direction * params_.radiusMeters;
    const double baseWavelength = std::max(1.0, params_.colorVariationWavelengthMeters);
    const double groundWavelength = std::max(1.0, params_.colorVariationGroundWavelengthMeters);
    double variation = 0.0;
    if (params_.colorVariation > 0.0)
    {
        const double baseFade = lowPassFade(lowPassCutoffMeters, baseWavelength);
        const double detailFade = lowPassFade(lowPassCutoffMeters, baseWavelength / 8.0);
        const double groundFade = lowPassFade(lowPassCutoffMeters, groundWavelength);
        if (baseFade > 0.0)
        {
            variation += 0.52 * baseFade
                * fbm(seedJitterOctaves_, surfacePosition, baseWavelength, FbmParams{2, 2.0, 0.5});
        }
        if (detailFade > 0.0)
        {
            variation += 0.22 * detailFade
                * fbm(seedDetailJitterOctaves_, surfacePosition, baseWavelength / 8.0, FbmParams{2, 2.0, 0.5});
        }
        if (groundFade > 0.0)
        {
            variation += 0.26 * groundFade
                * fbm(seedGroundJitterOctaves_, surfacePosition, groundWavelength, FbmParams{2, 2.0, 0.5});
        }
        variation *= params_.colorVariation;
    }
    constexpr double kChannelWeights[3] = {1.0, 0.85, 0.70};
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        sample.color[channel] = static_cast<float>(clamp01(color[channel] * (1.0 + variation * kChannelWeights[channel])));
    }
    return sample;
}

} // namespace planet
} // namespace engine
