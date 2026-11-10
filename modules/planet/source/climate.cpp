// Climate and biome classification (see climate.hpp).

#include <engine/planet/climate.hpp>

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

// Linear blend of two double RGB arrays into `out` (component-wise, weight in
// [0, 1]; safe in-place - each channel is computed independently).
void blendColors(const double *rowA, const double *rowB, double weight, double out[3])
{
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        out[channel] = rowA[channel] + (rowB[channel] - rowA[channel]) * weight;
    }
}

// fbm in [-1, 1] sampled at a PHYSICAL position (meters) over a metric
// wavelength, so climate features keep their size at any planet radius.
double sphereFbm(std::int64_t seed, const Vec3 &surfacePosition, double wavelengthMeters, std::uint32_t octaves)
{
    const double frequency = 1.0 / std::max(1.0, wavelengthMeters);
    double       x         = surfacePosition.x * frequency;
    double       y         = surfacePosition.y * frequency;
    double       z         = surfacePosition.z * frequency;
    double       amplitude = 1.0;
    double       sum       = 0.0;
    double       norm      = 0.0;
    for (std::uint32_t octave = 0; octave < octaves; ++octave)
    {
        sum += amplitude * noise::simplex3ImproveXY(hashCombine64(seed, octave), x, y, z);
        norm += amplitude;
        x *= 2.0;
        y *= 2.0;
        z *= 2.0;
        amplitude *= 0.5;
    }
    return sum / std::max(norm, 1.0e-9);
}

} // namespace

ClimateSampler::ClimateSampler(const PlanetGenerationParams &params)
    : params_(params)
    , seedTemperature_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x51A7E0)))
    , seedMoisture_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x40157)))
    , seedMoistureWarp_(static_cast<std::int64_t>(hashCombine64(params.seed, 0x3A9E51)))
{
}

ClimateSample ClimateSampler::sampleSeaLevel(const Vec3 &direction) const
{
    ClimateSample sample;

    // Latitude from the configured spin axis: 0 at the equator, pi/2 at the
    // poles (absolute value - climate is hemispherically symmetric).
    const double absSinLatitude = clamp01(std::abs(dot(direction, Vec3{params_.axis[0], params_.axis[1], params_.axis[2]})));
    const double latitudeDegrees = std::asin(absSinLatitude) * (180.0 / 3.14159265358979323846);

    // Temperature: equator-to-pole gradient plus a wandering noise field so
    // climate bands are not perfect circles.
    double temperature = params_.equatorTemperature
                       + (params_.poleTemperature - params_.equatorTemperature) * absSinLatitude;
    const Vec3 surfacePosition = direction * params_.radiusMeters;
    temperature += params_.temperaturePerturbation
                 * sphereFbm(seedTemperature_, surfacePosition, params_.temperatureWavelengthMeters, 3);
    sample.temperature01 = clamp01(temperature);

    // Moisture: base humidity noise plus the latitude belts (ITCZ wet belt,
    // subtropical dry belts, mild temperate wet belt). The humidity field is
    // domain-warped - sampling a bent position turns the round fbm blobs into
    // the elongated, organic shapes real rainfall systems follow (unwarped,
    // biome patches read as camouflage spots on the terrain).
    Vec3 moisturePosition = surfacePosition;
    if (params_.moistureWarpAmplitude > 0.0)
    {
        const double warpScale = 1.0 / std::max(1.0, params_.moistureWavelengthMeters * 1.6);
        const double warpX = noise::simplex3ImproveXY(hashCombine64(seedMoistureWarp_, 0),
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        const double warpY = noise::simplex3ImproveXY(hashCombine64(seedMoistureWarp_, 1),
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        const double warpZ = noise::simplex3ImproveXY(hashCombine64(seedMoistureWarp_, 2),
            surfacePosition.x * warpScale, surfacePosition.y * warpScale, surfacePosition.z * warpScale);
        moisturePosition = surfacePosition
            + params_.moistureWarpAmplitude * params_.moistureWavelengthMeters * Vec3{warpX, warpY, warpZ};
    }
    double moisture = 0.5 + params_.moisturePerturbation * sphereFbm(seedMoisture_, moisturePosition, params_.moistureWavelengthMeters, 4);
    if (params_.moistureLatitudeBands)
    {
        moisture += 0.20 * latitudeBelt(latitudeDegrees, 0.0, 11.0);   // ITCZ
        moisture -= 0.26 * latitudeBelt(latitudeDegrees, 26.0, 10.0);  // subtropical highs
        moisture += 0.10 * latitudeBelt(latitudeDegrees, 54.0, 13.0);  // temperate westerlies
        moisture -= 0.06 * latitudeBelt(latitudeDegrees, 88.0, 10.0);  // polar highs
    }
    sample.moisture01 = clamp01(moisture);
    return sample;
}

ClimateSample ClimateSampler::sample(const Vec3 &direction, double radiusMeters, double slope01) const
{
    ClimateSample sample   = sampleSeaLevel(direction);
    const double  seaRadius = params_.derivedSeaRadiusMeters;
    const double  elevation = radiusMeters - seaRadius;

    // Altitude lapse rate cools the air above sea level (warms below it,
    // mildly - ocean basins stay temperate).
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
    const double beachW     = (1.0 - smoothstep(params_.beachWidthMeters - beachBand, params_.beachWidthMeters + beachBand, elevation))
        * smoothstep(0.25 - tBlend, 0.25 + tBlend, t)
        * (1.0 - smoothstep(params_.rockSlopeThreshold * 0.5, params_.rockSlopeThreshold, slope01));
    const double snowBand   = std::max(1.0, params_.snowLineMeters * params_.elevationBlendFraction);
    // Snow where the elevation climbs past the climate snow line (weight 0
    // below the band, 1 above it) or where the air passes the freeze line.
    const double snowW      = std::max(
        smoothstep(snowLine - snowBand, snowLine + snowBand, elevation),
        1.0 - smoothstep(0.14 - tBlend, 0.14 + tBlend, t));
    const double rockW      = smoothstep(params_.rockSlopeThreshold - sBlend, params_.rockSlopeThreshold + sBlend, slope01);

    double beachRow[3];
    double snowRow[3];
    double rockRow[3];
    rowOf(paletteRow(Biome::Beach), beachRow);
    rowOf(paletteRow(Biome::Snow), snowRow);
    rowOf(paletteRow(Biome::Rock), rockRow);
    if (rockW > 0.0 && params_.rockStrataContrast > 0.0)
    {
        // Sediment stripes on rock faces: a luminance band keyed to the strata
        // band spacing (the same spacing that folds the geometric strata
        // terms), so cliff walls read as layered sediment instead of flat
        // gray sheets. The band phase follows the body's shell like the
        // geometric strata do.
        const double strataPhase = 6.283185307179586 * (radiusMeters / std::max(1.0, params_.strataBandMeters));
        const double stripe = 1.0 + params_.rockStrataContrast * std::sin(strataPhase);
        for (std::size_t channel = 0; channel < 3; ++channel)
        {
            rockRow[channel] = std::clamp(rockRow[channel] * stripe, 0.0, 1.0);
        }
    }
    blendColors(color, beachRow, beachW * (1.0 - snowW), color);
    blendColors(color, snowRow, snowW, color);
    blendColors(color, rockRow, rockW, color);

    // Albedo: jittered by a per-point luminance variation so large biomes do
    // not read as flat plastic sheets. The jitter noise is a pure function of
    // direction (order independent).
    const double jitter
        = 1.0 + params_.colorVariation * sphereFbm(hashCombine64(seedMoisture_, 0xB10B), direction * params_.radiusMeters, 260.0, 2);
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        sample.color[channel] = static_cast<float>(clamp01(color[channel] * jitter));
    }
    return sample;
}

} // namespace planet
} // namespace engine
