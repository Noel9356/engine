// PlanetGenerationParams::validate - clamps every configured field into its
// documented range and derives the aggregate values (root cube, octree depth,
// shell band, sea radius) the rest of the module relies on. See params.hpp
// for the meaning of each field.

#include <cstddef>
#include <cstdint>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace engine
{
namespace planet
{
namespace
{

double clampRange(double value, double low, double high)
{
    if (!std::isfinite(value))
    {
        return std::clamp(0.5 * (low + high), low, high);
    }
    return std::clamp(value, low, high);
}

} // namespace

bool PlanetGenerationParams::validate(std::string &error)
{
    // ---- identity and scale ---------------------------------------------------
    if (!std::isfinite(radiusMeters) || radiusMeters < 1.0 || radiusMeters > 1.0e9)
    {
        error = "planet: radius_meters must be finite and within [1, 1e9]";
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(centerMeters[axis]))
        {
            error = "planet: center_meters entries must be finite";
            return false;
        }
    }
    const double axisLength = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (!std::isfinite(axisLength) || axisLength < 1.0e-6)
    {
        axis[0] = 0.0;
        axis[1] = 1.0;
        axis[2] = 0.0;
    }
    else
    {
        axis[0] /= axisLength;
        axis[1] /= axisLength;
        axis[2] /= axisLength;
    }

    // ---- octree / LOD ----------------------------------------------------------
    leafGridN     = static_cast<std::uint32_t>(clampRange(static_cast<double>(leafGridN), 4.0, 64.0));
    // Chunk faces must sit on coarse-lattice planes (the conforming boundary
    // sampling assumes it), so the grid resolution has to be even.
    if (leafGridN % 2u != 0u)
    {
        leafGridN -= 1u;
    }
    // Finest-voxel floor: the chunk key packs a 4-bit depth and 20-bit
    // lattice coordinates per axis (see types.hpp), so the octree depth is
    // capped at 15 and the finest grid holds 2^15 chunks per root axis
    // (2^depth * leafGridN lattice steps). With leafGridN <= 64 that bounds
    // voxels at rootSize / 2^19; deeper refinement than the cap allows is
    // silently clamped.
    targetVoxelMeters = clampRange(targetVoxelMeters, radiusMeters / 131072.0, radiusMeters / 4.0);
    maxDepthLimit = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxDepthLimit), 1.0, static_cast<double>(maxSupportedOctreeDepth)));
    lodAngularVoxelRadians = clampRange(lodAngularVoxelRadians, 0.0005, 0.5);
    lodBandLimitFactor = clampRange(lodBandLimitFactor, 1.0, 4.0);
    boundingFactor    = clampRange(boundingFactor, 1.05, 4.0);
    maxConcurrentBuilds  = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxConcurrentBuilds), 1.0, 64.0));
    maxNewBuildsPerUpdate = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxNewBuildsPerUpdate), 1.0, 256.0));
    maxLivePlanets        = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxLivePlanets), 1.0, 4096.0));
    chunkEvictionUpdates  = static_cast<std::uint32_t>(clampRange(static_cast<double>(chunkEvictionUpdates), 2.0, 100000.0));
    chunkCacheCapacity    = static_cast<std::uint32_t>(clampRange(static_cast<double>(chunkCacheCapacity), 16.0, 65536.0));

    // ---- relief -----------------------------------------------------------------
    continentAmplitudeMeters  = clampRange(continentAmplitudeMeters, 1.0, 0.25 * radiusMeters);
    continentWavelengthMeters = clampRange(continentWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 32.0);
    continentsFbm.octaves = static_cast<std::uint32_t>(clampRange(static_cast<double>(continentsFbm.octaves), 1.0, 12.0));
    continentsFbm.lacunarity = clampRange(continentsFbm.lacunarity, 1.0, 4.0);
    continentsFbm.gain       = clampRange(continentsFbm.gain, 0.1, 0.9);

    domainWarpAmplitude       = clampRange(domainWarpAmplitude, 0.0, 4.0);
    domainWarpWavelengthMeters = clampRange(domainWarpWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 32.0);

    mountainAmplitudeMeters  = clampRange(mountainAmplitudeMeters, 0.0, 0.25 * radiusMeters);
    mountainWavelengthMeters = clampRange(mountainWavelengthMeters, radiusMeters / 4096.0, radiusMeters * 8.0);
    mountainMaskLow          = clampRange(mountainMaskLow, -1.0, 0.95);
    mountainMaskHigh         = clampRange(mountainMaskHigh, mountainMaskLow + 0.01, 1.0);
    mountainSharpness        = clampRange(mountainSharpness, 0.25, 8.0);
    mountainsFbm.octaves     = static_cast<std::uint32_t>(clampRange(static_cast<double>(mountainsFbm.octaves), 1.0, 12.0));
    mountainsFbm.lacunarity  = clampRange(mountainsFbm.lacunarity, 1.0, 4.0);
    mountainsFbm.gain        = clampRange(mountainsFbm.gain, 0.1, 0.9);

    mountainMassFraction         = clampRange(mountainMassFraction, 0.0, 1.0);
    rangeClusterStrength         = clampRange(rangeClusterStrength, 0.0, 1.0);
    rangeClusterWavelengthMeters = clampRange(rangeClusterWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 32.0);

    regionalAmplitudeMeters  = clampRange(regionalAmplitudeMeters, 0.0, 0.1 * radiusMeters);
    regionalWavelengthMeters = clampRange(regionalWavelengthMeters, radiusMeters / 8192.0, radiusMeters * 4.0);
    regionalFbm.octaves      = static_cast<std::uint32_t>(clampRange(static_cast<double>(regionalFbm.octaves), 1.0, 12.0));
    regionalFbm.lacunarity   = clampRange(regionalFbm.lacunarity, 1.0, 4.0);
    regionalFbm.gain         = clampRange(regionalFbm.gain, 0.1, 0.9);

    hillAmplitudeMeters  = clampRange(hillAmplitudeMeters, 0.0, 0.1 * radiusMeters);
    hillWavelengthMeters = clampRange(hillWavelengthMeters, radiusMeters / 8192.0, radiusMeters * 4.0);
    hillsFbm.octaves     = static_cast<std::uint32_t>(clampRange(static_cast<double>(hillsFbm.octaves), 1.0, 12.0));
    hillsFbm.lacunarity  = clampRange(hillsFbm.lacunarity, 1.0, 4.0);
    hillsFbm.gain        = clampRange(hillsFbm.gain, 0.1, 0.9);

    detailAmplitudeMeters  = clampRange(detailAmplitudeMeters, 0.0, 0.02 * radiusMeters);
    detailWavelengthMeters = clampRange(detailWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    detailPlainsFraction   = clampRange(detailPlainsFraction, 0.0, 1.0);
    detailFbm.octaves      = static_cast<std::uint32_t>(clampRange(static_cast<double>(detailFbm.octaves), 1.0, 12.0));
    detailFbm.lacunarity   = clampRange(detailFbm.lacunarity, 1.0, 4.0);
    detailFbm.gain         = clampRange(detailFbm.gain, 0.1, 0.9);

    terraceStrength       = clampRange(terraceStrength, 0.0, 1.0);
    terraceSteps          = clampRange(terraceSteps, 1.0, 64.0);
    terraceRiserFraction = clampRange(terraceRiserFraction, 0.04, 0.5);
    cliffWavelengthMeters = clampRange(cliffWavelengthMeters, radiusMeters / 8192.0, radiusMeters * 4.0);
    cliffSteepnessHigh    = clampRange(cliffSteepnessHigh, 0.05, 1.0);
    cliffSteepnessLow     = clampRange(cliffSteepnessLow, 0.01, cliffSteepnessHigh - 0.02);
    cliffSharpness        = clampRange(cliffSharpness, 0.5, 8.0);

    overhangAmplitudeMeters  = clampRange(overhangAmplitudeMeters, 0.0, 0.05 * radiusMeters);
    overhangWavelengthMeters = clampRange(overhangWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    overhangPlainsFraction   = clampRange(overhangPlainsFraction, 0.0, 1.0);
    strataAmplitudeMeters    = clampRange(strataAmplitudeMeters, 0.0, 0.01 * radiusMeters);
    strataBandMeters         = clampRange(strataBandMeters, radiusMeters / 16384.0, radiusMeters);
    strataWarp               = clampRange(strataWarp, 0.0, 4.0);

    coastalFlattenMeters = clampRange(coastalFlattenMeters, 0.0, 0.1 * radiusMeters);
    coastalFlattenFloor  = clampRange(coastalFlattenFloor, 0.0, 1.0);

    caveThreshold           = clampRange(caveThreshold, 0.01, 0.6);
    caveWormWavelengthMeters = clampRange(caveWormWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    caveCarveMeters         = clampRange(caveCarveMeters, 0.0, 0.02 * radiusMeters);

    // ---- oceans / climate --------------------------------------------------------
    shallowWaterMeters   = clampRange(shallowWaterMeters, 0.0, radiusMeters);
    deepWaterMeters      = clampRange(deepWaterMeters, shallowWaterMeters, radiusMeters);
    // The deepened floor must stay a surface feature: an abyss deeper than a
    // small fraction of the radius would sink the sea floor past the inner
    // shell the octree prunes against (and, on tiny planets, through the
    // core), so the depth scales with the body. Clamped BEFORE the shell
    // relief aggregation below reads it - the aggregation feeds the octree's
    // pruning band and the surface scans, which must never see a value the
    // clamp would reject.
    abyssalDepthMeters   = clampRange(abyssalDepthMeters, 0.0, 0.08 * radiusMeters);

    // The aggregate relief band every generation term can reach. The ocean
    // surface has to stay inside it: the octree's sea-straddle exemption and
    // the mesher's water-pass placement both assume the sea sphere lies
    // within the shell band the tree prunes against, so a sea level offset
    // beyond the relief amplitudes would strand the ocean outside every
    // candidate node (an invisible ocean).
    const double shellReliefMeters = continentAmplitudeMeters + regionalAmplitudeMeters + mountainAmplitudeMeters
        + hillAmplitudeMeters + detailAmplitudeMeters + overhangAmplitudeMeters + strataAmplitudeMeters
        + caveCarveMeters + abyssalDepthMeters;
    seaLevelOffsetMeters = clampRange(seaLevelOffsetMeters, -shellReliefMeters, shellReliefMeters);

    equatorTemperature = clampRange(equatorTemperature, 0.0, 1.0);
    poleTemperature    = clampRange(poleTemperature, 0.0, equatorTemperature);
    temperaturePerturbation = clampRange(temperaturePerturbation, 0.0, 1.0);
    temperatureWavelengthMeters = clampRange(temperatureWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 16.0);
    temperatureLapseMeters      = clampRange(temperatureLapseMeters, 1.0, radiusMeters);
    moistureWavelengthMeters    = clampRange(moistureWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 16.0);
    moisturePerturbation        = clampRange(moisturePerturbation, 0.0, 1.0);
    moistureWarpAmplitude       = clampRange(moistureWarpAmplitude, 0.0, 2.0);
    moistureBeltEquatorAmplitude     = clampRange(moistureBeltEquatorAmplitude, -1.0, 1.0);
    moistureBeltSubtropicalAmplitude = clampRange(moistureBeltSubtropicalAmplitude, -1.0, 1.0);
    moistureBeltTemperateAmplitude   = clampRange(moistureBeltTemperateAmplitude, -1.0, 1.0);
    moistureBeltPolarAmplitude       = clampRange(moistureBeltPolarAmplitude, -1.0, 1.0);
    coastalMoistureBoost        = clampRange(coastalMoistureBoost, 0.0, 1.0);
    coastalMoistureBandMeters   = clampRange(coastalMoistureBandMeters, 1.0, radiusMeters);
    colorVariation       = clampRange(colorVariation, 0.0, 1.0);
    colorVariationWavelengthMeters = clampRange(colorVariationWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    colorVariationGroundWavelengthMeters = clampRange(colorVariationGroundWavelengthMeters, radiusMeters / 16384.0, colorVariationWavelengthMeters);
    soilPatchStrength            = clampRange(soilPatchStrength, 0.0, 1.0);
    soilPatchWavelengthMeters    = clampRange(soilPatchWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    dryPatchStrength             = clampRange(dryPatchStrength, 0.0, 1.0);
    dryPatchWavelengthMeters     = clampRange(dryPatchWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    snowLineMeters       = clampRange(snowLineMeters, 0.0, 0.5 * radiusMeters);
    beachWidthMeters     = clampRange(beachWidthMeters, 0.0, 0.05 * radiusMeters);
    beachWetWidthMeters  = clampRange(beachWetWidthMeters, 0.0, 0.05 * radiusMeters);
    beachWetDarkening    = clampRange(beachWetDarkening, 0.0, 0.95);
    rockSlopeThreshold   = clampRange(rockSlopeThreshold, 0.05, 1.0);
    rockStrataContrast   = clampRange(rockStrataContrast, 0.0, 1.0);
    biomeBlendWidth        = clampRange(biomeBlendWidth, 0.005, 0.2);
    slopeBlendWidth        = clampRange(slopeBlendWidth, 0.01, 0.5);
    elevationBlendFraction = clampRange(elevationBlendFraction, 0.02, 1.0);

    for (std::size_t index = 0; index < biomeCount * 3; ++index)
    {
        biomePalette[index] = std::clamp(biomePalette[index], 0.0f, 1.0f);
    }

    // ---- surface walking -------------------------------------------------------
    surfaceGravityMetersPerSecond2 = clampRange(surfaceGravityMetersPerSecond2, 0.05, 100.0);
    walkSpeedMetersPerSecond       = clampRange(walkSpeedMetersPerSecond, 0.1, 1000.0);
    walkSprintMultiplier           = clampRange(walkSprintMultiplier, 1.0, 20.0);
    jumpSpeedMetersPerSecond       = clampRange(jumpSpeedMetersPerSecond, 0.0, 100.0);
    eyeHeightMeters                = clampRange(eyeHeightMeters, 0.1, 100.0);
    walkGroundSnapMeters           = clampRange(walkGroundSnapMeters, 0.0, 1000.0);
    walkMaxStepUpMeters            = clampRange(walkMaxStepUpMeters, 0.0, 100.0);
    walkMaxClimbGrade              = clampRange(walkMaxClimbGrade, 0.1, 100.0);
    swimSpeedMetersPerSecond       = clampRange(swimSpeedMetersPerSecond, 0.1, 1000.0);
    swimWadeDepthMeters            = clampRange(swimWadeDepthMeters, 0.0, 1000.0);

    // ---- atmosphere -------------------------------------------------------------
    atmosphereHeightMeters = clampRange(atmosphereHeightMeters, radiusMeters / 1024.0, radiusMeters / 2.0);
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        atmosphereZenithColor[channel]  = std::clamp(atmosphereZenithColor[channel], 0.0f, 1.0f);
        atmosphereHorizonColor[channel] = std::clamp(atmosphereHorizonColor[channel], 0.0f, 1.0f);
    }

    // ---- water --------------------------------------------------------------------
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        waterColorShallow[channel] = std::clamp(waterColorShallow[channel], 0.0f, 1.0f);
        waterColorDeep[channel]    = std::clamp(waterColorDeep[channel], 0.0f, 1.0f);
    }
    waterRoughness = std::clamp(waterRoughness, 0.0f, 1.0f);

    // ---- clouds --------------------------------------------------------------------
    cloudCoverageAmount    = clampRange(cloudCoverageAmount, 0.0, 1.0);
    cloudWavelengthMeters  = clampRange(cloudWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 4.0);
    cloudAltitudeMeters    = clampRange(cloudAltitudeMeters, radiusMeters / 8192.0, radiusMeters / 4.0);
    cloudDetailStrength    = clampRange(cloudDetailStrength, 0.0, 1.0);
    cloudEdgeSoftness      = clampRange(cloudEdgeSoftness, 0.01, 0.6);
    cloudWeatherStrength   = clampRange(cloudWeatherStrength, 0.0, 1.0);
    cloudWeatherWavelengthMultiple = clampRange(cloudWeatherWavelengthMultiple, 1.0, 16.0);
    cloudLatitudeBandStrength = clampRange(cloudLatitudeBandStrength, 0.0, 0.5);
    cloudBandFrequency     = clampRange(cloudBandFrequency, 0.0, 24.0);
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        cloudColor[channel] = std::clamp(cloudColor[channel], 0.0f, 1.0f);
    }
    cloudShadowStrength = clampRange(cloudShadowStrength, 0.0, 1.0);
    {
        const double sunLength = std::sqrt(cloudShadowSunDirection[0] * cloudShadowSunDirection[0]
            + cloudShadowSunDirection[1] * cloudShadowSunDirection[1]
            + cloudShadowSunDirection[2] * cloudShadowSunDirection[2]);
        if (!std::isfinite(sunLength) || sunLength < 1.0e-6)
        {
            cloudShadowSunDirection[0] = 0.0;
            cloudShadowSunDirection[1] = 1.0;
            cloudShadowSunDirection[2] = 0.0;
        }
        else
        {
            cloudShadowSunDirection[0] /= sunLength;
            cloudShadowSunDirection[1] /= sunLength;
            cloudShadowSunDirection[2] /= sunLength;
        }
    }
    starsDensity    = clampRange(starsDensity, 0.0, 1.0);
    starsBrightness = clampRange(starsBrightness, 0.0, 4.0);
    seaIceTemperature = clampRange(seaIceTemperature, 0.0, 1.0);
    seaIceTransition  = clampRange(seaIceTransition, 0.005, 0.5);
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        waterColorIce[channel] = std::clamp(waterColorIce[channel], 0.0f, 1.0f);
    }
    waterDepthTintMeters        = clampRange(waterDepthTintMeters, 1.0, radiusMeters);
    waterShoreMixMeters         = clampRange(waterShoreMixMeters, 0.0, radiusMeters);
    waterFoamWidthMeters        = clampRange(waterFoamWidthMeters, 0.0, 0.01 * radiusMeters);
    waterFoamStrength           = clampRange(waterFoamStrength, 0.0, 1.0);
    waterWaveAmplitude        = clampRange(waterWaveAmplitude, 0.0, 1.0);
    waterWaveWavelengthMeters = clampRange(waterWaveWavelengthMeters, radiusMeters / 16384.0, radiusMeters);

    // ---- derived values -------------------------------------------------------------
    const double relief = shellReliefMeters;
    derivedRootHalfMeters = radiusMeters * boundingFactor;
    derivedRootSizeMeters = 2.0 * derivedRootHalfMeters;

    // Deepest level whose leaf voxels still meet the target size, capped by
    // the configured limit and the 20-bit coordinate packing.
    std::uint32_t depth = 0;
    while (depth < maxDepthLimit && (derivedRootSizeMeters / std::ldexp(static_cast<double>(leafGridN), static_cast<int>(depth) + 1)) > targetVoxelMeters)
    {
        ++depth;
    }
    derivedMaxDepth        = depth;
    derivedLeafSizeMeters  = std::ldexp(derivedRootSizeMeters, -static_cast<int>(depth));
    derivedVoxelMeters     = derivedLeafSizeMeters / static_cast<double>(leafGridN);
    derivedMinShellRadius  = std::max(0.0, radiusMeters - relief);
    derivedMaxShellRadius  = radiusMeters + relief;
    derivedMaxReliefMeters = relief;
    derivedSeaRadiusMeters = radiusMeters + seaLevelOffsetMeters;
    // The broad mountain mass field rides at a fixed multiple of the crest
    // wavelength (see reliefImpl); derived here so the octree's gradient
    // bounds and the density field agree on one number.
    derivedMountainMassWavelengthMeters = std::max(1.0, mountainWavelengthMeters * 1.8);
    return true;
}

} // namespace planet
} // namespace engine
