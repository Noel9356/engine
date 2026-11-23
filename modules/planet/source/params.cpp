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
    // voxels at rootSize / 2^21 (rootSize / 2^19 at the leafGridN = 16
    // default); deeper refinement than the cap allows is silently clamped.
    targetVoxelMeters = clampRange(targetVoxelMeters, radiusMeters / 131072.0, radiusMeters / 4.0);
    selectionUpdateDistanceMeters = clampRange(selectionUpdateDistanceMeters, 0.0, radiusMeters * 0.1);
    maxDepthLimit = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxDepthLimit), 1.0, static_cast<double>(maxSupportedOctreeDepth)));
    lodAngularVoxelRadians = clampRange(lodAngularVoxelRadians, 0.0005, 0.5);
    lodBandLimitFactor = clampRange(lodBandLimitFactor, 1.0, 4.0);
    lodScaffoldBuildsPerUpdate = static_cast<std::uint32_t>(clampRange(static_cast<double>(lodScaffoldBuildsPerUpdate), 0.0, 256.0));
    boundingFactor    = clampRange(boundingFactor, 1.05, 4.0);
    maxConcurrentBuilds  = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxConcurrentBuilds), 1.0, 64.0));
    maxNewBuildsPerUpdate = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxNewBuildsPerUpdate), 1.0, 256.0));
    nodeRecordLifetimeUpdates = static_cast<std::uint32_t>(clampRange(static_cast<double>(nodeRecordLifetimeUpdates), 16.0, 65536.0));
    nodeRecordPruneIntervalUpdates = static_cast<std::uint32_t>(clampRange(static_cast<double>(nodeRecordPruneIntervalUpdates), 1.0, 4096.0));
    if (nodeRecordPruneIntervalUpdates > nodeRecordLifetimeUpdates)
    {
        nodeRecordPruneIntervalUpdates = nodeRecordLifetimeUpdates;
    }
    starvedInlineBuildIntervalUpdates = static_cast<std::uint32_t>(clampRange(static_cast<double>(starvedInlineBuildIntervalUpdates), 0.0, 4096.0));
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
    mountainMassWavelengthMultiple = clampRange(mountainMassWavelengthMultiple, 1.0, 8.0);
    mountainCrestMassCoupling    = clampRange(mountainCrestMassCoupling, 0.0, 1.0);
    rangeClusterStrength         = clampRange(rangeClusterStrength, 0.0, 1.0);
    rangeClusterWavelengthMeters = clampRange(rangeClusterWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 32.0);
    rangeClusterGateHigh = clampRange(rangeClusterGateHigh, rangeClusterGateLow + 0.02, 1.0);
    rangeClusterGateLow  = clampRange(rangeClusterGateLow, -1.0, rangeClusterGateHigh - 0.02);

    mountainErosionWeight         = clampRange(mountainErosionWeight, 0.0, 1.0);
    mountainErosionFaceBlendWidth = clampRange(mountainErosionFaceBlendWidth, 1.0e-4, 0.25);
    mountainErosionTerrainFrequency = clampRange(mountainErosionTerrainFrequency, 0.25, 64.0);
    mountainErosionTerrainAmplitude = clampRange(mountainErosionTerrainAmplitude, 0.0, 1.0);
    mountainErosionTerrainOctaves   = static_cast<std::uint32_t>(
        clampRange(static_cast<double>(mountainErosionTerrainOctaves), 1.0, static_cast<double>(maxFbmOctaves)));
    mountainErosionTerrainGain       = clampRange(mountainErosionTerrainGain, 0.01, 0.99);
    mountainErosionTerrainLacunarity = clampRange(mountainErosionTerrainLacunarity, 1.0, 4.0);
    mountainErosionHeightOffsetLow   = clampRange(mountainErosionHeightOffsetLow, -4.0, 4.0);
    mountainErosionHeightOffsetBlend = clampRange(mountainErosionHeightOffsetBlend, 0.0, 1.0);
    mountainErosionScale         = clampRange(mountainErosionScale, 0.01, 8.0);
    mountainErosionStrength      = clampRange(mountainErosionStrength, 0.0, 4.0);
    mountainErosionGullyWeight   = clampRange(mountainErosionGullyWeight, 0.0, 4.0);
    mountainErosionDetail        = clampRange(mountainErosionDetail, 0.1, 8.0);
    mountainErosionCellScale     = clampRange(mountainErosionCellScale, 0.1, 4.0);
    mountainErosionNormalization = clampRange(mountainErosionNormalization, 0.0, 1.0);
    mountainErosionOctaves       = static_cast<std::uint32_t>(
        clampRange(static_cast<double>(mountainErosionOctaves), 1.0, static_cast<double>(maxFbmOctaves)));
    mountainErosionGain       = clampRange(mountainErosionGain, 0.1, 0.99);
    mountainErosionLacunarity = clampRange(mountainErosionLacunarity, 1.0, 4.0);
    mountainErosionRoundingRidge  = clampRange(mountainErosionRoundingRidge, 0.0, 4.0);
    mountainErosionRoundingCrease = clampRange(mountainErosionRoundingCrease, 0.0, 4.0);
    mountainErosionRoundingGlobal = clampRange(mountainErosionRoundingGlobal, 0.0, 4.0);
    mountainErosionRoundingGain   = clampRange(mountainErosionRoundingGain, 0.5, 8.0);
    mountainErosionOnsetInput     = clampRange(mountainErosionOnsetInput, 0.05, 16.0);
    mountainErosionOnsetOctave    = clampRange(mountainErosionOnsetOctave, 0.05, 16.0);
    mountainErosionOnsetRidgeMap  = clampRange(mountainErosionOnsetRidgeMap, 0.05, 16.0);
    mountainErosionOnsetRidgeMask = clampRange(mountainErosionOnsetRidgeMask, 0.05, 16.0);
    mountainErosionAssumedSlope      = clampRange(mountainErosionAssumedSlope, 0.0, 16.0);
    mountainErosionAssumedSlopeBlend = clampRange(mountainErosionAssumedSlopeBlend, 0.0, 1.0);
    mountainErosionProfileHigh = clampRange(mountainErosionProfileHigh, mountainErosionProfileLow + 0.01, 2.0);
    mountainErosionProfileLow  = clampRange(mountainErosionProfileLow, -2.0, mountainErosionProfileHigh - 0.01);

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
    strataGateHigh = clampRange(strataGateHigh, 0.05, 1.0);
    strataGateLow  = clampRange(strataGateLow, 0.0, strataGateHigh - 0.02);

    // ---- geologic provinces -------------------------------------------------------
    provinceWavelengthMeters = provinceWavelengthMeters <= 0.0
        ? 0.0 // derived below (1.6 x continent wavelength), after the continent clamp
        : clampRange(provinceWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 32.0);
    provinceRockVariation = clampRange(provinceRockVariation, 0.0, 1.0);
    provinceStrataBias    = clampRange(provinceStrataBias, 0.0, 1.0);
    provinceTerraceBias   = clampRange(provinceTerraceBias, 0.0, 1.0);

    coastalFlattenMeters = clampRange(coastalFlattenMeters, 0.0, 0.1 * radiusMeters);
    coastalFlattenFloor  = clampRange(coastalFlattenFloor, 0.0, 1.0);
    coastalFlattenReachFraction = clampRange(coastalFlattenReachFraction, 0.5, 8.0);
    coastalFlattenFullFraction  = clampRange(coastalFlattenFullFraction, 0.0, coastalFlattenReachFraction);

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
    abyssalMaskHigh = clampRange(abyssalMaskHigh, -0.99, 1.0);
    abyssalMaskLow  = clampRange(abyssalMaskLow, -1.0, abyssalMaskHigh - 0.02);
    submarineFadeHighFraction = clampRange(submarineFadeHighFraction, 0.01, 1.0);
    submarineFadeLowFraction  = clampRange(submarineFadeLowFraction, 0.0, submarineFadeHighFraction);

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
    climateContinentalityStrength = clampRange(climateContinentalityStrength, 0.0, 1.0);
    climateRainShadowStrength     = clampRange(climateRainShadowStrength, 0.0, 1.0);
    colorVariation       = clampRange(colorVariation, 0.0, 1.0);
    colorVariationWavelengthMeters = clampRange(colorVariationWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    colorVariationGroundWavelengthMeters = clampRange(colorVariationGroundWavelengthMeters, radiusMeters / 16384.0, colorVariationWavelengthMeters);
    soilPatchStrength            = clampRange(soilPatchStrength, 0.0, 1.0);
    soilPatchWavelengthMeters    = clampRange(soilPatchWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    dryPatchStrength             = clampRange(dryPatchStrength, 0.0, 1.0);
    dryPatchWavelengthMeters     = clampRange(dryPatchWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    snowLineMeters       = clampRange(snowLineMeters, 0.0, 0.5 * radiusMeters);
    beachWidthMeters     = clampRange(beachWidthMeters, 0.0, 0.05 * radiusMeters);
    // The wobble may pinch the band to zero width but must never invert it
    // (a negative width would beach the INLAND side of the contour).
    beachShorelineWobble = clampRange(beachShorelineWobble, 0.0, 1.0);
    beachWetWidthMeters  = clampRange(beachWetWidthMeters, 0.0, 0.05 * radiusMeters);
    beachWetDarkening    = clampRange(beachWetDarkening, 0.0, 0.95);
    rockSlopeThreshold   = clampRange(rockSlopeThreshold, 0.05, 1.0);
    rockStrataContrast   = clampRange(rockStrataContrast, 0.0, 1.0);

    // ---- surface texturing -------------------------------------------------------
    // Texture tiles may span from a few meters to a whole chunk face; below
    // the finest voxel's band limit they would fade out with the LOD anyway.
    terrainTextureTileMetersGround = clampRange(terrainTextureTileMetersGround, 0.1, 0.25 * radiusMeters);
    terrainTextureTileMetersRock   = clampRange(terrainTextureTileMetersRock, 0.1, 0.25 * radiusMeters);
    waterTextureTileMeters         = clampRange(waterTextureTileMeters, 0.1, 0.25 * radiusMeters);
    // The warp amplitude stays well under one tile (a full tile would fold
    // the frame onto itself) and its wavelength cannot outgrow the planet.
    textureUvWarpStrength         = clampRange(textureUvWarpStrength, 0.0, 0.45);
    textureUvWarpWavelengthMeters = clampRange(textureUvWarpWavelengthMeters, 4.0, 0.25 * radiusMeters);
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
    walkTerminalVelocityMetersPerSecond = clampRange(walkTerminalVelocityMetersPerSecond, 1.0, 1000.0);
    swimSpeedMetersPerSecond       = clampRange(swimSpeedMetersPerSecond, 0.1, 1000.0);
    swimWadeDepthMeters            = clampRange(swimWadeDepthMeters, 0.0, 1000.0);

    // ---- atmosphere -------------------------------------------------------------
    atmosphereHeightMeters = clampRange(atmosphereHeightMeters, radiusMeters / 1024.0, radiusMeters / 2.0);
    sunDirectTintStrength  = clampRange(sunDirectTintStrength, 0.0, 1.0);
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
    waterRoughness        = std::clamp(waterRoughness, 0.0f, 1.0f);
    waterRoughnessOrbital = std::clamp(waterRoughnessOrbital, 0.0f, 1.0f);

    // ---- clouds --------------------------------------------------------------------
    cloudCoverageAmount    = clampRange(cloudCoverageAmount, 0.0, 1.0);
    cloudWavelengthMeters  = clampRange(cloudWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 4.0);
    cloudAltitudeMeters    = clampRange(cloudAltitudeMeters, radiusMeters / 8192.0, radiusMeters / 4.0);
    cloudDetailStrength    = clampRange(cloudDetailStrength, 0.0, 1.0);
    cloudBillowStrength    = clampRange(cloudBillowStrength, 0.0, 1.0);
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
    cloudShadowSoftness = clampRange(cloudShadowSoftness, 0.0, 1.0);
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
    // The disc range must stay inside the pixel scale of a capture: a
    // minimum above a few milliradians reads as a coin, a maximum above
    // half a degree as a blob; and the range itself must be ordered.
    starDiscRadiusMinRadians = clampRange(starDiscRadiusMinRadians, 0.0002, 0.006);
    starDiscRadiusMaxRadians = clampRange(starDiscRadiusMaxRadians, 0.0003, 0.012);
    if (starDiscRadiusMaxRadians < starDiscRadiusMinRadians)
    {
        std::swap(starDiscRadiusMinRadians, starDiscRadiusMaxRadians);
    }
    galaxyBandStrength     = clampRange(galaxyBandStrength, 0.0, 1.0);
    galaxyBandWidthRadians = clampRange(galaxyBandWidthRadians, 0.05, 1.2);

    seaIceTemperature = clampRange(seaIceTemperature, 0.0, 1.0);
    seaIceTransition  = clampRange(seaIceTransition, 0.005, 0.5);
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        waterColorIce[channel] = std::clamp(waterColorIce[channel], 0.0f, 1.0f);
    }
    waterDepthTintMeters        = clampRange(waterDepthTintMeters, 1.0, radiusMeters);
    waterShoreMixMeters         = clampRange(waterShoreMixMeters, 0.0, radiusMeters);
    waterShoreSandBrightness    = clampRange(waterShoreSandBrightness, 1.0, 4.0);
    waterFoamWidthMeters        = clampRange(waterFoamWidthMeters, 0.0, 0.01 * radiusMeters);
    waterFoamStrength           = clampRange(waterFoamStrength, 0.0, 1.0);
    waterFoamFalloffExponent    = clampRange(waterFoamFalloffExponent, 1.0, 8.0);
    waterWaveAmplitude        = clampRange(waterWaveAmplitude, 0.0, 1.0);
    waterWaveWavelengthMeters = clampRange(waterWaveWavelengthMeters, radiusMeters / 16384.0, radiusMeters);
    waterWaveChopFraction     = clampRange(waterWaveChopFraction, 0.0, 1.0);

    // ---- moon / aurora ---------------------------------------------------------------
    moonAngularRadiusRadians    = clampRange(moonAngularRadiusRadians, 0.0005, 0.02);
    moonOrbitalPeriodSeconds    = clampRange(moonOrbitalPeriodSeconds, 60.0, 1.0e6);
    moonOrbitInclinationRadians = clampRange(moonOrbitInclinationRadians, 0.0, 1.55);
    moonAlbedo                  = clampRange(moonAlbedo, 0.0, 8.0);
    auroraStrength              = clampRange(auroraStrength, 0.0, 1.0);

    // ---- surface scatter ----------------------------------------------------------
    vegetationTreeDensity      = clampRange(vegetationTreeDensity, 0.0, 4.0);
    vegetationBushDensity      = clampRange(vegetationBushDensity, 0.0, 4.0);
    vegetationGrassDensity     = clampRange(vegetationGrassDensity, 0.0, 4.0);
    vegetationTreeRadiusMeters  = clampRange(vegetationTreeRadiusMeters, 100.0, 20000.0);
    vegetationGrassRadiusMeters = clampRange(vegetationGrassRadiusMeters, 5.0, 200.0);
    surfaceRockDensity      = clampRange(surfaceRockDensity, 0.0, 4.0);
    surfaceRockRadiusMeters = clampRange(surfaceRockRadiusMeters, 20.0, 4000.0);
    surfaceShadowStrength   = clampRange(surfaceShadowStrength, 0.0, 2.0);
    whitecapStrength        = clampRange(whitecapStrength, 0.0, 1.0);
    whitecapWindSpeedMps    = clampRange(whitecapWindSpeedMps, 0.0, 40.0);
    surfFoamStrength        = clampRange(surfFoamStrength, 0.0, 1.0);
    birdFlockCount          = clampRange(birdFlockCount, 0.0, 16.0);
    birdFlockSize           = clampRange(birdFlockSize, 0.0, 64.0);
    valleyMistStrength      = clampRange(valleyMistStrength, 0.0, 1.0);
    scatterRebuildDistanceMeters = clampRange(scatterRebuildDistanceMeters, 4.0, 500.0);

    // ---- atmosphere (aerial perspective) / debug -----------------------------------
    aerialPerspectiveScaleMeters = clampRange(aerialPerspectiveScaleMeters, 50.0, std::max(50.0, atmosphereHeightMeters));
    aerialPerspectiveAsymptote   = clampRange(aerialPerspectiveAsymptote, 0.0, 1.0);
    debugChunkBorderWidthMeters    = clampRange(debugChunkBorderWidthMeters, 0.05, 0.01 * radiusMeters);

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
    derivedVoxelMeters     = std::ldexp(derivedRootSizeMeters, -static_cast<int>(depth)) / static_cast<double>(leafGridN);
    derivedMinShellRadius  = std::max(0.0, radiusMeters - relief);
    derivedMaxShellRadius  = radiusMeters + relief;
    derivedMaxReliefMeters = relief;
    derivedSeaRadiusMeters = radiusMeters + seaLevelOffsetMeters;
    // The broad mountain mass field rides at the configured multiple of the
    // crest wavelength (see reliefImpl); derived here so the octree's
    // gradient bounds and the density field agree on one number.
    derivedMountainMassWavelengthMeters = std::max(1.0, mountainWavelengthMeters * mountainMassWavelengthMultiple);
    // Geologic provinces default to a broad multiple of the continent
    // wavelength: provinces span whole landmass regions, a level above the
    // continent field's own feature scale.
    derivedProvinceWavelengthMeters = provinceWavelengthMeters > 0.0
        ? provinceWavelengthMeters
        : std::max(1.0, continentWavelengthMeters * 1.6);
    // Sampling stride of the mesher's macro-climate grid: a sixteenth of the
    // smallest climate-field wavelength the interpolated fields carry
    // (temperature, moisture, clouds, provinces), so the grid resolves every
    // octave those fields hold with several samples. The floor keeps the
    // grid coarse enough that the packed cell indices cannot collide on any
    // supported radius.
    derivedMacroClimateStrideMeters = std::max(radiusMeters / 262144.0,
        0.0625 * std::min(std::min(temperatureWavelengthMeters, moistureWavelengthMeters),
            std::min(cloudWavelengthMeters, derivedProvinceWavelengthMeters)));
    return true;
}

} // namespace planet
} // namespace engine
