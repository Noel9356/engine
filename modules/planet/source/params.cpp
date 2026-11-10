// PlanetGenerationParams::validate - clamps every configured field into its
// documented range and derives the aggregate values (root cube, octree depth,
// shell band, sea radius) the rest of the module relies on. See params.hpp
// for the meaning of each field.

#include <engine/planet/params.hpp>

#include <algorithm>
#include <cmath>

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
    seaLevelOffsetMeters = clampRange(seaLevelOffsetMeters, -0.5 * radiusMeters, 0.5 * radiusMeters);

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
    // Finest-voxel floor: the chunk key packs depth + 20-bit lattice
    // coordinates per axis, so the finest grid may hold 2^20 cells per root
    // axis (2^depth * leafGridN). With leafGridN <= 64 that allows voxels down
    // to rootSize / 2^20 ~= radius / 37449 at the default bounding factor; the
    // floor below leaves a full extra octave of headroom.
    targetVoxelMeters = clampRange(targetVoxelMeters, radiusMeters / 131072.0, radiusMeters / 4.0);
    maxDepthLimit = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxDepthLimit), 1.0, static_cast<double>(maxSupportedOctreeDepth)));
    lodAngularVoxelRadians = clampRange(lodAngularVoxelRadians, 0.0005, 0.5);
    lodBandLimitFactor = clampRange(lodBandLimitFactor, 1.0, 4.0);
    boundingFactor    = clampRange(boundingFactor, 1.05, 4.0);
    maxConcurrentBuilds  = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxConcurrentBuilds), 1.0, 64.0));
    maxNewBuildsPerUpdate = static_cast<std::uint32_t>(clampRange(static_cast<double>(maxNewBuildsPerUpdate), 1.0, 256.0));
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
    cliffWavelengthMeters = clampRange(cliffWavelengthMeters, radiusMeters / 8192.0, radiusMeters * 4.0);

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
    waterDepthTintMeters = clampRange(waterDepthTintMeters, 1.0, radiusMeters);

    equatorTemperature = clampRange(equatorTemperature, 0.0, 1.0);
    poleTemperature    = clampRange(poleTemperature, 0.0, equatorTemperature);
    temperaturePerturbation = clampRange(temperaturePerturbation, 0.0, 1.0);
    temperatureWavelengthMeters = clampRange(temperatureWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 16.0);
    temperatureLapseMeters      = clampRange(temperatureLapseMeters, 1.0, radiusMeters);
    moistureWavelengthMeters    = clampRange(moistureWavelengthMeters, radiusMeters / 1024.0, radiusMeters * 16.0);
    moisturePerturbation        = clampRange(moisturePerturbation, 0.0, 1.0);
    moistureWarpAmplitude       = clampRange(moistureWarpAmplitude, 0.0, 2.0);
    coastalMoistureBoost        = clampRange(coastalMoistureBoost, 0.0, 1.0);
    coastalMoistureBandMeters   = clampRange(coastalMoistureBandMeters, 1.0, radiusMeters);
    colorVariation       = clampRange(colorVariation, 0.0, 1.0);
    snowLineMeters       = clampRange(snowLineMeters, 0.0, 0.5 * radiusMeters);
    beachWidthMeters     = clampRange(beachWidthMeters, 0.0, 0.05 * radiusMeters);
    rockSlopeThreshold   = clampRange(rockSlopeThreshold, 0.05, 1.0);
    rockStrataContrast   = clampRange(rockStrataContrast, 0.0, 1.0);
    biomeBlendWidth        = clampRange(biomeBlendWidth, 0.005, 0.2);
    slopeBlendWidth        = clampRange(slopeBlendWidth, 0.01, 0.5);
    elevationBlendFraction = clampRange(elevationBlendFraction, 0.02, 1.0);

    for (std::size_t index = 0; index < biomeCount * 3; ++index)
    {
        biomePalette[index] = std::clamp(biomePalette[index], 0.0f, 1.0f);
    }

    // ---- water --------------------------------------------------------------------
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        waterColorShallow[channel] = std::clamp(waterColorShallow[channel], 0.0f, 1.0f);
        waterColorDeep[channel]    = std::clamp(waterColorDeep[channel], 0.0f, 1.0f);
    }
    waterRoughness = std::clamp(waterRoughness, 0.0f, 1.0f);

    // ---- derived values -------------------------------------------------------------
    const double relief = continentAmplitudeMeters + regionalAmplitudeMeters + mountainAmplitudeMeters + hillAmplitudeMeters
                        + detailAmplitudeMeters + overhangAmplitudeMeters + strataAmplitudeMeters + caveCarveMeters;
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
