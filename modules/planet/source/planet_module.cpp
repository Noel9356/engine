// planet module implementation.
//
// Bridges the pure generation core (Planet facade, see planet.hpp) into the
// engine's capability system: reads the [planet] configuration section once
// in construct(), hands out per-planets through createPlanet (each layering
// its PlanetDescription overrides over the configured defaults), and streams
// visible chunk meshes to consumers as borrowed views.

#include <cstddef>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/planet.hpp>
#include <engine/planet/types.hpp>

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace engine
{
namespace planet
{
namespace
{

void setError(PlanetError *output, PlanetError value)
{
    if (output != nullptr)
    {
        *output = value;
    }
}

// Configuration-key table: one row per [planet] key, describing how the value
// lands in PlanetGenerationParams. The module's config_keys list, the shipped
// config.toml and this table describe the same set of keys; keeping the
// mapping data-driven means every new knob is one row here plus its field in
// the params struct (mirrored in config.toml), not a fresh if-chain.
enum class SettingKind : std::uint32_t
{
    Floating,
    Integer,
    Boolean
};

using Setter = void (*)(PlanetGenerationParams &params, double value);

struct SettingRow
{
    const char  *key;
    SettingKind  kind;
    Setter       setter;
    // Direct int64 path for rows whose value is genuinely 64-bit (the seed);
    // nullptr for every uint32-width row.
    void (*int64Setter)(PlanetGenerationParams &params, std::int64_t value) = nullptr;
};

// Integer-valued params are stored as unsigned widths (counts, capacities,
// octave numbers); validate() clamps them into their safe ranges afterwards.
// A raw config value must round-trip through that path as a NON-negative,
// finite number first: casting a negative or huge double straight to the
// unsigned field is undefined at the cast site and never reaches the clamp.
// Values above the unsigned range clamp to it here; validate() applies the
// real per-field bound.
void setUnsignedParam(std::uint32_t &field, double value)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        field = 0; // 0 is always off-range: validate() clamps it to the field's floor
        return;
    }
    field = static_cast<std::uint32_t>(std::min(value, 4.294967295e9));
}

void setSeedRaw(PlanetGenerationParams &params, std::int64_t value)
{
    // Seeds are hashed through 64-bit streams; the wrap of a negative
    // integer is as deterministic as any other seed. The raw int64 path
    // keeps every bit (a double detour would drop the bits above 2^53).
    params.seed = static_cast<std::uint64_t>(value);
}

void setSeed(PlanetGenerationParams &params, double value)
{
    setSeedRaw(params, static_cast<std::int64_t>(value));
}

void setRadius(PlanetGenerationParams &params, double value) { params.radiusMeters = value; }
void setSeaLevel(PlanetGenerationParams &params, double value) { params.seaLevelOffsetMeters = value; }
void setAbyssalDepth(PlanetGenerationParams &params, double value) { params.abyssalDepthMeters = value; }
void setAbyssalMaskLow(PlanetGenerationParams &params, double value) { params.abyssalMaskLow = value; }
void setAbyssalMaskHigh(PlanetGenerationParams &params, double value) { params.abyssalMaskHigh = value; }
void setSubmarineFadeLowFraction(PlanetGenerationParams &params, double value) { params.submarineFadeLowFraction = value; }
void setSubmarineFadeHighFraction(PlanetGenerationParams &params, double value) { params.submarineFadeHighFraction = value; }
void setLeafGridN(PlanetGenerationParams &params, double value) { setUnsignedParam(params.leafGridN, value); }
void setTargetVoxel(PlanetGenerationParams &params, double value) { params.targetVoxelMeters = value; }
void setMaxDepthLimit(PlanetGenerationParams &params, double value) { setUnsignedParam(params.maxDepthLimit, value); }
void setLodAngularVoxel(PlanetGenerationParams &params, double value) { params.lodAngularVoxelRadians = value; }
void setLodBandLimit(PlanetGenerationParams &params, double value) { params.lodBandLimitFactor = value; }
void setHorizonCulling(PlanetGenerationParams &params, double value) { params.horizonCulling = value != 0.0; }
void setLodMissingFallback(PlanetGenerationParams &params, double value) { params.lodMissingFallback = value != 0.0; }
void setLodScaffoldBuildsPerUpdate(PlanetGenerationParams &params, double value) { setUnsignedParam(params.lodScaffoldBuildsPerUpdate, value); }
void setBoundingFactor(PlanetGenerationParams &params, double value) { params.boundingFactor = value; }
void setSelectionUpdateDistance(PlanetGenerationParams &params, double value) { params.selectionUpdateDistanceMeters = value; }
void setNodeRecordLifetime(PlanetGenerationParams &params, double value) { setUnsignedParam(params.nodeRecordLifetimeUpdates, value); }
void setNodeRecordPruneInterval(PlanetGenerationParams &params, double value) { setUnsignedParam(params.nodeRecordPruneIntervalUpdates, value); }
void setMaxConcurrentBuilds(PlanetGenerationParams &params, double value) { setUnsignedParam(params.maxConcurrentBuilds, value); }
void setMaxNewBuildsPerUpdate(PlanetGenerationParams &params, double value) { setUnsignedParam(params.maxNewBuildsPerUpdate, value); }
void setStarvedInlineBuildInterval(PlanetGenerationParams &params, double value) { setUnsignedParam(params.starvedInlineBuildIntervalUpdates, value); }
void setMaxLivePlanets(PlanetGenerationParams &params, double value) { setUnsignedParam(params.maxLivePlanets, value); }
void setChunkEvictionUpdates(PlanetGenerationParams &params, double value) { setUnsignedParam(params.chunkEvictionUpdates, value); }
void setChunkCacheCapacity(PlanetGenerationParams &params, double value) { setUnsignedParam(params.chunkCacheCapacity, value); }
void setContinentAmplitude(PlanetGenerationParams &params, double value) { params.continentAmplitudeMeters = value; }
void setContinentWavelength(PlanetGenerationParams &params, double value) { params.continentWavelengthMeters = value; }
void setContinentOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.continentsFbm.octaves, value); }
void setDomainWarpAmplitude(PlanetGenerationParams &params, double value) { params.domainWarpAmplitude = value; }
void setDomainWarpWavelength(PlanetGenerationParams &params, double value) { params.domainWarpWavelengthMeters = value; }
void setMountainAmplitude(PlanetGenerationParams &params, double value) { params.mountainAmplitudeMeters = value; }
void setMountainWavelength(PlanetGenerationParams &params, double value) { params.mountainWavelengthMeters = value; }
void setMountainMaskLow(PlanetGenerationParams &params, double value) { params.mountainMaskLow = value; }
void setMountainMaskHigh(PlanetGenerationParams &params, double value) { params.mountainMaskHigh = value; }
void setMountainSharpness(PlanetGenerationParams &params, double value) { params.mountainSharpness = value; }
void setMountainOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.mountainsFbm.octaves, value); }
void setMountainMassFraction(PlanetGenerationParams &params, double value) { params.mountainMassFraction = value; }
void setMountainMassWavelengthMultiple(PlanetGenerationParams &params, double value) { params.mountainMassWavelengthMultiple = value; }
void setMountainCrestMassCoupling(PlanetGenerationParams &params, double value) { params.mountainCrestMassCoupling = value; }
void setRangeClusterGateLow(PlanetGenerationParams &params, double value) { params.rangeClusterGateLow = value; }
void setRangeClusterGateHigh(PlanetGenerationParams &params, double value) { params.rangeClusterGateHigh = value; }
void setRangeClusterStrength(PlanetGenerationParams &params, double value) { params.rangeClusterStrength = value; }
void setRangeClusterWavelength(PlanetGenerationParams &params, double value) { params.rangeClusterWavelengthMeters = value; }
void setMountainErosionWeight(PlanetGenerationParams &params, double value) { params.mountainErosionWeight = value; }
void setMountainErosionFaceBlendWidth(PlanetGenerationParams &params, double value) { params.mountainErosionFaceBlendWidth = value; }
void setMountainErosionTerrainFrequency(PlanetGenerationParams &params, double value) { params.mountainErosionTerrainFrequency = value; }
void setMountainErosionTerrainAmplitude(PlanetGenerationParams &params, double value) { params.mountainErosionTerrainAmplitude = value; }
void setMountainErosionTerrainOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.mountainErosionTerrainOctaves, value); }
void setMountainErosionTerrainGain(PlanetGenerationParams &params, double value) { params.mountainErosionTerrainGain = value; }
void setMountainErosionTerrainLacunarity(PlanetGenerationParams &params, double value) { params.mountainErosionTerrainLacunarity = value; }
void setMountainErosionHeightOffsetLow(PlanetGenerationParams &params, double value) { params.mountainErosionHeightOffsetLow = value; }
void setMountainErosionHeightOffsetBlend(PlanetGenerationParams &params, double value) { params.mountainErosionHeightOffsetBlend = value; }
void setMountainErosionScale(PlanetGenerationParams &params, double value) { params.mountainErosionScale = value; }
void setMountainErosionStrength(PlanetGenerationParams &params, double value) { params.mountainErosionStrength = value; }
void setMountainErosionGullyWeight(PlanetGenerationParams &params, double value) { params.mountainErosionGullyWeight = value; }
void setMountainErosionDetail(PlanetGenerationParams &params, double value) { params.mountainErosionDetail = value; }
void setMountainErosionCellScale(PlanetGenerationParams &params, double value) { params.mountainErosionCellScale = value; }
void setMountainErosionNormalization(PlanetGenerationParams &params, double value) { params.mountainErosionNormalization = value; }
void setMountainErosionOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.mountainErosionOctaves, value); }
void setMountainErosionGain(PlanetGenerationParams &params, double value) { params.mountainErosionGain = value; }
void setMountainErosionLacunarity(PlanetGenerationParams &params, double value) { params.mountainErosionLacunarity = value; }
void setMountainErosionRoundingRidge(PlanetGenerationParams &params, double value) { params.mountainErosionRoundingRidge = value; }
void setMountainErosionRoundingCrease(PlanetGenerationParams &params, double value) { params.mountainErosionRoundingCrease = value; }
void setMountainErosionRoundingGlobal(PlanetGenerationParams &params, double value) { params.mountainErosionRoundingGlobal = value; }
void setMountainErosionRoundingGain(PlanetGenerationParams &params, double value) { params.mountainErosionRoundingGain = value; }
void setMountainErosionOnsetInput(PlanetGenerationParams &params, double value) { params.mountainErosionOnsetInput = value; }
void setMountainErosionOnsetOctave(PlanetGenerationParams &params, double value) { params.mountainErosionOnsetOctave = value; }
void setMountainErosionOnsetRidgeMap(PlanetGenerationParams &params, double value) { params.mountainErosionOnsetRidgeMap = value; }
void setMountainErosionOnsetRidgeMask(PlanetGenerationParams &params, double value) { params.mountainErosionOnsetRidgeMask = value; }
void setMountainErosionAssumedSlope(PlanetGenerationParams &params, double value) { params.mountainErosionAssumedSlope = value; }
void setMountainErosionAssumedSlopeBlend(PlanetGenerationParams &params, double value) { params.mountainErosionAssumedSlopeBlend = value; }
void setMountainErosionProfileLow(PlanetGenerationParams &params, double value) { params.mountainErosionProfileLow = value; }
void setMountainErosionProfileHigh(PlanetGenerationParams &params, double value) { params.mountainErosionProfileHigh = value; }
void setRegionalAmplitude(PlanetGenerationParams &params, double value) { params.regionalAmplitudeMeters = value; }
void setRegionalWavelength(PlanetGenerationParams &params, double value) { params.regionalWavelengthMeters = value; }
void setRegionalOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.regionalFbm.octaves, value); }
void setHillAmplitude(PlanetGenerationParams &params, double value) { params.hillAmplitudeMeters = value; }
void setHillWavelength(PlanetGenerationParams &params, double value) { params.hillWavelengthMeters = value; }
void setHillOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.hillsFbm.octaves, value); }
void setDetailAmplitude(PlanetGenerationParams &params, double value) { params.detailAmplitudeMeters = value; }
void setDetailWavelength(PlanetGenerationParams &params, double value) { params.detailWavelengthMeters = value; }
void setDetailPlainsFraction(PlanetGenerationParams &params, double value) { params.detailPlainsFraction = value; }
void setDetailOctaves(PlanetGenerationParams &params, double value) { setUnsignedParam(params.detailFbm.octaves, value); }
void setTerraceStrength(PlanetGenerationParams &params, double value) { params.terraceStrength = value; }
void setTerraceRiserFraction(PlanetGenerationParams &params, double value) { params.terraceRiserFraction = value; }
void setTerraceSteps(PlanetGenerationParams &params, double value) { params.terraceSteps = value; }
void setCliffWavelength(PlanetGenerationParams &params, double value) { params.cliffWavelengthMeters = value; }
void setOverhangAmplitude(PlanetGenerationParams &params, double value) { params.overhangAmplitudeMeters = value; }
void setOverhangWavelength(PlanetGenerationParams &params, double value) { params.overhangWavelengthMeters = value; }
void setOverhangPlainsFraction(PlanetGenerationParams &params, double value) { params.overhangPlainsFraction = value; }
void setStrataAmplitude(PlanetGenerationParams &params, double value) { params.strataAmplitudeMeters = value; }
void setStrataBand(PlanetGenerationParams &params, double value) { params.strataBandMeters = value; }
void setStrataWarp(PlanetGenerationParams &params, double value) { params.strataWarp = value; }
void setStrataGateLow(PlanetGenerationParams &params, double value) { params.strataGateLow = value; }
void setStrataGateHigh(PlanetGenerationParams &params, double value) { params.strataGateHigh = value; }
void setProvincesEnabled(PlanetGenerationParams &params, double value) { params.provincesEnabled = value != 0.0; }
void setProvinceWavelength(PlanetGenerationParams &params, double value) { params.provinceWavelengthMeters = value; }
void setProvinceRockVariation(PlanetGenerationParams &params, double value) { params.provinceRockVariation = value; }
void setProvinceStrataBias(PlanetGenerationParams &params, double value) { params.provinceStrataBias = value; }
void setProvinceTerraceBias(PlanetGenerationParams &params, double value) { params.provinceTerraceBias = value; }
void setCoastalFlattenMeters(PlanetGenerationParams &params, double value) { params.coastalFlattenMeters = value; }
void setCoastalFlattenFloor(PlanetGenerationParams &params, double value) { params.coastalFlattenFloor = value; }
void setCoastalFlattenFullFraction(PlanetGenerationParams &params, double value) { params.coastalFlattenFullFraction = value; }
void setCoastalFlattenReachFraction(PlanetGenerationParams &params, double value) { params.coastalFlattenReachFraction = value; }
void setCavesEnabled(PlanetGenerationParams &params, double value) { params.cavesEnabled = value != 0.0; }
void setCaveThreshold(PlanetGenerationParams &params, double value) { params.caveThreshold = value; }
void setCaveWormWavelength(PlanetGenerationParams &params, double value) { params.caveWormWavelengthMeters = value; }
void setCaveCarve(PlanetGenerationParams &params, double value) { params.caveCarveMeters = value; }
void setShallowWater(PlanetGenerationParams &params, double value) { params.shallowWaterMeters = value; }
void setDeepWater(PlanetGenerationParams &params, double value) { params.deepWaterMeters = value; }
void setWaterDepthTint(PlanetGenerationParams &params, double value) { params.waterDepthTintMeters = value; }
void setWaterShoreMix(PlanetGenerationParams &params, double value) { params.waterShoreMixMeters = value; }
void setEquatorTemperature(PlanetGenerationParams &params, double value) { params.equatorTemperature = value; }
void setPoleTemperature(PlanetGenerationParams &params, double value) { params.poleTemperature = value; }
void setTemperaturePerturbation(PlanetGenerationParams &params, double value) { params.temperaturePerturbation = value; }
void setTemperatureWavelength(PlanetGenerationParams &params, double value) { params.temperatureWavelengthMeters = value; }
void setTemperatureLapse(PlanetGenerationParams &params, double value) { params.temperatureLapseMeters = value; }
void setMoistureWavelength(PlanetGenerationParams &params, double value) { params.moistureWavelengthMeters = value; }
void setMoisturePerturbation(PlanetGenerationParams &params, double value) { params.moisturePerturbation = value; }
void setMoistureWarpAmplitude(PlanetGenerationParams &params, double value) { params.moistureWarpAmplitude = value; }
void setCoastalMoistureBoost(PlanetGenerationParams &params, double value) { params.coastalMoistureBoost = value; }
void setCoastalMoistureBand(PlanetGenerationParams &params, double value) { params.coastalMoistureBandMeters = value; }
void setClimateContinentality(PlanetGenerationParams &params, double value) { params.climateContinentalityStrength = value; }
void setClimateRainShadow(PlanetGenerationParams &params, double value) { params.climateRainShadowStrength = value; }
void setClimateWindZones(PlanetGenerationParams &params, double value) { params.climateWindZonesEnabled = value != 0.0; }
void setMoistureLatitudeBands(PlanetGenerationParams &params, double value) { params.moistureLatitudeBands = value != 0.0; }
void setMoistureBeltEquator(PlanetGenerationParams &params, double value) { params.moistureBeltEquatorAmplitude = value; }
void setMoistureBeltSubtropical(PlanetGenerationParams &params, double value) { params.moistureBeltSubtropicalAmplitude = value; }
void setMoistureBeltTemperate(PlanetGenerationParams &params, double value) { params.moistureBeltTemperateAmplitude = value; }
void setMoistureBeltPolar(PlanetGenerationParams &params, double value) { params.moistureBeltPolarAmplitude = value; }
void setColorVariation(PlanetGenerationParams &params, double value) { params.colorVariation = value; }
void setColorVariationWavelength(PlanetGenerationParams &params, double value) { params.colorVariationWavelengthMeters = value; }
void setColorVariationGroundWavelength(PlanetGenerationParams &params, double value) { params.colorVariationGroundWavelengthMeters = value; }
void setSoilPatchStrength(PlanetGenerationParams &params, double value) { params.soilPatchStrength = value; }
void setSoilPatchWavelength(PlanetGenerationParams &params, double value) { params.soilPatchWavelengthMeters = value; }
void setDryPatchStrength(PlanetGenerationParams &params, double value) { params.dryPatchStrength = value; }
void setDryPatchWavelength(PlanetGenerationParams &params, double value) { params.dryPatchWavelengthMeters = value; }
void setSnowLine(PlanetGenerationParams &params, double value) { params.snowLineMeters = value; }
void setBeachWidth(PlanetGenerationParams &params, double value) { params.beachWidthMeters = value; }
void setRockSlopeThreshold(PlanetGenerationParams &params, double value) { params.rockSlopeThreshold = value; }
void setRockStrataContrast(PlanetGenerationParams &params, double value) { params.rockStrataContrast = value; }
void setTerrainTextureTileGround(PlanetGenerationParams &params, double value) { params.terrainTextureTileMetersGround = value; }
void setTerrainTextureTileRock(PlanetGenerationParams &params, double value) { params.terrainTextureTileMetersRock = value; }
void setWaterTextureTile(PlanetGenerationParams &params, double value) { params.waterTextureTileMeters = value; }
void setBiomeBlendWidth(PlanetGenerationParams &params, double value) { params.biomeBlendWidth = value; }
void setSlopeBlendWidth(PlanetGenerationParams &params, double value) { params.slopeBlendWidth = value; }
void setElevationBlendFraction(PlanetGenerationParams &params, double value) { params.elevationBlendFraction = value; }
void setWaterEnabled(PlanetGenerationParams &params, double value) { params.waterEnabled = value != 0.0; }
void setWaterRoughness(PlanetGenerationParams &params, double value) { params.waterRoughness = static_cast<float>(value); }
void setSeaIceTemperature(PlanetGenerationParams &params, double value) { params.seaIceTemperature = value; }
void setSeaIceTransition(PlanetGenerationParams &params, double value) { params.seaIceTransition = value; }
void setWaterWaveAmplitude(PlanetGenerationParams &params, double value) { params.waterWaveAmplitude = value; }
void setWaterWaveWavelength(PlanetGenerationParams &params, double value) { params.waterWaveWavelengthMeters = value; }
void setSurfaceGravity(PlanetGenerationParams &params, double value) { params.surfaceGravityMetersPerSecond2 = value; }
void setWalkSpeed(PlanetGenerationParams &params, double value) { params.walkSpeedMetersPerSecond = value; }
void setWalkSprintMultiplier(PlanetGenerationParams &params, double value) { params.walkSprintMultiplier = value; }
void setJumpSpeed(PlanetGenerationParams &params, double value) { params.jumpSpeedMetersPerSecond = value; }
void setEyeHeight(PlanetGenerationParams &params, double value) { params.eyeHeightMeters = value; }
void setWalkGroundSnap(PlanetGenerationParams &params, double value) { params.walkGroundSnapMeters = value; }
void setWalkMaxStepUp(PlanetGenerationParams &params, double value) { params.walkMaxStepUpMeters = value; }
void setWalkMaxClimbGrade(PlanetGenerationParams &params, double value) { params.walkMaxClimbGrade = value; }
void setWalkTerminalVelocity(PlanetGenerationParams &params, double value) { params.walkTerminalVelocityMetersPerSecond = value; }
void setSwimEnabled(PlanetGenerationParams &params, double value) { params.swimEnabled = value != 0.0; }
void setSwimSpeed(PlanetGenerationParams &params, double value) { params.swimSpeedMetersPerSecond = value; }
void setSwimWadeDepth(PlanetGenerationParams &params, double value) { params.swimWadeDepthMeters = value; }
void setAtmosphereEnabled(PlanetGenerationParams &params, double value) { params.atmosphereEnabled = value != 0.0; }
void setAtmosphereHeight(PlanetGenerationParams &params, double value) { params.atmosphereHeightMeters = value; }
void setAerialPerspectiveScaleFraction(PlanetGenerationParams &params, double value) { params.aerialPerspectiveScaleFraction = value; }
void setAerialPerspectiveAsymptote(PlanetGenerationParams &params, double value) { params.aerialPerspectiveAsymptote = value; }
void setDebugChunkBorders(PlanetGenerationParams &params, double value) { params.debugChunkBorders = value != 0.0; }
void setDebugChunkBorderWidth(PlanetGenerationParams &params, double value) { params.debugChunkBorderWidthMeters = value; }
void setCliffSteepnessLow(PlanetGenerationParams &params, double value) { params.cliffSteepnessLow = value; }
void setCliffSteepnessHigh(PlanetGenerationParams &params, double value) { params.cliffSteepnessHigh = value; }
void setCliffSharpness(PlanetGenerationParams &params, double value) { params.cliffSharpness = value; }
void setBeachWetWidth(PlanetGenerationParams &params, double value) { params.beachWetWidthMeters = value; }
void setBeachWetDarkening(PlanetGenerationParams &params, double value) { params.beachWetDarkening = value; }
void setWaterFoamWidth(PlanetGenerationParams &params, double value) { params.waterFoamWidthMeters = value; }
void setWaterFoamStrength(PlanetGenerationParams &params, double value) { params.waterFoamStrength = value; }
void setWaterFoamFalloffExponent(PlanetGenerationParams &params, double value) { params.waterFoamFalloffExponent = value; }
void setWaterWaveChopFraction(PlanetGenerationParams &params, double value) { params.waterWaveChopFraction = value; }
void setWaterShoreSandBrightness(PlanetGenerationParams &params, double value) { params.waterShoreSandBrightness = value; }
void setCloudsEnabled(PlanetGenerationParams &params, double value) { params.cloudsEnabled = value != 0.0; }
void setCloudCoverage(PlanetGenerationParams &params, double value) { params.cloudCoverageAmount = value; }
void setCloudWavelength(PlanetGenerationParams &params, double value) { params.cloudWavelengthMeters = value; }
void setCloudAltitude(PlanetGenerationParams &params, double value) { params.cloudAltitudeMeters = value; }
void setCloudDetailStrength(PlanetGenerationParams &params, double value) { params.cloudDetailStrength = value; }
void setCloudEdgeSoftness(PlanetGenerationParams &params, double value) { params.cloudEdgeSoftness = value; }
void setCloudWeatherStrength(PlanetGenerationParams &params, double value) { params.cloudWeatherStrength = value; }
void setCloudWeatherWavelengthMultiple(PlanetGenerationParams &params, double value) { params.cloudWeatherWavelengthMultiple = value; }
void setCloudLatitudeBandStrength(PlanetGenerationParams &params, double value) { params.cloudLatitudeBandStrength = value; }
void setCloudBandFrequency(PlanetGenerationParams &params, double value) { params.cloudBandFrequency = value; }
void setCloudShadowStrength(PlanetGenerationParams &params, double value) { params.cloudShadowStrength = value; }
void setCloudShadowSoftness(PlanetGenerationParams &params, double value) { params.cloudShadowSoftness = value; }
void setStarsEnabled(PlanetGenerationParams &params, double value) { params.starsEnabled = value != 0.0; }
void setStarsDensity(PlanetGenerationParams &params, double value) { params.starsDensity = value; }
void setStarsBrightness(PlanetGenerationParams &params, double value) { params.starsBrightness = value; }

constexpr SettingRow kSettingRows[] = {
    {"seed",                      SettingKind::Integer, setSeed, setSeedRaw},
    {"radius_meters",             SettingKind::Floating, setRadius},
    {"sea_level_offset_meters",   SettingKind::Floating, setSeaLevel},
    {"abyssal_depth_meters",      SettingKind::Floating, setAbyssalDepth},
    {"abyssal_mask_low",          SettingKind::Floating, setAbyssalMaskLow},
    {"abyssal_mask_high",         SettingKind::Floating, setAbyssalMaskHigh},
    {"submarine_fade_low_fraction",  SettingKind::Floating, setSubmarineFadeLowFraction},
    {"submarine_fade_high_fraction", SettingKind::Floating, setSubmarineFadeHighFraction},
    {"leaf_grid_n",               SettingKind::Integer, setLeafGridN},
    {"target_voxel_meters",       SettingKind::Floating, setTargetVoxel},
    {"max_depth_limit",           SettingKind::Integer, setMaxDepthLimit},
    {"lod_angular_voxel_radians", SettingKind::Floating, setLodAngularVoxel},
    {"lod_band_limit_factor",     SettingKind::Floating, setLodBandLimit},
    {"lod_missing_fallback",      SettingKind::Boolean,  setLodMissingFallback},
    {"lod_scaffold_builds_per_update", SettingKind::Integer, setLodScaffoldBuildsPerUpdate},
    {"horizon_culling",           SettingKind::Boolean,  setHorizonCulling},
    {"bounding_factor",           SettingKind::Floating, setBoundingFactor},
    {"selection_update_distance_meters", SettingKind::Floating, setSelectionUpdateDistance},
    {"node_record_lifetime_updates",      SettingKind::Integer, setNodeRecordLifetime},
    {"node_record_prune_interval_updates", SettingKind::Integer, setNodeRecordPruneInterval},
    {"max_concurrent_builds",     SettingKind::Integer, setMaxConcurrentBuilds},
    {"max_new_builds_per_update", SettingKind::Integer, setMaxNewBuildsPerUpdate},
    {"starved_inline_build_interval_updates", SettingKind::Integer, setStarvedInlineBuildInterval},
    {"max_live_planets",          SettingKind::Integer, setMaxLivePlanets},
    {"chunk_eviction_updates",    SettingKind::Integer, setChunkEvictionUpdates},
    {"chunk_cache_capacity",      SettingKind::Integer, setChunkCacheCapacity},
    {"continent_amplitude_meters",    SettingKind::Floating, setContinentAmplitude},
    {"continent_wavelength_meters",   SettingKind::Floating, setContinentWavelength},
    {"continent_octaves",             SettingKind::Integer,  setContinentOctaves},
    {"domain_warp_amplitude",         SettingKind::Floating, setDomainWarpAmplitude},
    {"domain_warp_wavelength_meters", SettingKind::Floating, setDomainWarpWavelength},
    {"mountain_amplitude_meters",     SettingKind::Floating, setMountainAmplitude},
    {"mountain_wavelength_meters",    SettingKind::Floating, setMountainWavelength},
    {"mountain_mask_low",             SettingKind::Floating, setMountainMaskLow},
    {"mountain_mask_high",            SettingKind::Floating, setMountainMaskHigh},
    {"mountain_sharpness",            SettingKind::Floating, setMountainSharpness},
    {"mountain_octaves",              SettingKind::Integer,  setMountainOctaves},
    {"mountain_mass_fraction",        SettingKind::Floating, setMountainMassFraction},
    {"mountain_mass_wavelength_multiple", SettingKind::Floating, setMountainMassWavelengthMultiple},
    {"mountain_crest_mass_coupling",  SettingKind::Floating, setMountainCrestMassCoupling},
    {"range_cluster_strength",        SettingKind::Floating, setRangeClusterStrength},
    {"range_cluster_gate_low",        SettingKind::Floating, setRangeClusterGateLow},
    {"range_cluster_gate_high",       SettingKind::Floating, setRangeClusterGateHigh},
    {"range_cluster_wavelength_meters", SettingKind::Floating, setRangeClusterWavelength},
    {"mountain_erosion_weight",           SettingKind::Floating, setMountainErosionWeight},
    {"mountain_erosion_face_blend_width", SettingKind::Floating, setMountainErosionFaceBlendWidth},
    {"mountain_erosion_terrain_frequency", SettingKind::Floating, setMountainErosionTerrainFrequency},
    {"mountain_erosion_terrain_amplitude", SettingKind::Floating, setMountainErosionTerrainAmplitude},
    {"mountain_erosion_terrain_octaves",  SettingKind::Integer,  setMountainErosionTerrainOctaves},
    {"mountain_erosion_terrain_gain",     SettingKind::Floating, setMountainErosionTerrainGain},
    {"mountain_erosion_terrain_lacunarity", SettingKind::Floating, setMountainErosionTerrainLacunarity},
    {"mountain_erosion_height_offset_low", SettingKind::Floating, setMountainErosionHeightOffsetLow},
    {"mountain_erosion_height_offset_blend", SettingKind::Floating, setMountainErosionHeightOffsetBlend},
    {"mountain_erosion_scale",            SettingKind::Floating, setMountainErosionScale},
    {"mountain_erosion_strength",         SettingKind::Floating, setMountainErosionStrength},
    {"mountain_erosion_gully_weight",     SettingKind::Floating, setMountainErosionGullyWeight},
    {"mountain_erosion_detail",           SettingKind::Floating, setMountainErosionDetail},
    {"mountain_erosion_cell_scale",       SettingKind::Floating, setMountainErosionCellScale},
    {"mountain_erosion_normalization",    SettingKind::Floating, setMountainErosionNormalization},
    {"mountain_erosion_octaves",          SettingKind::Integer,  setMountainErosionOctaves},
    {"mountain_erosion_gain",             SettingKind::Floating, setMountainErosionGain},
    {"mountain_erosion_lacunarity",       SettingKind::Floating, setMountainErosionLacunarity},
    {"mountain_erosion_rounding_ridge",   SettingKind::Floating, setMountainErosionRoundingRidge},
    {"mountain_erosion_rounding_crease",  SettingKind::Floating, setMountainErosionRoundingCrease},
    {"mountain_erosion_rounding_global",  SettingKind::Floating, setMountainErosionRoundingGlobal},
    {"mountain_erosion_rounding_gain",    SettingKind::Floating, setMountainErosionRoundingGain},
    {"mountain_erosion_onset_input",      SettingKind::Floating, setMountainErosionOnsetInput},
    {"mountain_erosion_onset_octave",     SettingKind::Floating, setMountainErosionOnsetOctave},
    {"mountain_erosion_onset_ridge_map",  SettingKind::Floating, setMountainErosionOnsetRidgeMap},
    {"mountain_erosion_onset_ridge_mask", SettingKind::Floating, setMountainErosionOnsetRidgeMask},
    {"mountain_erosion_assumed_slope",        SettingKind::Floating, setMountainErosionAssumedSlope},
    {"mountain_erosion_assumed_slope_blend",  SettingKind::Floating, setMountainErosionAssumedSlopeBlend},
    {"mountain_erosion_profile_low",      SettingKind::Floating, setMountainErosionProfileLow},
    {"mountain_erosion_profile_high",     SettingKind::Floating, setMountainErosionProfileHigh},
    {"regional_amplitude_meters",     SettingKind::Floating, setRegionalAmplitude},
    {"regional_wavelength_meters",    SettingKind::Floating, setRegionalWavelength},
    {"regional_octaves",              SettingKind::Integer,  setRegionalOctaves},
    {"hill_amplitude_meters",         SettingKind::Floating, setHillAmplitude},
    {"hill_wavelength_meters",        SettingKind::Floating, setHillWavelength},
    {"hill_octaves",                  SettingKind::Integer,  setHillOctaves},
    {"detail_amplitude_meters",       SettingKind::Floating, setDetailAmplitude},
    {"detail_wavelength_meters",      SettingKind::Floating, setDetailWavelength},
    {"detail_plains_fraction",        SettingKind::Floating, setDetailPlainsFraction},
    {"detail_octaves",                SettingKind::Integer,  setDetailOctaves},
    {"terrace_strength",              SettingKind::Floating, setTerraceStrength},
    {"terrace_steps",                 SettingKind::Floating, setTerraceSteps},
    {"terrace_riser_fraction",        SettingKind::Floating, setTerraceRiserFraction},
    {"cliff_wavelength_meters",       SettingKind::Floating, setCliffWavelength},
    {"cliff_steepness_low",           SettingKind::Floating, setCliffSteepnessLow},
    {"cliff_steepness_high",          SettingKind::Floating, setCliffSteepnessHigh},
    {"cliff_sharpness",               SettingKind::Floating, setCliffSharpness},
    {"overhang_amplitude_meters",     SettingKind::Floating, setOverhangAmplitude},
    {"overhang_wavelength_meters",    SettingKind::Floating, setOverhangWavelength},
    {"overhang_plains_fraction",      SettingKind::Floating, setOverhangPlainsFraction},
    {"strata_amplitude_meters",       SettingKind::Floating, setStrataAmplitude},
    {"strata_band_meters",            SettingKind::Floating, setStrataBand},
    {"strata_warp",                   SettingKind::Floating, setStrataWarp},
    {"strata_gate_low",               SettingKind::Floating, setStrataGateLow},
    {"strata_gate_high",              SettingKind::Floating, setStrataGateHigh},
    {"provinces_enabled",             SettingKind::Boolean,  setProvincesEnabled},
    {"province_wavelength_meters",    SettingKind::Floating, setProvinceWavelength},
    {"province_rock_variation",       SettingKind::Floating, setProvinceRockVariation},
    {"province_strata_bias",          SettingKind::Floating, setProvinceStrataBias},
    {"province_terrace_bias",         SettingKind::Floating, setProvinceTerraceBias},
    {"coastal_flatten_meters",        SettingKind::Floating, setCoastalFlattenMeters},
    {"coastal_flatten_floor",         SettingKind::Floating, setCoastalFlattenFloor},
    {"coastal_flatten_full_fraction",  SettingKind::Floating, setCoastalFlattenFullFraction},
    {"coastal_flatten_reach_fraction", SettingKind::Floating, setCoastalFlattenReachFraction},
    {"caves_enabled",                 SettingKind::Boolean,  setCavesEnabled},
    {"cave_threshold",                SettingKind::Floating, setCaveThreshold},
    {"cave_worm_wavelength_meters",   SettingKind::Floating, setCaveWormWavelength},
    {"cave_carve_meters",             SettingKind::Floating, setCaveCarve},
    {"shallow_water_meters",          SettingKind::Floating, setShallowWater},
    {"deep_water_meters",             SettingKind::Floating, setDeepWater},
    {"water_depth_tint_meters",       SettingKind::Floating, setWaterDepthTint},
    {"water_shore_mix_meters",        SettingKind::Floating, setWaterShoreMix},
    {"equator_temperature",           SettingKind::Floating, setEquatorTemperature},
    {"pole_temperature",              SettingKind::Floating, setPoleTemperature},
    {"temperature_perturbation",      SettingKind::Floating, setTemperaturePerturbation},
    {"temperature_wavelength_meters", SettingKind::Floating, setTemperatureWavelength},
    {"temperature_lapse_meters",      SettingKind::Floating, setTemperatureLapse},
    {"moisture_wavelength_meters",    SettingKind::Floating, setMoistureWavelength},
    {"moisture_perturbation",         SettingKind::Floating, setMoisturePerturbation},
    {"moisture_warp_amplitude",       SettingKind::Floating, setMoistureWarpAmplitude},
    {"climate_continentality_strength", SettingKind::Floating, setClimateContinentality},
    {"climate_rain_shadow_strength",     SettingKind::Floating, setClimateRainShadow},
    {"climate_wind_zones_enabled",        SettingKind::Boolean,  setClimateWindZones},
    {"coastal_moisture_boost",        SettingKind::Floating, setCoastalMoistureBoost},
    {"coastal_moisture_band_meters",  SettingKind::Floating, setCoastalMoistureBand},
    {"moisture_latitude_bands",       SettingKind::Boolean,  setMoistureLatitudeBands},
    {"moisture_belt_equator_amplitude",     SettingKind::Floating, setMoistureBeltEquator},
    {"moisture_belt_subtropical_amplitude", SettingKind::Floating, setMoistureBeltSubtropical},
    {"moisture_belt_temperate_amplitude",   SettingKind::Floating, setMoistureBeltTemperate},
    {"moisture_belt_polar_amplitude",       SettingKind::Floating, setMoistureBeltPolar},
    {"color_variation",               SettingKind::Floating, setColorVariation},
    {"color_variation_wavelength_meters", SettingKind::Floating, setColorVariationWavelength},
    {"color_variation_ground_wavelength_meters", SettingKind::Floating, setColorVariationGroundWavelength},
    {"soil_patch_strength",           SettingKind::Floating, setSoilPatchStrength},
    {"soil_patch_wavelength_meters",  SettingKind::Floating, setSoilPatchWavelength},
    {"dry_patch_strength",            SettingKind::Floating, setDryPatchStrength},
    {"dry_patch_wavelength_meters",   SettingKind::Floating, setDryPatchWavelength},
    {"snow_line_meters",              SettingKind::Floating, setSnowLine},
    {"beach_width_meters",            SettingKind::Floating, setBeachWidth},
    {"beach_wet_width_meters",        SettingKind::Floating, setBeachWetWidth},
    {"beach_wet_darkening",           SettingKind::Floating, setBeachWetDarkening},
    {"rock_slope_threshold",          SettingKind::Floating, setRockSlopeThreshold},
    {"rock_strata_contrast",          SettingKind::Floating, setRockStrataContrast},
    {"terrain_texture_tile_meters_ground", SettingKind::Floating, setTerrainTextureTileGround},
    {"terrain_texture_tile_meters_rock",   SettingKind::Floating, setTerrainTextureTileRock},
    {"water_texture_tile_meters",          SettingKind::Floating, setWaterTextureTile},
    {"biome_blend_width",             SettingKind::Floating, setBiomeBlendWidth},
    {"slope_blend_width",             SettingKind::Floating, setSlopeBlendWidth},
    {"elevation_blend_fraction",      SettingKind::Floating, setElevationBlendFraction},
    {"water_enabled",                 SettingKind::Boolean,  setWaterEnabled},
    {"water_roughness",               SettingKind::Floating, setWaterRoughness},
    {"water_foam_width_meters",       SettingKind::Floating, setWaterFoamWidth},
    {"water_foam_strength",           SettingKind::Floating, setWaterFoamStrength},
    {"water_foam_falloff_exponent",   SettingKind::Floating, setWaterFoamFalloffExponent},
    {"clouds_enabled",                SettingKind::Boolean,  setCloudsEnabled},
    {"cloud_coverage",                SettingKind::Floating, setCloudCoverage},
    {"cloud_wavelength_meters",       SettingKind::Floating, setCloudWavelength},
    {"cloud_altitude_meters",         SettingKind::Floating, setCloudAltitude},
    {"cloud_detail_strength",         SettingKind::Floating, setCloudDetailStrength},
    {"cloud_edge_softness",           SettingKind::Floating, setCloudEdgeSoftness},
    {"cloud_weather_strength",        SettingKind::Floating, setCloudWeatherStrength},
    {"cloud_weather_wavelength_multiple", SettingKind::Floating, setCloudWeatherWavelengthMultiple},
    {"cloud_latitude_band_strength",  SettingKind::Floating, setCloudLatitudeBandStrength},
    {"cloud_band_frequency",          SettingKind::Floating, setCloudBandFrequency},
    {"cloud_shadow_strength",         SettingKind::Floating, setCloudShadowStrength},
    {"cloud_shadow_softness",         SettingKind::Floating, setCloudShadowSoftness},
    {"stars_enabled",                 SettingKind::Boolean,  setStarsEnabled},
    {"stars_density",                 SettingKind::Floating, setStarsDensity},
    {"stars_brightness",              SettingKind::Floating, setStarsBrightness},
    {"sea_ice_temperature",           SettingKind::Floating, setSeaIceTemperature},
    {"sea_ice_transition",            SettingKind::Floating, setSeaIceTransition},
    {"water_wave_amplitude",          SettingKind::Floating, setWaterWaveAmplitude},
    {"water_wave_wavelength_meters",  SettingKind::Floating, setWaterWaveWavelength},
    {"water_wave_chop_fraction",      SettingKind::Floating, setWaterWaveChopFraction},
    {"water_shore_sand_brightness",   SettingKind::Floating, setWaterShoreSandBrightness},
    {"surface_gravity_mps2",          SettingKind::Floating, setSurfaceGravity},
    {"walk_speed_mps",                SettingKind::Floating, setWalkSpeed},
    {"walk_sprint_multiplier",        SettingKind::Floating, setWalkSprintMultiplier},
    {"jump_speed_mps",                SettingKind::Floating, setJumpSpeed},
    {"eye_height_meters",             SettingKind::Floating, setEyeHeight},
    {"walk_ground_snap_meters",       SettingKind::Floating, setWalkGroundSnap},
    {"walk_max_step_up_meters",       SettingKind::Floating, setWalkMaxStepUp},
    {"walk_max_climb_grade",          SettingKind::Floating, setWalkMaxClimbGrade},
    {"walk_terminal_velocity_mps",    SettingKind::Floating, setWalkTerminalVelocity},
    {"swim_enabled",                  SettingKind::Boolean,  setSwimEnabled},
    {"swim_speed_mps",                SettingKind::Floating, setSwimSpeed},
    {"swim_wade_depth_meters",        SettingKind::Floating, setSwimWadeDepth},
    {"atmosphere_enabled",            SettingKind::Boolean,  setAtmosphereEnabled},
    {"atmosphere_height_meters",      SettingKind::Floating, setAtmosphereHeight},
    {"aerial_perspective_scale_fraction", SettingKind::Floating, setAerialPerspectiveScaleFraction},
    {"aerial_perspective_asymptote",  SettingKind::Floating, setAerialPerspectiveAsymptote},
    {"debug_chunk_borders",           SettingKind::Boolean,  setDebugChunkBorders},
    {"debug_chunk_border_width_meters", SettingKind::Floating, setDebugChunkBorderWidth},
};

// Reads the module's [planet] configuration section into one params struct.
// Every key is optional; missing keys keep the struct's documented default
// (which mirrors config.toml). Array keys (axis, palette, water colors) are
// read separately; scalar values are clamped by validate().
PlanetGenerationParams paramsFromConfiguration(ParticipantContext &context)
{
    PlanetGenerationParams params;

    for (const SettingRow &row : kSettingRows)
    {
        switch (row.kind)
        {
            case SettingKind::Floating:
                if (const auto value = context.configurationFloatingInSection("planet", row.key))
                {
                    row.setter(params, *value);
                }
                break;
            case SettingKind::Integer:
                if (const auto value = context.configurationIntegerInSection("planet", row.key))
                {
                    // 64-bit integer params (the seed) must keep their full
                    // width: funneling an int64 through a double loses the
                    // bits above 2^53. Rows with a direct int64 setter take
                    // the raw value; the rest are uint32-width fields whose
                    // setter range-checks before the cast.
                    if (row.int64Setter != nullptr)
                    {
                        row.int64Setter(params, *value);
                    }
                    else
                    {
                        row.setter(params, static_cast<double>(*value));
                    }
                }
                break;
            case SettingKind::Boolean:
                if (const auto value = context.configurationBooleanInSection("planet", row.key))
                {
                    row.setter(params, *value ? 1.0 : 0.0);
                }
                break;
        }
    }

    if (const auto array = context.configurationFloatingArrayInSection("planet", "axis"); array.size() == 3)
    {
        params.axis[0] = array[0];
        params.axis[1] = array[1];
        params.axis[2] = array[2];
    }
    if (const auto zenith = context.configurationFloatingArrayInSection("planet", "atmosphere_zenith_color"); zenith.size() == 3)
    {
        params.atmosphereZenithColor[0] = static_cast<float>(zenith[0]);
        params.atmosphereZenithColor[1] = static_cast<float>(zenith[1]);
        params.atmosphereZenithColor[2] = static_cast<float>(zenith[2]);
    }
    if (const auto horizon = context.configurationFloatingArrayInSection("planet", "atmosphere_horizon_color"); horizon.size() == 3)
    {
        params.atmosphereHorizonColor[0] = static_cast<float>(horizon[0]);
        params.atmosphereHorizonColor[1] = static_cast<float>(horizon[1]);
        params.atmosphereHorizonColor[2] = static_cast<float>(horizon[2]);
    }
    if (const auto palette = context.configurationFloatingArrayInSection("planet", "biome_palette"); !palette.empty())
    {
        const std::size_t count = std::min(palette.size(), static_cast<std::size_t>(biomeCount) * 3u);
        for (std::size_t index = 0; index < count; ++index)
        {
            params.biomePalette[index] = static_cast<float>(palette[index]);
        }
    }
    if (const auto water = context.configurationFloatingArrayInSection("planet", "water_color_shallow"); water.size() == 3)
    {
        params.waterColorShallow[0] = static_cast<float>(water[0]);
        params.waterColorShallow[1] = static_cast<float>(water[1]);
        params.waterColorShallow[2] = static_cast<float>(water[2]);
    }
    if (const auto water = context.configurationFloatingArrayInSection("planet", "water_color_deep"); water.size() == 3)
    {
        params.waterColorDeep[0] = static_cast<float>(water[0]);
        params.waterColorDeep[1] = static_cast<float>(water[1]);
        params.waterColorDeep[2] = static_cast<float>(water[2]);
    }
    if (const auto water = context.configurationFloatingArrayInSection("planet", "water_color_ice"); water.size() == 3)
    {
        params.waterColorIce[0] = static_cast<float>(water[0]);
        params.waterColorIce[1] = static_cast<float>(water[1]);
        params.waterColorIce[2] = static_cast<float>(water[2]);
    }
    if (const auto clouds = context.configurationFloatingArrayInSection("planet", "cloud_color"); clouds.size() == 3)
    {
        params.cloudColor[0] = static_cast<float>(clouds[0]);
        params.cloudColor[1] = static_cast<float>(clouds[1]);
        params.cloudColor[2] = static_cast<float>(clouds[2]);
    }
    if (const auto shadowSun = context.configurationFloatingArrayInSection("planet", "cloud_shadow_sun_direction");
        shadowSun.size() == 3)
    {
        params.cloudShadowSunDirection[0] = shadowSun[0];
        params.cloudShadowSunDirection[1] = shadowSun[1];
        params.cloudShadowSunDirection[2] = shadowSun[2];
    }

    return params;
}

Vec3 worldPointToPlanetLocal(const PlanetGenerationParams &params, const double *vector)
{
    return Vec3{vector[0] - params.centerMeters[0], vector[1] - params.centerMeters[1], vector[2] - params.centerMeters[2]};
}

// Module singleton backing the "planet" capability. The configured params act
// as the template for every created planet; live planets live in a handle map.
class PlanetModule final : public ModuleInterface
{
public:
    PlanetModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        templateParams_ = paramsFromConfiguration(context);
        std::string     error;
        if (!templateParams_.validate(error))
        {
            // Config hostile beyond repair: fall back to the documented
            // defaults so the module still constructs (createPlanet will
            // re-validate per description). The reason lands on stderr so a
            // mistuned config is diagnosable instead of silently replaced.
            std::fprintf(stderr, "planet module: [planet] configuration rejected (%s); using built-in defaults\n",
                error.c_str());
            templateParams_ = PlanetGenerationParams{};
            std::string fallbackError;
            templateParams_.validate(fallbackError);
        }
    }

    void start() override
    {
    }

    void stop() override
    {
        // Drop every live planet (each ~Planet drains its in-flight builds).
        std::lock_guard<std::mutex> lock(planetsMutex_);
        planets_.clear();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == PlanetCapability::capabilityName ? &capability_ : nullptr;
    }

    // ---- capability implementation -------------------------------------------------

    PlanetHandle createPlanet(const PlanetDescription *description, PlanetError *error)
    {
        if (description == nullptr)
        {
            setError(error, PlanetError::InvalidDescription);
            return invalidPlanet;
        }
        PlanetGenerationParams params = templateParams_;
        if (description->seed != 0)
        {
            params.seed = description->seed;
        }
        if (description->radiusMeters > 0.0)
        {
            params.radiusMeters = description->radiusMeters;
        }
        if (description->seaLevelOffsetMeters != 0.0)
        {
            params.seaLevelOffsetMeters = description->seaLevelOffsetMeters;
        }
        if (description->overrideCenter)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                params.centerMeters[axis] = description->centerMeters[axis];
            }
        }
        std::string validationError;
        if (!params.validate(validationError))
        {
            setError(error, PlanetError::InvalidDescription);
            return invalidPlanet;
        }

        // Each live planet owns a worker pool; the configured cap bounds how
        // many pools one module instance can stack. A cheap pre-check fails
        // fast under the lock; the decisive check and the insertion share
        // ONE critical section (and the handle counter lives under the same
        // lock), so concurrent createPlanet calls can never overshoot the
        // cap or collide on a handle. The planet itself is constructed
        // outside the lock (it spins up its worker pool).
        {
            std::lock_guard<std::mutex> lock(planetsMutex_);
            if (planets_.size() >= params.maxLivePlanets)
            {
                setError(error, PlanetError::CapacityReached);
                return invalidPlanet;
            }
        }
        auto         instance = std::make_unique<Planet>(params);
        PlanetHandle handle;
        {
            std::lock_guard<std::mutex> lock(planetsMutex_);
            if (planets_.size() >= params.maxLivePlanets)
            {
                setError(error, PlanetError::CapacityReached);
                return invalidPlanet;
            }
            handle = nextHandle_++;
            planets_.emplace(handle, std::move(instance));
        }
        setError(error, PlanetError::None);
        return handle;
    }

    void destroyPlanet(PlanetHandle handle)
    {
        // ~Planet drains in-flight builds before its field dies.
        std::lock_guard<std::mutex> lock(planetsMutex_);
        planets_.erase(handle);
    }

    Planet *planetFor(PlanetHandle handle) const
    {
        std::lock_guard<std::mutex> lock(planetsMutex_);
        const auto it = planets_.find(handle);
        return it != planets_.end() ? it->second.get() : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = PlanetCapability::capabilityName,
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {PlanetCapability::capabilityName}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createPlanet = [](void *self, const PlanetDescription *description, PlanetError *error)
        { return static_cast<PlanetModule *>(self)->createPlanet(description, error); };
        capability_.destroyPlanet = [](void *self, PlanetHandle handle)
        { static_cast<PlanetModule *>(self)->destroyPlanet(handle); };
        capability_.updatePlanet = [](void *self, PlanetHandle handle, const double eyeWorld[3])
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr)
            {
                return;
            }
            const Vec3 eyeLocal = worldPointToPlanetLocal(planet->params(), eyeWorld);
            planet->update(eyeLocal);
        };
        capability_.visibleChunkCapacity = [](void *self, PlanetHandle handle) -> std::uint32_t
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            return planet != nullptr ? planet->visibleChunkCapacity() : 0u;
        };
        capability_.copyVisibleChunks = [](void *self, PlanetHandle handle, PlanetChunkView *output, std::uint32_t capacity)
            -> std::uint32_t
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            return planet != nullptr ? planet->copyVisibleChunks(output, capacity) : 0u;
        };
        capability_.sampleSurfaceRadius = [](void *self, PlanetHandle handle, const double directionUnit[3], double *outRadius)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || directionUnit == nullptr || outRadius == nullptr)
            {
                return false;
            }
            const Vec3 direction{directionUnit[0], directionUnit[1], directionUnit[2]};
            const double lengthSquared = dot(direction, direction);
            if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12)
            {
                return false;
            }
            *outRadius = planet->surfaceRadius(direction * (1.0 / std::sqrt(lengthSquared)));
            return true;
        };
        capability_.sampleDensity = [](void *self, PlanetHandle handle, const double pointWorld[3], double *outDensity)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || pointWorld == nullptr || outDensity == nullptr)
            {
                return false;
            }
            // World coordinates -> planet-local (the center offset the module
            // applies to chunk placements).
            const Vec3 point = worldPointToPlanetLocal(planet->params(), pointWorld);
            *outDensity = planet->densityAt(point);
            return true;
        };
        capability_.sampleClimate = [](void *self, PlanetHandle handle, const double directionUnit[3], double radiusMeters,
            ClimateSample *outSample)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || directionUnit == nullptr || outSample == nullptr)
            {
                return false;
            }
            const Vec3 direction{directionUnit[0], directionUnit[1], directionUnit[2]};
            const double lengthSquared = dot(direction, direction);
            if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12)
            {
                return false;
            }
            *outSample = planet->sampleClimate(direction * (1.0 / std::sqrt(lengthSquared)), radiusMeters, 0.0);
            return true;
        };
        capability_.sampleRelief = [](void *self, PlanetHandle handle, const double directionUnit[3],
            ReliefSample *outRelief)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || directionUnit == nullptr || outRelief == nullptr)
            {
                return false;
            }
            const Vec3 direction{directionUnit[0], directionUnit[1], directionUnit[2]};
            const double lengthSquared = dot(direction, direction);
            if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12)
            {
                return false;
            }
            *outRelief = planet->sampleRelief(direction * (1.0 / std::sqrt(lengthSquared)));
            return true;
        };
        capability_.copyStatistics = [](void *self, PlanetHandle handle, PlanetStatistics *outStatistics)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || outStatistics == nullptr)
            {
                return false;
            }
            *outStatistics = planet->statistics();
            return true;
        };
        capability_.copyGenerationParams = [](void *self, PlanetHandle handle, PlanetGenerationParams *outParams)
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr || outParams == nullptr)
            {
                return false;
            }
            *outParams = planet->params();
            return true;
        };
        capability_.hashChunk = [](void *self, PlanetHandle handle, std::uint32_t depth, std::uint32_t x, std::uint32_t y,
            std::uint32_t z) -> std::uint64_t
        {
            Planet *planet = static_cast<PlanetModule *>(self)->planetFor(handle);
            if (planet == nullptr)
            {
                return 0;
            }
            // The chunk key packs the depth into 4 bits (see types.hpp);
            // out-of-range coordinates would hash a lattice the octree can
            // never address, so reject them instead of returning a hash of
            // numerically meaningless geometry. maxSupportedOctreeDepth
            // itself is addressable (the 4-bit field holds it), and each
            // coordinate must index a cell of that depth's lattice (2^depth
            // cells per axis) - anything beyond lies outside the root cube.
            if (depth > maxSupportedOctreeDepth)
            {
                return 0;
            }
            const std::uint32_t cellCount = std::uint32_t{1} << depth;
            if (x >= cellCount || y >= cellCount || z >= cellCount)
            {
                return 0;
            }
            return planet->buildChunkHash(ChunkCoord{depth, x, y, z});
        };
    }

    ParticipantManifest      manifest_;
    PlanetCapability         capability_{};
    PlanetGenerationParams   templateParams_{};
    mutable std::mutex                                             planetsMutex_;
    std::map<PlanetHandle, std::unique_ptr<Planet>>               planets_;
    PlanetHandle                                                  nextHandle_ = 1;
};

} // namespace
} // namespace planet
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "planet" capability; no
// required or optional capabilities, no declared permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"planet"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "planet",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::planet::PlanetModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
