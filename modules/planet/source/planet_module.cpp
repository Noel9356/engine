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
};

void setSeed(PlanetGenerationParams &params, double value)
{
    // Seeds are hashed through 64-bit streams; the wrap of a negative
    // integer is as deterministic as any other seed.
    params.seed = static_cast<std::uint64_t>(static_cast<std::int64_t>(value));
}

void setRadius(PlanetGenerationParams &params, double value) { params.radiusMeters = value; }
void setSeaLevel(PlanetGenerationParams &params, double value) { params.seaLevelOffsetMeters = value; }
void setAbyssalDepth(PlanetGenerationParams &params, double value) { params.abyssalDepthMeters = value; }
void setLeafGridN(PlanetGenerationParams &params, double value) { params.leafGridN = static_cast<std::uint32_t>(value); }
void setTargetVoxel(PlanetGenerationParams &params, double value) { params.targetVoxelMeters = value; }
void setMaxDepthLimit(PlanetGenerationParams &params, double value) { params.maxDepthLimit = static_cast<std::uint32_t>(value); }
void setLodAngularVoxel(PlanetGenerationParams &params, double value) { params.lodAngularVoxelRadians = value; }
void setLodBandLimit(PlanetGenerationParams &params, double value) { params.lodBandLimitFactor = value; }
void setHorizonCulling(PlanetGenerationParams &params, double value) { params.horizonCulling = value != 0.0; }
void setBoundingFactor(PlanetGenerationParams &params, double value) { params.boundingFactor = value; }
void setMaxConcurrentBuilds(PlanetGenerationParams &params, double value) { params.maxConcurrentBuilds = static_cast<std::uint32_t>(value); }
void setMaxNewBuildsPerUpdate(PlanetGenerationParams &params, double value) { params.maxNewBuildsPerUpdate = static_cast<std::uint32_t>(value); }
void setMaxLivePlanets(PlanetGenerationParams &params, double value) { params.maxLivePlanets = static_cast<std::uint32_t>(value); }
void setChunkEvictionUpdates(PlanetGenerationParams &params, double value) { params.chunkEvictionUpdates = static_cast<std::uint32_t>(value); }
void setChunkCacheCapacity(PlanetGenerationParams &params, double value) { params.chunkCacheCapacity = static_cast<std::uint32_t>(value); }
void setContinentAmplitude(PlanetGenerationParams &params, double value) { params.continentAmplitudeMeters = value; }
void setContinentWavelength(PlanetGenerationParams &params, double value) { params.continentWavelengthMeters = value; }
void setContinentOctaves(PlanetGenerationParams &params, double value) { params.continentsFbm.octaves = static_cast<std::uint32_t>(value); }
void setDomainWarpAmplitude(PlanetGenerationParams &params, double value) { params.domainWarpAmplitude = value; }
void setDomainWarpWavelength(PlanetGenerationParams &params, double value) { params.domainWarpWavelengthMeters = value; }
void setMountainAmplitude(PlanetGenerationParams &params, double value) { params.mountainAmplitudeMeters = value; }
void setMountainWavelength(PlanetGenerationParams &params, double value) { params.mountainWavelengthMeters = value; }
void setMountainMaskLow(PlanetGenerationParams &params, double value) { params.mountainMaskLow = value; }
void setMountainMaskHigh(PlanetGenerationParams &params, double value) { params.mountainMaskHigh = value; }
void setMountainSharpness(PlanetGenerationParams &params, double value) { params.mountainSharpness = value; }
void setMountainOctaves(PlanetGenerationParams &params, double value) { params.mountainsFbm.octaves = static_cast<std::uint32_t>(value); }
void setMountainMassFraction(PlanetGenerationParams &params, double value) { params.mountainMassFraction = value; }
void setRangeClusterStrength(PlanetGenerationParams &params, double value) { params.rangeClusterStrength = value; }
void setRangeClusterWavelength(PlanetGenerationParams &params, double value) { params.rangeClusterWavelengthMeters = value; }
void setRegionalAmplitude(PlanetGenerationParams &params, double value) { params.regionalAmplitudeMeters = value; }
void setRegionalWavelength(PlanetGenerationParams &params, double value) { params.regionalWavelengthMeters = value; }
void setRegionalOctaves(PlanetGenerationParams &params, double value) { params.regionalFbm.octaves = static_cast<std::uint32_t>(value); }
void setHillAmplitude(PlanetGenerationParams &params, double value) { params.hillAmplitudeMeters = value; }
void setHillWavelength(PlanetGenerationParams &params, double value) { params.hillWavelengthMeters = value; }
void setHillOctaves(PlanetGenerationParams &params, double value) { params.hillsFbm.octaves = static_cast<std::uint32_t>(value); }
void setDetailAmplitude(PlanetGenerationParams &params, double value) { params.detailAmplitudeMeters = value; }
void setDetailWavelength(PlanetGenerationParams &params, double value) { params.detailWavelengthMeters = value; }
void setDetailPlainsFraction(PlanetGenerationParams &params, double value) { params.detailPlainsFraction = value; }
void setDetailOctaves(PlanetGenerationParams &params, double value) { params.detailFbm.octaves = static_cast<std::uint32_t>(value); }
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
void setCoastalFlattenMeters(PlanetGenerationParams &params, double value) { params.coastalFlattenMeters = value; }
void setCoastalFlattenFloor(PlanetGenerationParams &params, double value) { params.coastalFlattenFloor = value; }
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
void setSwimEnabled(PlanetGenerationParams &params, double value) { params.swimEnabled = value != 0.0; }
void setSwimSpeed(PlanetGenerationParams &params, double value) { params.swimSpeedMetersPerSecond = value; }
void setSwimWadeDepth(PlanetGenerationParams &params, double value) { params.swimWadeDepthMeters = value; }
void setAtmosphereEnabled(PlanetGenerationParams &params, double value) { params.atmosphereEnabled = value != 0.0; }
void setAtmosphereHeight(PlanetGenerationParams &params, double value) { params.atmosphereHeightMeters = value; }
void setCliffSteepnessLow(PlanetGenerationParams &params, double value) { params.cliffSteepnessLow = value; }
void setCliffSteepnessHigh(PlanetGenerationParams &params, double value) { params.cliffSteepnessHigh = value; }
void setCliffSharpness(PlanetGenerationParams &params, double value) { params.cliffSharpness = value; }
void setBeachWetWidth(PlanetGenerationParams &params, double value) { params.beachWetWidthMeters = value; }
void setBeachWetDarkening(PlanetGenerationParams &params, double value) { params.beachWetDarkening = value; }
void setWaterFoamWidth(PlanetGenerationParams &params, double value) { params.waterFoamWidthMeters = value; }
void setWaterFoamStrength(PlanetGenerationParams &params, double value) { params.waterFoamStrength = value; }
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
void setStarsEnabled(PlanetGenerationParams &params, double value) { params.starsEnabled = value != 0.0; }
void setStarsDensity(PlanetGenerationParams &params, double value) { params.starsDensity = value; }
void setStarsBrightness(PlanetGenerationParams &params, double value) { params.starsBrightness = value; }

constexpr SettingRow kSettingRows[] = {
    {"seed",                      SettingKind::Integer, setSeed},
    {"radius_meters",             SettingKind::Floating, setRadius},
    {"sea_level_offset_meters",   SettingKind::Floating, setSeaLevel},
    {"abyssal_depth_meters",      SettingKind::Floating, setAbyssalDepth},
    {"leaf_grid_n",               SettingKind::Integer, setLeafGridN},
    {"target_voxel_meters",       SettingKind::Floating, setTargetVoxel},
    {"max_depth_limit",           SettingKind::Integer, setMaxDepthLimit},
    {"lod_angular_voxel_radians", SettingKind::Floating, setLodAngularVoxel},
    {"lod_band_limit_factor",     SettingKind::Floating, setLodBandLimit},
    {"horizon_culling",           SettingKind::Boolean,  setHorizonCulling},
    {"bounding_factor",           SettingKind::Floating, setBoundingFactor},
    {"max_concurrent_builds",     SettingKind::Integer, setMaxConcurrentBuilds},
    {"max_new_builds_per_update", SettingKind::Integer, setMaxNewBuildsPerUpdate},
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
    {"range_cluster_strength",        SettingKind::Floating, setRangeClusterStrength},
    {"range_cluster_wavelength_meters", SettingKind::Floating, setRangeClusterWavelength},
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
    {"coastal_flatten_meters",        SettingKind::Floating, setCoastalFlattenMeters},
    {"coastal_flatten_floor",         SettingKind::Floating, setCoastalFlattenFloor},
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
    {"biome_blend_width",             SettingKind::Floating, setBiomeBlendWidth},
    {"slope_blend_width",             SettingKind::Floating, setSlopeBlendWidth},
    {"elevation_blend_fraction",      SettingKind::Floating, setElevationBlendFraction},
    {"water_enabled",                 SettingKind::Boolean,  setWaterEnabled},
    {"water_roughness",               SettingKind::Floating, setWaterRoughness},
    {"water_foam_width_meters",       SettingKind::Floating, setWaterFoamWidth},
    {"water_foam_strength",           SettingKind::Floating, setWaterFoamStrength},
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
    {"stars_enabled",                 SettingKind::Boolean,  setStarsEnabled},
    {"stars_density",                 SettingKind::Floating, setStarsDensity},
    {"stars_brightness",              SettingKind::Floating, setStarsBrightness},
    {"sea_ice_temperature",           SettingKind::Floating, setSeaIceTemperature},
    {"sea_ice_transition",            SettingKind::Floating, setSeaIceTransition},
    {"water_wave_amplitude",          SettingKind::Floating, setWaterWaveAmplitude},
    {"water_wave_wavelength_meters",  SettingKind::Floating, setWaterWaveWavelength},
    {"surface_gravity_mps2",          SettingKind::Floating, setSurfaceGravity},
    {"walk_speed_mps",                SettingKind::Floating, setWalkSpeed},
    {"walk_sprint_multiplier",        SettingKind::Floating, setWalkSprintMultiplier},
    {"jump_speed_mps",                SettingKind::Floating, setJumpSpeed},
    {"eye_height_meters",             SettingKind::Floating, setEyeHeight},
    {"walk_ground_snap_meters",       SettingKind::Floating, setWalkGroundSnap},
    {"walk_max_step_up_meters",       SettingKind::Floating, setWalkMaxStepUp},
    {"walk_max_climb_grade",          SettingKind::Floating, setWalkMaxClimbGrade},
    {"swim_enabled",                  SettingKind::Boolean,  setSwimEnabled},
    {"swim_speed_mps",                SettingKind::Floating, setSwimSpeed},
    {"swim_wade_depth_meters",        SettingKind::Floating, setSwimWadeDepth},
    {"atmosphere_enabled",            SettingKind::Boolean,  setAtmosphereEnabled},
    {"atmosphere_height_meters",      SettingKind::Floating, setAtmosphereHeight},
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
                    row.setter(params, static_cast<double>(*value));
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
        context_        = &context;
        templateParams_ = paramsFromConfiguration(context);
        std::string     error;
        if (!templateParams_.validate(error))
        {
            // Config hostile beyond repair: fall back to the documented
            // defaults so the module still constructs (createPlanet will
            // re-validate per description).
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
            const auto &params = planet->params();
            const Vec3 point{pointWorld[0] - params.centerMeters[0],
                pointWorld[1] - params.centerMeters[1],
                pointWorld[2] - params.centerMeters[2]};
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
    ParticipantContext      *context_ = nullptr;
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
