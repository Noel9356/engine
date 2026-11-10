// planet module implementation.
//
// Bridges the pure generation core (Planet facade, see planet.hpp) into the
// engine's capability system: reads the [planet] configuration section once
// in construct(), hands out per-planets through createPlanet (each layering
// its PlanetDescription overrides over the configured defaults), and streams
// visible chunk meshes to consumers as borrowed views.

#include <engine/planet/capabilities.hpp>
#include <engine/planet/planet.hpp>

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

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

// Reads the module's [planet] configuration section into one params struct.
// Every key is optional; missing keys keep the struct's documented default
// (which mirrors config.toml). Values are clamped by validate().
PlanetGenerationParams paramsFromConfiguration(ParticipantContext &context)
{
    PlanetGenerationParams params;

    const auto integer = [&context](const char *key) { return context.configurationIntegerInSection("planet", key); };
    const auto floating = [&context](const char *key) { return context.configurationFloatingInSection("planet", key); };
    const auto boolean = [&context](const char *key) { return context.configurationBooleanInSection("planet", key); };

    if (const auto value = integer("seed"))
    {
        params.seed = static_cast<std::uint64_t>(*value);
    }
    if (const auto value = floating("radius_meters"))
    {
        params.radiusMeters = *value;
    }
    if (const auto value = floating("sea_level_offset_meters"))
    {
        params.seaLevelOffsetMeters = *value;
    }
    if (const auto array = context.configurationFloatingArrayInSection("planet", "axis"); array.size() == 3)
    {
        params.axis[0] = array[0];
        params.axis[1] = array[1];
        params.axis[2] = array[2];
    }

    if (const auto value = integer("leaf_grid_n"))
    {
        params.leafGridN = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = floating("target_voxel_meters"))
    {
        params.targetVoxelMeters = *value;
    }
    if (const auto value = integer("max_depth_limit"))
    {
        params.maxDepthLimit = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = floating("lod_angular_voxel_radians"))
    {
        params.lodAngularVoxelRadians = *value;
    }
    if (const auto value = floating("lod_band_limit_factor"))
    {
        params.lodBandLimitFactor = *value;
    }
    if (const auto value = boolean("horizon_culling"))
    {
        params.horizonCulling = *value;
    }
    if (const auto value = floating("bounding_factor"))
    {
        params.boundingFactor = *value;
    }
    if (const auto value = integer("max_concurrent_builds"))
    {
        params.maxConcurrentBuilds = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = integer("max_new_builds_per_update"))
    {
        params.maxNewBuildsPerUpdate = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = integer("chunk_eviction_updates"))
    {
        params.chunkEvictionUpdates = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = integer("chunk_cache_capacity"))
    {
        params.chunkCacheCapacity = static_cast<std::uint32_t>(*value);
    }

    if (const auto value = floating("continent_amplitude_meters"))
    {
        params.continentAmplitudeMeters = *value;
    }
    if (const auto value = floating("continent_wavelength_meters"))
    {
        params.continentWavelengthMeters = *value;
    }
    if (const auto value = integer("continent_octaves"))
    {
        params.continentsFbm.octaves = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = floating("domain_warp_amplitude"))
    {
        params.domainWarpAmplitude = *value;
    }
    if (const auto value = floating("domain_warp_wavelength_meters"))
    {
        params.domainWarpWavelengthMeters = *value;
    }

    if (const auto value = floating("mountain_amplitude_meters"))
    {
        params.mountainAmplitudeMeters = *value;
    }
    if (const auto value = floating("mountain_wavelength_meters"))
    {
        params.mountainWavelengthMeters = *value;
    }
    if (const auto value = floating("mountain_mask_low"))
    {
        params.mountainMaskLow = *value;
    }
    if (const auto value = floating("mountain_mask_high"))
    {
        params.mountainMaskHigh = *value;
    }
    if (const auto value = floating("mountain_sharpness"))
    {
        params.mountainSharpness = *value;
    }
    if (const auto value = integer("mountain_octaves"))
    {
        params.mountainsFbm.octaves = static_cast<std::uint32_t>(*value);
    }
    if (const auto value = floating("mountain_mass_fraction"))
    {
        params.mountainMassFraction = *value;
    }
    if (const auto value = floating("range_cluster_strength"))
    {
        params.rangeClusterStrength = *value;
    }
    if (const auto value = floating("range_cluster_wavelength_meters"))
    {
        params.rangeClusterWavelengthMeters = *value;
    }

    if (const auto value = floating("regional_amplitude_meters"))
    {
        params.regionalAmplitudeMeters = *value;
    }
    if (const auto value = floating("regional_wavelength_meters"))
    {
        params.regionalWavelengthMeters = *value;
    }
    if (const auto value = integer("regional_octaves"))
    {
        params.regionalFbm.octaves = static_cast<std::uint32_t>(*value);
    }

    if (const auto value = floating("hill_amplitude_meters"))
    {
        params.hillAmplitudeMeters = *value;
    }
    if (const auto value = floating("hill_wavelength_meters"))
    {
        params.hillWavelengthMeters = *value;
    }
    if (const auto value = integer("hill_octaves"))
    {
        params.hillsFbm.octaves = static_cast<std::uint32_t>(*value);
    }

    if (const auto value = floating("detail_amplitude_meters"))
    {
        params.detailAmplitudeMeters = *value;
    }
    if (const auto value = floating("detail_wavelength_meters"))
    {
        params.detailWavelengthMeters = *value;
    }
    if (const auto value = floating("detail_plains_fraction"))
    {
        params.detailPlainsFraction = *value;
    }
    if (const auto value = integer("detail_octaves"))
    {
        params.detailFbm.octaves = static_cast<std::uint32_t>(*value);
    }

    if (const auto value = floating("terrace_strength"))
    {
        params.terraceStrength = *value;
    }
    if (const auto value = floating("terrace_steps"))
    {
        params.terraceSteps = *value;
    }
    if (const auto value = floating("cliff_wavelength_meters"))
    {
        params.cliffWavelengthMeters = *value;
    }

    if (const auto value = floating("overhang_amplitude_meters"))
    {
        params.overhangAmplitudeMeters = *value;
    }
    if (const auto value = floating("overhang_wavelength_meters"))
    {
        params.overhangWavelengthMeters = *value;
    }
    if (const auto value = floating("overhang_plains_fraction"))
    {
        params.overhangPlainsFraction = *value;
    }
    if (const auto value = floating("strata_amplitude_meters"))
    {
        params.strataAmplitudeMeters = *value;
    }
    if (const auto value = floating("strata_band_meters"))
    {
        params.strataBandMeters = *value;
    }
    if (const auto value = floating("strata_warp"))
    {
        params.strataWarp = *value;
    }

    if (const auto value = floating("coastal_flatten_meters"))
    {
        params.coastalFlattenMeters = *value;
    }
    if (const auto value = floating("coastal_flatten_floor"))
    {
        params.coastalFlattenFloor = *value;
    }

    if (const auto value = boolean("caves_enabled"))
    {
        params.cavesEnabled = *value;
    }
    if (const auto value = floating("cave_threshold"))
    {
        params.caveThreshold = *value;
    }
    if (const auto value = floating("cave_worm_wavelength_meters"))
    {
        params.caveWormWavelengthMeters = *value;
    }
    if (const auto value = floating("cave_carve_meters"))
    {
        params.caveCarveMeters = *value;
    }

    if (const auto value = floating("shallow_water_meters"))
    {
        params.shallowWaterMeters = *value;
    }
    if (const auto value = floating("deep_water_meters"))
    {
        params.deepWaterMeters = *value;
    }
    if (const auto value = floating("water_depth_tint_meters"))
    {
        params.waterDepthTintMeters = *value;
    }

    if (const auto value = floating("equator_temperature"))
    {
        params.equatorTemperature = *value;
    }
    if (const auto value = floating("pole_temperature"))
    {
        params.poleTemperature = *value;
    }
    if (const auto value = floating("temperature_perturbation"))
    {
        params.temperaturePerturbation = *value;
    }
    if (const auto value = floating("temperature_wavelength_meters"))
    {
        params.temperatureWavelengthMeters = *value;
    }
    if (const auto value = floating("temperature_lapse_meters"))
    {
        params.temperatureLapseMeters = *value;
    }
    if (const auto value = floating("moisture_wavelength_meters"))
    {
        params.moistureWavelengthMeters = *value;
    }
    if (const auto value = floating("moisture_perturbation"))
    {
        params.moisturePerturbation = *value;
    }
    if (const auto value = floating("moisture_warp_amplitude"))
    {
        params.moistureWarpAmplitude = *value;
    }
    if (const auto value = floating("coastal_moisture_boost"))
    {
        params.coastalMoistureBoost = *value;
    }
    if (const auto value = floating("coastal_moisture_band_meters"))
    {
        params.coastalMoistureBandMeters = *value;
    }
    if (const auto value = boolean("moisture_latitude_bands"))
    {
        params.moistureLatitudeBands = *value;
    }
    if (const auto value = floating("color_variation"))
    {
        params.colorVariation = *value;
    }
    if (const auto value = floating("snow_line_meters"))
    {
        params.snowLineMeters = *value;
    }
    if (const auto value = floating("beach_width_meters"))
    {
        params.beachWidthMeters = *value;
    }
    if (const auto value = floating("rock_slope_threshold"))
    {
        params.rockSlopeThreshold = *value;
    }
    if (const auto value = floating("rock_strata_contrast"))
    {
        params.rockStrataContrast = *value;
    }
    if (const auto value = floating("biome_blend_width"))
    {
        params.biomeBlendWidth = *value;
    }
    if (const auto value = floating("slope_blend_width"))
    {
        params.slopeBlendWidth = *value;
    }
    if (const auto value = floating("elevation_blend_fraction"))
    {
        params.elevationBlendFraction = *value;
    }
    if (const auto palette = context.configurationFloatingArrayInSection("planet", "biome_palette"); !palette.empty())
    {
        const std::size_t count = std::min(palette.size(), static_cast<std::size_t>(biomeCount) * 3u);
        for (std::size_t index = 0; index < count; ++index)
        {
            params.biomePalette[index] = static_cast<float>(palette[index]);
        }
    }

    if (const auto value = boolean("water_enabled"))
    {
        params.waterEnabled = *value;
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
    if (const auto value = floating("water_roughness"))
    {
        params.waterRoughness = static_cast<float>(*value);
    }

    return params;
}

Vec3 worldDirectionToPlanetLocal(const PlanetGenerationParams &params, const double *vector)
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
        context_       = &context;
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

        TaskScheduler &scheduler = context_ != nullptr ? context_->taskScheduler() : unusedScheduler_;
        auto           instance  = std::make_unique<Planet>(params, &scheduler);
        const PlanetHandle handle = nextHandle_++;
        {
            std::lock_guard<std::mutex> lock(planetsMutex_);
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
            const Vec3 eyeLocal = worldDirectionToPlanetLocal(planet->params(), eyeWorld);
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
    TaskScheduler                                                 unusedScheduler_{}; // construct()-only fallback
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
