// radiance_cache module implementation.
//
// Binds the "radiance_cache" capability: a registry of GI scene surfaces under one
// mutex, a camera-driven evaluate() that runs the Lumen-style pipeline stages
// on the kernel task scheduler (surface cache capture, amortized direct and
// indirect lighting, world-space radiance cache, screen-space final gather),
// a probe-grid snapshot copy API, device profile resolution, statistics, and
// debug-view exports. The heavy lifting lives in radiance_cache_scene.cpp,
// radiance_cache_lighting.cpp, radiance_cache_debug.cpp, and radiance_cache_debug_views.cpp.
//
// Lumen reference: the stage structure, budgets, and blending behaviors
// follow Epic's published Lumen design ("Lumen: Real-time Global Illumination
// in Unreal Engine 5", SIGGRAPH 2022 Advances course; "Lumen Technical
// Details", Epic Games documentation) scaled to a CPU implementation.

#include <cstdint>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/radiance_cache/capabilities.hpp>

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/device_profile.hpp>
#include <engine/util/hardware_info.hpp>
#include <engine/util/ppm.hpp>
#include "radiance_cache_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace engine
{
namespace radiance_cache
{

namespace
{

// Writes an error code through an optional out-parameter.
void setError(GiError *output, GiError value)
{
    if (output != nullptr)
    {
        *output = value;
    }
}

glm::mat4 makeTransform(const float *columnMajor16)
{
    return glm::make_mat4(columnMajor16);
}

// Quality presets and their budget scaling come from
// engine/renderer/lighting_ray_core.hpp (shared with reflection_field); Auto
// derives its scale from the host's logical processor count, which this
// module queries once at construction.

void applySettingsToState(const GiSettings &settings, GiPipelineState &state, float autoScale)
{
    state.settings = settings;
    const float scale = settings.quality == 0 ? autoScale : qualityScale(settings.quality, 0u);
    // lighting.update_speed scales the two lighting refresh budgets on top of
    // the quality preset (Lumen's "Lumen Scene Lighting Update Speed").
    const double lightingScale = static_cast<double>(scale) * static_cast<double>(settings.lightingUpdateSpeed);
    state.settings.captureTexelsPerFrame  = std::max<std::uint64_t>(4096,
        static_cast<std::uint64_t>(static_cast<double>(settings.captureTexelsPerFrame) * scale));
    state.settings.directTexelsPerFrame   = std::max<std::uint64_t>(1024,
        static_cast<std::uint64_t>(static_cast<double>(settings.directTexelsPerFrame) * lightingScale));
    state.settings.indirectTexelsPerFrame = std::max<std::uint64_t>(1024,
        static_cast<std::uint64_t>(static_cast<double>(settings.indirectTexelsPerFrame) * lightingScale));
    state.settings.probeUpdatesPerFrame =
        std::max(4u, static_cast<std::uint32_t>(static_cast<float>(settings.probeUpdatesPerFrame) * scale));
    // Caustic photon counts scale with the quality preset like every other
    // amortized budget (a floor keeps every tier converging).
    state.settings.causticsPhotonsPerFrame = std::max(64u,
        static_cast<std::uint32_t>(static_cast<float>(settings.causticsPhotonsPerFrame) * scale));
    // The exported gather field follows the same scaling: the probe grid
    // coarsens (gatherScreenScale grows by the inverse of the scale) and the
    // field itself drops one resolution step, so weak devices keep the
    // contact-scale benefit at a fraction of the rays. Both floors keep the
    // lowest tier functional.
    state.settings.gatherScreenScale = std::min(64u,
        std::max(8u, static_cast<std::uint32_t>(std::lround(static_cast<float>(settings.gatherScreenScale) / std::max(scale, 0.25f)))));
    state.settings.gatherFieldScale = std::min(4u,
        std::max(1u, settings.gatherFieldScale + (scale < 0.5f ? 1u : 0u)));
}


// The module singleton backing the "radiance_cache" capability. moduleMutex_ guards
// the registry, settings, and the whole evaluate() call; statistics live in
// the pipeline state (atomic counters for the parallel stages). The probe
// grid is published as an immutable snapshot, so copyProbes never waits for a
// running evaluate().
class RadianceCacheModule final : public ModuleInterface
{
public:
    RadianceCacheModule()
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
        context_ = &context;
        // Optional lighting sources: without them the module uses the same
        // built-in defaults as the renderer.
        pbrCapability_          = context.tryAcquireCapability<pbr::PbrCapability>();
        directLightingCapability_ = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();
        reloadSubscription_     = context.eventBus()
                                .channel<ConfigurationReloadedEvent>()
                                .subscribe<RadianceCacheModule, &RadianceCacheModule::onConfigurationReloaded>(this);
        {
            std::lock_guard<std::mutex> lock(moduleMutex_);
            readConfigurationLocked();
            rebuildPipelineAfterConfigChangeLocked();
        }
    }

    void start() override
    {}

    void stop() override
    {
        if (context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
        }
        context_ = nullptr;
        std::lock_guard<std::mutex> lock(moduleMutex_);
        state_.surfaces.clear();
        surfaceIndexByHandle_.clear();
        state_.cardRefs.clear();
        state_.tlas.clear();
        state_.tlasSurfaces.clear();
        state_.hasView          = false;
        state_.environmentValid = false;
        // The stopped module owns nothing: the probe levels, their direction
        // tables, and the gather-field view buffers are dropped too, so the
        // snapshots published below report an empty cache instead of the
        // last lit frame under a bumped version. A restart rebuilds every
        // one of them from the configuration.
        state_.level0                       = GiProbeLevel{};
        state_.level1                       = GiProbeLevel{};
        state_.probeDirections.clear();
        state_.probeDirectionWeights.clear();
        state_.probeDirectionBasis.clear();
        state_.screenProbeDirections.clear();
        state_.screenProbeDirectionWeights.clear();
        state_.screenProbeDirectionBasis.clear();
        state_.viewWidth                = 0;
        state_.viewHeight               = 0;
        state_.gatherViewportWidth      = 0;
        state_.gatherViewportHeight     = 0;
        state_.screenProbeWidth         = 0;
        state_.screenProbeHeight        = 0;
        state_.gatherHasHistory         = false;
        state_.sceneDirty               = true;
        state_.causticTransmittersDirty = true;
        state_.causticEmissiveCardsDirty = true;
        publishSnapshotLocked();
        publishGatherSnapshotLocked();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        if (name == RadianceCacheCapability::capabilityName)
        {
            return &capability_;
        }
        if (name == RadianceGatherCapability::capabilityName)
        {
            return &gatherCapability_;
        }
        return nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                    = "radiance_cache",
            .kind                    = ParticipantKind::EngineModule,
            .version                 = "0.1.0",
            .providedCapabilities    = {"radiance_cache", "radiance_cache.gather"},
            .optionalCapabilities    = {"pbr", "direct_lighting"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createSurface = [](void *self, const GiSurfaceDescription *description, GiError *error)
        { return static_cast<RadianceCacheModule *>(self)->createSurfaceImpl(description, error); };
        capability_.updateSurface = [](void *self, GiSurfaceHandle surface, const GiSurfaceDescription *description,
                                      GiError *error)
        { return static_cast<RadianceCacheModule *>(self)->updateSurfaceImpl(surface, description, error); };
        capability_.updateSurfaceTransform = [](void *self, GiSurfaceHandle surface, const float *transform)
        { return static_cast<RadianceCacheModule *>(self)->updateSurfaceTransformImpl(surface, transform); };
        capability_.destroySurface = [](void *self, GiSurfaceHandle surface)
        { return static_cast<RadianceCacheModule *>(self)->destroySurfaceImpl(surface); };
        capability_.setView = [](void *self, const float *viewProjection, const float *cameraPosition)
        { return static_cast<RadianceCacheModule *>(self)->setViewImpl(viewProjection, cameraPosition); };
        capability_.evaluate = [](void *self, double deltaSeconds, GiError *error)
        { return static_cast<RadianceCacheModule *>(self)->evaluateImpl(deltaSeconds, error); };
        capability_.copyProbes = [](void                          *self,
                                  GiProbe                        *output,
                                  std::uint32_t                   capacity,
                                  GiProbeHeader                  *outHeader,
                                  std::uint64_t                  *outVersion)
        { return static_cast<RadianceCacheModule *>(self)->copyProbesImpl(output, capacity, outHeader, outVersion); };
        capability_.statistics = [](void *self, GiStatistics *outStatistics)
        { return static_cast<RadianceCacheModule *>(self)->statisticsImpl(outStatistics); };
        capability_.resolveProfile = [](void *self, const renderer::DeviceProfile *device, GiProfile *outProfile)
        { return static_cast<RadianceCacheModule *>(self)->resolveProfileImpl(device, outProfile); };
        capability_.debugExport = [](void *self, GiDebugView view, const char *filePath, GiError *error)
        { return static_cast<RadianceCacheModule *>(self)->debugExportImpl(view, filePath, error); };
        capability_.sampleCausticsIrradiance
            = [](void *self, const float *position, const float *normal, float *outIrradiance)
        { return static_cast<RadianceCacheModule *>(self)->sampleCausticsIrradianceImpl(position, normal, outIrradiance); };

        gatherCapability_.self = this;
        gatherCapability_.setViewport = [](void *self, std::uint32_t viewportWidth, std::uint32_t viewportHeight)
        { return static_cast<RadianceCacheModule *>(self)->setGatherViewportImpl(viewportWidth, viewportHeight); };
        gatherCapability_.copyIrradianceField
            = [](void *self, float *output, std::uint32_t capacityTexels, RadianceGatherHeader *outHeader,
                  std::uint64_t *outVersion)
        { return static_cast<RadianceCacheModule *>(self)->copyIrradianceFieldImpl(
            output, capacityTexels, outHeader, outVersion); };
    }

    // ---- configuration ------------------------------------------------------

    void readConfigurationLocked()
    {
        GiSettings settings;
        auto integer = [&](const char *key, std::int64_t fallback, std::int64_t minimum, std::int64_t maximum)
        {
            const auto value = context_->configurationIntegerInSection("radiance_cache", key);
            if (!value.has_value() || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto floating = [&](const char *key, double fallback, double minimum, double maximum)
        {
            const auto value = context_->configurationFloatingInSection("radiance_cache", key);
            if (!value.has_value() || !std::isfinite(*value) || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto boolean = [&](const char *key, bool fallback)
        {
            const auto value = context_->configurationBooleanInSection("radiance_cache", key);
            return value.has_value() ? *value : fallback;
        };
        auto environmentVector = [&](const char *key, glm::vec3 fallback, bool nonNegative)
        {
            const auto values = context_->configurationFloatingArrayInSection("radiance_cache", key);
            if (values.size() != 3 || !std::isfinite(values[0]) || !std::isfinite(values[1])
                || !std::isfinite(values[2]) || (nonNegative && (values[0] < 0.0 || values[1] < 0.0
                    || values[2] < 0.0)))
            {
                return fallback;
            }
            return glm::vec3(static_cast<float>(values[0]), static_cast<float>(values[1]),
                static_cast<float>(values[2]));
        };
        settings.enabled                  = boolean("enabled", true);
        settings.skipUnchangedFrames      = boolean("skip_unchanged_frames", true);
        settings.quality                  = static_cast<std::uint32_t>(integer("quality", 3, 0, 4));
        settings.maxSurfaces              = static_cast<std::uint32_t>(integer("scene.max_surfaces", 256, 1, 65536));
        settings.maxTrianglesPerSurface   = static_cast<std::uint64_t>(
            integer("scene.max_triangles_per_surface", 200000, 3, 100000000));
        settings.maxCardsPerSurface       = static_cast<std::uint32_t>(integer("cards.per_surface_max", 12, 1, 64));
        settings.texelsPerMeter           = static_cast<float>(floating("cards.texels_per_meter", 24.0, 1.0, 256.0));
        settings.maxTexelsPerCardAxis     = static_cast<std::uint32_t>(
            integer("cards.max_texels_per_card_axis", 128, 4, 512));
        settings.cardDepthLayers          = static_cast<std::uint32_t>(
            integer("cards.depth_layers", 2, 1, static_cast<std::int64_t>(maxCardLayers)));
        settings.maxTexelsPerSurface      = static_cast<std::uint64_t>(
            integer("cards.max_texels_per_surface", 524288, 4096, 8388608));
        settings.captureTexelsPerFrame    = static_cast<std::uint64_t>(
            integer("capture.texels_per_frame", 262144, 4096, 16777216));
        settings.directTexelsPerFrame     = static_cast<std::uint64_t>(
            integer("lighting.direct_texels_per_frame", 65536, 1024, 16777216));
        settings.indirectTexelsPerFrame   = static_cast<std::uint64_t>(
            integer("lighting.indirect_texels_per_frame", 65536, 1024, 16777216));
        settings.indirectRaysPerTexel     = static_cast<std::uint32_t>(
            integer("lighting.indirect_rays_per_texel", 1, 1, 64));
        settings.sunShadows               = boolean("lighting.sun_shadows", true);
        settings.pointLightShadows        = boolean("lighting.point_light_shadows", false);
        settings.lightingUpdateSpeed      = static_cast<float>(floating("lighting.update_speed", 1.0, 0.05, 16.0));
        settings.indirectTemporalBlend =
            static_cast<float>(floating("lighting.indirect_temporal_blend", 0.85, 0.0, 0.99));
        settings.maxTraceDistance         = static_cast<float>(floating("trace.max_distance_m", 50.0, 1.0, 2000.0));
        settings.traceBias                = static_cast<float>(floating("trace.bias_m", 0.02, 0.001, 0.5));
        settings.maxTransmissionSegments  = static_cast<std::uint32_t>(
            integer("trace.max_transmission_segments", 3, 0, 16));
        settings.probeSpacing             = static_cast<float>(floating("probes.spacing_m", 2.0, 0.25, 32.0));
        // Odd values only (an odd dimension keeps a probe exactly on the
        // camera axis); even values round up so a stray config cannot build
        // an axis-symmetric lattice.
        settings.probeClipDimension = static_cast<std::uint32_t>(
                                 integer("probes.clip_dimension", probeGridDimension, 3, 33))
                               | 1u;
        settings.probeDirectionResolution = static_cast<std::uint32_t>(
            integer("probes.direction_resolution", 32,
                static_cast<std::int64_t>(minProbeDirectionResolution),
                static_cast<std::int64_t>(maxProbeDirectionResolution)));
        settings.probeUpdatesPerFrame     = static_cast<std::uint32_t>(
            integer("probes.updates_per_frame", 64, 1,
                static_cast<std::int64_t>(settings.probeClipDimension) * settings.probeClipDimension
                    * settings.probeClipDimension * 2));
        settings.probeRaysPerUpdate       = static_cast<std::uint32_t>(
            integer("probes.rays_per_update", 256, 1, 4096));
        settings.probeHysteresis          = static_cast<float>(floating("probes.hysteresis", 0.2, 0.0, 0.99));
        settings.probeRelocationMargin    = static_cast<float>(
            floating("probes.relocation_margin", 0.05, 0.005, 0.25));
        settings.dynamicRelightRadius     = static_cast<float>(
            floating("probes.dynamic_relight_radius_m", 2.0, 0.0, 64.0));
        settings.probeLevel1SpacingFactor =
            static_cast<float>(floating("probes.level1_spacing_factor", 4.0, 2.0, 16.0));
        settings.screenRayDistance =
            static_cast<float>(floating("gather.screen_ray_distance_m", 20.0, 0.05, 64.0));
        settings.gatherScreenScale        = static_cast<std::uint32_t>(integer("gather.screen_scale", 8, 1, 64));
        settings.gatherRaysPerProbe       = static_cast<std::uint32_t>(integer("gather.rays_per_probe", 32, 1, 256));
        settings.gatherImportanceSampling = boolean("gather.importance_sampling", true);
        settings.gatherTemporalBlend      = static_cast<float>(floating("gather.temporal_blend", 0.85, 0.0, 0.99));
        settings.gatherProbeTemporalBlend = static_cast<float>(floating("gather.probe_temporal_blend", 0.9, 0.0, 0.99));
        settings.gatherSpatialFilter      = static_cast<std::uint32_t>(integer("gather.spatial_filter", 3, 0, 9));
        settings.gatherPlaneDepthTolerance = static_cast<float>(
            floating("gather.plane_depth_tolerance", 0.35, 0.01, 4.0));
        settings.gatherInterpolationRadius = static_cast<std::uint32_t>(
            integer("gather.interpolation_radius", 2, 1, 4));
        settings.gatherFieldEnabled       = boolean("gather.field_enabled", true);
        settings.gatherFieldScale         = static_cast<std::uint32_t>(integer("gather.field_scale", 4, 1, 4));
        settings.causticsEnabled           = boolean("caustics.enabled", true);
        settings.causticsPhotonsPerFrame   = static_cast<std::uint32_t>(
            integer("caustics.photons_per_frame", 1024, 16, 65536));
        settings.causticsTemporalBlend     = static_cast<float>(floating("caustics.temporal_blend", 0.9, 0.0, 0.99));
        settings.causticsFootprintMinM     = static_cast<float>(
            floating("caustics.footprint_min_m", 0.02, 0.001, 0.25));
        settings.causticsFootprintMaxM     = static_cast<float>(
            floating("caustics.footprint_max_m", 0.75, 0.05, 8.0));
        settings.causticsEmissiveSources   = boolean("caustics.emissive_sources", true);
        settings.debugInternalWidth       = static_cast<std::uint32_t>(integer("debug.internal_width", 320, 16, 1920));
        settings.debugInternalHeight      = static_cast<std::uint32_t>(integer("debug.internal_height", 180, 16, 1080));
        settings.referenceSamples         = static_cast<std::uint32_t>(integer("debug.reference_samples", 64, 1, 4096));
        settings.referenceBounces         = static_cast<std::uint32_t>(integer("debug.reference_bounces", 4, 1, 16));
        settings.debugIncludeAmbient      = boolean("debug.include_ambient", false);
        settings.continuousGather         = boolean("debug.continuous_gather", false);
        settings.exportGatherFrames       = static_cast<std::uint32_t>(integer("debug.export_gather_frames", 24, 1, 64));
        settings.debugExportFieldPrefix =
            context_->configurationStringInSection("radiance_cache", "debug.export_field_prefix").value_or("");
        // Fallback environment (used when the optional pbr capability is
        // absent): the same defaults the renderer's built-in environment
        // carries, overridable so a headless consumer without pbr can still
        // tune the sun and the hemispheric skylight.
        fallbackEnvironment_.sunDirection = environmentVector("environment.sun_direction",
            glm::vec3(0.45f, 0.62f, 0.35f), false);
        fallbackEnvironment_.sunColor     = environmentVector("environment.sun_color", glm::vec3(1.0f, 0.96f, 0.88f), true);
        fallbackEnvironment_.sunIntensity =
            static_cast<float>(floating("environment.sun_intensity", 3.0, 0.0, 1.0e6));
        fallbackEnvironment_.sunEnabled   = boolean("environment.sun_enabled", true);
        fallbackEnvironment_.ambientSky    = environmentVector("environment.ambient_sky", glm::vec3(0.55f, 0.65f, 0.80f), true);
        fallbackEnvironment_.ambientGround = environmentVector("environment.ambient_ground", glm::vec3(0.35f, 0.30f, 0.25f), true);
        fallbackEnvironment_.ambientIntensity =
            static_cast<float>(floating("environment.ambient_intensity", 0.5, 0.0, 1.0e6));
        settings_                         = settings;
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        bool relevant = false;
        for (const std::string &key : event.changedKeys)
        {
            if (key.rfind("radiance_cache.", 0) == 0)
            {
                relevant = true;
                break;
            }
        }
        if (!relevant)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        readConfigurationLocked();
        rebuildPipelineAfterConfigChangeLocked();
    }

    // Settings that change capacity or layout (probe resolution, spacing,
    // card placement, debug resolution) rebuild the derived structures. Every
    // structure whose inputs actually changed is rebuilt - the old code only
    // built the probe levels once, so a live change of probes.direction_resolution
    // left probe maps sized for the previous resolution.
    void rebuildPipelineAfterConfigChangeLocked()
    {
        applySettingsToState(settings_, state_, qualityScale(0u, processorCount_));
        state_.registryCapacity = settings_.maxSurfaces;
        const std::uint32_t resolution = std::max(settings_.probeDirectionResolution, 4u);
        const bool probesChanged = state_.probeDirections.empty() || resolution != builtDirectionResolution_
            || settings_.probeSpacing != builtProbeSpacing_ || settings_.probeLevel1SpacingFactor != builtLevel1Factor_
            || settings_.probeClipDimension != builtProbeDimension_ || state_.level0.probes.empty();
        if (probesChanged)
        {
            buildProbeDirectionTable(state_);
            buildProbeLevels(state_);
            builtDirectionResolution_ = resolution;
            builtProbeSpacing_        = settings_.probeSpacing;
            builtLevel1Factor_        = settings_.probeLevel1SpacingFactor;
            builtProbeDimension_      = settings_.probeClipDimension;
            if (state_.hasView)
            {
                snapProbeLevels(state_, state_.cameraPosition);
            }
        }
        const CardKey cardKey{settings_.texelsPerMeter, settings_.maxTexelsPerCardAxis, settings_.maxCardsPerSurface,
            settings_.cardDepthLayers, settings_.maxTexelsPerSurface};
        if (!(cardKey == builtCardKey_))
        {
            builtCardKey_ = cardKey;
            for (GiSurface &surface : state_.surfaces)
            {
                placeSurfaceCards(state_.settings, surface);
            }
            rebuildCardRefs(state_);
        }
        state_.screenProbeWidth  = 0;
        state_.screenProbeHeight = 0;
        state_.screenProbeRadiance.clear();
        state_.gatherHasHistory = false;
        state_.sceneDirty       = true;
        state_.causticTransmittersDirty = true;
        state_.causticEmissiveCardsDirty = true;
        publishSnapshotLocked();
        publishGatherSnapshotLocked();
    }

    // ---- surface registry -----------------------------------------------------

    // Handle -> index into state_.surfaces, maintained at every registry
    // mutation under moduleMutex_. Dynamic surfaces re-look up their entry
    // every frame (updateSurfaceTransform), so the registry must answer in
    // O(1) at registry capacities in the tens of thousands, not scan the
    // vector. Destroy swaps the last surface into the hole (see
    // destroySurfaceImpl), which keeps the fix-up O(1) at the cost of a
    // registration order the consumers never observe - they hold handles.
    std::uint32_t findSurfaceLocked(GiSurfaceHandle handle) const
    {
        const auto found = surfaceIndexByHandle_.find(handle);
        if (found == surfaceIndexByHandle_.end() || found->second >= state_.surfaces.size())
        {
            return 0xffffffffu;
        }
        return found->second;
    }

    void applySurfaceDescriptionLocked(GiSurface &surface, const GiSurfaceDescription &description, GiError *error)
    {
        GiSurfaceGeometry geometry;
        buildSurfaceGeometry(geometry, description, settings_.maxTrianglesPerSurface, error);
        if (error != nullptr && *error != GiError::None)
        {
            return;
        }
        buildSurfaceBvh(geometry);
        surface.geometry         = std::move(geometry);
        surface.albedo           = glm::clamp(glm::vec3(description.albedoColor[0], description.albedoColor[1],
            description.albedoColor[2]), glm::vec3(0.0f), glm::vec3(1.0e6f));
        surface.emissive         = glm::max(
            glm::vec3(description.emissiveColor[0], description.emissiveColor[1], description.emissiveColor[2]),
            glm::vec3(0.0f))
            * std::max(description.emissiveIntensity, 0.0f);
        surface.opacity   = std::clamp(std::isfinite(description.opacity) ? description.opacity : 1.0f, 0.0f, 1.0f);
        surface.indexOfRefraction = std::clamp(
            std::isfinite(description.indexOfRefraction) ? description.indexOfRefraction : 1.5f, 1.0f, 64.0f);
        surface.flagBits = description.flagBits;
        surface.excluded = (description.flagBits & giSurfaceFlagExcluded) != 0u;
        surface.unlit    = (description.flagBits & giSurfaceFlagUnlit) != 0u;
        surface.dynamic  = (description.flagBits & giSurfaceFlagDynamic) != 0u;
        surface.cards.clear();
        // A singular transform is rejected like any other malformed
        // description: the surface keeps the geometry it was built with and
        // the caller reports the error.
        if (!updateSurfaceTransform(surface, makeTransform(description.transform)))
        {
            if (error != nullptr)
            {
                *error = GiError::InvalidDescription;
            }
            return;
        }
        // Only this surface's cards are rebuilt (and recaptured); every other
        // surface keeps its captured and lit data. The old code re-placed the
        // cards of ALL surfaces on every registry change.
        placeSurfaceCards(state_.settings, surface);
    }

    void registryChangedLocked()
    {
        rebuildCardRefs(state_);
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
        // Caustics source lists follow the registry (new transmitters, new
        // emitters, changed emission).
        state_.causticTransmittersDirty = true;
        state_.causticEmissiveCardsDirty = true;
        ++state_.gatherRasterSerial;
        causticReceiverIndexDirty_ = true;
        state_.causticsDecayFrames = std::max<std::uint32_t>(state_.causticsDecayFrames, 256);
    }

    // Marks the probes around a changed world volume for a full retrace (the
    // swept volume of a move/deform, or the volume itself for a
    // create/destroy), expanded by probes.dynamic_relight_radius_m so the
    // neighboring probes also refresh. probeUpdatesPerFrame bounds how much
    // retrace work a frame can absorb.
    void markDynamicRelightLocked(const glm::vec3 &minA, const glm::vec3 &maxA, const glm::vec3 &minB,
        const glm::vec3 &maxB)
    {
        if (settings_.dynamicRelightRadius <= 0.0f)
        {
            return;
        }
        const glm::vec3 expansion(settings_.dynamicRelightRadius);
        const glm::vec3 volumeMin = glm::min(minA, minB) - expansion;
        const glm::vec3 volumeMax = glm::max(maxA, maxB) + expansion;
        markProbesNearVolume(state_, volumeMin, volumeMax);
    }

    GiSurfaceHandle createSurfaceImpl(const GiSurfaceDescription *description, GiError *error)
    {
        if (description == nullptr)
        {
            setError(error, GiError::InvalidDescription);
            return invalidGiSurface;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (!validateDescription(*description))
        {
            setError(error, GiError::InvalidDescription);
            return invalidGiSurface;
        }
        if (state_.surfaces.size() >= state_.registryCapacity)
        {
            setError(error, GiError::CapacityReached);
            return invalidGiSurface;
        }
        if (state_.nextHandle == 0xffffffffu)
        {
            setError(error, GiError::CapacityReached);
            return invalidGiSurface;
        }
        GiError   localError = GiError::None;
        GiSurface surface;
        surface.handle = state_.nextHandle;
        applySurfaceDescriptionLocked(surface, *description, &localError);
        if (localError != GiError::None)
        {
            setError(error, localError);
            return invalidGiSurface;
        }
        // The handle is consumed only once the surface really exists.
        const GiSurfaceHandle handle = state_.nextHandle++;
        surfaceIndexByHandle_.emplace(handle, static_cast<std::uint32_t>(state_.surfaces.size()));
        state_.surfaces.push_back(std::move(surface));
        registryChangedLocked();
        // A new surface lights its neighborhood: the probes around it retrace
        // from scratch so the world radiance cache picks it up immediately.
        markDynamicRelightLocked(state_.surfaces.back().worldMin, state_.surfaces.back().worldMax,
            state_.surfaces.back().worldMin, state_.surfaces.back().worldMax);
        setError(error, GiError::None);
        return handle;
    }

    bool validateDescription(const GiSurfaceDescription &description) const
    {
        if (description.vertexCount == 0 || description.indexCount < 3 || description.indexCount % 3 != 0)
        {
            return false;
        }
        for (std::uint32_t index = 0; index < 16; ++index)
        {
            if (!std::isfinite(description.transform[index]))
            {
                return false;
            }
        }
        if (!std::isfinite(description.emissiveIntensity) || description.emissiveIntensity < 0.0f)
        {
            return false;
        }
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(description.albedoColor[axis]) || description.albedoColor[axis] < 0.0f
                || !std::isfinite(description.emissiveColor[axis]) || description.emissiveColor[axis] < 0.0f)
            {
                return false;
            }
        }
        if (!std::isfinite(description.opacity) || description.opacity < 0.0f || description.opacity > 1.0f
            || !std::isfinite(description.indexOfRefraction) || description.indexOfRefraction < 1.0f)
        {
            return false;
        }
        return true;
    }

    bool updateSurfaceImpl(GiSurfaceHandle handle, const GiSurfaceDescription *description, GiError *error)
    {
        if (description == nullptr)
        {
            setError(error, GiError::InvalidDescription);
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(handle);
        if (index == 0xffffffffu)
        {
            setError(error, GiError::InvalidHandle);
            return false;
        }
        if (!validateDescription(*description))
        {
            setError(error, GiError::InvalidDescription);
            return false;
        }
        // Build into a copy so a rejected description leaves the live surface
        // untouched (the old code overwrote it half-way).
        GiSurface replacement;
        replacement.handle = handle;
        GiError localError = GiError::None;
        const glm::vec3 oldMin = state_.surfaces[index].worldMin;
        const glm::vec3 oldMax = state_.surfaces[index].worldMax;
        applySurfaceDescriptionLocked(replacement, *description, &localError);
        if (localError != GiError::None)
        {
            setError(error, localError);
            return false;
        }
        state_.surfaces[index] = std::move(replacement);
        registryChangedLocked();
        // A deformed surface relights the volume it covered and now covers.
        markDynamicRelightLocked(oldMin, oldMax, state_.surfaces[index].worldMin, state_.surfaces[index].worldMax);
        setError(error, GiError::None);
        return true;
    }

    bool updateSurfaceTransformImpl(GiSurfaceHandle handle, const float *transformColumnMajor16)
    {
        if (transformColumnMajor16 == nullptr)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(handle);
        if (index == 0xffffffffu)
        {
            return false;
        }
        for (std::uint32_t element = 0; element < 16; ++element)
        {
            if (!std::isfinite(transformColumnMajor16[element]))
            {
                return false;
            }
        }
        GiSurface &     surface   = state_.surfaces[index];
        const glm::vec3 oldMin = surface.worldMin;
        const glm::vec3 oldMax = surface.worldMax;
        const glm::mat4 transform = makeTransform(transformColumnMajor16);
        if (transform == surface.transform)
        {
            return true; // unchanged: keep the frame clean for the static skip
        }
        causticReceiverIndexDirty_ = true;
        // A singular transform is refused and the previous one kept; the
        // caller only sees the boolean.
        if (!updateSurfaceTransform(surface, transform))
        {
            return false;
        }
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
        // A moved surface relights the volume it swept through.
        markDynamicRelightLocked(oldMin, oldMax, surface.worldMin, surface.worldMax);
        return true;
    }

    bool destroySurfaceImpl(GiSurfaceHandle handle)
    {
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(handle);
        if (index == 0xffffffffu)
        {
            return false;
        }
        // A removed surface leaves its volume dark/changed: the probes around
        // it retrace from scratch.
        const glm::vec3 oldMin = state_.surfaces[index].worldMin;
        const glm::vec3 oldMax = state_.surfaces[index].worldMax;
        // Swap-and-pop: the last surface moves into the hole and its map
        // entry follows, so removal stays O(1) like the lookup (a middle
        // erase would shift every later index and force an O(n) map walk).
        surfaceIndexByHandle_.erase(handle);
        if (index + 1u < state_.surfaces.size())
        {
            state_.surfaces[index] = std::move(state_.surfaces.back());
            surfaceIndexByHandle_[state_.surfaces[index].handle] = index;
        }
        state_.surfaces.pop_back();
        registryChangedLocked();
        markDynamicRelightLocked(oldMin, oldMax, oldMin, oldMax);
        return true;
    }

    bool setViewImpl(const float *viewProjectionColumnMajor16, const float *cameraPosition3)
    {
        if (viewProjectionColumnMajor16 == nullptr || cameraPosition3 == nullptr)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        for (std::uint32_t element = 0; element < 16; ++element)
        {
            if (!std::isfinite(viewProjectionColumnMajor16[element]))
            {
                return false;
            }
        }
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(cameraPosition3[axis]))
            {
                return false;
            }
        }
        // Static-frame skip change detection: an identical repeated setView
        // (same matrices, same eye) leaves the frame clean.
        bool viewChanged = true;
        if (state_.hasView)
        {
            viewChanged = false;
            for (std::uint32_t column = 0; column < 4; ++column)
            {
                for (std::uint32_t row = 0; row < 4; ++row)
                {
                    if (state_.viewProjection[column][row] != viewProjectionColumnMajor16[column * 4 + row])
                    {
                        viewChanged = true;
                    }
                }
            }
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                if (state_.cameraPosition[axis] != cameraPosition3[axis])
                {
                    viewChanged = true;
                }
            }
        }
        if (viewChanged)
        {
            state_.viewDirty = true;
        }
        state_.viewProjection  = glm::make_mat4(viewProjectionColumnMajor16);
        state_.cameraPosition  = glm::make_vec3(cameraPosition3);
        state_.hasView         = true;
        return true;
    }

    // ---- environment snapshot ---------------------------------------------------

    // Snapshots the pbr environment and direct lights into the frame state.
    // Returns true when the snapshot differs from the previous one (version
    // compare), which is the environment-change signal for the static-frame skip.
    bool snapshotEnvironmentLocked()
    {
        GiEnvironmentSnapshot snapshot;
        std::uint64_t         pbrVersion    = 0;
        std::uint64_t         lightsVersion = 0;
        // Fallback environment first (used when the optional pbr capability is
        // absent or its copy fails): the same defaults the renderer's built-in
        // environment carries, overridable through the [radiance_cache.environment]
        // keys so a headless consumer without pbr can still tune the sun and
        // the hemispheric skylight.
        snapshot.hasSun           = true;
        snapshot.sunDirection     = glm::normalize(fallbackEnvironment_.sunDirection);
        if (!std::isfinite(snapshot.sunDirection.x) || !std::isfinite(snapshot.sunDirection.y)
            || !std::isfinite(snapshot.sunDirection.z))
        {
            // A zero (or degenerate) configured direction: fall back to the
            // built-in default instead of a NaN direction.
            snapshot.sunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
        }
        snapshot.sunColor        = fallbackEnvironment_.sunColor;
        snapshot.sunIntensity    = fallbackEnvironment_.sunIntensity;
        snapshot.sunEnabled      = fallbackEnvironment_.sunEnabled;
        snapshot.ambientSky      = fallbackEnvironment_.ambientSky;
        snapshot.ambientGround   = fallbackEnvironment_.ambientGround;
        snapshot.ambientIntensity = fallbackEnvironment_.ambientIntensity;
        pbrVersion = lighting::copyPbrEnvironment(pbrCapability_, snapshot);
        if (directLightingCapability_ != nullptr && directLightingCapability_->copyLights != nullptr)
        {
            // The scratch is a member, not a local: steady-state evaluates
            // stop hitting the allocator for the same worst-case light set.
            directLightsScratch_.resize(direct_lighting::maxDirectLights);
            const std::uint32_t count = directLightingCapability_->copyLights(directLightingCapability_->self,
                directLightsScratch_.data(), static_cast<std::uint32_t>(directLightsScratch_.size()), &lightsVersion);
            snapshot.directLights.assign(directLightsScratch_.begin(),
                directLightsScratch_.begin() + std::min<std::size_t>(count, directLightsScratch_.size()));
            snapshot.hasDirectLights = !snapshot.directLights.empty();
        }
        // Precompute the per-light constants the shading loop needs and drop
        // inactive lights; the shared lighting::prepareLight fills the same
        // structure reflection_field snapshots, so the two modules' direct
        // terms stay identical.
        snapshot.preparedLights.clear();
        snapshot.preparedLights.reserve(snapshot.directLights.size());
        for (const direct_lighting::DirectLight &light : snapshot.directLights)
        {
            GiEnvironmentSnapshot::PreparedDirectLight prepared;
            if (lighting::prepareLight(light, prepared))
            {
                snapshot.preparedLights.push_back(prepared);
            }
        }
        snapshot.hasDirectLights = !snapshot.preparedLights.empty();
        const bool environmentChanged = !state_.environmentValid
            || pbrVersion != state_.environmentVersion || lightsVersion != state_.directLightsVersion;
        state_.environmentVersion    = pbrVersion;
        state_.directLightsVersion   = lightsVersion;
        state_.environment = std::move(snapshot);
        state_.environmentValid = true;
        return environmentChanged;
    }

    // ---- evaluate: the Lumen-style frame -----------------------------------------

    TaskScheduler *schedulerLocked()
    {
        return context_ != nullptr ? &context_->taskScheduler() : nullptr;
    }

    bool evaluateImpl(double deltaSeconds, GiError *error)
    {
        (void) deltaSeconds;
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (!state_.hasView)
        {
            setError(error, GiError::ViewNotSet);
            return false;
        }
        if (!settings_.enabled)
        {
            setError(error, GiError::None);
            return true;
        }
        const auto startTime = std::chrono::steady_clock::now();
        const bool environmentChanged = snapshotEnvironmentLocked();
        ++state_.frameIndex;
        state_.captureTexelsThisFrame.store(0, std::memory_order_relaxed);
        state_.directTexelsThisFrame.store(0, std::memory_order_relaxed);
        state_.indirectTexelsThisFrame.store(0, std::memory_order_relaxed);
        state_.indirectRaysThisFrame.store(0, std::memory_order_relaxed);
        state_.probeTracesThisFrame.store(0, std::memory_order_relaxed);
        state_.screenProbesThisFrame.store(0, std::memory_order_relaxed);
        state_.gatherRaysThisFrame.store(0, std::memory_order_relaxed);

        // Static-frame skip: when the camera, the scene, and the environment
        // are all unchanged and every cache has had enough updates to settle
        // (the pipeline derives that count from the budgets and the amount of
        // cached texels and probes), re-running would only re-blend the same
        // radiance with fresh rays. Return early - and keep the probe version
        // so a renderer polling copyProbes skips its upload too.
        const bool viewChanged  = state_.viewDirty;
        const bool sceneChanged = state_.sceneDirty;
        state_.viewDirty  = false;
        state_.sceneDirty = false;
        if (viewChanged || sceneChanged || environmentChanged)
        {
            state_.framesSinceDirty = 0;
        }
        if (settings_.skipUnchangedFrames && state_.hasPreviousView && !viewChanged && !sceneChanged
            && !environmentChanged && state_.framesSinceDirty >= state_.convergenceFrames)
        {
            state_.framesSkippedStatic.fetch_add(1, std::memory_order_relaxed);
            lastEvaluateMilliseconds_ = 0.0;
            setError(error, GiError::None);
            return true;
        }
        ++state_.framesSinceDirty;

        TaskScheduler *scheduler = schedulerLocked();
        if (!runGiFrame(state_, scheduler))
        {
            // A stage failed half-way: restart capture and lighting so no card
            // keeps a partially written state.
            for (GiSurface &surface : state_.surfaces)
            {
                invalidateSurfaceCards(surface);
            }
            state_.framesSinceDirty = 0;
            state_.sceneDirty       = true;
            setError(error, GiError::EvaluationFailed);
            return false;
        }
        state_.hasPreviousView = true;
        // The exported gather field (radiance_cache.gather): resolved every
        // evaluate once a consumer pushed a viewport, so its contact-scale
        // structure (short-range occlusion, caustic irradiance) reaches the
        // renderer's ambient. The static-frame skip above keeps converged
        // views frozen, and the pass shares the debug gather's budget
        // knobs (gather.screen_scale / rays_per_probe / filters).
        const bool runGatherField = state_.gatherViewportWidth != 0 && state_.gatherViewportHeight != 0
            && settings_.gatherFieldEnabled;
        if (runGatherField || settings_.continuousGather)
        {
            try
            {
                runGatherPasses(state_, scheduler, 1);
            }
            catch (...)
            {
                setError(error, GiError::EvaluationFailed);
                return false;
            }
            state_.totalRaysTraced.fetch_add(state_.gatherRaysThisFrame.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
        }
        publishSnapshotLocked();
        publishGatherSnapshotLocked();

        const auto endTime = std::chrono::steady_clock::now();
        lastEvaluateMilliseconds_ =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count())
            / 1000.0;
        setError(error, GiError::None);
        return true;
    }

    // ---- read side ---------------------------------------------------------------

    // Builds and publishes the immutable probe snapshot (module mutex held).
    // The previous snapshot's probe buffer is recycled into the new snapshot
    // when no renderer still holds a reference, so steady-state publishing
    // does not reallocate per frame.
    void publishSnapshotLocked()
    {
        std::shared_ptr<GiProbeSnapshot> previous;
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            previous = snapshot_;
        }
        auto snapshot = std::make_shared<GiProbeSnapshot>();
        if (previous != nullptr && previous.use_count() == 1)
        {
            // Sole owner: reuse the buffer (buildProbeSnapshot clears and
            // refills; the capacity survives).
            snapshot->probes = std::move(previous->probes);
        }
        buildProbeSnapshot(state_, *snapshot);
        snapshot->version = probeVersion_.fetch_add(1u, std::memory_order_acq_rel) + 1u;
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot_ = std::move(snapshot);
    }

    // Builds and publishes the immutable gather-field snapshot (module mutex
    // held; the copy side reads it under gatherSnapshotMutex_ exactly like
    // copyProbes reads the probe snapshot). A state without a resolved field
    // (no viewport pushed yet, or the very first frames) publishes an empty
    // snapshot so the copy side reports a clean zero instead of a stale view.
    void publishGatherSnapshotLocked()
    {
        auto snapshot = std::make_shared<GiGatherSnapshot>();
        if (state_.viewWidth != 0 && state_.gatherFiltered.size()
            == static_cast<std::size_t>(state_.viewWidth) * state_.viewHeight)
        {
            snapshot->header.width  = state_.viewWidth;
            snapshot->header.height = state_.viewHeight;
            // viewProjection is stored column-major in the pipeline state's
            // glm::mat4 (glm's own layout), the same convention the header
            // exports.
            for (std::uint32_t column = 0; column < 4; ++column)
            {
                for (std::uint32_t row = 0; row < 4; ++row)
                {
                    snapshot->header.viewProjection[column * 4 + row] = state_.viewProjection[column][row];
                }
            }
            snapshot->header.cameraPosition[0] = state_.cameraPosition.x;
            snapshot->header.cameraPosition[1] = state_.cameraPosition.y;
            snapshot->header.cameraPosition[2] = state_.cameraPosition.z;
            const std::size_t texelCount = state_.gatherFiltered.size();
            snapshot->field.resize(texelCount * radianceGatherTexelFloats);
            for (std::size_t texel = 0; texel < texelCount; ++texel)
            {
                const glm::vec3 &irradiance = state_.gatherFiltered[texel];
                float           *outTexel   = snapshot->field.data() + texel * radianceGatherTexelFloats;
                outTexel[0] = irradiance.r;
                outTexel[1] = irradiance.g;
                outTexel[2] = irradiance.b;
                outTexel[3] = texel < state_.gatherFieldValid.size() && state_.gatherFieldValid[texel] != 0 ? 1.0f : 0.0f;
                outTexel[4] = texel < state_.gatherFieldOcclusion.size() ? state_.gatherFieldOcclusion[texel] : 1.0f;
            }
        }
        // Diagnostic dump (radiance_cache.debug.export_field_prefix): the raw
        // exported field exactly as the renderer receives it - irradiance
        // through a fixed reinhard for display, occlusion as a gray scale.
        // Overwritten every publish, so the final files are the last publish.
        if (!settings_.debugExportFieldPrefix.empty() && snapshot->field.size()
            == static_cast<std::size_t>(state_.viewWidth) * state_.viewHeight * radianceGatherTexelFloats)
        {
            const std::uint32_t dumpWidth  = state_.viewWidth;
            const std::uint32_t dumpHeight = state_.viewHeight;
            std::vector<std::uint8_t> rgb(static_cast<std::size_t>(dumpWidth) * dumpHeight * 3, 0u);
            std::vector<std::uint8_t> gray(static_cast<std::size_t>(dumpWidth) * dumpHeight * 3, 0u);
            for (std::size_t texel = 0; texel < static_cast<std::size_t>(dumpWidth) * dumpHeight; ++texel)
            {
                const float *source = snapshot->field.data() + texel * radianceGatherTexelFloats;
                const auto   store  = [](float value)
                {
                    return static_cast<std::uint8_t>(std::clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f);
                };
                for (std::uint32_t channel = 0; channel < 3; ++channel)
                {
                    const float c = source[channel];
                    rgb[texel * 3 + channel] = store(c / (1.0f + c));
                }
                const std::uint8_t occlusionByte = store(source[4]);
                gray[texel * 3 + 0] = occlusionByte;
                gray[texel * 3 + 1] = occlusionByte;
                gray[texel * 3 + 2] = occlusionByte;
            }
            const std::string prefix         = settings_.debugExportFieldPrefix;
            const std::string irradiancePath = prefix + "_irradiance.ppm";
            const std::string occlusionPath  = prefix + "_occlusion.ppm";
            util::writePpmRgb(irradiancePath.c_str(), dumpWidth, dumpHeight, rgb.data(),
                static_cast<std::size_t>(dumpWidth) * 3);
            util::writePpmRgb(occlusionPath.c_str(), dumpWidth, dumpHeight, gray.data(),
                static_cast<std::size_t>(dumpWidth) * 3);
        }
        snapshot->version = state_.gatherFieldVersion;
        std::lock_guard<std::mutex> lock(gatherSnapshotMutex_);
        gatherSnapshot_ = std::move(snapshot);
    }

    // Rebuilds the caustics receiver index from the live registry (the
    // module mutex held): receiver-flagged opaque surfaces in registry order,
    // so the sampler's first-match answer is exactly the full scan's.
    void rebuildCausticReceiverIndexLocked()
    {
        causticReceiverIndex_.clear();
        for (std::size_t surface = 0; surface < state_.surfaces.size(); ++surface)
        {
            const GiSurface &receiver = state_.surfaces[surface];
            if (!receiver.causticReceiver || receiver.opacity < 1.0f)
            {
                continue;
            }
            causticReceiverIndex_.push_back(CausticReceiverEntry{
                static_cast<std::uint32_t>(surface), receiver.worldMin, receiver.worldMax});
        }
        causticReceiverIndexDirty_ = false;
    }

    bool sampleCausticsIrradianceImpl(const float *position, const float *normal, float *outIrradiance)
    {
        if (position == nullptr || normal == nullptr || outIrradiance == nullptr)
        {
            return false;
        }
        const glm::vec3 samplePosition = glm::make_vec3(position);
        const glm::vec3 sampleNormal   = glm::make_vec3(normal);
        // The registry's read side: serialize against mutators and evaluate
        // like every other surface read (an uncontended lock between frames -
        // the reflection field samples from its own evaluate, never during
        // this module's).
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (causticReceiverIndexDirty_)
        {
            rebuildCausticReceiverIndexLocked();
        }
        for (const CausticReceiverEntry &entry : causticReceiverIndex_)
        {
            // Bounds first (cheap reject); the card-texel read then confirms
            // the position actually lies on the receiver's surface.
            if (glm::any(glm::lessThan(samplePosition, entry.worldMin))
                || glm::any(glm::greaterThan(samplePosition, entry.worldMax)))
            {
                continue;
            }
            glm::vec3 irradiance(0.0f);
            if (sampleSurfaceCausticsIrradiance(state_, entry.surface, samplePosition, sampleNormal, irradiance))
            {
                outIrradiance[0] = irradiance.x;
                outIrradiance[1] = irradiance.y;
                outIrradiance[2] = irradiance.z;
                return true;
            }
        }
        return false;
    }

    std::uint32_t copyProbesImpl(GiProbe *output, std::uint32_t capacity, GiProbeHeader *outHeader,
        std::uint64_t *outVersion) const
    {
        // Only the tiny snapshot mutex: a renderer polling every frame never
        // waits for a running evaluate().
        std::shared_ptr<const GiProbeSnapshot> snapshot;
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            snapshot = snapshot_;
        }
        if (snapshot == nullptr)
        {
            if (outVersion != nullptr)
            {
                *outVersion = 0;
            }
            if (outHeader != nullptr)
            {
                *outHeader = GiProbeHeader{};
            }
            return 0;
        }
        if (outVersion != nullptr)
        {
            *outVersion = snapshot->version;
        }
        if (outHeader != nullptr)
        {
            *outHeader = snapshot->header;
        }
        if (output == nullptr || capacity == 0)
        {
            return 0;
        }
        const std::uint32_t copied = std::min<std::uint32_t>(capacity, static_cast<std::uint32_t>(snapshot->probes.size()));
        std::copy(snapshot->probes.begin(), snapshot->probes.begin() + copied, output);
        return copied;
    }

    bool setGatherViewportImpl(std::uint32_t viewportWidth, std::uint32_t viewportHeight)
    {
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (viewportWidth == 0 || viewportHeight == 0)
        {
            // Zero dimensions deactivate the field (the pass and the export
            // publish an empty snapshot, so the consumer's fallback ambient
            // takes over cleanly).
            state_.gatherViewportWidth  = 0;
            state_.gatherViewportHeight = 0;
            publishGatherSnapshotLocked();
            return true;
        }
        state_.gatherViewportWidth  = viewportWidth;
        state_.gatherViewportHeight = viewportHeight;
        return true;
    }

    std::uint32_t copyIrradianceFieldImpl(float *output, std::uint32_t capacityTexels,
        RadianceGatherHeader *outHeader, std::uint64_t *outVersion) const
    {
        // Only the gather snapshot mutex, the copyProbes contract: a renderer
        // polling every frame never waits for a running evaluate().
        std::shared_ptr<const GiGatherSnapshot> snapshot;
        {
            std::lock_guard<std::mutex> lock(gatherSnapshotMutex_);
            snapshot = gatherSnapshot_;
        }
        if (snapshot == nullptr)
        {
            if (outVersion != nullptr)
            {
                *outVersion = 0;
            }
            if (outHeader != nullptr)
            {
                *outHeader = RadianceGatherHeader{};
            }
            return 0;
        }
        if (outVersion != nullptr)
        {
            *outVersion = snapshot->version;
        }
        if (outHeader != nullptr)
        {
            *outHeader = snapshot->header;
        }
        if (output == nullptr || capacityTexels == 0)
        {
            return 0;
        }
        const std::size_t texelCount = snapshot->field.size() / radianceGatherTexelFloats;
        const std::size_t copiedTexels = std::min<std::size_t>(capacityTexels, texelCount);
        std::copy(snapshot->field.begin(), snapshot->field.begin() + copiedTexels * radianceGatherTexelFloats, output);
        return static_cast<std::uint32_t>(copiedTexels);
    }

    bool statisticsImpl(GiStatistics *outStatistics) const
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        outStatistics->frameIndex               = state_.frameIndex;
        outStatistics->surfaces                 = static_cast<std::uint32_t>(state_.surfaces.size());
        std::uint64_t triangles                 = 0;
        std::uint64_t cardTexels                = 0;
        for (const GiSurface &surface : state_.surfaces)
        {
            triangles += surface.geometry.indices.size() / 3;
            for (const GiCard &card : surface.cards)
            {
                cardTexels += card.totalTexels();
            }
        }
        outStatistics->triangles                = triangles;
        outStatistics->cards                    = static_cast<std::uint32_t>(state_.cardRefs.size());
        outStatistics->cardTexels               = cardTexels;
        outStatistics->captureTexelsThisFrame   = state_.captureTexelsThisFrame.load(std::memory_order_relaxed);
        outStatistics->directTexelsThisFrame    = state_.directTexelsThisFrame.load(std::memory_order_relaxed);
        outStatistics->indirectTexelsThisFrame  = state_.indirectTexelsThisFrame.load(std::memory_order_relaxed);
        outStatistics->indirectRaysThisFrame    = state_.indirectRaysThisFrame.load(std::memory_order_relaxed);
        outStatistics->probeTracesThisFrame     = state_.probeTracesThisFrame.load(std::memory_order_relaxed);
        outStatistics->screenProbesThisFrame    = state_.screenProbesThisFrame.load(std::memory_order_relaxed);
        outStatistics->gatherRaysThisFrame      = state_.gatherRaysThisFrame.load(std::memory_order_relaxed);
        outStatistics->totalRaysTraced          = state_.totalRaysTraced.load(std::memory_order_relaxed);
        outStatistics->framesSkippedStatic      = state_.framesSkippedStatic.load(std::memory_order_relaxed);
        outStatistics->lastEvaluateMilliseconds = lastEvaluateMilliseconds_;
        return true;
    }

    bool resolveProfileImpl(const renderer::DeviceProfile *device, GiProfile *outProfile) const
    {
        if (device == nullptr || outProfile == nullptr)
        {
            return false;
        }
        // Device-class scaling like the render graph's profile resolution:
        // Vulkan discrete 1.0, integrated 0.5, everything else (other Vulkan
        // classes, non-Vulkan, CPU) the 0.25 floor so headless use stays
        // responsive.
        float scale = 0.25f;
        if (device->isVulkan)
        {
            switch (device->deviceClass)
            {
            case renderer::DeviceClass::DiscreteGpu:
                scale = 1.0f;
                break;
            case renderer::DeviceClass::IntegratedGpu:
                scale = 0.5f;
                break;
            default:
                scale = 0.25f;
                break;
            }
        }
        outProfile->captureTexelsPerFrame =
            std::max(4096u, static_cast<std::uint32_t>(static_cast<float>(settings_.captureTexelsPerFrame) * scale));
        outProfile->directTexelsPerFrame =
            std::max(1024u, static_cast<std::uint32_t>(static_cast<float>(settings_.directTexelsPerFrame) * scale));
        outProfile->indirectTexelsPerFrame =
            std::max(1024u, static_cast<std::uint32_t>(static_cast<float>(settings_.indirectTexelsPerFrame) * scale));
        outProfile->probeUpdatesPerFrame =
            std::max(4u, static_cast<std::uint32_t>(static_cast<float>(settings_.probeUpdatesPerFrame) * scale));
        return true;
    }

    // The debug views are produced on demand: the GBuffer is rasterized for
    // the current camera and, for the views that include the screen-space
    // gather, a few accumulation passes run first so the temporal filter has
    // converged (unless debug.continuous_gather keeps it warm every frame).
    bool debugExportImpl(GiDebugView view, const char *filePath, GiError *error)
    {
        if (filePath == nullptr)
        {
            setError(error, GiError::ExportFailed);
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (!state_.hasView || !state_.environmentValid)
        {
            setError(error, GiError::ViewNotSet);
            return false;
        }
        try
        {
            ensureSceneTlas(state_);
            TaskScheduler *scheduler = schedulerLocked();
            const bool needsGather = view == GiDebugView::FinalImage || view == GiDebugView::GiOnly;
            const std::uint32_t passes = needsGather
                ? (settings_.continuousGather ? 1u : std::max(settings_.exportGatherFrames, 1u))
                : 0u;
            runGatherPasses(state_, scheduler, passes);
            return exportDebugView(state_, scheduler, view, filePath, error);
        }
        catch (...)
        {
            setError(error, GiError::ExportFailed);
            return false;
        }
    }

    struct CardKey
    {
        float         texelsPerMeter;
        std::uint32_t maxAxis;
        std::uint32_t maxCards;
        std::uint32_t layers;
        std::uint64_t maxTexels;
        bool operator==(const CardKey &other) const
        {
            return texelsPerMeter == other.texelsPerMeter && maxAxis == other.maxAxis && maxCards == other.maxCards
                && layers == other.layers && maxTexels == other.maxTexels;
        }
    };

    ParticipantManifest        manifest_;
    RadianceCacheCapability          capability_{};
    // The screen-space gather field export ("radiance_cache.gather"); the
    // same module instance backs both capabilities and one mutex serializes
    // them.
    RadianceGatherCapability          gatherCapability_{};
    ParticipantContext        *context_                  = nullptr;
    pbr::PbrCapability        *pbrCapability_            = nullptr;
    direct_lighting::DirectLightingCapability *directLightingCapability_ = nullptr;
    SubscriptionToken reloadSubscription_ = 0;

    // Built-in environment used when the pbr capability is absent; fed from
    // the [radiance_cache.environment] configuration keys.
    struct FallbackEnvironment
    {
        glm::vec3 sunDirection{0.45f, 0.62f, 0.35f};
        glm::vec3 sunColor{1.0f, 0.96f, 0.88f};
        float     sunIntensity     = 3.0f;
        bool      sunEnabled       = true;
        glm::vec3 ambientSky{0.55f, 0.65f, 0.80f};
        glm::vec3 ambientGround{0.35f, 0.30f, 0.25f};
        float     ambientIntensity = 0.5f;
    };
    FallbackEnvironment fallbackEnvironment_;
    // Worst-case light buffer reused by the per-frame environment snapshot.
    std::vector<direct_lighting::DirectLight> directLightsScratch_;
    // Host shape queried once (the Auto quality preset derives its scale from
    // it; system queries must not run per frame).
    std::uint32_t processorCount_ = util::logicalProcessorCount();

    mutable std::mutex moduleMutex_;
    GiSettings         settings_;
    GiPipelineState    state_;
    // Registry lookup side of state_.surfaces (see findSurfaceLocked).
    std::unordered_map<GiSurfaceHandle, std::uint32_t> surfaceIndexByHandle_;
    std::uint32_t      builtDirectionResolution_ = 0;
    float              builtProbeSpacing_        = 0.0f;
    float              builtLevel1Factor_        = 0.0f;
    std::uint32_t      builtProbeDimension_      = 0;
    CardKey            builtCardKey_{0.0f, 0, 0, 0, 0};
    struct CausticReceiverEntry
    {
        std::uint32_t surface;
        glm::vec3     worldMin;
        glm::vec3     worldMax;
    };
    std::vector<CausticReceiverEntry> causticReceiverIndex_;
    bool                              causticReceiverIndexDirty_ = true;
    double             lastEvaluateMilliseconds_ = 0.0;
    std::atomic<std::uint64_t> probeVersion_{0};
    mutable std::mutex                     snapshotMutex_;
    std::shared_ptr<GiProbeSnapshot> snapshot_;
    // Gather-field snapshot (the copyIrradianceField read side), published
    // under its own snapshot mutex like the probe snapshot.
    mutable std::mutex                     gatherSnapshotMutex_;
    std::shared_ptr<GiGatherSnapshot> gatherSnapshot_;
};

} // namespace
} // namespace radiance_cache
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and its two provided capabilities - "radiance_cache"
// (world probe grid) and "radiance_cache.gather" (screen-space final gather
// field); the pbr and direct_lighting environments are optional inputs, no
// permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"radiance_cache", "radiance_cache.gather"};
    static const char *const                    optional[] = {"pbr", "direct_lighting"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "radiance_cache",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::radiance_cache::RadianceCacheModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 2,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = optional,
            .optionalCapabilityCount = 2,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
