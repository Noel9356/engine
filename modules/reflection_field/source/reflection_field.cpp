// reflection_field module implementation.
//
// Binds the "reflection_field" capability: a registry of reflection scene
// surfaces under one mutex, a camera-driven evaluate() that runs the
// Lumen-style reflection pipeline on the kernel task scheduler (GBuffer,
// GGX-importance-sampled reflection rays through screen traces / scene
// traces / radiance-cache reuse, hit lighting with the radiance_cache probe grid,
// and the spatial-reuse / temporal / bilateral denoising chain), a
// screen-space reflection field snapshot copy API, device profile
// resolution, statistics, and debug-view exports. The heavy lifting lives in
// reflection_scene.cpp, reflection_pipeline.cpp, reflection_denoise.cpp, and
// reflection_debug.cpp.
//
// Lumen reference: the stage structure, roughness bands, and denoising
// behavior follow Epic's published Lumen design ("Lumen: Real-time Global
// Illumination in Unreal Engine 5", SIGGRAPH 2022 Advances course; "Lumen
// Technical Details", Epic Games documentation) scaled to a CPU
// implementation.

#include <atomic>
#include <cstdint>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/reflection_field/capabilities.hpp>

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/device_profile.hpp>
#include <engine/util/hardware_info.hpp>
#include <engine/util/ppm.hpp>
#include "reflection_field_internal.hpp"

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
namespace reflection_field
{

namespace
{

// Writes an error code through an optional out-parameter.
void setError(ReflectionError *output, ReflectionError value)
{
    if (output != nullptr)
    {
        *output = value;
    }
}

// Quality presets and their scaling come from
// engine/renderer/lighting_ray_core.hpp (shared with radiance_cache); Auto derives
// its scale from the host's logical processor count, which this module
// queries once at construction.

void applySettingsToState(const ReflectionSettings &settings, RfPipelineState &state, float autoScale)
{
    state.settings = settings;
    // The quality preset scales the linear resolution (screen_scale is the
    // linear divisor), then the ray budget.
    const float scale = settings.quality == 0 ? autoScale : qualityScale(settings.quality, 0u);
    if (scale != 1.0f)
    {
        const float inverse = 1.0f / scale;
        state.settings.screenScale = std::max<std::uint32_t>(
            static_cast<std::uint32_t>(std::lround(static_cast<float>(settings.screenScale) * inverse)), 1u);
        state.settings.raysPerPixel = std::max<std::uint32_t>(
            static_cast<std::uint32_t>(std::lround(static_cast<float>(settings.raysPerPixel) * scale)), 1u);
        state.settings.spatialReuseRadius = std::max<std::uint32_t>(
            static_cast<std::uint32_t>(std::lround(static_cast<float>(settings.spatialReuseRadius) * scale)), 0u);
    }
    // Static-frame skip convergence: after N clean frames a published texel
    // changes by less than 1% of the fresh trace - exponentially,
    // (1 - strength)^N, once the length ramp reaches the strength cap, or as
    // the running mean's per-frame share 1/(N+1) when strength runs at the
    // unbounded 1.0 - so a frozen field is a converged one (the same contract
    // as radiance_cache's budget-derived convergenceFrames).
    const float strength = std::clamp(settings.temporalStrength, 0.0f, 1.0f);
    // The blend weight is the running-mean ramp N/(N+1) until it reaches the
    // strength cap (at N = strength / (1 - strength)), then the exponential
    // (1 - strength) share. A static frame is converged when a fresh sample
    // moves a published texel by about one percent: that is the ramp's end -
    // freezing earlier would publish a field whose texels are still mostly
    // their last few fresh draws (for strength 0.98 the old exponential-only
    // estimate returned 2 frames, which froze the field long before its
    // history had accumulated anything).
    state.convergenceFrames = strength <= 1.0e-3f
        ? 1u
        : strength >= 0.999f
              ? 100u
              : static_cast<std::uint32_t>(std::clamp(
                    std::ceil(strength / (1.0f - strength)), 1.0f, 128.0f));
    // The registry capacity follows the configured scene bound (the setting
    // is the documented limit createSurface enforces).
    state.registryCapacity = settings.maxSurfaces;
}

// The module singleton backing the "reflection_field" capability. moduleMutex_
// guards the registry, settings, and the whole evaluate() call; statistics
// live in the pipeline state (atomic counters for the parallel stages). The
// reflection field is published as an immutable snapshot, so copyField never
// waits for a running evaluate().
class ReflectionFieldModule final : public ModuleInterface
{
public:
    ReflectionFieldModule()
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
        // Optional providers: the pbr environment + direct lights feed hit
        // lighting, and radiance_cache's world radiance cache supplies the diffuse
        // GI seen in reflections and the rough-lobe radiance reuse.
        pbrCapability_          = context.tryAcquireCapability<pbr::PbrCapability>();
        directLightingCapability_ = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();
        radianceCacheCapability_        = context.tryAcquireCapability<radiance_cache::RadianceCacheCapability>();
        reloadSubscription_     = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<ReflectionFieldModule, &ReflectionFieldModule::onConfigurationReloaded>(this);
        {
            std::lock_guard<std::mutex> lock(moduleMutex_);
            readConfigurationLocked();
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
        state_.tlas.clear();
        state_.tlasSurfaces.clear();
        state_.hasView          = false;
        state_.environmentValid = false;
        // The stopped module owns nothing: the frame GBuffer, the filtered
        // field, and the temporal history are dropped too, so the snapshot
        // published below reports an empty field instead of the last traced
        // frame under a bumped version. A restart rebuilds every one of them
        // from the first evaluate().
        state_.fieldWidth  = 0;
        state_.fieldHeight = 0;
        state_.gbufferDepth.clear();
        state_.gbufferPosition.clear();
        state_.gbufferNormal.clear();
        state_.gbufferAlbedo.clear();
        state_.gbufferRoughness.clear();
        state_.gbufferMetallic.clear();
        state_.gbufferOpacity.clear();
        state_.gbufferIor.clear();
        state_.gbufferEmissive.clear();
        state_.gbufferUnlit.clear();
        state_.gbufferSurface.clear();
        state_.fieldRaw.clear();
        state_.fieldFiltered.clear();
        state_.historyValid = false;
        // The probe-grid copy follows the provider's own stop semantics: a
        // stale grid would keep feeding reflections an environment the
        // radiance_cache no longer stands behind.
        state_.probeGrid              = RfProbeGrid{};
        state_.sceneDirty             = true;
        publishSnapshotLocked();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ReflectionFieldCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                    = "reflection_field",
            .kind                    = ParticipantKind::EngineModule,
            .version                 = "0.1.0",
            .providedCapabilities    = {"reflection_field"},
            .optionalCapabilities    = {"pbr", "direct_lighting", "radiance_cache"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createSurface = [](void *self, const ReflectionSurfaceDescription *description, ReflectionError *error)
        { return static_cast<ReflectionFieldModule *>(self)->createSurfaceImpl(description, error); };
        capability_.updateSurface = [](void *self, ReflectionSurfaceHandle surface,
                                      const ReflectionSurfaceDescription *description, ReflectionError *error)
        { return static_cast<ReflectionFieldModule *>(self)->updateSurfaceImpl(surface, description, error); };
        capability_.updateSurfaceTransform = [](void *self, ReflectionSurfaceHandle surface, const float *transform)
        { return static_cast<ReflectionFieldModule *>(self)->updateSurfaceTransformImpl(surface, transform); };
        capability_.destroySurface = [](void *self, ReflectionSurfaceHandle surface)
        { return static_cast<ReflectionFieldModule *>(self)->destroySurfaceImpl(surface); };
        capability_.setView = [](void *self, const float *viewProjection, const float *cameraPosition,
                                std::uint32_t viewportWidth, std::uint32_t viewportHeight)
        { return static_cast<ReflectionFieldModule *>(self)->setViewImpl(viewProjection, cameraPosition, viewportWidth, viewportHeight); };
        capability_.evaluate = [](void *self, double deltaSeconds, ReflectionError *error)
        { return static_cast<ReflectionFieldModule *>(self)->evaluateImpl(deltaSeconds, error); };
        capability_.copyField = [](void *self, float *output, std::uint32_t capacityTexels,
                                   ReflectionFieldHeader *outHeader, std::uint64_t *outVersion)
        { return static_cast<ReflectionFieldModule *>(self)->copyFieldImpl(output, capacityTexels, outHeader, outVersion); };
        capability_.statistics = [](void *self, ReflectionStatistics *outStatistics)
        { return static_cast<ReflectionFieldModule *>(self)->statisticsImpl(outStatistics); };
        capability_.resolveProfile = [](void *self, const renderer::DeviceProfile *device, ReflectionProfile *outProfile)
        { return static_cast<ReflectionFieldModule *>(self)->resolveProfileImpl(device, outProfile); };
        capability_.debugExport = [](void *self, ReflectionDebugView view, const char *filePath, ReflectionError *error)
        { return static_cast<ReflectionFieldModule *>(self)->debugExportImpl(view, filePath, error); };
    }

    // ---- configuration ------------------------------------------------------

    void readConfigurationLocked()
    {
        ReflectionSettings settings;
        auto integer = [&](const char *key, std::int64_t fallback, std::int64_t minimum, std::int64_t maximum)
        {
            const auto value = context_->configurationIntegerInSection("reflection_field", key);
            if (!value.has_value() || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto floating = [&](const char *key, double fallback, double minimum, double maximum)
        {
            const auto value = context_->configurationFloatingInSection("reflection_field", key);
            if (!value.has_value() || !std::isfinite(*value) || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto boolean = [&](const char *key, bool fallback)
        {
            const auto value = context_->configurationBooleanInSection("reflection_field", key);
            return value.has_value() ? *value : fallback;
        };
        auto environmentVector = [&](const char *key, glm::vec3 fallback, bool nonNegative)
        {
            const auto values = context_->configurationFloatingArrayInSection("reflection_field", key);
            if (values.size() != 3 || !std::isfinite(values[0]) || !std::isfinite(values[1])
                || !std::isfinite(values[2]) || (nonNegative && (values[0] < 0.0 || values[1] < 0.0
                    || values[2] < 0.0)))
            {
                return fallback;
            }
            return glm::vec3(static_cast<float>(values[0]), static_cast<float>(values[1]),
                static_cast<float>(values[2]));
        };
        settings.enabled               = boolean("enabled", true);
        settings.skipUnchangedFrames   = boolean("skip_unchanged_frames", true);
        settings.quality              = static_cast<std::uint32_t>(integer("quality", 3, 0, 4));
        settings.maxSurfaces           = static_cast<std::uint32_t>(integer("scene.max_surfaces", 256, 1, 65536));
        settings.maxTrianglesPerSurface = static_cast<std::uint64_t>(
            integer("scene.max_triangles_per_surface", 200000, 3, 100000000));
        settings.screenScale           = static_cast<std::uint32_t>(integer("screen_scale", 2, 1, 64));
        settings.raysPerPixel          = static_cast<std::uint32_t>(integer("rays_per_pixel", 1, 1, 64));
        settings.mirrorBandTaps        = static_cast<std::uint32_t>(integer("mirror_band_taps", 4, 1, 16));
        settings.screenRayDistance     = static_cast<float>(floating("screen_ray_distance_m", 2.0, 0.05, 64.0));
        settings.roughRayBandMax       = static_cast<float>(floating("roughness.ray_band_max", 0.3, 0.0, 1.0));
        settings.roughMaxTrace         = static_cast<float>(floating("roughness.max_trace", 0.4, 0.0, 1.0));
        settings.maxTraceDistance      = static_cast<float>(floating("trace.max_distance_m", 50.0, 1.0, 2000.0));
        settings.traceBias             = static_cast<float>(floating("trace.bias_m", 0.02, 0.001, 0.5));
        settings.lobeAlphaFloor        = static_cast<float>(floating("trace.lobe_alpha_floor", 0.02, 0.0, 0.25));
        settings.transmissionLobeAlphaFloor
            = static_cast<float>(floating("trace.transmission_lobe_alpha_floor", 0.02, 0.0, 0.25));
        settings.skySunDiscDegrees     = static_cast<float>(
            floating("sky.sun_disc_angular_radius_deg", 2.3, 0.05, 15.0));
        settings.temporalStrength      = static_cast<float>(floating("temporal.strength", 0.99, 0.0, 1.0));
        settings.updateBudgetMilliseconds = floating("update_budget_milliseconds", 0.0, 0.0, 1000.0);
        settings.neighborhoodClampSigma = static_cast<float>(
            floating("temporal.neighborhood_clamp_sigma", 1.0, 0.0, 16.0));
        settings.varianceThreshold    = static_cast<float>(floating("filter.variance_threshold", 0.02, 0.0, 4.0));
        settings.spatialReuseRadius    = static_cast<std::uint32_t>(integer("filter.spatial_reuse_radius", 1, 0, 9));
        settings.bilateralEnabled      = boolean("filter.bilateral", true);
        settings.bilateralRadius       = static_cast<std::uint32_t>(integer("filter.bilateral_radius", 2, 1, 9));
        settings.screenTraceMaxSteps   = static_cast<std::uint32_t>(integer("screen_trace.max_steps", 48, 4, 256));
        settings.screenTraceThickness  = static_cast<float>(floating("screen_trace.thickness_m", 0.25, 0.01, 8.0));
        settings.screenTraceGrazingLimit = static_cast<float>(floating("screen_trace.grazing_limit", 0.25, 0.0, 1.0));
        settings.screenTracesEnabled   = boolean("screen_trace.enabled", true);
        settings.maxTransmissionSegments = static_cast<std::uint32_t>(
            integer("trace.max_transmission_segments", 3, 0, 16));
        settings.referenceSamples      = static_cast<std::uint32_t>(integer("debug.reference_samples", 16, 1, 4096));
        settings_                      = settings;
        // The capacity key is the documented registry bound.
        state_.registryCapacity = settings_.maxSurfaces;
        // Fallback environment (used when the optional pbr capability is
        // absent or its copy fails): the same defaults the renderer's built-in
        // environment carries, overridable so a headless consumer without pbr
        // can still tune the sun and the hemispheric skylight the reflection
        // rays see.
        fallbackEnvironment_.sunDirection = environmentVector("environment.sun_direction",
            glm::vec3(0.45f, 0.62f, 0.35f), false);
        fallbackEnvironment_.sunColor     = environmentVector("environment.sun_color", glm::vec3(1.0f, 0.96f, 0.88f), true);
        fallbackEnvironment_.sunIntensity =
            static_cast<float>(floating("environment.sun_intensity", 3.0, 0.0, 1.0e6));
        fallbackEnvironment_.sunEnabled   = boolean("environment.sun_enabled", true);
        fallbackEnvironment_.ambientSky    = environmentVector("environment.ambient_sky",
            glm::vec3(0.55f, 0.65f, 0.80f), true);
        fallbackEnvironment_.ambientGround = environmentVector("environment.ambient_ground",
            glm::vec3(0.35f, 0.30f, 0.25f), true);
        fallbackEnvironment_.ambientIntensity =
            static_cast<float>(floating("environment.ambient_intensity", 0.5, 0.0, 1.0e6));
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        bool relevant = false;
        for (const std::string &key : event.changedKeys)
        {
            if (key.rfind("reflection_field.", 0) == 0)
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
        applySettingsToState(settings_, state_, qualityScale(0u, processorCount_));
        // A resolution change invalidates the temporal history and the
        // raster caches.
        state_.historyValid = false;
        state_.sceneDirty   = true;
        ++state_.rasterSerial;
        publishSnapshotLocked();
    }

    // ---- surface registry -----------------------------------------------------

    // Handle -> index into state_.surfaces, maintained at every registry
    // mutation under moduleMutex_. Dynamic surfaces re-look up their entry
    // every frame (updateSurfaceTransform), so the registry must answer in
    // O(1) at registry capacities in the tens of thousands, not scan the
    // vector. Destroy swaps the last surface into the hole (see
    // destroySurfaceImpl), which keeps the fix-up O(1) at the cost of a
    // registration order the consumers never observe - they hold handles.
    std::uint32_t findSurfaceLocked(ReflectionSurfaceHandle handle) const
    {
        const auto found = surfaceIndexByHandle_.find(handle);
        if (found == surfaceIndexByHandle_.end() || found->second >= state_.surfaces.size())
        {
            return 0xffffffffu;
        }
        return found->second;
    }

    void applySurfaceDescriptionLocked(RfSurface &surface, const ReflectionSurfaceDescription &description,
        ReflectionError *error)
    {
        RfSurfaceGeometry geometry;
        rfBuildSurfaceGeometry(geometry, description, settings_.maxTrianglesPerSurface, error);
        if (error != nullptr && *error != ReflectionError::None)
        {
            return;
        }
        rfBuildSurfaceBvh(geometry);
        surface.geometry = std::move(geometry);
        surface.albedo   = glm::clamp(glm::vec3(description.albedoColor[0], description.albedoColor[1],
            description.albedoColor[2]), glm::vec3(0.0f), glm::vec3(1.0e6f));
        surface.emissive = glm::max(
            glm::vec3(description.emissiveColor[0], description.emissiveColor[1], description.emissiveColor[2]),
            glm::vec3(0.0f))
            * std::max(description.emissiveIntensity, 0.0f);
        surface.roughness = std::clamp(description.roughness, 0.0f, 1.0f);
        surface.metallic  = std::clamp(description.metallic, 0.0f, 1.0f);
        surface.opacity   = std::clamp(std::isfinite(description.opacity) ? description.opacity : 1.0f, 0.0f, 1.0f);
        surface.indexOfRefraction = std::clamp(
            std::isfinite(description.indexOfRefraction) ? description.indexOfRefraction : 1.5f, 1.0f, 64.0f);
        surface.flagBits = description.flagBits;
        surface.excluded = (description.flagBits & reflectionSurfaceFlagExcluded) != 0u;
        surface.unlit    = (description.flagBits & reflectionSurfaceFlagUnlit) != 0u;
        surface.dynamic  = (description.flagBits & reflectionSurfaceFlagDynamic) != 0u;
        // A singular transform is refused like any other malformed
        // description: the caller reports the error and discards the surface
        // (the registry paths build into a fresh surface, so a rejection never
        // publishes a half-updated one).
        if (!rfUpdateSurfaceTransform(surface, glm::make_mat4(description.transform)))
        {
            if (error != nullptr)
            {
                *error = ReflectionError::InvalidDescription;
            }
        }
    }

    void registryChangedLocked()
    {
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
        // Registry changes shift surface indices: every projected-triangle
        // cache rebuilds on the next raster pass.
        ++state_.rasterSerial;
    }

    ReflectionSurfaceHandle createSurfaceImpl(const ReflectionSurfaceDescription *description, ReflectionError *error)
    {
        if (description == nullptr)
        {
            setError(error, ReflectionError::InvalidDescription);
            return invalidReflectionSurface;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (!validateDescription(*description))
        {
            setError(error, ReflectionError::InvalidDescription);
            return invalidReflectionSurface;
        }
        if (state_.surfaces.size() >= state_.registryCapacity)
        {
            setError(error, ReflectionError::CapacityReached);
            return invalidReflectionSurface;
        }
        if (state_.nextHandle == invalidReflectionSurface || state_.nextHandle == 0xffffffffu)
        {
            setError(error, ReflectionError::CapacityReached);
            return invalidReflectionSurface;
        }
        ReflectionError localError = ReflectionError::None;
        RfSurface       surface;
        surface.handle = state_.nextHandle;
        applySurfaceDescriptionLocked(surface, *description, &localError);
        if (localError != ReflectionError::None)
        {
            setError(error, localError);
            return invalidReflectionSurface;
        }
        // The handle is consumed only once the surface really exists.
        const ReflectionSurfaceHandle handle = state_.nextHandle++;
        surfaceIndexByHandle_.emplace(handle, static_cast<std::uint32_t>(state_.surfaces.size()));
        state_.surfaces.push_back(std::move(surface));
        registryChangedLocked();
        setError(error, ReflectionError::None);
        return handle;
    }

    bool validateDescription(const ReflectionSurfaceDescription &description) const
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
        if (!std::isfinite(description.roughness) || !std::isfinite(description.metallic))
        {
            return false;
        }
        return true;
    }

    bool updateSurfaceImpl(ReflectionSurfaceHandle surface, const ReflectionSurfaceDescription *description,
        ReflectionError *error)
    {
        if (description == nullptr)
        {
            setError(error, ReflectionError::InvalidDescription);
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(surface);
        if (index == 0xffffffffu)
        {
            setError(error, ReflectionError::InvalidHandle);
            return false;
        }
        if (!validateDescription(*description))
        {
            setError(error, ReflectionError::InvalidDescription);
            return false;
        }
        // Build into a copy so a rejected description leaves the live
        // surface untouched.
        RfSurface replacement;
        replacement.handle = surface;
        ReflectionError localError = ReflectionError::None;
        applySurfaceDescriptionLocked(replacement, *description, &localError);
        if (localError != ReflectionError::None)
        {
            setError(error, localError);
            return false;
        }
        state_.surfaces[index] = std::move(replacement);
        registryChangedLocked();
        setError(error, ReflectionError::None);
        return true;
    }

    bool updateSurfaceTransformImpl(ReflectionSurfaceHandle surface, const float *transformColumnMajor16)
    {
        if (transformColumnMajor16 == nullptr)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(surface);
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
        RfSurface   &replacementTarget = state_.surfaces[index];
        const glm::mat4 transform = glm::make_mat4(transformColumnMajor16);
        if (transform == replacementTarget.transform)
        {
            return true; // unchanged: keep the frame clean for the static skip
        }
        // A singular transform is refused and the previous one kept; the
        // caller only sees the boolean.
        if (!rfUpdateSurfaceTransform(replacementTarget, transform))
        {
            return false;
        }
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
        return true;
    }

    bool destroySurfaceImpl(ReflectionSurfaceHandle surface)
    {
        std::lock_guard<std::mutex> lock(moduleMutex_);
        const std::uint32_t index = findSurfaceLocked(surface);
        if (index == 0xffffffffu)
        {
            return false;
        }
        // Swap-and-pop: the last surface moves into the hole and its map
        // entry follows, so removal stays O(1) like the lookup (a middle
        // erase would shift every later index and force an O(n) map walk).
        surfaceIndexByHandle_.erase(surface);
        if (index + 1u < state_.surfaces.size())
        {
            state_.surfaces[index] = std::move(state_.surfaces.back());
            surfaceIndexByHandle_[state_.surfaces[index].handle] = index;
        }
        state_.surfaces.pop_back();
        registryChangedLocked();
        return true;
    }

    bool setViewImpl(const float *viewProjectionColumnMajor16, const float *cameraPosition3, std::uint32_t viewportWidth,
        std::uint32_t viewportHeight)
    {
        if (viewProjectionColumnMajor16 == nullptr || cameraPosition3 == nullptr || viewportWidth == 0
            || viewportHeight == 0)
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
        // (same matrices, same eye, same viewport) leaves the frame clean.
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
            if (state_.viewportWidth != viewportWidth || state_.viewportHeight != viewportHeight)
            {
                viewChanged = true;
            }
        }
        if (viewChanged)
        {
            state_.viewDirty = true;
            // Projected-triangle caches depend on the view matrices.
            ++state_.rasterSerial;
        }
        state_.viewProjection  = glm::make_mat4(viewProjectionColumnMajor16);
        state_.cameraPosition  = glm::make_vec3(cameraPosition3);
        state_.viewportWidth   = viewportWidth;
        state_.viewportHeight  = viewportHeight;
        state_.hasView         = true;
        return true;
    }

    // ---- environment snapshot ---------------------------------------------------

    // Snapshots the pbr environment and direct lights into the frame state.
    // Returns true when the snapshot differs from the previous one (version
    // compare), which is the environment-change signal for the static-frame
    // skip.
    bool snapshotEnvironmentLocked()
    {
        RfEnvironmentSnapshot snapshot;
        std::uint64_t         pbrVersion    = 0;
        std::uint64_t         lightsVersion = 0;
        // Fallback environment first (used when the optional pbr capability is
        // absent or its copy fails): the same defaults the renderer's built-in
        // environment carries, overridable through the
        // [reflection_field.environment] keys so a headless consumer without
        // pbr can still tune the sun and the hemispheric skylight.
        snapshot.hasSun           = true;
        snapshot.sunEnabled       = fallbackEnvironment_.sunEnabled;
        snapshot.sunDirection     = glm::normalize(fallbackEnvironment_.sunDirection);
        if (!std::isfinite(snapshot.sunDirection.x) || !std::isfinite(snapshot.sunDirection.y)
            || !std::isfinite(snapshot.sunDirection.z))
        {
            // A zero (or degenerate) configured direction: fall back to the
            // built-in default instead of a NaN direction.
            snapshot.sunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
        }
        snapshot.sunColor         = fallbackEnvironment_.sunColor;
        snapshot.sunIntensity     = fallbackEnvironment_.sunIntensity;
        snapshot.ambientSky       = fallbackEnvironment_.ambientSky;
        snapshot.ambientGround    = fallbackEnvironment_.ambientGround;
        snapshot.ambientIntensity = fallbackEnvironment_.ambientIntensity;
        pbrVersion = lighting::copyPbrEnvironment(pbrCapability_, snapshot);
        if (directLightingCapability_ != nullptr && directLightingCapability_->copyLights != nullptr)
        {
            // The scratch is a member, not a local (the same steady-state
            // allocation rule radiance_cache's snapshot applies): evaluate() runs
            // every frame, so the worst-case light buffer is reserved once
            // instead of reallocated per frame.
            directLightsScratch_.resize(direct_lighting::maxDirectLights);
            const std::uint32_t count = directLightingCapability_->copyLights(directLightingCapability_->self,
                directLightsScratch_.data(), static_cast<std::uint32_t>(directLightsScratch_.size()), &lightsVersion);
            const direct_lighting::DirectLight *lights = directLightsScratch_.data();
            const std::size_t lightCount = std::min<std::size_t>(count, directLightsScratch_.size());
            // Precompute the per-light constants the shading loop needs and
            // drop inactive lights; the shared lighting::prepareLight fills the
            // same structure radiance_cache snapshots, so the two modules' direct
            // terms stay identical.
            snapshot.preparedLights.reserve(lightCount);
            for (std::size_t index = 0; index < lightCount; ++index)
            {
                RfEnvironmentSnapshot::PreparedDirectLight prepared;
                if (lighting::prepareLight(lights[index], prepared))
                {
                    snapshot.preparedLights.push_back(prepared);
                }
            }
        }
        const bool environmentChanged = !state_.environmentValid
            || pbrVersion != state_.environmentVersion || lightsVersion != state_.directLightsVersion;
        state_.environmentVersion    = pbrVersion;
        state_.directLightsVersion   = lightsVersion;
        state_.environment = std::move(snapshot);
        state_.environmentValid = true;
        return environmentChanged;
    }

    // ---- evaluate: the Lumen-style reflection frame -------------------------------

    TaskScheduler *schedulerLocked()
    {
        return context_ != nullptr ? &context_->taskScheduler() : nullptr;
    }

    bool evaluateImpl(double deltaSeconds, ReflectionError *error)
    {
        (void) deltaSeconds;
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (!state_.hasView)
        {
            setError(error, ReflectionError::ViewNotSet);
            return false;
        }
        if (!settings_.enabled)
        {
            setError(error, ReflectionError::None);
            return true;
        }
        const auto startTime = std::chrono::steady_clock::now();
        const bool environmentChanged = snapshotEnvironmentLocked();
        // The world radiance cache feeding hit lighting (and the rough-lobe
        // reuse band) converges over several frames: a probe-grid version
        // change is a dirty signal exactly like an environment change, so the
        // static skip never freezes a field traced against unconverged GI.
        const bool probeGridChanged = rfRefreshProbeGrid(state_, radianceCacheCapability_);
        // The same coupling's caustics read: hit shading samples the photon
        // pass's deposited irradiance through the capability (see
        // rfShadeHitPoint); the pointer is refreshed every frame so a late
        // acquire (the provider module loading after this one) is picked up.
        state_.radianceCacheCapability = radianceCacheCapability_;
        ++state_.frameIndex;

        // Static-frame skip: when the camera, the scene, and the environment
        // are all unchanged and the temporal history has had the blend's
        // convergence window to settle (see convergenceFrames), re-running
        // would only re-blend the same radiance. Return early - and keep the
        // field version so a renderer polling copyField skips its upload too.
        const bool viewChanged  = state_.viewDirty;
        const bool sceneChanged = state_.sceneDirty;
        state_.viewDirty  = false;
        state_.sceneDirty = false;
        if (viewChanged || sceneChanged || environmentChanged || probeGridChanged)
        {
            state_.framesSinceDirty = 0;
        }
        if (settings_.skipUnchangedFrames && state_.hasPreviousView && !viewChanged && !sceneChanged
            && !environmentChanged && !probeGridChanged
            && state_.framesSinceDirty >= state_.convergenceFrames)
        {
            state_.framesSkippedStatic.fetch_add(1, std::memory_order_relaxed);
            lastEvaluateMilliseconds_ = 0.0;
            setError(error, ReflectionError::None);
            return true;
        }
        ++state_.framesSinceDirty;

        TaskScheduler *scheduler = schedulerLocked();
        rfEnsureSceneTlas(state_);
        applySettingsToState(settings_, state_, qualityScale(0u, processorCount_));
        if (!rfRunReflectionFrame(state_, scheduler))
        {
            state_.framesSinceDirty = 0;
            state_.sceneDirty       = true;
            setError(error, ReflectionError::EvaluationFailed);
            return false;
        }
        state_.hasPreviousView = true;
        publishSnapshotLocked();

        const auto endTime = std::chrono::steady_clock::now();
        lastEvaluateMilliseconds_ =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count())
            / 1000.0;
        // Adaptive stride governor (update_budget_milliseconds): an EMA of
        // the measured evaluate cost retunes the trace interleave in integer
        // steps (hysteresis: at most one step every 8 frames), so the field
        // refreshes at whatever rate THIS machine can afford inside the
        // budget. Stride 1 = every texel every frame (the fixed pipeline).
        if (settings_.updateBudgetMilliseconds > 0.0 && lastEvaluateMilliseconds_ > 0.0)
        {
            state_.tracePhase += 1;
            state_.governorMeasuredMs = state_.governorMeasuredMs <= 0.0
                ? lastEvaluateMilliseconds_
                : 0.7 * state_.governorMeasuredMs + 0.3 * lastEvaluateMilliseconds_;
            if (state_.governorHoldFrames > 0)
            {
                --state_.governorHoldFrames;
            }
            else
            {
                const double budget = settings_.updateBudgetMilliseconds;
                if (state_.governorMeasuredMs > budget && state_.traceStride < 8)
                {
                    state_.traceStride += 1;
                    state_.governorHoldFrames = 8;
                }
                else if (state_.governorMeasuredMs < budget * 0.7 && state_.traceStride > 1)
                {
                    state_.traceStride -= 1;
                    state_.governorHoldFrames = 8;
                }
            }
        }
        setError(error, ReflectionError::None);
        return true;
    }

    // ---- read side ---------------------------------------------------------------

    // Builds and publishes the immutable field snapshot (module mutex held).
    // The previous snapshot's buffer is recycled into the new snapshot when
    // no renderer still holds a reference, so steady-state publishing does
    // not reallocate per frame.
    void publishSnapshotLocked()
    {
        std::shared_ptr<FieldSnapshot> previous;
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            previous = snapshot_;
        }
        auto snapshot = std::make_shared<FieldSnapshot>();
        if (previous != nullptr && previous.use_count() == 1)
        {
            snapshot->radiance  = std::move(previous->radiance);
            snapshot->roughness = std::move(previous->roughness);
        }
        buildFieldSnapshot(state_, *snapshot);
        snapshot->version = fieldVersion_.fetch_add(1u, std::memory_order_acq_rel) + 1u;
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot_ = std::move(snapshot);
    }

    std::uint32_t copyFieldImpl(float *output, std::uint32_t capacityTexels, ReflectionFieldHeader *outHeader,
        std::uint64_t *outVersion) const
    {
        // Only the tiny snapshot mutex: a renderer polling every frame never
        // waits for a running evaluate().
        std::shared_ptr<const FieldSnapshot> snapshot;
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
                *outHeader = ReflectionFieldHeader{};
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
            return snapshot->header.width != 0 && snapshot->header.height != 0
                ? snapshot->header.width * snapshot->header.height
                : 0;
        }
        const std::uint32_t available = snapshot->header.width != 0 && snapshot->header.height != 0
            ? snapshot->header.width * snapshot->header.height
            : 0;
        const std::uint32_t copied = std::min(available, capacityTexels);
        if (copied != 0)
        {
            // Interleaved texels (see reflectionFieldTexelFloats): the
            // radiance triplet, then the texel's surface roughness (the
            // renderer's sample gate material key).
            for (std::uint32_t texel = 0; texel < copied; ++texel)
            {
                output[texel * reflectionFieldTexelFloats + 0] = snapshot->radiance[texel * 3 + 0];
                output[texel * reflectionFieldTexelFloats + 1] = snapshot->radiance[texel * 3 + 1];
                output[texel * reflectionFieldTexelFloats + 2] = snapshot->radiance[texel * 3 + 2];
                output[texel * reflectionFieldTexelFloats + 3] = snapshot->roughness[texel];
            }
        }
        return copied;
    }

    bool statisticsImpl(ReflectionStatistics *outStatistics) const
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        // The registry and the counters mutate under the module mutex: a
        // statistics poll from another thread must not race the surfaces
        // vector while an evaluate() frame is running.
        std::lock_guard<std::mutex> lock(moduleMutex_);
        ReflectionStatistics &statistics = *outStatistics;
        statistics.frameIndex               = state_.frameIndex;
        statistics.surfaces                 = static_cast<std::uint32_t>(state_.surfaces.size());
        std::uint64_t triangles            = 0;
        for (const RfSurface &surface : state_.surfaces)
        {
            triangles += surface.geometry.indices.size() / 3;
        }
        statistics.triangles                = triangles;
        statistics.fieldWidth               = state_.fieldWidth;
        statistics.fieldHeight              = state_.fieldHeight;
        statistics.pixelsTracedThisFrame    = state_.pixelsTracedThisFrame.load(std::memory_order_relaxed);
        statistics.raysTracedThisFrame      = state_.raysTracedThisFrame.load(std::memory_order_relaxed);
        statistics.raysReusedCacheThisFrame = state_.raysReusedCacheThisFrame.load(std::memory_order_relaxed);
        statistics.screenTraceHitsThisFrame = state_.screenTraceHitsThisFrame.load(std::memory_order_relaxed);
        statistics.sceneTraceHitsThisFrame  = state_.sceneTraceHitsThisFrame.load(std::memory_order_relaxed);
        statistics.transmissionSegmentsThisFrame
            = state_.transmissionSegmentsThisFrame.load(std::memory_order_relaxed);
        statistics.totalRaysTraced          = state_.totalRaysTraced.load(std::memory_order_relaxed);
        statistics.framesSkippedStatic      = state_.framesSkippedStatic.load(std::memory_order_relaxed);
        statistics.lastEvaluateMilliseconds = lastEvaluateMilliseconds_;
        return true;
    }

    bool resolveProfileImpl(const renderer::DeviceProfile *device, ReflectionProfile *outProfile) const
    {
        if (outProfile == nullptr)
        {
            return false;
        }
        // Settings mutate under the module mutex (live reload); the profile
        // reads them under the same lock.
        std::lock_guard<std::mutex> lock(moduleMutex_);
        ReflectionProfile profile;
        profile.screenScale      = settings_.screenScale;
        profile.raysPerPixel     = settings_.raysPerPixel;
        profile.temporalStrength = settings_.temporalStrength;
        if (device != nullptr)
        {
            // A CPU device halves the linear resolution; a discrete GPU runs
            // the configured values; integrated and virtual take one step
            // coarser.
            switch (device->deviceClass)
            {
            case renderer::DeviceClass::DiscreteGpu:
                break;
            case renderer::DeviceClass::Cpu:
                profile.screenScale = std::min(profile.screenScale * 2u, 64u);
                profile.raysPerPixel = 1u;
                break;
            case renderer::DeviceClass::IntegratedGpu:
            case renderer::DeviceClass::VirtualGpu:
            case renderer::DeviceClass::Unknown:
            default:
                profile.screenScale = std::min(profile.screenScale + profile.screenScale / 2u, 64u);
                break;
            }
        }
        *outProfile = profile;
        return true;
    }

    bool debugExportImpl(ReflectionDebugView view, const char *filePath, ReflectionError *error)
    {
        if (filePath == nullptr || filePath[0] == '\0')
        {
            setError(error, ReflectionError::ExportFailed);
            return false;
        }
        std::lock_guard<std::mutex> lock(moduleMutex_);
        std::vector<std::uint8_t> rgb;
        if (!rfComposeDebugView(state_, schedulerLocked(), view, rgb))
        {
            setError(error, ReflectionError::ExportFailed);
            return false;
        }
        // The shared util writer (engine::util::writePpmRgb) - the same
        // overflow-guarded export path radiance_cache's debug views use.
        if (!util::writePpmRgb(filePath, state_.fieldWidth, state_.fieldHeight, rgb.data(),
                static_cast<std::size_t>(state_.fieldWidth) * 3u))
        {
            setError(error, ReflectionError::ExportFailed);
            return false;
        }
        setError(error, ReflectionError::None);
        return true;
    }

    // ---- data ------------------------------------------------------------------

    struct FieldSnapshot
    {
        ReflectionFieldHeader header{};
        std::vector<float>    radiance;  // width * height * 3, linear HDR
        std::vector<float>    roughness; // width * height, the texel's surface roughness
        std::uint64_t         version = 0;
    };

    void buildFieldSnapshot(const RfPipelineState &state, FieldSnapshot &snapshot) const
    {
        ReflectionFieldHeader header;
        header.width     = state.fieldWidth;
        header.height    = state.fieldHeight;
        std::memcpy(header.viewProjection, glm::value_ptr(state.viewProjection), sizeof(header.viewProjection));
        std::memcpy(header.cameraPosition, glm::value_ptr(state.cameraPosition), sizeof(header.cameraPosition));
        header.maxRoughness = state.settings.roughMaxTrace;
        snapshot.header     = header;
        const std::size_t texelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
        // resize (not assign): at steady state the buffer keeps its size, so
        // the loop below overwrites every texel without re-zeroing it first.
        snapshot.radiance.resize(texelCount * 3);
        snapshot.roughness.resize(texelCount);
        if (!state.fieldFiltered.empty())
        {
            for (std::size_t texel = 0; texel < texelCount; ++texel)
            {
                snapshot.radiance[texel * 3 + 0] = state.fieldFiltered[texel].x;
                snapshot.radiance[texel * 3 + 1] = state.fieldFiltered[texel].y;
                snapshot.radiance[texel * 3 + 2] = state.fieldFiltered[texel].z;
                // The texel's surface roughness travels with the radiance:
                // texels the pass never traced (background, unlit, above the
                // roughness band) keep the GBuffer's neutral 1.0, which the
                // renderer's sample gate reads as "no response here".
                snapshot.roughness[texel] = state.gbufferRoughness[texel];
            }
        }
        else
        {
            std::fill(snapshot.roughness.begin(), snapshot.roughness.end(), 1.0f);
        }
    }

    ParticipantManifest                    manifest_;
    ParticipantContext                    *context_ = nullptr;
    pbr::PbrCapability                    *pbrCapability_ = nullptr;
    direct_lighting::DirectLightingCapability *directLightingCapability_ = nullptr;
    radiance_cache::RadianceCacheCapability           *radianceCacheCapability_ = nullptr;
    SubscriptionToken                      reloadSubscription_ = 0;
    // Worst-case light buffer reused by the per-frame environment snapshot.
    std::vector<direct_lighting::DirectLight> directLightsScratch_;
    // Host shape queried once (the Auto quality preset derives its scale from
    // it; system queries must not run per frame).
    std::uint32_t                          processorCount_ = util::logicalProcessorCount();

    mutable std::mutex moduleMutex_;
    mutable std::mutex snapshotMutex_;
    std::shared_ptr<FieldSnapshot>   snapshot_;
    std::atomic<std::uint64_t>       fieldVersion_{0};

    ReflectionSettings                  settings_;
    RfPipelineState                     state_;
    // Registry lookup side of state_.surfaces (see findSurfaceLocked).
    std::unordered_map<ReflectionSurfaceHandle, std::uint32_t> surfaceIndexByHandle_;
    // Built-in environment used when the pbr capability is absent or its copy
    // fails; fed from the [reflection_field.environment] configuration keys.
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
    ReflectionFieldCapability           capability_{};
    double                              lastEvaluateMilliseconds_ = 0.0;
};

} // namespace

} // namespace reflection_field
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "reflection_field" capability;
// the pbr and direct_lighting environments and the radiance_cache world radiance
// cache are optional inputs, no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"reflection_field"};
    static const char *const                    optional[] = {"pbr", "direct_lighting", "radiance_cache"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "reflection_field",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::reflection_field::ReflectionFieldModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = optional,
            .optionalCapabilityCount = 3,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
