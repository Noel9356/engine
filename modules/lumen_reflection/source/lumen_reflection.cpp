// lumen_reflection module implementation.
//
// Binds the "lumen_reflection" capability: a registry of reflection scene
// surfaces under one mutex, a camera-driven evaluate() that runs the
// Lumen-style reflection pipeline on the kernel task scheduler (GBuffer,
// GGX-importance-sampled reflection rays through screen traces / scene
// traces / radiance-cache reuse, hit lighting with the lumen_gi probe grid,
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
#include <engine/lumen_reflection/capabilities.hpp>

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/device_profile.hpp>
#include "lumen_reflection_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace engine
{
namespace lumen_reflection
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

// Quality presets scale the field resolution and the ray budget like the
// render graph's device resolution: Low 0.25x, Medium 0.5x, High 1x, Ultra
// 2x of the configured values; Auto resolves through resolveProfile's device
// class.
float qualityScale(std::uint32_t quality)
{
    switch (quality)
    {
    case 1: // Low
        return 0.25f;
    case 2: // Medium
        return 0.5f;
    case 4: // Ultra
        return 2.0f;
    case 3: // High
    default:
        return 1.0f;
    }
}

void applySettingsToState(const ReflectionSettings &settings, RfPipelineState &state)
{
    state.settings = settings;
    // The quality preset scales the linear resolution (screen_scale is the
    // linear divisor), then the ray budget.
    const float scale = settings.quality == 0 ? 1.0f : qualityScale(settings.quality);
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
}

// The module singleton backing the "lumen_reflection" capability. moduleMutex_
// guards the registry, settings, and the whole evaluate() call; statistics
// live in the pipeline state (atomic counters for the parallel stages). The
// reflection field is published as an immutable snapshot, so copyField never
// waits for a running evaluate().
class LumenReflectionModule final : public ModuleInterface
{
public:
    LumenReflectionModule()
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
        // lighting, and lumen_gi's world radiance cache supplies the diffuse
        // GI seen in reflections and the rough-lobe radiance reuse.
        pbrCapability_          = context.tryAcquireCapability<pbr::PbrCapability>();
        directLightingCapability_ = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();
        lumenGiCapability_        = context.tryAcquireCapability<lumen_gi::LumenGiCapability>();
        reloadSubscription_     = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<LumenReflectionModule, &LumenReflectionModule::onConfigurationReloaded>(this);
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
        state_.tlas.clear();
        state_.tlasSurfaces.clear();
        state_.hasView          = false;
        state_.environmentValid = false;
        publishSnapshotLocked();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == LumenReflectionCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                    = "lumen_reflection",
            .kind                    = ParticipantKind::EngineModule,
            .version                 = "0.1.0",
            .providedCapabilities    = {"lumen_reflection"},
            .optionalCapabilities    = {"pbr", "direct_lighting", "lumen_gi"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createSurface = [](void *self, const ReflectionSurfaceDescription *description, ReflectionError *error)
        { return static_cast<LumenReflectionModule *>(self)->createSurfaceImpl(description, error); };
        capability_.updateSurface = [](void *self, ReflectionSurfaceHandle surface,
                                      const ReflectionSurfaceDescription *description, ReflectionError *error)
        { return static_cast<LumenReflectionModule *>(self)->updateSurfaceImpl(surface, description, error); };
        capability_.updateSurfaceTransform = [](void *self, ReflectionSurfaceHandle surface, const float *transform)
        { return static_cast<LumenReflectionModule *>(self)->updateSurfaceTransformImpl(surface, transform); };
        capability_.destroySurface = [](void *self, ReflectionSurfaceHandle surface)
        { return static_cast<LumenReflectionModule *>(self)->destroySurfaceImpl(surface); };
        capability_.setView = [](void *self, const float *viewProjection, const float *cameraPosition,
                                std::uint32_t viewportWidth, std::uint32_t viewportHeight)
        { return static_cast<LumenReflectionModule *>(self)->setViewImpl(viewProjection, cameraPosition, viewportWidth, viewportHeight); };
        capability_.evaluate = [](void *self, double deltaSeconds, ReflectionError *error)
        { return static_cast<LumenReflectionModule *>(self)->evaluateImpl(deltaSeconds, error); };
        capability_.copyField = [](void *self, float *output, std::uint32_t capacityTexels,
                                   ReflectionFieldHeader *outHeader, std::uint64_t *outVersion)
        { return static_cast<LumenReflectionModule *>(self)->copyFieldImpl(output, capacityTexels, outHeader, outVersion); };
        capability_.statistics = [](void *self, ReflectionStatistics *outStatistics)
        { return static_cast<LumenReflectionModule *>(self)->statisticsImpl(outStatistics); };
        capability_.resolveProfile = [](void *self, const renderer::DeviceProfile *device, ReflectionProfile *outProfile)
        { return static_cast<LumenReflectionModule *>(self)->resolveProfileImpl(device, outProfile); };
        capability_.debugExport = [](void *self, ReflectionDebugView view, const char *filePath, ReflectionError *error)
        { return static_cast<LumenReflectionModule *>(self)->debugExportImpl(view, filePath, error); };
    }

    // ---- configuration ------------------------------------------------------

    void readConfigurationLocked()
    {
        ReflectionSettings settings;
        auto integer = [&](const char *key, std::int64_t fallback, std::int64_t minimum, std::int64_t maximum)
        {
            const auto value = context_->configurationIntegerInSection("lumen_reflection", key);
            if (!value.has_value() || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto floating = [&](const char *key, double fallback, double minimum, double maximum)
        {
            const auto value = context_->configurationFloatingInSection("lumen_reflection", key);
            if (!value.has_value() || !std::isfinite(*value) || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto boolean = [&](const char *key, bool fallback)
        {
            const auto value = context_->configurationBooleanInSection("lumen_reflection", key);
            return value.has_value() ? *value : fallback;
        };
        settings.enabled               = boolean("enabled", true);
        settings.skipUnchangedFrames   = boolean("skip_unchanged_frames", true);
        settings.quality              = static_cast<std::uint32_t>(integer("quality", 3, 0, 4));
        settings.maxSurfaces           = static_cast<std::uint32_t>(integer("scene.max_surfaces", 256, 1, 65536));
        settings.maxTrianglesPerSurface = static_cast<std::uint64_t>(
            integer("scene.max_triangles_per_surface", 200000, 3, 100000000));
        settings.screenScale           = static_cast<std::uint32_t>(integer("screen_scale", 4, 1, 64));
        settings.raysPerPixel          = static_cast<std::uint32_t>(integer("rays_per_pixel", 1, 1, 64));
        settings.screenRayDistance     = static_cast<float>(floating("screen_ray_distance_m", 2.0, 0.05, 64.0));
        settings.roughRayBandMax       = static_cast<float>(floating("roughness.ray_band_max", 0.3, 0.0, 1.0));
        settings.roughMaxTrace         = static_cast<float>(floating("roughness.max_trace", 0.4, 0.0, 1.0));
        settings.maxTraceDistance      = static_cast<float>(floating("trace.max_distance_m", 50.0, 1.0, 2000.0));
        settings.traceBias             = static_cast<float>(floating("trace.bias_m", 0.02, 0.001, 0.5));
        settings.temporalStrength      = static_cast<float>(floating("temporal.strength", 0.9, 0.0, 0.99));
        settings.neighborhoodClampSigma = static_cast<float>(
            floating("temporal.neighborhood_clamp_sigma", 1.0, 0.0, 16.0));
        settings.spatialReuseRadius    = static_cast<std::uint32_t>(integer("filter.spatial_reuse_radius", 1, 0, 9));
        settings.bilateralEnabled      = boolean("filter.bilateral", true);
        settings.bilateralRadius       = static_cast<std::uint32_t>(integer("filter.bilateral_radius", 2, 1, 9));
        settings.referenceSamples      = static_cast<std::uint32_t>(integer("debug.reference_samples", 16, 1, 4096));
        settings_                      = settings;
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        bool relevant = false;
        for (const std::string &key : event.changedKeys)
        {
            if (key.rfind("lumen_reflection.", 0) == 0)
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
        applySettingsToState(settings_, state_);
        // A resolution change invalidates the temporal history.
        state_.historyValid = false;
        state_.sceneDirty   = true;
        publishSnapshotLocked();
    }

    // ---- surface registry -----------------------------------------------------

    std::uint32_t findSurfaceLocked(ReflectionSurfaceHandle handle) const
    {
        for (std::size_t index = 0; index < state_.surfaces.size(); ++index)
        {
            if (state_.surfaces[index].handle == handle)
            {
                return static_cast<std::uint32_t>(index);
            }
        }
        return 0xffffffffu;
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
        surface.flagBits = description.flagBits;
        surface.excluded = (description.flagBits & reflectionSurfaceFlagExcluded) != 0u;
        surface.unlit    = (description.flagBits & reflectionSurfaceFlagUnlit) != 0u;
        surface.dynamic  = (description.flagBits & reflectionSurfaceFlagDynamic) != 0u;
        rfUpdateSurfaceTransform(surface, glm::make_mat4(description.transform));
    }

    void registryChangedLocked()
    {
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
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
        rfUpdateSurfaceTransform(replacementTarget, transform);
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
        state_.surfaces.erase(state_.surfaces.begin() + index);
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
        snapshot.sunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
        snapshot.sunColor     = glm::vec3(1.0f, 0.96f, 0.88f);
        snapshot.sunIntensity = 3.0f;
        snapshot.sunEnabled   = true;
        snapshot.ambientSky     = glm::vec3(0.55f, 0.65f, 0.80f);
        snapshot.ambientGround  = glm::vec3(0.35f, 0.30f, 0.25f);
        snapshot.ambientIntensity = 0.5f;
        snapshot.exposure       = 1.0f;
        snapshot.toneMapping    = pbr::ToneMapping::AcesFilmic;
        if (pbrCapability_ != nullptr && pbrCapability_->copyEnvironment != nullptr)
        {
            pbr::PbrEnvironment environment{};
            std::uint64_t       version = 0;
            if (pbrCapability_->copyEnvironment(pbrCapability_->self, &environment, &version))
            {
                pbrVersion = version;
                snapshot.hasSun           = true;
                snapshot.sunEnabled       = environment.sun.enabled;
                const glm::vec3 sun(environment.sun.direction[0], environment.sun.direction[1], environment.sun.direction[2]);
                if (glm::dot(sun, sun) > 1.0e-12f)
                {
                    snapshot.sunDirection = glm::normalize(sun);
                }
                snapshot.sunColor         = glm::vec3(
                    environment.sun.color[0], environment.sun.color[1], environment.sun.color[2]);
                snapshot.sunIntensity     = environment.sun.intensity;
                snapshot.ambientSky       = glm::vec3(environment.ambient.skyColor[0], environment.ambient.skyColor[1],
                    environment.ambient.skyColor[2]);
                snapshot.ambientGround    = glm::vec3(environment.ambient.groundColor[0],
                    environment.ambient.groundColor[1], environment.ambient.groundColor[2]);
                snapshot.ambientIntensity = environment.ambient.intensity;
                snapshot.exposure         = environment.exposure;
                snapshot.toneMapping      = environment.toneMapping;
            }
        }
        if (directLightingCapability_ != nullptr && directLightingCapability_->copyLights != nullptr)
        {
            std::vector<direct_lighting::DirectLight> lights(direct_lighting::maxDirectLights);
            const std::uint32_t count = directLightingCapability_->copyLights(directLightingCapability_->self,
                lights.data(), static_cast<std::uint32_t>(lights.size()), &lightsVersion);
            lights.resize(std::min<std::size_t>(count, lights.size()));
            // Precompute the per-light constants the shading loop needs and
            // drop inactive lights.
            for (const direct_lighting::DirectLight &light : lights)
            {
                const direct_lighting::DirectLightDescription &description = light.description;
                const glm::vec3 color(description.color[0], description.color[1], description.color[2]);
                if (description.intensity <= 0.0f || (color.x <= 0.0f && color.y <= 0.0f && color.z <= 0.0f))
                {
                    continue;
                }
                RfEnvironmentSnapshot::PreparedDirectLight prepared;
                prepared.type      = description.type;
                prepared.color     = color;
                prepared.intensity = description.intensity;
                prepared.position  = glm::vec3(description.position[0], description.position[1], description.position[2]);
                prepared.range     = std::max(description.range, 1.0e-3f);
                if (description.type == direct_lighting::DirectLightType::Directional)
                {
                    prepared.direction = -glm::normalize(
                        glm::vec3(description.direction[0], description.direction[1], description.direction[2]));
                }
                else if (description.type == direct_lighting::DirectLightType::Spot)
                {
                    prepared.direction = glm::normalize(
                        glm::vec3(description.direction[0], description.direction[1], description.direction[2]));
                    prepared.innerCosine = std::cos(description.innerConeRadians);
                    prepared.outerCosine = std::cos(description.outerConeRadians);
                    prepared.coneDivisor = std::max(prepared.innerCosine - prepared.outerCosine, 1.0e-5f);
                }
                else
                {
                    prepared.direction = glm::vec3(0.0f);
                }
                snapshot.preparedLights.push_back(prepared);
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
        const bool probeGridChanged = rfRefreshProbeGrid(state_, lumenGiCapability_);
        ++state_.frameIndex;

        // Static-frame skip: when the camera, the scene, and the environment
        // are all unchanged and the temporal history has had a full frame to
        // accumulate, re-running would only re-trace the same rays. Return
        // early - and keep the field version so a renderer polling copyField
        // skips its upload too.
        const bool viewChanged  = state_.viewDirty;
        const bool sceneChanged = state_.sceneDirty;
        state_.viewDirty  = false;
        state_.sceneDirty = false;
        if (viewChanged || sceneChanged || environmentChanged || probeGridChanged)
        {
            state_.framesSinceDirty = 0;
        }
        if (settings_.skipUnchangedFrames && state_.hasPreviousView && !viewChanged && !sceneChanged
            && !environmentChanged && !probeGridChanged && state_.framesSinceDirty >= 1)
        {
            state_.framesSkippedStatic.fetch_add(1, std::memory_order_relaxed);
            lastEvaluateMilliseconds_ = 0.0;
            setError(error, ReflectionError::None);
            return true;
        }
        ++state_.framesSinceDirty;

        TaskScheduler *scheduler = schedulerLocked();
        rfEnsureSceneTlas(state_);
        applySettingsToState(settings_, state_);
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
            snapshot->radiance = std::move(previous->radiance);
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
            std::memcpy(output, snapshot->radiance.data(), static_cast<std::size_t>(copied) * 3 * sizeof(float));
        }
        return copied;
    }

    bool statisticsImpl(ReflectionStatistics *outStatistics) const
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
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
        std::ofstream file(filePath, std::ios::binary | std::ios::trunc);
        if (!file)
        {
            setError(error, ReflectionError::ExportFailed);
            return false;
        }
        const std::uint32_t width  = state_.fieldWidth;
        const std::uint32_t height = state_.fieldHeight;
        file << "P6\n" << width << ' ' << height << "\n255\n";
        file.write(reinterpret_cast<const char *>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
        if (!file)
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
        std::vector<float>    radiance; // width * height * 3, linear HDR
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
        snapshot.radiance.assign(texelCount * 3, 0.0f);
        if (!state.fieldFiltered.empty())
        {
            for (std::size_t texel = 0; texel < texelCount; ++texel)
            {
                snapshot.radiance[texel * 3 + 0] = state.fieldFiltered[texel].x;
                snapshot.radiance[texel * 3 + 1] = state.fieldFiltered[texel].y;
                snapshot.radiance[texel * 3 + 2] = state.fieldFiltered[texel].z;
            }
        }
    }

    ParticipantManifest                    manifest_;
    ParticipantContext                    *context_ = nullptr;
    pbr::PbrCapability                    *pbrCapability_ = nullptr;
    direct_lighting::DirectLightingCapability *directLightingCapability_ = nullptr;
    lumen_gi::LumenGiCapability           *lumenGiCapability_ = nullptr;
    SubscriptionToken                      reloadSubscription_ = 0;

    mutable std::mutex moduleMutex_;
    mutable std::mutex snapshotMutex_;
    std::shared_ptr<FieldSnapshot>   snapshot_;
    std::atomic<std::uint64_t>       fieldVersion_{0};

    ReflectionSettings                  settings_;
    RfPipelineState                     state_;
    LumenReflectionCapability           capability_{};
    double                              lastEvaluateMilliseconds_ = 0.0;
};

} // namespace

} // namespace lumen_reflection
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "lumen_reflection" capability;
// the pbr and direct_lighting environments and the lumen_gi world radiance
// cache are optional inputs, no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"lumen_reflection"};
    static const char *const                    optional[] = {"pbr", "direct_lighting", "lumen_gi"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "lumen_reflection",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::lumen_reflection::LumenReflectionModule(); },
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
