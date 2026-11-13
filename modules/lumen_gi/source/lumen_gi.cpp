// lumen_gi module implementation.
//
// Binds the "lumen_gi" capability: a registry of GI scene surfaces under one
// mutex, a camera-driven evaluate() that runs the Lumen-style pipeline stages
// on the kernel task scheduler (surface cache capture, amortized direct and
// indirect lighting, world-space radiance cache, screen-space final gather),
// a probe-grid snapshot copy API, device profile resolution, statistics, and
// debug-view exports. The heavy lifting lives in lumen_gi_scene.cpp,
// lumen_gi_lighting.cpp, lumen_gi_debug.cpp, and lumen_gi_debug_views.cpp.
//
// Lumen reference: the stage structure, budgets, and blending behaviors
// follow Epic's published Lumen design ("Lumen: Real-time Global Illumination
// in Unreal Engine 5", SIGGRAPH 2022 Advances course; "Lumen Technical
// Details", Epic Games documentation) scaled to a CPU implementation.

#include <engine/lumen_gi/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>

namespace engine
{
namespace lumen_gi
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

// Quality presets scale the amortization budgets like the render graph's
// device resolution: Low 0.25x, Medium 0.5x, High 1x, Ultra 2x of the
// configured values; Auto resolves through resolveProfile's device class.
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

void applySettingsToState(const GiSettings &settings, GiPipelineState &state)
{
    state.settings = settings;
    const float scale = settings.quality == 0 ? 1.0f : qualityScale(settings.quality);
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
}


// The module singleton backing the "lumen_gi" capability. moduleMutex_ guards
// the registry, settings, and the whole evaluate() call; statistics live in
// the pipeline state (atomic counters for the parallel stages). The probe
// grid is published as an immutable snapshot, so copyProbes never waits for a
// running evaluate().
class LumenGiModule final : public ModuleInterface
{
public:
    LumenGiModule()
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
                                .subscribe<LumenGiModule, &LumenGiModule::onConfigurationReloaded>(this);
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
        state_.cardRefs.clear();
        state_.tlas.clear();
        state_.tlasSurfaces.clear();
        state_.hasView          = false;
        state_.environmentValid = false;
        publishSnapshotLocked();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == LumenGiCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                    = "lumen_gi",
            .kind                    = ParticipantKind::EngineModule,
            .version                 = "0.1.0",
            .providedCapabilities    = {"lumen_gi"},
            .optionalCapabilities    = {"pbr", "direct_lighting"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createSurface = [](void *self, const GiSurfaceDescription *description, GiError *error)
        { return static_cast<LumenGiModule *>(self)->createSurfaceImpl(description, error); };
        capability_.updateSurface = [](void *self, GiSurfaceHandle surface, const GiSurfaceDescription *description,
                                      GiError *error)
        { return static_cast<LumenGiModule *>(self)->updateSurfaceImpl(surface, description, error); };
        capability_.updateSurfaceTransform = [](void *self, GiSurfaceHandle surface, const float *transform)
        { return static_cast<LumenGiModule *>(self)->updateSurfaceTransformImpl(surface, transform); };
        capability_.destroySurface = [](void *self, GiSurfaceHandle surface)
        { return static_cast<LumenGiModule *>(self)->destroySurfaceImpl(surface); };
        capability_.setView = [](void *self, const float *viewProjection, const float *cameraPosition)
        { return static_cast<LumenGiModule *>(self)->setViewImpl(viewProjection, cameraPosition); };
        capability_.evaluate = [](void *self, double deltaSeconds, GiError *error)
        { return static_cast<LumenGiModule *>(self)->evaluateImpl(deltaSeconds, error); };
        capability_.copyProbes = [](void                          *self,
                                  GiProbe                        *output,
                                  std::uint32_t                   capacity,
                                  GiProbeHeader                  *outHeader,
                                  std::uint64_t                  *outVersion)
        { return static_cast<LumenGiModule *>(self)->copyProbesImpl(output, capacity, outHeader, outVersion); };
        capability_.statistics = [](void *self, GiStatistics *outStatistics)
        { return static_cast<LumenGiModule *>(self)->statisticsImpl(outStatistics); };
        capability_.resolveProfile = [](void *self, const renderer::DeviceProfile *device, GiProfile *outProfile)
        { return static_cast<LumenGiModule *>(self)->resolveProfileImpl(device, outProfile); };
        capability_.debugExport = [](void *self, GiDebugView view, const char *filePath, GiError *error)
        { return static_cast<LumenGiModule *>(self)->debugExportImpl(view, filePath, error); };
    }

    // ---- configuration ------------------------------------------------------

    void readConfigurationLocked()
    {
        GiSettings settings;
        auto integer = [&](const char *key, std::int64_t fallback, std::int64_t minimum, std::int64_t maximum)
        {
            const auto value = context_->configurationIntegerInSection("lumen_gi", key);
            if (!value.has_value() || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto floating = [&](const char *key, double fallback, double minimum, double maximum)
        {
            const auto value = context_->configurationFloatingInSection("lumen_gi", key);
            if (!value.has_value() || !std::isfinite(*value) || *value < minimum || *value > maximum)
            {
                return fallback;
            }
            return *value;
        };
        auto boolean = [&](const char *key, bool fallback)
        {
            const auto value = context_->configurationBooleanInSection("lumen_gi", key);
            return value.has_value() ? *value : fallback;
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
        settings.captureTexelsPerFrame    = static_cast<std::uint64_t>(
            integer("capture.texels_per_frame", 262144, 4096, 16777216));
        settings.directTexelsPerFrame     = static_cast<std::uint64_t>(
            integer("lighting.direct_texels_per_frame", 65536, 1024, 16777216));
        settings.indirectTexelsPerFrame   = static_cast<std::uint64_t>(
            integer("lighting.indirect_texels_per_frame", 65536, 1024, 16777216));
        settings.indirectRaysPerTexel     = static_cast<std::uint32_t>(
            integer("lighting.indirect_rays_per_texel", 4, 1, 64));
        settings.sunShadows               = boolean("lighting.sun_shadows", true);
        settings.pointLightShadows        = boolean("lighting.point_light_shadows", false);
        settings.lightingUpdateSpeed      = static_cast<float>(floating("lighting.update_speed", 1.0, 0.05, 16.0));
        settings.indirectTemporalBlend =
            static_cast<float>(floating("lighting.indirect_temporal_blend", 0.8, 0.0, 0.99));
        settings.maxTraceDistance         = static_cast<float>(floating("trace.max_distance_m", 50.0, 1.0, 2000.0));
        settings.probeSpacing             = static_cast<float>(floating("probes.spacing_m", 2.0, 0.25, 32.0));
        settings.probeDirectionResolution = static_cast<std::uint32_t>(
            integer("probes.direction_resolution", 32, 4, 64));
        settings.probeUpdatesPerFrame     = static_cast<std::uint32_t>(
            integer("probes.updates_per_frame", 64, 1, 1458));
        settings.probeRaysPerUpdate       = static_cast<std::uint32_t>(
            integer("probes.rays_per_update", 256, 1, 4096));
        settings.probeHysteresis          = static_cast<float>(floating("probes.hysteresis", 0.2, 0.0, 0.99));
        settings.probeLevel1SpacingFactor =
            static_cast<float>(floating("probes.level1_spacing_factor", 4.0, 2.0, 16.0));
        settings.screenRayDistance =
            static_cast<float>(floating("gather.screen_ray_distance_m", 2.0, 0.05, 64.0));
        settings.gatherScreenScale        = static_cast<std::uint32_t>(integer("gather.screen_scale", 8, 1, 64));
        settings.gatherRaysPerProbe       = static_cast<std::uint32_t>(integer("gather.rays_per_probe", 16, 1, 256));
        settings.gatherImportanceSampling = boolean("gather.importance_sampling", true);
        settings.gatherTemporalBlend      = static_cast<float>(floating("gather.temporal_blend", 0.85, 0.0, 0.99));
        settings.gatherSpatialFilter      = static_cast<std::uint32_t>(integer("gather.spatial_filter", 3, 0, 9));
        settings.gatherPlaneDepthTolerance = static_cast<float>(
            floating("gather.plane_depth_tolerance", 0.35, 0.01, 4.0));
        settings.debugInternalWidth       = static_cast<std::uint32_t>(integer("debug.internal_width", 320, 16, 1920));
        settings.debugInternalHeight      = static_cast<std::uint32_t>(integer("debug.internal_height", 180, 16, 1080));
        settings.referenceSamples         = static_cast<std::uint32_t>(integer("debug.reference_samples", 64, 1, 4096));
        settings.referenceBounces         = static_cast<std::uint32_t>(integer("debug.reference_bounces", 4, 1, 16));
        settings.debugIncludeAmbient      = boolean("debug.include_ambient", false);
        settings.continuousGather         = boolean("debug.continuous_gather", false);
        settings.exportGatherFrames       = static_cast<std::uint32_t>(integer("debug.export_gather_frames", 16, 1, 64));
        settings_                         = settings;
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        bool relevant = false;
        for (const std::string &key : event.changedKeys)
        {
            if (key.rfind("lumen_gi.", 0) == 0)
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
        applySettingsToState(settings_, state_);
        state_.registryCapacity = settings_.maxSurfaces;
        const std::uint32_t resolution = std::max(settings_.probeDirectionResolution, 4u);
        const bool probesChanged = state_.probeDirections.empty() || resolution != builtDirectionResolution_
            || settings_.probeSpacing != builtProbeSpacing_ || settings_.probeLevel1SpacingFactor != builtLevel1Factor_
            || state_.level0.probes.empty();
        if (probesChanged)
        {
            buildProbeDirectionTable(state_);
            buildProbeLevels(state_);
            builtDirectionResolution_ = resolution;
            builtProbeSpacing_        = settings_.probeSpacing;
            builtLevel1Factor_        = settings_.probeLevel1SpacingFactor;
            if (state_.hasView)
            {
                snapProbeLevels(state_, state_.cameraPosition);
            }
        }
        const CardKey cardKey{settings_.texelsPerMeter, settings_.maxTexelsPerCardAxis, settings_.maxCardsPerSurface,
            settings_.cardDepthLayers};
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
        publishSnapshotLocked();
    }

    // ---- surface registry -----------------------------------------------------

    std::uint32_t findSurfaceLocked(GiSurfaceHandle handle) const
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
        surface.flagBits = description.flagBits;
        surface.excluded = (description.flagBits & giSurfaceFlagExcluded) != 0u;
        surface.unlit    = (description.flagBits & giSurfaceFlagUnlit) != 0u;
        surface.dynamic  = (description.flagBits & giSurfaceFlagDynamic) != 0u;
        surface.cards.clear();
        updateSurfaceTransform(surface, makeTransform(description.transform));
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
        if (state_.nextHandle == invalidGiSurface || state_.nextHandle == 0xffffffffu)
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
        state_.surfaces.push_back(std::move(surface));
        registryChangedLocked();
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
        applySurfaceDescriptionLocked(replacement, *description, &localError);
        if (localError != GiError::None)
        {
            setError(error, localError);
            return false;
        }
        state_.surfaces[index] = std::move(replacement);
        registryChangedLocked();
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
        const glm::mat4 transform = makeTransform(transformColumnMajor16);
        if (transform == surface.transform)
        {
            return true; // unchanged: keep the frame clean for the static skip
        }
        updateSurfaceTransform(surface, transform);
        state_.tlasDirty  = true;
        state_.sceneDirty = true;
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
        state_.surfaces.erase(state_.surfaces.begin() + index);
        registryChangedLocked();
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
            snapshot.hasDirectLights = !lights.empty();
            snapshot.directLights    = std::move(lights);
        }
        // Precompute the per-light constants the shading loop needs and drop
        // inactive lights; the per-texel evaluation then skips them without
        // re-normalizing directions or re-evaluating cone cosines.
        snapshot.preparedLights.clear();
        snapshot.preparedLights.reserve(snapshot.directLights.size());
        for (const direct_lighting::DirectLight &light : snapshot.directLights)
        {
            const direct_lighting::DirectLightDescription &description = light.description;
            const glm::vec3 color(description.color[0], description.color[1], description.color[2]);
            if (description.intensity <= 0.0f || (color.x <= 0.0f && color.y <= 0.0f && color.z <= 0.0f))
            {
                continue;
            }
            GiEnvironmentSnapshot::PreparedDirectLight prepared;
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
        if (settings_.continuousGather)
        {
            try
            {
                prepareDebugBuffers(state_, scheduler, 1);
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
            prepareDebugBuffers(state_, scheduler, passes);
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
        bool operator==(const CardKey &other) const
        {
            return texelsPerMeter == other.texelsPerMeter && maxAxis == other.maxAxis && maxCards == other.maxCards
                && layers == other.layers;
        }
    };

    ParticipantManifest        manifest_;
    LumenGiCapability          capability_{};
    ParticipantContext        *context_                  = nullptr;
    pbr::PbrCapability        *pbrCapability_            = nullptr;
    direct_lighting::DirectLightingCapability *directLightingCapability_ = nullptr;
    SubscriptionToken reloadSubscription_ = 0;

    mutable std::mutex moduleMutex_;
    GiSettings         settings_;
    GiPipelineState    state_;
    std::uint32_t      builtDirectionResolution_ = 0;
    float              builtProbeSpacing_        = 0.0f;
    float              builtLevel1Factor_        = 0.0f;
    CardKey            builtCardKey_{0.0f, 0, 0, 0};
    double             lastEvaluateMilliseconds_ = 0.0;
    std::atomic<std::uint64_t> probeVersion_{0};
    mutable std::mutex                     snapshotMutex_;
    std::shared_ptr<GiProbeSnapshot> snapshot_;
};

} // namespace
} // namespace lumen_gi
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "lumen_gi" capability; the pbr
// and direct_lighting environments are optional inputs, no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"lumen_gi"};
    static const char *const                    optional[] = {"pbr", "direct_lighting"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "lumen_gi",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::lumen_gi::LumenGiModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = optional,
            .optionalCapabilityCount = 2,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
