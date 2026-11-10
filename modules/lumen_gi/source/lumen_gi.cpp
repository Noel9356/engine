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
    state.settings.captureTexelsPerFrame  = std::max<std::uint64_t>(4096,
        static_cast<std::uint64_t>(static_cast<double>(settings.captureTexelsPerFrame) * scale));
    state.settings.directTexelsPerFrame   = std::max<std::uint64_t>(1024,
        static_cast<std::uint64_t>(static_cast<double>(settings.directTexelsPerFrame) * scale));
    state.settings.indirectTexelsPerFrame = std::max<std::uint64_t>(1024,
        static_cast<std::uint64_t>(static_cast<double>(settings.indirectTexelsPerFrame) * scale));
    state.settings.probeUpdatesPerFrame =
        std::max(4u, static_cast<std::uint32_t>(static_cast<float>(settings.probeUpdatesPerFrame) * scale));
}

// One stage task executed on the scheduler; failures reject the graph.
// (Referenced by the scheduler-driven stage paths; kept [[maybe_unused]] so
// configurations that inline the stage work do not warn.)
[[maybe_unused]] void runStageChunk(const std::function<void()> &work)
{
    work();
}

// The module singleton backing the "lumen_gi" capability. moduleMutex_ guards
// the registry, settings, and the whole evaluate() call; statistics live in
// the pipeline state (atomic counters for the parallel stages).
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
        state_.cards.clear();
        state_.hasView          = false;
        state_.environmentValid = false;
        probeVersion_.fetch_add(1u, std::memory_order_release);
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
        settings.captureTexelsPerFrame    = static_cast<std::uint64_t>(
            integer("capture.texels_per_frame", 262144, 4096, 16777216));
        settings.directTexelsPerFrame     = static_cast<std::uint64_t>(
            integer("lighting.direct_texels_per_frame", 65536, 1024, 16777216));
        settings.indirectTexelsPerFrame   = static_cast<std::uint64_t>(
            integer("lighting.indirect_texels_per_frame", 65536, 1024, 16777216));
        settings.sunShadows               = boolean("lighting.sun_shadows", true);
        settings.pointLightShadows        = boolean("lighting.point_light_shadows", false);
        settings.lightingUpdateSpeed      = static_cast<float>(floating("lighting.update_speed", 1.0, 0.05, 16.0));
        settings.indirectTemporalBlend =
            static_cast<float>(floating("lighting.indirect_temporal_blend", 0.8, 0.0, 1.0));
        settings.maxTraceDistance         = static_cast<float>(floating("trace.max_distance_m", 50.0, 1.0, 2000.0));
        settings.probeSpacing             = static_cast<float>(floating("probes.spacing_m", 2.0, 0.25, 32.0));
        settings.probeDirectionResolution = static_cast<std::uint32_t>(
            integer("probes.direction_resolution", 32, 4, 64));
        settings.probeUpdatesPerFrame     = static_cast<std::uint32_t>(
            integer("probes.updates_per_frame", 64, 1, 1458));
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
    // debug resolution) require rebuilding the derived structures.
    void rebuildPipelineAfterConfigChangeLocked()
    {
        applySettingsToState(settings_, state_);
        state_.registryCapacity = settings_.maxSurfaces;
        if (state_.probeDirections.empty()
            || state_.settings.probeDirectionResolution != builtDirectionResolution_)
        {
            buildProbeDirectionTable(state_);
            builtDirectionResolution_ = state_.settings.probeDirectionResolution;
        }
        if (state_.level0.dimensions[0] == 0)
        {
            buildProbeLevels(state_);
            if (state_.hasView)
            {
                snapProbeLevels(state_, state_.cameraPosition);
            }
        }
        state_.screenProbeWidth  = 0;
        state_.screenProbeHeight = 0;
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
        surface.transform        = makeTransform(description.transform);
        surface.inverseTransform = glm::inverse(surface.transform);
        surface.normalMatrix     = glm::inverseTranspose(glm::mat3(surface.transform));
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
        updateSurfaceWorldBounds(surface);
    }

    void rebuildCardsForSurfaceLocked(std::uint32_t surfaceIndex)
    {
        // Rebuild one surface's card range: cards live in a flat vector; the
        // simple approach for registry mutations is a full re-place (scene
        // setup is rare; per-frame work never touches this path).
        state_.cards.clear();
        for (std::uint32_t index = 0; index < state_.surfaces.size(); ++index)
        {
            placeSurfaceCards(state_, state_.surfaces[index], index);
        }
        (void) surfaceIndex;
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
        const GiSurfaceHandle handle = state_.nextHandle++;
        GiError            localError = GiError::None;
        GiSurface          surface;
        surface.handle = handle;
        applySurfaceDescriptionLocked(surface, *description, &localError);
        if (localError != GiError::None)
        {
            setError(error, localError);
            return invalidGiSurface;
        }
        state_.surfaces.push_back(std::move(surface));
        rebuildCardsForSurfaceLocked(0);
        state_.sceneDirty = true;
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
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(description.albedoColor[axis]) || description.albedoColor[axis] < 0.0f
                || !std::isfinite(description.emissiveColor[axis]) || description.emissiveColor[axis] < 0.0f
                || !std::isfinite(description.emissiveIntensity) || description.emissiveIntensity < 0.0f)
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
        GiError    localError = GiError::None;
        applySurfaceDescriptionLocked(state_.surfaces[index], *description, &localError);
        if (localError != GiError::None)
        {
            setError(error, localError);
            return false;
        }
        rebuildCardsForSurfaceLocked(index);
        state_.sceneDirty = true;
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
        GiSurface &surface = state_.surfaces[index];
        for (std::uint32_t element = 0; element < 16; ++element)
        {
            if (!std::isfinite(transformColumnMajor16[element]))
            {
                return false;
            }
        }
        surface.transform        = makeTransform(transformColumnMajor16);
        surface.inverseTransform = glm::inverse(surface.transform);
        surface.normalMatrix     = glm::inverseTranspose(glm::mat3(surface.transform));
        updateSurfaceWorldBounds(surface);
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
        rebuildCardsForSurfaceLocked(0);
        state_.sceneDirty = true;
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
        // Static-frame skip change detection: an identical repeated setView
        // (same matrices, same eye) leaves the frame clean. Column-major
        // float compare; -0.0 vs 0.0 counts as changed (harmless: one extra
        // evaluate).
        bool viewChanged = true;
        if (state_.hasView)
        {
            viewChanged = false;
            for (std::uint32_t column = 0; column < 4; ++column)
            {
                for (std::uint32_t row = 0; row < 4; ++row)
                {
                    if (state_.viewProjection[column][row]
                        != viewProjectionColumnMajor16[column * 4 + row])
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
    // compare against the stored environmentVersion/directLightsVersion),
    // which is the environment-change signal for the static-frame skip.
    bool snapshotEnvironmentLocked()
    {
        GiEnvironmentSnapshot snapshot;
        std::uint64_t         pbrVersion    = 0;
        std::uint64_t         lightsVersion = 0;
        const glm::vec3       defaultSunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
        snapshot.sunDirection = defaultSunDirection;
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
                snapshot.sunDirection     = glm::normalize(glm::vec3(
                    environment.sun.direction[0], environment.sun.direction[1], environment.sun.direction[2]));
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
            directLightingCapability_->copyLights(directLightingCapability_->self, nullptr, 0, &lightsVersion);
            std::vector<direct_lighting::DirectLight> lights(
                std::min<std::uint64_t>(lightsVersion == 0 ? 0 : 64, direct_lighting::maxDirectLights));
            // maxDirectLights caps the snapshot; the registry may hold more.
            lights.assign(direct_lighting::maxDirectLights, direct_lighting::DirectLight{});
            const std::uint32_t count = directLightingCapability_->copyLights(directLightingCapability_->self,
                lights.data(), static_cast<std::uint32_t>(lights.size()), &lightsVersion);
            lights.resize(count);
            snapshot.hasDirectLights = count > 0;
            snapshot.directLights    = std::move(lights);
        }
        const bool environmentChanged = !state_.environmentValid
            || pbrVersion != state_.environmentVersion || lightsVersion != state_.directLightsVersion;
        state_.environmentVersion    = pbrVersion;
        state_.directLightsVersion   = lightsVersion;
        state_.environment = std::move(snapshot);
        state_.environmentValid = true;
        return environmentChanged;
    }

    // ---- evaluate: the Lumen-style frame -----------------------------------------

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
        state_.probeTracesThisFrame.store(0, std::memory_order_relaxed);
        state_.screenProbesThisFrame.store(0, std::memory_order_relaxed);
        state_.gatherRaysThisFrame.store(0, std::memory_order_relaxed);

        // Static-frame skip: when the camera, the scene, and the environment
        // are all unchanged and the caches have had a full probe-update cycle
        // to converge, re-running the pipeline would only re-blend the same
        // radiance with fresh random rays. Return early instead - and do not
        // bump the probe version, so a renderer polling copyProbes also skips
        // its per-frame snapshot upload.
        const bool viewChanged  = state_.viewDirty;
        const bool sceneChanged = state_.sceneDirty;
        state_.viewDirty  = false;
        state_.sceneDirty = false;
        if (viewChanged || sceneChanged || environmentChanged)
        {
            state_.framesSinceDirty = 0;
        }
        const std::uint32_t probeBudget = std::max(1u, state_.settings.probeUpdatesPerFrame);
        const std::size_t   largestLevel = std::max(state_.level0.probes.size(), state_.level1.probes.size());
        const std::uint32_t staticSkipAfterFrames = static_cast<std::uint32_t>(
            (largestLevel + probeBudget - 1) / probeBudget) + 2u;
        if (settings_.skipUnchangedFrames && state_.hasPreviousView && !viewChanged && !sceneChanged
            && !environmentChanged && state_.framesSinceDirty >= staticSkipAfterFrames)
        {
            state_.framesSkippedStatic.fetch_add(1, std::memory_order_relaxed);
            lastEvaluateMilliseconds_ = 0.0;
            setError(error, GiError::None);
            return true;
        }
        ++state_.framesSinceDirty;

        // Update camera-distance priorities for capture amortization.
        for (GiSurface &surface : state_.surfaces)
        {
            const glm::vec3 center = 0.5f * (surface.worldMin + surface.worldMax);
            for (std::uint32_t cardIndex = surface.firstCard; cardIndex < surface.firstCard + surface.cardCount;
                ++cardIndex)
            {
                state_.cards[cardIndex].cameraDistance = glm::distance(center, state_.cameraPosition);
            }
        }
        snapProbeLevels(state_, state_.cameraPosition);

        // Build the per-stage work: capture, direct, and indirect budgets are
        // filled by camera distance and staleness (Lumen's priority scheme).
        std::vector<GiTexelWork> captureWork;
        std::vector<GiTexelWork> directWork;
        std::vector<GiTexelWork> indirectWork;
        buildTexelWork(captureWork, directWork, indirectWork);

        TaskScheduler &scheduler = context_->taskScheduler();
        TaskGraph      graph;
        constexpr std::uint32_t workChunk = 8; // work items per task
        auto addChunks = [&](std::vector<GiTexelWork> &work, std::vector<std::uint32_t> &nodeIds,
                             void (*chunkFunction)(GiPipelineState &, const GiTexelWork *, std::uint32_t))
        {
            for (std::size_t begin = 0; begin < work.size(); begin += workChunk)
            {
                const std::size_t count = std::min(work.size() - begin, static_cast<std::size_t>(workChunk));
                const std::uint32_t node = graph.addTask([&state = state_, &work, begin, count, chunkFunction]()
                    { chunkFunction(state, work.data() + begin, static_cast<std::uint32_t>(count)); });
                nodeIds.push_back(node);
            }
        };
        std::vector<std::uint32_t> captureNodes;
        std::vector<std::uint32_t> directNodes;
        std::vector<std::uint32_t> indirectNodes;
        addChunks(captureWork, captureNodes, captureWorkChunk);
        addChunks(directWork, directNodes, directWorkChunk);
        addChunks(indirectWork, indirectNodes, indirectWorkChunk);

        // Probe updates: chunk the probe indices of both levels.
        std::vector<std::uint32_t> probeNodes;
        const std::uint32_t        probeUpdateBudget = state_.settings.probeUpdatesPerFrame;
        // Rolling update window: each frame refreshes the next
        // probeUpdatesPerFrame probes of every level (round-robin), so the
        // whole cache cycles like Lumen's amortized probe updates.
        auto addProbeChunks = [&](GiPipelineState::ProbeLevel &level, std::uint32_t levelIndex)
        {
            const std::uint32_t total   = static_cast<std::uint32_t>(level.probes.size());
            const std::uint32_t updates = std::min(probeUpdateBudget, total);
            constexpr std::uint32_t probeChunk = 12;
            std::uint32_t emitted = 0;
            while (emitted < updates)
            {
                const std::uint32_t begin = (level.updateCursor + emitted) % total;
                const std::uint32_t span  = std::min({updates - emitted, probeChunk, total - begin});
                const std::uint32_t node = graph.addTask([&state = state_, levelIndex, begin, span]()
                    { updateProbeChunk(state, levelIndex, begin, begin + span); });
                probeNodes.push_back(node);
                emitted += span;
            }
            level.updateCursor = (level.updateCursor + updates) % total;
        };
        addProbeChunks(state_.level0, 0);
        addProbeChunks(state_.level1, 1);

        // Screen gather + compose (debug view buffers), single nodes.
        const std::uint32_t rasterNode = graph.addTask(
            [&state = state_]()
            {
                rasterizeViewGBuffer(state);
            });
        const std::uint32_t screenProbeNode = graph.addTask(
            [&state = state_]()
            {
                traceScreenProbes(state);
            });
        const std::uint32_t resolveNode = graph.addTask(
            [&state = state_]()
            {
                resolvePixelIrradiance(state);
            });

        // Stage dependencies: capture -> direct -> indirect -> probes ->
        // raster -> screen probes -> resolve. Direct and indirect also depend
        // on capture; probes depend on indirect (fresh radiance).
        for (const std::uint32_t node : captureNodes)
        {
            for (const std::uint32_t consumer : directNodes)
            {
                graph.addDependency(node, consumer);
            }
        }
        for (const std::uint32_t node : directNodes)
        {
            for (const std::uint32_t consumer : indirectNodes)
            {
                graph.addDependency(node, consumer);
            }
        }
        for (const std::uint32_t node : indirectNodes)
        {
            for (const std::uint32_t consumer : probeNodes)
            {
                graph.addDependency(node, consumer);
            }
        }
        for (const std::uint32_t node : indirectNodes)
        {
            graph.addDependency(node, rasterNode);
        }
        graph.addDependency(rasterNode, screenProbeNode);
        graph.addDependency(screenProbeNode, resolveNode);

        try
        {
            graph.execute(scheduler);
        }
        catch (...)
        {
            setError(error, GiError::None);
            return false;
        }

        // Lifetime ray counter (GiStatistics::totalRaysTraced): this frame's
        // probe traces plus screen-space gather rays. The per-frame counters
        // reset at the start of the next evaluate.
        state_.totalRaysTraced.fetch_add(
            static_cast<std::uint64_t>(state_.probeTracesThisFrame.load(std::memory_order_relaxed))
                + static_cast<std::uint64_t>(state_.gatherRaysThisFrame.load(std::memory_order_relaxed)),
            std::memory_order_relaxed);

        // Roll the temporal state: previous indirect becomes current, the
        // screen gather buffers become the reprojection source.
        for (GiCard &card : state_.cards)
        {
            card.indirectPrevious.swap(card.indirectRadiance);
        }
        // Keep both buffer views consistent: gatherIrradiancePrevious was
        // swapped inside resolvePixelIrradiance.
        state_.previousViewProjection  = state_.viewProjection;
        state_.previousCameraPosition  = state_.cameraPosition;
        state_.previousViewWidth       = state_.viewWidth;
        state_.previousViewHeight      = state_.viewHeight;
        state_.previousViewDepth       = state_.viewDepth;
        state_.hasPreviousView         = true;
        probeVersion_.fetch_add(1u, std::memory_order_release);

        const auto endTime = std::chrono::steady_clock::now();
        lastEvaluateMilliseconds_ =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count())
            / 1000.0;
        setError(error, GiError::None);
        return true;
    }

    static void captureWorkChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count)
    {
        for (std::uint32_t item = 0; item < count; ++item)
        {
            GiCard &         card    = state.cards[work[item].cardIndex];
            const GiSurface &surface = state.surfaces[card.surfaceIndex];
            captureCard(state, card, surface, work[item].texelBegin, work[item].texelEnd);
            card.lastCaptureFrame = state.frameIndex;
        }
        std::uint32_t processed = 0;
        for (std::uint32_t item = 0; item < count; ++item)
        {
            processed += work[item].texelEnd - work[item].texelBegin;
        }
        state.captureTexelsThisFrame.fetch_add(processed, std::memory_order_relaxed);
    }

    static void directWorkChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count)
    {
        gatherDirectLightingChunk(state, work, count);
    }

    static void indirectWorkChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count)
    {
        gatherIndirectLightingChunk(state, work, count);
    }

    // Builds the amortized work lists. Priority follows Lumen's
    // LastUsed - LastUpdated scheme: the stalest cards come first, with
    // camera distance as the tiebreak, so every card cycles through each
    // budget instead of near cards starving far ones permanently.
    void buildTexelWork(std::vector<GiTexelWork> &captureWork, std::vector<GiTexelWork> &directWork,
        std::vector<GiTexelWork> &indirectWork)
    {
        captureWork.clear();
        directWork.clear();
        indirectWork.clear();
        std::vector<std::uint32_t> cardOrder(state_.cards.size());
        for (std::uint32_t index = 0; index < cardOrder.size(); ++index)
        {
            cardOrder[index] = index;
        }
        const std::uint64_t frame = state_.frameIndex;
        std::sort(cardOrder.begin(),
            cardOrder.end(),
            [this, frame](std::uint32_t left, std::uint32_t right)
            {
                const GiCard &leftCard  = state_.cards[left];
                const GiCard &rightCard = state_.cards[right];
                // Staleness: frames since the card was last touched, per use.
                const std::uint64_t leftCaptureAge  = frame - leftCard.lastCaptureFrame;
                const std::uint64_t rightCaptureAge = frame - rightCard.lastCaptureFrame;
                if (leftCaptureAge != rightCaptureAge)
                {
                    return leftCaptureAge > rightCaptureAge;
                }
                return leftCard.cameraDistance < rightCard.cameraDistance;
            });
        std::vector<std::uint32_t> lightingOrder = cardOrder;
        std::sort(lightingOrder.begin(),
            lightingOrder.end(),
            [this, frame](std::uint32_t left, std::uint32_t right)
            {
                const GiCard &leftCard  = state_.cards[left];
                const GiCard &rightCard = state_.cards[right];
                const std::uint64_t leftAge  = std::min(frame - leftCard.lastDirectFrame,
                    frame - leftCard.lastIndirectFrame);
                const std::uint64_t rightAge = std::min(frame - rightCard.lastDirectFrame,
                    frame - rightCard.lastIndirectFrame);
                if (leftAge != rightAge)
                {
                    return leftAge > rightAge;
                }
                return leftCard.cameraDistance < rightCard.cameraDistance;
            });
        // Capture: nearest, stalest cards first within the texel budget.
        std::uint64_t captureBudget = state_.settings.captureTexelsPerFrame;
        for (const std::uint32_t cardIndex : cardOrder)
        {
            if (captureBudget == 0)
            {
                break;
            }
            GiCard &card = state_.cards[cardIndex];
            const std::uint32_t texelCount = card.texelWidth * card.texelHeight;
            const std::uint32_t count      = static_cast<std::uint32_t>(
                std::min<std::uint64_t>(texelCount, captureBudget));
            captureWork.push_back(GiTexelWork{cardIndex, 0, count});
            captureBudget -= count;
        }
        // Direct + indirect: stalest-first ordering.
        std::uint64_t directBudget   = state_.settings.directTexelsPerFrame;
        std::uint64_t indirectBudget = state_.settings.indirectTexelsPerFrame;
        constexpr std::uint32_t texelStride = 4096; // work unit granularity
        for (const std::uint32_t cardIndex : lightingOrder)
        {
            GiCard &card = state_.cards[cardIndex];
            const std::uint32_t texelCount = card.texelWidth * card.texelHeight;
            for (std::uint32_t begin = 0; begin < texelCount; begin += texelStride)
            {
                const std::uint32_t end = std::min(begin + texelStride, texelCount);
                if (directBudget > 0)
                {
                    const std::uint32_t count = static_cast<std::uint32_t>(
                        std::min<std::uint64_t>(end - begin, directBudget));
                    directWork.push_back(GiTexelWork{cardIndex, begin, begin + count});
                    directBudget -= count;
                }
                if (indirectBudget > 0)
                {
                    const std::uint32_t count = static_cast<std::uint32_t>(
                        std::min<std::uint64_t>(end - begin, indirectBudget));
                    indirectWork.push_back(GiTexelWork{cardIndex, begin, begin + count});
                    indirectBudget -= count;
                }
                if (directBudget == 0 && indirectBudget == 0)
                {
                    return;
                }
            }
        }
    }

    // ---- read side ---------------------------------------------------------------

    std::uint32_t copyProbesImpl(GiProbe *output, std::uint32_t capacity, GiProbeHeader *outHeader,
        std::uint64_t *outVersion) const
    {
        std::lock_guard<std::mutex> lock(moduleMutex_);
        if (outVersion != nullptr)
        {
            *outVersion = probeVersion_.load(std::memory_order_acquire);
        }
        if (outHeader != nullptr)
        {
            outHeader->gridCenter[0] = state_.cameraPosition.x;
            outHeader->gridCenter[1] = state_.cameraPosition.y;
            outHeader->gridCenter[2] = state_.cameraPosition.z;
            outHeader->level0Spacing = state_.level0.spacing;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                outHeader->level0Dimensions[axis] = state_.level0.dimensions[axis];
                outHeader->level1Dimensions[axis] = state_.level1.dimensions[axis];
            }
            outHeader->level1Spacing = state_.level1.spacing;
            outHeader->directionResolution = state_.settings.probeDirectionResolution;
            outHeader->level0ProbeOffset  = 0;
            outHeader->level1ProbeOffset  = static_cast<std::uint32_t>(state_.level0.probes.size());
            outHeader->probeCount         = static_cast<std::uint32_t>(
                state_.level0.probes.size() + state_.level1.probes.size());
        }
        if (output == nullptr || capacity == 0)
        {
            return 0;
        }
        std::uint32_t copied = 0;
        auto copyLevel = [&](const GiPipelineState::ProbeLevel &level, std::uint32_t levelIndex)
        {
            for (const GiPipelineState::ProbeLevel::Probe &probe : level.probes)
            {
                if (copied >= capacity)
                {
                    return;
                }
                GiProbe &target = output[copied++];
                target.position[0] = probe.position.x;
                target.position[1] = probe.position.y;
                target.position[2] = probe.position.z;
                for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
                {
                    target.sh2[coefficient / 3][coefficient % 3] = probe.sh2[coefficient];
                }
                target.validity = probe.validity;
                target.level    = levelIndex;
            }
        };
        copyLevel(state_.level0, 0);
        copyLevel(state_.level1, 1);
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
        }
        for (const GiCard &card : state_.cards)
        {
            cardTexels += static_cast<std::uint64_t>(card.texelWidth) * card.texelHeight;
        }
        outStatistics->triangles                = triangles;
        outStatistics->cards                    = static_cast<std::uint32_t>(state_.cards.size());
        outStatistics->cardTexels               = cardTexels;
        outStatistics->captureTexelsThisFrame   = state_.captureTexelsThisFrame.load(std::memory_order_relaxed);
        outStatistics->directTexelsThisFrame    = state_.directTexelsThisFrame.load(std::memory_order_relaxed);
        outStatistics->indirectTexelsThisFrame  = state_.indirectTexelsThisFrame.load(std::memory_order_relaxed);
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
        // non-Vulkan 0.125, Vulkan discrete 1.0, integrated 0.5, otherwise
        // 0.25. CPU devices get the floor so headless use stays responsive.
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
        else if (device->deviceClass == renderer::DeviceClass::Cpu)
        {
            scale = 0.25f;
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
        const bool exported = exportDebugView(state_, view, filePath, error);
        return exported;
    }

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
    double             lastEvaluateMilliseconds_ = 0.0;
    std::atomic<std::uint64_t> probeVersion_{1};
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
