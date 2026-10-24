#include "renderer.hpp"

#include "renderer_state.hpp"

#include "engine/kernel/event_bus.hpp"
#include "engine/kernel/filesystem_utilities.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

namespace engine
{

// RendererModule lifecycle ------------------------------------------------------------

RendererModule::RendererModule()
{
    componentManifest.name = "renderer";
    componentManifest.kind = ComponentKind::Module;
    // The implementation-bound member name: this module is the Vulkan
    // implementation of the "renderer" family. Consumers require the family;
    // a mod that needs this backend names the member exactly.
    componentManifest.providedCapabilities = {"renderer.vulkan"};
    // "input" is deliberately NOT in requiredCapabilities: the renderer
    // degrades to a static camera when the input module is absent (minimal
    // builds). The permission is still declared so an acquisition attempt
    // passes the permission gate when the capability exists.
    componentManifest.requiredCapabilities = {"logging", "window"};
    // "capability.scene" is declared WITHOUT a graph edge on "scene": the
    // renderer degrades to the static [scene] world when the scene module is
    // absent (minimal builds), exactly like the optional input acquisition
    // above. When the module IS present, the lazy acquisition at start
    // passes the permission gate because of this declaration.
    componentManifest.requiredPermissions  = {"capability.logging", "capability.window", "capability.input", "capability.scene"};
    // Every configuration key the renderer reads (its own section and the
    // [scene] world content it rasterizes) - the known-key registry behind
    // --validate's unknown-key warnings is built from these declarations.
    componentManifest.knownConfigKeys = {
        "renderer.enable_validation",
        "renderer.fail_on_validation_error",
        "renderer.vsync",
        "renderer.prefer_discrete_device",
        "renderer.max_frames_in_flight",
        "renderer.exposure",
        "renderer.tonemap",
        "renderer.bloom",
        "renderer.bloom_threshold",
        "renderer.bloom_strength",
        "renderer.shader_directory",
        "renderer.texture_directory",
        "scene.sky_radiance",
        "scene.sky.zenith",
        "scene.sky.horizon",
        "scene.sky.ground",
        "scene.sky.sun_disk",
        "scene.sky.sun_disk_size_degrees",
        "scene.camera.position",
        "scene.camera.look_at",
        "scene.camera.fov",
        "scene.camera.near",
        "scene.camera.far",
        "scene.camera.free_fly",
        "scene.camera.fly_speed",
        "scene.camera.fly_boost",
        "scene.camera.look_sensitivity",
        "scene.sun.direction",
        "scene.sun.color",
        "scene.sun.intensity",
        "scene.objects",
    };
}

RendererModule::~RendererModule()
{
    stop();
}

void RendererModule::construct(
    ComponentContext &context
)
{
    boundContext     = &context;
    logger           = &context.acquireCapability<LoggerCapability>();
    windowCapability = &context.acquireCapability<WindowCapability>();

    state = std::make_unique<RendererState>();

    // Mouse/keyboard camera control is optional: the fly controller keeps
    // the configured pose when the input module is not part of this build.
    try
    {
        state->inputCapability = &context.acquireCapability<InputCapability>();
    }
    catch (const std::exception &)
    {
        state->inputCapability = nullptr;
    }

    // Channel references are cached once: channels are heap anchored inside
    // the bus and stay valid for the engine lifetime, so the frame loop can
    // publish without map lookups and without touching the context again.
    frameBeginChannel           = &context.eventBus().channel<FrameBeginEvent>();
    frameRenderedChannel        = &context.eventBus().channel<FrameRenderedEvent>();
    state->frameCapturedChannel = &context.eventBus().channel<FrameCapturedEvent>();

    state->enableValidation      = context.configurationBoolean("enable_validation").value_or(true);
    state->failOnValidationError = context.configurationBoolean("fail_on_validation_error").value_or(false);
    state->vsync                 = context.configurationBoolean("vsync").value_or(true);
    state->preferDiscreteDevice  = context.configurationBoolean("prefer_discrete_device").value_or(true);
    const auto frames            = context.configurationInteger("max_frames_in_flight").value_or(defaultFramesInFlight);
    state->maxFramesInFlight = static_cast<std::uint32_t>(std::clamp<std::int64_t>(frames, 1, maxFramesInFlightLimit));
    state->exposure          = static_cast<float>(context.configurationFloating("exposure").value_or(1.0));
    state->tonemapMode       = context.configurationString("tonemap").value_or("aces");
    // Bloom (post-processing): opt-in, thresholded HDR bright pass + separable
    // blur, added back pre-tonemap (scene_composite_final.frag).
    state->bloomEnabled   = context.configurationBoolean("bloom").value_or(false);
    state->bloomThreshold = static_cast<float>(context.configurationFloating("bloom_threshold").value_or(1.0));
    state->bloomStrength  = static_cast<float>(context.configurationFloating("bloom_strength").value_or(0.15));
    if (state->tonemapMode != "aces" && state->tonemapMode != "reinhard" && state->tonemapMode != "none")
    {
        throw std::runtime_error(
            logPrefix + "unknown tonemap mode '" + state->tonemapMode + "' (known: aces, reinhard, none)"
        );
    }

    // Shared world content ([scene]): validated at construct so configuration
    // errors fail startup with a precise diagnostic instead of at first frame.
    std::string sceneError;
    if (!SceneDescription::parseFromConfiguration(context, state->scene, sceneError))
    {
        throw std::runtime_error(logPrefix + "" + sceneError);
    }
    // Lighting composition: the active lighting techniques attach themselves
    // during their own construct phases (demand-driven activation decides
    // which modules exist); the renderer only composes what attached.
    const std::string shaderDirectory = context.configurationString("shader_directory").value_or("shaders");
    state->shaderDirectory            = shaderDirectory;
    state->textureDirectory           = context.configurationString("texture_directory").value_or("");

    EventChannel<WindowResizedEvent> &resizedChannel = context.eventBus().channel<WindowResizedEvent>();
    resizeSubscription                               = EventSubscription(
        &context.eventBus(),
        typeid(WindowResizedEvent),
        resizedChannel.subscribe<RendererModule, &RendererModule::handleWindowResized>(this)
    );

    // Hot reload (optional module): shader change events re-create the
    // pipelines from the re-compiled SPIR-V. The subscription is free when
    // the module is absent (nobody publishes the event).
    EventChannel<HotReloadEvent> &hotReloadChannel = context.eventBus().channel<HotReloadEvent>();
    hotReloadSubscription                          = EventSubscription(
        &context.eventBus(),
        typeid(HotReloadEvent),
        hotReloadChannel.subscribe<RendererModule, &RendererModule::handleHotReload>(this)
    );
}

void RendererModule::start()
{
    state->resolvedShaderDirectory  = state->shaderDirectory.is_relative()
                                        ? (executableDirectory() / state->shaderDirectory)
                                        : state->shaderDirectory;
    state->resolvedTextureDirectory = state->textureDirectory.is_relative()
                                        ? (executableDirectory() / state->textureDirectory)
                                        : state->textureDirectory;
    state->initialize(*windowCapability, *logger);

    // Scene registry connection: after initialize (all GPU resources exist,
    // including the capacity-sized primitive buffers) but before any frame.
    // The static world is pushed into the registry here, so scripts see the
    // configured room through the same handles they spawn with.
    connectSceneRegistry();

    if (state->lightingTechniques.empty())
    {
        logger->logMessage(
            LogLevel::Information,
            logPrefix
                + "no lighting techniques attached; the scene renders unlit "
                  "(albedo + emissive + sky)"
        );
    }
    else
    {
        std::string attached;
        for (const auto &technique : state->lightingTechniques)
        {
            if (!attached.empty())
            {
                attached += ", ";
            }
            attached += technique->techniqueName();
        }
        logger->logMessage(
            LogLevel::Information,
            logPrefix + "lighting techniques = [" + attached + "] (" + std::to_string(state->lightingTechniques.size())
                + " attached)"
        );
    }
}

void RendererModule::stop()
{
    resizeSubscription.release();
    sceneAddedSubscription.release();
    sceneRemovedSubscription.release();
    sceneChangedSubscription.release();
    if (state != nullptr && state->device != nullptr)
    {
        state->device->waitIdle();
    }
    state.reset();
    boundContext         = nullptr;
    frameBeginChannel    = nullptr;
    frameRenderedChannel = nullptr;
    windowCapability     = nullptr;
    logger               = nullptr;
}

void *RendererModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "renderer.vulkan")
    {
        return static_cast<RendererCapability *>(this);
    }
    return nullptr;
}

void RendererModule::attachLightingTechnique(
    const std::shared_ptr<LightingTechniquePass> &pass
)
{
    if (state == nullptr || pass == nullptr)
    {
        return;
    }
    // One pass per algorithm name: a re-attach replaces the previous pass
    // (keeps hot-reload style experimentation sane).
    std::erase_if(
        state->lightingTechniques,
        [&pass](const std::shared_ptr<LightingTechniquePass> &existing)
        { return std::string_view(existing->techniqueName()) == pass->techniqueName(); }
    );
    state->lightingTechniques.push_back(pass);
    // The record order is a physical contract (the member comment): direct
    // emitters evaluate before the passes that transport their light. Enforce
    // it here instead of trusting the configuration's module order; the
    // stable sort keeps equal-priority passes in attach order.
    const auto targetRank = [](const std::shared_ptr<LightingTechniquePass> &technique)
    {
        if (hasLightingTarget(technique->targets(), LightingTarget::Direct))
        {
            return 0;
        }
        if (hasLightingTarget(technique->targets(), LightingTarget::IndirectDiffuse))
        {
            return 1;
        }
        return 2;
    };
    std::stable_sort(
        state->lightingTechniques.begin(),
        state->lightingTechniques.end(),
        [&targetRank](
            const std::shared_ptr<LightingTechniquePass> &left, const std::shared_ptr<LightingTechniquePass> &right
        ) { return targetRank(left) < targetRank(right); }
    );
    // Recompute which radiance targets exist: the composite reads a target
    // only when at least one attached technique declares it.
    state->hasDirectRadianceTarget   = false;
    state->hasIndirectDiffuseTarget  = false;
    state->hasIndirectSpecularTarget = false;
    for (const auto &technique : state->lightingTechniques)
    {
        state->hasDirectRadianceTarget |= hasLightingTarget(technique->targets(), LightingTarget::Direct);
        state->hasIndirectDiffuseTarget |= hasLightingTarget(technique->targets(), LightingTarget::IndirectDiffuse);
        state->hasIndirectSpecularTarget |= hasLightingTarget(technique->targets(), LightingTarget::IndirectSpecular);
    }
}

void RendererModule::handleWindowResized(
    const WindowResizedEvent & /*event*/
)
{
    if (state != nullptr)
    {
        state->framebufferResized = true;
    }
}

void RendererModule::connectSceneRegistry()
{
    if (boundContext == nullptr || state == nullptr)
    {
        return;
    }
    // Tri-state resolution at START time: every component has constructed by
    // now, so an available scene module registered its capability. Absent
    // module (or denied permission) means the world stays the static [scene]
    // array - a precise, logged decision, not a silent mode change.
    if (sceneResolutionState == 0)
    {
        try
        {
            state->sceneCapability = &boundContext->acquireCapability<SceneCapability>();
            sceneResolutionState   = 1;
        }
        catch (const std::exception &error)
        {
            state->sceneCapability = nullptr;
            sceneResolutionState   = -1;
            logger->logMessage(
                LogLevel::Information,
                logPrefix + std::string("scene capability not available; the world stays the static [scene] ")
                    + "configuration (" + error.what() + ")"
            );
        }
    }
    if (sceneResolutionState != 1 || state->sceneCapability == nullptr)
    {
        return;
    }

    // Subscribe BEFORE pushing the world: any mutation from here on - by
    // scripts whose top level runs during another module's start, or by
    // sibling modules - marks the data dirty for the first frame.
    EventBus                             &bus        = boundContext->eventBus();
    EventChannel<SceneObjectAddedEvent>    &addedChannel    = bus.channel<SceneObjectAddedEvent>();
    EventChannel<SceneObjectRemovedEvent>  &removedChannel  = bus.channel<SceneObjectRemovedEvent>();
    EventChannel<SceneObjectChangedEvent>  &changedChannel  = bus.channel<SceneObjectChangedEvent>();
    sceneAddedSubscription = EventSubscription(
        &bus, typeid(SceneObjectAddedEvent), addedChannel.subscribe<RendererModule, &RendererModule::handleSceneAdded>(this)
    );
    sceneRemovedSubscription = EventSubscription(
        &bus,
        typeid(SceneObjectRemovedEvent),
        removedChannel.subscribe<RendererModule, &RendererModule::handleSceneRemoved>(this)
    );
    sceneChangedSubscription = EventSubscription(
        &bus, typeid(SceneObjectChangedEvent), changedChannel.subscribe<RendererModule, &RendererModule::handleSceneChanged>(this)
    );

    // Push the static world into the registry: one handle per configured
    // object, in scene order. From here the registry is the single source of
    // truth for OBJECTS; the configured camera, sun, sky and texture sets
    // stay the renderer's own.
    state->registryHandles.clear();
    state->registryHandles.reserve(state->scene.objects.size());
    for (const SceneObject &object : state->scene.objects)
    {
        const SceneCapability::ObjectHandle handle =
            state->sceneCapability->addObject(RendererState::sceneObjectToDescriptor(object));
        state->registryHandles.push_back(handle);
    }

    // Initial sync: adopt the registry (which now holds the static world plus
    // anything registered before the renderer started) and upload once. The
    // first frame then renders exactly what the registry holds.
    state->sceneRegistryDirty = false;
    state->refreshSceneRegistry();
    logger->logMessage(
        LogLevel::Information,
        logPrefix + "scene registry connected: " + std::to_string(state->activeObjects.size())
            + " object(s) live, capacity " + std::to_string(SceneCapability::maxObjects)
    );
}

void RendererModule::handleSceneAdded(
    const SceneObjectAddedEvent & /*event*/
)
{
    if (state != nullptr)
    {
        state->sceneRegistryDirty = true;
    }
}

void RendererModule::handleSceneRemoved(
    const SceneObjectRemovedEvent & /*event*/
)
{
    if (state != nullptr)
    {
        state->sceneRegistryDirty = true;
    }
}

void RendererModule::handleSceneChanged(
    const SceneObjectChangedEvent & /*event*/
)
{
    if (state != nullptr)
    {
        state->sceneRegistryDirty = true;
    }
}

void RendererModule::handleHotReload(
    const HotReloadEvent &event
)
{
    if (state == nullptr)
    {
        return;
    }
    if (event.kind == hot_reload_kind::shader)
    {
        // Any .spv change re-creates every renderer pipeline: the modules
        // are loaded from disk at creation, and a partial reload (one
        // pipeline, the changed one) would need per-pipeline change
        // tracking for no dev-loop gain.
        shadersNeedReload = true;
    }
}

void RendererModule::renderFrame()
{
    if (state == nullptr || state->swapchain == nullptr)
    {
        throw std::runtime_error(logPrefix + "renderFrame called before start");
    }
    if (frameBeginChannel == nullptr || frameRenderedChannel == nullptr)
    {
        throw std::runtime_error(logPrefix + "renderFrame called before construct");
    }

    FrameBeginEvent begin;
    begin.frameIndex = state->frameCounter;
    frameBeginChannel->publish(begin);

    if (state->framebufferResized)
    {
        state->recreateSwapchain(*windowCapability, *logger);
    }

    // Hot reloaded shaders: re-create the pipelines from the re-compiled
    // SPIR-V. The device is idle here (frame serialization, problem 38), and
    // createPipelines re-reads the shader files from disk and rebuilds every
    // descriptor set that binds frame images.
    if (shadersNeedReload && state->device != nullptr)
    {
        shadersNeedReload = false;
        state->device->waitIdle();
        state->createPipelines();
        logger->logMessage(LogLevel::Information, logPrefix + "shaders reloaded: pipelines re-created");
    }

    state->renderFrame(*logger);

    // Optional loud failure for development runs: abort at the first observed
    // validation error instead of continuing through a storm of duplicate
    // reports. Off by default; the headless smoke runs surface the same
    // condition through their exit code without it.
    if (state->failOnValidationError && state->validationErrorSeen.load())
    {
        throw std::runtime_error(
            logPrefix
            + "graphics backend reported a validation error "
              "(fail_on_validation_error is enabled)"
        );
    }

    FrameRenderedEvent rendered;
    rendered.frameIndex = state->frameCounter;
    frameRenderedChannel->publish(rendered);
}

void RendererModule::waitUntilIdle()
{
    if (state != nullptr && state->device != nullptr)
    {
        state->device->waitIdle();
    }
}

void RendererModule::requestFrameCapture()
{
    if (state != nullptr)
    {
        state->captureRequested = true;
    }
}

bool RendererModule::validationErrorsObserved() const
{
    return state != nullptr && state->validationErrorSeen.load();
}

std::array<float, 3> RendererModule::cameraPosition() const
{
    return state != nullptr ? state->flyCamera.position() : std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> RendererModule::cameraLookAt() const
{
    return state != nullptr ? state->flyCamera.lookTarget() : std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

void RendererModule::setCameraPosition(
    const std::array<float, 3> &position
)
{
    if (state != nullptr)
    {
        state->flyCamera.setPosition(position);
    }
}

void RendererModule::setCameraLookAt(
    const std::array<float, 3> &lookAt
)
{
    if (state != nullptr)
    {
        state->flyCamera.lookAtPoint(lookAt);
    }
}

} // namespace engine
