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
    componentManifest.requiredPermissions  = {"capability.logging", "capability.window", "capability.input"};
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
    if (state != nullptr && state->device != nullptr)
    {
        state->device->waitIdle();
    }
    state.reset();
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

} // namespace engine
