#include "animation.hpp"

#include <algorithm>
#include <cmath>

namespace engine
{

namespace
{

// The easing evaluator. Kept as one pure function: adding a curve is one case
// here plus one constant in animation_easing, and nothing else in the module
// knows the curve shapes exist.
float evaluateEasing(
    std::uint32_t easing, float t
)
{
    switch (easing)
    {
        case animation_easing::ease_in:
            return t * t;
        case animation_easing::ease_out:
            return 1.0f - (1.0f - t) * (1.0f - t);
        case animation_easing::ease_in_out:
            // Smoothstep: the classic C1-continuous in/out blend.
            return t * t * (3.0f - 2.0f * t);
        case animation_easing::linear:
        default:
            return t;
    }
}

// Maps the elapsed time into the curve parameter [0, 1] per the loop mode.
float curveParameter(
    const double elapsedSeconds, const double durationSeconds, const std::uint32_t mode
)
{
    if (durationSeconds <= 0.0)
    {
        return 1.0f;
    }
    const double phase = elapsedSeconds / durationSeconds;
    switch (mode)
    {
        case animation_mode::loop:
        {
            const double wrapped = phase - std::floor(phase);
            return static_cast<float>(wrapped);
        }
        case animation_mode::ping_pong:
        {
            // Triangle wave: 2 cycles per full period, mirrored back down.
            const double wrapped  = phase - std::floor(phase);
            const double triangle = wrapped <= 0.5 ? wrapped * 2.0 : 2.0 - wrapped * 2.0;
            return static_cast<float>(triangle);
        }
        case animation_mode::once:
        default:
            return static_cast<float>(std::clamp(phase, 0.0, 1.0));
    }
}

} // namespace

AnimationModule::AnimationModule()
{
    componentManifest.name = "animation";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"animation.tweens"};
    // The scene registry is a hard edge (object tweens write through it); the
    // lighting state is optional (light tweens degrade precisely when the pbr
    // module is absent).
    componentManifest.requiredCapabilities = {"logging", "scene"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.scene", "capability.lighting"};
}

AnimationModule::~AnimationModule()
{
    stop();
}

void AnimationModule::construct(
    ComponentContext &context
)
{
    boundContext    = &context;
    logger          = &context.acquireCapability<LoggerCapability>();
    sceneCapability = &context.acquireCapability<SceneCapability>();

    // Lighting is optional: the graph edge is deliberately absent, so a build
    // without the pbr module still animates objects. First use decides.
    EventBus                        &bus                    = context.eventBus();
    frameBeginChannel                                        = &bus.channel<FrameBeginEvent>();
    tweenFinishedChannel                                     = &bus.channel<TweenFinishedEvent>();
    frameBeginSubscription                                   = EventSubscription(
        &bus, typeid(FrameBeginEvent), frameBeginChannel->subscribe<AnimationModule, &AnimationModule::forwardFrameBegin>(this)
    );
}

void AnimationModule::start()
{
    logger->logMessage(LogLevel::Information, "animation module started");
}

void AnimationModule::stop()
{
    frameBeginSubscription.release();
    tweens.clear();
    boundContext       = nullptr;
    logger             = nullptr;
    sceneCapability    = nullptr;
    lightingCapability = nullptr;
}

void *AnimationModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "animation.tweens")
    {
        return static_cast<AnimationCapability *>(this);
    }
    return nullptr;
}

AnimationCapability::TweenHandle AnimationModule::animateObject(
    SceneCapability::ObjectHandle objectHandle, const TweenDescription &tween
)
{
    if (objectHandle == SceneCapability::invalidObject)
    {
        if (logger != nullptr)
        {
            logger->logMessage(LogLevel::Warning, "animation module: animateObject refused (invalid object handle)");
        }
        return invalidTween;
    }
    if (tween.channel != animation_channel::position && tween.channel != animation_channel::rotation
        && tween.channel != animation_channel::emissive)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "animation module: animateObject refused (channel " + std::to_string(tween.channel)
                    + " does not drive a scene object)"
            );
        }
        return invalidTween;
    }
    // The target must exist NOW: a tween on a typo'd handle is a programming
    // error the caller wants to hear about, not a tween that silently never
    // moves anything.
    SceneObjectDescriptor current;
    if (!sceneCapability->object(objectHandle, current))
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "animation module: animateObject refused (object " + std::to_string(objectHandle) + " is not live)"
            );
        }
        return invalidTween;
    }

    ActiveTween active;
    active.handle             = nextHandle++;
    active.channel            = tween.channel;
    active.targetHandle       = objectHandle;
    active.fromPosition       = tween.fromPosition;
    active.toPosition         = tween.toPosition;
    active.fromValue          = tween.fromValue;
    active.toValue            = tween.toValue;
    active.durationSeconds    = tween.durationSeconds;
    active.startDelaySeconds  = tween.startDelaySeconds;
    active.easing             = tween.easing;
    active.mode               = tween.mode;
    active.started            = std::chrono::steady_clock::now();
    tweens.push_back(active);
    return active.handle;
}

AnimationCapability::TweenHandle AnimationModule::animatePointLight(
    LightingCapability::PointLightHandle lightHandle, const TweenDescription &tween
)
{
    if (tween.channel != animation_channel::lightIntensity && tween.channel != animation_channel::lightRadius)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "animation module: animatePointLight refused (channel " + std::to_string(tween.channel)
                    + " does not drive a light)"
            );
        }
        return invalidTween;
    }
    // Lighting is resolved lazily at first USE (construct order does not
    // guarantee the pbr module registered before this module constructed;
    // every construct is done by start, and tweens are created from start on).
    if (lightingCapability == nullptr && boundContext != nullptr)
    {
        try
        {
            lightingCapability = &boundContext->acquireCapability<LightingCapability>();
        }
        catch (const std::exception &)
        {
            lightingCapability = nullptr;
        }
    }
    if (lightingCapability == nullptr)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "animation module: animatePointLight refused (the lighting capability is not part of this build)"
            );
        }
        return invalidTween;
    }
    if (lightHandle == LightingCapability::invalidPointLight || lightHandle > lightingCapability->pointLightCount())
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "animation module: animatePointLight refused (light " + std::to_string(lightHandle)
                    + " is not live; light handles are renumbered by removals)"
            );
        }
        return invalidTween;
    }

    ActiveTween active;
    active.handle            = nextHandle++;
    active.channel           = tween.channel;
    active.targetHandle      = lightHandle;
    active.fromValue         = tween.fromValue;
    active.toValue           = tween.toValue;
    active.durationSeconds   = tween.durationSeconds;
    active.startDelaySeconds = tween.startDelaySeconds;
    active.easing            = tween.easing;
    active.mode              = tween.mode;
    active.started           = std::chrono::steady_clock::now();
    tweens.push_back(active);
    return active.handle;
}

bool AnimationModule::cancelTween(
    TweenHandle handle
)
{
    const auto found = std::find_if(
        tweens.begin(), tweens.end(), [handle](const ActiveTween &tween) { return tween.handle == handle; }
    );
    if (found == tweens.end())
    {
        return false;
    }
    tweens.erase(found);
    return true;
}

std::uint32_t AnimationModule::activeTweenCount() const
{
    return static_cast<std::uint32_t>(tweens.size());
}

bool AnimationModule::applyTween(
    const ActiveTween &tween, const std::array<float, 3> &position, float value
)
{
    switch (tween.channel)
    {
        case animation_channel::position:
        {
            // Rotation is preserved: a position tween must not reorient.
            SceneObjectDescriptor current;
            if (!sceneCapability->object(tween.targetHandle, current))
            {
                return false;
            }
            return sceneCapability->setObjectTransform(tween.targetHandle, position, current.rotationYRadians);
        }
        case animation_channel::rotation:
        {
            SceneObjectDescriptor current;
            if (!sceneCapability->object(tween.targetHandle, current))
            {
                return false;
            }
            return sceneCapability->setObjectTransform(tween.targetHandle, current.position, value);
        }
        case animation_channel::emissive:
        {
            SceneObjectDescriptor current;
            if (!sceneCapability->object(tween.targetHandle, current))
            {
                return false;
            }
            current.emissiveStrength = value;
            return sceneCapability->setObjectMaterial(tween.targetHandle, current);
        }
        case animation_channel::lightIntensity:
        case animation_channel::lightRadius:
        {
            if (lightingCapability == nullptr)
            {
                return false;
            }
            // Light handles are index+1; read-modify-write keeps the other
            // fields exactly as they are.
            PointLightDescription light = lightingCapability->pointLight(tween.targetHandle - 1);
            if (tween.channel == animation_channel::lightIntensity)
            {
                light.intensity = value;
            }
            else
            {
                light.radius = value;
            }
            lightingCapability->updatePointLight(tween.targetHandle, light);
            return true;
        }
        default:
            return false;
    }
}

void AnimationModule::advanceTweens(
    double frameDeltaSeconds
)
{
    if (tweens.empty())
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    (void) frameDeltaSeconds;

    std::vector<ActiveTween> survivors;
    survivors.reserve(tweens.size());
    for (ActiveTween &tween : tweens)
    {
        const double elapsed = std::chrono::duration<double>(now - tween.started).count()
                             - tween.startDelaySeconds;
        if (elapsed < 0.0)
        {
            survivors.push_back(std::move(tween));
            continue;
        }
        const float raw     = curveParameter(elapsed, tween.durationSeconds, tween.mode);
        const float eased   = evaluateEasing(tween.easing, raw);
        float       value   = 0.0f;
        std::array<float, 3> position{};
        switch (tween.channel)
        {
            case animation_channel::position:
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    position[axis] = tween.fromPosition[axis]
                                   + (tween.toPosition[axis] - tween.fromPosition[axis]) * eased;
                }
                break;
            case animation_channel::rotation:
            case animation_channel::emissive:
            case animation_channel::lightIntensity:
            case animation_channel::lightRadius:
                value = tween.fromValue + (tween.toValue - tween.fromValue) * eased;
                break;
            default:
                break;
        }
        const bool alive = applyTween(tween, position, value);
        if (!alive)
        {
            // Target vanished (object removed, light renumbered): drop
            // silently - the removal already published its own event.
            continue;
        }
        if (tween.mode == animation_mode::once && raw >= 1.0f)
        {
            if (tweenFinishedChannel != nullptr)
            {
                tweenFinishedChannel->publish(TweenFinishedEvent{.tweenHandle = tween.handle});
            }
            continue;
        }
        survivors.push_back(std::move(tween));
    }
    tweens = std::move(survivors);
}

void AnimationModule::forwardFrameBegin(
    const FrameBeginEvent & /*event*/
)
{
    const auto now = std::chrono::steady_clock::now();
    if (!clockSeeded)
    {
        // First frame: no meaningful delta exists (the engine may have spent
        // arbitrary time starting up); seed the clock and animate from here.
        clockSeeded = true;
        lastTick    = now;
        return;
    }
    const double frameDelta = std::chrono::duration<double>(now - lastTick).count();
    lastTick                = now;
    advanceTweens(frameDelta);
}

} // namespace engine
