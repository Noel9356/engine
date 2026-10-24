#ifndef ENGINE_MODULES_ANIMATION_HPP
#define ENGINE_MODULES_ANIMATION_HPP

#include "animation_capability.hpp"
#include "logging_capability.hpp"
#include "renderer_capability.hpp"
#include "scene_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace engine
{

// Module "animation": tween drivers over the scene registry and the light
// state.
//
// The module subscribes to the frame event, advances every active tween by
// the frame delta and writes the new values through the consumed
// capabilities (scene for objects, lighting for point lights). It owns no
// state of its own beyond the tween list: the scene registry stays the one
// source of truth for objects, and every write flows through the same
// permission-checked, event-publishing surface a script would use - which
// also means the renderer's incremental rebuild picks animated objects up
// with no animation-specific renderer path.
class AnimationModule final : public ModuleInterface, public AnimationCapability
{
public:
    AnimationModule();
    ~AnimationModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // AnimationCapability.
    TweenHandle      animateObject(SceneCapability::ObjectHandle objectHandle, const TweenDescription &tween)
        override;
    TweenHandle      animatePointLight(LightingCapability::PointLightHandle lightHandle, const TweenDescription &tween)
        override;
    bool             cancelTween(TweenHandle handle) override;
    std::uint32_t    activeTweenCount() const override;

private:
    struct ActiveTween
    {
        TweenHandle      handle        = invalidTween;
        std::uint32_t    channel       = animation_channel::position;
        // The target: a scene object handle or a point light handle (the
        // channel decides which capability the write goes to).
        std::uint32_t    targetHandle  = 0;
        std::array<float, 3> fromPosition = {0.0f, 0.0f, 0.0f};
        std::array<float, 3> toPosition   = {0.0f, 0.0f, 0.0f};
        float            fromValue     = 0.0f;
        float            toValue       = 1.0f;
        double           durationSeconds  = 1.0;
        double           startDelaySeconds = 0.0;
        std::uint32_t    easing        = animation_easing::linear;
        std::uint32_t    mode          = animation_mode::once;
        std::chrono::steady_clock::time_point started{};
    };

    void forwardFrameBegin(const FrameBeginEvent &event);
    void advanceTweens(
        double frameDeltaSeconds
    );
    // Writes one tween's current value to its target. Returns false when the
    // target vanished (object removed, light removed): the tween then drops.
    bool applyTween(
        const ActiveTween &tween, const std::array<float, 3> &position, float value
    );

    ComponentManifest   componentManifest;
    ComponentContext   *boundContext       = nullptr;
    LoggerCapability   *logger             = nullptr;
    SceneCapability    *sceneCapability    = nullptr;
    // Tri-state optional lighting resolution: light tweens degrade to a
    // one-time log when the pbr module is absent from the build.
    LightingCapability *lightingCapability = nullptr;

    EventSubscription   frameBeginSubscription;
    EventChannel<FrameBeginEvent> *frameBeginChannel = nullptr;
    EventChannel<TweenFinishedEvent> *tweenFinishedChannel = nullptr;

    std::vector<ActiveTween> tweens;
    TweenHandle              nextHandle   = invalidTween + 1;
    // Frame delta clock: the module ticks on FrameBeginEvent and measures the
    // real time between ticks (steady_clock; the engine is single threaded by
    // contract, so no clock skew is possible).
    std::chrono::steady_clock::time_point lastTick{};
    bool                                  clockSeeded = false;
};

} // namespace engine

#endif // ENGINE_MODULES_ANIMATION_HPP
