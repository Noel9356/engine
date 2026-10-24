#ifndef ENGINE_MODULES_ANIMATION_ANIMATION_CAPABILITY_HPP
#define ENGINE_MODULES_ANIMATION_ANIMATION_CAPABILITY_HPP

// The published capability surface of the animation module. Every consumer -
// other modules, native mods, the app - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic: the targets are
// scene object and light handles (primitive values), the parameterization is
// plain numbers.

#include "lighting_capability.hpp"
#include "scene_capability.hpp"

#include <array>
#include <cstdint>

namespace engine
{

// What a tween drives. The channel constants are part of the capability
// contract (scripts and mods name them as numbers through the bridge).
namespace animation_channel
{
inline constexpr std::uint32_t position        = 0; // object transform: fromPosition -> toPosition
inline constexpr std::uint32_t rotation        = 1; // object transform: fromValue -> toValue (radians, Y)
inline constexpr std::uint32_t emissive        = 2; // object material: fromValue -> toValue
inline constexpr std::uint32_t lightIntensity = 3; // point light: fromValue -> toValue
inline constexpr std::uint32_t lightRadius    = 4; // point light: fromValue -> toValue
} // namespace animation_channel

// Interpolation curves. The engine ships the classic quartet; more curves
// are additive constants in this namespace plus one case in the evaluator.
namespace animation_easing
{
inline constexpr std::uint32_t linear     = 0;
inline constexpr std::uint32_t ease_in    = 1; // t * t
inline constexpr std::uint32_t ease_out   = 2; // 1 - (1 - t) * (1 - t)
inline constexpr std::uint32_t ease_in_out = 3; // smoothstep
} // namespace animation_easing

// Loop behavior after one duration has elapsed.
namespace animation_mode
{
inline constexpr std::uint32_t once     = 0; // play, publish TweenFinishedEvent, drop
inline constexpr std::uint32_t loop     = 1; // wrap around, never finishes
inline constexpr std::uint32_t ping_pong = 2; // reverse at each end, never finishes
} // namespace animation_mode

// One tween's parameterization. Every field has a sane default, so a minimal
// description ({ fromPosition, toPosition }) is already a valid one-second
// linear move.
struct TweenDescription
{
    std::uint32_t       channel      = animation_channel::position;
    // Position tweens (channel position); the scalar channels use from/toValue.
    std::array<float, 3> fromPosition = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> toPosition   = {0.0f, 0.0f, 0.0f};
    float                fromValue    = 0.0f;
    float                toValue      = 1.0f;
    double               durationSeconds      = 1.0;
    double               startDelaySeconds    = 0.0;
    std::uint32_t        easing               = animation_easing::linear;
    std::uint32_t        mode                 = animation_mode::once;
};

// Published when a once-mode tween reached its end. Loop and ping-pong tweens
// never finish and never publish this event.
struct TweenFinishedEvent
{
    std::uint64_t tweenHandle = 0;
};

// Capability "animation": tween drivers over the scene registry and the
// light state. The module advances every active tween on the frame event and
// writes the new values through the consumed capabilities - a tween is a
// driver, not a parallel representation: the scene registry stays the one
// source of truth for objects, the lighting state for lights.
class AnimationCapability
{
public:
    static constexpr const char *capabilityName = "animation";

    // Tweens are addressed by handle; handles are never reused.
    using TweenHandle = std::uint64_t;

    static constexpr TweenHandle invalidTween = 0;

    virtual ~AnimationCapability() = default;

    // Starts one tween driving a scene object (channels position, rotation,
    // emissive). Returns invalidTween for a channel/target mismatch or an
    // unknown object - the provider logs the precise reason.
    virtual TweenHandle animateObject(
        SceneCapability::ObjectHandle objectHandle, const TweenDescription &tween
    ) = 0;

    // Starts one tween driving a point light (channels lightIntensity,
    // lightRadius). The light handle semantics are the lighting capability's:
    // a removed light invalidates the tween, which then drops itself.
    virtual TweenHandle animatePointLight(
        LightingCapability::PointLightHandle lightHandle, const TweenDescription &tween
    ) = 0;

    // Cancels one tween. Returns false for unknown handles (including
    // already-finished one-shots).
    virtual bool cancelTween(
        TweenHandle handle
    ) = 0;

    virtual std::uint32_t activeTweenCount() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_ANIMATION_ANIMATION_CAPABILITY_HPP
