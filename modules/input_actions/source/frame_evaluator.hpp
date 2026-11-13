// Per-frame evaluation pipeline: combine binding values per action, apply
// shaping (deadzone, invert, scale, response curve) and smoothing, run the
// threshold test, and propagate per-frame control consumption.
//
// The evaluator owns the scratch containers used during advanceFrame and
// reads device state + context state through references supplied by the
// module. Not thread-safe; the module serializes calls.

#ifndef ENGINE_INPUT_ACTIONS_FRAME_EVALUATOR_HPP
#define ENGINE_INPUT_ACTIONS_FRAME_EVALUATOR_HPP

#include "configuration.hpp"
#include "context_registry.hpp"
#include "device_state.hpp"
#include "internal.hpp"

#include <engine/input_actions/capabilities.hpp>

#include <unordered_set>
#include <vector>

namespace engine::input_actions
{
namespace internal
{

class FrameEvaluator
{
public:
    FrameEvaluator(const InputActionsConfiguration &config, DeviceStateRegistry &devices, ContextRegistry &contexts)
        : config_(config), devices_(devices), contexts_(contexts)
    {
        // Pre-size the frame scratch containers from the configured caps so the
        // steady-state frame does not allocate.
        consumedControls_.reserve(
            std::min<std::size_t>(config.maximumActionsPerContext * config.maximumBindingsPerAction, 65536));
        pendingConsumption_.reserve(
            config.maximumActionsPerContext * std::min<std::size_t>(config.maximumBindingsPerAction, 8));
        activeControls_.reserve(config.maximumBindingsPerAction);
    }

    // Re-evaluates every action for the frame. Non-finite or non-positive
    // deltaSeconds skips time-based effects; the delta is clamped to
    // maximumFrameDeltaSeconds otherwise.
    void advanceFrame(float deltaSeconds);

private:
    // Reads one binding against the current device state, applies axis
    // deadzone (rescaled so [dz, 1] maps to [0, 1] sign-preserving), then
    // invert, scale, and the response curve |v|^exponent.
    float readBinding(const InputBinding &binding);

    const InputActionsConfiguration              &config_;
    DeviceStateRegistry                          &devices_;
    ContextRegistry                              &contexts_;
    std::unordered_set<ControlKey, ControlKeyHash> consumedControls_;
    std::vector<ControlKey>                       pendingConsumption_;
    std::vector<ControlKey>                       activeControls_;
};

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_FRAME_EVALUATOR_HPP
