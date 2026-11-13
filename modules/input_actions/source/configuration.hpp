// Cap values parsed from ParticipantContext in construct().
//
// Every field has a built-in default identical to the value in config.toml so
// the module works with no configuration at all. The loader in
// loadInputActionsConfiguration applies each key only when present, finite,
// and inside the documented range; otherwise the default remains in effect.

#ifndef ENGINE_INPUT_ACTIONS_CONFIGURATION_HPP
#define ENGINE_INPUT_ACTIONS_CONFIGURATION_HPP

#include <engine/kernel/participant_interface.hpp>

#include <cstddef>
#include <limits>

namespace engine::input_actions
{
namespace internal
{

struct InputActionsConfiguration
{
    std::size_t maximumContexts          = 64;
    std::size_t maximumActionsPerContext = 1024;
    std::size_t maximumBindingsPerAction = 64;
    std::size_t maximumDevices           = 32;
    std::size_t maximumControlsPerDevice = 4096;
    float       defaultDeadzone          = 0.15f;
    float       maximumSmoothingSeconds  = 2.0f;
    float       maximumFrameDeltaSeconds = 0.25f;
};

// Reads every documented [input_actions] key through the ParticipantContext
// helpers. Each key applies only when present, has the expected TOML type,
// passes std::isfinite, and falls inside its documented range; otherwise the
// built-in default (identical to the config.toml value) remains in effect.
inline InputActionsConfiguration loadInputActionsConfiguration(ParticipantContext &context)
{
    InputActionsConfiguration config;
    if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_contexts");
        value && *value > 0 && *value <= 4096)
        config.maximumContexts = static_cast<std::size_t>(*value);
    if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_actions_per_context");
        value && *value > 0 && *value <= 65536)
        config.maximumActionsPerContext = static_cast<std::size_t>(*value);
    if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_bindings_per_action");
        value && *value > 0 && *value <= 4096)
        config.maximumBindingsPerAction = static_cast<std::size_t>(*value);
    if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_devices");
        value && *value > 0 && *value <= 1024)
        config.maximumDevices = static_cast<std::size_t>(*value);
    if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_controls_per_device");
        value && *value > 0 && *value <= 65536)
        config.maximumControlsPerDevice = static_cast<std::size_t>(*value);
    if (const auto value = context.configurationFloatingInSection("input_actions", "default_deadzone");
        value && std::isfinite(*value) && *value >= 0.0 && *value < 1.0)
        config.defaultDeadzone = static_cast<float>(*value);
    if (const auto value = context.configurationFloatingInSection("input_actions", "maximum_smoothing_seconds");
        value && std::isfinite(*value) && *value >= 0.0 && *value <= 10.0)
        config.maximumSmoothingSeconds = static_cast<float>(*value);
    if (const auto value = context.configurationFloatingInSection("input_actions", "maximum_frame_delta_seconds");
        value && std::isfinite(*value) && *value > 0.0 && *value <= 1.0)
        config.maximumFrameDeltaSeconds = static_cast<float>(*value);
    return config;
}

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_CONFIGURATION_HPP
