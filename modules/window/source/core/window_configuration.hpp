// [window] configuration parsing for the window module: the parsed
// configuration struct and the readers that turn a ParticipantContext's
// [window] section into it. Kept separate from the module class so the
// validation rules stay testable on their own.

#ifndef ENGINE_WINDOW_MODULE_CONFIGURATION_HPP
#define ENGINE_WINDOW_MODULE_CONFIGURATION_HPP

#include <engine/kernel/participant_interface.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace engine
{
namespace window
{

// Built-in fallbacks used when [window] configuration does not say otherwise.
inline constexpr std::uint32_t defaultWindowWidth         = 1280;
inline constexpr std::uint32_t defaultWindowHeight        = 720;
inline constexpr std::uint32_t defaultEventQueueCapacity  = 4096;
inline constexpr const char   *defaultWindowTitle         = "engine";
inline constexpr bool         defaultGamepadPolling      = true;
inline constexpr float        defaultGamepadAxisEpsilon  = 0.0005f;

// Parsed [window] section: window-creation defaults plus runtime knobs.
// The optional<> booleans distinguish "unset" from false so window creation
// only fills fields the caller genuinely left unset. Creation defaults apply
// to windows created afterwards only; the gamepad knobs apply immediately,
// including on reload.
struct WindowModuleConfiguration
{
    // "", "glfw", or "headless"; empty means "auto" (GLFW preferred).
    std::string backend;

    std::uint32_t width  = defaultWindowWidth;
    std::uint32_t height = defaultWindowHeight;

    std::uint32_t minimumWidth  = 0;
    std::uint32_t minimumHeight = 0;
    std::uint32_t maximumWidth  = 0;
    std::uint32_t maximumHeight = 0;

    std::uint32_t eventQueueCapacity = defaultEventQueueCapacity;

    std::int32_t positionX = 0;
    std::int32_t positionY = 0;
    bool         hasPosition = false;

    std::optional<bool> resizable;
    std::optional<bool> visible;
    std::optional<bool> decorated;
    std::optional<bool> floating;

    bool hasMaximized = false;
    bool maximized    = false;

    bool hasFocusOnShow = false;
    bool focusOnShow    = true;

    bool  hasOpacity = false;
    float opacity    = 1.0f;

    std::string title = defaultWindowTitle;

    // Runtime knobs: gamepad polling can be switched off entirely (the
    // per-frame joystick scan is skipped), and the axis noise threshold
    // tuned (0 keeps every axis change).
    bool  gamepadPolling     = defaultGamepadPolling;
    float gamepadAxisEpsilon = defaultGamepadAxisEpsilon;
};

// Reads the complete [window] section. Invalid values are reported on
// stderr and ignored, keeping the built-in default shown in the struct.
WindowModuleConfiguration readWindowModuleConfiguration(const ParticipantContext &context);

// True when a ConfigurationReloadedEvent changed any key starting with
// "window." (the module re-reads its section only then).
bool touchesWindowConfiguration(const ConfigurationReloadedEvent &event);

} // namespace window
} // namespace engine

#endif
