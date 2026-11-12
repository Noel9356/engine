#include "camera_bindings.hpp"

#include <array>
#include <cstdio>
#include <string>
#include <utility>

namespace demo
{
namespace window = engine::window;

// Bit position of a key in the 64-bit key-state mask returned by
// keyStateMask(); returns 0 for keys outside the representable range.
std::uint64_t keyMaskBit(window::Key key) noexcept
{
    const auto value = static_cast<std::uint32_t>(key);
    return value > 0 && value < 64 ? (std::uint64_t{1} << value) : 0;
}

using WindowKeyEntry = std::pair<const char *, window::Key>;

// Canonical lowercase names for window keys. The demo publishes these names in
// WindowKeyEvent, so script plugins see the same spelling that configuration
// parsing accepts.
constexpr WindowKeyEntry windowKeyNames[]
    = {{"unknown", window::Key::Unknown},
        {"escape", window::Key::Escape},
        {"space", window::Key::Space},
        {"w", window::Key::W},
        {"a", window::Key::A},
        {"s", window::Key::S},
        {"d", window::Key::D},
        {"left", window::Key::Left},
        {"right", window::Key::Right},
        {"up", window::Key::Up},
        {"down", window::Key::Down},
        {"left_shift", window::Key::LeftShift},
        {"left_control", window::Key::LeftControl},
        {"q", window::Key::Q},
        {"e", window::Key::E},
        {"r", window::Key::R},
        {"f", window::Key::F},
        {"z", window::Key::Z},
        {"x", window::Key::X},
        {"c", window::Key::C},
        {"v", window::Key::V},
        {"b", window::Key::B},
        {"n", window::Key::N},
        {"m", window::Key::M},
        {"t", window::Key::T},
        {"g", window::Key::G},
        {"y", window::Key::Y},
        {"u", window::Key::U},
        {"i", window::Key::I},
        {"o", window::Key::O},
        {"p", window::Key::P},
        {"j", window::Key::J},
        {"k", window::Key::K},
        {"l", window::Key::L},
        {"h", window::Key::H},
        {"num0", window::Key::Num0},
        {"num1", window::Key::Num1},
        {"num2", window::Key::Num2},
        {"num3", window::Key::Num3},
        {"num4", window::Key::Num4},
        {"num5", window::Key::Num5},
        {"num6", window::Key::Num6},
        {"num7", window::Key::Num7},
        {"num8", window::Key::Num8},
        {"num9", window::Key::Num9},
        {"tab", window::Key::Tab},
        {"enter", window::Key::Enter},
        {"backspace", window::Key::Backspace},
        {"left_alt", window::Key::LeftAlt},
        {"right_shift", window::Key::RightShift},
        {"right_control", window::Key::RightControl},
        {"right_alt", window::Key::RightAlt}};

// Reverse lookup used when publishing WindowKeyEvent; the names match the
// spelling parseWindowKey accepts from configuration.
const char *windowKeyName(window::Key key) noexcept
{
    for (const WindowKeyEntry &entry : windowKeyNames)
        if (entry.second == key)
            return entry.first;
    return "unknown";
}

// Parses a lowercase key name from configuration; returns false if unknown.
bool parseWindowKey(const std::string &name, window::Key &out) noexcept
{
    for (const WindowKeyEntry &entry : windowKeyNames)
        if (name == entry.first)
        {
            out = entry.second;
            return true;
        }
    return false;
}

// Loads the [camera] "*_key" overrides for every binding. Unknown key names
// are reported on stderr and keep the WASD-style default.
DemoCameraKeyBindings loadCameraKeyBindings(engine::ParticipantContext &context)
{
    DemoCameraKeyBindings                                                               result{};
    const std::array<std::pair<const char *, window::Key DemoCameraKeyBindings::*>, 13> fields
        = {{{"move_forward_key", &DemoCameraKeyBindings::moveForward},
            {"move_backward_key", &DemoCameraKeyBindings::moveBackward},
            {"move_left_key", &DemoCameraKeyBindings::moveLeft},
            {"move_right_key", &DemoCameraKeyBindings::moveRight},
            {"move_up_key", &DemoCameraKeyBindings::moveUp},
            {"move_down_key", &DemoCameraKeyBindings::moveDown},
            {"orbit_zoom_in_key", &DemoCameraKeyBindings::orbitZoomIn},
            {"orbit_zoom_out_key", &DemoCameraKeyBindings::orbitZoomOut},
            {"sprint_key", &DemoCameraKeyBindings::sprint},
            {"look_left_key", &DemoCameraKeyBindings::lookLeft},
            {"look_right_key", &DemoCameraKeyBindings::lookRight},
            {"look_up_key", &DemoCameraKeyBindings::lookUp},
            {"look_down_key", &DemoCameraKeyBindings::lookDown}}};
    for (const auto &[name, member] : fields)
    {
        const auto configured = context.configurationStringInSection("camera", name);
        if (!configured)
            continue;
        window::Key parsed = window::Key::Unknown;
        if (parseWindowKey(*configured, parsed))
            result.*member = parsed;
        else
            std::fprintf(stderr, "demo: ignoring unknown [camera] %s '%s'\n", name, configured->c_str());
    }
    return result;
}

} // namespace demo
