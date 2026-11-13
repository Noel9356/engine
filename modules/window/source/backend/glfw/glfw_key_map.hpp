// GLFW key-code <-> engine Key translation for the GLFW backend.
//
// One constexpr table drives both directions so the mapping cannot drift
// apart: mapGlfwKey indexes a table built from keyPairs by GLFW code,
// mapEngineKey by engine key. Keys absent from the table translate to
// Key::Unknown / GLFW_KEY_UNKNOWN.

#ifndef ENGINE_WINDOW_MODULE_GLFW_KEY_MAP_HPP
#define ENGINE_WINDOW_MODULE_GLFW_KEY_MAP_HPP

#if defined(ENGINE_WINDOW_BACKEND_GLFW)

#include <engine/window/capabilities.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <array>
#include <cstddef>
#include <utility>

namespace engine
{
namespace window
{
namespace glfw_keys
{

// (engine Key, GLFW key code) pairs; every tracked key other than Unknown.
// Slash is the last tracked key, so the table covers the full 64-bit mask.
constexpr std::array<std::pair<Key, int>, 63> keyPairs{{
    {Key::Escape, GLFW_KEY_ESCAPE},
    {Key::Space, GLFW_KEY_SPACE},
    {Key::W, GLFW_KEY_W},
    {Key::A, GLFW_KEY_A},
    {Key::S, GLFW_KEY_S},
    {Key::D, GLFW_KEY_D},
    {Key::Left, GLFW_KEY_LEFT},
    {Key::Right, GLFW_KEY_RIGHT},
    {Key::Up, GLFW_KEY_UP},
    {Key::Down, GLFW_KEY_DOWN},
    {Key::LeftShift, GLFW_KEY_LEFT_SHIFT},
    {Key::LeftControl, GLFW_KEY_LEFT_CONTROL},
    {Key::Q, GLFW_KEY_Q},
    {Key::E, GLFW_KEY_E},
    {Key::R, GLFW_KEY_R},
    {Key::F, GLFW_KEY_F},
    {Key::Z, GLFW_KEY_Z},
    {Key::X, GLFW_KEY_X},
    {Key::C, GLFW_KEY_C},
    {Key::V, GLFW_KEY_V},
    {Key::B, GLFW_KEY_B},
    {Key::N, GLFW_KEY_N},
    {Key::M, GLFW_KEY_M},
    {Key::T, GLFW_KEY_T},
    {Key::G, GLFW_KEY_G},
    {Key::Y, GLFW_KEY_Y},
    {Key::U, GLFW_KEY_U},
    {Key::I, GLFW_KEY_I},
    {Key::O, GLFW_KEY_O},
    {Key::P, GLFW_KEY_P},
    {Key::J, GLFW_KEY_J},
    {Key::K, GLFW_KEY_K},
    {Key::L, GLFW_KEY_L},
    {Key::H, GLFW_KEY_H},
    {Key::Num0, GLFW_KEY_0},
    {Key::Num1, GLFW_KEY_1},
    {Key::Num2, GLFW_KEY_2},
    {Key::Num3, GLFW_KEY_3},
    {Key::Num4, GLFW_KEY_4},
    {Key::Num5, GLFW_KEY_5},
    {Key::Num6, GLFW_KEY_6},
    {Key::Num7, GLFW_KEY_7},
    {Key::Num8, GLFW_KEY_8},
    {Key::Num9, GLFW_KEY_9},
    {Key::Tab, GLFW_KEY_TAB},
    {Key::Enter, GLFW_KEY_ENTER},
    {Key::Backspace, GLFW_KEY_BACKSPACE},
    {Key::LeftAlt, GLFW_KEY_LEFT_ALT},
    {Key::RightShift, GLFW_KEY_RIGHT_SHIFT},
    {Key::RightControl, GLFW_KEY_RIGHT_CONTROL},
    {Key::RightAlt, GLFW_KEY_RIGHT_ALT},
    {Key::Delete, GLFW_KEY_DELETE},
    {Key::Home, GLFW_KEY_HOME},
    {Key::End, GLFW_KEY_END},
    {Key::PageUp, GLFW_KEY_PAGE_UP},
    {Key::PageDown, GLFW_KEY_PAGE_DOWN},
    {Key::Insert, GLFW_KEY_INSERT},
    {Key::Grave, GLFW_KEY_GRAVE_ACCENT},
    {Key::Minus, GLFW_KEY_MINUS},
    {Key::Equal, GLFW_KEY_EQUAL},
    {Key::Comma, GLFW_KEY_COMMA},
    {Key::Period, GLFW_KEY_PERIOD},
    {Key::Slash, GLFW_KEY_SLASH},
}};

static_assert(keyPairs.size() == static_cast<std::size_t>(Key::Slash), "every tracked key must be mapped exactly once");

// O(1) lookup tables compiled from keyPairs: GLFW code -> engine Key and
// engine Key -> GLFW code (unmapped slots hold Unknown / GLFW_KEY_UNKNOWN).
inline constexpr auto glfwToEngine = [] {
    std::array<Key, GLFW_KEY_LAST + 1> table{};
    for (const auto &pair : keyPairs)
    {
        table[static_cast<std::size_t>(pair.second)] = pair.first;
    }
    return table;
}();

inline constexpr auto engineToGlfw = [] {
    std::array<int, 64> table{};
    for (auto &entry : table)
    {
        entry = GLFW_KEY_UNKNOWN;
    }
    for (const auto &pair : keyPairs)
    {
        table[static_cast<std::size_t>(pair.first)] = pair.second;
    }
    return table;
}();

// GLFW key code -> engine Key (Unknown for keys the module does not track).
inline Key mapGlfwKey(int key)
{
    return key >= 0 && key <= GLFW_KEY_LAST ? glfwToEngine[static_cast<std::size_t>(key)] : Key::Unknown;
}

// Engine Key -> GLFW key code (GLFW_KEY_UNKNOWN for Key::Unknown).
inline int mapEngineKey(Key key)
{
    const std::size_t index = static_cast<std::size_t>(key);
    return index < 64 ? engineToGlfw[index] : GLFW_KEY_UNKNOWN;
}

} // namespace glfw_keys
} // namespace window
} // namespace engine

#endif

#endif
