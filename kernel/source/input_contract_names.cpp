// The key code name table of the input contract (engine/capabilities/
// input_contract.hpp). One shared definition in the kernel: the name mapping
// is contract metadata, not input-backend detail, and every consumer - the
// input module, the screenshot trigger configuration, the Lua script bridge -
// already links the kernel. In the monolithic layout a module-local copy
// worked because one executable link resolved the symbol for everyone; across
// module plugins each shared library resolves its own symbols, so a
// module-owned definition silently strands the other plugins (undefined
// symbol at load). The kernel is the one binary every component binds to.

#include "engine/capabilities/input_contract.hpp"

#include <string>

namespace engine
{

namespace
{

const char* keyCodeNameTable(KeyCode keyCode)
{
    switch (keyCode)
    {
    case KeyCode::Space: return "Space";
    case KeyCode::Apostrophe: return "Apostrophe";
    case KeyCode::Comma: return "Comma";
    case KeyCode::Minus: return "Minus";
    case KeyCode::Period: return "Period";
    case KeyCode::Slash: return "Slash";
    case KeyCode::Key0: return "0";
    case KeyCode::Key1: return "1";
    case KeyCode::Key2: return "2";
    case KeyCode::Key3: return "3";
    case KeyCode::Key4: return "4";
    case KeyCode::Key5: return "5";
    case KeyCode::Key6: return "6";
    case KeyCode::Key7: return "7";
    case KeyCode::Key8: return "8";
    case KeyCode::Key9: return "9";
    case KeyCode::Semicolon: return "Semicolon";
    case KeyCode::Equal: return "Equal";
    case KeyCode::A: return "A";
    case KeyCode::B: return "B";
    case KeyCode::C: return "C";
    case KeyCode::D: return "D";
    case KeyCode::E: return "E";
    case KeyCode::F: return "F";
    case KeyCode::G: return "G";
    case KeyCode::H: return "H";
    case KeyCode::I: return "I";
    case KeyCode::J: return "J";
    case KeyCode::K: return "K";
    case KeyCode::L: return "L";
    case KeyCode::M: return "M";
    case KeyCode::N: return "N";
    case KeyCode::O: return "O";
    case KeyCode::P: return "P";
    case KeyCode::Q: return "Q";
    case KeyCode::R: return "R";
    case KeyCode::S: return "S";
    case KeyCode::T: return "T";
    case KeyCode::U: return "U";
    case KeyCode::V: return "V";
    case KeyCode::W: return "W";
    case KeyCode::X: return "X";
    case KeyCode::Y: return "Y";
    case KeyCode::Z: return "Z";
    case KeyCode::LeftBracket: return "LeftBracket";
    case KeyCode::Backslash: return "Backslash";
    case KeyCode::RightBracket: return "RightBracket";
    case KeyCode::GraveAccent: return "GraveAccent";
    case KeyCode::Escape: return "Escape";
    case KeyCode::Enter: return "Enter";
    case KeyCode::Tab: return "Tab";
    case KeyCode::Backspace: return "Backspace";
    case KeyCode::Insert: return "Insert";
    case KeyCode::Delete: return "Delete";
    case KeyCode::Right: return "Right";
    case KeyCode::Left: return "Left";
    case KeyCode::Down: return "Down";
    case KeyCode::Up: return "Up";
    case KeyCode::PageUp: return "PageUp";
    case KeyCode::PageDown: return "PageDown";
    case KeyCode::Home: return "Home";
    case KeyCode::End: return "End";
    case KeyCode::CapsLock: return "CapsLock";
    case KeyCode::ScrollLock: return "ScrollLock";
    case KeyCode::NumLock: return "NumLock";
    case KeyCode::PrintScreen: return "PrintScreen";
    case KeyCode::Pause: return "Pause";
    case KeyCode::F1: return "F1";
    case KeyCode::F2: return "F2";
    case KeyCode::F3: return "F3";
    case KeyCode::F4: return "F4";
    case KeyCode::F5: return "F5";
    case KeyCode::F6: return "F6";
    case KeyCode::F7: return "F7";
    case KeyCode::F8: return "F8";
    case KeyCode::F9: return "F9";
    case KeyCode::F10: return "F10";
    case KeyCode::F11: return "F11";
    case KeyCode::F12: return "F12";
    case KeyCode::LeftShift: return "LeftShift";
    case KeyCode::LeftControl: return "LeftControl";
    case KeyCode::LeftAlt: return "LeftAlt";
    case KeyCode::LeftSuper: return "LeftSuper";
    case KeyCode::RightShift: return "RightShift";
    case KeyCode::RightControl: return "RightControl";
    case KeyCode::RightAlt: return "RightAlt";
    case KeyCode::RightSuper: return "RightSuper";
    case KeyCode::Menu: return "Menu";
    default: return nullptr;
    }
}

struct KeyCodeNameEntry
{
    const char* name;
    KeyCode keyCode;
};

const KeyCodeNameEntry* keyCodeNameEntries()
{
    static const KeyCodeNameEntry entries[] = {
        {"Space", KeyCode::Space},       {"Apostrophe", KeyCode::Apostrophe},
        {"Comma", KeyCode::Comma},       {"Minus", KeyCode::Minus},
        {"Period", KeyCode::Period},     {"Slash", KeyCode::Slash},
        {"0", KeyCode::Key0},            {"1", KeyCode::Key1},
        {"2", KeyCode::Key2},            {"3", KeyCode::Key3},
        {"4", KeyCode::Key4},            {"5", KeyCode::Key5},
        {"6", KeyCode::Key6},            {"7", KeyCode::Key7},
        {"8", KeyCode::Key8},            {"9", KeyCode::Key9},
        {"Semicolon", KeyCode::Semicolon}, {"Equal", KeyCode::Equal},
        {"A", KeyCode::A},               {"B", KeyCode::B},
        {"C", KeyCode::C},               {"D", KeyCode::D},
        {"E", KeyCode::E},               {"F", KeyCode::F},
        {"G", KeyCode::G},               {"H", KeyCode::H},
        {"I", KeyCode::I},               {"J", KeyCode::J},
        {"K", KeyCode::K},               {"L", KeyCode::L},
        {"M", KeyCode::M},               {"N", KeyCode::N},
        {"O", KeyCode::O},               {"P", KeyCode::P},
        {"Q", KeyCode::Q},               {"R", KeyCode::R},
        {"S", KeyCode::S},               {"T", KeyCode::T},
        {"U", KeyCode::U},               {"V", KeyCode::V},
        {"W", KeyCode::W},               {"X", KeyCode::X},
        {"Y", KeyCode::Y},               {"Z", KeyCode::Z},
        {"LeftBracket", KeyCode::LeftBracket}, {"Backslash", KeyCode::Backslash},
        {"RightBracket", KeyCode::RightBracket}, {"GraveAccent", KeyCode::GraveAccent},
        {"Escape", KeyCode::Escape},     {"Enter", KeyCode::Enter},
        {"Tab", KeyCode::Tab},           {"Backspace", KeyCode::Backspace},
        {"Insert", KeyCode::Insert},     {"Delete", KeyCode::Delete},
        {"Right", KeyCode::Right},       {"Left", KeyCode::Left},
        {"Down", KeyCode::Down},         {"Up", KeyCode::Up},
        {"PageUp", KeyCode::PageUp},     {"PageDown", KeyCode::PageDown},
        {"Home", KeyCode::Home},         {"End", KeyCode::End},
        {"CapsLock", KeyCode::CapsLock}, {"ScrollLock", KeyCode::ScrollLock},
        {"NumLock", KeyCode::NumLock},   {"PrintScreen", KeyCode::PrintScreen},
        {"Pause", KeyCode::Pause},       {"F1", KeyCode::F1},
        {"F2", KeyCode::F2},             {"F3", KeyCode::F3},
        {"F4", KeyCode::F4},             {"F5", KeyCode::F5},
        {"F6", KeyCode::F6},             {"F7", KeyCode::F7},
        {"F8", KeyCode::F8},             {"F9", KeyCode::F9},
        {"F10", KeyCode::F10},           {"F11", KeyCode::F11},
        {"F12", KeyCode::F12},           {"LeftShift", KeyCode::LeftShift},
        {"LeftControl", KeyCode::LeftControl}, {"LeftAlt", KeyCode::LeftAlt},
        {"LeftSuper", KeyCode::LeftSuper}, {"RightShift", KeyCode::RightShift},
        {"RightControl", KeyCode::RightControl}, {"RightAlt", KeyCode::RightAlt},
        {"RightSuper", KeyCode::RightSuper}, {"Menu", KeyCode::Menu},
        {nullptr, KeyCode::Unknown}, // sentinel
    };
    return entries;
}

} // namespace

const char* keyCodeName(KeyCode keyCode)
{
    return keyCodeNameTable(keyCode);
}

KeyCode keyCodeFromName(const std::string& keyName)
{
    for (const KeyCodeNameEntry* entry = keyCodeNameEntries(); entry->name != nullptr; ++entry)
    {
        if (keyName == entry->name)
        {
            return entry->keyCode;
        }
    }
    return KeyCode::Unknown;
}

} // namespace engine
