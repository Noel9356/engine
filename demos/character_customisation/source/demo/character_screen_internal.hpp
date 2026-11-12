// Private helpers shared by CharacterScreen implementation units.
#ifndef ENGINE_DEMO_CHARACTER_SCREEN_INTERNAL_HPP
#define ENGINE_DEMO_CHARACTER_SCREEN_INTERNAL_HPP

#include "character_screen.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace demo
{
using engine::character::CharacterError;
using engine::character::CharacterHandle;
using engine::character::InstanceId;
using engine::character::ParameterDefinition;
using engine::character::ValidationIssue;
using engine::renderer::MeshHandle;
using engine::renderer::MeshMaterial;
using engine::ui::UiCaptureState;
using engine::ui::UiContextHandle;
using engine::ui::UiDrawCommand;
using engine::ui::UiDrawData;
using engine::ui::UiError;
using engine::ui::UiFrameDescription;
using engine::ui::UiKey;
using engine::ui::UiWindowFlags;
using engine::window::Key;
using engine::window::MouseButton;
using engine::window::WindowEvent;
using engine::window::WindowEventKind;
using engine::window::WindowHandle;

namespace screen_detail
{
inline constexpr const char *kRecipeFileName = "character_recipe.toml";
inline constexpr const char *kFontRelativePath = "fonts/Inter-Variable.ttf";

inline bool isSkinPart(const char *partId)
{
    static constexpr const char *skinParts[] = {
        "torso_humanoid", "arm_humanoid_left", "arm_humanoid_right", "leg_humanoid_left", "leg_humanoid_right",
        "neck_transition", "head_human",
    };
    if (partId == nullptr)
        return false;
    for (const char *candidate : skinParts)
        if (std::strcmp(partId, candidate) == 0)
            return true;
    return false;
}

inline UiKey mapWindowKey(Key key)
{
    switch (key)
    {
    case Key::Escape: return UiKey::Escape;
    case Key::Space: return UiKey::Space;
    case Key::Enter: return UiKey::Enter;
    case Key::Tab: return UiKey::Tab;
    case Key::Backspace: return UiKey::Backspace;
    case Key::Left: return UiKey::Left;
    case Key::Right: return UiKey::Right;
    case Key::Up: return UiKey::Up;
    case Key::Down: return UiKey::Down;
    case Key::W: return UiKey::W;
    case Key::A: return UiKey::A;
    case Key::S: return UiKey::S;
    case Key::D: return UiKey::D;
    case Key::Q: return UiKey::Q;
    case Key::E: return UiKey::E;
    case Key::LeftShift: return UiKey::LeftShift;
    case Key::LeftControl: return UiKey::LeftControl;
    case Key::LeftAlt: return UiKey::LeftAlt;
    case Key::Num0: return UiKey::D0;
    case Key::Num1: return UiKey::D1;
    case Key::Num2: return UiKey::D2;
    case Key::Num3: return UiKey::D3;
    case Key::Num4: return UiKey::D4;
    case Key::Num5: return UiKey::D5;
    case Key::Num6: return UiKey::D6;
    case Key::Num7: return UiKey::D7;
    case Key::Num8: return UiKey::D8;
    case Key::Num9: return UiKey::D9;
    default: return UiKey::None;
    }
}

inline std::uint64_t characterScreenKeyMaskBit(Key key)
{
    const auto value = static_cast<std::uint32_t>(key);
    return value > 0 && value < 64 ? (std::uint64_t{1} << value) : 0;
}

inline void setYawRotation(float *outQuaternion4, float angleRadians)
{
    outQuaternion4[0] = 0.0f;
    outQuaternion4[1] = std::sin(angleRadians * 0.5f);
    outQuaternion4[2] = 0.0f;
    outQuaternion4[3] = std::cos(angleRadians * 0.5f);
}

inline void setRollRotation(float *outQuaternion4, float angleRadians)
{
    outQuaternion4[0] = 0.0f;
    outQuaternion4[1] = 0.0f;
    outQuaternion4[2] = std::sin(angleRadians * 0.5f);
    outQuaternion4[3] = std::cos(angleRadians * 0.5f);
}
} // namespace screen_detail
} // namespace demo

#endif
