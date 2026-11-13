// Enum -> ImGui mapping helpers.
//
// Single source of truth for the UiKey/UiGamepadButton/UiWindowFlags/UiColor/
// UiStyleVariable and the new flag enums -> ImGui counterparts. Defined out
// of line so the widget TUs share the same translation unit instead of
// regenerating the switch tables in every TU.

#include "ui_backend.hpp"

#if defined(ENGINE_UI_BACKEND_IMGUI)

namespace engine
{
namespace ui
{
namespace backend
{

ImGuiKey nativeKey(UiKey key) noexcept
{
    switch (key)
    {
    case UiKey::Tab:
        return ImGuiKey_Tab;
    case UiKey::Left:
        return ImGuiKey_LeftArrow;
    case UiKey::Right:
        return ImGuiKey_RightArrow;
    case UiKey::Up:
        return ImGuiKey_UpArrow;
    case UiKey::Down:
        return ImGuiKey_DownArrow;
    case UiKey::PageUp:
        return ImGuiKey_PageUp;
    case UiKey::PageDown:
        return ImGuiKey_PageDown;
    case UiKey::Home:
        return ImGuiKey_Home;
    case UiKey::End:
        return ImGuiKey_End;
    case UiKey::Insert:
        return ImGuiKey_Insert;
    case UiKey::Delete:
        return ImGuiKey_Delete;
    case UiKey::Backspace:
        return ImGuiKey_Backspace;
    case UiKey::Space:
        return ImGuiKey_Space;
    case UiKey::Enter:
        return ImGuiKey_Enter;
    case UiKey::Escape:
        return ImGuiKey_Escape;
    case UiKey::Apostrophe:
        return ImGuiKey_Apostrophe;
    case UiKey::Comma:
        return ImGuiKey_Comma;
    case UiKey::Minus:
        return ImGuiKey_Minus;
    case UiKey::Period:
        return ImGuiKey_Period;
    case UiKey::Slash:
        return ImGuiKey_Slash;
    case UiKey::Semicolon:
        return ImGuiKey_Semicolon;
    case UiKey::Equal:
        return ImGuiKey_Equal;
    case UiKey::LeftBracket:
        return ImGuiKey_LeftBracket;
    case UiKey::Backslash:
        return ImGuiKey_Backslash;
    case UiKey::RightBracket:
        return ImGuiKey_RightBracket;
    case UiKey::GraveAccent:
        return ImGuiKey_GraveAccent;
    case UiKey::CapsLock:
        return ImGuiKey_CapsLock;
    case UiKey::ScrollLock:
        return ImGuiKey_ScrollLock;
    case UiKey::NumLock:
        return ImGuiKey_NumLock;
    case UiKey::PrintScreen:
        return ImGuiKey_PrintScreen;
    case UiKey::Pause:
        return ImGuiKey_Pause;
    case UiKey::KeypadDecimal:
        return ImGuiKey_KeypadDecimal;
    case UiKey::KeypadDivide:
        return ImGuiKey_KeypadDivide;
    case UiKey::KeypadMultiply:
        return ImGuiKey_KeypadMultiply;
    case UiKey::KeypadSubtract:
        return ImGuiKey_KeypadSubtract;
    case UiKey::KeypadAdd:
        return ImGuiKey_KeypadAdd;
    case UiKey::KeypadEnter:
        return ImGuiKey_KeypadEnter;
    case UiKey::KeypadEqual:
        return ImGuiKey_KeypadEqual;
    case UiKey::LeftControl:
        return ImGuiKey_LeftCtrl;
    case UiKey::LeftShift:
        return ImGuiKey_LeftShift;
    case UiKey::LeftAlt:
        return ImGuiKey_LeftAlt;
    case UiKey::LeftSuper:
        return ImGuiKey_LeftSuper;
    case UiKey::RightControl:
        return ImGuiKey_RightCtrl;
    case UiKey::RightShift:
        return ImGuiKey_RightShift;
    case UiKey::RightAlt:
        return ImGuiKey_RightAlt;
    case UiKey::RightSuper:
        return ImGuiKey_RightSuper;
    case UiKey::D0:
        return ImGuiKey_0;
    case UiKey::D1:
        return ImGuiKey_1;
    case UiKey::D2:
        return ImGuiKey_2;
    case UiKey::D3:
        return ImGuiKey_3;
    case UiKey::D4:
        return ImGuiKey_4;
    case UiKey::D5:
        return ImGuiKey_5;
    case UiKey::D6:
        return ImGuiKey_6;
    case UiKey::D7:
        return ImGuiKey_7;
    case UiKey::D8:
        return ImGuiKey_8;
    case UiKey::D9:
        return ImGuiKey_9;
    case UiKey::A:
        return ImGuiKey_A;
    case UiKey::B:
        return ImGuiKey_B;
    case UiKey::C:
        return ImGuiKey_C;
    case UiKey::D:
        return ImGuiKey_D;
    case UiKey::E:
        return ImGuiKey_E;
    case UiKey::F:
        return ImGuiKey_F;
    case UiKey::G:
        return ImGuiKey_G;
    case UiKey::H:
        return ImGuiKey_H;
    case UiKey::I:
        return ImGuiKey_I;
    case UiKey::J:
        return ImGuiKey_J;
    case UiKey::K:
        return ImGuiKey_K;
    case UiKey::L:
        return ImGuiKey_L;
    case UiKey::M:
        return ImGuiKey_M;
    case UiKey::N:
        return ImGuiKey_N;
    case UiKey::O:
        return ImGuiKey_O;
    case UiKey::P:
        return ImGuiKey_P;
    case UiKey::Q:
        return ImGuiKey_Q;
    case UiKey::R:
        return ImGuiKey_R;
    case UiKey::S:
        return ImGuiKey_S;
    case UiKey::T:
        return ImGuiKey_T;
    case UiKey::U:
        return ImGuiKey_U;
    case UiKey::V:
        return ImGuiKey_V;
    case UiKey::W:
        return ImGuiKey_W;
    case UiKey::X:
        return ImGuiKey_X;
    case UiKey::Y:
        return ImGuiKey_Y;
    case UiKey::Z:
        return ImGuiKey_Z;
    case UiKey::F1:
        return ImGuiKey_F1;
    case UiKey::F2:
        return ImGuiKey_F2;
    case UiKey::F3:
        return ImGuiKey_F3;
    case UiKey::F4:
        return ImGuiKey_F4;
    case UiKey::F5:
        return ImGuiKey_F5;
    case UiKey::F6:
        return ImGuiKey_F6;
    case UiKey::F7:
        return ImGuiKey_F7;
    case UiKey::F8:
        return ImGuiKey_F8;
    case UiKey::F9:
        return ImGuiKey_F9;
    case UiKey::F10:
        return ImGuiKey_F10;
    case UiKey::F11:
        return ImGuiKey_F11;
    case UiKey::F12:
        return ImGuiKey_F12;
    case UiKey::Keypad0:
        return ImGuiKey_Keypad0;
    case UiKey::Keypad1:
        return ImGuiKey_Keypad1;
    case UiKey::Keypad2:
        return ImGuiKey_Keypad2;
    case UiKey::Keypad3:
        return ImGuiKey_Keypad3;
    case UiKey::Keypad4:
        return ImGuiKey_Keypad4;
    case UiKey::Keypad5:
        return ImGuiKey_Keypad5;
    case UiKey::Keypad6:
        return ImGuiKey_Keypad6;
    case UiKey::Keypad7:
        return ImGuiKey_Keypad7;
    case UiKey::Keypad8:
        return ImGuiKey_Keypad8;
    case UiKey::Keypad9:
        return ImGuiKey_Keypad9;
    case UiKey::None:
        break;
    }
    return ImGuiKey_None;
}

ImGuiKey nativeGamepadButton(UiGamepadButton button) noexcept
{
    switch (button)
    {
    case UiGamepadButton::FaceLeft:
        return ImGuiKey_GamepadFaceLeft;
    case UiGamepadButton::FaceRight:
        return ImGuiKey_GamepadFaceRight;
    case UiGamepadButton::FaceUp:
        return ImGuiKey_GamepadFaceUp;
    case UiGamepadButton::FaceDown:
        return ImGuiKey_GamepadFaceDown;
    case UiGamepadButton::DpadLeft:
        return ImGuiKey_GamepadDpadLeft;
    case UiGamepadButton::DpadRight:
        return ImGuiKey_GamepadDpadRight;
    case UiGamepadButton::DpadUp:
        return ImGuiKey_GamepadDpadUp;
    case UiGamepadButton::DpadDown:
        return ImGuiKey_GamepadDpadDown;
    case UiGamepadButton::LeftShoulder:
        return ImGuiKey_GamepadL1;
    case UiGamepadButton::RightShoulder:
        return ImGuiKey_GamepadR1;
    case UiGamepadButton::LeftTrigger:
        return ImGuiKey_GamepadL2;
    case UiGamepadButton::RightTrigger:
        return ImGuiKey_GamepadR2;
    case UiGamepadButton::LeftStick:
        return ImGuiKey_GamepadL3;
    case UiGamepadButton::RightStick:
        return ImGuiKey_GamepadR3;
    case UiGamepadButton::Start:
        return ImGuiKey_GamepadStart;
    case UiGamepadButton::Back:
        return ImGuiKey_GamepadBack;
    }
    return ImGuiKey_None;
}

ImGuiWindowFlags nativeWindowFlags(UiWindowFlags flags) noexcept
{
    const std::uint32_t value  = static_cast<std::uint32_t>(flags);
    ImGuiWindowFlags    result = 0;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoTitleBar))
        result |= ImGuiWindowFlags_NoTitleBar;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoResize))
        result |= ImGuiWindowFlags_NoResize;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoMove))
        result |= ImGuiWindowFlags_NoMove;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoScrollbar))
        result |= ImGuiWindowFlags_NoScrollbar;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoCollapse))
        result |= ImGuiWindowFlags_NoCollapse;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::AlwaysAutoResize))
        result |= ImGuiWindowFlags_AlwaysAutoResize;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoBackground))
        result |= ImGuiWindowFlags_NoBackground;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoSavedSettings))
        result |= ImGuiWindowFlags_NoSavedSettings;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::MenuBar))
        result |= ImGuiWindowFlags_MenuBar;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoBringToFront))
        result |= ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoNavigation))
        result |= ImGuiWindowFlags_NoNav;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::HorizontalScrollbar))
        result |= ImGuiWindowFlags_HorizontalScrollbar;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoFocusOnAppearing))
        result |= ImGuiWindowFlags_NoFocusOnAppearing;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoNavInputs))
        result |= ImGuiWindowFlags_NoNavInputs;
    if (value & static_cast<std::uint32_t>(UiWindowFlags::NoNavFocus))
        result |= ImGuiWindowFlags_NoNavFocus;
    return result;
}

ImGuiCol nativeColor(UiColor color) noexcept
{
    switch (color)
    {
    case UiColor::Text:
        return ImGuiCol_Text;
    case UiColor::TextDisabled:
        return ImGuiCol_TextDisabled;
    case UiColor::WindowBackground:
        return ImGuiCol_WindowBg;
    case UiColor::ChildBackground:
        return ImGuiCol_ChildBg;
    case UiColor::PopupBackground:
        return ImGuiCol_PopupBg;
    case UiColor::Border:
        return ImGuiCol_Border;
    case UiColor::BorderShadow:
        return ImGuiCol_BorderShadow;
    case UiColor::FrameBackground:
        return ImGuiCol_FrameBg;
    case UiColor::FrameHovered:
        return ImGuiCol_FrameBgHovered;
    case UiColor::FrameActive:
        return ImGuiCol_FrameBgActive;
    case UiColor::TitleBackground:
        return ImGuiCol_TitleBg;
    case UiColor::TitleBackgroundActive:
        return ImGuiCol_TitleBgActive;
    case UiColor::TitleBackgroundCollapsed:
        return ImGuiCol_TitleBgCollapsed;
    case UiColor::MenuBarBackground:
        return ImGuiCol_MenuBarBg;
    case UiColor::ScrollbarBackground:
        return ImGuiCol_ScrollbarBg;
    case UiColor::ScrollbarGrab:
        return ImGuiCol_ScrollbarGrab;
    case UiColor::ScrollbarGrabHovered:
        return ImGuiCol_ScrollbarGrabHovered;
    case UiColor::ScrollbarGrabActive:
        return ImGuiCol_ScrollbarGrabActive;
    case UiColor::CheckMark:
        return ImGuiCol_CheckMark;
    case UiColor::SliderGrab:
        return ImGuiCol_SliderGrab;
    case UiColor::SliderGrabActive:
        return ImGuiCol_SliderGrabActive;
    case UiColor::Button:
        return ImGuiCol_Button;
    case UiColor::ButtonHovered:
        return ImGuiCol_ButtonHovered;
    case UiColor::ButtonActive:
        return ImGuiCol_ButtonActive;
    case UiColor::Header:
        return ImGuiCol_Header;
    case UiColor::HeaderHovered:
        return ImGuiCol_HeaderHovered;
    case UiColor::HeaderActive:
        return ImGuiCol_HeaderActive;
    case UiColor::Separator:
        return ImGuiCol_Separator;
    case UiColor::SeparatorHovered:
        return ImGuiCol_SeparatorHovered;
    case UiColor::SeparatorActive:
        return ImGuiCol_SeparatorActive;
    case UiColor::ResizeGrip:
        return ImGuiCol_ResizeGrip;
    case UiColor::ResizeGripHovered:
        return ImGuiCol_ResizeGripHovered;
    case UiColor::ResizeGripActive:
        return ImGuiCol_ResizeGripActive;
    case UiColor::Tab:
        return ImGuiCol_Tab;
    case UiColor::TabHovered:
        return ImGuiCol_TabHovered;
    case UiColor::TabActive:
        return ImGuiCol_TabActive;
    case UiColor::TabUnfocused:
        return ImGuiCol_TabUnfocused;
    case UiColor::TabUnfocusedActive:
        return ImGuiCol_TabUnfocusedActive;
    case UiColor::PlotLines:
        return ImGuiCol_PlotLines;
    case UiColor::PlotLinesHovered:
        return ImGuiCol_PlotLinesHovered;
    case UiColor::PlotHistogram:
        return ImGuiCol_PlotHistogram;
    case UiColor::PlotHistogramHovered:
        return ImGuiCol_PlotHistogramHovered;
    case UiColor::TableHeaderBackground:
        return ImGuiCol_TableHeaderBg;
    case UiColor::TableBorderStrong:
        return ImGuiCol_TableBorderStrong;
    case UiColor::TableBorderLight:
        return ImGuiCol_TableBorderLight;
    case UiColor::TableRowBackground:
        return ImGuiCol_TableRowBg;
    case UiColor::TableRowBackgroundAlt:
        return ImGuiCol_TableRowBgAlt;
    case UiColor::TextSelectedBackground:
        return ImGuiCol_TextSelectedBg;
    case UiColor::DragDropTarget:
        return ImGuiCol_DragDropTarget;
    case UiColor::NavHighlight:
        return ImGuiCol_NavHighlight;
    case UiColor::NavWindowingHighlight:
        return ImGuiCol_NavWindowingHighlight;
    case UiColor::NavWindowingDimBackground:
        return ImGuiCol_NavWindowingDimBg;
    case UiColor::ModalWindowDimBackground:
        return ImGuiCol_ModalWindowDimBg;
    }
    return ImGuiCol_Text;
}

ImGuiStyleVar nativeStyleVariable(UiStyleVariable variable) noexcept
{
    switch (variable)
    {
    case UiStyleVariable::Alpha:
        return ImGuiStyleVar_Alpha;
    case UiStyleVariable::DisabledAlpha:
        return ImGuiStyleVar_DisabledAlpha;
    case UiStyleVariable::WindowRounding:
        return ImGuiStyleVar_WindowRounding;
    case UiStyleVariable::WindowBorderSize:
        return ImGuiStyleVar_WindowBorderSize;
    case UiStyleVariable::ChildRounding:
        return ImGuiStyleVar_ChildRounding;
    case UiStyleVariable::ChildBorderSize:
        return ImGuiStyleVar_ChildBorderSize;
    case UiStyleVariable::PopupRounding:
        return ImGuiStyleVar_PopupRounding;
    case UiStyleVariable::PopupBorderSize:
        return ImGuiStyleVar_PopupBorderSize;
    case UiStyleVariable::FrameRounding:
        return ImGuiStyleVar_FrameRounding;
    case UiStyleVariable::FrameBorderSize:
        return ImGuiStyleVar_FrameBorderSize;
    case UiStyleVariable::IndentSpacing:
        return ImGuiStyleVar_IndentSpacing;
    case UiStyleVariable::ScrollbarSize:
        return ImGuiStyleVar_ScrollbarSize;
    case UiStyleVariable::ScrollbarRounding:
        return ImGuiStyleVar_ScrollbarRounding;
    case UiStyleVariable::GrabRounding:
        return ImGuiStyleVar_GrabRounding;
    case UiStyleVariable::GrabMinSize:
        return ImGuiStyleVar_GrabMinSize;
    case UiStyleVariable::TabRounding:
        return ImGuiStyleVar_TabRounding;
    case UiStyleVariable::TabBorderSize:
        return ImGuiStyleVar_TabBorderSize;
    case UiStyleVariable::TabBarBorderSize:
        return ImGuiStyleVar_TabBarBorderSize;
    case UiStyleVariable::TabBarOverlineSize:
        return ImGuiStyleVar_TabBarOverlineSize;
    case UiStyleVariable::ImageBorderSize:
        return ImGuiStyleVar_ImageBorderSize;
    case UiStyleVariable::TableAngledHeadersAngle:
        return ImGuiStyleVar_TableAngledHeadersAngle;
    case UiStyleVariable::SeparatorTextBorderSize:
        return ImGuiStyleVar_SeparatorTextBorderSize;
    }
    return ImGuiStyleVar_Alpha;
}

ImGuiChildFlags nativeChildFlags(UiChildFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiChildFlags    result = 0;
    if (value & static_cast<std::uint32_t>(UiChildFlags::Border))
        result |= ImGuiChildFlags_Border;
    if (value & static_cast<std::uint32_t>(UiChildFlags::AlwaysUseWindowPadding))
        result |= ImGuiChildFlags_AlwaysUseWindowPadding;
    if (value & static_cast<std::uint32_t>(UiChildFlags::ResizeX))
        result |= ImGuiChildFlags_ResizeX;
    if (value & static_cast<std::uint32_t>(UiChildFlags::ResizeY))
        result |= ImGuiChildFlags_ResizeY;
    if (value & static_cast<std::uint32_t>(UiChildFlags::AutoResizeX))
        result |= ImGuiChildFlags_AutoResizeX;
    if (value & static_cast<std::uint32_t>(UiChildFlags::AutoResizeY))
        result |= ImGuiChildFlags_AutoResizeY;
    if (value & static_cast<std::uint32_t>(UiChildFlags::AlwaysAutoResize))
        result |= ImGuiChildFlags_AlwaysAutoResize;
    if (value & static_cast<std::uint32_t>(UiChildFlags::FrameStyle))
        result |= ImGuiChildFlags_FrameStyle;
    if (value & static_cast<std::uint32_t>(UiChildFlags::NavFlattened))
        result |= ImGuiChildFlags_NavFlattened;
    return result;
}

ImGuiTreeNodeFlags nativeTreeNodeFlags(UiTreeNodeFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTreeNodeFlags result = 0;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::Selected))
        result |= ImGuiTreeNodeFlags_Selected;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::Framed))
        result |= ImGuiTreeNodeFlags_Framed;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::AllowItemOverlap))
        result |= ImGuiTreeNodeFlags_AllowItemOverlap;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::NoTreePushOnOpen))
        result |= ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::NoAutoOpenOnLog))
        result |= ImGuiTreeNodeFlags_NoAutoOpenOnLog;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::DefaultOpen))
        result |= ImGuiTreeNodeFlags_DefaultOpen;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::OpenOnDoubleClick))
        result |= ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::OpenOnArrow))
        result |= ImGuiTreeNodeFlags_OpenOnArrow;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::Leaf))
        result |= ImGuiTreeNodeFlags_Leaf;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::Bullet))
        result |= ImGuiTreeNodeFlags_Bullet;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::FramePadding))
        result |= ImGuiTreeNodeFlags_FramePadding;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::SpanAvailWidth))
        result |= ImGuiTreeNodeFlags_SpanAvailWidth;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::SpanFullWidth))
        result |= ImGuiTreeNodeFlags_SpanFullWidth;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::SpanAllColumns))
        result |= ImGuiTreeNodeFlags_SpanAllColumns;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::NavLeftJumpsBackHere))
        result |= ImGuiTreeNodeFlags_NavLeftJumpsBackHere;
    if (value & static_cast<std::uint32_t>(UiTreeNodeFlags::CollapsingHeader))
        result |= ImGuiTreeNodeFlags_CollapsingHeader;
    return result;
}

ImGuiSelectableFlags nativeSelectableFlags(UiSelectableFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiSelectableFlags result = 0;
    if (value & static_cast<std::uint32_t>(UiSelectableFlags::NoAutoClosePopups))
        result |= ImGuiSelectableFlags_NoAutoClosePopups;
    if (value & static_cast<std::uint32_t>(UiSelectableFlags::SpanAllColumns))
        result |= ImGuiSelectableFlags_SpanAllColumns;
    if (value & static_cast<std::uint32_t>(UiSelectableFlags::AllowDoubleClick))
        result |= ImGuiSelectableFlags_AllowDoubleClick;
    if (value & static_cast<std::uint32_t>(UiSelectableFlags::Disabled))
        result |= ImGuiSelectableFlags_Disabled;
    if (value & static_cast<std::uint32_t>(UiSelectableFlags::AllowItemOverlap))
        result |= ImGuiSelectableFlags_AllowItemOverlap;
    return result;
}

ImGuiPopupFlags nativePopupFlags(UiPopupFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiPopupFlags    result = 0;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::MouseButtonLeft))
        result |= ImGuiPopupFlags_MouseButtonLeft;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::MouseButtonRight))
        result |= ImGuiPopupFlags_MouseButtonRight;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::MouseButtonMiddle))
        result |= ImGuiPopupFlags_MouseButtonMiddle;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::NoOpenOverExistingPopup))
        result |= ImGuiPopupFlags_NoOpenOverExistingPopup;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::NoOpenOverItems))
        result |= ImGuiPopupFlags_NoOpenOverItems;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::AnyPopupId))
        result |= ImGuiPopupFlags_AnyPopupId;
    if (value & static_cast<std::uint32_t>(UiPopupFlags::AnyPopupLevel))
        result |= ImGuiPopupFlags_AnyPopupLevel;
    return result;
}

ImGuiTabBarFlags nativeTabBarFlags(UiTabBarFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTabBarFlags   result = 0;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::Reorderable))
        result |= ImGuiTabBarFlags_Reorderable;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::AutoSelectNewTabs))
        result |= ImGuiTabBarFlags_AutoSelectNewTabs;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::TabListPopupButton))
        result |= ImGuiTabBarFlags_TabListPopupButton;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::NoCloseWithMiddleMouseButton))
        result |= ImGuiTabBarFlags_NoCloseWithMiddleMouseButton;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::NoTabListScrollingButtons))
        result |= ImGuiTabBarFlags_NoTabListScrollingButtons;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::NoTooltip))
        result |= ImGuiTabBarFlags_NoTooltip;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::DrawSelectedOverline))
        result |= ImGuiTabBarFlags_DrawSelectedOverline;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::FittingPolicyResizeDown))
        result |= ImGuiTabBarFlags_FittingPolicyResizeDown;
    if (value & static_cast<std::uint32_t>(UiTabBarFlags::FittingPolicyScroll))
        result |= ImGuiTabBarFlags_FittingPolicyScroll;
    return result;
}

ImGuiTabItemFlags nativeTabItemFlags(UiTabItemFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTabItemFlags  result = 0;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::UnsavedDocument))
        result |= ImGuiTabItemFlags_UnsavedDocument;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::SetSelected))
        result |= ImGuiTabItemFlags_SetSelected;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::NoCloseWithMiddleMouseButton))
        result |= ImGuiTabItemFlags_NoCloseWithMiddleMouseButton;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::NoPushId))
        result |= ImGuiTabItemFlags_NoPushId;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::NoTooltip))
        result |= ImGuiTabItemFlags_NoTooltip;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::NoReorder))
        result |= ImGuiTabItemFlags_NoReorder;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::Leading))
        result |= ImGuiTabItemFlags_Leading;
    if (value & static_cast<std::uint32_t>(UiTabItemFlags::Trailing))
        result |= ImGuiTabItemFlags_Trailing;
    return result;
}

ImGuiTableFlags nativeTableFlags(UiTableFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTableFlags    result = 0;
    if (value & static_cast<std::uint32_t>(UiTableFlags::Resizable))
        result |= ImGuiTableFlags_Resizable;
    if (value & static_cast<std::uint32_t>(UiTableFlags::Reorderable))
        result |= ImGuiTableFlags_Reorderable;
    if (value & static_cast<std::uint32_t>(UiTableFlags::Hideable))
        result |= ImGuiTableFlags_Hideable;
    if (value & static_cast<std::uint32_t>(UiTableFlags::Sortable))
        result |= ImGuiTableFlags_Sortable;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoSavedSettings))
        result |= ImGuiTableFlags_NoSavedSettings;
    if (value & static_cast<std::uint32_t>(UiTableFlags::ContextMenuInBody))
        result |= ImGuiTableFlags_ContextMenuInBody;
    if (value & static_cast<std::uint32_t>(UiTableFlags::RowBackground))
        result |= ImGuiTableFlags_RowBg;
    if (value & static_cast<std::uint32_t>(UiTableFlags::BordersInnerHorizontal))
        result |= ImGuiTableFlags_BordersInnerH;
    if (value & static_cast<std::uint32_t>(UiTableFlags::BordersOuterHorizontal))
        result |= ImGuiTableFlags_BordersOuterH;
    if (value & static_cast<std::uint32_t>(UiTableFlags::BordersInnerVertical))
        result |= ImGuiTableFlags_BordersInnerV;
    if (value & static_cast<std::uint32_t>(UiTableFlags::BordersOuterVertical))
        result |= ImGuiTableFlags_BordersOuterV;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoBordersInBody))
        result |= ImGuiTableFlags_NoBordersInBody;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoBordersInBodyUntilResize))
        result |= ImGuiTableFlags_NoBordersInBodyUntilResize;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SizingFixedFit))
        result |= ImGuiTableFlags_SizingFixedFit;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SizingFixedSame))
        result |= ImGuiTableFlags_SizingFixedSame;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SizingStretchProp))
        result |= ImGuiTableFlags_SizingStretchProp;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SizingStretchSame))
        result |= ImGuiTableFlags_SizingStretchSame;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoHostExtendX))
        result |= ImGuiTableFlags_NoHostExtendX;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoHostExtendY))
        result |= ImGuiTableFlags_NoHostExtendY;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoKeepColumnsVisible))
        result |= ImGuiTableFlags_NoKeepColumnsVisible;
    if (value & static_cast<std::uint32_t>(UiTableFlags::PreciseWidths))
        result |= ImGuiTableFlags_PreciseWidths;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoClip))
        result |= ImGuiTableFlags_NoClip;
    if (value & static_cast<std::uint32_t>(UiTableFlags::PadOuterX))
        result |= ImGuiTableFlags_PadOuterX;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoPadOuterX))
        result |= ImGuiTableFlags_NoPadOuterX;
    if (value & static_cast<std::uint32_t>(UiTableFlags::NoPadInnerX))
        result |= ImGuiTableFlags_NoPadInnerX;
    if (value & static_cast<std::uint32_t>(UiTableFlags::ScrollX))
        result |= ImGuiTableFlags_ScrollX;
    if (value & static_cast<std::uint32_t>(UiTableFlags::ScrollY))
        result |= ImGuiTableFlags_ScrollY;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SortMulti))
        result |= ImGuiTableFlags_SortMulti;
    if (value & static_cast<std::uint32_t>(UiTableFlags::SortTristate))
        result |= ImGuiTableFlags_SortTristate;
    return result;
}

ImGuiTableColumnFlags nativeTableColumnFlags(UiTableColumnFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTableColumnFlags result = 0;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::DefaultSort))
        result |= ImGuiTableColumnFlags_DefaultSort;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::IndentEnable))
        result |= ImGuiTableColumnFlags_IndentEnable;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoHeaderLabel))
        result |= ImGuiTableColumnFlags_NoHeaderLabel;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoHeaderWidth))
        result |= ImGuiTableColumnFlags_NoHeaderWidth;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoHide))
        result |= ImGuiTableColumnFlags_NoHide;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoClip))
        result |= ImGuiTableColumnFlags_NoClip;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoSort))
        result |= ImGuiTableColumnFlags_NoSort;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoSortAscending))
        result |= ImGuiTableColumnFlags_NoSortAscending;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoSortDescending))
        result |= ImGuiTableColumnFlags_NoSortDescending;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoReorder))
        result |= ImGuiTableColumnFlags_NoReorder;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::NoResize))
        result |= ImGuiTableColumnFlags_NoResize;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::WidthFixed))
        result |= ImGuiTableColumnFlags_WidthFixed;
    if (value & static_cast<std::uint32_t>(UiTableColumnFlags::WidthStretch))
        result |= ImGuiTableColumnFlags_WidthStretch;
    return result;
}

ImGuiTableRowFlags nativeTableRowFlags(UiTableRowFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiTableRowFlags result = 0;
    if (value & static_cast<std::uint32_t>(UiTableRowFlags::Headers))
        result |= ImGuiTableRowFlags_Headers;
    return result;
}

ImGuiColorEditFlags nativeColorEditFlags(UiColorEditFlags flags) noexcept
{
    const std::uint32_t value = static_cast<std::uint32_t>(flags);
    ImGuiColorEditFlags result = 0;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoAlpha))
        result |= ImGuiColorEditFlags_NoAlpha;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoPicker))
        result |= ImGuiColorEditFlags_NoPicker;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoOptions))
        result |= ImGuiColorEditFlags_NoOptions;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoSmallPreview))
        result |= ImGuiColorEditFlags_NoSmallPreview;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoInputs))
        result |= ImGuiColorEditFlags_NoInputs;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoTooltip))
        result |= ImGuiColorEditFlags_NoTooltip;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoDragDrop))
        result |= ImGuiColorEditFlags_NoDragDrop;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::NoBorder))
        result |= ImGuiColorEditFlags_NoBorder;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::AlphaBar))
        result |= ImGuiColorEditFlags_AlphaBar;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::AlphaPreview))
        result |= ImGuiColorEditFlags_AlphaPreview;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::AlphaPreviewHalf))
        result |= ImGuiColorEditFlags_AlphaPreviewHalf;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::DisplayRGB))
        result |= ImGuiColorEditFlags_DisplayRGB;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::DisplayHSV))
        result |= ImGuiColorEditFlags_DisplayHSV;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::DisplayHex))
        result |= ImGuiColorEditFlags_DisplayHex;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::Uint8))
        result |= ImGuiColorEditFlags_Uint8;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::Float))
        result |= ImGuiColorEditFlags_Float;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::PickerHueBar))
        result |= ImGuiColorEditFlags_PickerHueBar;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::PickerHueWheel))
        result |= ImGuiColorEditFlags_PickerHueWheel;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::InputRGB))
        result |= ImGuiColorEditFlags_InputRGB;
    if (value & static_cast<std::uint32_t>(UiColorEditFlags::InputHSV))
        result |= ImGuiColorEditFlags_InputHSV;
    return result;
}

ImGuiMouseCursor nativeMouseCursor(UiMouseCursor cursor) noexcept
{
    switch (cursor)
    {
    case UiMouseCursor::None:
        return ImGuiMouseCursor_None;
    case UiMouseCursor::Arrow:
        return ImGuiMouseCursor_Arrow;
    case UiMouseCursor::TextInput:
        return ImGuiMouseCursor_TextInput;
    case UiMouseCursor::ResizeAll:
        return ImGuiMouseCursor_ResizeAll;
    case UiMouseCursor::ResizeNS:
        return ImGuiMouseCursor_ResizeNS;
    case UiMouseCursor::ResizeEW:
        return ImGuiMouseCursor_ResizeEW;
    case UiMouseCursor::ResizeNESW:
        return ImGuiMouseCursor_ResizeNESW;
    case UiMouseCursor::ResizeNWSE:
        return ImGuiMouseCursor_ResizeNWSE;
    case UiMouseCursor::Hand:
        return ImGuiMouseCursor_Hand;
    case UiMouseCursor::NotAllowed:
        return ImGuiMouseCursor_NotAllowed;
    }
    return ImGuiMouseCursor_Arrow;
}

UiMouseCursor publicMouseCursor(ImGuiMouseCursor cursor) noexcept
{
    switch (cursor)
    {
    case ImGuiMouseCursor_None:
        return UiMouseCursor::None;
    case ImGuiMouseCursor_Arrow:
        return UiMouseCursor::Arrow;
    case ImGuiMouseCursor_TextInput:
        return UiMouseCursor::TextInput;
    case ImGuiMouseCursor_ResizeAll:
        return UiMouseCursor::ResizeAll;
    case ImGuiMouseCursor_ResizeNS:
        return UiMouseCursor::ResizeNS;
    case ImGuiMouseCursor_ResizeEW:
        return UiMouseCursor::ResizeEW;
    case ImGuiMouseCursor_ResizeNESW:
        return UiMouseCursor::ResizeNESW;
    case ImGuiMouseCursor_ResizeNWSE:
        return UiMouseCursor::ResizeNWSE;
    case ImGuiMouseCursor_Hand:
        return UiMouseCursor::Hand;
    case ImGuiMouseCursor_NotAllowed:
        return UiMouseCursor::NotAllowed;
    default:
        break;
    }
    return UiMouseCursor::Arrow;
}

ImGuiTableBgTarget nativeTableBgTarget(UiTableBgTarget target) noexcept
{
    switch (target)
    {
    case UiTableBgTarget::None:
        return ImGuiTableBgTarget_None;
    case UiTableBgTarget::RowBg0:
        return ImGuiTableBgTarget_RowBg0;
    case UiTableBgTarget::RowBg1:
        return ImGuiTableBgTarget_RowBg1;
    case UiTableBgTarget::CellBg:
        return ImGuiTableBgTarget_CellBg;
    }
    return ImGuiTableBgTarget_None;
}

void applyStyle(UiStyle style) noexcept
{
    switch (style)
    {
    case UiStyle::Dark:
        ImGui::StyleColorsDark();
        break;
    case UiStyle::Light:
        ImGui::StyleColorsLight();
        break;
    case UiStyle::Classic:
        ImGui::StyleColorsClassic();
        break;
    }
}

} // namespace backend
} // namespace ui
} // namespace engine

#endif // ENGINE_UI_BACKEND_IMGUI
