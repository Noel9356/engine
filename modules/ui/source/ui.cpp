// ui module implementation.
//
// Wraps Dear ImGui behind the UiCapability function-pointer table: contexts
// bound to a single owner thread, the beginFrame -> widgets -> endFrame
// lifecycle, and per-frame draw data copied into the ABI's plain structs.
// Without ENGINE_UI_BACKEND_IMGUI every entry fails with BackendUnavailable.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/ui/capabilities.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#if defined(ENGINE_UI_BACKEND_IMGUI)
#include <imgui.h>

#include <algorithm>
#include <climits>
#include <limits>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>
#endif

namespace engine
{
namespace ui
{
namespace
{

// Writes an error code through an optional out-parameter.
void setError(UiError *outError, UiError error)
{
    if (outError != nullptr)
    {
        *outError = error;
    }
}

#if defined(ENGINE_UI_BACKEND_IMGUI)

// One created context: the native ImGui context plus the draw-data buffers
// the ABI hands out. ownerThread is fixed at creation and checked on every
// call; nextFont hands out font handles (0 is reserved for the default font).
struct UiContextRecord
{
    ImGuiContext                              *native      = nullptr;
    std::thread::id                            ownerThread = std::this_thread::get_id();
    UiContextDescription                       description{};
    bool                                       frameActive = false;
    bool                                       touchPointerActive = false;
    std::uint32_t                              touchPointerId = 0;
    UiFontHandle                               nextFont    = 1;
    std::unordered_map<UiFontHandle, ImFont *> fonts;
    std::vector<UiVertex>                      vertices;
    std::vector<std::uint32_t>                 indices;
    std::vector<UiDrawCommand>                 commands;
    UiDrawData                                 drawData{};

    ~UiContextRecord()
    {
        if (native != nullptr)
        {
            ImGui::SetCurrentContext(native);
            if (frameActive)
            {
                ImGui::EndFrame();
            }
            ImGui::DestroyContext(native);
        }
    }
};

// Maps a UiKey to its ImGui counterpart; ImGuiKey_None for UiKey::None.
ImGuiKey nativeKey(UiKey key)
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

// Maps a gamepad button to its ImGui gamepad-key counterpart.
ImGuiKey nativeGamepadButton(UiGamepadButton button)
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

// Translates the ABI bit flags into ImGui window flags.
ImGuiWindowFlags nativeWindowFlags(UiWindowFlags flags)
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
    return result;
}

// Maps a style color slot to its ImGui color (Text as the fallback).
ImGuiCol nativeColor(UiColor color)
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
    case UiColor::CheckMark:
        return ImGuiCol_CheckMark;
    case UiColor::SliderGrab:
        return ImGuiCol_SliderGrab;
    case UiColor::SliderGrabActive:
        return ImGuiCol_SliderGrabActive;
    }
    return ImGuiCol_Text;
}

// Maps a style variable to its ImGui style-var counterpart.
ImGuiStyleVar nativeStyleVariable(UiStyleVariable variable)
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
    case UiStyleVariable::FrameRounding:
        return ImGuiStyleVar_FrameRounding;
    case UiStyleVariable::FrameBorderSize:
        return ImGuiStyleVar_FrameBorderSize;
    case UiStyleVariable::PopupRounding:
        return ImGuiStyleVar_PopupRounding;
    case UiStyleVariable::PopupBorderSize:
        return ImGuiStyleVar_PopupBorderSize;
    case UiStyleVariable::ScrollbarRounding:
        return ImGuiStyleVar_ScrollbarRounding;
    case UiStyleVariable::GrabRounding:
        return ImGuiStyleVar_GrabRounding;
    case UiStyleVariable::TabRounding:
        return ImGuiStyleVar_TabRounding;
    }
    return ImGuiStyleVar_Alpha;
}

#endif

// Module implementation behind the "ui" capability. All ImGui work happens
// on the thread that created the context; the module owns its contexts until
// stop() destroys them.
class UiModule final : public ModuleInterface
{
public:
    UiModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads [ui] save_window_layout (default false). Dear ImGui persists its
    // window layout by writing imgui.ini from inside NewFrame, rate-limited to
    // once every ~5 seconds - synchronous file I/O in the frame path that
    // surfaces as a frame-time spike. Debug-overlay layout persistence stays
    // off unless explicitly enabled.
    void construct(ParticipantContext &context) override
    {
        saveWindowLayout_ = context.configurationBooleanInSection("ui", "save_window_layout").value_or(false);
    }

    void start() override
    {}

    void stop() override
    {
        // ImGui state does not survive module restart: drop every context.
#if defined(ENGINE_UI_BACKEND_IMGUI)
        contexts_.clear();
        ownerThread_    = {};
        hasOwnerThread_ = false;
#endif
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        return capabilityName == UiCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name = "ui", .kind = ParticipantKind::EngineModule, .version = "0.1.0", .providedCapabilities = {"ui"}};
    }

    // Wires every capability function pointer to a lambda that forwards to
    // the matching *Impl method on this module.
    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_UI_BIND(field, method)                                                                                  \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<UiModule *>(self)->method(args...); }
        ENGINE_UI_BIND(createContext, createContextImpl);
        ENGINE_UI_BIND(destroyContext, destroyContextImpl);
        ENGINE_UI_BIND(beginFrame, beginFrameImpl);
        ENGINE_UI_BIND(endFrame, endFrameImpl);
        ENGINE_UI_BIND(drawData, drawDataImpl);
        ENGINE_UI_BIND(captureState, captureStateImpl);
        ENGINE_UI_BIND(addMousePosition, addMousePositionImpl);
        ENGINE_UI_BIND(addMouseButton, addMouseButtonImpl);
        ENGINE_UI_BIND(addTouch, addTouchImpl);
        ENGINE_UI_BIND(addMouseWheel, addMouseWheelImpl);
        ENGINE_UI_BIND(addKey, addKeyImpl);
        ENGINE_UI_BIND(addGamepadButton, addGamepadButtonImpl);
        ENGINE_UI_BIND(addGamepadAxis, addGamepadAxisImpl);
        ENGINE_UI_BIND(addText, addTextImpl);
        ENGINE_UI_BIND(addFocus, addFocusImpl);
        ENGINE_UI_BIND(beginWindow, beginWindowImpl);
        ENGINE_UI_BIND(endWindow, endWindowImpl);
        ENGINE_UI_BIND(text, textImpl);
        ENGINE_UI_BIND(textColored, textColoredImpl);
        ENGINE_UI_BIND(separator, separatorImpl);
        ENGINE_UI_BIND(sameLine, sameLineImpl);
        ENGINE_UI_BIND(spacing, spacingImpl);
        ENGINE_UI_BIND(button, buttonImpl);
        ENGINE_UI_BIND(checkbox, checkboxImpl);
        ENGINE_UI_BIND(sliderFloat, sliderFloatImpl);
        ENGINE_UI_BIND(inputText, inputTextImpl);
        ENGINE_UI_BIND(inputInt, inputIntImpl);
        ENGINE_UI_BIND(inputFloat, inputFloatImpl);
        ENGINE_UI_BIND(sliderInt, sliderIntImpl);
        ENGINE_UI_BIND(dragFloat, dragFloatImpl);
        ENGINE_UI_BIND(combo, comboImpl);
        ENGINE_UI_BIND(selectable, selectableImpl);
        ENGINE_UI_BIND(radioButton, radioButtonImpl);
        ENGINE_UI_BIND(progressBar, progressBarImpl);
        ENGINE_UI_BIND(image, imageImpl);
        ENGINE_UI_BIND(addFontFromFile, addFontFromFileImpl);
        ENGINE_UI_BIND(pushFont, pushFontImpl);
        ENGINE_UI_BIND(popFont, popFontImpl);
        ENGINE_UI_BIND(fontAtlas, fontAtlasImpl);
        ENGINE_UI_BIND(setFontTexture, setFontTextureImpl);
        ENGINE_UI_BIND(pushStyleColor, pushStyleColorImpl);
        ENGINE_UI_BIND(popStyleColor, popStyleColorImpl);
        ENGINE_UI_BIND(pushStyleVariable, pushStyleVariableImpl);
        ENGINE_UI_BIND(popStyleVariable, popStyleVariableImpl);
        ENGINE_UI_BIND(setNextWindowPosition, setNextWindowPositionImpl);
        ENGINE_UI_BIND(setNextWindowSize, setNextWindowSizeImpl);
#undef ENGINE_UI_BIND
    }

#if defined(ENGINE_UI_BACKEND_IMGUI)
    // Returns null for unknown handles and for handles owned by another
    // thread; callers then report InvalidHandle (createContext reports
    // WrongThread separately for the same rule).
    UiContextRecord *findContext(UiContextHandle handle)
    {
        const auto iterator = contexts_.find(handle);
        if (iterator == contexts_.end() || iterator->second->ownerThread != std::this_thread::get_id())
        {
            return nullptr;
        }
        return iterator->second.get();
    }

    static bool inFrame(const UiContextRecord *context)
    {
        return context != nullptr && context->frameActive;
    }

    static void makeCurrent(UiContextRecord &context)
    {
        ImGui::SetCurrentContext(context.native);
    }

    // Resets the per-frame buffers; called before NewFrame and on failures.
    static void clearDrawData(UiContextRecord &context)
    {
        context.vertices.clear();
        context.indices.clear();
        context.commands.clear();
        context.drawData = {};
    }

    // Flattens ImGui's draw lists into the ABI structs, rebasing per-command
    // offsets across lists and dropping commands with custom callbacks (only
    // the reset-render-state marker survives). Totals above the description's
    // maximums clear the buffers and fail the frame.
    static bool copyDrawData(UiContextRecord &context)
    {
        const ImDrawData *source = ImGui::GetDrawData();
        if (source == nullptr || !source->Valid)
        {
            clearDrawData(context);
            return true;
        }

        std::uint64_t vertexCount  = 0;
        std::uint64_t indexCount   = 0;
        std::uint64_t commandCount = 0;
        for (int listIndex = 0; listIndex < source->CmdListsCount; ++listIndex)
        {
            const ImDrawList *list = source->CmdLists[listIndex];
            vertexCount += static_cast<std::uint64_t>(list->VtxBuffer.Size);
            indexCount += static_cast<std::uint64_t>(list->IdxBuffer.Size);
            commandCount += static_cast<std::uint64_t>(list->CmdBuffer.Size);
        }

        if (vertexCount > context.description.maximumVertices || indexCount > context.description.maximumIndices
            || commandCount > context.description.maximumCommands
            || vertexCount > std::numeric_limits<std::uint32_t>::max()
            || indexCount > std::numeric_limits<std::uint32_t>::max()
            || commandCount > std::numeric_limits<std::uint32_t>::max())
        {
            clearDrawData(context);
            return false;
        }

        context.vertices.clear();
        context.indices.clear();
        context.commands.clear();
        context.vertices.reserve(static_cast<std::size_t>(vertexCount));
        context.indices.reserve(static_cast<std::size_t>(indexCount));
        context.commands.reserve(static_cast<std::size_t>(commandCount));

        std::uint32_t vertexBase = 0;
        std::uint32_t indexBase  = 0;
        for (int listIndex = 0; listIndex < source->CmdListsCount; ++listIndex)
        {
            const ImDrawList *list = source->CmdLists[listIndex];
            for (const ImDrawVert &vertex : list->VtxBuffer)
            {
                UiVertex output{};
                output.position[0] = vertex.pos.x;
                output.position[1] = vertex.pos.y;
                output.uv[0]       = vertex.uv.x;
                output.uv[1]       = vertex.uv.y;
                output.color       = vertex.col;
                context.vertices.push_back(output);
            }
            for (const ImDrawIdx index : list->IdxBuffer)
            {
                context.indices.push_back(static_cast<std::uint32_t>(index));
            }
            for (const ImDrawCmd &command : list->CmdBuffer)
            {
                if (command.UserCallback != nullptr && command.UserCallback != ImDrawCallback_ResetRenderState)
                {
                    continue;
                }
                UiDrawCommand output{};
                output.indexOffset      = indexBase + command.IdxOffset;
                output.vertexOffset     = vertexBase + command.VtxOffset;
                output.elementCount     = command.ElemCount;
                output.clipRectangle[0] = command.ClipRect.x;
                output.clipRectangle[1] = command.ClipRect.y;
                output.clipRectangle[2] = command.ClipRect.z;
                output.clipRectangle[3] = command.ClipRect.w;
                output.texture          = static_cast<UiTextureHandle>(command.TextureId);
                output.resetRenderState = command.UserCallback == ImDrawCallback_ResetRenderState;
                context.commands.push_back(output);
            }
            vertexBase += static_cast<std::uint32_t>(list->VtxBuffer.Size);
            indexBase += static_cast<std::uint32_t>(list->IdxBuffer.Size);
        }

        context.drawData.vertices            = context.vertices.empty() ? nullptr : context.vertices.data();
        context.drawData.vertexCount         = static_cast<std::uint32_t>(context.vertices.size());
        context.drawData.indices             = context.indices.empty() ? nullptr : context.indices.data();
        context.drawData.indexCount          = static_cast<std::uint32_t>(context.indices.size());
        context.drawData.commands            = context.commands.empty() ? nullptr : context.commands.data();
        context.drawData.commandCount        = static_cast<std::uint32_t>(context.commands.size());
        context.drawData.displayPosition[0]  = source->DisplayPos.x;
        context.drawData.displayPosition[1]  = source->DisplayPos.y;
        context.drawData.displaySize[0]      = source->DisplaySize.x;
        context.drawData.displaySize[1]      = source->DisplaySize.y;
        context.drawData.framebufferScale[0] = source->FramebufferScale.x;
        context.drawData.framebufferScale[1] = source->FramebufferScale.y;
        return true;
    }
#endif

    // Creates a context. A null description uses the defaults; zero-valued
    // reserve/maximum fields fall back to the same defaults; reserve above
    // maximum is rejected. The first context fixes the owner thread for all.
    UiContextHandle createContextImpl(const UiContextDescription *description, UiError *error)
    {
        setError(error, UiError::None);
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextDescription resolved{};
        if (description != nullptr)
        {
            resolved = *description;
        }
        const std::thread::id currentThread = std::this_thread::get_id();
        if (hasOwnerThread_ && ownerThread_ != currentThread)
        {
            setError(error, UiError::WrongThread);
            return invalidUiContext;
        }
        if (!std::isfinite(resolved.globalScale) || resolved.globalScale <= 0.0f)
        {
            setError(error, UiError::InvalidDescription);
            return invalidUiContext;
        }
        if (resolved.reserveVertices == 0)
            resolved.reserveVertices = 4096;
        if (resolved.reserveIndices == 0)
            resolved.reserveIndices = 8192;
        if (resolved.reserveCommands == 0)
            resolved.reserveCommands = 256;
        if (resolved.maximumVertices == 0)
            resolved.maximumVertices = 1'048'576;
        if (resolved.maximumIndices == 0)
            resolved.maximumIndices = 2'097'152;
        if (resolved.maximumCommands == 0)
            resolved.maximumCommands = 65'536;
        if (resolved.reserveVertices > resolved.maximumVertices || resolved.reserveIndices > resolved.maximumIndices
            || resolved.reserveCommands > resolved.maximumCommands)
        {
            setError(error, UiError::InvalidDescription);
            return invalidUiContext;
        }
        auto record         = std::make_unique<UiContextRecord>();
        record->ownerThread = currentThread;
        record->description = resolved;
        record->vertices.reserve(resolved.reserveVertices);
        record->indices.reserve(resolved.reserveIndices);
        record->commands.reserve(resolved.reserveCommands);
        record->native = ImGui::CreateContext();
        if (record->native == nullptr)
        {
            setError(error, UiError::CreationFailed);
            return invalidUiContext;
        }
        makeCurrent(*record);
        IMGUI_CHECKVERSION();
        ImGui::StyleColorsDark();
        ImGui::GetStyle().ScaleAllSizes(resolved.globalScale);
        ImGuiIO &io        = ImGui::GetIO();
        io.FontGlobalScale = resolved.globalScale;
        if (resolved.keyboardNavigation)
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        if (resolved.gamepadNavigation)
        {
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
            io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
        }
        io.BackendPlatformName = "engine_ui_input";
        io.BackendRendererName = "engine_ui_draw_data";
        io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
        // Null disables the automatic ini load/save; without it ImGui writes
        // imgui.ini from NewFrame every ~5s after any window change, which is
        // a synchronous file write inside the frame (frame-time spike).
        if (!saveWindowLayout_)
            io.IniFilename = nullptr;
        record->fonts.emplace(defaultUiFont, io.Fonts->AddFontDefault());

        // Handles start at 1; skip 0 (invalidUiContext) if the counter wraps.
        const UiContextHandle handle = nextContext_++;
        if (handle == invalidUiContext)
        {
            setError(error, UiError::CreationFailed);
            return invalidUiContext;
        }
        try
        {
            const auto [iterator, inserted] = contexts_.emplace(handle, std::move(record));
            (void)iterator;
            if (!inserted)
            {
                setError(error, UiError::CreationFailed);
                return invalidUiContext;
            }
        }
        catch (...)
        {
            setError(error, UiError::CreationFailed);
            return invalidUiContext;
        }
        ownerThread_    = currentThread;
        hasOwnerThread_ = true;
        return handle;
#else
        (void)description;
        setError(error, UiError::BackendUnavailable);
        return invalidUiContext;
#endif
    }

    void destroyContextImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        const auto iterator = contexts_.find(handle);
        if (iterator == contexts_.end() || iterator->second->ownerThread != std::this_thread::get_id())
            return;
        contexts_.erase(iterator);
        if (contexts_.empty())
        {
            ownerThread_    = {};
            hasOwnerThread_ = false;
        }
#else
        (void)handle;
#endif
    }

    // Starts a frame: validates the description (positive finite sizes and
    // delta), clamps deltaSeconds to [1 ms, 0.25 s] for ImGui's timers, clears
    // the previous frame's draw data, and calls ImGui::NewFrame().
    bool beginFrameImpl(UiContextHandle handle, const UiFrameDescription *description, UiError *error)
    {
        setError(error, UiError::None);
#if defined(ENGINE_UI_BACKEND_IMGUI)
        if (description == nullptr || !std::isfinite(description->displayWidth)
            || !std::isfinite(description->displayHeight) || description->displayWidth <= 0.0f
            || description->displayHeight <= 0.0f || !std::isfinite(description->framebufferScaleX)
            || !std::isfinite(description->framebufferScaleY) || description->framebufferScaleX <= 0.0f
            || description->framebufferScaleY <= 0.0f || !std::isfinite(description->deltaSeconds)
            || description->deltaSeconds <= 0.0f)
        {
            setError(error, UiError::InvalidDescription);
            return false;
        }
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
        {
            setError(error, UiError::InvalidHandle);
            return false;
        }
        if (record->frameActive)
        {
            setError(error, UiError::FrameAlreadyActive);
            return false;
        }
        makeCurrent(*record);
        ImGuiIO &io                = ImGui::GetIO();
        io.DisplaySize             = ImVec2(description->displayWidth, description->displayHeight);
        io.DisplayFramebufferScale = ImVec2(description->framebufferScaleX, description->framebufferScaleY);
        io.DeltaTime               = std::clamp(description->deltaSeconds, 1.0f / 1000.0f, 0.25f);
        clearDrawData(*record);
        ImGui::NewFrame();
        record->frameActive = true;
        return true;
#else
        (void)handle;
        (void)description;
        setError(error, UiError::BackendUnavailable);
        return false;
#endif
    }

    // Ends the frame: ImGui::Render() produces the draw lists, which are
    // copied into the ABI buffers. CapacityExceeded (with cleared draw data)
    // is reported when a configured maximum was exceeded.
    bool endFrameImpl(UiContextHandle handle, UiError *error)
    {
        setError(error, UiError::None);
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
        {
            setError(error, UiError::InvalidHandle);
            return false;
        }
        if (!record->frameActive)
        {
            setError(error, UiError::FrameNotActive);
            return false;
        }
        makeCurrent(*record);
        ImGui::Render();
        record->frameActive = false;
        if (!copyDrawData(*record))
        {
            setError(error, UiError::CapacityExceeded);
            return false;
        }
        return true;
#else
        (void)handle;
        setError(error, UiError::BackendUnavailable);
        return false;
#endif
    }

    // Draw data of the last ended frame; the pointer lives until the next
    // beginFrame on the same context.
    const UiDrawData *drawDataImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        return record != nullptr ? &record->drawData : nullptr;
#else
        (void)handle;
        return nullptr;
#endif
    }

    bool captureStateImpl(UiContextHandle handle, UiCaptureState *outState)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        if (outState == nullptr)
            return false;
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
            return false;
        makeCurrent(*record);
        const ImGuiIO &io   = ImGui::GetIO();
        outState->mouse     = io.WantCaptureMouse;
        outState->keyboard  = io.WantCaptureKeyboard;
        outState->textInput = io.WantTextInput;
        return true;
#else
        (void)handle;
        (void)outState;
        return false;
#endif
    }

    // Input sink: events are queued into ImGui's IO and take effect on the
    // next NewFrame. Non-finite or out-of-range input is silently dropped.
    void addMousePositionImpl(UiContextHandle handle, float x, float y)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || !std::isfinite(x) || !std::isfinite(y))
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddMousePosEvent(x, y);
#else
        (void)handle;
        (void)x;
        (void)y;
#endif
    }

    void addMouseButtonImpl(UiContextHandle handle, std::uint32_t button, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || button >= ImGuiMouseButton_COUNT)
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddMouseButtonEvent(static_cast<int>(button), down);
#else
        (void)handle;
        (void)button;
        (void)down;
#endif
    }

    void addTouchImpl(UiContextHandle handle, std::uint32_t touchId, UiTouchPhase phase, float x, float y)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || static_cast<std::uint32_t>(phase) > static_cast<std::uint32_t>(UiTouchPhase::Cancelled)
            || !std::isfinite(x) || !std::isfinite(y))
            return;
        makeCurrent(*record);
        ImGuiIO &io = ImGui::GetIO();
        switch (phase)
        {
        case UiTouchPhase::Began:
            if (record->touchPointerActive)
                return;
            record->touchPointerActive = true;
            record->touchPointerId = touchId;
            io.AddMousePosEvent(x, y);
            io.AddMouseButtonEvent(0, true);
            break;
        case UiTouchPhase::Moved:
            if (record->touchPointerActive && record->touchPointerId == touchId)
                io.AddMousePosEvent(x, y);
            break;
        case UiTouchPhase::Ended:
        case UiTouchPhase::Cancelled:
            if (record->touchPointerActive && record->touchPointerId == touchId)
            {
                io.AddMousePosEvent(x, y);
                io.AddMouseButtonEvent(0, false);
                record->touchPointerActive = false;
            }
            break;
        }
#else
        (void)handle;
        (void)touchId;
        (void)phase;
        (void)x;
        (void)y;
#endif
    }

    void addMouseWheelImpl(UiContextHandle handle, float horizontal, float vertical)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || !std::isfinite(horizontal) || !std::isfinite(vertical))
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddMouseWheelEvent(horizontal, vertical);
#else
        (void)handle;
        (void)horizontal;
        (void)vertical;
#endif
    }

    void addKeyImpl(UiContextHandle handle, UiKey key, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        const ImGuiKey   mapped = nativeKey(key);
        if (record == nullptr || mapped == ImGuiKey_None)
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddKeyEvent(mapped, down);
#else
        (void)handle;
        (void)key;
        (void)down;
#endif
    }

    void addGamepadButtonImpl(UiContextHandle handle, UiGamepadButton button, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        const ImGuiKey   mapped = nativeGamepadButton(button);
        if (record == nullptr || mapped == ImGuiKey_None)
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddKeyEvent(mapped, down);
#else
        (void)handle;
        (void)button;
        (void)down;
#endif
    }

    // Analog axes are split into ImGui's positive/negative directional keys;
    // the value is clamped to [-1, 1] and triggers only drive the positive
    // key.
    void addGamepadAxisImpl(UiContextHandle handle, UiGamepadAxis axis, float value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || !std::isfinite(value))
            return;
        makeCurrent(*record);
        ImGuiIO    &io       = ImGui::GetIO();
        const float amount   = std::clamp(value, -1.0f, 1.0f);
        const auto  sendAxis = [&io](ImGuiKey positive, ImGuiKey negative, float position)
        {
            io.AddKeyAnalogEvent(positive, position > 0.0f, std::max(position, 0.0f));
            io.AddKeyAnalogEvent(negative, position < 0.0f, std::max(-position, 0.0f));
        };
        switch (axis)
        {
        case UiGamepadAxis::LeftStickX:
            sendAxis(ImGuiKey_GamepadLStickRight, ImGuiKey_GamepadLStickLeft, amount);
            break;
        case UiGamepadAxis::LeftStickY:
            sendAxis(ImGuiKey_GamepadLStickDown, ImGuiKey_GamepadLStickUp, amount);
            break;
        case UiGamepadAxis::RightStickX:
            sendAxis(ImGuiKey_GamepadRStickRight, ImGuiKey_GamepadRStickLeft, amount);
            break;
        case UiGamepadAxis::RightStickY:
            sendAxis(ImGuiKey_GamepadRStickDown, ImGuiKey_GamepadRStickUp, amount);
            break;
        case UiGamepadAxis::LeftTrigger:
            io.AddKeyAnalogEvent(ImGuiKey_GamepadL2, amount > 0.0f, std::max(amount, 0.0f));
            break;
        case UiGamepadAxis::RightTrigger:
            io.AddKeyAnalogEvent(ImGuiKey_GamepadR2, amount > 0.0f, std::max(amount, 0.0f));
            break;
        }
#else
        (void)handle;
        (void)axis;
        (void)value;
#endif
    }

    // Text input as a Unicode codepoint; surrogates and values above
    // 0x10FFFF are rejected.
    void addTextImpl(UiContextHandle handle, std::uint32_t codepoint)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || codepoint == 0 || codepoint > 0x10FFFFu
            || (codepoint >= 0xD800u && codepoint <= 0xDFFFu))
            return;
        makeCurrent(*record);
        ImGui::GetIO().AddInputCharacter(codepoint);
#else
        (void)handle;
        (void)codepoint;
#endif
    }

    void addFocusImpl(UiContextHandle handle, bool focused)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
            return;
        makeCurrent(*record);
        ImGuiIO &io = ImGui::GetIO();
        if (!focused && record->touchPointerActive)
        {
            io.AddMouseButtonEvent(0, false);
            record->touchPointerActive = false;
        }
        io.AddFocusEvent(focused);
#else
        (void)handle;
        (void)focused;
#endif
    }

    // Begins an immediate-mode window. Returns ImGui::Begin's result: false
    // means collapsed or closed, but endWindow must still be called.
    bool beginWindowImpl(UiContextHandle handle, const char *name, bool *open, UiWindowFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || name == nullptr)
            return false;
        makeCurrent(*record);
        return ImGui::Begin(name, open, nativeWindowFlags(flags));
#else
        (void)handle;
        (void)name;
        (void)open;
        (void)flags;
        return false;
#endif
    }

    void endWindowImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return;
        makeCurrent(*record);
        ImGui::End();
#else
        (void)handle;
#endif
    }

    void textImpl(UiContextHandle handle, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || value == nullptr)
            return;
        makeCurrent(*record);
        ImGui::TextUnformatted(value);
#else
        (void)handle;
        (void)value;
#endif
    }

    void textColoredImpl(UiContextHandle handle, float red, float green, float blue, float alpha, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || value == nullptr || !std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue)
            || !std::isfinite(alpha))
            return;
        makeCurrent(*record);
        ImGui::TextColored(ImVec4(red, green, blue, alpha), "%s", value);
#else
        (void)handle;
        (void)red;
        (void)green;
        (void)blue;
        (void)alpha;
        (void)value;
#endif
    }

    void separatorImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return;
        makeCurrent(*record);
        ImGui::Separator();
#else
        (void)handle;
#endif
    }

    void sameLineImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return;
        makeCurrent(*record);
        ImGui::SameLine();
#else
        (void)handle;
#endif
    }

    void spacingImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return;
        makeCurrent(*record);
        ImGui::Spacing();
#else
        (void)handle;
#endif
    }

    bool buttonImpl(UiContextHandle handle, const char *label, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || !std::isfinite(width) || !std::isfinite(height))
            return false;
        makeCurrent(*record);
        return ImGui::Button(label, ImVec2(width, height));
#else
        (void)handle;
        (void)label;
        (void)width;
        (void)height;
        return false;
#endif
    }

    bool checkboxImpl(UiContextHandle handle, const char *label, bool *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr)
            return false;
        makeCurrent(*record);
        return ImGui::Checkbox(label, value);
#else
        (void)handle;
        (void)label;
        (void)value;
        return false;
#endif
    }

    bool sliderFloatImpl(
        UiContextHandle handle, const char *label, float *value, float minimum, float maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(minimum)
            || !std::isfinite(maximum) || minimum > maximum)
            return false;
        makeCurrent(*record);
        return ImGui::SliderFloat(label, value, minimum, maximum, format != nullptr ? format : "%.3f");
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool inputTextImpl(UiContextHandle handle, const char *label, char *buffer, std::uint32_t bufferSize)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || buffer == nullptr || bufferSize == 0)
            return false;
        makeCurrent(*record);
        return ImGui::InputText(label, buffer, static_cast<std::size_t>(bufferSize));
#else
        (void)handle;
        (void)label;
        (void)buffer;
        (void)bufferSize;
        return false;
#endif
    }

    bool inputIntImpl(UiContextHandle handle, const char *label, std::int32_t *value,
        std::int32_t step, std::int32_t fastStep)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr || step < 0 || fastStep < 0)
            return false;
        makeCurrent(*record);
        int nativeValue = static_cast<int>(*value);
        const bool changed = ImGui::InputInt(label, &nativeValue, static_cast<int>(step), static_cast<int>(fastStep));
        *value = static_cast<std::int32_t>(nativeValue);
        return changed;
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)step;
        (void)fastStep;
        return false;
#endif
    }

    bool inputFloatImpl(UiContextHandle handle, const char *label, float *value, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(*value))
            return false;
        makeCurrent(*record);
        return ImGui::InputFloat(label, value, 0.0f, 0.0f, format != nullptr ? format : "%.3f");
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)format;
        return false;
#endif
    }

    bool sliderIntImpl(UiContextHandle handle, const char *label, std::int32_t *value,
        std::int32_t minimum, std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr || minimum > maximum)
            return false;
        makeCurrent(*record);
        int nativeValue = static_cast<int>(*value);
        const bool changed = ImGui::SliderInt(label, &nativeValue, static_cast<int>(minimum),
            static_cast<int>(maximum), format != nullptr ? format : "%d");
        *value = static_cast<std::int32_t>(nativeValue);
        return changed;
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool dragFloatImpl(UiContextHandle handle, const char *label, float *value, float speed,
        float minimum, float maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(*value)
            || !std::isfinite(speed) || speed < 0.0f || !std::isfinite(minimum)
            || !std::isfinite(maximum) || minimum >= maximum)
            return false;
        makeCurrent(*record);
        return ImGui::DragFloat(label, value, speed, minimum, maximum, format != nullptr ? format : "%.3f");
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)speed;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool comboImpl(UiContextHandle handle, const char *label, std::int32_t *currentItem,
        const char *const *items, std::uint32_t itemCount)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || currentItem == nullptr || items == nullptr
            || itemCount == 0 || itemCount > static_cast<std::uint32_t>(INT_MAX)
            || *currentItem < -1 || (*currentItem >= 0 && static_cast<std::uint32_t>(*currentItem) >= itemCount))
            return false;
        for (std::uint32_t index = 0; index < itemCount; ++index)
            if (items[index] == nullptr)
                return false;
        makeCurrent(*record);
        int nativeCurrentItem = static_cast<int>(*currentItem);
        const bool changed = ImGui::Combo(label, &nativeCurrentItem, items, static_cast<int>(itemCount));
        *currentItem = static_cast<std::int32_t>(nativeCurrentItem);
        return changed;
#else
        (void)handle;
        (void)label;
        (void)currentItem;
        (void)items;
        (void)itemCount;
        return false;
#endif
    }

    bool selectableImpl(UiContextHandle handle, const char *label, bool *selected, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || selected == nullptr
            || !std::isfinite(width) || !std::isfinite(height))
            return false;
        makeCurrent(*record);
        const bool activated = ImGui::Selectable(label, *selected, ImGuiSelectableFlags_None, ImVec2(width, height));
        if (activated)
            *selected = !*selected;
        return activated;
#else
        (void)handle;
        (void)label;
        (void)selected;
        (void)width;
        (void)height;
        return false;
#endif
    }

    bool radioButtonImpl(UiContextHandle handle, const char *label, std::int32_t *value, std::int32_t buttonValue)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || label == nullptr || value == nullptr)
            return false;
        makeCurrent(*record);
        int nativeValue = static_cast<int>(*value);
        const bool changed = ImGui::RadioButton(label, &nativeValue, static_cast<int>(buttonValue));
        *value = static_cast<std::int32_t>(nativeValue);
        return changed;
#else
        (void)handle;
        (void)label;
        (void)value;
        (void)buttonValue;
        return false;
#endif
    }

    void progressBarImpl(UiContextHandle handle, float fraction, float width, float height, const char *overlay)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(fraction) || !std::isfinite(width) || !std::isfinite(height))
            return;
        makeCurrent(*record);
        ImGui::ProgressBar(std::clamp(fraction, 0.0f, 1.0f), ImVec2(width, height), overlay);
#else
        (void)handle;
        (void)fraction;
        (void)width;
        (void)height;
        (void)overlay;
#endif
    }

    void imageImpl(UiContextHandle handle, UiTextureHandle texture, float width, float height, float uv0x, float uv0y,
        float uv1x, float uv1y)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(width) || !std::isfinite(height))
            return;
        makeCurrent(*record);
        ImGui::Image(static_cast<ImTextureID>(texture), ImVec2(width, height), ImVec2(uv0x, uv0y), ImVec2(uv1x, uv1y));
#else
        (void)handle;
        (void)texture;
        (void)width;
        (void)height;
        (void)uv0x;
        (void)uv0y;
        (void)uv1x;
        (void)uv1y;
#endif
    }

    // Loads a TTF font; only valid outside an active frame because it builds
    // the font atlas. Fails when the file cannot be loaded or built.
    bool addFontFromFileImpl(UiContextHandle handle, const char *filePath, float pixelSize, UiFontHandle *outFont)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr || record->frameActive || filePath == nullptr || filePath[0] == '\0'
            || !std::isfinite(pixelSize) || pixelSize <= 0.0f || outFont == nullptr)
            return false;
        makeCurrent(*record);
        ImFont *font = ImGui::GetIO().Fonts->AddFontFromFileTTF(filePath, pixelSize);
        if (font == nullptr || !ImGui::GetIO().Fonts->Build())
            return false;
        const UiFontHandle fontHandle = record->nextFont++;
        if (fontHandle == defaultUiFont)
            return false;
        record->fonts.emplace(fontHandle, font);
        *outFont = fontHandle;
        return true;
#else
        (void)handle;
        (void)filePath;
        (void)pixelSize;
        (void)outFont;
        return false;
#endif
    }

    bool pushFontImpl(UiContextHandle handle, UiFontHandle fontHandle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return false;
        const auto font = record->fonts.find(fontHandle);
        if (font == record->fonts.end())
            return false;
        makeCurrent(*record);
        ImGui::PushFont(font->second);
        return true;
#else
        (void)handle;
        (void)fontHandle;
        return false;
#endif
    }

    void popFontImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record))
            return;
        makeCurrent(*record);
        ImGui::PopFont();
#else
        (void)handle;
#endif
    }

    // RGBA pixels of the built font atlas (owned by ImGui, see UiFontAtlas).
    bool fontAtlasImpl(UiContextHandle handle, UiFontAtlas *outAtlas)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        if (outAtlas == nullptr)
            return false;
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
            return false;
        makeCurrent(*record);
        unsigned char *pixels        = nullptr;
        int            width         = 0;
        int            height        = 0;
        int            bytesPerPixel = 0;
        ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height, &bytesPerPixel);
        if (pixels == nullptr || width <= 0 || height <= 0 || bytesPerPixel != 4)
            return false;
        outAtlas->pixels        = pixels;
        outAtlas->width         = static_cast<std::uint32_t>(width);
        outAtlas->height        = static_cast<std::uint32_t>(height);
        outAtlas->bytesPerPixel = static_cast<std::uint32_t>(bytesPerPixel);
        return true;
#else
        (void)handle;
        (void)outAtlas;
        return false;
#endif
    }

    bool setFontTextureImpl(UiContextHandle handle, UiTextureHandle texture)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (record == nullptr)
            return false;
        makeCurrent(*record);
        ImGui::GetIO().Fonts->SetTexID(static_cast<ImTextureID>(texture));
        return true;
#else
        (void)handle;
        (void)texture;
        return false;
#endif
    }

    bool pushStyleColorImpl(UiContextHandle handle, UiColor color, float red, float green, float blue, float alpha)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue)
            || !std::isfinite(alpha))
            return false;
        makeCurrent(*record);
        ImGui::PushStyleColor(nativeColor(color), ImVec4(red, green, blue, alpha));
        return true;
#else
        (void)handle;
        (void)color;
        (void)red;
        (void)green;
        (void)blue;
        (void)alpha;
        return false;
#endif
    }

    void popStyleColorImpl(UiContextHandle handle, std::uint32_t count)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || count == 0)
            return;
        makeCurrent(*record);
        ImGui::PopStyleColor(static_cast<int>(std::min<std::uint32_t>(count, INT_MAX)));
#else
        (void)handle;
        (void)count;
#endif
    }

    bool pushStyleVariableImpl(UiContextHandle handle, UiStyleVariable variable, float value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(value))
            return false;
        makeCurrent(*record);
        ImGui::PushStyleVar(nativeStyleVariable(variable), value);
        return true;
#else
        (void)handle;
        (void)variable;
        (void)value;
        return false;
#endif
    }

    void popStyleVariableImpl(UiContextHandle handle, std::uint32_t count)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || count == 0)
            return;
        makeCurrent(*record);
        ImGui::PopStyleVar(static_cast<int>(std::min<std::uint32_t>(count, INT_MAX)));
#else
        (void)handle;
        (void)count;
#endif
    }

    void setNextWindowPositionImpl(UiContextHandle handle, float x, float y, float pivotX, float pivotY)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pivotX)
            || !std::isfinite(pivotY))
            return;
        makeCurrent(*record);
        ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always, ImVec2(pivotX, pivotY));
#else
        (void)handle;
        (void)x;
        (void)y;
        (void)pivotX;
        (void)pivotY;
#endif
    }

    void setNextWindowSizeImpl(UiContextHandle handle, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        UiContextRecord *record = findContext(handle);
        if (!inFrame(record) || !std::isfinite(width) || !std::isfinite(height))
            return;
        makeCurrent(*record);
        ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
#else
        (void)handle;
        (void)width;
        (void)height;
#endif
    }

    ParticipantManifest manifest_;
    UiCapability        capability_{};
    // See construct(): false keeps ImGui from touching imgui.ini in NewFrame.
    bool saveWindowLayout_ = false;
#if defined(ENGINE_UI_BACKEND_IMGUI)
    // ownerThread_ pins every context to one thread (creation and use).
    // nextContext_ hands out monotonic handles starting at 1.
    std::unordered_map<UiContextHandle, std::unique_ptr<UiContextRecord>> contexts_;
    std::thread::id                                                       ownerThread_{};
    bool                                                                  hasOwnerThread_ = false;
    UiContextHandle                                                       nextContext_    = 1;
#endif
};

} // namespace
} // namespace ui

} // namespace engine

// Module entry point queried by the kernel after dlopen: provides "ui" and
// declares no required/optional capabilities or permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    providedCapabilities[] = {"ui"};
    static const engine::EngineModuleEntryTable entry                  = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "ui",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface                             *{ return new engine::ui::UiModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
