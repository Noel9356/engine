// Popups, modal dialogs, menu bars, menu items, tooltips.
//
// Popup open/close pairs follow ImGui's contract: openPopup queues an open
// request that beginPopup consumes; closeCurrentPopup closes the popup that
// owns the current item. beginPopup*/endPopup must be paired even when the
// begin call returned false (ImGui tolerates the spurious endPopup).

#include "ui_backend.hpp"

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

void openPopup(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return;
    makeCurrent(*record);
    ImGui::OpenPopup(name, nativePopupFlags(flags));
}

void openPopupId(UiModuleState &state, UiContextHandle handle, std::uint32_t identifier, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::OpenPopup(static_cast<ImGuiID>(identifier), nativePopupFlags(flags));
}

bool beginPopup(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::BeginPopup(name, nativePopupFlags(flags));
}

bool beginPopupModal(UiModuleState &state, UiContextHandle handle, const char *name, bool *open, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::BeginPopupModal(name, open, nativePopupFlags(flags));
}

bool beginPopupContextItem(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::BeginPopupContextItem(name, nativePopupFlags(flags));
}

bool beginPopupContextWindow(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::BeginPopupContextWindow(name, nativePopupFlags(flags));
}

bool beginPopupContextVoid(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::BeginPopupContextVoid(name, nativePopupFlags(flags));
}

void endPopup(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndPopup();
}

void closeCurrentPopup(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::CloseCurrentPopup();
}

bool isPopupOpen(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::IsPopupOpen(name, nativePopupFlags(flags));
}

bool beginMenuBar(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::BeginMenuBar();
}

void endMenuBar(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndMenuBar();
}

bool beginMenu(UiModuleState &state, UiContextHandle handle, const char *label, bool enabled)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::BeginMenu(label, enabled);
}

void endMenu(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndMenu();
}

bool menuItem(UiModuleState &state, UiContextHandle handle, const char *label, const char *shortcut,
    bool *selected, bool enabled)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::MenuItem(label, shortcut, selected, enabled);
}

bool menuItemActivated(UiModuleState &state, UiContextHandle handle, const char *label, const char *shortcut,
    bool enabled)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::MenuItem(label, shortcut, false, enabled);
}

void setTooltip(UiModuleState &state, UiContextHandle handle, const char *value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || value == nullptr)
        return;
    makeCurrent(*record);
    ImGui::SetTooltip("%s", value);
}

void beginTooltip(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::BeginTooltip();
}

void endTooltip(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndTooltip();
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
