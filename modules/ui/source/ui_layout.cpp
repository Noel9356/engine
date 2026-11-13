// Layout primitives: child windows, groups, indent, dummy, tree nodes,
// collapsing headers, and disabled sections.
//
// All calls guard on inFrame; layout calls outside an active frame are
// silently ignored (no ImGui state is mutated). Tree-node pop must pair with
// every treeNode/treeNodeEx that returned true; collapsingHeader never needs
// a pop because it sets the NoTreePushOnOpen flag internally.

#include "ui_backend.hpp"

#include <cmath>
#include <cstdint>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

bool beginChild(UiModuleState &state, UiContextHandle handle, const char *name, float width, float height,
    UiChildFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr || !std::isfinite(width) || !std::isfinite(height))
        return false;
    makeCurrent(*record);
    return ImGui::BeginChild(name, ImVec2(width, height), nativeChildFlags(flags));
}

void endChild(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndChild();
}

void beginGroup(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::BeginGroup();
}

void endGroup(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndGroup();
}

void indent(UiModuleState &state, UiContextHandle handle, float width)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(width))
        return;
    makeCurrent(*record);
    ImGui::Indent(width);
}

void unindent(UiModuleState &state, UiContextHandle handle, float width)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(width))
        return;
    makeCurrent(*record);
    ImGui::Unindent(width);
}

bool treeNode(UiModuleState &state, UiContextHandle handle, const char *label)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::TreeNode(label);
}

bool treeNodeEx(UiModuleState &state, UiContextHandle handle, const char *label, UiTreeNodeFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::TreeNodeEx(label, nativeTreeNodeFlags(flags));
}

void treePop(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::TreePop();
}

void setNextItemOpen(UiModuleState &state, UiContextHandle handle, bool open)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::SetNextItemOpen(open, ImGuiCond_Always);
}

bool collapsingHeader(UiModuleState &state, UiContextHandle handle, const char *label, UiTreeNodeFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::CollapsingHeader(label, nativeTreeNodeFlags(flags));
}

bool collapsingHeaderBool(UiModuleState &state, UiContextHandle handle, const char *label, bool *open,
    UiTreeNodeFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || open == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::CollapsingHeader(label, open, nativeTreeNodeFlags(flags));
}

void beginDisabled(UiModuleState &state, UiContextHandle handle, bool disabled)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::BeginDisabled(disabled);
}

void endDisabled(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndDisabled();
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
