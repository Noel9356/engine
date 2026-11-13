// Tab bars and tables.
//
// Tab bars: beginTabBar/endTabBar bracket a strip; each tab is
// beginTabItem/endTabItem (always paired, even when begin returned false).
// Tables: beginTable/endTable bracket a table; rows advance with
// tableNextRow, columns with tableNextColumn or tableSetColumnIndex;
// tableSetupColumn configures each column before the first row.

#include "ui_backend.hpp"

#include <cstdint>
#include <limits>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

bool beginTabBar(UiModuleState &state, UiContextHandle handle, const char *name, UiTabBarFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::BeginTabBar(name, nativeTabBarFlags(flags));
}

void endTabBar(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndTabBar();
}

bool beginTabItem(UiModuleState &state, UiContextHandle handle, const char *label, bool *open, UiTabItemFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::BeginTabItem(label, open, nativeTabItemFlags(flags));
}

void endTabItem(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndTabItem();
}

void setTabItemClosed(UiModuleState &state, UiContextHandle handle, const char *label)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return;
    makeCurrent(*record);
    ImGui::SetTabItemClosed(label);
}

bool beginTable(UiModuleState &state, UiContextHandle handle, const char *name, std::uint32_t columns,
    UiTableFlags flags, float outerSizeX, float outerSizeY, float innerWidth)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr || columns == 0
        || columns > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        return false;
    makeCurrent(*record);
    return ImGui::BeginTable(name, static_cast<int>(columns), nativeTableFlags(flags),
        ImVec2(outerSizeX, outerSizeY), innerWidth);
}

void endTable(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::EndTable();
}

void tableNextRow(UiModuleState &state, UiContextHandle handle, UiTableRowFlags flags, float minRowHeight)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::TableNextRow(nativeTableRowFlags(flags), minRowHeight);
}

bool tableNextColumn(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::TableNextColumn();
}

bool tableSetColumnIndex(UiModuleState &state, UiContextHandle handle, std::uint32_t columnIndex)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::TableSetColumnIndex(static_cast<int>(columnIndex));
}

void tableSetupColumn(UiModuleState &state, UiContextHandle handle, const char *label, UiTableColumnFlags flags,
    float initWidthOrWeight, std::uint32_t identifier)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return;
    makeCurrent(*record);
    ImGui::TableSetupColumn(label, nativeTableColumnFlags(flags), initWidthOrWeight,
        static_cast<ImGuiID>(identifier));
}

void tableSetupScrollFreeze(UiModuleState &state, UiContextHandle handle, std::uint32_t columns, std::uint32_t rows)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::TableSetupScrollFreeze(static_cast<int>(columns), static_cast<int>(rows));
}

void tableHeadersRow(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::TableHeadersRow();
}

void tableHeader(UiModuleState &state, UiContextHandle handle, const char *label)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr)
        return;
    makeCurrent(*record);
    ImGui::TableHeader(label);
}

std::int32_t tableColumnCount(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0;
    makeCurrent(*record);
    return ImGui::TableGetColumnCount();
}

std::int32_t tableColumnIndex(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0;
    makeCurrent(*record);
    return ImGui::TableGetColumnIndex();
}

std::int32_t tableRowIndex(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0;
    makeCurrent(*record);
    return ImGui::TableGetRowIndex();
}

const char *tableColumnName(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return nullptr;
    makeCurrent(*record);
    return ImGui::TableGetColumnName(columnIndex);
}

bool tableColumnIsEnabled(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return (ImGui::TableGetColumnFlags(columnIndex) & ImGuiTableColumnFlags_IsEnabled) != 0;
}

bool tableColumnIsSorted(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return (ImGui::TableGetColumnFlags(columnIndex) & ImGuiTableColumnFlags_IsSorted) != 0;
}

void tableSetBgColor(UiModuleState &state, UiContextHandle handle, UiTableBgTarget target, std::uint32_t color)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::TableSetBgColor(nativeTableBgTarget(target), static_cast<ImU32>(color));
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
