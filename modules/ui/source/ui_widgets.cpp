// Windows, widgets, vector sliders/drags, color edits, plots, item state
// queries.
//
// Every call resolves the context through findContext and forwards to the
// matching ImGui:: function. Calls outside an active frame are silently
// ignored. Fallible widget calls validate their arguments before touching
// ImGui so the consumer's state is not mutated by a rejected call.

#include "ui_backend.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

// --- Windows ---------------------------------------------------------------

bool beginWindow(UiModuleState &state, UiContextHandle handle, const char *name, bool *open, UiWindowFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::Begin(name, open, nativeWindowFlags(flags));
}

void endWindow(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::End();
}

// --- Text and spacing ------------------------------------------------------

void text(UiModuleState &state, UiContextHandle handle, const char *value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || value == nullptr)
        return;
    makeCurrent(*record);
    ImGui::TextUnformatted(value);
}

void textColored(UiModuleState &state, UiContextHandle handle, float red, float green, float blue, float alpha,
    const char *value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || value == nullptr || !std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue)
        || !std::isfinite(alpha))
        return;
    makeCurrent(*record);
    ImGui::TextColored(ImVec4(red, green, blue, alpha), "%s", value);
}

void bulletText(UiModuleState &state, UiContextHandle handle, const char *value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || value == nullptr)
        return;
    makeCurrent(*record);
    ImGui::BulletText("%s", value);
}

void separator(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::Separator();
}

void sameLine(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::SameLine();
}

void spacing(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::Spacing();
}

void newLine(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::NewLine();
}

void dummy(UiModuleState &state, UiContextHandle handle, float width, float height)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(width) || !std::isfinite(height))
        return;
    makeCurrent(*record);
    ImGui::Dummy(ImVec2(width, height));
}

// --- Buttons and selects ---------------------------------------------------

bool button(UiModuleState &state, UiContextHandle handle, const char *label, float width, float height)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || !std::isfinite(width) || !std::isfinite(height))
        return false;
    makeCurrent(*record);
    return ImGui::Button(label, ImVec2(width, height));
}

bool checkbox(UiModuleState &state, UiContextHandle handle, const char *label, bool *value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::Checkbox(label, value);
}

bool selectable(UiModuleState &state, UiContextHandle handle, const char *label, bool *selected, float width,
    float height)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || selected == nullptr || !std::isfinite(width)
        || !std::isfinite(height))
        return false;
    makeCurrent(*record);
    const bool activated = ImGui::Selectable(label, *selected, ImGuiSelectableFlags_None, ImVec2(width, height));
    if (activated)
        *selected = !*selected;
    return activated;
}

bool selectableEx(UiModuleState &state, UiContextHandle handle, const char *label, const bool *selected,
    float width, float height, UiSelectableFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || selected == nullptr || !std::isfinite(width)
        || !std::isfinite(height))
        return false;
    makeCurrent(*record);
    return ImGui::Selectable(label, *selected, nativeSelectableFlags(flags), ImVec2(width, height));
}

bool radioButton(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t buttonValue)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr)
        return false;
    makeCurrent(*record);
    int nativeValue = static_cast<int>(*value);
    const bool changed = ImGui::RadioButton(label, &nativeValue, static_cast<int>(buttonValue));
    *value = static_cast<std::int32_t>(nativeValue);
    return changed;
}

void progressBar(UiModuleState &state, UiContextHandle handle, float fraction, float width, float height,
    const char *overlay)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(fraction) || !std::isfinite(width) || !std::isfinite(height))
        return;
    makeCurrent(*record);
    ImGui::ProgressBar(std::clamp(fraction, 0.0f, 1.0f), ImVec2(width, height), overlay);
}

void image(UiModuleState &state, UiContextHandle handle, UiTextureHandle texture, float width, float height,
    float uv0x, float uv0y, float uv1x, float uv1y)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(width) || !std::isfinite(height))
        return;
    makeCurrent(*record);
    ImGui::Image(static_cast<ImTextureID>(texture), ImVec2(width, height), ImVec2(uv0x, uv0y), ImVec2(uv1x, uv1y));
}

// --- Numeric widgets -------------------------------------------------------

bool sliderFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, float minimum,
    float maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(minimum)
        || !std::isfinite(maximum) || minimum > maximum)
        return false;
    makeCurrent(*record);
    return ImGui::SliderFloat(label, value, minimum, maximum, format != nullptr ? format : "%.3f");
}

bool inputText(UiModuleState &state, UiContextHandle handle, const char *label, char *buffer,
    std::uint32_t bufferSize)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || buffer == nullptr || bufferSize == 0)
        return false;
    makeCurrent(*record);
    return ImGui::InputText(label, buffer, static_cast<std::size_t>(bufferSize));
}

bool inputInt(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t step, std::int32_t fastStep)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr || step < 0 || fastStep < 0)
        return false;
    makeCurrent(*record);
    int nativeValue = static_cast<int>(*value);
    const bool changed = ImGui::InputInt(label, &nativeValue, static_cast<int>(step), static_cast<int>(fastStep));
    *value = static_cast<std::int32_t>(nativeValue);
    return changed;
}

bool inputFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(*value))
        return false;
    makeCurrent(*record);
    return ImGui::InputFloat(label, value, 0.0f, 0.0f, format != nullptr ? format : "%.3f");
}

bool sliderInt(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t minimum, std::int32_t maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr || minimum > maximum)
        return false;
    makeCurrent(*record);
    int nativeValue = static_cast<int>(*value);
    const bool changed = ImGui::SliderInt(label, &nativeValue, static_cast<int>(minimum),
        static_cast<int>(maximum), format != nullptr ? format : "%d");
    *value = static_cast<std::int32_t>(nativeValue);
    return changed;
}

bool dragFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, float speed,
    float minimum, float maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || value == nullptr || !std::isfinite(*value)
        || !std::isfinite(speed) || speed < 0.0f || !std::isfinite(minimum)
        || !std::isfinite(maximum) || minimum >= maximum)
        return false;
    makeCurrent(*record);
    return ImGui::DragFloat(label, value, speed, minimum, maximum, format != nullptr ? format : "%.3f");
}

bool combo(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *currentItem,
    const char *const *items, std::uint32_t itemCount)
{
    UiContextRecord *record = findContext(state, handle);
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
}

// --- Vector sliders/drags --------------------------------------------------

bool sliderFloatN(UiModuleState &state, UiContextHandle handle, const char *label, float *values, int components,
    float minimum, float maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || !std::isfinite(minimum)
        || !std::isfinite(maximum) || minimum > maximum)
        return false;
    makeCurrent(*record);
    const char *resolvedFormat = format != nullptr ? format : "%.3f";
    switch (components)
    {
    case 2:
        return ImGui::SliderFloat2(label, values, minimum, maximum, resolvedFormat);
    case 3:
        return ImGui::SliderFloat3(label, values, minimum, maximum, resolvedFormat);
    case 4:
        return ImGui::SliderFloat4(label, values, minimum, maximum, resolvedFormat);
    default:
        return false;
    }
}

bool sliderIntN(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *values,
    int components, std::int32_t minimum, std::int32_t maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || minimum > maximum)
        return false;
    makeCurrent(*record);
    const char *resolvedFormat = format != nullptr ? format : "%d";
    int nativeValues[4] = {0, 0, 0, 0};
    for (int i = 0; i < components; ++i)
        nativeValues[i] = static_cast<int>(values[i]);
    bool changed = false;
    switch (components)
    {
    case 2:
        changed = ImGui::SliderInt2(label, nativeValues, minimum, maximum, resolvedFormat);
        break;
    case 3:
        changed = ImGui::SliderInt3(label, nativeValues, minimum, maximum, resolvedFormat);
        break;
    case 4:
        changed = ImGui::SliderInt4(label, nativeValues, minimum, maximum, resolvedFormat);
        break;
    default:
        return false;
    }
    if (changed)
        for (int i = 0; i < components; ++i)
            values[i] = static_cast<std::int32_t>(nativeValues[i]);
    return changed;
}

bool dragFloatN(UiModuleState &state, UiContextHandle handle, const char *label, float *values, int components,
    float speed, float minimum, float maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || !std::isfinite(speed) || speed < 0.0f
        || !std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum)
        return false;
    makeCurrent(*record);
    const char *resolvedFormat = format != nullptr ? format : "%.3f";
    switch (components)
    {
    case 2:
        return ImGui::DragFloat2(label, values, speed, minimum, maximum, resolvedFormat);
    case 3:
        return ImGui::DragFloat3(label, values, speed, minimum, maximum, resolvedFormat);
    default:
        return false;
    }
}

bool dragIntN(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *values,
    int components, float speed, std::int32_t minimum, std::int32_t maximum, const char *format)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || !std::isfinite(speed) || speed < 0.0f
        || minimum >= maximum)
        return false;
    makeCurrent(*record);
    const char *resolvedFormat = format != nullptr ? format : "%d";
    int nativeValues[3] = {0, 0, 0};
    for (int i = 0; i < components; ++i)
        nativeValues[i] = static_cast<int>(values[i]);
    bool changed = false;
    switch (components)
    {
    case 2:
        changed = ImGui::DragInt2(label, nativeValues, speed, minimum, maximum, resolvedFormat);
        break;
    case 3:
        changed = ImGui::DragInt3(label, nativeValues, speed, minimum, maximum, resolvedFormat);
        break;
    default:
        return false;
    }
    if (changed)
        for (int i = 0; i < components; ++i)
            values[i] = static_cast<std::int32_t>(nativeValues[i]);
    return changed;
}

// --- Color edits -----------------------------------------------------------

bool colorEdit3(UiModuleState &state, UiContextHandle handle, const char *label, float values[3],
    UiColorEditFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::ColorEdit3(label, values, nativeColorEditFlags(flags));
}

bool colorEdit4(UiModuleState &state, UiContextHandle handle, const char *label, float values[4],
    UiColorEditFlags flags)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr)
        return false;
    makeCurrent(*record);
    return ImGui::ColorEdit4(label, values, nativeColorEditFlags(flags));
}

bool colorButton(UiModuleState &state, UiContextHandle handle, const char *descriptionId, const float values[4],
    UiColorEditFlags flags, float width, float height)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || descriptionId == nullptr || values == nullptr || !std::isfinite(width)
        || !std::isfinite(height))
        return false;
    makeCurrent(*record);
    return ImGui::ColorButton(descriptionId, ImVec4(values[0], values[1], values[2], values[3]),
        nativeColorEditFlags(flags), ImVec2(width, height));
}

// --- Plots -----------------------------------------------------------------

void plotLines(UiModuleState &state, UiContextHandle handle, const char *label, const float *values,
    std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay, float graphWidth,
    float graphHeight)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || valueCount == 0
        || valueCount > static_cast<std::uint32_t>(INT_MAX))
        return;
    makeCurrent(*record);
    ImGui::PlotLines(label, values, static_cast<int>(valueCount), 0, overlay, valueMinimum, valueMaximum,
        ImVec2(graphWidth, graphHeight));
}

void plotHistogram(UiModuleState &state, UiContextHandle handle, const char *label, const float *values,
    std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay, float graphWidth,
    float graphHeight)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || label == nullptr || values == nullptr || valueCount == 0
        || valueCount > static_cast<std::uint32_t>(INT_MAX))
        return;
    makeCurrent(*record);
    ImGui::PlotHistogram(label, values, static_cast<int>(valueCount), 0, overlay, valueMinimum, valueMaximum,
        ImVec2(graphWidth, graphHeight));
}

// --- Item state queries ----------------------------------------------------

bool isItemHovered(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemHovered();
}

bool isItemActive(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemActive();
}

bool isItemFocused(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemFocused();
}

bool isItemClicked(UiModuleState &state, UiContextHandle handle, std::uint32_t button)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || button >= ImGuiMouseButton_COUNT)
        return false;
    makeCurrent(*record);
    return ImGui::IsItemClicked(static_cast<int>(button));
}

bool isItemVisible(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemVisible();
}

bool isItemActivated(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemActivated();
}

bool isItemDeactivated(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemDeactivated();
}

bool isItemDeactivatedAfterEdit(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsItemDeactivatedAfterEdit();
}

bool isAnyItemHovered(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsAnyItemHovered();
}

bool isAnyItemActive(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsAnyItemActive();
}

bool isWindowAppearing(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsWindowAppearing();
}

bool isWindowCollapsed(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsWindowCollapsed();
}

bool isWindowFocused(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsWindowFocused(ImGuiFocusedFlags_None);
}

bool isWindowHovered(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    makeCurrent(*record);
    return ImGui::IsWindowHovered(ImGuiHoveredFlags_None);
}

// --- Window placement queries ----------------------------------------------

float windowWidth(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0.0f;
    makeCurrent(*record);
    return ImGui::GetWindowWidth();
}

float windowHeight(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0.0f;
    makeCurrent(*record);
    return ImGui::GetWindowHeight();
}

float windowPositionX(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0.0f;
    makeCurrent(*record);
    return ImGui::GetWindowPos().x;
}

float windowPositionY(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return 0.0f;
    makeCurrent(*record);
    return ImGui::GetWindowPos().y;
}

void setNextWindowFocus(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::SetNextWindowFocus();
}

void setNextWindowCollapsed(UiModuleState &state, UiContextHandle handle, bool collapsed)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::SetNextWindowCollapsed(collapsed, ImGuiCond_Always);
}

void setWindowFocus(UiModuleState &state, UiContextHandle handle, const char *name)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return;
    makeCurrent(*record);
    ImGui::SetWindowFocus(name);
}

void setWindowCollapsedNamed(UiModuleState &state, UiContextHandle handle, const char *name, bool collapsed)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || name == nullptr)
        return;
    makeCurrent(*record);
    ImGui::SetWindowCollapsed(name, collapsed, ImGuiCond_Always);
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
