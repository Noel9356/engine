// Font management, style stack, window placement, mouse cursor.
//
// Font calls only mutate the atlas outside an active frame: addFontFromFile
// and addFontFromMemoryTTF call Fonts->Build() which rebuilds the atlas and
// must not happen mid-frame. pushFont/popFont bracket a widget range and
// require an active frame. The style stack requires an active frame too.
// Window placement helpers (setNextWindowPosition/setNextWindowSize) are
// "next window" calls: they must be issued immediately before beginWindow.

#include "ui_backend.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

bool addFontFromFile(UiModuleState &state, UiContextHandle handle, const char *filePath, float pixelSize,
    UiFontHandle *outFont)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || record->frameActive || filePath == nullptr || filePath[0] == '\0'
        || !std::isfinite(pixelSize) || pixelSize <= 0.0f || outFont == nullptr)
        return false;
    makeCurrent(*record);
    ImFont *font = ImGui::GetIO().Fonts->AddFontFromFileTTF(filePath, pixelSize);
    if (font == nullptr)
        return false;
    if (!ImGui::GetIO().Fonts->Build())
        return false;
    const UiFontHandle fontHandle = record->nextFont++;
    if (fontHandle == defaultUiFont)
        return false;
    record->fonts.emplace(fontHandle, font);
    *outFont = fontHandle;
    return true;
}

// Loads a TTF from memory. The data is copied into the atlas's ownership
// when FontConfig::FontDataOwnedByAtlas is true (the default), so the caller
// may free the buffer after the call returns. Pixel size must be positive.
bool addFontFromMemoryTTF(UiModuleState &state, UiContextHandle handle, const void *data, std::uint32_t dataSize,
    float pixelSize, UiFontHandle *outFont)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || record->frameActive || data == nullptr || dataSize == 0
        || !std::isfinite(pixelSize) || pixelSize <= 0.0f || outFont == nullptr)
        return false;
    makeCurrent(*record);
    ImFontConfig fontConfig{};
    fontConfig.FontDataOwnedByAtlas = true;
    ImFont *font = ImGui::GetIO().Fonts->AddFontFromMemoryTTF(
        const_cast<void *>(data), static_cast<int>(dataSize), pixelSize, &fontConfig);
    if (font == nullptr)
        return false;
    if (!ImGui::GetIO().Fonts->Build())
        return false;
    const UiFontHandle fontHandle = record->nextFont++;
    if (fontHandle == defaultUiFont)
        return false;
    record->fonts.emplace(fontHandle, font);
    *outFont = fontHandle;
    return true;
}

bool pushFont(UiModuleState &state, UiContextHandle handle, UiFontHandle fontHandle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return false;
    const auto font = record->fonts.find(fontHandle);
    if (font == record->fonts.end())
        return false;
    makeCurrent(*record);
    ImGui::PushFont(font->second);
    return true;
}

void popFont(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::PopFont();
}

bool fontAtlas(UiModuleState &state, UiContextHandle handle, UiFontAtlas *outAtlas)
{
    if (outAtlas == nullptr)
        return false;
    UiContextRecord *record = findContext(state, handle);
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
}

bool setFontTexture(UiModuleState &state, UiContextHandle handle, UiTextureHandle texture)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr)
        return false;
    makeCurrent(*record);
    ImGui::GetIO().Fonts->SetTexID(static_cast<ImTextureID>(texture));
    return true;
}

bool pushStyleColor(UiModuleState &state, UiContextHandle handle, UiColor color, float red, float green,
    float blue, float alpha)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue)
        || !std::isfinite(alpha))
        return false;
    makeCurrent(*record);
    ImGui::PushStyleColor(nativeColor(color), ImVec4(red, green, blue, alpha));
    return true;
}

void popStyleColor(UiModuleState &state, UiContextHandle handle, std::uint32_t count)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || count == 0)
        return;
    makeCurrent(*record);
    ImGui::PopStyleColor(static_cast<int>(std::min<std::uint32_t>(count, INT_MAX)));
}

bool pushStyleVariable(UiModuleState &state, UiContextHandle handle, UiStyleVariable variable, float value)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(value))
        return false;
    makeCurrent(*record);
    ImGui::PushStyleVar(nativeStyleVariable(variable), value);
    return true;
}

void popStyleVariable(UiModuleState &state, UiContextHandle handle, std::uint32_t count)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || count == 0)
        return;
    makeCurrent(*record);
    ImGui::PopStyleVar(static_cast<int>(std::min<std::uint32_t>(count, INT_MAX)));
}

void setNextWindowPosition(UiModuleState &state, UiContextHandle handle, float x, float y, float pivotX,
    float pivotY)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pivotX)
        || !std::isfinite(pivotY))
        return;
    makeCurrent(*record);
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always, ImVec2(pivotX, pivotY));
}

void setNextWindowSize(UiModuleState &state, UiContextHandle handle, float width, float height)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record) || !std::isfinite(width) || !std::isfinite(height))
        return;
    makeCurrent(*record);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
}

void setMouseCursor(UiModuleState &state, UiContextHandle handle, UiMouseCursor cursor)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return;
    makeCurrent(*record);
    ImGui::SetMouseCursor(nativeMouseCursor(cursor));
}

UiMouseCursor mouseCursor(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    if (!inFrame(record))
        return UiMouseCursor::None;
    makeCurrent(*record);
    return publicMouseCursor(ImGui::GetMouseCursor());
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
