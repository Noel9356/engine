// Private backend surface shared by the ui module's translation units.
//
// Owns the ImGui-facing state types: the per-context record (native context
// plus the per-frame draw-data buffers), the module state (context table,
// owner-thread pin, resolved configuration), and the mapping/validation/
// context/input/widget/font/style helper declarations the capability
// implementations forward to. Compiled into every ui_* TU; backend-dependent
// code is guarded by ENGINE_UI_BACKEND_IMGUI.

#ifndef ENGINE_UI_BACKEND_HPP
#define ENGINE_UI_BACKEND_HPP

#include <engine/ui/capabilities.hpp>

#include <cstdint>
#include <string>

#if defined(ENGINE_UI_BACKEND_IMGUI)
#include <imgui.h>

#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>
#endif

namespace engine
{
namespace ui
{
namespace backend
{

// Writes an error code through an optional out-parameter.
inline void setError(UiError *outError, UiError error)
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
// iniFilenameStorage keeps the per-context ini path alive for the native
// context's lifetime; io.IniFilename points into it.
struct UiContextRecord
{
    ImGuiContext                              *native             = nullptr;
    std::thread::id                            ownerThread        = std::this_thread::get_id();
    UiContextDescription                       description{};
    bool                                       frameActive        = false;
    bool                                       touchPointerActive = false;
    std::uint32_t                              touchPointerId     = 0;
    UiFontHandle                               nextFont           = 1;
    std::unordered_map<UiFontHandle, ImFont *> fonts;
    std::vector<UiVertex>                      vertices;
    std::vector<std::uint32_t>                 indices;
    std::vector<UiDrawCommand>                 commands;
    UiDrawData                                 drawData{};
    std::string                                iniFilenameStorage;

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
            native = nullptr;
        }
    }
};

#endif

// Module state shared across TUs: the context table, the owner-thread pin,
// and the resolved [ui] configuration. Always defined so the module entry TU
// can hold one even when the backend is disabled. The context table and
// owner-thread pin only exist when the ImGui backend is compiled in.
struct UiModuleState
{
#if defined(ENGINE_UI_BACKEND_IMGUI)
    std::unordered_map<UiContextHandle, std::unique_ptr<UiContextRecord>> contexts;
    std::thread::id                                                       ownerThread{};
    bool                                                                  hasOwnerThread   = false;
    UiContextHandle                                                       nextContext      = 1;
#endif

    // Resolved [ui] configuration (read once in construct(); see
    // configuration.md). iniFilename provides the default ini path used when
    // saveWindowLayout is true and a context did not override it. defaultStyle
    // and defaultFontSize are applied to every new context that did not pin
    // its own in UiContextDescription.
    bool         saveWindowLayout  = false;
    std::string  iniFilename;
    UiStyle      defaultStyle      = UiStyle::Dark;
    float        defaultFontSize   = 0.0f;
};

#if defined(ENGINE_UI_BACKEND_IMGUI)

// --- Internal helpers (defined in ui_context.cpp) -------------------------

UiContextRecord *findContext(UiModuleState &state, UiContextHandle handle) noexcept;
bool             inFrame(const UiContextRecord *context) noexcept;
void             makeCurrent(UiContextRecord &context) noexcept;
void             clearDrawData(UiContextRecord &context) noexcept;
bool             copyDrawData(UiContextRecord &context) noexcept;
bool             resolveContextDescription(const UiContextDescription *description, UiContextDescription &resolved,
    const UiModuleState &state) noexcept;
bool             validFrameDescription(const UiFrameDescription &description) noexcept;

// --- Enum -> ImGui mapping helpers (defined in ui_mappings.cpp) -----------

ImGuiKey            nativeKey(UiKey key) noexcept;
ImGuiKey            nativeGamepadButton(UiGamepadButton button) noexcept;
ImGuiWindowFlags    nativeWindowFlags(UiWindowFlags flags) noexcept;
ImGuiCol            nativeColor(UiColor color) noexcept;
ImGuiStyleVar       nativeStyleVariable(UiStyleVariable variable) noexcept;
ImGuiChildFlags     nativeChildFlags(UiChildFlags flags) noexcept;
ImGuiTreeNodeFlags  nativeTreeNodeFlags(UiTreeNodeFlags flags) noexcept;
ImGuiSelectableFlags nativeSelectableFlags(UiSelectableFlags flags) noexcept;
ImGuiPopupFlags     nativePopupFlags(UiPopupFlags flags) noexcept;
ImGuiTabBarFlags    nativeTabBarFlags(UiTabBarFlags flags) noexcept;
ImGuiTabItemFlags   nativeTabItemFlags(UiTabItemFlags flags) noexcept;
ImGuiTableFlags     nativeTableFlags(UiTableFlags flags) noexcept;
ImGuiTableColumnFlags nativeTableColumnFlags(UiTableColumnFlags flags) noexcept;
ImGuiTableRowFlags  nativeTableRowFlags(UiTableRowFlags flags) noexcept;
ImGuiColorEditFlags nativeColorEditFlags(UiColorEditFlags flags) noexcept;
ImGuiMouseCursor    nativeMouseCursor(UiMouseCursor cursor) noexcept;
UiMouseCursor       publicMouseCursor(ImGuiMouseCursor cursor) noexcept;
ImGuiTableBgTarget  nativeTableBgTarget(UiTableBgTarget target) noexcept;
void                applyStyle(UiStyle style) noexcept;

// --- Context lifetime and frame (defined in ui_context.cpp) ---------------

UiContextHandle     createContext(UiModuleState &state, const UiContextDescription *description, UiError *error);
void                destroyContext(UiModuleState &state, UiContextHandle handle);
bool                beginFrame(UiModuleState &state, UiContextHandle handle, const UiFrameDescription *description,
    UiError *error);
bool                endFrame(UiModuleState &state, UiContextHandle handle, UiError *error);
const UiDrawData    *drawData(UiModuleState &state, UiContextHandle handle);
bool                captureState(UiModuleState &state, UiContextHandle handle, UiCaptureState *outState);

// --- Input event sink (defined in ui_input.cpp) ---------------------------

void addMousePosition(UiModuleState &state, UiContextHandle handle, float x, float y);
void addMouseButton(UiModuleState &state, UiContextHandle handle, std::uint32_t button, bool down);
void addTouch(UiModuleState &state, UiContextHandle handle, std::uint32_t touchId, UiTouchPhase phase, float x,
    float y);
void addMouseWheel(UiModuleState &state, UiContextHandle handle, float horizontal, float vertical);
void addKey(UiModuleState &state, UiContextHandle handle, UiKey key, bool down);
void addGamepadButton(UiModuleState &state, UiContextHandle handle, UiGamepadButton button, bool down);
void addGamepadAxis(UiModuleState &state, UiContextHandle handle, UiGamepadAxis axis, float value);
void addText(UiModuleState &state, UiContextHandle handle, std::uint32_t codepoint);
void addFocus(UiModuleState &state, UiContextHandle handle, bool focused);

// --- Windows, widgets, vector sliders, color edits, plots, queries --------
// (defined in ui_widgets.cpp)

bool beginWindow(UiModuleState &state, UiContextHandle handle, const char *name, bool *open, UiWindowFlags flags);
void endWindow(UiModuleState &state, UiContextHandle handle);
void text(UiModuleState &state, UiContextHandle handle, const char *value);
void textColored(UiModuleState &state, UiContextHandle handle, float red, float green, float blue, float alpha,
    const char *value);
void bulletText(UiModuleState &state, UiContextHandle handle, const char *value);
void separator(UiModuleState &state, UiContextHandle handle);
void sameLine(UiModuleState &state, UiContextHandle handle);
void spacing(UiModuleState &state, UiContextHandle handle);
void newLine(UiModuleState &state, UiContextHandle handle);
void dummy(UiModuleState &state, UiContextHandle handle, float width, float height);
bool button(UiModuleState &state, UiContextHandle handle, const char *label, float width, float height);
bool checkbox(UiModuleState &state, UiContextHandle handle, const char *label, bool *value);
bool selectable(UiModuleState &state, UiContextHandle handle, const char *label, bool *selected, float width,
    float height);
bool selectableEx(UiModuleState &state, UiContextHandle handle, const char *label, const bool *selected,
    float width, float height, UiSelectableFlags flags);
bool radioButton(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t buttonValue);
void progressBar(UiModuleState &state, UiContextHandle handle, float fraction, float width, float height,
    const char *overlay);
void image(UiModuleState &state, UiContextHandle handle, UiTextureHandle texture, float width, float height,
    float uv0x, float uv0y, float uv1x, float uv1y);
bool sliderFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, float minimum,
    float maximum, const char *format);
bool inputText(UiModuleState &state, UiContextHandle handle, const char *label, char *buffer,
    std::uint32_t bufferSize);
bool inputInt(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t step, std::int32_t fastStep);
bool inputFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, const char *format);
bool sliderInt(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *value,
    std::int32_t minimum, std::int32_t maximum, const char *format);
bool dragFloat(UiModuleState &state, UiContextHandle handle, const char *label, float *value, float speed,
    float minimum, float maximum, const char *format);
bool combo(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *currentItem,
    const char *const *items, std::uint32_t itemCount);
bool sliderFloatN(UiModuleState &state, UiContextHandle handle, const char *label, float *values, int components,
    float minimum, float maximum, const char *format);
bool sliderIntN(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *values,
    int components, std::int32_t minimum, std::int32_t maximum, const char *format);
bool dragFloatN(UiModuleState &state, UiContextHandle handle, const char *label, float *values, int components,
    float speed, float minimum, float maximum, const char *format);
bool dragIntN(UiModuleState &state, UiContextHandle handle, const char *label, std::int32_t *values,
    int components, float speed, std::int32_t minimum, std::int32_t maximum, const char *format);
bool colorEdit3(UiModuleState &state, UiContextHandle handle, const char *label, float values[3],
    UiColorEditFlags flags);
bool colorEdit4(UiModuleState &state, UiContextHandle handle, const char *label, float values[4],
    UiColorEditFlags flags);
bool colorButton(UiModuleState &state, UiContextHandle handle, const char *descriptionId, const float values[4],
    UiColorEditFlags flags, float width, float height);
void plotLines(UiModuleState &state, UiContextHandle handle, const char *label, const float *values,
    std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay, float graphWidth,
    float graphHeight);
void plotHistogram(UiModuleState &state, UiContextHandle handle, const char *label, const float *values,
    std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay, float graphWidth,
    float graphHeight);
bool isItemHovered(UiModuleState &state, UiContextHandle handle);
bool isItemActive(UiModuleState &state, UiContextHandle handle);
bool isItemFocused(UiModuleState &state, UiContextHandle handle);
bool isItemClicked(UiModuleState &state, UiContextHandle handle, std::uint32_t button);
bool isItemVisible(UiModuleState &state, UiContextHandle handle);
bool isItemActivated(UiModuleState &state, UiContextHandle handle);
bool isItemDeactivated(UiModuleState &state, UiContextHandle handle);
bool isItemDeactivatedAfterEdit(UiModuleState &state, UiContextHandle handle);
bool isAnyItemHovered(UiModuleState &state, UiContextHandle handle);
bool isAnyItemActive(UiModuleState &state, UiContextHandle handle);
bool isWindowAppearing(UiModuleState &state, UiContextHandle handle);
bool isWindowCollapsed(UiModuleState &state, UiContextHandle handle);
bool isWindowFocused(UiModuleState &state, UiContextHandle handle);
bool isWindowHovered(UiModuleState &state, UiContextHandle handle);
float windowWidth(UiModuleState &state, UiContextHandle handle);
float windowHeight(UiModuleState &state, UiContextHandle handle);
float windowPositionX(UiModuleState &state, UiContextHandle handle);
float windowPositionY(UiModuleState &state, UiContextHandle handle);
void setNextWindowFocus(UiModuleState &state, UiContextHandle handle);
void setNextWindowCollapsed(UiModuleState &state, UiContextHandle handle, bool collapsed);
void setWindowFocus(UiModuleState &state, UiContextHandle handle, const char *name);
void setWindowCollapsedNamed(UiModuleState &state, UiContextHandle handle, const char *name, bool collapsed);

// --- Layout primitives, tree, disabled (defined in ui_layout.cpp) ---------

bool beginChild(UiModuleState &state, UiContextHandle handle, const char *name, float width, float height,
    UiChildFlags flags);
void endChild(UiModuleState &state, UiContextHandle handle);
void beginGroup(UiModuleState &state, UiContextHandle handle);
void endGroup(UiModuleState &state, UiContextHandle handle);
void indent(UiModuleState &state, UiContextHandle handle, float width);
void unindent(UiModuleState &state, UiContextHandle handle, float width);
bool treeNode(UiModuleState &state, UiContextHandle handle, const char *label);
bool treeNodeEx(UiModuleState &state, UiContextHandle handle, const char *label, UiTreeNodeFlags flags);
void treePop(UiModuleState &state, UiContextHandle handle);
void setNextItemOpen(UiModuleState &state, UiContextHandle handle, bool open);
bool collapsingHeader(UiModuleState &state, UiContextHandle handle, const char *label, UiTreeNodeFlags flags);
bool collapsingHeaderBool(UiModuleState &state, UiContextHandle handle, const char *label, bool *open,
    UiTreeNodeFlags flags);
void beginDisabled(UiModuleState &state, UiContextHandle handle, bool disabled);
void endDisabled(UiModuleState &state, UiContextHandle handle);

// --- Popups, menus, tooltips (defined in ui_popup.cpp) --------------------

void openPopup(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
void openPopupId(UiModuleState &state, UiContextHandle handle, std::uint32_t identifier, UiPopupFlags flags);
bool beginPopup(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
bool beginPopupModal(UiModuleState &state, UiContextHandle handle, const char *name, bool *open, UiPopupFlags flags);
bool beginPopupContextItem(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
bool beginPopupContextWindow(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
bool beginPopupContextVoid(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
void endPopup(UiModuleState &state, UiContextHandle handle);
void closeCurrentPopup(UiModuleState &state, UiContextHandle handle);
bool isPopupOpen(UiModuleState &state, UiContextHandle handle, const char *name, UiPopupFlags flags);
bool beginMenuBar(UiModuleState &state, UiContextHandle handle);
void endMenuBar(UiModuleState &state, UiContextHandle handle);
bool beginMenu(UiModuleState &state, UiContextHandle handle, const char *label, bool enabled);
void endMenu(UiModuleState &state, UiContextHandle handle);
bool menuItem(UiModuleState &state, UiContextHandle handle, const char *label, const char *shortcut,
    bool *selected, bool enabled);
bool menuItemActivated(UiModuleState &state, UiContextHandle handle, const char *label, const char *shortcut,
    bool enabled);
void setTooltip(UiModuleState &state, UiContextHandle handle, const char *value);
void beginTooltip(UiModuleState &state, UiContextHandle handle);
void endTooltip(UiModuleState &state, UiContextHandle handle);

// --- Tabs and tables (defined in ui_table.cpp) ----------------------------

bool beginTabBar(UiModuleState &state, UiContextHandle handle, const char *name, UiTabBarFlags flags);
void endTabBar(UiModuleState &state, UiContextHandle handle);
bool beginTabItem(UiModuleState &state, UiContextHandle handle, const char *label, bool *open, UiTabItemFlags flags);
void endTabItem(UiModuleState &state, UiContextHandle handle);
void setTabItemClosed(UiModuleState &state, UiContextHandle handle, const char *label);
bool beginTable(UiModuleState &state, UiContextHandle handle, const char *name, std::uint32_t columns,
    UiTableFlags flags, float outerSizeX, float outerSizeY, float innerWidth);
void endTable(UiModuleState &state, UiContextHandle handle);
void tableNextRow(UiModuleState &state, UiContextHandle handle, UiTableRowFlags flags, float minRowHeight);
bool tableNextColumn(UiModuleState &state, UiContextHandle handle);
bool tableSetColumnIndex(UiModuleState &state, UiContextHandle handle, std::uint32_t columnIndex);
void tableSetupColumn(UiModuleState &state, UiContextHandle handle, const char *label, UiTableColumnFlags flags,
    float initWidthOrWeight, std::uint32_t identifier);
void tableSetupScrollFreeze(UiModuleState &state, UiContextHandle handle, std::uint32_t columns, std::uint32_t rows);
void tableHeadersRow(UiModuleState &state, UiContextHandle handle);
void tableHeader(UiModuleState &state, UiContextHandle handle, const char *label);
std::int32_t tableColumnCount(UiModuleState &state, UiContextHandle handle);
std::int32_t tableColumnIndex(UiModuleState &state, UiContextHandle handle);
std::int32_t tableRowIndex(UiModuleState &state, UiContextHandle handle);
const char *tableColumnName(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex);
bool tableColumnIsEnabled(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex);
bool tableColumnIsSorted(UiModuleState &state, UiContextHandle handle, std::int32_t columnIndex);
void tableSetBgColor(UiModuleState &state, UiContextHandle handle, UiTableBgTarget target, std::uint32_t color);

// --- Fonts, style, window placement, cursor (defined in ui_fonts_style.cpp)

bool addFontFromFile(UiModuleState &state, UiContextHandle handle, const char *filePath, float pixelSize,
    UiFontHandle *outFont);
bool addFontFromMemoryTTF(UiModuleState &state, UiContextHandle handle, const void *data, std::uint32_t dataSize,
    float pixelSize, UiFontHandle *outFont);
bool pushFont(UiModuleState &state, UiContextHandle handle, UiFontHandle fontHandle);
void popFont(UiModuleState &state, UiContextHandle handle);
bool fontAtlas(UiModuleState &state, UiContextHandle handle, UiFontAtlas *outAtlas);
bool setFontTexture(UiModuleState &state, UiContextHandle handle, UiTextureHandle texture);
bool pushStyleColor(UiModuleState &state, UiContextHandle handle, UiColor color, float red, float green,
    float blue, float alpha);
void popStyleColor(UiModuleState &state, UiContextHandle handle, std::uint32_t count);
bool pushStyleVariable(UiModuleState &state, UiContextHandle handle, UiStyleVariable variable, float value);
void popStyleVariable(UiModuleState &state, UiContextHandle handle, std::uint32_t count);
void setNextWindowPosition(UiModuleState &state, UiContextHandle handle, float x, float y, float pivotX,
    float pivotY);
void setNextWindowSize(UiModuleState &state, UiContextHandle handle, float width, float height);
void setMouseCursor(UiModuleState &state, UiContextHandle handle, UiMouseCursor cursor);
UiMouseCursor mouseCursor(UiModuleState &state, UiContextHandle handle);

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine

#endif
