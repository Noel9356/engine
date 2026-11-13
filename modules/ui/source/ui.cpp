// ui module entry point.
//
// Owns the ParticipantManifest, the UiModuleState (context table, owner-thread
// pin, resolved [ui] configuration), and the UiCapability function-pointer
// table. The capability lambdas forward to UiModule::...Impl one-liners that
// delegate to engine::ui::backend:: free functions defined in the dedicated
// ui_* TUs. Without ENGINE_UI_BACKEND_IMGUI every entry fails with
// BackendUnavailable.

#include "ui_backend.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/ui/capabilities.hpp>

#include <cstdint>
#include <cstdio>
#include <string>

namespace engine
{
namespace ui
{
namespace
{

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

    // Reads [ui] configuration once. The module does not subscribe to
    // ConfigurationReloadedEvent because context creation freezes the values
    // into each UiContextRecord: a reload would only affect later contexts
    // and the host can already override every field per-context.
    void construct(ParticipantContext &context) override
    {
        state_.saveWindowLayout = context.configurationBooleanInSection("ui", "save_window_layout").value_or(false);

        if (const auto path = context.configurationStringInSection("ui", "ini_filename"))
        {
            state_.iniFilename = *path;
        }

        if (const auto style = context.configurationStringInSection("ui", "default_style"))
        {
            if (*style == "dark")
                state_.defaultStyle = UiStyle::Dark;
            else if (*style == "light")
                state_.defaultStyle = UiStyle::Light;
            else if (*style == "classic")
                state_.defaultStyle = UiStyle::Classic;
            else
                std::fprintf(stderr,
                    "ui module: ignoring [ui] default_style %s; expected dark, light, or classic\n",
                    style->c_str());
        }

        if (const auto fontSize = context.configurationFloatingInSection("ui", "default_font_size"))
        {
            if (std::isfinite(*fontSize) && *fontSize >= 0.0)
            {
                state_.defaultFontSize = static_cast<float>(*fontSize);
            }
            else
            {
                std::fprintf(stderr,
                    "ui module: ignoring [ui] default_font_size %g; expected a non-negative finite value\n",
                    *fontSize);
            }
        }
    }

    void start() override
    {}

    void stop() override
    {
        // ImGui state does not survive module restart: drop every context
        // and release the owner-thread pin so a fresh start can re-establish
        // it on whichever thread constructs the first context.
#if defined(ENGINE_UI_BACKEND_IMGUI)
        state_.contexts.clear();
        state_.ownerThread    = {};
        state_.hasOwnerThread = false;
        state_.nextContext    = 1;
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
    // the matching *Impl method on this module. The lambdas keep the C-style
    // self pointer convention so consumers call ui.field(ui.self, ...) and
    // land on a method that has the typed UiModuleState available.
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
        ENGINE_UI_BIND(selectableEx, selectableExImpl);
        ENGINE_UI_BIND(radioButton, radioButtonImpl);
        ENGINE_UI_BIND(progressBar, progressBarImpl);
        ENGINE_UI_BIND(image, imageImpl);
        ENGINE_UI_BIND(bulletText, bulletTextImpl);
        ENGINE_UI_BIND(beginChild, beginChildImpl);
        ENGINE_UI_BIND(endChild, endChildImpl);
        ENGINE_UI_BIND(beginGroup, beginGroupImpl);
        ENGINE_UI_BIND(endGroup, endGroupImpl);
        ENGINE_UI_BIND(indent, indentImpl);
        ENGINE_UI_BIND(unindent, unindentImpl);
        ENGINE_UI_BIND(newLine, newLineImpl);
        ENGINE_UI_BIND(dummy, dummyImpl);
        ENGINE_UI_BIND(treeNode, treeNodeImpl);
        ENGINE_UI_BIND(treeNodeEx, treeNodeExImpl);
        ENGINE_UI_BIND(treePop, treePopImpl);
        ENGINE_UI_BIND(setNextItemOpen, setNextItemOpenImpl);
        ENGINE_UI_BIND(collapsingHeader, collapsingHeaderImpl);
        ENGINE_UI_BIND(collapsingHeaderBool, collapsingHeaderBoolImpl);
        ENGINE_UI_BIND(beginDisabled, beginDisabledImpl);
        ENGINE_UI_BIND(endDisabled, endDisabledImpl);
        ENGINE_UI_BIND(openPopup, openPopupImpl);
        ENGINE_UI_BIND(openPopupId, openPopupIdImpl);
        ENGINE_UI_BIND(beginPopup, beginPopupImpl);
        ENGINE_UI_BIND(beginPopupModal, beginPopupModalImpl);
        ENGINE_UI_BIND(beginPopupContextItem, beginPopupContextItemImpl);
        ENGINE_UI_BIND(beginPopupContextWindow, beginPopupContextWindowImpl);
        ENGINE_UI_BIND(beginPopupContextVoid, beginPopupContextVoidImpl);
        ENGINE_UI_BIND(endPopup, endPopupImpl);
        ENGINE_UI_BIND(closeCurrentPopup, closeCurrentPopupImpl);
        ENGINE_UI_BIND(isPopupOpen, isPopupOpenImpl);
        ENGINE_UI_BIND(beginMenuBar, beginMenuBarImpl);
        ENGINE_UI_BIND(endMenuBar, endMenuBarImpl);
        ENGINE_UI_BIND(beginMenu, beginMenuImpl);
        ENGINE_UI_BIND(endMenu, endMenuImpl);
        ENGINE_UI_BIND(menuItem, menuItemImpl);
        ENGINE_UI_BIND(menuItemActivated, menuItemActivatedImpl);
        ENGINE_UI_BIND(beginTabBar, beginTabBarImpl);
        ENGINE_UI_BIND(endTabBar, endTabBarImpl);
        ENGINE_UI_BIND(beginTabItem, beginTabItemImpl);
        ENGINE_UI_BIND(endTabItem, endTabItemImpl);
        ENGINE_UI_BIND(setTabItemClosed, setTabItemClosedImpl);
        ENGINE_UI_BIND(beginTable, beginTableImpl);
        ENGINE_UI_BIND(endTable, endTableImpl);
        ENGINE_UI_BIND(tableNextRow, tableNextRowImpl);
        ENGINE_UI_BIND(tableNextColumn, tableNextColumnImpl);
        ENGINE_UI_BIND(tableSetColumnIndex, tableSetColumnIndexImpl);
        ENGINE_UI_BIND(tableSetupColumn, tableSetupColumnImpl);
        ENGINE_UI_BIND(tableSetupScrollFreeze, tableSetupScrollFreezeImpl);
        ENGINE_UI_BIND(tableHeadersRow, tableHeadersRowImpl);
        ENGINE_UI_BIND(tableHeader, tableHeaderImpl);
        ENGINE_UI_BIND(tableColumnCount, tableColumnCountImpl);
        ENGINE_UI_BIND(tableColumnIndex, tableColumnIndexImpl);
        ENGINE_UI_BIND(tableRowIndex, tableRowIndexImpl);
        ENGINE_UI_BIND(tableColumnName, tableColumnNameImpl);
        ENGINE_UI_BIND(tableColumnIsEnabled, tableColumnIsEnabledImpl);
        ENGINE_UI_BIND(tableColumnIsSorted, tableColumnIsSortedImpl);
        ENGINE_UI_BIND(tableSetBgColor, tableSetBgColorImpl);
        ENGINE_UI_BIND(setTooltip, setTooltipImpl);
        ENGINE_UI_BIND(beginTooltip, beginTooltipImpl);
        ENGINE_UI_BIND(endTooltip, endTooltipImpl);
        ENGINE_UI_BIND(sliderFloat2, sliderFloat2Impl);
        ENGINE_UI_BIND(sliderFloat3, sliderFloat3Impl);
        ENGINE_UI_BIND(sliderFloat4, sliderFloat4Impl);
        ENGINE_UI_BIND(sliderInt2, sliderInt2Impl);
        ENGINE_UI_BIND(sliderInt3, sliderInt3Impl);
        ENGINE_UI_BIND(sliderInt4, sliderInt4Impl);
        ENGINE_UI_BIND(dragFloat2, dragFloat2Impl);
        ENGINE_UI_BIND(dragFloat3, dragFloat3Impl);
        ENGINE_UI_BIND(dragInt2, dragInt2Impl);
        ENGINE_UI_BIND(dragInt3, dragInt3Impl);
        ENGINE_UI_BIND(colorEdit3, colorEdit3Impl);
        ENGINE_UI_BIND(colorEdit4, colorEdit4Impl);
        ENGINE_UI_BIND(colorButton, colorButtonImpl);
        ENGINE_UI_BIND(plotLines, plotLinesImpl);
        ENGINE_UI_BIND(plotHistogram, plotHistogramImpl);
        ENGINE_UI_BIND(isItemHovered, isItemHoveredImpl);
        ENGINE_UI_BIND(isItemActive, isItemActiveImpl);
        ENGINE_UI_BIND(isItemFocused, isItemFocusedImpl);
        ENGINE_UI_BIND(isItemClicked, isItemClickedImpl);
        ENGINE_UI_BIND(isItemVisible, isItemVisibleImpl);
        ENGINE_UI_BIND(isItemActivated, isItemActivatedImpl);
        ENGINE_UI_BIND(isItemDeactivated, isItemDeactivatedImpl);
        ENGINE_UI_BIND(isItemDeactivatedAfterEdit, isItemDeactivatedAfterEditImpl);
        ENGINE_UI_BIND(isAnyItemHovered, isAnyItemHoveredImpl);
        ENGINE_UI_BIND(isAnyItemActive, isAnyItemActiveImpl);
        ENGINE_UI_BIND(isWindowAppearing, isWindowAppearingImpl);
        ENGINE_UI_BIND(isWindowCollapsed, isWindowCollapsedImpl);
        ENGINE_UI_BIND(isWindowFocused, isWindowFocusedImpl);
        ENGINE_UI_BIND(isWindowHovered, isWindowHoveredImpl);
        ENGINE_UI_BIND(windowWidth, windowWidthImpl);
        ENGINE_UI_BIND(windowHeight, windowHeightImpl);
        ENGINE_UI_BIND(windowPositionX, windowPositionXImpl);
        ENGINE_UI_BIND(windowPositionY, windowPositionYImpl);
        ENGINE_UI_BIND(setNextWindowFocus, setNextWindowFocusImpl);
        ENGINE_UI_BIND(setNextWindowCollapsed, setNextWindowCollapsedImpl);
        ENGINE_UI_BIND(setWindowFocus, setWindowFocusImpl);
        ENGINE_UI_BIND(setWindowCollapsedNamed, setWindowCollapsedNamedImpl);
        ENGINE_UI_BIND(setMouseCursor, setMouseCursorImpl);
        ENGINE_UI_BIND(mouseCursor, mouseCursorImpl);
        ENGINE_UI_BIND(addFontFromFile, addFontFromFileImpl);
        ENGINE_UI_BIND(addFontFromMemoryTTF, addFontFromMemoryTTFImpl);
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

    // --- Context lifetime --------------------------------------------------

    UiContextHandle createContextImpl(const UiContextDescription *description, UiError *error)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::createContext(state_, description, error);
#else
        (void)description;
        backend::setError(error, UiError::BackendUnavailable);
        return invalidUiContext;
#endif
    }

    void destroyContextImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::destroyContext(state_, handle);
#else
        (void)handle;
#endif
    }

    bool beginFrameImpl(UiContextHandle handle, const UiFrameDescription *description, UiError *error)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginFrame(state_, handle, description, error);
#else
        (void)handle;
        (void)description;
        backend::setError(error, UiError::BackendUnavailable);
        return false;
#endif
    }

    bool endFrameImpl(UiContextHandle handle, UiError *error)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::endFrame(state_, handle, error);
#else
        (void)handle;
        backend::setError(error, UiError::BackendUnavailable);
        return false;
#endif
    }

    const UiDrawData *drawDataImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::drawData(state_, handle);
#else
        (void)handle;
        return nullptr;
#endif
    }

    bool captureStateImpl(UiContextHandle handle, UiCaptureState *outState)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::captureState(state_, handle, outState);
#else
        (void)handle;
        (void)outState;
        return false;
#endif
    }

    // --- Input -------------------------------------------------------------

    void addMousePositionImpl(UiContextHandle handle, float x, float y)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addMousePosition(state_, handle, x, y);
#else
        (void)handle;
        (void)x;
        (void)y;
#endif
    }

    void addMouseButtonImpl(UiContextHandle handle, std::uint32_t button, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addMouseButton(state_, handle, button, down);
#else
        (void)handle;
        (void)button;
        (void)down;
#endif
    }

    void addTouchImpl(UiContextHandle handle, std::uint32_t touchId, UiTouchPhase phase, float x, float y)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addTouch(state_, handle, touchId, phase, x, y);
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
        backend::addMouseWheel(state_, handle, horizontal, vertical);
#else
        (void)handle;
        (void)horizontal;
        (void)vertical;
#endif
    }

    void addKeyImpl(UiContextHandle handle, UiKey key, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addKey(state_, handle, key, down);
#else
        (void)handle;
        (void)key;
        (void)down;
#endif
    }

    void addGamepadButtonImpl(UiContextHandle handle, UiGamepadButton button, bool down)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addGamepadButton(state_, handle, button, down);
#else
        (void)handle;
        (void)button;
        (void)down;
#endif
    }

    void addGamepadAxisImpl(UiContextHandle handle, UiGamepadAxis axis, float value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addGamepadAxis(state_, handle, axis, value);
#else
        (void)handle;
        (void)axis;
        (void)value;
#endif
    }

    void addTextImpl(UiContextHandle handle, std::uint32_t codepoint)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addText(state_, handle, codepoint);
#else
        (void)handle;
        (void)codepoint;
#endif
    }

    void addFocusImpl(UiContextHandle handle, bool focused)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::addFocus(state_, handle, focused);
#else
        (void)handle;
        (void)focused;
#endif
    }

    // --- Windows and widgets ----------------------------------------------

    bool beginWindowImpl(UiContextHandle handle, const char *name, bool *open, UiWindowFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginWindow(state_, handle, name, open, flags);
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
        backend::endWindow(state_, handle);
#else
        (void)handle;
#endif
    }

    void textImpl(UiContextHandle handle, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::text(state_, handle, value);
#else
        (void)handle;
        (void)value;
#endif
    }

    void textColoredImpl(UiContextHandle handle, float red, float green, float blue, float alpha, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::textColored(state_, handle, red, green, blue, alpha, value);
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
        backend::separator(state_, handle);
#else
        (void)handle;
#endif
    }

    void sameLineImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::sameLine(state_, handle);
#else
        (void)handle;
#endif
    }

    void spacingImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::spacing(state_, handle);
#else
        (void)handle;
#endif
    }

    bool buttonImpl(UiContextHandle handle, const char *label, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::button(state_, handle, label, width, height);
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
        return backend::checkbox(state_, handle, label, value);
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
        return backend::sliderFloat(state_, handle, label, value, minimum, maximum, format);
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
        return backend::inputText(state_, handle, label, buffer, bufferSize);
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
        return backend::inputInt(state_, handle, label, value, step, fastStep);
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
        return backend::inputFloat(state_, handle, label, value, format);
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
        return backend::sliderInt(state_, handle, label, value, minimum, maximum, format);
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
        return backend::dragFloat(state_, handle, label, value, speed, minimum, maximum, format);
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
        return backend::combo(state_, handle, label, currentItem, items, itemCount);
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
        return backend::selectable(state_, handle, label, selected, width, height);
#else
        (void)handle;
        (void)label;
        (void)selected;
        (void)width;
        (void)height;
        return false;
#endif
    }

    bool selectableExImpl(UiContextHandle handle, const char *label, const bool *selected, float width, float height,
        UiSelectableFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::selectableEx(state_, handle, label, selected, width, height, flags);
#else
        (void)handle;
        (void)label;
        (void)selected;
        (void)width;
        (void)height;
        (void)flags;
        return false;
#endif
    }

    bool radioButtonImpl(UiContextHandle handle, const char *label, std::int32_t *value, std::int32_t buttonValue)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::radioButton(state_, handle, label, value, buttonValue);
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
        backend::progressBar(state_, handle, fraction, width, height, overlay);
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
        backend::image(state_, handle, texture, width, height, uv0x, uv0y, uv1x, uv1y);
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

    void bulletTextImpl(UiContextHandle handle, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::bulletText(state_, handle, value);
#else
        (void)handle;
        (void)value;
#endif
    }

    // --- Layout ------------------------------------------------------------

    bool beginChildImpl(UiContextHandle handle, const char *name, float width, float height, UiChildFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginChild(state_, handle, name, width, height, flags);
#else
        (void)handle;
        (void)name;
        (void)width;
        (void)height;
        (void)flags;
        return false;
#endif
    }

    void endChildImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endChild(state_, handle);
#else
        (void)handle;
#endif
    }

    void beginGroupImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::beginGroup(state_, handle);
#else
        (void)handle;
#endif
    }

    void endGroupImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endGroup(state_, handle);
#else
        (void)handle;
#endif
    }

    void indentImpl(UiContextHandle handle, float width)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::indent(state_, handle, width);
#else
        (void)handle;
        (void)width;
#endif
    }

    void unindentImpl(UiContextHandle handle, float width)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::unindent(state_, handle, width);
#else
        (void)handle;
        (void)width;
#endif
    }

    void newLineImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::newLine(state_, handle);
#else
        (void)handle;
#endif
    }

    void dummyImpl(UiContextHandle handle, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::dummy(state_, handle, width, height);
#else
        (void)handle;
        (void)width;
        (void)height;
#endif
    }

    // --- Tree --------------------------------------------------------------

    bool treeNodeImpl(UiContextHandle handle, const char *label)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::treeNode(state_, handle, label);
#else
        (void)handle;
        (void)label;
        return false;
#endif
    }

    bool treeNodeExImpl(UiContextHandle handle, const char *label, UiTreeNodeFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::treeNodeEx(state_, handle, label, flags);
#else
        (void)handle;
        (void)label;
        (void)flags;
        return false;
#endif
    }

    void treePopImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::treePop(state_, handle);
#else
        (void)handle;
#endif
    }

    void setNextItemOpenImpl(UiContextHandle handle, bool open)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setNextItemOpen(state_, handle, open);
#else
        (void)handle;
        (void)open;
#endif
    }

    bool collapsingHeaderImpl(UiContextHandle handle, const char *label, UiTreeNodeFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::collapsingHeader(state_, handle, label, flags);
#else
        (void)handle;
        (void)label;
        (void)flags;
        return false;
#endif
    }

    bool collapsingHeaderBoolImpl(UiContextHandle handle, const char *label, bool *open, UiTreeNodeFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::collapsingHeaderBool(state_, handle, label, open, flags);
#else
        (void)handle;
        (void)label;
        (void)open;
        (void)flags;
        return false;
#endif
    }

    // --- Disabled ----------------------------------------------------------

    void beginDisabledImpl(UiContextHandle handle, bool disabled)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::beginDisabled(state_, handle, disabled);
#else
        (void)handle;
        (void)disabled;
#endif
    }

    void endDisabledImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endDisabled(state_, handle);
#else
        (void)handle;
#endif
    }

    // --- Popups ------------------------------------------------------------

    void openPopupImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::openPopup(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
#endif
    }

    void openPopupIdImpl(UiContextHandle handle, std::uint32_t identifier, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::openPopupId(state_, handle, identifier, flags);
#else
        (void)handle;
        (void)identifier;
        (void)flags;
#endif
    }

    bool beginPopupImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginPopup(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    bool beginPopupModalImpl(UiContextHandle handle, const char *name, bool *open, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginPopupModal(state_, handle, name, open, flags);
#else
        (void)handle;
        (void)name;
        (void)open;
        (void)flags;
        return false;
#endif
    }

    bool beginPopupContextItemImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginPopupContextItem(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    bool beginPopupContextWindowImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginPopupContextWindow(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    bool beginPopupContextVoidImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginPopupContextVoid(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    void endPopupImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endPopup(state_, handle);
#else
        (void)handle;
#endif
    }

    void closeCurrentPopupImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::closeCurrentPopup(state_, handle);
#else
        (void)handle;
#endif
    }

    bool isPopupOpenImpl(UiContextHandle handle, const char *name, UiPopupFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isPopupOpen(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    // --- Menus -------------------------------------------------------------

    bool beginMenuBarImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginMenuBar(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    void endMenuBarImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endMenuBar(state_, handle);
#else
        (void)handle;
#endif
    }

    bool beginMenuImpl(UiContextHandle handle, const char *label, bool enabled)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginMenu(state_, handle, label, enabled);
#else
        (void)handle;
        (void)label;
        (void)enabled;
        return false;
#endif
    }

    void endMenuImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endMenu(state_, handle);
#else
        (void)handle;
#endif
    }

    bool menuItemImpl(UiContextHandle handle, const char *label, const char *shortcut, bool *selected, bool enabled)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::menuItem(state_, handle, label, shortcut, selected, enabled);
#else
        (void)handle;
        (void)label;
        (void)shortcut;
        (void)selected;
        (void)enabled;
        return false;
#endif
    }

    bool menuItemActivatedImpl(UiContextHandle handle, const char *label, const char *shortcut, bool enabled)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::menuItemActivated(state_, handle, label, shortcut, enabled);
#else
        (void)handle;
        (void)label;
        (void)shortcut;
        (void)enabled;
        return false;
#endif
    }

    // --- Tabs --------------------------------------------------------------

    bool beginTabBarImpl(UiContextHandle handle, const char *name, UiTabBarFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginTabBar(state_, handle, name, flags);
#else
        (void)handle;
        (void)name;
        (void)flags;
        return false;
#endif
    }

    void endTabBarImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endTabBar(state_, handle);
#else
        (void)handle;
#endif
    }

    bool beginTabItemImpl(UiContextHandle handle, const char *label, bool *open, UiTabItemFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginTabItem(state_, handle, label, open, flags);
#else
        (void)handle;
        (void)label;
        (void)open;
        (void)flags;
        return false;
#endif
    }

    void endTabItemImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endTabItem(state_, handle);
#else
        (void)handle;
#endif
    }

    void setTabItemClosedImpl(UiContextHandle handle, const char *label)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setTabItemClosed(state_, handle, label);
#else
        (void)handle;
        (void)label;
#endif
    }

    // --- Tables ------------------------------------------------------------

    bool beginTableImpl(UiContextHandle handle, const char *name, std::uint32_t columns, UiTableFlags flags,
        float outerSizeX, float outerSizeY, float innerWidth)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::beginTable(state_, handle, name, columns, flags, outerSizeX, outerSizeY, innerWidth);
#else
        (void)handle;
        (void)name;
        (void)columns;
        (void)flags;
        (void)outerSizeX;
        (void)outerSizeY;
        (void)innerWidth;
        return false;
#endif
    }

    void endTableImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endTable(state_, handle);
#else
        (void)handle;
#endif
    }

    void tableNextRowImpl(UiContextHandle handle, UiTableRowFlags flags, float minRowHeight)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableNextRow(state_, handle, flags, minRowHeight);
#else
        (void)handle;
        (void)flags;
        (void)minRowHeight;
#endif
    }

    bool tableNextColumnImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableNextColumn(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool tableSetColumnIndexImpl(UiContextHandle handle, std::uint32_t columnIndex)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableSetColumnIndex(state_, handle, columnIndex);
#else
        (void)handle;
        (void)columnIndex;
        return false;
#endif
    }

    void tableSetupColumnImpl(UiContextHandle handle, const char *label, UiTableColumnFlags flags,
        float initWidthOrWeight, std::uint32_t identifier)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableSetupColumn(state_, handle, label, flags, initWidthOrWeight, identifier);
#else
        (void)handle;
        (void)label;
        (void)flags;
        (void)initWidthOrWeight;
        (void)identifier;
#endif
    }

    void tableSetupScrollFreezeImpl(UiContextHandle handle, std::uint32_t columns, std::uint32_t rows)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableSetupScrollFreeze(state_, handle, columns, rows);
#else
        (void)handle;
        (void)columns;
        (void)rows;
#endif
    }

    void tableHeadersRowImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableHeadersRow(state_, handle);
#else
        (void)handle;
#endif
    }

    void tableHeaderImpl(UiContextHandle handle, const char *label)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableHeader(state_, handle, label);
#else
        (void)handle;
        (void)label;
#endif
    }

    std::int32_t tableColumnCountImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableColumnCount(state_, handle);
#else
        (void)handle;
        return 0;
#endif
    }

    std::int32_t tableColumnIndexImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableColumnIndex(state_, handle);
#else
        (void)handle;
        return 0;
#endif
    }

    std::int32_t tableRowIndexImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableRowIndex(state_, handle);
#else
        (void)handle;
        return 0;
#endif
    }

    const char *tableColumnNameImpl(UiContextHandle handle, std::int32_t columnIndex)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableColumnName(state_, handle, columnIndex);
#else
        (void)handle;
        (void)columnIndex;
        return nullptr;
#endif
    }

    bool tableColumnIsEnabledImpl(UiContextHandle handle, std::int32_t columnIndex)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableColumnIsEnabled(state_, handle, columnIndex);
#else
        (void)handle;
        (void)columnIndex;
        return false;
#endif
    }

    bool tableColumnIsSortedImpl(UiContextHandle handle, std::int32_t columnIndex)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::tableColumnIsSorted(state_, handle, columnIndex);
#else
        (void)handle;
        (void)columnIndex;
        return false;
#endif
    }

    void tableSetBgColorImpl(UiContextHandle handle, UiTableBgTarget target, std::uint32_t color)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::tableSetBgColor(state_, handle, target, color);
#else
        (void)handle;
        (void)target;
        (void)color;
#endif
    }

    // --- Tooltips ----------------------------------------------------------

    void setTooltipImpl(UiContextHandle handle, const char *value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setTooltip(state_, handle, value);
#else
        (void)handle;
        (void)value;
#endif
    }

    void beginTooltipImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::beginTooltip(state_, handle);
#else
        (void)handle;
#endif
    }

    void endTooltipImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::endTooltip(state_, handle);
#else
        (void)handle;
#endif
    }

    // --- Vector sliders/drags ---------------------------------------------

    bool sliderFloat2Impl(UiContextHandle handle, const char *label, float values[2], float minimum, float maximum,
        const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderFloatN(state_, handle, label, values, 2, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool sliderFloat3Impl(UiContextHandle handle, const char *label, float values[3], float minimum, float maximum,
        const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderFloatN(state_, handle, label, values, 3, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool sliderFloat4Impl(UiContextHandle handle, const char *label, float values[4], float minimum, float maximum,
        const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderFloatN(state_, handle, label, values, 4, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool sliderInt2Impl(UiContextHandle handle, const char *label, std::int32_t values[2], std::int32_t minimum,
        std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderIntN(state_, handle, label, values, 2, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool sliderInt3Impl(UiContextHandle handle, const char *label, std::int32_t values[3], std::int32_t minimum,
        std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderIntN(state_, handle, label, values, 3, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool sliderInt4Impl(UiContextHandle handle, const char *label, std::int32_t values[4], std::int32_t minimum,
        std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::sliderIntN(state_, handle, label, values, 4, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool dragFloat2Impl(UiContextHandle handle, const char *label, float values[2], float speed, float minimum,
        float maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::dragFloatN(state_, handle, label, values, 2, speed, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)speed;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool dragFloat3Impl(UiContextHandle handle, const char *label, float values[3], float speed, float minimum,
        float maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::dragFloatN(state_, handle, label, values, 3, speed, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)speed;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool dragInt2Impl(UiContextHandle handle, const char *label, std::int32_t values[2], float speed,
        std::int32_t minimum, std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::dragIntN(state_, handle, label, values, 2, speed, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)speed;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    bool dragInt3Impl(UiContextHandle handle, const char *label, std::int32_t values[3], float speed,
        std::int32_t minimum, std::int32_t maximum, const char *format)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::dragIntN(state_, handle, label, values, 3, speed, minimum, maximum, format);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)speed;
        (void)minimum;
        (void)maximum;
        (void)format;
        return false;
#endif
    }

    // --- Color edits -------------------------------------------------------

    bool colorEdit3Impl(UiContextHandle handle, const char *label, float values[3], UiColorEditFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::colorEdit3(state_, handle, label, values, flags);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)flags;
        return false;
#endif
    }

    bool colorEdit4Impl(UiContextHandle handle, const char *label, float values[4], UiColorEditFlags flags)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::colorEdit4(state_, handle, label, values, flags);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)flags;
        return false;
#endif
    }

    bool colorButtonImpl(UiContextHandle handle, const char *descriptionId, const float values[4],
        UiColorEditFlags flags, float width, float height)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::colorButton(state_, handle, descriptionId, values, flags, width, height);
#else
        (void)handle;
        (void)descriptionId;
        (void)values;
        (void)flags;
        (void)width;
        (void)height;
        return false;
#endif
    }

    // --- Plots -------------------------------------------------------------

    void plotLinesImpl(UiContextHandle handle, const char *label, const float *values, std::uint32_t valueCount,
        float valueMinimum, float valueMaximum, const char *overlay, float graphWidth, float graphHeight)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::plotLines(state_, handle, label, values, valueCount, valueMinimum, valueMaximum, overlay,
            graphWidth, graphHeight);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)valueCount;
        (void)valueMinimum;
        (void)valueMaximum;
        (void)overlay;
        (void)graphWidth;
        (void)graphHeight;
#endif
    }

    void plotHistogramImpl(UiContextHandle handle, const char *label, const float *values, std::uint32_t valueCount,
        float valueMinimum, float valueMaximum, const char *overlay, float graphWidth, float graphHeight)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::plotHistogram(state_, handle, label, values, valueCount, valueMinimum, valueMaximum, overlay,
            graphWidth, graphHeight);
#else
        (void)handle;
        (void)label;
        (void)values;
        (void)valueCount;
        (void)valueMinimum;
        (void)valueMaximum;
        (void)overlay;
        (void)graphWidth;
        (void)graphHeight;
#endif
    }

    // --- Item/window state queries ----------------------------------------

    bool isItemHoveredImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemHovered(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemActiveImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemActive(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemFocusedImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemFocused(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemClickedImpl(UiContextHandle handle, std::uint32_t button)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemClicked(state_, handle, button);
#else
        (void)handle;
        (void)button;
        return false;
#endif
    }

    bool isItemVisibleImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemVisible(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemActivatedImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemActivated(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemDeactivatedImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemDeactivated(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isItemDeactivatedAfterEditImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isItemDeactivatedAfterEdit(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isAnyItemHoveredImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isAnyItemHovered(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isAnyItemActiveImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isAnyItemActive(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isWindowAppearingImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isWindowAppearing(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isWindowCollapsedImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isWindowCollapsed(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isWindowFocusedImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isWindowFocused(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    bool isWindowHoveredImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::isWindowHovered(state_, handle);
#else
        (void)handle;
        return false;
#endif
    }

    // --- Window placement --------------------------------------------------

    float windowWidthImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::windowWidth(state_, handle);
#else
        (void)handle;
        return 0.0f;
#endif
    }

    float windowHeightImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::windowHeight(state_, handle);
#else
        (void)handle;
        return 0.0f;
#endif
    }

    float windowPositionXImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::windowPositionX(state_, handle);
#else
        (void)handle;
        return 0.0f;
#endif
    }

    float windowPositionYImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::windowPositionY(state_, handle);
#else
        (void)handle;
        return 0.0f;
#endif
    }

    void setNextWindowFocusImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setNextWindowFocus(state_, handle);
#else
        (void)handle;
#endif
    }

    void setNextWindowCollapsedImpl(UiContextHandle handle, bool collapsed)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setNextWindowCollapsed(state_, handle, collapsed);
#else
        (void)handle;
        (void)collapsed;
#endif
    }

    void setWindowFocusImpl(UiContextHandle handle, const char *name)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setWindowFocus(state_, handle, name);
#else
        (void)handle;
        (void)name;
#endif
    }

    void setWindowCollapsedNamedImpl(UiContextHandle handle, const char *name, bool collapsed)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setWindowCollapsedNamed(state_, handle, name, collapsed);
#else
        (void)handle;
        (void)name;
        (void)collapsed;
#endif
    }

    // --- Mouse cursor ------------------------------------------------------

    void setMouseCursorImpl(UiContextHandle handle, UiMouseCursor cursor)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setMouseCursor(state_, handle, cursor);
#else
        (void)handle;
        (void)cursor;
#endif
    }

    UiMouseCursor mouseCursorImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::mouseCursor(state_, handle);
#else
        (void)handle;
        return UiMouseCursor::None;
#endif
    }

    // --- Fonts -------------------------------------------------------------

    bool addFontFromFileImpl(UiContextHandle handle, const char *filePath, float pixelSize, UiFontHandle *outFont)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::addFontFromFile(state_, handle, filePath, pixelSize, outFont);
#else
        (void)handle;
        (void)filePath;
        (void)pixelSize;
        (void)outFont;
        return false;
#endif
    }

    bool addFontFromMemoryTTFImpl(UiContextHandle handle, const void *data, std::uint32_t dataSize, float pixelSize,
        UiFontHandle *outFont)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::addFontFromMemoryTTF(state_, handle, data, dataSize, pixelSize, outFont);
#else
        (void)handle;
        (void)data;
        (void)dataSize;
        (void)pixelSize;
        (void)outFont;
        return false;
#endif
    }

    bool pushFontImpl(UiContextHandle handle, UiFontHandle fontHandle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::pushFont(state_, handle, fontHandle);
#else
        (void)handle;
        (void)fontHandle;
        return false;
#endif
    }

    void popFontImpl(UiContextHandle handle)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::popFont(state_, handle);
#else
        (void)handle;
#endif
    }

    bool fontAtlasImpl(UiContextHandle handle, UiFontAtlas *outAtlas)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::fontAtlas(state_, handle, outAtlas);
#else
        (void)handle;
        (void)outAtlas;
        return false;
#endif
    }

    bool setFontTextureImpl(UiContextHandle handle, UiTextureHandle texture)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::setFontTexture(state_, handle, texture);
#else
        (void)handle;
        (void)texture;
        return false;
#endif
    }

    // --- Style and window placement ---------------------------------------

    bool pushStyleColorImpl(UiContextHandle handle, UiColor color, float red, float green, float blue, float alpha)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::pushStyleColor(state_, handle, color, red, green, blue, alpha);
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
        backend::popStyleColor(state_, handle, count);
#else
        (void)handle;
        (void)count;
#endif
    }

    bool pushStyleVariableImpl(UiContextHandle handle, UiStyleVariable variable, float value)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        return backend::pushStyleVariable(state_, handle, variable, value);
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
        backend::popStyleVariable(state_, handle, count);
#else
        (void)handle;
        (void)count;
#endif
    }

    void setNextWindowPositionImpl(UiContextHandle handle, float x, float y, float pivotX, float pivotY)
    {
#if defined(ENGINE_UI_BACKEND_IMGUI)
        backend::setNextWindowPosition(state_, handle, x, y, pivotX, pivotY);
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
        backend::setNextWindowSize(state_, handle, width, height);
#else
        (void)handle;
        (void)width;
        (void)height;
#endif
    }

    ParticipantManifest                       manifest_{};
    UiCapability                             capability_{};
    backend::UiModuleState                   state_;
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
