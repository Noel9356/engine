#include "menu_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

namespace demo
{
namespace
{

// The menu window is a fixed, frameless overlay: the Lua script positions and
// sizes it, and these flags keep ImGui from adding chrome or remembering state.
ui::UiWindowFlags menuWindowFlags() noexcept
{
    using ui::UiWindowFlags;
    constexpr std::uint32_t bits
        = static_cast<std::uint32_t>(UiWindowFlags::NoTitleBar) | static_cast<std::uint32_t>(UiWindowFlags::NoResize)
        | static_cast<std::uint32_t>(UiWindowFlags::NoMove) | static_cast<std::uint32_t>(UiWindowFlags::NoScrollbar)
        | static_cast<std::uint32_t>(UiWindowFlags::NoCollapse)
        | static_cast<std::uint32_t>(UiWindowFlags::NoSavedSettings);
    return static_cast<UiWindowFlags>(bits);
}

// Argument marshalling helpers. Every native checks its arguments before use so
// a mistake on the Lua side fails the call instead of crashing the engine.

bool numberArgument(
    const script_runtime::ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index, float &outValue)
{
    if (arguments == nullptr || index >= argumentCount
        || arguments[index].type != script_runtime::ScriptValueType::Number
        || !std::isfinite(arguments[index].numberValue))
    {
        return false;
    }
    outValue = static_cast<float>(arguments[index].numberValue);
    return true;
}

bool textArgument(const script_runtime::ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    const char *&outValue)
{
    if (arguments == nullptr || index >= argumentCount
        || arguments[index].type != script_runtime::ScriptValueType::String || arguments[index].stringData == nullptr)
    {
        return false;
    }
    outValue = arguments[index].stringData;
    return true;
}

bool finishWithBoolean(
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount, bool value)
{
    if (outResultCount != nullptr)
        *outResultCount = resultCapacity != 0 ? 1u : 0u;
    if (results == nullptr || resultCapacity == 0)
        return false;
    results[0].type         = script_runtime::ScriptValueType::Boolean;
    results[0].booleanValue = value;
    return true;
}

} // namespace

bool MenuBridge::create(const ui::UiCapability &uiCapability, const renderer::RenderCapability &renderCapability,
    renderer::RendererHandle renderer)
{
    destroy();
    if (renderer == renderer::invalidRenderer || uiCapability.createContext == nullptr
        || uiCapability.destroyContext == nullptr || uiCapability.beginFrame == nullptr
        || uiCapability.endFrame == nullptr || uiCapability.drawData == nullptr || uiCapability.fontAtlas == nullptr
        || uiCapability.setFontTexture == nullptr || uiCapability.setNextWindowPosition == nullptr
        || uiCapability.setNextWindowSize == nullptr || uiCapability.beginWindow == nullptr
        || uiCapability.endWindow == nullptr || uiCapability.text == nullptr
        || renderCapability.createTexture == nullptr || renderCapability.destroyTexture == nullptr
        || renderCapability.drawOverlay == nullptr)
    {
        return false;
    }

    ui::UiContextHandle context = ui::invalidUiContext;
    {
        ui::UiContextDescription contextDescription{};
        ui::UiError              uiError = ui::UiError::None;
        context                          = uiCapability.createContext(uiCapability.self, &contextDescription, &uiError);
        if (context == ui::invalidUiContext)
            return false;
    }

    // The ui module rasterizes its default font into an atlas; it becomes an
    // ordinary render texture that the overlay pipeline samples.
    ui::UiFontAtlas atlas{};
    if (!uiCapability.fontAtlas(uiCapability.self, context, &atlas) || atlas.pixels == nullptr || atlas.width == 0
        || atlas.height == 0 || atlas.bytesPerPixel != 4)
    {
        uiCapability.destroyContext(uiCapability.self, context);
        return false;
    }

    const renderer::TextureDescription textureDescription{
        .width          = atlas.width,
        .height         = atlas.height,
        .pixels         = atlas.pixels,
        .filter         = renderer::TextureFilter::Linear,
        .address        = renderer::TextureAddress::ClampEdge,
        .pixelFormat    = renderer::TexturePixelFormat::RGBA8,
        .origin         = renderer::TextureOrigin::TopLeft,
        .rowStrideBytes = atlas.width * atlas.bytesPerPixel,
    };
    renderer::RenderError         renderError = renderer::RenderError::None;
    const renderer::TextureHandle fontTexture
        = renderCapability.createTexture(renderCapability.self, renderer, &textureDescription, &renderError);
    if (fontTexture == renderer::invalidTexture)
    {
        uiCapability.destroyContext(uiCapability.self, context);
        return false;
    }
    if (!uiCapability.setFontTexture(uiCapability.self, context, static_cast<ui::UiTextureHandle>(fontTexture)))
    {
        renderCapability.destroyTexture(renderCapability.self, renderer, fontTexture);
        uiCapability.destroyContext(uiCapability.self, context);
        return false;
    }

    ui_          = &uiCapability;
    render_      = &renderCapability;
    renderer_    = renderer;
    context_     = context;
    fontTexture_ = fontTexture;
    return true;
}

void MenuBridge::destroy() noexcept
{
    if (ui_ != nullptr && context_ != ui::invalidUiContext)
    {
        // A script error could have abandoned an active frame; close it so the
        // ui backend does not observe a context destroyed mid-frame.
        if (frameActive_ && ui_->endFrame != nullptr)
        {
            ui::UiError uiError = ui::UiError::None;
            (void)ui_->endFrame(ui_->self, context_, &uiError);
            frameActive_ = false;
        }
        if (ui_->destroyContext != nullptr)
            ui_->destroyContext(ui_->self, context_);
    }
    if (render_ != nullptr && renderer_ != renderer::invalidRenderer && fontTexture_ != renderer::invalidTexture
        && render_->destroyTexture != nullptr)
    {
        render_->destroyTexture(render_->self, renderer_, fontTexture_);
    }
    ui_          = nullptr;
    render_      = nullptr;
    renderer_    = renderer::invalidRenderer;
    context_     = ui::invalidUiContext;
    fontTexture_ = renderer::invalidTexture;
    vertices_.clear();
    indices_.clear();
    commands_.clear();
}

bool MenuBridge::registerNatives(
    script_runtime::ScriptRuntimeCapability &scriptRuntime, script_runtime::ScriptRuntimeHandle runtime)
{
    if (runtime == script_runtime::invalidScriptRuntime)
        return false;
    struct NativeBinding
    {
        const char                          *name;
        script_runtime::ScriptNativeFunction function;
    };
    static const NativeBinding bindings[] = {
        {"menu_begin_frame", &MenuBridge::beginFrameNative},
        {"menu_end_frame", &MenuBridge::endFrameNative},
        {"menu_set_next_window_position", &MenuBridge::setNextWindowPositionNative},
        {"menu_set_next_window_size", &MenuBridge::setNextWindowSizeNative},
        {"menu_begin_window", &MenuBridge::beginWindowNative},
        {"menu_end_window", &MenuBridge::endWindowNative},
        {"menu_text", &MenuBridge::textNative},
    };
    for (const NativeBinding &binding : bindings)
    {
        if (!scriptRuntime.registerNativeFunction(scriptRuntime.self, runtime, binding.name, binding.function, this))
        {
            return false;
        }
    }
    return true;
}

bool MenuBridge::beginFrameNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    auto *bridge            = static_cast<MenuBridge *>(userData);
    float displayWidth      = 0.0f;
    float displayHeight     = 0.0f;
    float framebufferScaleX = 0.0f;
    float framebufferScaleY = 0.0f;
    float deltaSeconds      = 0.0f;
    bool  started           = false;
    if (bridge != nullptr && bridge->ui_ != nullptr && !bridge->frameActive_ && bridge->context_ != ui::invalidUiContext
        && numberArgument(arguments, argumentCount, 0, displayWidth)
        && numberArgument(arguments, argumentCount, 1, displayHeight)
        && numberArgument(arguments, argumentCount, 2, framebufferScaleX)
        && numberArgument(arguments, argumentCount, 3, framebufferScaleY)
        && numberArgument(arguments, argumentCount, 4, deltaSeconds) && displayWidth > 0.0f && displayHeight > 0.0f
        && framebufferScaleX > 0.0f && framebufferScaleY > 0.0f)
    {
        const double safeDeltaSeconds
            = std::isfinite(deltaSeconds) ? std::clamp<double>(deltaSeconds, 1.0e-3, 0.25) : (1.0 / 60.0);
        const ui::UiFrameDescription frame{
            .displayWidth      = displayWidth,
            .displayHeight     = displayHeight,
            .framebufferScaleX = framebufferScaleX,
            .framebufferScaleY = framebufferScaleY,
            .deltaSeconds      = static_cast<float>(safeDeltaSeconds)};
        ui::UiError uiError  = ui::UiError::None;
        started              = bridge->ui_->beginFrame(bridge->ui_->self, bridge->context_, &frame, &uiError);
        bridge->frameActive_ = started;
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, started);
}

bool MenuBridge::endFrameNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    (void)arguments;
    (void)argumentCount;
    auto *bridge    = static_cast<MenuBridge *>(userData);
    bool  submitted = false;
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->render_ != nullptr && bridge->frameActive_
        && bridge->context_ != ui::invalidUiContext && bridge->renderer_ != renderer::invalidRenderer)
    {
        bridge->frameActive_ = false;
        ui::UiError uiError  = ui::UiError::None;
        if (bridge->ui_->endFrame(bridge->ui_->self, bridge->context_, &uiError))
        {
            const ui::UiDrawData *drawData = bridge->ui_->drawData(bridge->ui_->self, bridge->context_);
            if (drawData != nullptr && drawData->vertices != nullptr && drawData->indices != nullptr
                && drawData->commands != nullptr && drawData->vertexCount != 0 && drawData->indexCount != 0
                && drawData->commandCount != 0)
            {
                try
                {
                    // Translate the ui draw list into the overlay format the
                    // renderer module consumes, then submit it in one call.
                    bridge->vertices_.resize(drawData->vertexCount);
                    bridge->indices_.assign(drawData->indices, drawData->indices + drawData->indexCount);
                    bridge->commands_.resize(drawData->commandCount);
                    for (std::uint32_t index = 0; index < drawData->vertexCount; ++index)
                    {
                        const ui::UiVertex    &source = drawData->vertices[index];
                        renderer::OverlayVertex &target = bridge->vertices_[index];
                        target.position[0]            = source.position[0];
                        target.position[1]            = source.position[1];
                        target.uv[0]                  = source.uv[0];
                        target.uv[1]                  = source.uv[1];
                        target.color                  = source.color;
                    }
                    for (std::uint32_t index = 0; index < drawData->commandCount; ++index)
                    {
                        const ui::UiDrawCommand    &source = drawData->commands[index];
                        renderer::OverlayDrawCommand &target = bridge->commands_[index];
                        target.indexOffset                 = source.indexOffset;
                        target.vertexOffset                = source.vertexOffset;
                        target.elementCount                = source.elementCount;
                        std::copy_n(source.clipRectangle, 4, target.clipRectangle);
                        target.texture          = source.texture <= std::numeric_limits<renderer::TextureHandle>::max()
                                                    ? static_cast<renderer::TextureHandle>(source.texture)
                                                    : renderer::invalidTexture;
                        target.resetRenderState = source.resetRenderState;
                    }
                    const renderer::OverlayDrawData overlay{
                        .vertices         = bridge->vertices_.data(),
                        .vertexCount      = static_cast<std::uint32_t>(bridge->vertices_.size()),
                        .indices          = bridge->indices_.data(),
                        .indexCount       = static_cast<std::uint32_t>(bridge->indices_.size()),
                        .commands         = bridge->commands_.data(),
                        .commandCount     = static_cast<std::uint32_t>(bridge->commands_.size()),
                        .displayPosition  = {drawData->displayPosition[0], drawData->displayPosition[1]},
                        .displaySize      = {drawData->displaySize[0], drawData->displaySize[1]},
                        .framebufferScale = {drawData->framebufferScale[0], drawData->framebufferScale[1]},
                    };
                    bridge->render_->drawOverlay(bridge->render_->self, bridge->renderer_, &overlay);
                    submitted = true;
                }
                catch (...)
                {
                    bridge->vertices_.clear();
                    bridge->indices_.clear();
                    bridge->commands_.clear();
                }
            }
            else
            {
                // An empty draw list is legitimate (nothing visible this frame).
                submitted = true;
            }
        }
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, submitted);
}

bool MenuBridge::setNextWindowPositionNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    auto *bridge = static_cast<MenuBridge *>(userData);
    float x      = 0.0f;
    float y      = 0.0f;
    float pivotX = 0.0f;
    float pivotY = 0.0f;
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->frameActive_
        && numberArgument(arguments, argumentCount, 0, x) && numberArgument(arguments, argumentCount, 1, y)
        && numberArgument(arguments, argumentCount, 2, pivotX) && numberArgument(arguments, argumentCount, 3, pivotY))
    {
        bridge->ui_->setNextWindowPosition(bridge->ui_->self, bridge->context_, x, y, pivotX, pivotY);
        return finishWithBoolean(results, resultCapacity, outResultCount, true);
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, false);
}

bool MenuBridge::setNextWindowSizeNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    auto *bridge = static_cast<MenuBridge *>(userData);
    float width  = 0.0f;
    float height = 0.0f;
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->frameActive_
        && numberArgument(arguments, argumentCount, 0, width) && numberArgument(arguments, argumentCount, 1, height)
        && width > 0.0f && height > 0.0f)
    {
        bridge->ui_->setNextWindowSize(bridge->ui_->self, bridge->context_, width, height);
        return finishWithBoolean(results, resultCapacity, outResultCount, true);
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, false);
}

bool MenuBridge::beginWindowNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    auto       *bridge = static_cast<MenuBridge *>(userData);
    const char *title  = nullptr;
    bool        opened = false;
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->frameActive_
        && textArgument(arguments, argumentCount, 0, title))
    {
        opened = bridge->ui_->beginWindow(bridge->ui_->self, bridge->context_, title, nullptr, menuWindowFlags());
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, opened);
}

bool MenuBridge::endWindowNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    (void)arguments;
    (void)argumentCount;
    auto *bridge = static_cast<MenuBridge *>(userData);
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->frameActive_)
    {
        bridge->ui_->endWindow(bridge->ui_->self, bridge->context_);
        return finishWithBoolean(results, resultCapacity, outResultCount, true);
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, false);
}

bool MenuBridge::textNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
    script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount)
{
    auto       *bridge = static_cast<MenuBridge *>(userData);
    const char *value  = nullptr;
    if (bridge != nullptr && bridge->ui_ != nullptr && bridge->frameActive_
        && textArgument(arguments, argumentCount, 0, value))
    {
        bridge->ui_->text(bridge->ui_->self, bridge->context_, value);
        return finishWithBoolean(results, resultCapacity, outResultCount, true);
    }
    return finishWithBoolean(results, resultCapacity, outResultCount, false);
}

} // namespace demo
