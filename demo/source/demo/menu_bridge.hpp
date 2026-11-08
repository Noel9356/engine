#ifndef DEMO_MENU_BRIDGE_HPP
#define DEMO_MENU_BRIDGE_HPP

#include <engine/renderer/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>
#include <engine/ui/capabilities.hpp>

#include <cstdint>
#include <vector>

namespace ui             = engine::ui;
namespace renderer         = engine::renderer;
namespace script_runtime = engine::script_runtime;

namespace demo
{

// C++ side of the main_menu Lua bridge. The mod's window layout and FPS
// statistics live in Lua; everything that needs the ui and renderer capabilities
// lives here. create() prepares the ui context and font texture, and
// registerNatives() installs the "menu_*" functions the Lua script calls while
// drawing its Performance window.
class MenuBridge
{
public:
    bool create(const ui::UiCapability &uiCapability, const renderer::RenderCapability &renderCapability,
        renderer::RendererHandle renderer);

    void destroy() noexcept;

    bool registerNatives(
        script_runtime::ScriptRuntimeCapability &scriptRuntime, script_runtime::ScriptRuntimeHandle runtime);

private:
    using NativeArguments = const script_runtime::ScriptValue *;

    static bool beginFrameNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool endFrameNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool setNextWindowPositionNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool setNextWindowSizeNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool beginWindowNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool endWindowNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);
    static bool textNative(void *userData, NativeArguments arguments, std::uint32_t argumentCount,
        script_runtime::ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);

    const ui::UiCapability         *ui_          = nullptr;
    const renderer::RenderCapability *render_      = nullptr;
    renderer::RendererHandle          renderer_    = renderer::invalidRenderer;
    ui::UiContextHandle             context_     = ui::invalidUiContext;
    renderer::TextureHandle           fontTexture_ = renderer::invalidTexture;
    bool                            frameActive_ = false;

    std::vector<renderer::OverlayVertex>      vertices_;
    std::vector<std::uint32_t>              indices_;
    std::vector<renderer::OverlayDrawCommand> commands_;
};

} // namespace demo

#endif
