#ifndef ENGINE_MODULES_LUA_SCRIPTING_HPP
#define ENGINE_MODULES_LUA_SCRIPTING_HPP

#include "engine/capabilities/input_contract.hpp"
#include "engine/capabilities/logging_contract.hpp"
#include "engine/capabilities/renderer_contract.hpp"
#include "engine/capabilities/screenshot_contract.hpp"
#include "engine/capabilities/window_contract.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace sol
{
class state;
}

namespace engine
{

class LoggerCapability;
class WindowCapability;
class ScreenshotCapability;

// Module "lua_scripting": hosts Lua 5.4 script mods through sol2. The kernel
// discovers script packages through their mod.toml manifests and hands each
// one over through the ScriptModHost contract; every script receives an
// "engine" table with a permission checked surface: logging, event
// subscription, window queries and, when the screenshot module is present,
// screenshot requests. Each script owns an isolated state; a script error
// fails startup with a precise report instead of silently skipping the mod.
class LuaScriptingModule final : public ModuleInterface, public ScriptModHost
{
public:
    LuaScriptingModule();
    ~LuaScriptingModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void          *capabilityPointerFor(const std::string &capabilityName) override;
    ScriptModHost *scriptModHost() override;

    // ScriptModHost: called by the kernel once per discovered script package.
    void hostScriptMod(const ScriptModDescription &description) override;

    // Script bridge entry points (called from the Lua API surface).
    void                                   logForScripts(LogLevel level, const std::string &message);
    std::tuple<std::int32_t, std::int32_t> windowSizeForScripts();
    void                                   requestScreenshotForScripts();

private:
    bool ensureScreenshotCapability();
    struct ScriptEnvironment
    {
        std::string                 path;
        std::string                 modId;
        std::unique_ptr<sol::state> state;
    };

    void loadScript(const ScriptModDescription &scriptMod);
    void forwardKeyPressed(const KeyPressedEvent &event);
    void forwardKeyReleased(const KeyReleasedEvent &event);
    void dispatchKeyEvent(bool pressed, const std::string &keyName);
    void forwardFrameBegin(const FrameBeginEvent &event);

    ComponentManifest     componentManifest;
    ComponentContext     *boundContext         = nullptr;
    LoggerCapability     *logger               = nullptr;
    WindowCapability     *windowCapability     = nullptr;
    ScreenshotCapability *screenshotCapability = nullptr;
    // Tri-state screenshot capability resolution: 0 unresolved, 1 available,
    // -1 decided unavailable (module compiled out or permission denied).
    int                               screenshotResolutionState = 0;
    EventSubscription                 keyPressedSubscription;
    EventSubscription                 keyReleasedSubscription;
    EventSubscription                 frameBeginSubscription;
    std::vector<ScriptModDescription> pendingScripts;
    std::vector<ScriptEnvironment>    environments;
};

} // namespace engine

#endif // ENGINE_MODULES_LUA_SCRIPTING_HPP
