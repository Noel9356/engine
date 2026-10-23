#include "lua_scripting.hpp"

#include <sol/sol.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <tuple>

namespace engine
{

namespace
{

// Bridge surface installed into every script state. Handlers live in plain
// Lua tables inside the state, so no sol types leak into the module header and
// no per event heap traffic happens outside the state.
//
// The engine table is the entire script mod ABI. It must stay implementation-,
// platform- and vendor-agnostic by construction: capability level concepts
// only (log lines, key names, window sizes, capture requests). A script
// written against this table runs unchanged on any operating system, any
// windowing backend and any rendering implementation - the renderer can be
// made from Vulkan or OpenGL without a script mod being able to tell.
// Native (C++) mods are not held to this: they may consume engine internals
// directly, at the cost of portability (mods/README.md, section "Portability").
void installEngineBridge(
    sol::state &lua, LuaScriptingModule &module
)
{
    sol::table engineTable = lua.create_named_table("engine");

    engineTable.set_function(
        "log_trace", [&module](const std::string &message) { module.logForScripts(LogLevel::Trace, message); }
    );
    engineTable.set_function(
        "log_debug", [&module](const std::string &message) { module.logForScripts(LogLevel::Debug, message); }
    );
    engineTable.set_function(
        "log_information",
        [&module](const std::string &message) { module.logForScripts(LogLevel::Information, message); }
    );
    engineTable.set_function(
        "log_warning", [&module](const std::string &message) { module.logForScripts(LogLevel::Warning, message); }
    );
    engineTable.set_function(
        "log_error", [&module](const std::string &message) { module.logForScripts(LogLevel::Error, message); }
    );

    engineTable["key_pressed_handlers"]  = lua.create_table();
    engineTable["key_released_handlers"] = lua.create_table();
    engineTable["frame_begin_handlers"]  = lua.create_table();

    engineTable.set_function(
        "subscribe_key_pressed",
        [&lua](sol::function handler)
        {
            sol::table handlers = lua["engine"]["key_pressed_handlers"];
            handlers.add(handler);
        }
    );
    engineTable.set_function(
        "subscribe_key_released",
        [&lua](sol::function handler)
        {
            sol::table handlers = lua["engine"]["key_released_handlers"];
            handlers.add(handler);
        }
    );
    engineTable.set_function(
        "subscribe_frame_begin",
        [&lua](sol::function handler)
        {
            sol::table handlers = lua["engine"]["frame_begin_handlers"];
            handlers.add(handler);
        }
    );

    engineTable.set_function("window_size", [&module]() { return module.windowSizeForScripts(); });

    engineTable.set_function("request_screenshot", [&module]() { module.requestScreenshotForScripts(); });
}

} // namespace

LuaScriptingModule::LuaScriptingModule()
{
    componentManifest.name                 = "lua_scripting";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {};
    componentManifest.requiredCapabilities = {"logging", "window"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.window", "capability.screenshot"};
}

LuaScriptingModule::~LuaScriptingModule()
{
    stop();
}

void LuaScriptingModule::construct(
    ComponentContext &context
)
{
    boundContext     = &context;
    logger           = &context.acquireCapability<LoggerCapability>();
    windowCapability = &context.acquireCapability<WindowCapability>();

    // The screenshot capability is deliberately NOT resolved here. This module
    // only requires logging and window, so the topological order may construct
    // it BEFORE the screenshot module registers its capability; an acquire at
    // construct time would then cache a permanent failure and script mods
    // would never be able to request captures (the F11 path of the example
    // mod was silently dead this way). resolveScreenshotCapability() retries
    // at first call, which happens inside the frame loop - after every
    // component has constructed.

    // Script mods are discovered by the kernel through their mod.toml files
    // and handed over one by one through the ScriptModHost contract during the
    // construct phase.

    EventBus                      &bus            = context.eventBus();
    EventChannel<KeyPressedEvent> &pressedChannel = bus.channel<KeyPressedEvent>();
    keyPressedSubscription                        = EventSubscription(
        &bus,
        typeid(KeyPressedEvent),
        pressedChannel.subscribe<LuaScriptingModule, &LuaScriptingModule::forwardKeyPressed>(this)
    );

    EventChannel<KeyReleasedEvent> &releasedChannel = bus.channel<KeyReleasedEvent>();
    keyReleasedSubscription                         = EventSubscription(
        &bus,
        typeid(KeyReleasedEvent),
        releasedChannel.subscribe<LuaScriptingModule, &LuaScriptingModule::forwardKeyReleased>(this)
    );

    // Frame events are renderer-driven: with no renderer module in the build
    // nothing publishes FrameBeginEvent and the handlers stay inert.
    EventChannel<FrameBeginEvent> &frameChannel = bus.channel<FrameBeginEvent>();
    frameBeginSubscription                      = EventSubscription(
        &bus,
        typeid(FrameBeginEvent),
        frameChannel.subscribe<LuaScriptingModule, &LuaScriptingModule::forwardFrameBegin>(this)
    );
}

ScriptModHost *LuaScriptingModule::scriptModHost()
{
    return this;
}

void LuaScriptingModule::hostScriptMod(
    const ScriptModDescription &description
)
{
    pendingScripts.push_back(description);
}

void LuaScriptingModule::start()
{
    for (const ScriptModDescription &scriptMod : pendingScripts)
    {
        loadScript(scriptMod);
    }
    logger->logMessage(
        LogLevel::Information,
        "lua scripting module started with " + std::to_string(environments.size()) + " script mod(s)"
    );
}

void LuaScriptingModule::stop()
{
    keyPressedSubscription.release();
    keyReleasedSubscription.release();
    frameBeginSubscription.release();
    environments.clear();
    pendingScripts.clear();
    boundContext         = nullptr;
    windowCapability     = nullptr;
    screenshotCapability = nullptr;
    logger               = nullptr;
}

void *LuaScriptingModule::capabilityPointerFor(
    const std::string & /*capabilityName*/
)
{
    return nullptr;
}

void LuaScriptingModule::logForScripts(
    LogLevel level, const std::string &message
)
{
    if (logger != nullptr)
    {
        logger->logMessage(level, message);
    }
}

std::tuple<std::int32_t, std::int32_t> LuaScriptingModule::windowSizeForScripts()
{
    if (windowCapability == nullptr)
    {
        return {0, 0};
    }
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return {0, 0};
    }
    return {
        static_cast<std::int32_t>(windowCapability->framebufferWidth(identifiers.front())),
        static_cast<std::int32_t>(windowCapability->framebufferHeight(identifiers.front()))
    };
}

bool LuaScriptingModule::ensureScreenshotCapability()
{
    if (screenshotResolutionState != 0) // 1 = available, -1 = decided unavailable
    {
        return screenshotResolutionState == 1;
    }
    if (boundContext == nullptr)
    {
        return false;
    }
    try
    {
        screenshotCapability      = &boundContext->acquireCapability<ScreenshotCapability>();
        screenshotResolutionState = 1;
    }
    catch (const std::exception &)
    {
        // The screenshot module is not part of this build (or its permission
        // is denied). The decision happens at first CALL time - every
        // component has constructed by then - so "unavailable" here is final;
        // caching it keeps a per-frame requester from spamming the log.
        screenshotCapability      = nullptr;
        screenshotResolutionState = -1;
    }
    return screenshotResolutionState == 1;
}

void LuaScriptingModule::requestScreenshotForScripts()
{
    if (!ensureScreenshotCapability())
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "lua scripting module: engine.request_screenshot called but the screenshot module is "
                "not part of this build"
            );
        }
        return;
    }
    screenshotCapability->requestScreenshot();
}

void LuaScriptingModule::loadScript(
    const ScriptModDescription &scriptMod
)
{
    const std::filesystem::path &resolved = scriptMod.scriptPath;
    if (!std::filesystem::exists(resolved))
    {
        throw std::runtime_error(
            "lua scripting module: script '" + resolved.string() + "' of mod '" + scriptMod.modId + "' does not exist"
        );
    }

    ScriptEnvironment environment;
    environment.path  = resolved.string();
    environment.modId = scriptMod.modId;
    environment.state = std::make_unique<sol::state>();
    sol::state &lua   = *environment.state;
    lua.open_libraries(sol::lib::base, sol::lib::package, sol::lib::math, sol::lib::string, sol::lib::table);

    installEngineBridge(lua, *this);

    const sol::protected_function_result result = lua.safe_script_file(resolved.string());
    if (!result.valid())
    {
        const sol::error error = result;
        throw std::runtime_error("lua scripting module: script '" + resolved.string() + "' failed: " + error.what());
    }
    logger->logMessage(LogLevel::Information, "lua scripting module: loaded '" + resolved.string() + "'");
    environments.push_back(std::move(environment));
}

void LuaScriptingModule::forwardKeyPressed(
    const KeyPressedEvent &event
)
{
    dispatchKeyEvent(true, keyCodeName(event.keyCode));
}

void LuaScriptingModule::forwardKeyReleased(
    const KeyReleasedEvent &event
)
{
    dispatchKeyEvent(false, keyCodeName(event.keyCode));
}

void LuaScriptingModule::dispatchKeyEvent(
    bool pressed, const std::string &keyName
)
{
    for (ScriptEnvironment &environment : environments)
    {
        sol::table handlers = (*environment
                .state)["engine"][pressed ? "key_pressed_handlers" : "key_released_handlers"];
        for (std::size_t index = 1; index <= handlers.size(); ++index)
        {
            sol::protected_function handler = handlers[index];
            if (!handler.valid())
            {
                continue;
            }
            const sol::protected_function_result result = handler(keyName);
            if (!result.valid())
            {
                const sol::error error = result;
                logger->logMessage(
                    LogLevel::Error,
                    "lua scripting module: handler error in '" + environment.path + "' (mod '" + environment.modId
                        + "'): " + error.what()
                );
            }
        }
    }
}

void LuaScriptingModule::forwardFrameBegin(
    const FrameBeginEvent &event
)
{
    for (ScriptEnvironment &environment : environments)
    {
        sol::table handlers = (*environment.state)["engine"]["frame_begin_handlers"];
        for (std::size_t index = 1; index <= handlers.size(); ++index)
        {
            sol::protected_function handler = handlers[index];
            if (!handler.valid())
            {
                continue;
            }
            const sol::protected_function_result result = handler(event.frameIndex);
            if (!result.valid())
            {
                const sol::error error = result;
                logger->logMessage(
                    LogLevel::Error,
                    "lua scripting module: frame handler error in '" + environment.path + "' (mod '" + environment.modId
                        + "'): " + error.what()
                );
            }
        }
    }
}

} // namespace engine
