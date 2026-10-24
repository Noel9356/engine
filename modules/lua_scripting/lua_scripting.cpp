#include "lua_scripting.hpp"

#include <sol/sol.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <tuple>

namespace engine
{

namespace
{

// Table field helpers for the bridge's object/light descriptions. Tables
// from Lua: { position = {0, 1, 2}, radius = 0.5 }. Every field is optional -
// the fallbacks match the SceneObjectDescriptor / light defaults, so a
// minimal { type = "sphere" } table spawns a valid object.
std::array<float, 3> vec3Field(
    const sol::table &table, const char *key, const std::array<float, 3> &fallback
)
{
    const sol::object value = table[key];
    if (value.get_type() == sol::type::table)
    {
        const sol::table components = value;
        return {
            components[1].get_or(0.0f),
            components[2].get_or(0.0f),
            components[3].get_or(0.0f),
        };
    }
    return fallback;
}

double numberField(
    const sol::table &table, const char *key, double fallback
)
{
    const sol::object value = table[key];
    if (value.get_type() == sol::type::number)
    {
        return value.as<double>();
    }
    return fallback;
}

std::string stringField(
    const sol::table &table, const char *key
)
{
    const sol::object value = table[key];
    if (value.get_type() == sol::type::string)
    {
        return value.as<std::string>();
    }
    return {};
}

// This module's own capability name, the implementation-bound member of the
// "scripting" family: it appears in the manifest, the plugin descriptor and
// the capability pointer dispatch below, so it is named once here (the kernel
// only ever receives it as data).
constexpr const char *scriptingCapabilityName = "scripting.lua";
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
    engineTable["text_input_handlers"]   = lua.create_table();

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
    engineTable.set_function(
        "subscribe_text_input",
        [&lua](sol::function handler)
        {
            sol::table handlers = lua["engine"]["text_input_handlers"];
            handlers.add(handler);
        }
    );

    engineTable.set_function("window_size", [&module]() { return module.windowSizeForScripts(); });

    engineTable.set_function("request_screenshot", [&module]() { module.requestScreenshotForScripts(); });

    // Input polling: names over keycodes ("W", "Escape", "left"), because
    // names are what the bridge can check against the platform-agnostic key
    // table and what a script author actually writes.
    engineTable.set_function("is_key_down", [&module](const std::string &keyName)
                             { return module.isKeyDownForScripts(keyName); });
    engineTable.set_function("is_mouse_down", [&module](const std::string &buttonName)
                             { return module.isMouseButtonDownForScripts(buttonName); });
    engineTable.set_function(
        "consume_mouse_delta",
        [&module]()
        {
            double x = 0.0;
            double y = 0.0;
            if (module.consumeMouseDeltaForScripts(x, y))
            {
                return std::make_tuple(x, y);
            }
            return std::make_tuple(0.0, 0.0);
        }
    );
    engineTable.set_function(
        "consume_scroll_delta",
        [&module]()
        {
            double x = 0.0;
            double y = 0.0;
            if (module.consumeScrollDeltaForScripts(x, y))
            {
                return std::make_tuple(x, y);
            }
            return std::make_tuple(0.0, 0.0);
        }
    );
    engineTable.set_function("action_strength", [&module](const std::string &actionName)
                             { return module.actionStrengthForScripts(actionName); });

    // Scene objects: the runtime registry (capability "scene").
    engineTable.set_function("spawn_object", [&module](const sol::table &description)
                             { return module.spawnObjectForScripts(description); });
    engineTable.set_function("remove_object", [&module](std::uint32_t handle)
                             { return module.removeObjectForScripts(handle); });
    engineTable.set_function("set_object_position", [&module](std::uint32_t handle, float x, float y, float z)
                             { return module.setObjectPositionForScripts(handle, x, y, z); });
    engineTable.set_function("set_object_rotation", [&module](std::uint32_t handle, float rotation)
                             { return module.setObjectRotationForScripts(handle, rotation); });
    engineTable.set_function("set_object_material", [&module](std::uint32_t handle, const sol::table &material)
                             { return module.setObjectMaterialForScripts(handle, material); });
    engineTable.set_function("object_count", [&module]() { return module.objectCountForScripts(); });

    // Lights (capability "lighting").
    engineTable.set_function("set_sun_light", [&module](const sol::table &light)
                             { module.setSunLightForScripts(light); });
    engineTable.set_function("clear_sun_light", [&module]() { module.clearSunLightForScripts(); });
    engineTable.set_function("add_point_light", [&module](const sol::table &light)
                             { return module.addPointLightForScripts(light); });
    engineTable.set_function("remove_point_light", [&module](std::uint32_t handle)
                             { return module.removePointLightForScripts(handle); });

    // Camera (capability "renderer" - the pose contract lives there).
    engineTable.set_function("camera_position", [&module]() { return module.cameraPositionForScripts(); });
    engineTable.set_function("camera_look_at", [&module]() { return module.cameraLookAtForScripts(); });
    engineTable.set_function("set_camera_position", [&module](float x, float y, float z)
                             { module.setCameraPositionForScripts(x, y, z); });
    engineTable.set_function("set_camera_look_at", [&module](float x, float y, float z)
                             { module.setCameraLookAtForScripts(x, y, z); });

    // Timers (self-contained; ticked between frames).
    engineTable.set_function("create_timer", [&module](double interval, bool repeating, sol::function callback)
                             { return module.createTimerForScripts(interval, repeating, std::move(callback)); });
    engineTable.set_function("cancel_timer", [&module](std::uint64_t handle)
                             { return module.cancelTimerForScripts(handle); });

    // Tweens (capability "animation").
    engineTable.set_function("animate_object", [&module](std::uint32_t handle, const sol::table &tween)
                             { return module.animateObjectForScripts(handle, tween); });
    engineTable.set_function("animate_light", [&module](std::uint32_t handle, const sol::table &tween)
                             { return module.animateLightForScripts(handle, tween); });
    engineTable.set_function("cancel_tween", [&module](std::uint64_t handle)
                             { return module.cancelTweenForScripts(handle); });

    // Content import (capability "asset").
    engineTable.set_function("import_gltf",
                             [&module](const std::string &path, sol::object fidelity)
                             {
                                 return module.importGltfForScripts(
                                     path, fidelity.valid() ? fidelity.as<std::string>() : std::string("mesh"));
                             });

    // Physics (capability "physics").
    engineTable.set_function("add_physics_body", [&module](std::uint32_t handle, const sol::table &body)
                             { return module.addPhysicsBodyForScripts(handle, body); });
    engineTable.set_function("remove_physics_body", [&module](std::uint32_t handle)
                             { return module.removePhysicsBodyForScripts(handle); });
    engineTable.set_function("set_body_velocity", [&module](std::uint32_t handle, float x, float y, float z)
                             { return module.setBodyVelocityForScripts(handle, x, y, z); });
    engineTable.set_function("body_velocity", [&module](std::uint32_t handle, sol::this_state state)
                             { return module.bodyVelocityForScripts(handle, state); });
    engineTable.set_function("set_gravity", [&module](float x, float y, float z)
                             { module.setGravityForScripts(x, y, z); });
    engineTable.set_function("body_count", [&module]() { return module.physicsBodyCountForScripts(); });
    engineTable.set_function("set_body_sleeping", [&module](std::uint32_t handle, bool sleeping)
                             { return module.setBodySleepingForScripts(handle, sleeping); });
    engineTable.set_function("is_body_awake", [&module](std::uint32_t handle)
                             { return module.isBodyAwakeForScripts(handle); });
    engineTable.set_function("raycast",
                             [&module](float ox, float oy, float oz, float dx, float dy, float dz, sol::object maxDistance,
                                       sol::this_state state)
                             {
                                 return module.raycastForScripts(
                                     ox, oy, oz, dx, dy, dz,
                                     maxDistance.valid() ? static_cast<float>(maxDistance.as<double>()) : 100.0f, state);
                             });

    // Audio (capability "audio").
    engineTable.set_function("play_sound", [&module](const std::string &path, sol::object volume)
                             { module.playSoundForScripts(path, volume.valid() ? volume.as<double>() : 1.0); });
    engineTable.set_function("set_master_volume", [&module](double volume)
                             { module.setMasterVolumeForScripts(volume); });
    engineTable.set_function("play_sound_3d",
                             [&module](const std::string &path, float x, float y, float z, sol::object volume)
                             {
                                 module.playSound3DForScripts(
                                     path, x, y, z, volume.valid() ? volume.as<double>() : 1.0);
                             });
    engineTable.set_function("set_listener_position", [&module](float x, float y, float z)
                             { module.setListenerPositionForScripts(x, y, z); });
    engineTable.set_function("set_listener_orientation",
                             [&module](float fx, float fy, float fz, float ux, float uy, float uz)
                             { module.setListenerOrientationForScripts(fx, fy, fz, ux, uy, uz); });

    // Persistence (capability "persistence").
    engineTable.set_function("save_scene", [&module](const std::string &path)
                             { return module.saveSceneForScripts(path); });
    engineTable.set_function("load_scene", [&module](const std::string &path)
                             { return module.loadSceneForScripts(path); });
    engineTable.set_function("clear_scene", [&module]() { return module.clearSceneForScripts(); });
}

} // namespace

LuaScriptingModule::LuaScriptingModule()
{
    componentManifest.name = "lua_scripting";
    componentManifest.kind = ComponentKind::Module;
    // This module is a script host: it provides the scripting capability (the
    // kernel resolves script mods' declared host capability to this module and
    // hands the packages over through the ScriptModHost contract).
    componentManifest.providedCapabilities = {scriptingCapabilityName};
    componentManifest.requiredCapabilities = {"logging", "window"};
    // The bridge's optional capabilities are declared as PERMISSIONS without
    // graph edges (the screenshot-capability pattern): a run without, say,
    // the scene module still starts; bridge calls into the absent capability
    // then fail precisely per call instead of at startup.
    componentManifest.requiredPermissions  = {"capability.logging", "capability.window", "capability.screenshot",
                                              "capability.input", "capability.scene", "capability.lighting",
                                              "capability.renderer", "capability.animation", "capability.asset",
                                              "capability.physics", "capability.audio", "capability.ui"};
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

    // Text input: the platform's layout-aware character stream, forwarded to
    // subscribing scripts as Unicode codepoints.
    EventChannel<TextInputEvent> &textInputChannel = bus.channel<TextInputEvent>();
    textInputSubscription                          = EventSubscription(
        &bus,
        typeid(TextInputEvent),
        textInputChannel.subscribe<LuaScriptingModule, &LuaScriptingModule::forwardTextInput>(this)
    );

    // Hot reload (optional module): a script change reloads that mod's state.
    EventChannel<HotReloadEvent> &hotReloadChannel = bus.channel<HotReloadEvent>();
    hotReloadSubscription                          = EventSubscription(
        &bus,
        typeid(HotReloadEvent),
        hotReloadChannel.subscribe<LuaScriptingModule, &LuaScriptingModule::handleHotReload>(this)
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
    textInputSubscription.release();
    hotReloadSubscription.release();
    // Timers hold sol::function references into the script states: they must
    // die BEFORE the states, or their destructors would resurrect freed
    // Lua registries.
    timers.clear();
    soundHandlesByPath.clear();
    environments.clear();
    pendingScripts.clear();
    boundContext         = nullptr;
    windowCapability     = nullptr;
    screenshotCapability = nullptr;
    logger               = nullptr;
}

void *LuaScriptingModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    // The scripting capability is realized by the ScriptModHost surface of
    // this module; the kernel resolves script hosting through it.
    if (capabilityName == scriptingCapabilityName)
    {
        return static_cast<void *>(static_cast<ScriptModHost *>(this));
    }
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

bool LuaScriptingModule::isKeyDownForScripts(
    const std::string &keyName
)
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return false;
    }
    return input->isKeyDown(keyCodeFromName(keyName));
}

bool LuaScriptingModule::isMouseButtonDownForScripts(
    const std::string &buttonName
)
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return false;
    }
    if (buttonName == "left")
    {
        return input->isMouseButtonDown(MouseButton::Left);
    }
    if (buttonName == "right")
    {
        return input->isMouseButtonDown(MouseButton::Right);
    }
    if (buttonName == "middle")
    {
        return input->isMouseButtonDown(MouseButton::Middle);
    }
    return false;
}

bool LuaScriptingModule::consumeMouseDeltaForScripts(
    double &deltaX, double &deltaY
)
{
    deltaX = deltaY = 0.0;
    // The ui module's overlay claims the pointer while it is visible (the
    // modal rule on UiCapability): scripts read a settled 0 motion while a
    // debug window is up, deterministically - no reliance on event or
    // subscription order between the bridge and the overlay's own drain.
    UiCapability *ui = resolveCapabilityForScripts<UiCapability>(uiResolutionState, uiWarnedUnavailable);
    if (ui != nullptr && ui->wantsMouse())
    {
        return true; // motion consumed by the overlay; scripts read zero
    }
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return false;
    }
    input->consumeMouseDelta(deltaX, deltaY);
    return true;
}

bool LuaScriptingModule::consumeScrollDeltaForScripts(
    double &deltaX, double &deltaY
)
{
    deltaX = deltaY = 0.0;
    // Same modal rule as the cursor deltas above.
    UiCapability *ui = resolveCapabilityForScripts<UiCapability>(uiResolutionState, uiWarnedUnavailable);
    if (ui != nullptr && ui->wantsMouse())
    {
        return true;
    }
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return false;
    }
    input->consumeScrollDelta(deltaX, deltaY);
    return true;
}

bool LuaScriptingModule::isGamepadDownForScripts(
    const std::string &controlName
)
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return false;
    }
    std::int32_t code    = 0;
    bool         isButton = false;
    if (!gamepadControlFromName(controlName, code, isButton) || !isButton)
    {
        return false;
    }
    return input->isGamepadButtonDown(static_cast<GamepadButton>(code));
}

bool LuaScriptingModule::gamepadConnectedForScripts()
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    return input != nullptr && input->isGamepadConnected();
}

double LuaScriptingModule::gamepadAxisForScripts(
    const std::string &axisName
)
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    if (input == nullptr)
    {
        return 0.0;
    }
    std::int32_t code    = 0;
    bool         isButton = true;
    if (!gamepadControlFromName(axisName, code, isButton) || isButton)
    {
        return 0.0;
    }
    return static_cast<double>(input->gamepadAxis(static_cast<GamepadAxis>(code)));
}

double LuaScriptingModule::actionStrengthForScripts(
    const std::string &actionName
)
{
    InputCapability *input =
        resolveCapabilityForScripts<InputCapability>(inputResolutionState, inputWarnedUnavailable);
    return input != nullptr ? static_cast<double>(input->actionStrength(actionName)) : 0.0;
}

std::uint32_t LuaScriptingModule::spawnObjectForScripts(
    const sol::table &description
)
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    if (scene == nullptr)
    {
        return SceneCapability::invalidObject;
    }
    SceneObjectDescriptor object;
    const std::string     typeName = stringField(description, "type");
    object.type = typeName == "sphere" ? scene_object_type::sphere : scene_object_type::box;
    object.position = vec3Field(description, "position", {0.0f, 0.0f, 0.0f});
    if (object.type == scene_object_type::sphere)
    {
        // Spheres keep exact normals under uniform radius: one number, stored
        // in x per the SceneObject convention (y/z ignored by the shaders).
        const float radius = static_cast<float>(numberField(description, "radius", 1.0));
        object.size        = {radius, radius, radius};
    }
    else
    {
        object.size = vec3Field(description, "half_extents", {1.0f, 1.0f, 1.0f});
    }
    object.rotationYRadians = static_cast<float>(numberField(description, "rotation", 0.0));
    object.albedo           = vec3Field(description, "albedo", {0.8f, 0.8f, 0.8f});
    object.roughness        = static_cast<float>(numberField(description, "roughness", 0.8));
    object.metallic         = static_cast<float>(numberField(description, "metallic", 0.0));
    object.emissiveStrength = static_cast<float>(numberField(description, "emissive", 0.0));
    object.uvScale          = static_cast<float>(numberField(description, "uv_scale", 1.0));
    object.textureSet       = stringField(description, "texture");
    return scene->addObject(object);
}

bool LuaScriptingModule::removeObjectForScripts(
    std::uint32_t handle
)
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    return scene != nullptr && scene->removeObject(handle);
}

bool LuaScriptingModule::setObjectPositionForScripts(
    std::uint32_t handle, float x, float y, float z
)
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    if (scene == nullptr)
    {
        return false;
    }
    // Rotation is preserved: a position move must not reorient anything.
    SceneObjectDescriptor current;
    if (!scene->object(handle, current))
    {
        return false;
    }
    return scene->setObjectTransform(handle, {x, y, z}, current.rotationYRadians);
}

bool LuaScriptingModule::setObjectRotationForScripts(
    std::uint32_t handle, float rotationYRadians
)
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    if (scene == nullptr)
    {
        return false;
    }
    SceneObjectDescriptor current;
    if (!scene->object(handle, current))
    {
        return false;
    }
    return scene->setObjectTransform(handle, current.position, rotationYRadians);
}

bool LuaScriptingModule::setObjectMaterialForScripts(
    std::uint32_t handle, const sol::table &material
)
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    if (scene == nullptr)
    {
        return false;
    }
    SceneObjectDescriptor current;
    if (!scene->object(handle, current))
    {
        return false;
    }
    // Only the fields the table names change: an omitted field keeps its
    // current value (the material edit composes, never resets).
    current.albedo           = vec3Field(material, "albedo", current.albedo);
    current.roughness        = static_cast<float>(numberField(material, "roughness", current.roughness));
    current.metallic         = static_cast<float>(numberField(material, "metallic", current.metallic));
    current.emissiveStrength = static_cast<float>(numberField(material, "emissive", current.emissiveStrength));
    current.uvScale          = static_cast<float>(numberField(material, "uv_scale", current.uvScale));
    const std::string texture = stringField(material, "texture");
    if (!texture.empty())
    {
        current.textureSet = texture;
    }
    return scene->setObjectMaterial(handle, current);
}

std::uint32_t LuaScriptingModule::objectCountForScripts()
{
    SceneCapability *scene =
        resolveCapabilityForScripts<SceneCapability>(sceneResolutionState, sceneWarnedUnavailable);
    return scene != nullptr ? scene->objectCount() : 0u;
}

void LuaScriptingModule::setSunLightForScripts(
    const sol::table &light
)
{
    LightingCapability *lighting =
        resolveCapabilityForScripts<LightingCapability>(lightingResolutionState, lightingWarnedUnavailable);
    if (lighting == nullptr)
    {
        return;
    }
    DirectionalLightDescription sun;
    const std::array<float, 3> sunDirection = vec3Field(light, "direction", {0.0f, -1.0f, 0.0f});
    const std::array<float, 3> sunColor     = vec3Field(light, "color", {1.0f, 1.0f, 1.0f});
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        sun.direction[axis] = sunDirection[axis];
        sun.color[axis]     = sunColor[axis];
    }
    sun.intensity = static_cast<float>(numberField(light, "intensity", 0.0));
    lighting->setSunLight(sun);
}

void LuaScriptingModule::clearSunLightForScripts()
{
    LightingCapability *lighting =
        resolveCapabilityForScripts<LightingCapability>(lightingResolutionState, lightingWarnedUnavailable);
    if (lighting != nullptr)
    {
        lighting->clearSunLight();
    }
}

std::uint32_t LuaScriptingModule::addPointLightForScripts(
    const sol::table &light
)
{
    LightingCapability *lighting =
        resolveCapabilityForScripts<LightingCapability>(lightingResolutionState, lightingWarnedUnavailable);
    if (lighting == nullptr)
    {
        return LightingCapability::invalidPointLight;
    }
    PointLightDescription point;
    const std::array<float, 3> pointPosition = vec3Field(light, "position", {0.0f, 0.0f, 0.0f});
    const std::array<float, 3> pointColor    = vec3Field(light, "color", {1.0f, 1.0f, 1.0f});
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        point.position[axis] = pointPosition[axis];
        point.color[axis]    = pointColor[axis];
    }
    point.intensity = static_cast<float>(numberField(light, "intensity", 0.0));
    point.radius    = static_cast<float>(numberField(light, "radius", 0.0));
    return lighting->addPointLight(point);
}

bool LuaScriptingModule::removePointLightForScripts(
    std::uint32_t handle
)
{
    LightingCapability *lighting =
        resolveCapabilityForScripts<LightingCapability>(lightingResolutionState, lightingWarnedUnavailable);
    if (lighting == nullptr)
    {
        return false;
    }
    lighting->removePointLight(handle);
    return true;
}

std::tuple<float, float, float> LuaScriptingModule::cameraPositionForScripts()
{
    RendererCapability *renderer =
        resolveCapabilityForScripts<RendererCapability>(rendererResolutionState, rendererWarnedUnavailable);
    if (renderer == nullptr)
    {
        return {0.0f, 0.0f, 0.0f};
    }
    const std::array<float, 3> position = renderer->cameraPosition();
    return {position[0], position[1], position[2]};
}

std::tuple<float, float, float> LuaScriptingModule::cameraLookAtForScripts()
{
    RendererCapability *renderer =
        resolveCapabilityForScripts<RendererCapability>(rendererResolutionState, rendererWarnedUnavailable);
    if (renderer == nullptr)
    {
        return {0.0f, 0.0f, 0.0f};
    }
    const std::array<float, 3> lookAt = renderer->cameraLookAt();
    return {lookAt[0], lookAt[1], lookAt[2]};
}

void LuaScriptingModule::setCameraPositionForScripts(
    float x, float y, float z
)
{
    RendererCapability *renderer =
        resolveCapabilityForScripts<RendererCapability>(rendererResolutionState, rendererWarnedUnavailable);
    if (renderer != nullptr)
    {
        renderer->setCameraPosition({x, y, z});
    }
}

void LuaScriptingModule::setCameraLookAtForScripts(
    float x, float y, float z
)
{
    RendererCapability *renderer =
        resolveCapabilityForScripts<RendererCapability>(rendererResolutionState, rendererWarnedUnavailable);
    if (renderer != nullptr)
    {
        renderer->setCameraLookAt({x, y, z});
    }
}

std::uint64_t LuaScriptingModule::createTimerForScripts(
    double intervalSeconds, bool repeating, sol::function callback
)
{
    if (intervalSeconds <= 0.0 || !callback.valid())
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "lua scripting module: engine.create_timer rejected (interval must be > 0, callback must be a "
                "function)"
            );
        }
        return 0;
    }
    ScriptTimer timer;
    timer.handle    = nextTimerHandle++;
    timer.interval  = intervalSeconds;
    timer.repeating = repeating;
    timer.callback  = std::move(callback);
    timer.nextFire  = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(intervalSeconds)
    );
    timers.push_back(std::move(timer));
    return timers.back().handle;
}

bool LuaScriptingModule::cancelTimerForScripts(
    std::uint64_t handle
)
{
    for (auto iterator = timers.begin(); iterator != timers.end(); ++iterator)
    {
        if (iterator->handle == handle)
        {
            timers.erase(iterator);
            return true;
        }
    }
    return false;
}

std::uint64_t LuaScriptingModule::animateObjectForScripts(
    std::uint32_t objectHandle, const sol::table &tween
)
{
    AnimationCapability *animation =
        resolveCapabilityForScripts<AnimationCapability>(animationResolutionState, animationWarnedUnavailable);
    if (animation == nullptr)
    {
        return AnimationCapability::invalidTween;
    }
    TweenDescription description;
    description.channel            = static_cast<std::uint32_t>(numberField(tween, "channel", animation_channel::position));
    description.fromPosition       = vec3Field(tween, "from_position", {0.0f, 0.0f, 0.0f});
    description.toPosition         = vec3Field(tween, "to_position", {0.0f, 0.0f, 0.0f});
    description.fromValue          = static_cast<float>(numberField(tween, "from_value", 0.0));
    description.toValue            = static_cast<float>(numberField(tween, "to_value", 1.0));
    description.durationSeconds    = numberField(tween, "duration", 1.0);
    description.startDelaySeconds  = numberField(tween, "delay", 0.0);
    description.easing             = static_cast<std::uint32_t>(numberField(tween, "easing", animation_easing::linear));
    description.mode               = static_cast<std::uint32_t>(numberField(tween, "mode", animation_mode::once));
    return animation->animateObject(objectHandle, description);
}

std::uint64_t LuaScriptingModule::animateLightForScripts(
    std::uint32_t lightHandle, const sol::table &tween
)
{
    AnimationCapability *animation =
        resolveCapabilityForScripts<AnimationCapability>(animationResolutionState, animationWarnedUnavailable);
    if (animation == nullptr)
    {
        return AnimationCapability::invalidTween;
    }
    TweenDescription description;
    description.channel            = static_cast<std::uint32_t>(numberField(tween, "channel", animation_channel::lightIntensity));
    description.fromValue          = static_cast<float>(numberField(tween, "from_value", 0.0));
    description.toValue            = static_cast<float>(numberField(tween, "to_value", 1.0));
    description.durationSeconds    = numberField(tween, "duration", 1.0);
    description.startDelaySeconds  = numberField(tween, "delay", 0.0);
    description.easing             = static_cast<std::uint32_t>(numberField(tween, "easing", animation_easing::linear));
    description.mode               = static_cast<std::uint32_t>(numberField(tween, "mode", animation_mode::once));
    return animation->animatePointLight(lightHandle, description);
}

bool LuaScriptingModule::cancelTweenForScripts(
    std::uint64_t handle
)
{
    AnimationCapability *animation =
        resolveCapabilityForScripts<AnimationCapability>(animationResolutionState, animationWarnedUnavailable);
    return animation != nullptr && animation->cancelTween(handle);
}

void LuaScriptingModule::playSoundForScripts(
    const std::string &path, double volume
)
{
    AudioCapability *audio =
        resolveCapabilityForScripts<AudioCapability>(audioResolutionState, audioWarnedUnavailable);
    if (audio == nullptr)
    {
        return;
    }
    // Load-once, play-many: the handle cache keeps repeated plays of the same
    // file from re-opening the decoder stream every call.
    auto found = soundHandlesByPath.find(path);
    if (found == soundHandlesByPath.end())
    {
        const AudioCapability::SoundHandle handle = audio->loadSound(path, false);
        if (handle == AudioCapability::invalidSound)
        {
            return;
        }
        found = soundHandlesByPath.emplace(path, handle).first;
    }
    audio->setVolume(found->second, static_cast<float>(volume));
    audio->play(found->second);
}

void LuaScriptingModule::setMasterVolumeForScripts(
    double volume
)
{
    AudioCapability *audio =
        resolveCapabilityForScripts<AudioCapability>(audioResolutionState, audioWarnedUnavailable);
    if (audio != nullptr)
    {
        audio->setMasterVolume(static_cast<float>(volume));
    }
}

void LuaScriptingModule::playSound3DForScripts(
    const std::string &path, float x, float y, float z, double volume
)
{
    AudioCapability *audio =
        resolveCapabilityForScripts<AudioCapability>(audioResolutionState, audioWarnedUnavailable);
    if (audio == nullptr)
    {
        return;
    }
    audio->playSound3D(path, {x, y, z}, static_cast<float>(volume));
}

void LuaScriptingModule::setListenerPositionForScripts(
    float x, float y, float z
)
{
    AudioCapability *audio =
        resolveCapabilityForScripts<AudioCapability>(audioResolutionState, audioWarnedUnavailable);
    if (audio != nullptr)
    {
        audio->setListenerPosition({x, y, z});
    }
}

void LuaScriptingModule::setListenerOrientationForScripts(
    float fx, float fy, float fz, float ux, float uy, float uz
)
{
    AudioCapability *audio =
        resolveCapabilityForScripts<AudioCapability>(audioResolutionState, audioWarnedUnavailable);
    if (audio != nullptr)
    {
        audio->setListenerOrientation({fx, fy, fz}, {ux, uy, uz});
    }
}

bool LuaScriptingModule::saveSceneForScripts(
    const std::string &path
)
{
    PersistenceCapability *persistence =
        resolveCapabilityForScripts<PersistenceCapability>(persistenceResolutionState, persistenceWarnedUnavailable);
    if (persistence == nullptr)
    {
        return false;
    }
    std::string errorReport;
    const bool  saved = persistence->saveScene(path, errorReport);
    if (!errorReport.empty() && logger != nullptr)
    {
        logger->logMessage(LogLevel::Error, errorReport);
    }
    return saved;
}

std::uint32_t LuaScriptingModule::loadSceneForScripts(
    const std::string &path
)
{
    PersistenceCapability *persistence =
        resolveCapabilityForScripts<PersistenceCapability>(persistenceResolutionState, persistenceWarnedUnavailable);
    if (persistence == nullptr)
    {
        return 0;
    }
    std::string errorReport;
    const std::uint32_t loaded = persistence->loadScene(path, errorReport);
    if (!errorReport.empty() && logger != nullptr)
    {
        logger->logMessage(LogLevel::Error, errorReport);
    }
    return loaded;
}

bool LuaScriptingModule::clearSceneForScripts()
{
    PersistenceCapability *persistence =
        resolveCapabilityForScripts<PersistenceCapability>(persistenceResolutionState, persistenceWarnedUnavailable);
    if (persistence == nullptr)
    {
        return false;
    }
    std::string errorReport;
    const bool  cleared = persistence->clearScene(errorReport);
    if (!errorReport.empty() && logger != nullptr)
    {
        logger->logMessage(LogLevel::Error, errorReport);
    }
    return cleared;
}

std::uint32_t LuaScriptingModule::addPhysicsBodyForScripts(
    std::uint32_t objectHandle, const sol::table &body
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    if (physics == nullptr)
    {
        return PhysicsCapability::invalidBody;
    }
    PhysicsCapability::BodyDescription description;
    description.restitution   = static_cast<float>(numberField(body, "restitution", 0.4));
    description.linearDamping = static_cast<float>(numberField(body, "damping", 0.02));
    return physics->addDynamicBody(objectHandle, description);
}

bool LuaScriptingModule::removePhysicsBodyForScripts(
    std::uint32_t bodyHandle
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    return physics != nullptr && physics->removeBody(bodyHandle);
}

bool LuaScriptingModule::setBodyVelocityForScripts(
    std::uint32_t bodyHandle, float x, float y, float z
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    return physics != nullptr && physics->setBodyVelocity(bodyHandle, {x, y, z});
}

sol::object LuaScriptingModule::bodyVelocityForScripts(
    std::uint32_t bodyHandle, sol::this_state state
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    std::array<float, 3> velocity = {0.0f, 0.0f, 0.0f};
    if (physics != nullptr)
    {
        physics->bodyVelocity(bodyHandle, velocity);
    }
    sol::state_view view(state);
    return sol::make_object(view, std::make_tuple(velocity[0], velocity[1], velocity[2]));
}

void LuaScriptingModule::setGravityForScripts(
    float x, float y, float z
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    if (physics != nullptr)
    {
        physics->setGravity({x, y, z});
    }
}

std::uint32_t LuaScriptingModule::physicsBodyCountForScripts()
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    return physics != nullptr ? physics->bodyCount() : 0u;
}

bool LuaScriptingModule::setBodySleepingForScripts(
    std::uint32_t bodyHandle, bool sleeping
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    return physics != nullptr ? physics->setBodySleeping(bodyHandle, sleeping) : false;
}

bool LuaScriptingModule::isBodyAwakeForScripts(
    std::uint32_t bodyHandle
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    return physics != nullptr ? physics->isBodyAwake(bodyHandle) : false;
}

sol::object LuaScriptingModule::raycastForScripts(
    float ox, float oy, float oz, float dx, float dy, float dz, float maxDistance, sol::this_state state
)
{
    PhysicsCapability *physics =
        resolveCapabilityForScripts<PhysicsCapability>(physicsResolutionState, physicsWarnedUnavailable);
    if (physics == nullptr)
    {
        return sol::make_object(state, sol::nil);
    }
    RaycastResult result;
    if (!physics->raycast({ox, oy, oz}, {dx, dy, dz}, maxDistance, result))
    {
        return sol::make_object(state, sol::nil); // a miss is nil, not a table of defaults
    }
    sol::state_view view(state);
    sol::table hit      = view.create_table();
    hit["body"]         = result.bodyHandle;
    hit["object"]       = result.objectHandle;
    hit["distance"]     = result.distance;
    hit["point"]        = view.create_table();
    hit["point"][1]     = result.point[0];
    hit["point"][2]     = result.point[1];
    hit["point"][3]     = result.point[2];
    hit["normal"]       = view.create_table();
    hit["normal"][1]    = result.normal[0];
    hit["normal"][2]    = result.normal[1];
    hit["normal"][3]    = result.normal[2];
    return sol::make_object(view, hit);
}

std::uint32_t LuaScriptingModule::importGltfForScripts(
    const std::string &path, const std::string &fidelity
)
{
    AssetCapability *asset =
        resolveCapabilityForScripts<AssetCapability>(assetResolutionState, assetWarnedUnavailable);
    if (asset == nullptr)
    {
        return 0;
    }
    std::uint32_t fidelityValue = asset_fidelity::full_mesh;
    if (fidelity == "bbox")
    {
        fidelityValue = asset_fidelity::bounding_proxies;
    }
    else if (fidelity != "mesh")
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "import_gltf: unknown fidelity '" + fidelity + "' (use \"mesh\" or \"bbox\")"
            );
        }
        return 0;
    }
    std::string errorReport;
    const std::uint32_t spawned = asset->importGltfIntoScene(path, fidelityValue, errorReport);
    if (!errorReport.empty() && logger != nullptr)
    {
        logger->logMessage(LogLevel::Error, errorReport);
    }
    return spawned;
}

void LuaScriptingModule::tickTimers()
{
    if (timers.empty())
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    // Due timers fire in registration order; a fired one-shot is dropped, a
    // repeating one schedules its next fire from NOW (catch-up would burst a
    // lagged frame into a storm of immediate refires).
    std::vector<ScriptTimer> stillPending;
    stillPending.reserve(timers.size());
    for (ScriptTimer &timer : timers)
    {
        if (now < timer.nextFire)
        {
            stillPending.push_back(std::move(timer));
            continue;
        }
        const sol::protected_function_result result = timer.callback();
        if (!result.valid())
        {
            const sol::error error = result;
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Error,
                    "lua scripting module: timer error, the timer was cancelled: " + std::string(error.what())
                );
            }
            continue; // one-shot AND repeating timers die on error: a broken callback stays broken
        }
        if (timer.repeating)
        {
            timer.nextFire = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timer.interval)
            );
            stillPending.push_back(std::move(timer));
        }
    }
    timers = std::move(stillPending);
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

void LuaScriptingModule::handleHotReload(
    const HotReloadEvent &event
)
{
    if (event.kind == hot_reload_kind::script && !event.modId.empty())
    {
        reloadScriptMod(event.modId);
    }
    // Manifest/scene/config changes need an assembly restart: log it once
    // per event (the watcher already debounces by mtime).
    else if (event.kind == hot_reload_kind::manifest || event.kind == hot_reload_kind::scene
             || event.kind == hot_reload_kind::config)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Information,
                "lua scripting module: '" + event.relativePath
                    + "' changed; configuration and manifest changes apply on the next run"
            );
        }
    }
}

void LuaScriptingModule::reloadScriptMod(
    const std::string &modId
)
{
    // Full state replacement: the script re-runs from the top (fresh
    // globals, fresh subscriptions). Subscriptions of the old state were
    // bus RAII guards tied to the environment's lifetime - destroying the
    // state releases them, so no stale handler can fire.
    for (ScriptEnvironment &environment : environments)
    {
        if (environment.modId != modId)
        {
            continue;
        }
        // The timers this environment created die with the state; drop them
        // first (the sol::function members would outlive their Lua registry).
        timers.erase(
            std::remove_if(
                timers.begin(),
                timers.end(),
                [&environment](const ScriptTimer &timer)
                {
                    sol::state_view view(timer.callback.lua_state());
                    return view == *environment.state;
                }
            ),
            timers.end()
        );

        const std::string path       = environment.path;
        const std::string scriptModId = environment.modId;
        environments.erase(
            std::remove_if(
                environments.begin(),
                environments.end(),
                [&path](const ScriptEnvironment &candidate) { return candidate.path == path; }
            ),
            environments.end()
        );

        ScriptModDescription description;
        description.modId         = scriptModId;
        description.scriptPath    = path;
        description.hostCapability = "scripting";
        try
        {
            loadScript(description);
            if (logger != nullptr)
            {
                logger->logMessage(LogLevel::Information, "lua scripting module: hot reloaded '" + path + "'");
            }
        }
        catch (const std::exception &error)
        {
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Error,
                    "lua scripting module: hot reload of '" + path + "' FAILED, the mod is now unloaded: " + error.what()
                );
            }
        }
        return;
    }
}

void LuaScriptingModule::forwardTextInput(
    const TextInputEvent &event
)
{
    for (ScriptEnvironment &environment : environments)
    {
        sol::table handlers = (*environment.state)["engine"]["text_input_handlers"];
        for (std::size_t index = 1; index <= handlers.size(); ++index)
        {
            sol::protected_function handler = handlers[index];
            if (!handler.valid())
            {
                continue;
            }
            const sol::protected_function_result result = handler(static_cast<std::uint64_t>(event.codepoint));
            if (!result.valid())
            {
                const sol::error error = result;
                logger->logMessage(
                    LogLevel::Error,
                    "lua scripting module: text input handler error in '" + environment.path + "' (mod '"
                        + environment.modId + "'): " + error.what()
                );
            }
        }
    }
}

void LuaScriptingModule::forwardFrameBegin(
    const FrameBeginEvent &event
)
{
    // Timers tick BEFORE the frame handlers: a timer that fires at t=1s runs
    // during the frame that crossed the mark, not the one after it.
    tickTimers();
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
