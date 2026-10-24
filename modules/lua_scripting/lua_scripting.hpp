#ifndef ENGINE_MODULES_LUA_SCRIPTING_HPP
#define ENGINE_MODULES_LUA_SCRIPTING_HPP

#include "animation_capability.hpp"
#include "asset_capability.hpp"
#include "audio_capability.hpp"
#include "hot_reload_capability.hpp"
#include "persistence_capability.hpp"
#include "physics_capability.hpp"
#include "input_capability.hpp"
#include "lighting_capability.hpp"
#include "logging_capability.hpp"
#include "renderer_capability.hpp"
#include "scene_capability.hpp"
#include "screenshot_capability.hpp"
#include "ui_capability.hpp"
#include "window_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>
#include <tuple>
#include <vector>

// sol2 in the module header: the timer registry stores sol::function values
// (copyable references into their owning script state), which needs the
// complete type. Only this module's translation units include this header,
// so the heavy include stays contained to the module.
#include <sol/sol.hpp>

namespace engine
{

class LoggerCapability;
class WindowCapability;
class ScreenshotCapability;

// Module "lua_scripting": hosts Lua 5.4 script mods through sol2. The kernel
// discovers script packages through their mod.toml manifests and hands each
// one over through the ScriptModHost contract; every script receives an
// "engine" table with a permission checked surface:
//
//   logging        log_trace/debug/information/warning/error
//   input          is_key_down, is_mouse_down, consume_mouse_delta,
//                  consume_scroll_delta (polled state)
//   scene          spawn_object, remove_object, set_object_position,
//                  set_object_rotation, set_object_material, object_count
//   lighting       set_sun_light, clear_sun_light, add_point_light,
//                  remove_point_light
//   renderer       camera_position, camera_look_at, set_camera_position,
//                  set_camera_look_at, window_size
//   screenshot     request_screenshot
//   (self)         subscribe_key_pressed/released/frame_begin,
//                  create_timer, cancel_timer
//
// Every capability-backed call RESOLVES ITS CAPABILITY PER CALL through the
// component context: the permission system's check runs on every bridge call
// (the per-call re-checking the module README promised as a roadmap item),
// and a capability whose module is absent from the build degrades to a
// precise, once-logged Lua-visible failure instead of a crash. Each script
// owns an isolated state; a script error fails startup with a precise report
// instead of silently skipping the mod.
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

    // Input polling (capability "input", per-call re-checked).
    bool isKeyDownForScripts(const std::string &keyName);
    bool isMouseButtonDownForScripts(const std::string &buttonName);
    bool consumeMouseDeltaForScripts(double &deltaX, double &deltaY);
    bool consumeScrollDeltaForScripts(double &deltaX, double &deltaY);
    bool isGamepadDownForScripts(const std::string &controlName);
    bool gamepadConnectedForScripts();
    double gamepadAxisForScripts(const std::string &axisName);
    double actionStrengthForScripts(const std::string &actionName);

    // Scene objects (capability "scene", per-call re-checked).
    std::uint32_t spawnObjectForScripts(const sol::table &description);
    bool          removeObjectForScripts(std::uint32_t handle);
    bool          setObjectPositionForScripts(std::uint32_t handle, float x, float y, float z);
    bool          setObjectRotationForScripts(std::uint32_t handle, float rotationYRadians);
    bool          setObjectMaterialForScripts(std::uint32_t handle, const sol::table &material);
    std::uint32_t objectCountForScripts();

    // Lights (capability "lighting", per-call re-checked).
    void          setSunLightForScripts(const sol::table &light);
    void          clearSunLightForScripts();
    std::uint32_t addPointLightForScripts(const sol::table &light);
    bool          removePointLightForScripts(std::uint32_t handle);

    // Camera (capability "renderer", per-call re-checked).
    std::tuple<float, float, float> cameraPositionForScripts();
    std::tuple<float, float, float> cameraLookAtForScripts();
    void                            setCameraPositionForScripts(float x, float y, float z);
    void                            setCameraLookAtForScripts(float x, float y, float z);

    // Timers (self-contained; they tick on the frame event, between frames).
    std::uint64_t createTimerForScripts(double intervalSeconds, bool repeating, sol::function callback);
    bool          cancelTimerForScripts(std::uint64_t handle);

    // Tweens (capability "animation", per-call re-checked).
    std::uint64_t animateObjectForScripts(std::uint32_t objectHandle, const sol::table &tween);
    std::uint64_t animateLightForScripts(std::uint32_t lightHandle, const sol::table &tween);
    bool          cancelTweenForScripts(std::uint64_t handle);

    // Content import (capability "asset", per-call re-checked). Returns the
    // number of spawned objects; a failed import returns 0 and logs the
    // precise reason (the script sees the count, not an exception - import
    // failures are content problems, not script bugs). Fidelity: "mesh"
    // (default) registers the triangles; "bbox" spawns the bounding-box
    // proxies of the v1 contract.
    std::uint32_t importGltfForScripts(const std::string &path, const std::string &fidelity);

    // Persistence (capability "persistence", per-call re-checked).
    bool          saveSceneForScripts(const std::string &path);
    std::uint32_t loadSceneForScripts(const std::string &path);
    bool          clearSceneForScripts();

    // Physics (capability "physics", per-call re-checked).
    // Audio (capability "audio", per-call re-checked).
    void          playSoundForScripts(const std::string &path, double volume);
    void          setMasterVolumeForScripts(double volume);
    void          playSound3DForScripts(const std::string &path, float x, float y, float z, double volume);
    void          setListenerPositionForScripts(float x, float y, float z);
    void          setListenerOrientationForScripts(float fx, float fy, float fz, float ux, float uy, float uz);

    std::uint32_t    addPhysicsBodyForScripts(std::uint32_t objectHandle, const sol::table &body);
    bool             removePhysicsBodyForScripts(std::uint32_t bodyHandle);
    bool             setBodyVelocityForScripts(std::uint32_t bodyHandle, float x, float y, float z);
    sol::object      bodyVelocityForScripts(std::uint32_t bodyHandle, sol::this_state state);
    void             setGravityForScripts(float x, float y, float z);
    std::uint32_t    physicsBodyCountForScripts();
    bool             setBodySleepingForScripts(std::uint32_t bodyHandle, bool sleeping);
    bool             isBodyAwakeForScripts(std::uint32_t bodyHandle);
    sol::object      raycastForScripts(float ox, float oy, float oz, float dx, float dy, float dz, float maxDistance,
                                       sol::this_state state);

private:
    bool ensureScreenshotCapability();

    // Per-call capability resolution with a failure cache: the SUCCESS path
    // re-acquires (and with it re-checks the permission) on every call; the
    // FAILURE path is cached (-1) because an absent module or a denied
    // permission is a run-long condition - the cache turns repeated throws
    // into a quiet false, with one logged warning.
    template <typename CapabilityType>
    CapabilityType *resolveCapabilityForScripts(
        int &resolutionState, bool &warnedUnavailable
    )
    {
        if (boundContext == nullptr || resolutionState == -1)
        {
            return nullptr;
        }
        try
        {
            CapabilityType &capability = boundContext->acquireCapability<CapabilityType>();
            resolutionState            = 1;
            return &capability;
        }
        catch (const std::exception &error)
        {
            resolutionState = -1;
            if (!warnedUnavailable && logger != nullptr)
            {
                warnedUnavailable = true;
                logger->logMessage(
                    LogLevel::Warning,
                    "lua scripting module: bridge calls into '" + std::string(CapabilityType::capabilityName)
                        + "' are unavailable for this run (" + error.what() + ")"
                );
            }
            return nullptr;
        }
    }

    struct ScriptEnvironment
    {
        std::string                 path;
        std::string                 modId;
        std::unique_ptr<sol::state> state;
    };

    // One registered timer. Timers are per-process (any script may create
    // them); the callback carries its own Lua state through the sol::function
    // reference, so one ticking loop serves every environment.
    struct ScriptTimer
    {
        std::uint64_t                         handle    = 0;
        double                                interval  = 1.0;
        bool                                  repeating = false;
        sol::function                         callback;
        std::chrono::steady_clock::time_point nextFire{};
    };

    void loadScript(const ScriptModDescription &scriptMod);
    void forwardKeyPressed(const KeyPressedEvent &event);
    void forwardKeyReleased(const KeyReleasedEvent &event);
    void dispatchKeyEvent(bool pressed, const std::string &keyName);
    void forwardFrameBegin(const FrameBeginEvent &event);
    void tickTimers();

    ComponentManifest     componentManifest;
    ComponentContext     *boundContext         = nullptr;
    LoggerCapability     *logger               = nullptr;
    WindowCapability     *windowCapability     = nullptr;
    ScreenshotCapability *screenshotCapability = nullptr;
    // Tri-state screenshot capability resolution: 0 unresolved, 1 available,
    // -1 decided unavailable (module compiled out or permission denied).
    int                               screenshotResolutionState = 0;
    bool                              screenshotWarnedUnavailable = false;
    // Tri-state resolution for the per-call re-checked bridge capabilities.
    int                               inputResolutionState    = 0;
    bool                              inputWarnedUnavailable    = false;
    int                               sceneResolutionState    = 0;
    bool                              sceneWarnedUnavailable    = false;
    int                               lightingResolutionState = 0;
    bool                              lightingWarnedUnavailable  = false;
    int                               rendererResolutionState = 0;
    bool                              rendererWarnedUnavailable  = false;
    int                               animationResolutionState = 0;
    bool                              animationWarnedUnavailable = false;
    int                               assetResolutionState = 0;
    bool                              assetWarnedUnavailable = false;
    int                               physicsResolutionState = 0;
    bool                              physicsWarnedUnavailable = false;
    int                               audioResolutionState = 0;
    bool                              audioWarnedUnavailable = false;
    int                               uiResolutionState = 0;
    bool                              uiWarnedUnavailable = false;
    int                               persistenceResolutionState = 0;
    bool                              persistenceWarnedUnavailable = false;
    std::unordered_map<std::string, AudioCapability::SoundHandle> soundHandlesByPath;
    EventSubscription                 keyPressedSubscription;
    EventSubscription                 keyReleasedSubscription;
    EventSubscription                 frameBeginSubscription;
    EventSubscription                 hotReloadSubscription;
    EventSubscription                 textInputSubscription;

    void forwardTextInput(const TextInputEvent &event);

    // Hot reload (modules/hot_reload_watcher): reloads the named script mod's state.
    void handleHotReload(const HotReloadEvent &event);
    void reloadScriptMod(const std::string &modId);
    std::vector<ScriptModDescription> pendingScripts;
    std::vector<ScriptEnvironment>    environments;
    std::vector<ScriptTimer>          timers;
    std::uint64_t                     nextTimerHandle = 1;
};

} // namespace engine

#endif // ENGINE_MODULES_LUA_SCRIPTING_HPP
