// input_actions module implementation.
//
// Maps raw device events to per-context virtual actions: contexts are
// evaluated in priority order each advanceFrame, binding values are combined
// per component and filtered, and higher-priority contexts can consume
// controls so overlapping context stacks behave predictably.

#include <engine/input_actions/capabilities.hpp>
#include "lua_bridge.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine::input_actions
{
namespace
{

// Per-device control snapshot. Control maps and touch contacts grow on demand
// and count toward the per-device control cap; accumulated values reset at
// the end of every advanceFrame.
struct TouchState
{
    float x = 0.0f;
    float y = 0.0f;
    float deltaX = 0.0f;
    float deltaY = 0.0f;
    float pressure = 1.0f;
    std::uint64_t sequence = 0;
    bool down = false;
    bool pressedThisFrame = false;
};

struct DeviceState
{
    std::unordered_map<std::uint32_t, bool>  keys;
    std::unordered_map<std::uint32_t, bool>  mouseButtons;
    std::unordered_map<std::uint32_t, bool>  gamepadButtons;
    std::unordered_map<std::uint32_t, float> gamepadAxes;
    std::unordered_map<std::uint32_t, TouchState> touches;
    std::uint64_t touchSequence = 0;
    // Accumulated [x, y] since the last frame, clamped to +/-1e6.
    float mouseDelta[2] = {0.0f, 0.0f};
    float mouseWheel[2] = {0.0f, 0.0f};
};

// Identifies a control (kind + device + code) for consumption tracking.
struct ControlKey
{
    InputControlKind control                                            = InputControlKind::Key;
    std::uint32_t    device                                             = 0;
    std::uint32_t    code                                               = 0;
    friend bool      operator==(const ControlKey &, const ControlKey &) = default;
};

// Fibonacci-style mixing hash for the consumed-control set.
struct ControlKeyHash
{
    std::size_t operator()(const ControlKey &key) const noexcept
    {
        std::size_t value = static_cast<std::size_t>(key.control);
        value             = value * 0x9e3779b1u + key.device;
        value             = value * 0x9e3779b1u + key.code;
        return value;
    }
};

// One action: immutable description, its bindings, and the last state
// computed by advanceFrame.
struct ActionRecord
{
    InputActionHandle         handle = invalidInputAction;
    std::string               name;
    InputActionType           type             = InputActionType::Button;
    float                     threshold        = 0.5f;
    InputActionCombineMode    combineMode      = InputActionCombineMode::Sum;
    float                     smoothingSeconds = 0.0f;
    std::vector<InputBinding> bindings;
    InputActionState          state{};
};

// One input context: priority/enabled flags and the actions it owns.
struct ContextRecord
{
    InputContextHandle                                  handle = invalidInputContext;
    std::string                                         name;
    std::int32_t                                        priority          = 0;
    bool                                                enabled           = true;
    bool                                                consumeWhenActive = false;
    std::unordered_map<InputActionHandle, ActionRecord> actions;
};

// ModuleInterface implementation; state persists across frames in the device
// snapshots and action states, and the module is safe only for single-thread
// use; capability entry points serialize access with an internal mutex.
class InputActionsModule final : public ModuleInterface
{
public:
    InputActionsModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }
    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        // Built-in defaults; each key applies only when present and in range.
        if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_contexts");
            value && *value > 0 && *value <= 4096)
            maximumContexts_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_actions_per_context");
            value && *value > 0 && *value <= 65536)
            maximumActionsPerContext_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_bindings_per_action");
            value && *value > 0 && *value <= 4096)
            maximumBindingsPerAction_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_devices");
            value && *value > 0 && *value <= 1024)
            maximumDevices_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("input_actions", "maximum_controls_per_device");
            value && *value > 0 && *value <= 65536)
            maximumControlsPerDevice_ = static_cast<std::size_t>(*value);
        // Pre-size the frame scratch containers from the configured caps.
        orderedContexts_.reserve(maximumContexts_);
        consumedControls_.reserve(std::min<std::size_t>(maximumActionsPerContext_ * maximumBindingsPerAction_, 65536));
        pendingConsumption_.reserve(maximumActionsPerContext_ * std::min<std::size_t>(maximumBindingsPerAction_, 8));
        activeControls_.reserve(maximumBindingsPerAction_);
        // Radial deadzone used by GamepadAxis bindings that pass -1; [0, 1).
        if (const auto value = context.configurationFloatingInSection("input_actions", "default_deadzone");
            value && std::isfinite(*value) && *value >= 0.0 && *value < 1.0)
            defaultDeadzone_ = static_cast<float>(*value);
        // Per-action smoothing cap; [0, 10] seconds.
        if (const auto value = context.configurationFloatingInSection("input_actions", "maximum_smoothing_seconds");
            value && std::isfinite(*value) && *value >= 0.0 && *value <= 10.0)
            maximumSmoothingSeconds_ = static_cast<float>(*value);
        // Clamp applied to advanceFrame's deltaSeconds; (0, 1] seconds.
        if (const auto value = context.configurationFloatingInSection("input_actions", "maximum_frame_delta_seconds");
            value && std::isfinite(*value) && *value > 0.0 && *value <= 1.0)
            maximumFrameDeltaSeconds_ = static_cast<float>(*value);

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.inputActions = &capability_;
            runtimeInitializer_ = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeInputActionsLuaRuntime, &luaBridgeContext_);
        }
    }
    void start() override
    {}
    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
        {
            std::lock_guard bridgeLock(luaBridgeContext_.apiMutex);
            luaBridgeContext_.inputActions = nullptr;
        }
        std::lock_guard lock(mutex_);
        // Drop all contexts and device state; the next frame rebuilds order.
        contexts_.clear();
        devices_.clear();
        orderedContexts_.clear();
        contextsOrderDirty_ = true;
    }
    void *capabilityPointerFor(const std::string &name) override
    {
        return name == InputActionsCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "input_actions",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"input_actions"},
            .optionalCapabilities = {"script_runtime"}};
    }
    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_INPUT_ACTIONS_BIND(field, method)                                                                       \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    {                                                                                                                   \
        auto *module = static_cast<InputActionsModule *>(self);                                                         \
        std::lock_guard lock(module->mutex_);                                                                           \
        return module->method(args...);                                                                                 \
    }
        ENGINE_INPUT_ACTIONS_BIND(createContext, createContextImpl);
        ENGINE_INPUT_ACTIONS_BIND(destroyContext, destroyContextImpl);
        ENGINE_INPUT_ACTIONS_BIND(setContextEnabled, setContextEnabledImpl);
        ENGINE_INPUT_ACTIONS_BIND(createAction, createActionImpl);
        ENGINE_INPUT_ACTIONS_BIND(removeAction, removeActionImpl);
        ENGINE_INPUT_ACTIONS_BIND(bindAction, bindActionImpl);
        ENGINE_INPUT_ACTIONS_BIND(clearBindings, clearBindingsImpl);
        ENGINE_INPUT_ACTIONS_BIND(removeBinding, removeBindingImpl);
        ENGINE_INPUT_ACTIONS_BIND(bindingCount, bindingCountImpl);
        ENGINE_INPUT_ACTIONS_BIND(copyBindings, copyBindingsImpl);
        ENGINE_INPUT_ACTIONS_BIND(feedEvent, feedEventImpl);
        ENGINE_INPUT_ACTIONS_BIND(feedWindowEvent, feedWindowEventImpl);
        ENGINE_INPUT_ACTIONS_BIND(advanceFrame, advanceFrameImpl);
        ENGINE_INPUT_ACTIONS_BIND(actionState, actionStateImpl);
        ENGINE_INPUT_ACTIONS_BIND(copyActionName, copyActionNameImpl);
        ENGINE_INPUT_ACTIONS_BIND(resetDeviceState, resetDeviceStateImpl);
#undef ENGINE_INPUT_ACTIONS_BIND
    }
    ContextRecord *findContext(InputContextHandle handle)
    {
        const auto iterator = contexts_.find(handle);
        return iterator == contexts_.end() ? nullptr : iterator->second.get();
    }
    ActionRecord *findAction(ContextRecord &context, InputActionHandle handle)
    {
        const auto iterator = context.actions.find(handle);
        return iterator == context.actions.end() ? nullptr : &iterator->second;
    }
    // Context handles skip the 0 sentinel; creating or destroying a context
    // marks the evaluation order stale for the next frame.
    InputContextHandle createContextImpl(const InputContextDescription *description)
    {
        if (contexts_.size() >= maximumContexts_)
            return invalidInputContext;
        if (nextContext_ == invalidInputContext || nextContext_ == std::numeric_limits<InputContextHandle>::max())
            return invalidInputContext;
        auto context    = std::make_unique<ContextRecord>();
        context->handle = nextContext_++;
        context->name = description != nullptr && description->name != nullptr ? description->name : "input_context";
        if (description != nullptr)
        {
            context->priority          = description->priority;
            context->enabled           = description->enabled;
            context->consumeWhenActive = description->consumeWhenActive;
        }
        const InputContextHandle handle = context->handle;
        contexts_.emplace(handle, std::move(context));
        contextsOrderDirty_ = true;
        return handle;
    }
    void destroyContextImpl(InputContextHandle handle)
    {
        if (contexts_.erase(handle) != 0)
            contextsOrderDirty_ = true;
    }
    // Disabling clears action states so no stale pressed/released edges are
    // observed while the context is off.
    bool setContextEnabledImpl(InputContextHandle handle, bool enabled)
    {
        ContextRecord *context = findContext(handle);
        if (context == nullptr)
            return false;
        context->enabled = enabled;
        if (!enabled)
            for (auto &[id, action] : context->actions)
            {
                (void)id;
                action.state = {};
            }
        return true;
    }
    // Full validation chain: type/combine enum ranges, threshold in [0, 1],
    // smoothing within the configured cap, unique non-empty name.
    InputActionHandle createActionImpl(InputContextHandle contextHandle, const InputActionDescription *description)
    {
        ContextRecord *context = findContext(contextHandle);
        if (context == nullptr || description == nullptr || description->name == nullptr || description->name[0] == '\0'
            || static_cast<std::uint32_t>(description->type) > static_cast<std::uint32_t>(InputActionType::Axis2D)
            || context->actions.size() >= maximumActionsPerContext_ || !std::isfinite(description->activationThreshold)
            || description->activationThreshold < 0.0f || description->activationThreshold > 1.0f
            || !std::isfinite(description->smoothingSeconds) || description->smoothingSeconds < 0.0f
            || description->smoothingSeconds > maximumSmoothingSeconds_
            || static_cast<std::uint32_t>(description->combineMode)
                   > static_cast<std::uint32_t>(InputActionCombineMode::LastNonZero))
            return invalidInputAction;
        // Names are unique per context; linear scan over the handle map.
        for (const auto &[id, action] : context->actions)
        {
            (void)id;
            if (action.name == description->name)
                return invalidInputAction;
        }
        if (nextAction_ == invalidInputAction || nextAction_ == std::numeric_limits<InputActionHandle>::max())
            return invalidInputAction;
        ActionRecord action{};
        action.handle = nextAction_++;
        action.name                    = description->name;
        action.type                    = description->type;
        action.threshold               = description->activationThreshold;
        action.combineMode             = description->combineMode;
        action.smoothingSeconds        = description->smoothingSeconds;
        const InputActionHandle result = action.handle;
        context->actions.emplace(result, std::move(action));
        return result;
    }
    bool removeActionImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        ContextRecord *context = findContext(contextHandle);
        return context != nullptr && context->actions.erase(actionHandle) != 0;
    }
    // Binding validation: enum range, finite scalars, deadzone in [-1, 1),
    // response exponent in (0, 8], and component rules per action type.
    bool bindActionImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, const InputBinding *binding)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr || binding == nullptr || binding->component > 1 || !std::isfinite(binding->scale)
            || !std::isfinite(binding->deadzone) || binding->deadzone < -1.0f || binding->deadzone >= 1.0f
            || !std::isfinite(binding->responseExponent) || binding->responseExponent <= 0.0f
            || binding->responseExponent > 8.0f
            || static_cast<std::uint32_t>(binding->control) > static_cast<std::uint32_t>(InputControlKind::TouchPressure))
            return false;
        if (action->bindings.size() >= maximumBindingsPerAction_)
            return false;
        const bool analogControl = binding->control == InputControlKind::GamepadAxis
            || binding->control == InputControlKind::TouchPositionX
            || binding->control == InputControlKind::TouchPositionY
            || binding->control == InputControlKind::TouchDeltaX
            || binding->control == InputControlKind::TouchDeltaY
            || binding->control == InputControlKind::TouchPressure;
        if (action->type == InputActionType::Button && (analogControl || binding->component != 0))
            return false;
        if (action->type == InputActionType::Axis1D && binding->component != 0)
            return false;
        action->bindings.push_back(*binding);
        return true;
    }
    bool clearBindingsImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr)
            return false;
        action->bindings.clear();
        action->state = {};
        return true;
    }
    bool removeBindingImpl(
        InputContextHandle contextHandle, InputActionHandle actionHandle, std::uint32_t bindingIndex)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr || bindingIndex >= action->bindings.size())
            return false;
        action->bindings.erase(action->bindings.begin() + bindingIndex);
        action->state = {};
        return true;
    }
    std::uint32_t bindingCountImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        return action != nullptr ? static_cast<std::uint32_t>(action->bindings.size()) : 0u;
    }
    bool copyBindingsImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, InputBinding *output,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr)
            return false;
        const std::uint32_t required = static_cast<std::uint32_t>(action->bindings.size());
        if (outRequired != nullptr)
            *outRequired = required;
        if (capacity < required || (required != 0 && output == nullptr))
            return false;
        if (required != 0)
            std::copy(action->bindings.begin(), action->bindings.end(), output);
        return true;
    }
    // Push a raw event into device state. FocusLost clears the device instead
    // of storing anything; unknown devices are created on demand until the
    // device cap is hit. Returns false for malformed events or cap refusals.
    bool feedEventImpl(const InputEvent *event)
    {
        if (event == nullptr || !std::isfinite(event->value) || !std::isfinite(event->secondaryValue)
            || !std::isfinite(event->tertiaryValue)
            || static_cast<std::uint32_t>(event->kind) > static_cast<std::uint32_t>(InputEventKind::TouchCancelled))
            return false;
        if (event->kind == InputEventKind::FocusLost)
        {
            const auto found = devices_.find(event->device);
            if (found == devices_.end())
                return true;
            DeviceState &device = found->second;
            device.keys.clear();
            device.mouseButtons.clear();
            device.gamepadButtons.clear();
            device.gamepadAxes.clear();
            device.touches.clear();
            device.mouseDelta[0] = device.mouseDelta[1] = device.mouseWheel[0] = device.mouseWheel[1] = 0.0f;
            return true;
        }
        auto found = devices_.find(event->device);
        if (found == devices_.end())
        {
            if (devices_.size() >= maximumDevices_)
                return false;
            try
            {
                found = devices_.try_emplace(event->device).first;
            }
            catch (...)
            {
                return false;
            }
        }
        DeviceState &device = found->second;
        // One stored slot per code/contact; the cap spans every control map of
        // the device combined. Existing controls always update in place.
        auto insertControl = [this, &device](auto &controls, std::uint32_t code, auto value)
        {
            const auto        item          = controls.find(code);
            const std::size_t totalControls = device.keys.size() + device.mouseButtons.size()
                                            + device.gamepadButtons.size() + device.gamepadAxes.size()
                                            + device.touches.size();
            if (item == controls.end() && totalControls >= maximumControlsPerDevice_)
                return false;
            if (item == controls.end())
            {
                try
                {
                    controls.emplace(code, value);
                }
                catch (...)
                {
                    return false;
                }
            }
            else
            {
                item->second = value;
            }
            return true;
        };
        switch (event->kind)
        {
        case InputEventKind::Key:
            return insertControl(device.keys, event->code, event->down);
        case InputEventKind::MouseButton:
            return insertControl(device.mouseButtons, event->code, event->down);
        case InputEventKind::GamepadButton:
            return insertControl(device.gamepadButtons, event->code, event->down);
        case InputEventKind::GamepadAxis:
            // Analog values are clamped to the conventional [-1, 1] range.
            return insertControl(device.gamepadAxes, event->code, std::clamp(event->value, -1.0f, 1.0f));
        case InputEventKind::MouseDelta:
            // Deltas accumulate across events within one frame.
            device.mouseDelta[0] = std::clamp(device.mouseDelta[0] + event->value, -1.0e6f, 1.0e6f);
            device.mouseDelta[1] = std::clamp(device.mouseDelta[1] + event->secondaryValue, -1.0e6f, 1.0e6f);
            return true;
        case InputEventKind::MouseWheel:
            device.mouseWheel[0] = std::clamp(device.mouseWheel[0] + event->value, -1.0e6f, 1.0e6f);
            device.mouseWheel[1] = std::clamp(device.mouseWheel[1] + event->secondaryValue, -1.0e6f, 1.0e6f);
            return true;
        case InputEventKind::TouchBegan:
        case InputEventKind::TouchMoved:
        case InputEventKind::TouchEnded:
        case InputEventKind::TouchCancelled: {
            if (event->code == anyTouchContact || event->value < 0.0f || event->value > 1.0f || event->secondaryValue < 0.0f
                || event->secondaryValue > 1.0f || event->tertiaryValue < 0.0f || event->tertiaryValue > 1.0f)
                return false;
            auto touch = device.touches.find(event->code);
            if (event->kind == InputEventKind::TouchBegan)
            {
                if (touch != device.touches.end() && touch->second.down)
                    return false;
                const std::size_t totalControls = device.keys.size() + device.mouseButtons.size()
                    + device.gamepadButtons.size() + device.gamepadAxes.size() + device.touches.size();
                if (touch == device.touches.end() && totalControls >= maximumControlsPerDevice_)
                    return false;
                TouchState state{};
                state.x = event->value;
                state.y = event->secondaryValue;
                state.pressure = event->tertiaryValue;
                state.down = true;
                state.pressedThisFrame = true;
                state.sequence = ++device.touchSequence;
                if (state.sequence == 0)
                    state.sequence = ++device.touchSequence;
                if (touch == device.touches.end())
                {
                    try
                    {
                        device.touches.emplace(event->code, state);
                    }
                    catch (...)
                    {
                        return false;
                    }
                }
                else
                {
                    touch->second = state;
                }
                return true;
            }
            if (touch == device.touches.end() || !touch->second.down)
                return false;
            TouchState &state = touch->second;
            state.deltaX += event->kind == InputEventKind::TouchMoved ? event->value - state.x : 0.0f;
            state.deltaY += event->kind == InputEventKind::TouchMoved ? event->secondaryValue - state.y : 0.0f;
            state.x = event->value;
            state.y = event->secondaryValue;
            state.pressure = event->tertiaryValue;
            state.sequence = ++device.touchSequence;
            if (state.sequence == 0)
                state.sequence = ++device.touchSequence;
            if (event->kind == InputEventKind::TouchEnded || event->kind == InputEventKind::TouchCancelled)
                state.down = false;
            return true;
        }
        case InputEventKind::FocusLost:
            break;
        }
        return false;
    }
    const TouchState *findTouch(
        const DeviceState &device, std::uint32_t contactId, std::uint32_t *outContactId = nullptr) const
    {
        if (contactId != anyTouchContact)
        {
            const auto found = device.touches.find(contactId);
            if (outContactId != nullptr)
                *outContactId = found != device.touches.end() ? found->first : anyTouchContact;
            return found != device.touches.end() ? &found->second : nullptr;
        }
        const TouchState *selected = nullptr;
        std::uint32_t selectedId = anyTouchContact;
        for (const auto &[id, touch] : device.touches)
        {
            if (selected == nullptr || (touch.down && !selected->down)
                || (touch.down == selected->down && touch.sequence > selected->sequence))
            {
                selected = &touch;
                selectedId = id;
            }
        }
        if (outContactId != nullptr)
            *outContactId = selectedId;
        return selected;
    }
    bool feedWindowEventImpl(window::WindowHandle windowHandle, const window::WindowEvent *event)
    {
        if (event == nullptr || windowHandle == window::invalidWindow)
            return false;

        InputEvent input{};
        input.device = keyboardMouseDevice;
        switch (event->kind)
        {
        case window::WindowEventKind::KeyPressed:
        case window::WindowEventKind::KeyReleased:
            input.kind = InputEventKind::Key;
            input.code = static_cast<std::uint32_t>(event->key);
            input.down = event->kind == window::WindowEventKind::KeyPressed;
            return input.code != static_cast<std::uint32_t>(window::Key::Unknown) && feedEventImpl(&input);
        case window::WindowEventKind::MouseButtonPressed:
        case window::WindowEventKind::MouseButtonReleased:
            input.kind = InputEventKind::MouseButton;
            input.code = static_cast<std::uint32_t>(event->mouseButton);
            input.down = event->kind == window::WindowEventKind::MouseButtonPressed;
            return input.code < 8u && feedEventImpl(&input);
        case window::WindowEventKind::CursorMoved:
            input.kind = InputEventKind::MouseDelta;
            input.value = static_cast<float>(std::clamp(event->cursorDeltaX,
                -static_cast<double>(std::numeric_limits<float>::max()),
                static_cast<double>(std::numeric_limits<float>::max())));
            input.secondaryValue = static_cast<float>(std::clamp(event->cursorDeltaY,
                -static_cast<double>(std::numeric_limits<float>::max()),
                static_cast<double>(std::numeric_limits<float>::max())));
            return feedEventImpl(&input);
        case window::WindowEventKind::MouseScrolled:
            input.kind = InputEventKind::MouseWheel;
            input.value = static_cast<float>(std::clamp(event->scrollX,
                -static_cast<double>(std::numeric_limits<float>::max()),
                static_cast<double>(std::numeric_limits<float>::max())));
            input.secondaryValue = static_cast<float>(std::clamp(event->scrollY,
                -static_cast<double>(std::numeric_limits<float>::max()),
                static_cast<double>(std::numeric_limits<float>::max())));
            return feedEventImpl(&input);
        case window::WindowEventKind::FocusLost:
            input.kind = InputEventKind::FocusLost;
            if (!feedEventImpl(&input))
                return false;
            // A lost window focus can prevent a native host from receiving
            // touch-end callbacks. Drop all forwarded contacts to avoid stuck touches.
            for (auto &[deviceId, device] : devices_)
            {
                if ((deviceId & touchDeviceIdBase) != 0)
                    device.touches.clear();
            }
            return true;
        case window::WindowEventKind::GamepadConnected:
            return true;
        case window::WindowEventKind::GamepadDisconnected:
            resetDeviceStateImpl(gamepadDeviceIdBase | (event->gamepadId & 0x3fffffffu));
            return true;
        case window::WindowEventKind::GamepadButtonPressed:
        case window::WindowEventKind::GamepadButtonReleased:
            input.device = gamepadDeviceIdBase | (event->gamepadId & 0x3fffffffu);
            input.kind = InputEventKind::GamepadButton;
            input.code = event->gamepadControl;
            input.down = event->kind == window::WindowEventKind::GamepadButtonPressed;
            return input.code < 15u && feedEventImpl(&input);
        case window::WindowEventKind::GamepadAxisMoved:
            input.device = gamepadDeviceIdBase | (event->gamepadId & 0x3fffffffu);
            input.kind = InputEventKind::GamepadAxis;
            input.code = event->gamepadControl;
            input.value = event->gamepadValue;
            return input.code < 6u && feedEventImpl(&input);
        case window::WindowEventKind::TouchBegan:
        case window::WindowEventKind::TouchMoved:
        case window::WindowEventKind::TouchEnded:
        case window::WindowEventKind::TouchCancelled:
            if (event->touchDevice > 0x7fffffffu)
                return false;
            input.device = touchDeviceIdBase | event->touchDevice;
            input.code = event->touchId;
            input.value = event->touchX;
            input.secondaryValue = event->touchY;
            input.tertiaryValue = event->touchPressure;
            switch (event->kind)
            {
            case window::WindowEventKind::TouchBegan: input.kind = InputEventKind::TouchBegan; break;
            case window::WindowEventKind::TouchMoved: input.kind = InputEventKind::TouchMoved; break;
            case window::WindowEventKind::TouchEnded: input.kind = InputEventKind::TouchEnded; break;
            case window::WindowEventKind::TouchCancelled: input.kind = InputEventKind::TouchCancelled; break;
            default: return false;
            }
            return feedEventImpl(&input);
        default:
            return true;
        }
    }

    // Evaluate one binding against current device state: look up the control
    // (unknown devices read as 0), apply the axis deadzone rescaled so
    // [dz, 1] maps to [0, 1] sign-preserving, then invert, scale, and apply
    // the response curve |v|^exponent.
    float readBinding(const InputBinding &binding)
    {
        const auto iterator = devices_.find(binding.device);
        if (iterator == devices_.end())
            return 0.0f;
        const DeviceState &device = iterator->second;
        float              value  = 0.0f;
        switch (binding.control)
        {
        case InputControlKind::Key: {
            const auto found = device.keys.find(binding.code);
            value            = found != device.keys.end() && found->second ? 1.0f : 0.0f;
            break;
        }
        case InputControlKind::MouseButton: {
            const auto found = device.mouseButtons.find(binding.code);
            value            = found != device.mouseButtons.end() && found->second ? 1.0f : 0.0f;
            break;
        }
        case InputControlKind::GamepadButton: {
            const auto found = device.gamepadButtons.find(binding.code);
            value            = found != device.gamepadButtons.end() && found->second ? 1.0f : 0.0f;
            break;
        }
        case InputControlKind::GamepadAxis: {
            const auto found = device.gamepadAxes.find(binding.code);
            value            = found != device.gamepadAxes.end() ? found->second : 0.0f;
            break;
        }
        case InputControlKind::MouseDeltaX:
            value = device.mouseDelta[0];
            break;
        case InputControlKind::MouseDeltaY:
            value = device.mouseDelta[1];
            break;
        case InputControlKind::MouseWheelX:
            value = device.mouseWheel[0];
            break;
        case InputControlKind::MouseWheelY:
            value = device.mouseWheel[1];
            break;
        case InputControlKind::TouchContact: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr && (touch->down || touch->pressedThisFrame) ? 1.0f : 0.0f;
            break;
        }
        case InputControlKind::TouchPositionX: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr ? touch->x : 0.0f;
            break;
        }
        case InputControlKind::TouchPositionY: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr ? touch->y : 0.0f;
            break;
        }
        case InputControlKind::TouchDeltaX: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr ? touch->deltaX : 0.0f;
            break;
        }
        case InputControlKind::TouchDeltaY: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr ? touch->deltaY : 0.0f;
            break;
        }
        case InputControlKind::TouchPressure: {
            const TouchState *touch = findTouch(device, binding.code);
            value = touch != nullptr ? touch->pressure : 0.0f;
            break;
        }
        }
        if (binding.control == InputControlKind::GamepadAxis)
        {
            // deadzone < 0 means "use the module default".
            const float deadzone = binding.deadzone >= 0.0f ? binding.deadzone : defaultDeadzone_;
            if (deadzone > 0.0f)
            {
                if (std::abs(value) <= deadzone)
                    value = 0.0f;
                else
                    value = std::copysign((std::abs(value) - deadzone) / (1.0f - deadzone), value);
            }
        }
        if (binding.invert)
            value = -value;
        value *= binding.scale;
        if (binding.responseExponent != 1.0f && value != 0.0f)
            value = std::copysign(std::pow(std::abs(value), binding.responseExponent), value);
        return value;
    }
    // Per-frame evaluation. Contexts are (lazily) ordered by descending
    // priority then handle; each enabled context combines its bindings per
    // action, and a context with consumeWhenActive hides the controls of its
    // active actions from every later context this frame. Ends by resetting
    // the accumulated mouse deltas and wheel values.
    void advanceFrameImpl(float deltaSeconds)
    {
        // Non-finite or non-positive deltas skip time-based effects.
        const float frameDelta = std::isfinite(deltaSeconds) && deltaSeconds > 0.0f
                                   ? std::min(deltaSeconds, maximumFrameDeltaSeconds_)
                                   : 0.0f;
        if (contextsOrderDirty_)
        {
            // Rebuild the evaluation order only after context changes.
            orderedContexts_.clear();
            for (auto &[id, context] : contexts_)
            {
                (void)id;
                orderedContexts_.push_back(context.get());
            }
            std::sort(orderedContexts_.begin(),
                orderedContexts_.end(),
                [](const ContextRecord *a, const ContextRecord *b)
                {
                    if (a->priority != b->priority)
                        return a->priority > b->priority;
                    return a->handle < b->handle;
                });
            contextsOrderDirty_ = false;
        }
        // Controls already claimed by higher-priority contexts this frame.
        consumedControls_.clear();
        for (ContextRecord *context : orderedContexts_)
        {
            pendingConsumption_.clear();
            bool contextActive = false;
            for (auto &[id, action] : context->actions)
            {
                (void)id;
                const bool       wasDown = action.state.down;
                InputActionState next{};
                activeControls_.clear();
                if (context->enabled)
                {
                    for (const InputBinding &binding : action.bindings)
                    {
                        ControlKey key{binding.control, binding.device, binding.code};
                        if (binding.code == anyTouchContact
                            && binding.control >= InputControlKind::TouchContact
                            && binding.control <= InputControlKind::TouchPressure)
                        {
                            const auto device = devices_.find(binding.device);
                            if (device != devices_.end())
                                (void)findTouch(device->second, anyTouchContact, &key.code);
                        }
                        // Wildcard touch bindings consume the resolved contact,
                        // so they also block lower-priority concrete bindings.
                        if (consumedControls_.contains(key))
                            continue;
                        const float value          = readBinding(binding);
                        float      &componentValue = next.value[binding.component];
                        switch (action.combineMode)
                        {
                        case InputActionCombineMode::Sum:
                            componentValue += value;
                            break;
                        case InputActionCombineMode::MaximumMagnitude:
                            if (std::abs(value) > std::abs(componentValue))
                                componentValue = value;
                            break;
                        case InputActionCombineMode::LastNonZero:
                            if (value != 0.0f)
                                componentValue = value;
                            break;
                        }
                        if (value != 0.0f)
                            activeControls_.push_back(key);
                    }
                    next.value[0] = std::clamp(next.value[0], -1.0f, 1.0f);
                    next.value[1] = std::clamp(next.value[1], -1.0f, 1.0f);
                    if (action.smoothingSeconds > 0.0f && action.type != InputActionType::Button && frameDelta > 0.0f)
                    {
                        // Exponential smoothing toward the raw value; the
                        // time constant is smoothingSeconds.
                        const float alpha = 1.0f - std::exp(-frameDelta / action.smoothingSeconds);
                        for (std::size_t component = 0; component < 2; ++component)
                            next.value[component] = action.state.value[component]
                                                  + (next.value[component] - action.state.value[component]) * alpha;
                    }
                    // Buttons threshold on |x|; Axis2D thresholds on the
                    // Euclidean magnitude (up to ~1.41 on the diagonal).
                    const float magnitude = action.type == InputActionType::Axis2D
                                              ? std::sqrt(next.value[0] * next.value[0] + next.value[1] * next.value[1])
                                              : std::abs(next.value[0]);
                    next.down             = magnitude >= action.threshold;
                    next.pressed          = next.down && !wasDown;
                    next.released         = !next.down && wasDown;
                }
                action.state = next;
                // Collect this context's active controls; they only become
                // consumed when the context opts in and actually fired.
                if (next.down)
                {
                    contextActive = true;
                    pendingConsumption_.insert(
                        pendingConsumption_.end(), activeControls_.begin(), activeControls_.end());
                }
            }
            if (context->enabled && context->consumeWhenActive && contextActive)
                consumedControls_.insert(pendingConsumption_.begin(), pendingConsumption_.end());
        }
        for (auto &[id, device] : devices_)
        {
            (void)id;
            // Relative deltas are per-frame; start the next frame at zero.
            device.mouseDelta[0] = device.mouseDelta[1] = 0.0f;
            device.mouseWheel[0] = device.mouseWheel[1] = 0.0f;
            for (auto touch = device.touches.begin(); touch != device.touches.end();)
            {
                touch->second.deltaX = 0.0f;
                touch->second.deltaY = 0.0f;
                touch->second.pressedThisFrame = false;
                if (!touch->second.down)
                    touch = device.touches.erase(touch);
                else
                    ++touch;
            }
        }
    }

    bool actionStateImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, InputActionState *output)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr || output == nullptr)
            return false;
        *output = action->state;
        return true;
    }
    bool copyActionNameImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, char *output,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ContextRecord *context = findContext(contextHandle);
        ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
        if (action == nullptr)
            return false;
        const std::uint32_t required = static_cast<std::uint32_t>(action->name.size() + 1);
        if (outRequired != nullptr)
            *outRequired = required;
        if (output == nullptr || capacity < required)
            return false;
        std::memcpy(output, action->name.c_str(), required);
        return true;
    }
    void resetDeviceStateImpl(std::uint32_t device)
    {
        devices_.erase(device);
    }

    // Module-wide scratch state for advanceFrame: the lazily sorted context
    // list and the per-frame consumption sets. All of it is touched only by
    // capability calls, which the module assumes come from a single thread.
    ParticipantManifest                                                    manifest_;
    InputActionsCapability                                                 capability_{};
    std::mutex                                                              mutex_;
    script_runtime::ScriptRuntimeCapability                                *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_
        = script_runtime::invalidScriptRuntimeInitializer;
    InputActionsLuaBridgeContext luaBridgeContext_{};
    std::unordered_map<InputContextHandle, std::unique_ptr<ContextRecord>> contexts_;
    std::unordered_map<std::uint32_t, DeviceState>                         devices_;
    std::vector<ContextRecord *>                                           orderedContexts_;
    bool                                                                   contextsOrderDirty_ = true;
    std::unordered_set<ControlKey, ControlKeyHash>                         consumedControls_;
    std::vector<ControlKey>                                                pendingConsumption_;
    std::vector<ControlKey>                                                activeControls_;
    InputContextHandle                                                     nextContext_              = 1;
    InputActionHandle                                                      nextAction_               = 1;
    std::size_t                                                            maximumContexts_          = 64;
    std::size_t                                                            maximumActionsPerContext_ = 1024;
    std::size_t                                                            maximumBindingsPerAction_ = 64;
    std::size_t                                                            maximumDevices_           = 32;
    std::size_t                                                            maximumControlsPerDevice_ = 4096;
    float                                                                  defaultDeadzone_          = 0.15f;
    float                                                                  maximumSmoothingSeconds_  = 2.0f;
    float                                                                  maximumFrameDeltaSeconds_ = 0.25f;
};

} // namespace
} // namespace engine::input_actions

// Module entry table: kernel ABI, module identity, and the single provided
// capability; no requirements, optional capabilities, or permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"input_actions"};
    static const char *const                    optional[] = {"script_runtime"};
    static const engine::EngineModuleEntryTable entry      = {
        .abiVersion    = engine::engineModuleAbiVersion,
        .moduleName    = "input_actions",
        .moduleVersion = "0.1.0",
        .createModule  = []() -> engine::ModuleInterface  *{ return new engine::input_actions::InputActionsModule(); },
        .providedCapabilities    = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = optional,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
