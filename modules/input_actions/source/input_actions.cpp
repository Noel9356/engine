// input_actions module entry point.
//
// Wires the device registry, context registry, and frame evaluator into a
// ModuleInterface implementation, serializes capability entry points with one
// mutex, and binds the C capability vtable. Internal types live in
// source/internal.hpp and the cooperating pieces each have their own header.

#include <engine/input_actions/capabilities.hpp>
#include "configuration.hpp"
#include "context_registry.hpp"
#include "device_state.hpp"
#include "frame_evaluator.hpp"
#include "internal.hpp"
#include "lua_bridge.hpp"
#include "window_adapter.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace engine::input_actions
{
namespace
{

// ModuleInterface implementation; state persists across frames in the device
// snapshots and action states, and the module is safe only for single-thread
// use; capability entry points serialize access with an internal mutex.
class InputActionsModule final : public ModuleInterface
{
public:
    InputActionsModule() : manifest_(makeManifest()) { assignCapabilityPointers(); }

    const ParticipantManifest &manifest() const override { return manifest_; }

    void construct(ParticipantContext &context) override
    {
        config_ = internal::loadInputActionsConfiguration(context);
        devices_   = std::make_unique<internal::DeviceStateRegistry>(config_);
        contexts_  = std::make_unique<internal::ContextRegistry>(config_);
        evaluator_ = std::make_unique<internal::FrameEvaluator>(config_, *devices_, *contexts_);
        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.inputActions  = &capability_;
            runtimeInitializer_             = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeInputActionsLuaRuntime, &luaBridgeContext_);
        }
    }
    void start() override {}
    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
        {
            std::lock_guard bridgeLock(luaBridgeContext_.apiMutex);
            luaBridgeContext_.inputActions = nullptr;
        }
        std::lock_guard lock(mutex_);
        if (contexts_)
            contexts_->clear();
        if (devices_)
            devices_->clear();
        // The frame evaluator's scratch containers are cheap to leave as-is;
        // clearing them only saves a small amount of reserved capacity, and
        // the next construct() rebuilds the evaluator anyway.
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
    // Each capability entry point locks the module mutex then forwards to the
    // matching registry/evaluator method. The lambda indirection keeps the
    // vtable self-contained so consumers call self->fn(self, ...) verbatim.
    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_INPUT_ACTIONS_BIND(field, method)                                                                       \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    {                                                                                                                   \
        auto          *module = static_cast<InputActionsModule *>(self);                                                \
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

    // --- Context CRUD ---
    InputContextHandle createContextImpl(const InputContextDescription *description)
    {
        if (description == nullptr)
            return invalidInputContext;
        return contexts_->createContext(*description);
    }
    void destroyContextImpl(InputContextHandle handle) { contexts_->destroyContext(handle); }
    bool setContextEnabledImpl(InputContextHandle handle, bool enabled)
    {
        return contexts_->setContextEnabled(handle, enabled);
    }

    // --- Action CRUD ---
    InputActionHandle createActionImpl(InputContextHandle contextHandle, const InputActionDescription *description)
    {
        if (description == nullptr)
            return invalidInputAction;
        return contexts_->createAction(contextHandle, *description);
    }
    bool removeActionImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        return contexts_->removeAction(contextHandle, actionHandle);
    }

    // --- Binding CRUD ---
    bool bindActionImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, const InputBinding *binding)
    {
        if (binding == nullptr)
            return false;
        return contexts_->bindAction(contextHandle, actionHandle, *binding);
    }
    bool clearBindingsImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        return contexts_->clearBindings(contextHandle, actionHandle);
    }
    bool removeBindingImpl(
        InputContextHandle contextHandle, InputActionHandle actionHandle, std::uint32_t bindingIndex)
    {
        return contexts_->removeBinding(contextHandle, actionHandle, bindingIndex);
    }
    std::uint32_t bindingCountImpl(InputContextHandle contextHandle, InputActionHandle actionHandle)
    {
        return contexts_->bindingCount(contextHandle, actionHandle);
    }
    bool copyBindingsImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, InputBinding *output,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        return contexts_->copyBindings(contextHandle, actionHandle, output, capacity, outRequired);
    }

    // --- Event ingestion ---
    bool feedEventImpl(const InputEvent *event)
    {
        if (event == nullptr)
            return false;
        return devices_->feedEvent(*event);
    }
    bool feedWindowEventImpl(window::WindowHandle windowHandle, const window::WindowEvent *event)
    {
        if (event == nullptr)
            return false;
        return internal::feedWindowEvent(*devices_, windowHandle, *event);
    }

    // --- Frame pipeline ---
    void advanceFrameImpl(float deltaSeconds) { evaluator_->advanceFrame(deltaSeconds); }

    // --- State access ---
    bool actionStateImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, InputActionState *output)
    {
        if (output == nullptr)
            return false;
        return contexts_->actionState(contextHandle, actionHandle, *output);
    }
    bool copyActionNameImpl(InputContextHandle contextHandle, InputActionHandle actionHandle, char *output,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        return contexts_->copyActionName(contextHandle, actionHandle, output, capacity, outRequired);
    }
    void resetDeviceStateImpl(std::uint32_t device) { devices_->reset(device); }

    ParticipantManifest                                              manifest_;
    InputActionsCapability                                           capability_{};
    internal::InputActionsConfiguration                              config_{};
    std::mutex                                                       mutex_;
    script_runtime::ScriptRuntimeCapability                         *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle                   runtimeInitializer_
        = script_runtime::invalidScriptRuntimeInitializer;
    InputActionsLuaBridgeContext                                     luaBridgeContext_{};
    std::unique_ptr<internal::DeviceStateRegistry>                  devices_;
    std::unique_ptr<internal::ContextRegistry>                       contexts_;
    std::unique_ptr<internal::FrameEvaluator>                        evaluator_;
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
        .abiVersion               = engine::engineModuleAbiVersion,
        .moduleName               = "input_actions",
        .moduleVersion            = "0.1.0",
        .createModule             = []() -> engine::ModuleInterface *{ return new engine::input_actions::InputActionsModule(); },
        .providedCapabilities     = provided,
        .providedCapabilityCount  = 1,
        .requiredCapabilities     = nullptr,
        .requiredCapabilityCount  = 0,
        .optionalCapabilities     = optional,
        .optionalCapabilityCount  = 1,
        .declaredPermissions      = nullptr,
        .declaredPermissionCount  = 0};
    return &entry;
}
