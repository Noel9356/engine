// Context and action records plus the CRUD registry that owns them.
//
// The registry is not thread-safe; the InputActionsModule serializes every
// capability entry point with one mutex and the registry assumes the caller
// holds it. Handles are dense std::uint32_t ids starting at 1; the constants
// invalidInputContext / invalidInputAction (0) are never handed out.

#ifndef ENGINE_INPUT_ACTIONS_CONTEXT_REGISTRY_HPP
#define ENGINE_INPUT_ACTIONS_CONTEXT_REGISTRY_HPP

#include "configuration.hpp"
#include "internal.hpp"

#include <engine/input_actions/capabilities.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine::input_actions
{
namespace internal
{

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
    // Parallel name set so createAction's uniqueness check is O(1) instead of
    // a linear scan over the action map.
    std::unordered_set<std::string>                     actionNames;
};

class ContextRegistry
{
public:
    explicit ContextRegistry(const InputActionsConfiguration &config) : config_(config) {}

    // --- Context lifecycle ---
    InputContextHandle createContext(const InputContextDescription &description);
    void                destroyContext(InputContextHandle handle);
    bool                setContextEnabled(InputContextHandle handle, bool enabled);

    // --- Action lifecycle ---
    InputActionHandle createAction(InputContextHandle contextHandle, const InputActionDescription &description);
    bool              removeAction(InputContextHandle contextHandle, InputActionHandle actionHandle);

    // --- Binding lifecycle ---
    bool           bindAction(InputContextHandle contextHandle, InputActionHandle actionHandle, const InputBinding &binding);
    bool           clearBindings(InputContextHandle contextHandle, InputActionHandle actionHandle);
    bool           removeBinding(InputContextHandle contextHandle, InputActionHandle actionHandle, std::uint32_t bindingIndex);
    std::uint32_t  bindingCount(InputContextHandle contextHandle, InputActionHandle actionHandle) const;
    bool           copyBindings(InputContextHandle contextHandle, InputActionHandle actionHandle, InputBinding *output,
        std::uint32_t capacity, std::uint32_t *outRequired) const;

    // --- State access ---
    bool actionState(InputContextHandle contextHandle, InputActionHandle actionHandle, InputActionState &output) const;
    bool copyActionName(InputContextHandle contextHandle, InputActionHandle actionHandle, char *output,
        std::uint32_t capacity, std::uint32_t *outRequired) const;

    // --- Iteration ---
    // Rebuilds the ordered context list if dirty (called once per frame).
    void refreshOrder();
    // Stable iteration over contexts in priority order; refreshOrder() must
    // have been called at least once.
    const std::vector<ContextRecord *> &orderedContexts() const noexcept { return orderedContexts_; }

    void clear()
    {
        contexts_.clear();
        orderedContexts_.clear();
        contextsOrderDirty_ = true;
    }

private:
    ContextRecord *findContext(InputContextHandle handle)
    {
        const auto it = contexts_.find(handle);
        return it == contexts_.end() ? nullptr : it->second.get();
    }
    const ContextRecord *findContext(InputContextHandle handle) const
    {
        const auto it = contexts_.find(handle);
        return it == contexts_.end() ? nullptr : it->second.get();
    }
    static ActionRecord *findAction(ContextRecord &context, InputActionHandle handle)
    {
        const auto it = context.actions.find(handle);
        return it == context.actions.end() ? nullptr : &it->second;
    }
    static const ActionRecord *findAction(const ContextRecord &context, InputActionHandle handle)
    {
        const auto it = context.actions.find(handle);
        return it == context.actions.end() ? nullptr : &it->second;
    }

    const InputActionsConfiguration                                            &config_;
    std::unordered_map<InputContextHandle, std::unique_ptr<ContextRecord>>     contexts_;
    std::vector<ContextRecord *>                                               orderedContexts_;
    bool                                                                        contextsOrderDirty_ = true;
    InputContextHandle                                                          nextContext_        = 1;
    InputActionHandle                                                           nextAction_         = 1;
};

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_CONTEXT_REGISTRY_HPP
