#include "context_registry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace engine::input_actions
{
namespace internal
{

InputContextHandle ContextRegistry::createContext(const InputContextDescription &description)
{
    if (contexts_.size() >= config_.maximumContexts)
        return invalidInputContext;
    if (nextContext_ == invalidInputContext || nextContext_ == std::numeric_limits<InputContextHandle>::max())
        return invalidInputContext;
    auto context    = std::make_unique<ContextRecord>();
    context->handle = nextContext_++;
    context->name   = description.name != nullptr ? description.name : "input_context";
    context->priority          = description.priority;
    context->enabled           = description.enabled;
    context->consumeWhenActive = description.consumeWhenActive;
    const InputContextHandle handle = context->handle;
    contexts_.emplace(handle, std::move(context));
    contextsOrderDirty_ = true;
    return handle;
}

void ContextRegistry::destroyContext(InputContextHandle handle)
{
    if (contexts_.erase(handle) != 0)
        contextsOrderDirty_ = true;
}

bool ContextRegistry::setContextEnabled(InputContextHandle handle, bool enabled)
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

InputActionHandle ContextRegistry::createAction(
    InputContextHandle contextHandle, const InputActionDescription &description)
{
    ContextRecord *context = findContext(contextHandle);
    if (context == nullptr || description.name == nullptr || description.name[0] == '\0'
        || static_cast<std::uint32_t>(description.type) > static_cast<std::uint32_t>(InputActionType::Axis2D)
        || context->actions.size() >= config_.maximumActionsPerContext || !std::isfinite(description.activationThreshold)
        || description.activationThreshold < 0.0f || description.activationThreshold > 1.0f
        || !std::isfinite(description.smoothingSeconds) || description.smoothingSeconds < 0.0f
        || description.smoothingSeconds > config_.maximumSmoothingSeconds
        || static_cast<std::uint32_t>(description.combineMode)
               > static_cast<std::uint32_t>(InputActionCombineMode::LastNonZero))
        return invalidInputAction;
    // Names are unique per context; the parallel unordered_set keeps this O(1).
    if (!context->actionNames.insert(description.name).second)
        return invalidInputAction;
    if (nextAction_ == invalidInputAction || nextAction_ == std::numeric_limits<InputActionHandle>::max())
    {
        context->actionNames.erase(description.name);
        return invalidInputAction;
    }
    ActionRecord action{};
    action.handle           = nextAction_++;
    action.name             = description.name;
    action.type             = description.type;
    action.threshold        = description.activationThreshold;
    action.combineMode      = description.combineMode;
    action.smoothingSeconds = description.smoothingSeconds;
    const InputActionHandle result = action.handle;
    context->actions.emplace(result, std::move(action));
    return result;
}

bool ContextRegistry::removeAction(InputContextHandle contextHandle, InputActionHandle actionHandle)
{
    ContextRecord *context = findContext(contextHandle);
    if (context == nullptr)
        return false;
    const auto it = context->actions.find(actionHandle);
    if (it == context->actions.end())
        return false;
    context->actionNames.erase(it->second.name);
    context->actions.erase(it);
    return true;
}

bool ContextRegistry::bindAction(
    InputContextHandle contextHandle, InputActionHandle actionHandle, const InputBinding &binding)
{
    ContextRecord *context = findContext(contextHandle);
    ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
    if (action == nullptr || binding.component > 1 || !std::isfinite(binding.scale) || !std::isfinite(binding.deadzone)
        || binding.deadzone < -1.0f || binding.deadzone >= 1.0f || !std::isfinite(binding.responseExponent)
        || binding.responseExponent <= 0.0f || binding.responseExponent > 8.0f
        || static_cast<std::uint32_t>(binding.control) > static_cast<std::uint32_t>(InputControlKind::MousePositionY))
        return false;
    if (action->bindings.size() >= config_.maximumBindingsPerAction)
        return false;
    // Per-type component rules: Button actions accept only digital controls
    // with component 0 (TouchContact is digital; touch position/delta/pressure,
    // gamepad axes, and mouse position are analog). Axis1D requires component 0.
    // Axis2D accepts components 0 and 1.
    if (action->type == InputActionType::Button && (isAnalogControl(binding.control) || binding.component != 0))
        return false;
    if (action->type == InputActionType::Axis1D && binding.component != 0)
        return false;
    action->bindings.push_back(binding);
    return true;
}

bool ContextRegistry::clearBindings(InputContextHandle contextHandle, InputActionHandle actionHandle)
{
    ContextRecord *context = findContext(contextHandle);
    ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
    if (action == nullptr)
        return false;
    action->bindings.clear();
    action->state = {};
    return true;
}

bool ContextRegistry::removeBinding(
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

std::uint32_t ContextRegistry::bindingCount(InputContextHandle contextHandle, InputActionHandle actionHandle) const
{
    const ContextRecord *context = findContext(contextHandle);
    const ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
    return action != nullptr ? static_cast<std::uint32_t>(action->bindings.size()) : 0u;
}

bool ContextRegistry::copyBindings(InputContextHandle contextHandle, InputActionHandle actionHandle, InputBinding *output,
    std::uint32_t capacity, std::uint32_t *outRequired) const
{
    const ContextRecord *context = findContext(contextHandle);
    const ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
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

bool ContextRegistry::actionState(
    InputContextHandle contextHandle, InputActionHandle actionHandle, InputActionState &output) const
{
    const ContextRecord *context = findContext(contextHandle);
    const ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
    if (action == nullptr)
        return false;
    output = action->state;
    return true;
}

bool ContextRegistry::copyActionName(InputContextHandle contextHandle, InputActionHandle actionHandle, char *output,
    std::uint32_t capacity, std::uint32_t *outRequired) const
{
    const ContextRecord *context = findContext(contextHandle);
    const ActionRecord  *action  = context != nullptr ? findAction(*context, actionHandle) : nullptr;
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

void ContextRegistry::refreshOrder()
{
    if (!contextsOrderDirty_)
        return;
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

} // namespace internal
} // namespace engine::input_actions
