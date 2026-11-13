// Lua bridge implementation for the character module.
//
// Every native validates its arguments before touching the capability so a
// script mistake fails the call instead of the engine. Functions:
//
//   character.create(profile_or_nil)        -> handle (0 on failure)
//   character.destroy(handle)
//   character.add_part(handle, part, name)  -> instance id (0 on failure)
//   character.remove_part(handle, instance) -> bool
//   character.replace_part(handle, instance, part) -> bool
//   character.connect(handle, parent, parent_connector, child, child_connector) -> bool
//   character.disconnect(handle, child)     -> bool
//   character.set_parameter(handle, name, value) -> bool
//   character.get_parameter(handle, name)   -> value, ok
//   character.set_connection_offset(handle, child, x, y, z [, qx, qy, qz, qw]) -> bool
//   character.set_assembly_transform(handle, x, y, z, qx, qy, qz, qw) -> bool
//   character.evaluate(handle)              -> bool
//   character.validate(handle)              -> issue_count
//   character.validation_issue(handle, index) -> code, instance, subject, message, ok
//   character.statistics(handle)            -> parts, connections, joints, triangles, issues, ok
//   character.parameter_count()             -> count
//   character.parameter_info(index)         -> id, display_name, minimum, maximum, default, ok
//   character.part_count()                  -> count
//   character.part_id(index)                -> part_id
//   character.instance_count(handle)        -> count
//   character.instance_id(handle, index)    -> instance id (0 when out of range)
//   character.instance_info(handle, instance) -> name, part_id, parent, ok
//   character.instance_transform(handle, instance) -> m00..m32 (16 numbers;
//       no values when the instance has no evaluated transform)
//   character.bounds(handle)                -> cx, cy, cz, radius, ok
//   character.save(handle, path) / character.load(handle, path) -> bool
//   character.last_error()                  -> message_or_nil

#include "lua_bridge.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace engine::character
{
namespace
{

using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;

bool numberArgument(const ScriptValue *arguments, std::uint32_t count, std::uint32_t index, double &out)
{
    if (index >= count || arguments[index].type != ScriptValueType::Number
        || !std::isfinite(arguments[index].numberValue))
    {
        return false;
    }
    out = arguments[index].numberValue;
    return true;
}

bool integralArgument(const ScriptValue *arguments, std::uint32_t count, std::uint32_t index, double minimum,
    double maximum, double &out)
{
    double value = 0.0;
    if (!numberArgument(arguments, count, index, value) || value < minimum || value > maximum
        || std::floor(value) != value)
    {
        return false;
    }
    out = value;
    return true;
}

bool handleArgument(const ScriptValue *arguments, std::uint32_t count, std::uint32_t index, CharacterHandle &out)
{
    double value = 0.0;
    if (!integralArgument(arguments, count, index, 1.0,
            static_cast<double>(std::numeric_limits<std::uint32_t>::max()), value))
    {
        return false;
    }
    out = static_cast<CharacterHandle>(value);
    return true;
}

bool instanceArgument(const ScriptValue *arguments, std::uint32_t count, std::uint32_t index, InstanceId &out)
{
    double value = 0.0;
    if (!integralArgument(arguments, count, index, 0.0,
            static_cast<double>(std::numeric_limits<std::uint32_t>::max()), value))
    {
        return false;
    }
    out = static_cast<InstanceId>(value);
    return true;
}

bool stringArgument(const ScriptValue *arguments, std::uint32_t count, std::uint32_t index, std::string_view &out)
{
    if (index >= count || arguments[index].type != ScriptValueType::String || arguments[index].stringData == nullptr)
    {
        return false;
    }
    out = std::string_view(arguments[index].stringData, arguments[index].stringSize);
    // Capability ids are C strings: a string with an embedded NUL would be
    // silently truncated, so it fails the call instead.
    if (out.find('\0') != std::string_view::npos)
    {
        return false;
    }
    return true;
}

bool finishBoolean(ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount, bool value)
{
    if (outCount != nullptr)
    {
        *outCount = capacity >= 1 ? 1u : 0u;
    }
    if (results == nullptr || capacity == 0)
    {
        return false;
    }
    results[0].type         = ScriptValueType::Boolean;
    results[0].booleanValue = value;
    return true;
}

bool finishNumber(ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount, double value)
{
    if (outCount != nullptr)
    {
        *outCount = capacity >= 1 ? 1u : 0u;
    }
    if (results == nullptr || capacity == 0)
    {
        return false;
    }
    results[0].type        = ScriptValueType::Number;
    results[0].numberValue = value;
    return true;
}

bool finishNothing(ScriptValue *, std::uint32_t, std::uint32_t *outCount)
{
    if (outCount != nullptr)
    {
        *outCount = 0u;
    }
    return true;
}

bool finishString(ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount, const char *value)
{
    if (outCount != nullptr)
    {
        *outCount = capacity >= 1 ? 1u : 0u;
    }
    if (results == nullptr || capacity == 0)
    {
        return false;
    }
    results[0].type       = ScriptValueType::String;
    results[0].stringData = value != nullptr ? value : "";
    results[0].stringSize = static_cast<std::uint32_t>(std::strlen(results[0].stringData));
    return true;
}

struct Guard
{
    explicit Guard(CharacterLuaBridgeContext *context)
        : lock(context != nullptr && context->mutex != nullptr ? std::unique_lock<std::recursive_mutex>(*context->mutex)
                                                               : std::unique_lock<std::recursive_mutex>()),
          bridge(context)
    {}

    std::unique_lock<std::recursive_mutex> lock;
    CharacterLuaBridgeContext             *bridge;
};

bool createNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard guard(static_cast<CharacterLuaBridgeContext *>(userData));
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count > 1)
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    const char *profileId = nullptr;
    if (count == 1)
    {
        if (arguments[0].type == ScriptValueType::String && arguments[0].stringData != nullptr)
        {
            profileId = arguments[0].stringData;
        }
        else if (arguments[0].type != ScriptValueType::Nil)
        {
            // Anything but a profile name or nil fails the call instead of
            // silently creating an empty assembly.
            return finishNumber(results, capacity, outCount, 0.0);
        }
    }
    CharacterError        error = CharacterError::None;
    const CharacterHandle handle
        = guard.bridge->character->createAssembly(guard.bridge->character->self, profileId, &error);
    return finishNumber(results, capacity, outCount, static_cast<double>(handle));
}

bool destroyNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    guard.bridge->character->destroyAssembly(guard.bridge->character->self, handle);
    return finishBoolean(results, capacity, outCount, true);
}

bool addPartNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    std::string_view partId;
    std::string_view instanceName;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 3
        || !handleArgument(arguments, count, 0, handle) || !stringArgument(arguments, count, 1, partId)
        || !stringArgument(arguments, count, 2, instanceName))
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    CharacterError   error    = CharacterError::None;
    const InstanceId instance = guard.bridge->character->addPart(
        guard.bridge->character->self, handle, std::string(partId).c_str(), std::string(instanceName).c_str(), &error);
    return finishNumber(results, capacity, outCount, static_cast<double>(instance));
}

bool removePartNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle   = invalidCharacter;
    InstanceId      instance = invalidInstance;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, instance))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->removePart(guard.bridge->character->self, handle, instance));
}

bool replacePartNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle   = invalidCharacter;
    InstanceId       instance = invalidInstance;
    std::string_view partId;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 3
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, instance)
        || !stringArgument(arguments, count, 2, partId))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->replacePart(
            guard.bridge->character->self, handle, instance, std::string(partId).c_str(), &error));
}

bool connectNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    InstanceId       parent = invalidInstance;
    InstanceId       child  = invalidInstance;
    std::string_view parentConnector;
    std::string_view childConnector;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 5
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, parent)
        || !stringArgument(arguments, count, 2, parentConnector) || !instanceArgument(arguments, count, 3, child)
        || !stringArgument(arguments, count, 4, childConnector))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->connect(guard.bridge->character->self,
            handle,
            parent,
            std::string(parentConnector).c_str(),
            child,
            std::string(childConnector).c_str(),
            nullptr,
            nullptr,
            &error));
}

bool disconnectNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    InstanceId      child  = invalidInstance;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, child))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    return finishBoolean(
        results, capacity, outCount, guard.bridge->character->disconnect(guard.bridge->character->self, handle, child));
}

bool setParameterNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    std::string_view parameter;
    double           value = 0.0;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 3
        || !handleArgument(arguments, count, 0, handle) || !stringArgument(arguments, count, 1, parameter)
        || !numberArgument(arguments, count, 2, value))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->setParameter(
            guard.bridge->character->self, handle, std::string(parameter).c_str(), static_cast<float>(value), &error));
}

bool getParameterNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    std::string_view parameter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !stringArgument(arguments, count, 1, parameter)
        || capacity < 2 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    float      value = 0.0f;
    const bool found = guard.bridge->character->getParameter(
        guard.bridge->character->self, handle, std::string(parameter).c_str(), &value);
    if (outCount != nullptr)
    {
        *outCount = 2u;
    }
    if (!found)
    {
        results[0].type         = ScriptValueType::Nil;
        results[1].type         = ScriptValueType::Boolean;
        results[1].booleanValue = false;
        return true;
    }
    results[0].type         = ScriptValueType::Number;
    results[0].numberValue  = value;
    results[1].type         = ScriptValueType::Boolean;
    results[1].booleanValue = true;
    return true;
}

bool setConnectionOffsetNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle         = invalidCharacter;
    InstanceId      child          = invalidInstance;
    double          translation[3] = {0.0, 0.0, 0.0};
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || (count != 5 && count != 9)
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, child))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        if (!numberArgument(arguments, count, 2 + axis, translation[axis]))
        {
            return finishBoolean(results, capacity, outCount, false);
        }
    }
    const float offsets[3]
        = {static_cast<float>(translation[0]), static_cast<float>(translation[1]), static_cast<float>(translation[2])};
    if (count == 5)
    {
        return finishBoolean(results,
            capacity,
            outCount,
            guard.bridge->character->setConnectionOffset(
                guard.bridge->character->self, handle, child, offsets, nullptr));
    }
    double rotation[4] = {0.0, 0.0, 0.0, 1.0};
    for (std::uint32_t axis = 0; axis < 4; ++axis)
    {
        if (!numberArgument(arguments, count, 5 + axis, rotation[axis]))
        {
            return finishBoolean(results, capacity, outCount, false);
        }
    }
    const float quaternion[4] = {static_cast<float>(rotation[0]), static_cast<float>(rotation[1]),
        static_cast<float>(rotation[2]), static_cast<float>(rotation[3])};
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->setConnectionOffset(
            guard.bridge->character->self, handle, child, offsets, quaternion));
}

bool setAssemblyTransformNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    double          values[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 8
        || !handleArgument(arguments, count, 0, handle))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    for (std::uint32_t index = 0; index < 7; ++index)
    {
        if (!numberArgument(arguments, count, 1 + index, values[index]))
        {
            return finishBoolean(results, capacity, outCount, false);
        }
    }
    const float translation[3] = {static_cast<float>(values[0]), static_cast<float>(values[1]),
        static_cast<float>(values[2])};
    const float rotation[4]    = {static_cast<float>(values[3]), static_cast<float>(values[4]),
        static_cast<float>(values[5]), static_cast<float>(values[6])};
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->setAssemblyTransform(
            guard.bridge->character->self, handle, translation, rotation));
}

bool evaluateNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(
        results, capacity, outCount, guard.bridge->character->evaluate(guard.bridge->character->self, handle, &error));
}

bool validateNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle))
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    return finishNumber(results,
        capacity,
        outCount,
        static_cast<double>(guard.bridge->character->validate(guard.bridge->character->self, handle, nullptr, 0)));
}

bool validationIssueNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle  = invalidCharacter;
    double          index   = 0.0;
    constexpr double minimum = 1.0;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle)
        || !integralArgument(arguments,
            count,
            1,
            minimum,
            static_cast<double>(std::numeric_limits<std::uint32_t>::max()),
            index)
        || capacity < 5 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    const std::uint32_t total = guard.bridge->character->validate(guard.bridge->character->self, handle, nullptr, 0);
    const std::uint32_t issueIndex = static_cast<std::uint32_t>(index) - 1u;
    if (issueIndex >= total)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    // The capability reports the total first; fetch the requested issue with
    // a one-shot buffer sized to it.
    std::vector<ValidationIssue> issues(issueIndex + 1u);
    guard.bridge->character->validate(
        guard.bridge->character->self, handle, issues.data(), static_cast<std::uint32_t>(issues.size()));
    const ValidationIssue &issue = issues[issueIndex];
    if (outCount != nullptr)
    {
        *outCount = 5u;
    }
    results[0].type        = ScriptValueType::Number;
    results[0].numberValue = static_cast<double>(issue.code);
    results[1].type        = ScriptValueType::Number;
    results[1].numberValue = static_cast<double>(issue.instance);
    results[2].type        = ScriptValueType::String;
    results[2].stringData  = issue.subject;
    results[2].stringSize  = static_cast<std::uint32_t>(std::strlen(issue.subject));
    results[3].type        = ScriptValueType::String;
    results[3].stringData  = issue.message;
    results[3].stringSize  = static_cast<std::uint32_t>(std::strlen(issue.message));
    results[4].type         = ScriptValueType::Boolean;
    results[4].booleanValue = true;
    return true;
}

bool statisticsNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle) || capacity < 6 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterStatistics statistics{};
    if (!guard.bridge->character->statistics(guard.bridge->character->self, handle, &statistics))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    if (outCount != nullptr)
    {
        *outCount = 6u;
    }
    results[0].type        = ScriptValueType::Number;
    results[0].numberValue = static_cast<double>(statistics.partCount);
    results[1].type        = ScriptValueType::Number;
    results[1].numberValue = static_cast<double>(statistics.connectionCount);
    results[2].type        = ScriptValueType::Number;
    results[2].numberValue = static_cast<double>(statistics.jointCount);
    results[3].type        = ScriptValueType::Number;
    results[3].numberValue = static_cast<double>(statistics.triangleCount);
    results[4].type        = ScriptValueType::Number;
    results[4].numberValue = static_cast<double>(statistics.issueCount);
    results[5].type         = ScriptValueType::Boolean;
    results[5].booleanValue = true;
    return true;
}

bool parameterCountNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    (void)arguments;
    (void)count;
    Guard guard(static_cast<CharacterLuaBridgeContext *>(userData));
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 0)
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    return finishNumber(results,
        capacity,
        outCount,
        static_cast<double>(guard.bridge->character->parameterCount(guard.bridge->character->self)));
}

bool parameterInfoNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    double           index = 0.0;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !integralArgument(arguments, count, 0, 1.0, static_cast<double>(std::numeric_limits<std::uint32_t>::max()),
            index)
        || capacity < 6 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    ParameterDefinition definition{};
    if (!guard.bridge->character->parameterInfo(
            guard.bridge->character->self, static_cast<std::uint32_t>(index) - 1u, &definition))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    if (outCount != nullptr)
    {
        *outCount = 6u;
    }
    results[0].type        = ScriptValueType::String;
    results[0].stringData  = definition.id != nullptr ? definition.id : "";
    results[0].stringSize  = static_cast<std::uint32_t>(std::strlen(results[0].stringData));
    results[1].type        = ScriptValueType::String;
    results[1].stringData  = definition.displayName != nullptr ? definition.displayName : "";
    results[1].stringSize  = static_cast<std::uint32_t>(std::strlen(results[1].stringData));
    results[2].type        = ScriptValueType::Number;
    results[2].numberValue = definition.minimum;
    results[3].type        = ScriptValueType::Number;
    results[3].numberValue = definition.maximum;
    results[4].type        = ScriptValueType::Number;
    results[4].numberValue = definition.defaultV;
    results[5].type         = ScriptValueType::Boolean;
    results[5].booleanValue = true;
    return true;
}

bool partCountNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    (void)arguments;
    (void)count;
    Guard guard(static_cast<CharacterLuaBridgeContext *>(userData));
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 0)
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    return finishNumber(results,
        capacity,
        outCount,
        static_cast<double>(guard.bridge->character->partCount(guard.bridge->character->self)));
}

bool partIdNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard  guard(static_cast<CharacterLuaBridgeContext *>(userData));
    double index = 0.0;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !integralArgument(arguments, count, 0, 1.0, static_cast<double>(std::numeric_limits<std::uint32_t>::max()),
            index))
    {
        return finishString(results, capacity, outCount, "");
    }
    const char *partId = nullptr;
    if (!guard.bridge->character->partId(
            guard.bridge->character->self, static_cast<std::uint32_t>(index) - 1u, &partId))
    {
        return finishString(results, capacity, outCount, "");
    }
    return finishString(results, capacity, outCount, partId);
}

bool instanceCountNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle))
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    return finishNumber(results,
        capacity,
        outCount,
        static_cast<double>(guard.bridge->character->instanceCount(guard.bridge->character->self, handle)));
}

bool instanceTransformNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle   = invalidCharacter;
    InstanceId      instance = invalidInstance;
    // script_runtime caps native results at 16; the matrix alone fills them,
    // so failure is reported as "no values" rather than a trailing flag.
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, instance)
        || capacity < 16 || results == nullptr)
    {
        if (outCount != nullptr)
        {
            *outCount = 0u;
        }
        return true;
    }
    float matrix[16] = {};
    if (!guard.bridge->character->instanceTransform(guard.bridge->character->self, handle, instance, matrix))
    {
        if (outCount != nullptr)
        {
            *outCount = 0u;
        }
        return true;
    }
    if (outCount != nullptr)
    {
        *outCount = 16u;
    }
    for (std::uint32_t index = 0; index < 16; ++index)
    {
        results[index].type        = ScriptValueType::Number;
        results[index].numberValue = matrix[index];
    }
    return true;
}

bool instanceIdNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    double          index  = 0.0;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle)
        || !integralArgument(arguments, count, 1, 1.0, static_cast<double>(std::numeric_limits<std::uint32_t>::max()),
            index))
    {
        return finishNumber(results, capacity, outCount, 0.0);
    }
    const InstanceId instance = guard.bridge->character->instanceId(
        guard.bridge->character->self, handle, static_cast<std::uint32_t>(index) - 1u);
    return finishNumber(results, capacity, outCount, static_cast<double>(instance));
}

bool instanceInfoNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle   = invalidCharacter;
    InstanceId      instance = invalidInstance;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !instanceArgument(arguments, count, 1, instance)
        || capacity < 4 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    const char *name   = nullptr;
    const char *partId = nullptr;
    InstanceId  parent = invalidInstance;
    if (!guard.bridge->character->instanceInfo(
            guard.bridge->character->self, handle, instance, &name, &partId, &parent))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    if (outCount != nullptr)
    {
        *outCount = 4u;
    }
    results[0].type        = ScriptValueType::String;
    results[0].stringData  = name != nullptr ? name : "";
    results[0].stringSize  = static_cast<std::uint32_t>(std::strlen(results[0].stringData));
    results[1].type        = ScriptValueType::String;
    results[1].stringData  = partId != nullptr ? partId : "";
    results[1].stringSize  = static_cast<std::uint32_t>(std::strlen(results[1].stringData));
    results[2].type        = ScriptValueType::Number;
    results[2].numberValue = static_cast<double>(parent);
    results[3].type         = ScriptValueType::Boolean;
    results[3].booleanValue = true;
    return true;
}

bool boundsNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard           guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle handle = invalidCharacter;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 1
        || !handleArgument(arguments, count, 0, handle) || capacity < 5 || results == nullptr)
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    float center[3] = {0.0f, 0.0f, 0.0f};
    float radius    = 0.0f;
    if (!guard.bridge->character->bounds(guard.bridge->character->self, handle, center, &radius))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    if (outCount != nullptr)
    {
        *outCount = 5u;
    }
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        results[axis].type        = ScriptValueType::Number;
        results[axis].numberValue = center[axis];
    }
    results[3].type        = ScriptValueType::Number;
    results[3].numberValue = radius;
    results[4].type         = ScriptValueType::Boolean;
    results[4].booleanValue = true;
    return true;
}

bool saveNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    std::string_view path;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !stringArgument(arguments, count, 1, path))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->saveRecipe(guard.bridge->character->self, handle, std::string(path).c_str(), &error));
}

bool loadNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    Guard            guard(static_cast<CharacterLuaBridgeContext *>(userData));
    CharacterHandle  handle = invalidCharacter;
    std::string_view path;
    if (guard.bridge == nullptr || guard.bridge->character == nullptr || count != 2
        || !handleArgument(arguments, count, 0, handle) || !stringArgument(arguments, count, 1, path))
    {
        return finishBoolean(results, capacity, outCount, false);
    }
    CharacterError error = CharacterError::None;
    return finishBoolean(results,
        capacity,
        outCount,
        guard.bridge->character->loadRecipe(guard.bridge->character->self, handle, std::string(path).c_str(), &error));
}

bool lastErrorNative(void *userData, const ScriptValue *arguments, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    (void)arguments;
    (void)count;
    Guard guard(static_cast<CharacterLuaBridgeContext *>(userData));
    if (guard.bridge == nullptr || guard.bridge->character == nullptr)
    {
        return finishNothing(results, capacity, outCount);
    }
    const char *message = guard.bridge->character->lastErrorMessage(guard.bridge->character->self);
    if (message == nullptr || message[0] == '\0')
    {
        return finishNothing(results, capacity, outCount);
    }
    return finishString(results, capacity, outCount, message);
}

struct NativeBinding
{
    const char          *name;
    ScriptNativeFunction function;
};

constexpr NativeBinding bindings[] = {
    {"character.create", &createNative},
    {"character.destroy", &destroyNative},
    {"character.add_part", &addPartNative},
    {"character.remove_part", &removePartNative},
    {"character.replace_part", &replacePartNative},
    {"character.connect", &connectNative},
    {"character.disconnect", &disconnectNative},
    {"character.set_parameter", &setParameterNative},
    {"character.get_parameter", &getParameterNative},
    {"character.set_connection_offset", &setConnectionOffsetNative},
    {"character.set_assembly_transform", &setAssemblyTransformNative},
    {"character.evaluate", &evaluateNative},
    {"character.validate", &validateNative},
    {"character.validation_issue", &validationIssueNative},
    {"character.statistics", &statisticsNative},
    {"character.parameter_count", &parameterCountNative},
    {"character.parameter_info", &parameterInfoNative},
    {"character.part_count", &partCountNative},
    {"character.part_id", &partIdNative},
    {"character.instance_count", &instanceCountNative},
    {"character.instance_id", &instanceIdNative},
    {"character.instance_info", &instanceInfoNative},
    {"character.instance_transform", &instanceTransformNative},
    {"character.bounds", &boundsNative},
    {"character.save", &saveNative},
    {"character.load", &loadNative},
    {"character.last_error", &lastErrorNative},
};

} // namespace

bool initializeCharacterLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept
{
    auto *context = static_cast<CharacterLuaBridgeContext *>(userData);
    if (context == nullptr || context->scriptRuntime == nullptr || context->character == nullptr
        || runtime == script_runtime::invalidScriptRuntime)
    {
        return false;
    }
    for (const NativeBinding &binding : bindings)
    {
        if (!context->scriptRuntime->registerNativeFunction(
                context->scriptRuntime->self, runtime, binding.name, binding.function, context))
        {
            return false;
        }
    }
    return true;
}

} // namespace engine::character
