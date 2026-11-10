// Shared value conversion helpers for script_runtime native-function bridges.
//
// These helpers validate the primitive values crossing the script/native
// boundary and initialize result slots consistently. They do not allocate and
// keep ScriptValue strings borrowed for the duration of the native call.

#ifndef ENGINE_SCRIPT_RUNTIME_NATIVE_VALUES_HPP
#define ENGINE_SCRIPT_RUNTIME_NATIVE_VALUES_HPP

#include <engine/script_runtime/capabilities.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>

namespace engine::script_runtime::native
{

inline bool number(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index, double &output,
    bool required = true) noexcept
{
    if (index >= argumentCount || arguments[index].type == ScriptValueType::Nil)
        return !required;
    if (arguments[index].type != ScriptValueType::Number || !std::isfinite(arguments[index].numberValue))
        return false;
    output = arguments[index].numberValue;
    return true;
}

inline bool uint32Value(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    std::uint32_t &output, std::uint32_t fallback = 0, bool required = true) noexcept
{
    double value = static_cast<double>(fallback);
    if (!number(arguments, argumentCount, index, value, required))
        return false;
    if (index >= argumentCount || arguments[index].type == ScriptValueType::Nil)
    {
        output = fallback;
        return true;
    }
    if (value < 0.0 || value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())
        || std::floor(value) != value)
        return false;
    output = static_cast<std::uint32_t>(value);
    return true;
}

inline bool int32Value(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    std::int32_t &output) noexcept
{
    double value = 0.0;
    if (!number(arguments, argumentCount, index, value)
        || value < static_cast<double>(std::numeric_limits<std::int32_t>::min())
        || value > static_cast<double>(std::numeric_limits<std::int32_t>::max()) || std::floor(value) != value)
        return false;
    output = static_cast<std::int32_t>(value);
    return true;
}

inline bool floatValue(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    float &output, float fallback = 0.0f, bool required = true) noexcept
{
    double value = static_cast<double>(fallback);
    if (!number(arguments, argumentCount, index, value, required))
        return false;
    if (index >= argumentCount || arguments[index].type == ScriptValueType::Nil)
    {
        output = fallback;
        return true;
    }
    if (value < -static_cast<double>(std::numeric_limits<float>::max())
        || value > static_cast<double>(std::numeric_limits<float>::max()))
        return false;
    output = static_cast<float>(value);
    return true;
}

inline bool boolean(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    bool &output) noexcept
{
    if (index >= argumentCount || arguments[index].type != ScriptValueType::Boolean)
        return false;
    output = arguments[index].booleanValue;
    return true;
}

inline bool optionalBoolean(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    bool &output, bool fallback = false, bool required = false) noexcept
{
    if (index >= argumentCount || arguments[index].type == ScriptValueType::Nil)
    {
        if (required)
            return false;
        output = fallback;
        return true;
    }
    return boolean(arguments, argumentCount, index, output);
}

inline bool stringValue(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    std::string_view &output) noexcept
{
    if (index >= argumentCount || arguments[index].type != ScriptValueType::String
        || arguments[index].stringData == nullptr)
        return false;
    output = std::string_view(arguments[index].stringData, arguments[index].stringSize);
    return true;
}

inline bool stringValue(const ScriptValue *arguments, std::uint32_t argumentCount, std::uint32_t index,
    const char *&output, bool required = true) noexcept
{
    if (index >= argumentCount || arguments[index].type == ScriptValueType::Nil)
        return !required;
    if (arguments[index].type != ScriptValueType::String || arguments[index].stringData == nullptr)
        return false;
    output = arguments[index].stringData;
    return true;
}

inline void resultNumber(ScriptValue *results, std::uint32_t index, double value) noexcept
{
    results[index] = {};
    results[index].type = ScriptValueType::Number;
    results[index].numberValue = value;
}

inline void resultBoolean(ScriptValue *results, std::uint32_t index, bool value) noexcept
{
    results[index] = {};
    results[index].type = ScriptValueType::Boolean;
    results[index].booleanValue = value;
}

inline void resultNil(ScriptValue *results, std::uint32_t index) noexcept
{
    results[index] = {};
}

inline void resultString(ScriptValue *results, std::uint32_t index, const char *value) noexcept
{
    const char *text = value != nullptr ? value : "";
    const std::size_t length = std::strlen(text);
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
        resultNil(results, index);
        return;
    }
    results[index] = {};
    results[index].type = ScriptValueType::String;
    results[index].stringData = text;
    results[index].stringSize = static_cast<std::uint32_t>(length);
}

} // namespace engine::script_runtime::native

#endif // ENGINE_SCRIPT_RUNTIME_NATIVE_VALUES_HPP
