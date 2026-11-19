// Shared helper for the engine's capability error conventions.
//
// Every capability surfaces a module-local error enum written through an
// optional out-parameter: a null pointer means "caller does not care" and the
// call stays otherwise unaffected. This tiny helper keeps that convention in
// one place instead of duplicating the same inline test in every module.

#pragma once

namespace engine::util
{

// Writes `value` through `output` when the caller passed one.
template <typename ErrorEnum>
inline void setModuleError(ErrorEnum *output, ErrorEnum value) noexcept
{
    if (output != nullptr)
    {
        *output = value;
    }
}

} // namespace engine::util
