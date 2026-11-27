// Flag-proof finiteness tests for floating-point data crossing the module's
// public boundaries (mesh streams, model matrices, configuration floats).
//
// Why not std::isfinite: the headless backend's translation unit is compiled
// with -ffast-math, whose -ffinite-math-only component lets GCC fold
// std::isfinite(x) to constant true - every guard written against it becomes
// dead code in that unit (the exact failure that let non-finite geometry reach
// the shadow rasterizer's integer scan indices). Testing the exponent bits
// through a memcpy re-interpretation is opaque to that assumption: the
// compiler cannot fold it under any combination of math flags, so a guard
// written with these helpers keeps guarding wherever the header is included.
//
// Pure policy header: no backend types, no engine dependencies - safe to
// include from the self-contained rasterizer policy headers (sun_shadows.hpp)
// and from the module core (renderer.cpp) alike.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace engine
{
namespace renderer
{

// True when value is neither NaN nor infinite (IEEE-754 binary32: every
// exponent bit pattern except the all-ones reserved range).
inline bool isFinite(float value) noexcept
{
    static_assert(sizeof(float) == sizeof(std::uint32_t), "IEEE-754 binary32 float expected");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// True when the first floatCount floats at values are all finite. Reads
// through memcpy, so the buffer's alignment does not matter (vertex streams
// arrive as void* from the public ABI and may be packed arbitrarily).
inline bool isFiniteRange(const void *values, std::size_t floatCount) noexcept
{
    if (values == nullptr)
    {
        return false;
    }
    const auto *bytes = static_cast<const std::uint8_t *>(values);
    for (std::size_t index = 0; index < floatCount; ++index)
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, bytes + index * sizeof(float), sizeof(bits));
        if ((bits & 0x7F800000u) == 0x7F800000u)
        {
            return false;
        }
    }
    return true;
}

} // namespace renderer
} // namespace engine
