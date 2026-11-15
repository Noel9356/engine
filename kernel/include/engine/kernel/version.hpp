// Engine version constants.
//
// Single source of the engine's version identity, reported by the kernel.

#pragma once

#include <cstdint>

namespace engine
{

inline constexpr std::uint32_t engineVersionMajor  = 0u;
inline constexpr std::uint32_t engineVersionMinor  = 1u;
inline constexpr std::uint32_t engineVersionPatch  = 0u;
inline constexpr const char   *engineVersionString = "0.1.0";

} // namespace engine

