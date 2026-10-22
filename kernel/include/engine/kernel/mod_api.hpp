#ifndef ENGINE_KERNEL_MOD_API_HPP
#define ENGINE_KERNEL_MOD_API_HPP

#include "component_interface.hpp"

#include <cstdint>

namespace engine
{

// Version of the mod entry contract below. Increase when the shape of the
// entry table or the lifecycle interfaces changes in a breaking way.
inline constexpr std::uint32_t engineModInterfaceVersion = 1;

// The one symbol every shared library mod exports (see the C ABI below).
// Named once here because the kernel resolves it by string and every mod
// defines it; a spelling drift on either side would fail startup with a
// confusing "symbol not found" instead of pointing at the contract.
inline constexpr const char* engineModEntryTableSymbol = "engine_query_mod_entry_table";

} // namespace engine

// The C ABI boundary between the engine and shared library mods. A C struct of
// function pointers keeps the loader decoupled from any C++ ABI beyond the
// virtual tables of the interfaces the mod implements; mods are still expected
// to be built with the same compiler family and standard library as the engine.
//
// Every shared library mod exports exactly one symbol:
//
//   extern "C" const engine::EngineModEntryTable* engine_query_mod_entry_table();
//
// The returned table must be static and valid for the lifetime of the library.
extern "C"
{
    struct EngineModEntryTable
    {
        std::uint32_t interfaceVersion;
        engine::ModInterface* (*createMod)();
        void (*destroyMod)(engine::ModInterface* mod);
    };
}

#endif // ENGINE_KERNEL_MOD_API_HPP
