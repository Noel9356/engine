// C ABI boundary for native game mods.
//
// A native mod .so exports engine_query_game_mod_entry_table; the kernel
// uses createMod/destroyMod so instance lifetime stays inside the mod
// library.

#ifndef ENGINE_KERNEL_GAME_MOD_API_HPP
#define ENGINE_KERNEL_GAME_MOD_API_HPP

#include <engine/kernel/participant_interface.hpp>

#include <cstdint>

namespace engine
{

// Bump on breaking GameModInterface changes; mod.toml engine_api must
// match.
inline constexpr std::uint32_t engineGameModInterfaceVersion = 1;

// Symbol name a native mod .so exports (extern "C").
inline constexpr const char *engineGameModEntryTableSymbol = "engine_query_game_mod_entry_table";

} // namespace engine

extern "C"
{
    // createMod/destroyMod pair; destroy instances createMod returned via
    // destroyMod (the creator module owns the deletion policy).
    struct EngineGameModEntryTable
    {
        // Must equal engineGameModInterfaceVersion; checked before createMod.
        std::uint32_t interfaceVersion;
        engine::GameModInterface *(*createMod)();
        void (*destroyMod)(engine::GameModInterface *mod);
    };
}

#endif
