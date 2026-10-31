#ifndef ENGINE_KERNEL_MOD_API_HPP
#define ENGINE_KERNEL_MOD_API_HPP

#include "./component_interface.hpp"

#include <cstdint>

namespace engine
{

inline constexpr std::uint32_t engineModInterfaceVersion = 1;

inline constexpr const char *engineModEntryTableSymbol = "engine_query_mod_entry_table";

} // namespace engine

extern "C"
{
    struct EngineModEntryTable
    {
        std::uint32_t interfaceVersion;
        engine::ModInterface *(*createMod)();
        void (*destroyMod)(engine::ModInterface *mod);
    };
}

#endif
