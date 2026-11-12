// C ABI boundary for native plugins.
//
// A native plugin .so exports engine_query_plugin_entry_table; the kernel
// uses createPlugin/destroyPlugin so instance lifetime stays inside the plugin
// library.

#ifndef ENGINE_KERNEL_PLUGIN_API_HPP
#define ENGINE_KERNEL_PLUGIN_API_HPP

#include <engine/kernel/participant_interface.hpp>

#include <cstdint>

namespace engine
{

// Bump on breaking PluginInterface changes; plugin.toml engine_api must
// match.
inline constexpr std::uint32_t enginePluginInterfaceVersion = 2;

// Symbol name a native plugin .so exports (extern "C").
inline constexpr const char *enginePluginEntryTableSymbol = "engine_query_plugin_entry_table";

} // namespace engine

extern "C"
{
    // createPlugin/destroyPlugin pair; destroy instances createPlugin returned via
    // destroyPlugin (the creator module owns the deletion policy).
    struct EnginePluginEntryTable
    {
        // Must equal enginePluginInterfaceVersion; checked before createPlugin.
        std::uint32_t interfaceVersion;
        engine::PluginInterface *(*createPlugin)();
        void (*destroyPlugin)(engine::PluginInterface *plugin);
    };
}

#endif
