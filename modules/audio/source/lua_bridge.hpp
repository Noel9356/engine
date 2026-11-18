#pragma once

#include <engine/audio/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <filesystem>
#include <vector>

namespace engine::audio
{

struct AudioLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    AudioCapability *audio = nullptr;
    // Canonicalized [audio] lua_allowed_path_roots entries; sound paths
    // supplied by scripts must resolve inside one of them, and an empty list
    // refuses every script sound load. Owned by the module (filled once in
    // construct(), read-only afterwards); the context only borrows it.
    const std::vector<std::filesystem::path> *scriptSoundRoots = nullptr;
};

bool initializeAudioLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::audio

