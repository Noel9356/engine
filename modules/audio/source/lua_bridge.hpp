#pragma once

#include <engine/audio/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <filesystem>
#include <mutex>
#include <vector>

namespace engine::audio
{

struct AudioLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    AudioCapability *audio = nullptr;
    // Guards `audio` for the bridge entries: the module's stop() nulls the
    // pointer under this mutex before closing the registry, and every bridge
    // call takes the mutex for its duration (see AudioApiScope in
    // lua_bridge.cpp).
    std::mutex apiMutex;
    // Canonicalized [audio] lua_allowed_path_roots entries; sound paths
    // supplied by scripts must resolve inside one of them, and an empty list
    // refuses every script sound load. Owned by the module (filled once in
    // construct(), read-only afterwards); the context only borrows it.
    const std::vector<std::filesystem::path> *scriptSoundRoots = nullptr;
};

bool initializeAudioLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::audio

