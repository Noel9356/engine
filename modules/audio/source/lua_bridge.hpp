#ifndef ENGINE_AUDIO_LUA_BRIDGE_HPP
#define ENGINE_AUDIO_LUA_BRIDGE_HPP

#include <engine/audio/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

namespace engine::audio
{

struct AudioLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    AudioCapability *audio = nullptr;
};

bool initializeAudioLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::audio

#endif
