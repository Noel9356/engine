// The module plugin seam of 'audio_miniaudio': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "audio.hpp"

namespace
{
const char *const audioProvidedCapabilities[] = {"audio.miniaudio"};
const char *const audioRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createaudioPluginModule()
{
    return new engine::AudioModule();
}

const engine::EngineModulePluginEntry audioPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "audio",
    .createModule            = &createaudioPluginModule,
    .providedCapabilities    = audioProvidedCapabilities,
    .providedCapabilityCount = std::size(audioProvidedCapabilities),
    .requiredCapabilities    = audioRequiredCapabilities,
    .requiredCapabilityCount = std::size(audioRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &audioPluginEntry;
}
