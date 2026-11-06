// Participant identity and requirements shared by modules, mods and the
// application.

#ifndef ENGINE_KERNEL_PARTICIPANT_MANIFEST_HPP
#define ENGINE_KERNEL_PARTICIPANT_MANIFEST_HPP

#include <string>
#include <vector>

namespace engine
{

// EngineModule: kernel-loaded .so module; NativeGameMod/ScriptGameMod:
// discovered mod packages (type = "native"/"script"); Application: the
// synthetic "<app>" participant.
enum class ParticipantKind
{
    EngineModule,
    NativeGameMod,
    ScriptGameMod,
    Application
};

// The graph-facing contract. optionalCapabilities are soft requirements;
// requiredParticipants names participants directly; knownConfigKeys
// registers the sections the participant reads (unknown-key warnings).
struct ParticipantManifest
{
    std::string     name = {};
    ParticipantKind kind = ParticipantKind::EngineModule;

    std::string version = "0.1.0";

    std::vector<std::string> providedCapabilities = {};
    std::vector<std::string> requiredCapabilities = {};
    std::vector<std::string> optionalCapabilities = {};
    std::vector<std::string> requiredParticipants = {};
    std::vector<std::string> requiredPermissions  = {};
    std::vector<std::string> knownConfigKeys      = {};
};

} // namespace engine

#endif
