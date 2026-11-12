// Kernel-internal formatting helpers: participant kind nouns and the
// "name (version, sha256)" suffix used in trust advisories.

#ifndef ENGINE_KERNEL_KERNEL_INTERNALS_HPP
#define ENGINE_KERNEL_KERNEL_INTERNALS_HPP

#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <string>

namespace engine
{

// Lowercase noun for reports ("module", "native plugin", ...).
inline std::string describeComponentKind(ParticipantKind kind)
{
    switch (kind)
    {
    case ParticipantKind::EngineModule:
        return "module";
    case ParticipantKind::NativePlugin:
        return "native plugin";
    case ParticipantKind::ScriptPlugin:
        return "script plugin";
    case ParticipantKind::Application:
        return "application";
    }
    return "participant";
}

// " (version X, sha256 Y)" suffix, omitting absent parts.
inline std::string advisoryIdentitySuffix(const ParticipantTrustAdvisory &advisory)
{
    std::string suffix;
    if (!advisory.version.empty())
    {
        suffix += " (version " + advisory.version;
        if (!advisory.sha256.empty())
        {
            suffix += ", sha256 " + advisory.sha256;
        }
        suffix += ")";
    }
    else if (!advisory.sha256.empty())
    {
        suffix += " (sha256 " + advisory.sha256 + ")";
    }
    return suffix;
}

} // namespace engine

#endif
