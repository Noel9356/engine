// TrustGate: risk advisories and trust decisions (kernel-internal).
//
// Collects per-participant advisories during gathering, then evaluate()
// (startup) and decideScanConsent (module scanning) enforce the [trust]
// configuration with the consent handler as fallback.

#ifndef ENGINE_KERNEL_TRUST_GATE_HPP
#define ENGINE_KERNEL_TRUST_GATE_HPP

#include <engine/kernel/configuration.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/module_descriptor.hpp>
#include <engine/kernel/sha256.hpp>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

class TrustGate final
{
public:
    // At or above this many acquired capabilities, a participant earns a "broad" advisory.
    static constexpr std::size_t broadCapabilityWarningThreshold = 5;

    void setConsentHandler(TrustConsentHandler handler)
    {
        consentHandler_ = std::move(handler);
    }

    void clearAdvisories()
    {
        advisories_.clear();
    }

    const std::vector<ParticipantTrustAdvisory> &advisories() const
    {
        return advisories_;
    }

    void appendPluginRiskWarning(const PluginManifest &pluginManifest, bool isNativePlugin, std::string &warnings);

    void appendModuleRiskWarning(const ModuleLibraryRecord &module, std::string &warnings);

    bool evaluate(
        std::string &report, bool enforce, std::string &warnings, const ConfigurationStore &configuration) const;

    // Indexes into the caller's advisory vector.
    struct ScanDecision
    {
        std::vector<std::size_t> approved;
        std::vector<std::size_t> undecided;
    };

    ScanDecision decideScanConsent(const std::vector<ParticipantTrustAdvisory> &scanAdvisories,
        const ConfigurationStore &configuration, std::string &warnings) const;

private:
    // Accumulated during gathering; cleared per gather.
    std::vector<ParticipantTrustAdvisory> advisories_;

    TrustConsentHandler consentHandler_;
};

} // namespace engine

#endif
