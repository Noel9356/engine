// Runtime capability provider index (kernel-internal).
//
// After participants construct, this index answers "who provides
// capability X" with exact and family (dotted) matches, enforcing the
// module/plugin visibility rule. Classification results cache once frozen.

#ifndef ENGINE_KERNEL_CAPABILITY_PROVIDER_INDEX_HPP
#define ENGINE_KERNEL_CAPABILITY_PROVIDER_INDEX_HPP

#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <cstddef>
#include <deque>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine
{

// One provider: participant identity, its provided capability names and
// the live instance pointers used to fetch capability interfaces.
struct CapabilityProviderEntry
{
    std::string name;

    ParticipantKind kind = ParticipantKind::EngineModule;

    std::vector<std::string> providedCapabilities;

    ModuleInterface  *moduleInstance  = nullptr;
    PluginInterface *pluginInstance = nullptr;
};

// Filled during the construct phase (addProvider per constructed
// participant), frozen afterwards; resolution throws std::logic_error on
// ambiguity or missing providers (try*/each variants return instead).
class CapabilityProviderIndex final
{
public:
    // Transparent-hash set shared with ParticipantContext::isCapabilityDemanded.
    using DemandedCapabilities = std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>>;

    // exactProviders: name matches. memberProviders: providers of a dotted
    // capability whose family satisfies the request (paired with the provided
    // name so the right interface is queried).
    struct ProviderSet
    {
        std::vector<CapabilityProviderEntry *> exactProviders;

        std::vector<std::pair<CapabilityProviderEntry *, std::string>> memberProviders;
    };

    // Back to empty/unfrozen for a new construct phase.
    void reset()
    {
        providers_.clear();
        demandedCapabilities_.clear();
        index_.clear();
        indexLookup_.clear();
        frozen_ = false;
    }

    void addProvider(CapabilityProviderEntry entry)
    {
        providers_.push_back(std::move(entry));
    }

    // After freeze, classified sets are cached; providers must not change.
    void freeze()
    {
        frozen_ = true;
    }

    // Demand closure is rebuilt every gather.
    void resetDemandedCapabilities()
    {
        demandedCapabilities_.clear();
    }

    // Registers a capability anybody demands (drives module activation);
    // returns false when already demanded.
    bool demandCapability(const std::string &capabilityName)
    {
        return demandedCapabilities_.insert(capabilityName).second;
    }

    const DemandedCapabilities &demandedCapabilities() const
    {
        return demandedCapabilities_;
    }

    void *resolveCapabilityPointer(ParticipantKind requesterKind, const std::string &capabilityName);

    bool hasSingleCapabilityProvider(const std::string &capabilityName, ParticipantKind requesterKind);

    void *tryResolveCapabilityPointer(ParticipantKind requesterKind, const std::string &capabilityName);

    std::vector<void *> resolveEachCapabilityPointer(ParticipantKind requesterKind, const std::string &capabilityName);

private:
    // Cache lookup when frozen; classifyProviders into the shared scratch
    // otherwise. The deque keeps cached ProviderSets stable while the lookup
    // map rehashes (references must not dangle).
    const ProviderSet &providerSetFor(const std::string &capabilityName);

    void classifyProviders(const std::string &capabilityName, ProviderSet &outSet);

    static bool isVisibleTo(const CapabilityProviderEntry &provider, ParticipantKind requesterKind)
    {
        return requesterKind != ParticipantKind::EngineModule || provider.kind == ParticipantKind::EngineModule;
    }

    static void *capabilityPointerOrNull(const CapabilityProviderEntry &provider, const std::string &capabilityName);

    static void *capabilityPointerOf(const CapabilityProviderEntry &provider, const std::string &capabilityName);

    // Owns the entries; ProviderSet holds non-owning pointers into it.
    std::vector<CapabilityProviderEntry> providers_;

    DemandedCapabilities demandedCapabilities_;

    // Reusable output of classifyProviders (used before freezing).
    ProviderSet scratch_;

    struct IndexEntry
    {
        std::string capabilityName;
        ProviderSet providers;
    };

    // Frozen-phase cache: capability name -> classified ProviderSet.
    std::deque<IndexEntry>                       index_;
    std::unordered_map<std::string, std::size_t> indexLookup_;

    bool frozen_ = false;
};

} // namespace engine

#endif
