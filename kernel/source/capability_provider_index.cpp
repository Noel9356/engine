// CapabilityProviderIndex implementation: classification, caching and
// resolution rules for capability pointers.

#include "capability_provider_index.hpp"
#include <engine/kernel/capability_name.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// indexLookup_ maps capability -> index_ slot; both grow together.
const CapabilityProviderIndex::ProviderSet &CapabilityProviderIndex::providerSetFor(const std::string &capabilityName)
{
    if (!frozen_)
    {
        classifyProviders(capabilityName, scratch_);
        return scratch_;
    }

    const auto cached = indexLookup_.find(capabilityName);
    if (cached != indexLookup_.end())
    {
        return index_[cached->second].providers;
    }

    classifyProviders(capabilityName, scratch_);
    const std::size_t index = index_.size();
    index_.push_back(IndexEntry{.capabilityName = capabilityName, .providers = scratch_});
    indexLookup_.emplace(index_.back().capabilityName, index);
    return index_.back().providers;
}

// One pass over all providers; a provider contributes at most one member
// match (break) but may appear several times as an exact provider.
void CapabilityProviderIndex::classifyProviders(const std::string &capabilityName, ProviderSet &outSet)
{
    outSet.exactProviders.clear();
    outSet.memberProviders.clear();
    for (CapabilityProviderEntry &provider : providers_)
    {
        for (const std::string &providedName : provider.providedCapabilities)
        {
            if (providedName == capabilityName)
            {
                outSet.exactProviders.push_back(&provider);
            }
            else if (capabilityRequirementSatisfiedBy(capabilityName, providedName))
            {
                outSet.memberProviders.emplace_back(&provider, providedName);
                break;
            }
        }
    }
}

// Asks the instance; nullptr means it declared the capability but does
// not implement it right now.
void *CapabilityProviderIndex::capabilityPointerOrNull(
    const CapabilityProviderEntry &provider, const std::string &capabilityName)
{
    if (provider.moduleInstance != nullptr)
    {
        return provider.moduleInstance->capabilityPointerFor(capabilityName);
    }
    if (provider.gameModInstance != nullptr)
    {
        return provider.gameModInstance->capabilityPointerFor(capabilityName);
    }
    return nullptr;
}

// Strict resolve: a declared-but-missing implementation is a kernel bug.
void *CapabilityProviderIndex::capabilityPointerOf(
    const CapabilityProviderEntry &provider, const std::string &capabilityName)
{
    void *capabilityPointer = capabilityPointerOrNull(provider, capabilityName);
    if (capabilityPointer == nullptr)
    {
        throw std::logic_error("participant '" + provider.name + "' declares capability '" + capabilityName
                               + "' but returned no implementation");
    }
    return capabilityPointer;
}

// Precedence: exact matches first, then family members; ambiguity among
// visible providers throws ([providers] selections were already applied
// at the graph level). Modules requesting mod-only capabilities get a
// dedicated message.
void *CapabilityProviderIndex::resolveCapabilityPointer(
    ParticipantKind requesterKind, const std::string &capabilityName)
{
    const ProviderSet &providers       = providerSetFor(capabilityName);
    const bool         moduleRequester = requesterKind == ParticipantKind::EngineModule;

    // A single provider may still be invisible (mod capability vs module
    // requester) and fall through to the error paths below.
    if (providers.exactProviders.size() == 1) [[likely]]
    {
        CapabilityProviderEntry *provider = providers.exactProviders.front();
        if (isVisibleTo(*provider, requesterKind)) [[likely]]
        {
            return capabilityPointerOf(*provider, capabilityName);
        }
    }
    else if (providers.exactProviders.size() > 1)
    {
        std::vector<CapabilityProviderEntry *> visibleProviders;
        visibleProviders.reserve(providers.exactProviders.size());
        for (CapabilityProviderEntry *provider : providers.exactProviders)
        {
            if (isVisibleTo(*provider, requesterKind))
            {
                visibleProviders.push_back(provider);
            }
        }
        if (visibleProviders.size() > 1)
        {
            std::string names;
            for (const CapabilityProviderEntry *provider : visibleProviders)
            {
                if (!names.empty())
                {
                    names += ", ";
                }
                names += "'" + provider->name + "'";
            }
            throw std::logic_error("capability '" + capabilityName + "' is provided by several participants (" + names
                                   + "); choose one in the configuration under [providers]");
        }
        if (visibleProviders.size() == 1) [[unlikely]]
        {
            return capabilityPointerOf(*visibleProviders.front(), capabilityName);
        }
    }

    // Family fallback: exactly one member implementation resolves to its
    // concrete (dotted) capability.
    if (providers.memberProviders.size() == 1) [[likely]]
    {
        const auto &member = providers.memberProviders.front();
        if (isVisibleTo(*member.first, requesterKind))
        {
            return capabilityPointerOf(*member.first, member.second);
        }
    }
    else if (providers.memberProviders.size() > 1)
    {
        std::vector<std::pair<CapabilityProviderEntry *, std::string>> visibleMembers;
        visibleMembers.reserve(providers.memberProviders.size());
        for (const auto &member : providers.memberProviders)
        {
            if (isVisibleTo(*member.first, requesterKind))
            {
                visibleMembers.push_back(member);
            }
        }
        if (visibleMembers.size() > 1)
        {
            std::string names;
            for (const auto &member : visibleMembers)
            {
                if (!names.empty())
                {
                    names += ", ";
                }
                names += "'" + member.first->name + "'";
            }
            throw std::logic_error(
                "capability family '" + capabilityName + "' is ambiguous: " + std::to_string(visibleMembers.size())
                + " implementations are active (" + names + "); choose one in the configuration under [providers]");
        }
        if (visibleMembers.size() == 1) [[unlikely]]
        {
            return capabilityPointerOf(*visibleMembers.front().first, visibleMembers.front().second);
        }
    }

    // Modules never see mod capabilities; name the mods so the manifest can
    // be fixed.
    if (moduleRequester)
    {
        std::string providerNames;
        for (const CapabilityProviderEntry *provider : providers.exactProviders)
        {
            if (!providerNames.empty())
            {
                providerNames += ", ";
            }
            providerNames += "'" + provider->name + "'";
        }
        for (const auto &member : providers.memberProviders)
        {
            if (!providerNames.empty())
            {
                providerNames += ", ";
            }
            providerNames += "'" + member.first->name + "'";
        }
        if (!providerNames.empty())
        {
            throw std::logic_error("capability '" + capabilityName + "' is provided only by mods (" + providerNames
                                   + "); modules can only use capabilities provided by other modules");
        }
    }

    // Reached when nothing visible matched: an acquire without a requirement
    // whose provider is available in this run.
    throw std::logic_error("capability '" + capabilityName
                           + "' has no active provider; a participant acquired it without a "
                             "requirement whose provider is available in this run");
}

// Visibility-aware unambiguity check (used by isCapabilityAvailable).
bool CapabilityProviderIndex::hasSingleCapabilityProvider(
    const std::string &capabilityName, ParticipantKind requesterKind)
{
    const ProviderSet &providers       = providerSetFor(capabilityName);
    const bool         moduleRequester = requesterKind == ParticipantKind::EngineModule;
    if (!providers.exactProviders.empty())
    {
        if (!moduleRequester)
        {
            return providers.exactProviders.size() == 1;
        }
        std::size_t visibleCount = 0;
        for (const CapabilityProviderEntry *provider : providers.exactProviders)
        {
            if (isVisibleTo(*provider, requesterKind))
            {
                ++visibleCount;
            }
        }
        return visibleCount == 1;
    }
    if (!moduleRequester)
    {
        return providers.memberProviders.size() == 1;
    }
    std::size_t visibleMembers = 0;
    for (const auto &member : providers.memberProviders)
    {
        if (isVisibleTo(*member.first, requesterKind))
        {
            ++visibleMembers;
        }
    }
    return visibleMembers == 1;
}

// Never throws: resolves only when exactly one visible candidate exists
// (exact first; members only when there are no exact providers at all).
void *CapabilityProviderIndex::tryResolveCapabilityPointer(
    ParticipantKind requesterKind, const std::string &capabilityName)
{
    const ProviderSet &providers = providerSetFor(capabilityName);

    std::size_t              visibleExact = 0;
    CapabilityProviderEntry *exactOne     = nullptr;
    for (CapabilityProviderEntry *provider : providers.exactProviders)
    {
        if (!isVisibleTo(*provider, requesterKind))
        {
            continue;
        }
        ++visibleExact;
        exactOne = provider;
    }
    if (visibleExact == 1)
    {
        return capabilityPointerOf(*exactOne, capabilityName);
    }
    if (providers.exactProviders.empty() || visibleExact == 0)
    {
        std::size_t                                       visibleMembers = 0;
        std::pair<CapabilityProviderEntry *, std::string> memberOne;
        for (const auto &member : providers.memberProviders)
        {
            if (!isVisibleTo(*member.first, requesterKind))
            {
                continue;
            }
            ++visibleMembers;
            memberOne = member;
        }
        if (visibleMembers == 1)
        {
            return capabilityPointerOf(*memberOne.first, memberOne.second);
        }
    }
    return nullptr;
}

// Extension model: every visible provider's pointer (skipping ones that
// return null); exact providers take precedence over members.
std::vector<void *> CapabilityProviderIndex::resolveEachCapabilityPointer(
    ParticipantKind requesterKind, const std::string &capabilityName)
{
    const ProviderSet &providers = providerSetFor(capabilityName);

    std::vector<void *> pointers;
    if (!providers.exactProviders.empty())
    {
        pointers.reserve(providers.exactProviders.size());
        for (CapabilityProviderEntry *provider : providers.exactProviders)
        {
            if (!isVisibleTo(*provider, requesterKind))
            {
                continue;
            }
            if (void *capabilityPointer = capabilityPointerOrNull(*provider, capabilityName);
                capabilityPointer != nullptr)
            {
                pointers.push_back(capabilityPointer);
            }
        }
        return pointers;
    }
    if (!providers.memberProviders.empty())
    {
        pointers.reserve(providers.memberProviders.size());
        for (const auto &member : providers.memberProviders)
        {
            if (!isVisibleTo(*member.first, requesterKind))
            {
                continue;
            }
            if (void *capabilityPointer = capabilityPointerOrNull(*member.first, member.second);
                capabilityPointer != nullptr)
            {
                pointers.push_back(capabilityPointer);
            }
        }
    }
    return pointers;
}

} // namespace engine
