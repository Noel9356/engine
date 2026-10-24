#include "engine/kernel/dependency_resolver.hpp"

#include "engine/kernel/capability_name.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <set>

namespace engine
{

namespace
{

using ManifestIndex = std::size_t;

// Depth first search state for cycle extraction. Cycle detection only ever
// distinguishes "not entered yet" from "on the current search path"; a fully
// processed node is never re-entered, so no third state exists.
enum class VisitState
{
    Unvisited,
    InProgress
};

} // namespace

DependencyResolver::DependencyResolver(
    std::vector<ComponentManifest> manifests, std::vector<ProviderSelection> selections
)
    : manifests_(std::move(manifests)), selections_(std::move(selections))
{
}

DependencyResolver::ResolutionResult DependencyResolver::resolve() const
{
    ResolutionResult result;

    const ManifestIndex manifestCount = manifests_.size();

    // Component name lookup.
    std::map<std::string, ManifestIndex> componentIndices;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        componentIndices.emplace(manifests_[index].name, index);
    }

    // Capability name to provider indices, in manifest order.
    std::map<std::string, std::vector<ManifestIndex>> capabilityProviders;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        for (const std::string &capabilityName : manifests_[index].providedCapabilities)
        {
            capabilityProviders[capabilityName].push_back(index);
        }
    }

    // Provider candidates for one requirement: exact declarations first, then
    // - when the requirement names a family - every member of that family.
    // A family requirement several exact providers satisfy is ambiguity among
    // the exact providers; members only come into play when no exact provider
    // exists.
    const auto candidateProviders = [&capabilityProviders](const std::string &requirement)
    {
        std::vector<ManifestIndex> candidates;
        const auto                 exact = capabilityProviders.find(requirement);
        if (exact != capabilityProviders.end())
        {
            candidates = exact->second;
            return candidates;
        }
        if (isImplementationBoundCapabilityName(requirement))
        {
            return candidates; // an implementation-bound name only matches exactly
        }
        for (const auto &entry : capabilityProviders)
        {
            if (capabilityRequirementSatisfiedBy(requirement, entry.first))
            {
                candidates.insert(candidates.end(), entry.second.begin(), entry.second.end());
            }
        }
        return candidates;
    };

    // Apply provider selections and report ambiguity or bad selections.
    std::map<std::string, ManifestIndex> selectedProvider; // requirement -> provider index
    {
        std::set<std::string> requirementNames;
        for (const ComponentManifest &manifest : manifests_)
        {
            requirementNames.insert(manifest.requiredCapabilities.begin(), manifest.requiredCapabilities.end());
        }
        for (const std::string &requirement : requirementNames)
        {
            const std::vector<ManifestIndex> providers = candidateProviders(requirement);

            const auto selection = std::find_if(
                selections_.begin(),
                selections_.end(),
                [&requirement](const ProviderSelection &candidate) { return candidate.capabilityName == requirement; }
            );

            if (providers.size() > 1)
            {
                if (selection == selections_.end())
                {
                    std::string providerList;
                    for (ManifestIndex providerIndex : providers)
                    {
                        if (!providerList.empty())
                        {
                            providerList += ", ";
                        }
                        providerList += manifests_[providerIndex].name;
                    }
                    ResolutionDiagnostic diagnostic;
                    diagnostic.kind    = ResolutionDiagnostic::Kind::AmbiguousCapability;
                    diagnostic.message = "capability '" + requirement + "' is provided by several components ["
                                       + providerList + "]; choose one in the configuration under [providers].";
                    result.diagnostics.push_back(std::move(diagnostic));
                }
                else
                {
                    const auto chosen = std::find_if(
                        providers.begin(),
                        providers.end(),
                        [this, &selection](ManifestIndex providerIndex)
                        { return manifests_[providerIndex].name == selection->providerName; }
                    );
                    if (chosen == providers.end())
                    {
                        ResolutionDiagnostic diagnostic;
                        diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider;
                        diagnostic.message = "configuration selects '" + selection->providerName + "' for capability '"
                                           + requirement + "' but that component does not provide the capability.";
                        diagnostic.involvementChain.push_back(selection->providerName);
                        diagnostic.involvementChain.push_back(requirement);
                        result.diagnostics.push_back(std::move(diagnostic));
                    }
                    else
                    {
                        selectedProvider.emplace(requirement, *chosen);
                    }
                }
            }
            else if (providers.size() == 1)
            {
                if (selection != selections_.end() && manifests_[providers.front()].name != selection->providerName)
                {
                    ResolutionDiagnostic diagnostic;
                    diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider;
                    diagnostic.message = "configuration selects '" + selection->providerName + "' for capability '"
                                       + requirement + "' but the only provider is '"
                                       + manifests_[providers.front()].name + "'.";
                    result.diagnostics.push_back(std::move(diagnostic));
                }
                else
                {
                    selectedProvider.emplace(requirement, providers.front());
                }
            }
        }
    }

    // Selections that name a capability nobody provides - neither exactly nor
    // through a family member.
    for (const ProviderSelection &selection : selections_)
    {
        if (candidateProviders(selection.capabilityName).empty())
        {
            ResolutionDiagnostic diagnostic;
            diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider;
            diagnostic.message = "configuration selects '" + selection.providerName + "' for capability '"
                               + selection.capabilityName + "' but no component provides that capability.";
            result.diagnostics.push_back(std::move(diagnostic));
        }
    }

    // A broken selection phase already carries the root cause (an ambiguity
    // the configuration must settle, or a selection nobody can satisfy). The
    // edge phase below would only repeat that failure per consumer - one
    // precise diagnostic per broken startup means the resolver reports the
    // root cause and stops (docs/architecture.md, the resolution contract).
    if (!result.diagnostics.empty())
    {
        result.successful = false;
        return result;
    }

    // Build consumer -> provider edges. Each edge is (providerIndex, consumerIndex).
    std::vector<std::pair<ManifestIndex, ManifestIndex>> edges;
    for (ManifestIndex consumerIndex = 0; consumerIndex < manifestCount; ++consumerIndex)
    {
        const ComponentManifest &consumer = manifests_[consumerIndex];

        for (const std::string &capabilityName : consumer.requiredCapabilities)
        {
            const auto provider = selectedProvider.find(capabilityName);
            if (provider == selectedProvider.end())
            {
                std::string availableList;
                for (const auto &entry : capabilityProviders)
                {
                    if (!availableList.empty())
                    {
                        availableList += ", ";
                    }
                    availableList += entry.first;
                }
                ResolutionDiagnostic diagnostic;
                diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownCapability;
                diagnostic.message = "component '" + consumer.name + "' requires capability '" + capabilityName
                                   + "' but no available component provides it."
                                   + (availableList.empty() ? std::string(" no capabilities are provided at all.")
                                                            : " available capabilities: " + availableList + ".");
                diagnostic.involvementChain.push_back(consumer.name);
                diagnostic.involvementChain.push_back(capabilityName);
                result.diagnostics.push_back(std::move(diagnostic));
                continue;
            }
            if (provider->second != consumerIndex)
            {
                edges.emplace_back(provider->second, consumerIndex);
            }
        }

        for (const std::string &componentName : consumer.requiredComponents)
        {
            const auto required = componentIndices.find(componentName);
            if (required == componentIndices.end())
            {
                ResolutionDiagnostic diagnostic;
                diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownComponent;
                diagnostic.message = "component '" + consumer.name + "' requires component '" + componentName
                                   + "' but no component with that name is available.";
                diagnostic.involvementChain.push_back(consumer.name);
                diagnostic.involvementChain.push_back(componentName);
                result.diagnostics.push_back(std::move(diagnostic));
                continue;
            }
            if (required->second != consumerIndex)
            {
                edges.emplace_back(required->second, consumerIndex);
            }
        }
    }

    // Deterministic Kahn topological ordering. Duplicate edges collapse through
    // the edge set so shared dependencies are counted once.
    std::set<std::pair<ManifestIndex, ManifestIndex>> uniqueEdges(edges.begin(), edges.end());
    std::vector<ManifestIndex>                        inDegree(manifestCount, 0);
    std::vector<std::vector<ManifestIndex>>           dependents(manifestCount);
    for (const auto &edge : uniqueEdges)
    {
        dependents[edge.first].push_back(edge.second);
        ++inDegree[edge.second];
    }

    std::vector<ManifestIndex> readyQueue;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        if (inDegree[index] == 0)
        {
            readyQueue.push_back(index);
        }
    }

    std::vector<ManifestIndex> order;
    order.reserve(manifestCount);
    while (!readyQueue.empty())
    {
        const ManifestIndex current = readyQueue.front();
        readyQueue.erase(readyQueue.begin());
        order.push_back(current);
        for (ManifestIndex dependent : dependents[current])
        {
            --inDegree[dependent];
            if (inDegree[dependent] == 0)
            {
                readyQueue.push_back(dependent);
            }
        }
    }

    if (order.size() != manifestCount)
    {
        // A cycle exists among the unordered components. Run a depth first
        // search over the adjacency list until a back edge is found, then
        // report every participant on that cycle path.
        std::vector<std::vector<ManifestIndex>> adjacency(manifestCount);
        for (const auto &edge : uniqueEdges)
        {
            adjacency[edge.first].push_back(edge.second);
        }

        std::vector<VisitState>    visitState(manifestCount, VisitState::Unvisited);
        std::vector<ManifestIndex> searchPath;
        std::vector<ManifestIndex> cycleNodes;

        const std::function<bool(ManifestIndex)> searchCycle = [&](ManifestIndex node) -> bool
        {
            visitState[node] = VisitState::InProgress;
            searchPath.push_back(node);
            for (ManifestIndex next : adjacency[node])
            {
                if (visitState[next] == VisitState::InProgress)
                {
                    const auto start = std::find(searchPath.begin(), searchPath.end(), next);
                    cycleNodes.assign(start, searchPath.end());
                    return true;
                }
                if (visitState[next] == VisitState::Unvisited && searchCycle(next))
                {
                    return true;
                }
            }
            searchPath.pop_back();
            return false;
        };

        for (ManifestIndex index = 0; index < manifestCount && cycleNodes.empty(); ++index)
        {
            if (visitState[index] == VisitState::Unvisited)
            {
                searchCycle(index);
            }
        }

        ResolutionDiagnostic diagnostic;
        diagnostic.kind = ResolutionDiagnostic::Kind::CyclicDependency;
        if (!cycleNodes.empty())
        {
            // cycleNodes holds the cycle path [entry, ..., current]; entry is
            // the node the back edge returns to, so closing the chain with the
            // entry name shows the full loop.
            std::string chain;
            for (ManifestIndex node : cycleNodes)
            {
                if (!chain.empty())
                {
                    chain += " -> ";
                }
                chain += manifests_[node].name;
            }
            chain += " -> " + manifests_[cycleNodes.front()].name;
            diagnostic.message = "dependency cycle detected: " + chain;
            for (ManifestIndex node : cycleNodes)
            {
                diagnostic.involvementChain.push_back(manifests_[node].name);
            }
        }
        else
        {
            diagnostic.message = "dependency cycle detected among the available components.";
        }
        result.diagnostics.push_back(std::move(diagnostic));
        result.successful = false;
        return result;
    }

    result.constructionOrder.reserve(manifestCount);
    for (ManifestIndex index : order)
    {
        result.constructionOrder.push_back(index);
    }
    result.successful = result.diagnostics.empty();
    return result;
}

} // namespace engine
