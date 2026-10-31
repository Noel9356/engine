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

enum class VisitState
{
    Unvisited,
    InProgress,
    Visited
};

} // namespace

DependencyResolver::DependencyResolver(
    const std::vector<ComponentManifest> &manifests,
    std::vector<ProviderSelection>        selections
)
    : manifests_(&manifests),
      selections_(std::move(selections))
{}

DependencyResolver::ResolutionResult DependencyResolver::resolve() const
{
    ResolutionResult result;

    const std::vector<ComponentManifest> &manifests = *manifests_;

    const ManifestIndex manifestCount = manifests.size();

    std::map<std::string, ManifestIndex> componentIndices;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        componentIndices.emplace(manifests[index].name, index);
    }

    std::map<std::string, std::vector<ManifestIndex>> capabilityProviders;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        for (const std::string &capabilityName : manifests[index].providedCapabilities)
        {
            capabilityProviders[capabilityName].push_back(index);
        }
    }

    const auto candidateProviders = [&capabilityProviders](const std::string &requirement) {
        std::vector<ManifestIndex> candidates;
        const auto                 exact = capabilityProviders.find(requirement);
        if (exact != capabilityProviders.end())
        {
            candidates = exact->second;
            return candidates;
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

    std::map<std::string, ManifestIndex> selectedProvider;

    std::map<std::string, std::vector<ManifestIndex>> extensionProviders;
    {
        std::set<std::string> requiredNames;
        for (const ComponentManifest &manifest : manifests)
        {
            requiredNames.insert(manifest.requiredCapabilities.begin(), manifest.requiredCapabilities.end());
        }
        std::set<std::string> requirementNames(requiredNames);
        for (const ComponentManifest &manifest : manifests)
        {
            requirementNames.insert(manifest.optionalCapabilities.begin(), manifest.optionalCapabilities.end());
        }
        for (const std::string &requirement : requirementNames)
        {
            const bool                       optionalOnly = requiredNames.find(requirement) == requiredNames.end();
            const std::vector<ManifestIndex> providers    = candidateProviders(requirement);

            const auto selection = std::find_if(
                selections_.begin(), selections_.end(), [&requirement](const ProviderSelection &candidate) {
                    return candidate.capabilityName == requirement;
                }
            );

            if (providers.size() > 1)
            {
                if (selection == selections_.end())
                {
                    if (optionalOnly)
                    {
                        extensionProviders.emplace(requirement, providers);
                        continue;
                    }
                    std::string providerList;
                    for (ManifestIndex providerIndex : providers)
                    {
                        if (!providerList.empty())
                        {
                            providerList += ", ";
                        }
                        providerList += manifests[providerIndex].name;
                    }
                    ResolutionDiagnostic diagnostic;
                    diagnostic.kind    = ResolutionDiagnostic::Kind::AmbiguousCapability;
                    diagnostic.message = "capability '" + requirement + "' is provided by several components ["
                                       + providerList + "]; choose one in the configuration under [providers].";
                    result.diagnostics.push_back(std::move(diagnostic));
                } else
                {
                    const auto chosen = std::find_if(
                        providers.begin(),
                        providers.end(),
                        [this, &selection, &manifests](ManifestIndex providerIndex) {
                            return manifests[providerIndex].name == selection->providerName;
                        }
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
                    } else
                    {
                        selectedProvider.emplace(requirement, *chosen);
                    }
                }
            } else if (providers.size() == 1)
            {
                if (selection != selections_.end() && manifests[providers.front()].name != selection->providerName)
                {
                    ResolutionDiagnostic diagnostic;
                    diagnostic.kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider;
                    diagnostic.message = "configuration selects '" + selection->providerName + "' for capability '"
                                       + requirement + "' but the only provider is '"
                                       + manifests[providers.front()].name + "'.";
                    result.diagnostics.push_back(std::move(diagnostic));
                } else
                {
                    selectedProvider.emplace(requirement, providers.front());
                }
            }
        }
    }

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

    if (!result.diagnostics.empty())
    {
        result.successful = false;
        return result;
    }

    std::vector<std::pair<ManifestIndex, ManifestIndex>> edges;
    for (ManifestIndex consumerIndex = 0; consumerIndex < manifestCount; ++consumerIndex)
    {
        const ComponentManifest &consumer         = manifests[consumerIndex];
        const bool               consumerIsModule = consumer.kind == ComponentKind::Module;

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
            if (consumerIsModule && manifests[provider->second].kind != ComponentKind::Module)
            {
                ResolutionDiagnostic diagnostic;
                diagnostic.kind    = ResolutionDiagnostic::Kind::ModuleModCapability;
                diagnostic.message = "module '" + consumer.name + "' requires capability '" + capabilityName
                                   + "' whose resolved provider is the mod '" + manifests[provider->second].name
                                   + "'; modules can only use capabilities provided by other modules."
                                     " Move the capability's provider into a module, or consume it from a mod.";
                diagnostic.involvementChain.push_back(consumer.name);
                diagnostic.involvementChain.push_back(capabilityName);
                diagnostic.involvementChain.push_back(manifests[provider->second].name);
                result.diagnostics.push_back(std::move(diagnostic));
                continue;
            }
            if (provider->second != consumerIndex)
            {
                edges.emplace_back(provider->second, consumerIndex);
            }
        }

        for (const std::string &capabilityName : consumer.optionalCapabilities)
        {
            const bool alsoRequired
                = std::find(consumer.requiredCapabilities.begin(), consumer.requiredCapabilities.end(), capabilityName)
               != consumer.requiredCapabilities.end();
            if (alsoRequired)
            {
                continue;
            }
            const auto provider = selectedProvider.find(capabilityName);
            if (provider == selectedProvider.end())
            {
                const auto extension = extensionProviders.find(capabilityName);
                if (extension != extensionProviders.end())
                {
                    for (const ManifestIndex providerIndex : extension->second)
                    {
                        if (providerIndex != consumerIndex
                            && !(consumerIsModule && manifests[providerIndex].kind != ComponentKind::Module))
                        {
                            edges.emplace_back(providerIndex, consumerIndex);
                        }
                    }
                }
                continue;
            }

            if (consumerIsModule && manifests[provider->second].kind != ComponentKind::Module)
            {
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

    std::size_t readyCursor = 0;
    while (readyCursor < readyQueue.size())
    {
        const ManifestIndex current = readyQueue[readyCursor++];
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
        std::vector<std::vector<ManifestIndex>> adjacency(manifestCount);
        for (const auto &edge : uniqueEdges)
        {
            adjacency[edge.first].push_back(edge.second);
        }

        std::vector<VisitState>    visitState(manifestCount, VisitState::Unvisited);
        std::vector<ManifestIndex> searchPath;
        std::vector<ManifestIndex> cycleNodes;

        const std::function<bool(ManifestIndex)> searchCycle = [&](ManifestIndex node) -> bool {
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

            visitState[node] = VisitState::Visited;
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
            std::string chain;
            for (ManifestIndex node : cycleNodes)
            {
                if (!chain.empty())
                {
                    chain += " -> ";
                }
                chain += manifests[node].name;
            }
            chain += " -> " + manifests[cycleNodes.front()].name;
            diagnostic.message = "dependency cycle detected: " + chain;
            for (ManifestIndex node : cycleNodes)
            {
                diagnostic.involvementChain.push_back(manifests[node].name);
            }
        } else
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
