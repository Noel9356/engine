// Capability dependency resolution: provider selection, graph build and
// topological construction order with cycle diagnostics.

#include <engine/kernel/capability_name.hpp>
#include <engine/kernel/dependency_resolver.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

// Index into the manifests vector.
using ManifestIndex = std::size_t;

// DFS colors for cycle detection (InProgress = on the current path).
enum class VisitState
{
    Unvisited,
    InProgress,
    Visited
};

} // namespace

DependencyResolver::DependencyResolver(
    const std::vector<ParticipantManifest> &manifests, std::vector<ProviderSelection> selections)
    : manifests_(&manifests),
      selections_(std::move(selections))
{}

DependencyResolver::ResolutionResult DependencyResolver::resolve() const
{
    ResolutionResult result;

    const std::vector<ParticipantManifest> &manifests = *manifests_;

    const ManifestIndex manifestCount = manifests.size();

    // Name -> index (first wins on duplicate names; the kernel rejects
    // duplicates before resolution).
    std::map<std::string, ManifestIndex> participantIndices;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        participantIndices.emplace(manifests[index].name, index);
    }

    // Provided capability name -> providing manifest indices.
    std::map<std::string, std::vector<ManifestIndex>> capabilityProviders;
    for (ManifestIndex index = 0; index < manifestCount; ++index)
    {
        for (const std::string &capabilityName : manifests[index].providedCapabilities)
        {
            capabilityProviders[capabilityName].push_back(index);
        }
    }

    // Exact name match first; otherwise any provided dotted capability whose
    // family satisfies the requirement ("renderer" <- "renderer.vulkan").
    const auto candidateProviders = [&capabilityProviders](const std::string &requirement)
    {
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

    // The single chosen provider per capability (from unique providers or a
    // [providers] selection).
    std::map<std::string, ManifestIndex> selectedProvider;

    // Ambiguous capabilities demanded ONLY optionally: every provider stays a
    // candidate (each optional consumer links to each) instead of failing.
    std::map<std::string, std::vector<ManifestIndex>> extensionProviders;
    {
        std::set<std::string> requiredNames;
        for (const ParticipantManifest &manifest : manifests)
        {
            requiredNames.insert(manifest.requiredCapabilities.begin(), manifest.requiredCapabilities.end());
        }
        std::set<std::string> requirementNames(requiredNames);
        for (const ParticipantManifest &manifest : manifests)
        {
            requirementNames.insert(manifest.optionalCapabilities.begin(), manifest.optionalCapabilities.end());
        }
        for (const std::string &requirement : requirementNames)
        {
            // True when no participant requires this capability (only optional lists
            // mention it).
            const bool                       optionalOnly = requiredNames.find(requirement) == requiredNames.end();
            const std::vector<ManifestIndex> providers    = candidateProviders(requirement);

            const auto selection = std::find_if(selections_.begin(),
                selections_.end(),
                [&requirement](const ProviderSelection &candidate) { return candidate.capabilityName == requirement; });

            // Ambiguity handling: fail with AmbiguousCapability, honor a [providers]
            // selection, or (optional-only) defer to the extension set.
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
                    result.diagnostics.push_back(ResolutionDiagnostic{
                        .kind    = ResolutionDiagnostic::Kind::AmbiguousCapability,
                        .message = "capability '" + requirement + "' is provided by several participants ["
                                 + providerList + "]; choose one in the configuration under [providers]."});
                }
                else
                {
                    const auto chosen = std::find_if(providers.begin(),
                        providers.end(),
                        [this, &selection, &manifests](ManifestIndex providerIndex)
                        { return manifests[providerIndex].name == selection->providerName; });
                    if (chosen == providers.end())
                    {
                        result.diagnostics.push_back(ResolutionDiagnostic{
                            .kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider,
                            .message = "configuration selects '" + selection->providerName + "' for capability '"
                                     + requirement + "' but that participant does not provide the capability.",
                            .involvementChain = {selection->providerName, requirement}});
                    }
                    else
                    {
                        selectedProvider.emplace(requirement, *chosen);
                    }
                }
                // A [providers] entry naming anyone else is an error even with exactly
                // one provider.
            }
            else if (providers.size() == 1)
            {
                if (selection != selections_.end() && manifests[providers.front()].name != selection->providerName)
                {
                    result.diagnostics.push_back(ResolutionDiagnostic{
                        .kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider,
                        .message = "configuration selects '" + selection->providerName + "' for capability '"
                                 + requirement + "' but the only provider is '" + manifests[providers.front()].name
                                 + "'."});
                }
                else
                {
                    selectedProvider.emplace(requirement, providers.front());
                }
            }
        }
    }

    // Selections for capabilities nobody provides at all.
    for (const ProviderSelection &selection : selections_)
    {
        if (candidateProviders(selection.capabilityName).empty())
        {
            result.diagnostics.push_back(ResolutionDiagnostic{
                .kind    = ResolutionDiagnostic::Kind::UnknownSelectedProvider,
                .message = "configuration selects '" + selection.providerName + "' for capability '"
                         + selection.capabilityName + "' but no participant provides that capability."});
        }
    }

    // No graph is built while selection problems exist (clearer diagnostics).
    if (!result.diagnostics.empty())
    {
        result.successful = false;
        return result;
    }

    // (provider, consumer) requirement edges; self-edges are skipped.
    std::vector<std::pair<ManifestIndex, ManifestIndex>> edges;
    for (ManifestIndex consumerIndex = 0; consumerIndex < manifestCount; ++consumerIndex)
    {
        const ParticipantManifest &consumer         = manifests[consumerIndex];
        const bool                 consumerIsModule = consumer.kind == ParticipantKind::EngineModule;

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
                result.diagnostics.push_back(ResolutionDiagnostic{
                    .kind             = ResolutionDiagnostic::Kind::UnknownCapability,
                    .message          = "participant '" + consumer.name + "' requires capability '" + capabilityName
                                      + "' but no available participant provides it."
                                      + (availableList.empty() ? std::string(" no capabilities are provided at all.")
                                                               : " available capabilities: " + availableList + "."),
                    .involvementChain = {consumer.name, capabilityName}});
                continue;
            }

            // Module isolation: modules may only depend on module-provided
            // capabilities (repeated for optional links below).
            if (consumerIsModule && manifests[provider->second].kind != ParticipantKind::EngineModule)
            {
                result.diagnostics.push_back(ResolutionDiagnostic{
                    .kind             = ResolutionDiagnostic::Kind::ModulePluginCapability,
                    .message          = "module '" + consumer.name + "' requires capability '" + capabilityName
                                      + "' whose resolved provider is the plugin '" + manifests[provider->second].name
                                      + "'; modules can only use capabilities provided by other modules."
                                        " Move the capability's provider into a module, or consume it from a plugin.",
                    .involvementChain = {consumer.name, capabilityName, manifests[provider->second].name}});
                continue;
            }
            if (provider->second != consumerIndex)
            {
                edges.emplace_back(provider->second, consumerIndex);
            }
        }

        // Optional links mirror the required rules but never diagnose a missing
        // provider (that is what "optional" means).
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
                            && !(consumerIsModule && manifests[providerIndex].kind != ParticipantKind::EngineModule))
                        {
                            edges.emplace_back(providerIndex, consumerIndex);
                        }
                    }
                }
                continue;
            }

            if (consumerIsModule && manifests[provider->second].kind != ParticipantKind::EngineModule)
            {
                continue;
            }
            if (provider->second != consumerIndex)
            {
                edges.emplace_back(provider->second, consumerIndex);
            }
        }

        // Direct participant-name requirements become edges too.
        for (const std::string &participantName : consumer.requiredParticipants)
        {
            const auto required = participantIndices.find(participantName);
            if (required == participantIndices.end())
            {
                result.diagnostics.push_back(ResolutionDiagnostic{
                    .kind             = ResolutionDiagnostic::Kind::UnknownComponent,
                    .message          = "participant '" + consumer.name + "' requires participant '" + participantName
                                      + "' but no participant with that name is available.",
                    .involvementChain = {consumer.name, participantName}});
                continue;
            }
            if (required->second != consumerIndex)
            {
                edges.emplace_back(required->second, consumerIndex);
            }
        }
    }

    // Deduplicates edges coming from shared requirements.
    std::set<std::pair<ManifestIndex, ManifestIndex>> uniqueEdges(edges.begin(), edges.end());
    std::vector<ManifestIndex>                        inDegree(manifestCount, 0);
    std::vector<std::vector<ManifestIndex>>           dependents(manifestCount);
    for (const auto &edge : uniqueEdges)
    {
        dependents[edge.first].push_back(edge.second);
        ++inDegree[edge.second];
    }

    // Kahn's algorithm: readyQueue grows as in-degrees hit zero; the cursor
    // walks it like a FIFO.
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

    // Incomplete order => cycle; a DFS finds one concrete cycle to report.
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

        // Recursive DFS; reaching an InProgress node means the path from it forms
        // a cycle, captured into cycleNodes.
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
        }
        else
        {
            diagnostic.message = "dependency cycle detected among the available participants.";
        }
        result.diagnostics.push_back(std::move(diagnostic));
        result.successful = false;
        return result;
    }

    // Providers-first topological order of manifest indices.
    result.constructionOrder.reserve(manifestCount);
    for (ManifestIndex index : order)
    {
        result.constructionOrder.push_back(index);
    }
    result.successful = result.diagnostics.empty();
    return result;
}

} // namespace engine
