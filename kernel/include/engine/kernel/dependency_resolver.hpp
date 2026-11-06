// Capability dependency resolution over participant manifests.
//
// Given every participant's manifest plus the configuration's [providers]
// selections, DependencyResolver picks one provider per required
// capability, builds the requirement graph and topologically sorts it
// into a construction order, reporting diagnostics for unknown or
// ambiguous capabilities and for cycles.

#ifndef ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP
#define ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP

#include <engine/kernel/participant_manifest.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace engine
{

class DependencyResolver final
{
public:
    // A [providers] entry: this capability must be satisfied by exactly this
    // participant (by name).
    struct ProviderSelection
    {
        std::string capabilityName;
        std::string providerName;
    };

    // One resolution problem, human-readable in message; involvementChain
    // lists the participant/capability names along the path (cycles list the
    // cycle order).
    struct ResolutionDiagnostic
    {
        // UnknownCapability / UnknownComponent: no provider for a required
        // capability or no participant with a required name; AmbiguousCapability:
        // several providers and no [providers] choice; UnknownSelectedProvider:
        // a [providers] entry names a participant that does not provide the
        // capability (or no one provides it); CyclicDependency: requirement
        // cycle; ModuleModCapability: a module requires a capability whose
        // resolved provider is a mod.
        enum class Kind
        {
            UnknownCapability,
            UnknownComponent,
            AmbiguousCapability,
            UnknownSelectedProvider,
            CyclicDependency,

            ModuleModCapability
        };

        Kind                     kind             = Kind::UnknownCapability;
        std::string              message          = {};
        std::vector<std::string> involvementChain = {};
    };

    // successful is false iff diagnostics is non-empty. constructionOrder
    // holds manifest indices sorted providers-first (valid only when
    // successful).
    struct ResolutionResult
    {
        bool successful = false;

        std::vector<std::size_t>          constructionOrder;
        std::vector<ResolutionDiagnostic> diagnostics;
    };

    // manifests must outlive the resolver; selections come from [providers].
    DependencyResolver(const std::vector<ParticipantManifest> &manifests, std::vector<ProviderSelection> selections);

    // Pure: no state changes, may be called repeatedly (also used to re-derive
    // the order for reports before anything is constructed).
    ResolutionResult resolve() const;

private:
    // Non-owning; the caller keeps manifests alive across resolve().
    const std::vector<ParticipantManifest> *manifests_;
    std::vector<ProviderSelection>          selections_;
};

} // namespace engine

#endif
