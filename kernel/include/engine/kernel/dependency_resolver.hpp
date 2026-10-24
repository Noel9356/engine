#ifndef ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP
#define ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP

#include "component_manifest.hpp"

#include <string>
#include <vector>

namespace engine
{

// Resolves the component graph: every component (module or mod) is ordered so
// that providers construct before consumers, shared capability requirements
// resolve to a single provider, and every failure mode produces one precise
// diagnostic instead of a vague error.
//
// Optional capabilities (ComponentManifest::optionalCapabilities) ride the
// same resolution machinery as required ones - exact providers first, then
// family members, then the configuration's [providers] choice, and an
// ambiguous optional capability is the same startup error, because a silent
// pick would make behavior depend on scan order. The one difference: zero
// providers resolve to "not available" instead of an error, and no edge is
// added. When a provider DOES exist, the edge is added like a requirement's -
// the consumer acquires the implementation during construct.
//
// The resolver is a pure function over manifests: it holds no backend state and
// never touches component instances, which is what makes the kernel startup
// validation a dry run.
class DependencyResolver final
{
public:
    // Explicit provider choice for a capability with several implementations,
    // read from the runtime configuration ("runtime configurability").
    struct ProviderSelection
    {
        std::string capabilityName;
        std::string providerName;
    };

    struct ResolutionDiagnostic
    {
        enum class Kind
        {
            UnknownCapability,       // a required capability has no provider at all
            UnknownComponent,        // a required component name matches nothing
            AmbiguousCapability,     // several providers, no configuration choice
            UnknownSelectedProvider, // configuration chose a provider that does not provide it
            CyclicDependency         // providers and consumers form a cycle
        };

        Kind                     kind = Kind::UnknownCapability;
        std::string              message;
        std::vector<std::string> involvementChain; // component or capability names on the path
    };

    struct ResolutionResult
    {
        bool successful = false;

        // Indices into the manifests vector that was handed to the resolver,
        // providers before consumers. Indices instead of pointers keep the
        // result free of lifetime coupling to resolver internals.
        std::vector<std::size_t>          constructionOrder;
        std::vector<ResolutionDiagnostic> diagnostics;
    };

    DependencyResolver(std::vector<ComponentManifest> manifests, std::vector<ProviderSelection> selections);

    ResolutionResult resolve() const;

private:
    std::vector<ComponentManifest> manifests_;
    std::vector<ProviderSelection> selections_;
};

} // namespace engine

#endif // ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP
