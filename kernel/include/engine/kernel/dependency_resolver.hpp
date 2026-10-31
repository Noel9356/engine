#ifndef ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP
#define ENGINE_KERNEL_DEPENDENCY_RESOLVER_HPP

#include "./component_manifest.hpp"

#include <string>
#include <vector>

namespace engine
{

class DependencyResolver final
{
public:
    struct ProviderSelection
    {
        std::string capabilityName;
        std::string providerName;
    };

    struct ResolutionDiagnostic
    {
        enum class Kind
        {
            UnknownCapability,
            UnknownComponent,
            AmbiguousCapability,
            UnknownSelectedProvider,
            CyclicDependency,

            ModuleModCapability
        };

        Kind                     kind = Kind::UnknownCapability;
        std::string              message;
        std::vector<std::string> involvementChain;
    };

    struct ResolutionResult
    {
        bool successful = false;

        std::vector<std::size_t>          constructionOrder;
        std::vector<ResolutionDiagnostic> diagnostics;
    };

    DependencyResolver(
        const std::vector<ComponentManifest> &manifests,
        std::vector<ProviderSelection>        selections
    );

    ResolutionResult resolve() const;

private:
    const std::vector<ComponentManifest> *manifests_;
    std::vector<ProviderSelection>        selections_;
};

} // namespace engine

#endif
