#ifndef ENGINE_KERNEL_CAPABILITY_REGISTRY_HPP
#define ENGINE_KERNEL_CAPABILITY_REGISTRY_HPP

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace engine
{

// Maps capability contract names ("window", "logging", ...) to the single
// provider instance selected during dependency resolution. When two components
// require the same capability they both receive this one shared instance; the
// registry is the mechanism that makes shared dependencies free of duplicates.
class CapabilityRegistry final
{
public:
    // Registers the implementation pointer under the capability name. Throws
    // std::logic_error when the capability is already provided: provider
    // conflicts must be resolved through configuration before construction.
    void registerCapability(const std::string& capabilityName, const std::string& providerName, void* capabilityPointer)
    {
        if (capabilities.find(capabilityName) != capabilities.end())
        {
            throw std::logic_error("capability '" + capabilityName + "' is already registered by provider '" +
                                   capabilities[capabilityName].providerName + "'");
        }
        capabilities.emplace(capabilityName, CapabilityEntry{capabilityPointer, providerName});
    }

    // Returns the raw implementation pointer. Callers cast to the concrete
    // capability contract type; ComponentContext performs the permission check
    // and the registry performs the existence check.
    void* acquireCapability(const std::string& capabilityName) const
    {
        const auto found = capabilities.find(capabilityName);
        if (found == capabilities.end())
        {
            throw std::logic_error("capability '" + capabilityName + "' is not registered");
        }
        return found->second.pointer;
    }

private:
    struct CapabilityEntry
    {
        void* pointer = nullptr;
        std::string providerName;
    };

    std::unordered_map<std::string, CapabilityEntry> capabilities;
};

} // namespace engine

#endif // ENGINE_KERNEL_CAPABILITY_REGISTRY_HPP
