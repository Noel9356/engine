#ifndef ENGINE_KERNEL_COMPONENT_INTERFACE_HPP
#define ENGINE_KERNEL_COMPONENT_INTERFACE_HPP

#include "component_manifest.hpp"
#include "event_bus.hpp"
#include "permission_system.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace engine
{

class ConfigurationStore;

// One script mod handed over to a hosting module. The kernel discovers script
// packages through their manifests; the host module owns the actual runtime
// (isolated Lua states, bridge tables) and the script lifecycle. Which of the
// package's required capabilities names its host is package data
// (ModManifest::hostCapability): the kernel resolves the provider of that
// capability without knowing what it is called.
struct ScriptModDescription
{
    std::string           modId;
    std::filesystem::path scriptPath;
    // The capability, from the package's required set, whose provider hosts
    // the script. The kernel only matches this name against provider
    // declarations; the meaning of the capability lives in the provider's
    // published header.
    std::string hostCapability;
};

// Interface of a module that hosts script mods. A script mod's package
// declares which of its required capabilities the host must provide; the
// kernel resolves that requirement to the active provider and talks to it
// through this contract only - never to a concrete scripting backend.
class ScriptModHost
{
public:
    virtual ~ScriptModHost() = default;

    // Called once per discovered script mod during the construct phase, in
    // dependency order and after the host module itself has constructed.
    // Throwing from here fails startup with a precise diagnostic.
    virtual void hostScriptMod(const ScriptModDescription &description) = 0;
};

// Thrown by ComponentContext when a component acquires a capability it has not
// been granted, and by the kernel on lifecycle misuse. Startup problems are
// collected as diagnostics instead, so this exception means a contract breach.
class ComponentAccessError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

// The kernel-side surface behind ComponentContext's capability acquisition.
// Implemented by EngineKernel. Every call resolves the implementation pointer
// PULL-BASED from the declaring provider at acquisition time: the request
// walks the active components, asks the declaring provider's
// capabilityPointerFor hook and hands the result straight to the caller -
// nothing is stored or registered anywhere between the phases.
class CapabilityResolver
{
public:
    virtual ~CapabilityResolver() = default;

    // Returns the implementation pointer of the named capability. Throws
    // std::logic_error with a precise diagnostic when the request cannot be
    // resolved (no provider, several providers, or a provider that returned
    // no implementation).
    virtual void *resolveCapabilityPointer(const std::string &requesterName, const std::string &capabilityName) = 0;
};

// Everything a module or a mod may touch from the kernel. The context enforces
// the permission system at the only point where cross component access happens:
// capability acquisition. Acquisition is type-erased by name here; the caller
// casts to the capability interface published by the providing module.
class ComponentContext final
{
public:
    // demandedCapabilities is the kernel's demand-closure set (which capability
    // names some component of this run actually requires). It carries no
    // meaning to the kernel; providers consult it to learn whether one of
    // THEIR OWN capabilities is wanted before contributing work. The host
    // context receives null (the host never contributes capability work).
    ComponentContext(
        EventBus                              &eventBus,
        CapabilityResolver                    &capabilityResolver,
        PermissionSystem                      &permissionSystem,
        const ConfigurationStore              &configuration,
        std::string                            componentName,
        const std::unordered_set<std::string> *demandedCapabilities
    );

    EventBus &eventBus()
    {
        return eventBus_;
    }

    // Acquires the implementation of one capability by name and returns it
    // type-erased. Fails when the permission "capability.<name>" was not
    // granted to this component (the engine blocks the access), or when the
    // name has no single active implementation - the resolver's exception
    // carries the precise diagnostic.
    void *acquireCapability(
        const std::string &capabilityName
    ) const
    {
        const std::string permissionName = std::string(capabilityPermissionPrefix) + capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionName))
        {
            throw ComponentAccessError(
                "component '" + componentName_ + "' was blocked by the permission system while acquiring '"
                + capabilityName + "': the permission '" + permissionName
                + "' was not granted. Declare the requirement in the manifest."
            );
        }
        return capabilityResolver_.resolveCapabilityPointer(componentName_, capabilityName);
    }

    // Typed convenience over the name-based acquire above: the caller states
    // the capability interface it compiled against (published by the providing
    // module) and the context resolves the pointer through the kernel's pull.
    // The cast is the caller's statement of intent; the kernel stays type
    // blind - this template lives at the consumer side and never names a
    // concrete provider.
    template <typename CapabilityType> CapabilityType &acquireCapability() const
    {
        void *capabilityPointer = acquireCapability(std::string(CapabilityType::capabilityName));
        return *static_cast<CapabilityType *>(capabilityPointer);
    }

    // Whether the demand closure of this run contains the named capability.
    // The kernel treats capability names as opaque strings; the question is
    // only meaningful to a provider asking about one of its OWN capabilities
    // ("should I contribute the work this capability stands for?"). Returns
    // false when the context has no demand set (the host context).
    bool isCapabilityDemanded(
        const std::string &capabilityName
    ) const
    {
        return demandedCapabilities_ != nullptr
            && demandedCapabilities_->find(capabilityName) != demandedCapabilities_->end();
    }

    // Typed configuration access into the component's own section.
    std::optional<std::string>  configurationString(const std::string &key) const;
    std::optional<std::int64_t> configurationInteger(const std::string &key) const;
    std::optional<bool>         configurationBoolean(const std::string &key) const;
    std::optional<double>       configurationFloating(const std::string &key) const;

    // Typed configuration access into a named shared section (for example the
    // world content in [scene]). Read-only by convention: components may read
    // shared content, only the host authors it.
    std::optional<std::string> configurationStringInSection(const std::string &section, const std::string &path) const;
    std::vector<std::string>
    configurationStringArrayInSection(const std::string &section, const std::string &path) const;
    std::optional<std::int64_t>
                          configurationIntegerInSection(const std::string &section, const std::string &path) const;
    std::optional<bool>   configurationBooleanInSection(const std::string &section, const std::string &path) const;
    std::optional<double> configurationFloatingInSection(const std::string &section, const std::string &path) const;
    std::vector<double> configurationFloatingArrayInSection(const std::string &section, const std::string &path) const;
    std::size_t         configurationArraySizeInSection(const std::string &section, const std::string &path) const;

private:
    EventBus                              &eventBus_;
    CapabilityResolver                    &capabilityResolver_;
    PermissionSystem                      &permissionSystem_;
    const ConfigurationStore              &configuration_;
    std::string                            componentName_;
    const std::unordered_set<std::string> *demandedCapabilities_ = nullptr;
};

// Lifecycle contract for modules. construct acquires dependencies and
// subscribes to events without touching backends; start initializes backends
// and publishes initial state; stop releases what start created in reverse.
// The kernel calls these phases in dependency order and the reverse on
// shutdown.
class ModuleInterface
{
public:
    virtual ~ModuleInterface() = default;

    virtual const ComponentManifest &manifest() const = 0;

    virtual void construct(ComponentContext &context) = 0;
    virtual void start()                              = 0;
    virtual void stop()                               = 0;

    // Returns this module typed as the implementation behind the given
    // capability name, or nullptr when the module does not provide it. The
    // kernel calls this at acquisition time - nothing is stored or registered:
    // the pull goes straight to the declaring provider, every time.
    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;

    // Returns the script host surface of this module, or nullptr when the
    // module does not host script mods. The kernel asks through this hook when
    // it constructs a script mod whose manifest names this module as host.
    virtual ScriptModHost *scriptModHost()
    {
        return nullptr;
    }
};

// Lifecycle contract for shared library mods. Identical phase shape to
// ModuleInterface: the dependency graph does not care which side of the
// module/mod line a component comes from.
class ModInterface
{
public:
    virtual ~ModInterface() = default;

    virtual const ComponentManifest &manifest() const = 0;

    virtual void construct(ComponentContext &context) = 0;
    virtual void start()                              = 0;
    virtual void stop()                               = 0;

    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;
};

} // namespace engine

#endif // ENGINE_KERNEL_COMPONENT_INTERFACE_HPP
