#ifndef ENGINE_KERNEL_COMPONENT_INTERFACE_HPP
#define ENGINE_KERNEL_COMPONENT_INTERFACE_HPP

#include "capability_registry.hpp"
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
// (isolated Lua states, bridge tables) and the script lifecycle.
struct ScriptModDescription
{
    std::string           modId;
    std::filesystem::path scriptPath;
};

// Interface of a module that hosts script mods. A script mod declares the
// scripting capability in its package manifest; the kernel resolves that
// requirement to the active provider module and talks to it through this
// contract only - never to a concrete scripting backend.
class ScriptModHost
{
public:
    // The capability name whose provider hosts script mods. Defined here
    // because both the kernel (host resolution) and the manifest loader
    // (package validation) build on it; modules that host script mods
    // provide it, script mods require it.
    static constexpr const char *scriptingCapabilityName = "scripting";

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

// Everything a module or a mod may touch from the kernel. The context enforces
// the permission system at the only point where cross component access happens:
// capability acquisition.
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
        CapabilityRegistry                    &capabilityRegistry,
        PermissionSystem                      &permissionSystem,
        const ConfigurationStore              &configuration,
        std::string                            componentName,
        const std::unordered_set<std::string> *demandedCapabilities
    );

    EventBus &eventBus()
    {
        return eventBus_;
    }

    // Acquires the shared provider instance of one capability contract. Fails
    // when the permission "capability.<name>" was not granted to this
    // component (the engine blocks the access), or when the capability does
    // not exist.
    template <typename CapabilityType> CapabilityType &acquireCapability() const
    {
        const std::string capabilityName = CapabilityType::capabilityName;
        const std::string permissionName = std::string(capabilityPermissionPrefix) + capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionName))
        {
            throw ComponentAccessError(
                "component '" + componentName_ + "' was blocked by the permission system while acquiring '"
                + capabilityName + "': the permission '" + permissionName
                + "' was not granted. Declare the requirement in the manifest."
            );
        }
        void *capabilityPointer = capabilityRegistry_.acquireCapability(capabilityName);
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
    CapabilityRegistry                    &capabilityRegistry_;
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

    // Returns this module typed as the capability contract registered under the
    // given capability name, or nullptr when the module does not provide it.
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
