#ifndef ENGINE_KERNEL_COMPONENT_MANIFEST_HPP
#define ENGINE_KERNEL_COMPONENT_MANIFEST_HPP

#include <string>
#include <vector>

namespace engine
{

// Modules and mods are both "components" in the dependency graph. The
// distinction only affects how the kernel obtains the instance: modules are
// compiled into (or linked against) the executable, mods arrive through shared
// libraries or scripting hosts.
enum class ComponentKind
{
    Module,
    SharedLibraryMod,
    ScriptMod
};

// Static description of one component, consumed by the dependency resolver and
// the permission system. A manifest must never require backend initialization:
// building it must be free of side effects so validation can run dry.
struct ComponentManifest
{
    std::string   name;
    ComponentKind kind = ComponentKind::Module;

    // Capability names this component makes available to others. A capability
    // is an engine level contract (its name is the contract's capabilityName);
    // it may be implemented by a module or by a mod.
    std::vector<std::string> providedCapabilities;

    // Capability names that must resolve to a started provider before this
    // component may construct. This is the direction-agnostic dependency edge:
    // modules may depend on mods and mods may depend on modules.
    std::vector<std::string> requiredCapabilities;

    // Capability names the component USES when a provider is present but does
    // not need to function: a missing provider is a valid state, never a
    // startup problem. The resolver applies the same determinism rules as for
    // required capabilities (a single provider, or the [providers] choice
    // among several - an ambiguous optional capability is still a startup
    // error, because silently picking one would depend on scan order); the
    // ONE difference is that zero providers resolves to "not available"
    // instead of an error. When a provider exists it is ordered before this
    // component (the same graph edge a requirement creates), and the derived
    // "capability.<name>" permission grant covers the acquisition. The
    // consumer probes availability at construct time through
    // ComponentContext::isCapabilityAvailable and acquires only then.
    // Optional capabilities deliberately do NOT join the demand closure:
    // using something optionally must never pull its provider into a run
    // that has no other use for it.
    std::vector<std::string> optionalCapabilities;

    // Component names this component must be ordered after, regardless of
    // capabilities. Rarely needed: capability requirements already order
    // providers before consumers.
    std::vector<std::string> requiredComponents;

    // Permission identifiers the component declares, for example
    // "capability.renderer" or "filesystem.write". The kernel grants declared
    // permissions unless the runtime configuration denies them.
    std::vector<std::string> requiredPermissions;

    // Configuration keys the component reads, as "<section>.<dotted path>"
    // ("scene.camera.fov"). The kernel aggregates these
    // into the known-key registry behind the unknown-key warnings:
    // a key the assembly or a staged override sets that nobody declares is
    // dead configuration and gets reported as a warning. Arrays of tables are
    // declared by the array's path ("scene.objects") - rows are authored
    // wholesale, so their inner keys need no individual declaration. Modules
    // fill this in their manifest; mod packages declare the same surface as
    // config_keys in mod.toml.
    std::vector<std::string> knownConfigKeys;
};

} // namespace engine

#endif // ENGINE_KERNEL_COMPONENT_MANIFEST_HPP
