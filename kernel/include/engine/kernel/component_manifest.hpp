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
    // into the known-key registry behind --validate's unknown-key warnings:
    // a key the assembly or the command line sets that nobody declares is
    // dead configuration and gets reported as a warning. Arrays of tables are
    // declared by the array's path ("scene.objects") - rows are authored
    // wholesale, so their inner keys need no individual declaration. Modules
    // fill this in their manifest; mod packages declare the same surface as
    // config_keys in mod.toml.
    std::vector<std::string> knownConfigKeys;
};

} // namespace engine

#endif // ENGINE_KERNEL_COMPONENT_MANIFEST_HPP
