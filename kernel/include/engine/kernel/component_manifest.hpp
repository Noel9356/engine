#ifndef ENGINE_KERNEL_COMPONENT_MANIFEST_HPP
#define ENGINE_KERNEL_COMPONENT_MANIFEST_HPP

#include <string>
#include <vector>

namespace engine
{

enum class ComponentKind
{
    Module,
    SharedLibraryMod,
    ScriptMod,
    Application
};

struct ComponentManifest
{
    std::string   name;
    ComponentKind kind = ComponentKind::Module;

    std::string version = "0.1.0";

    std::vector<std::string> providedCapabilities;

    std::vector<std::string> requiredCapabilities;

    std::vector<std::string> optionalCapabilities;

    std::vector<std::string> requiredComponents;

    std::vector<std::string> requiredPermissions;

    std::vector<std::string> knownConfigKeys;
};

} // namespace engine

#endif
