#ifndef ENGINE_KERNEL_MOD_MANIFEST_HPP
#define ENGINE_KERNEL_MOD_MANIFEST_HPP

#include "mod_api.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace engine
{

// Self-description of one mod package: a mod.toml file at the root of the
// package directory. The manifest is authoritative for the mod's identity and
// wiring; the kernel turns it into the component manifest used by the
// dependency resolver and the permission system. Schema:
//
//   id           = "example_cpp_mod"     # required, [a-zA-Z0-9_], unique
//   version      = "1.0.0"               # optional, defaults to 0.0.0
//   engine_api   = 1                     # required, must match the engine's
//                                        # engineModInterfaceVersion
//   type         = "native"              # required: "native" or "script"
//   entry        = "example_cpp_mod"     # required: native -> library base
//                                        # name (lib<entry>.so / <entry>.dll),
//                                        # script -> script file relative to
//                                        # the package directory
//   display_name = "Example Mod"         # optional, defaults to id
//   description  = "..."                 # optional
//   author       = "..."                 # optional
//   capabilities = ["example_capability"] # required capabilities (graph edges);
//                                        # each also grants its capability.<name>
//                                        # permission, so acquisitions need no
//                                        # second list. Mods never name a module;
//                                        # they name the capabilities they consume.
//   optional_capabilities = ["..."]      # capabilities the mod uses when a
//                                        # provider is active and functions
//                                        # without otherwise: a missing provider
//                                        # is a valid state, the capability is
//                                        # never demanded (its provider is not
//                                        # pulled into the run), and each entry
//                                        # still grants its capability.<name>
//                                        # permission and orders an existing
//                                        # provider before this mod.
//   host_capability = "example_capability" # script mods only: which ONE required
//                                        # capability's provider hosts the script
//                                        # runtime. The kernel resolves that
//                                        # provider without interpreting the name.
//   permissions  = ["filesystem.write"]  # extra permission grants beyond the
//                                        # derived capability ones (deniable
//                                        # in configuration)
//   requires     = []                    # component names to order after
//                                        # (any mod type)
//   config_keys  = ["example_mod.level"] # configuration keys the mod reads
//                                        # (feeds the known-key registry)
//   scene        = "scene.toml"          # optional: world content file the mod
//                                        # ships, relative to the package; see
//                                        # the scene source precedence below
//
// TOML rather than JSON keeps one configuration language across the engine
// and lets the kernel parse manifests with the dependency it already carries.
struct ModManifest
{
    std::string              id;
    std::string              version   = "0.0.0";
    std::uint32_t            engineApi = engineModInterfaceVersion;
    std::string              type; // "native" or "script"
    std::string              entry;
    std::string              displayName;
    std::string              description;
    std::string              author;
    std::vector<std::string> capabilities;
    // Capabilities used when a provider is active, never demanded. Wired into
    // the component manifest's optionalCapabilities: the resolver orders an
    // existing provider before this mod, the permission is derived, and a
    // missing provider degrades instead of failing startup. The mod probes
    // availability at construct time through ComponentContext.
    std::vector<std::string> optionalCapabilities;
    // Script mods only: the required capability whose provider hosts the
    // script. Package data, like the rest of the manifest - the kernel
    // resolves the provider of this capability but never interprets the name
    // (a script host is whatever module provides it). Empty for native mods.
    std::string              hostCapability;
    std::vector<std::string> permissions;
    std::vector<std::string> requiresComponents;
    // Configuration keys the mod reads, "<section>.<dotted path>" each. They
    // become the component manifest's knownConfigKeys, so keys the assembly
    // sets for the mod are recognized instead of warned about
    // as unknown. Optional; a mod that reads no configuration declares none.
    std::vector<std::string> configKeys;

    // World content the mod ships: a [scene]-only TOML file relative to the
    // package directory. When the assembly configuration defines no [scene]
    // section of its own, this file provides the world
    // (precedence: an explicitly staged scene file > assembly [scene] > the single claimed content
    // mod's scene file > code defaults). Several active mods may declare
    // scenes freely; only a claim that would be applied without an explicit
    // world source is checked for existence, and two or more claims with no
    // explicit world source are a startup error - the world would be
    // ambiguous. Optional; a pure behavior mod declares none.
    std::string scene;

    std::filesystem::path directory;

    bool isNative() const
    {
        return type == "native";
    }

    bool isScript() const
    {
        return type == "script";
    }
};

// Loads and validates mod.toml files. All failures come back as one precise
// message each, so a broken package names the offending field and the
// expected values instead of a generic parse error.
class ModManifestLoader final
{
public:
    static constexpr const char *manifestFileName = "mod.toml";

    bool
    loadFromFile(const std::filesystem::path &manifestPath, ModManifest &outManifest, std::string &errorMessage) const;
};

} // namespace engine

#endif // ENGINE_KERNEL_MOD_MANIFEST_HPP
