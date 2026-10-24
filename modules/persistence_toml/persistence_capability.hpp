#ifndef ENGINE_MODULES_PERSISTENCE_PERSISTENCE_CAPABILITY_HPP
#define ENGINE_MODULES_PERSISTENCE_PERSISTENCE_CAPABILITY_HPP

// The published capability surface of the persistence module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the module
// that provides the capability. Like every published capability header this
// stays implementation-, platform- and vendor-agnostic.
//
// The capability is the runtime save/load for the scene registry: the whole
// live object list (transforms, materials, mesh instances) serializes to one
// TOML file and reads back the same way - TOML everywhere, per the engine's
// configuration policy. Loading spawns the saved objects as NEW registry
// objects (handles are runtime values and are assigned fresh); it never
// merges into existing objects, so load-after-save is an exact restore only
// when the registry was empty or clearScene() ran first.
//
// Mesh objects are saved by their registry mesh NAME (handles are runtime
// values a file cannot legally carry): loading requires the named meshes to
// be registered at load time - the same run, or an import that ran first
// (imports register meshes under deterministic names, and registration is
// idempotent by name).

#include <cstdint>
#include <string>

namespace engine
{

class PersistenceCapability
{
public:
    static constexpr const char *capabilityName = "persistence";

    virtual ~PersistenceCapability() = default;

    // Writes every live scene-registry object to one TOML file (relative
    // paths resolve against the executable directory, the engine's path
    // convention). Returns false with a precise message when the registry
    // cannot be read or the file cannot be written (the permission system
    // gates that through the module's filesystem.write grant).
    virtual bool saveScene(
        const std::string &path, std::string &errorReport
    ) = 0;

    // Spawns every object of one saved TOML file into the scene registry.
    // Returns the number of objects spawned; a structurally invalid file
    // returns 0 with a precise message (file, table and key named). A file
    // whose mesh objects name unregistered meshes degrades honestly: those
    // rows are skipped with a precise log and the count reflects what
    // actually arrived.
    virtual std::uint32_t loadScene(
        const std::string &path, std::string &errorReport
    ) = 0;

    // Removes every live object from the scene registry (meshes stay: they
    // are assets, not instances). Returns false with a precise message only
    // when the registry is unavailable; an empty registry clears to true.
    virtual bool clearScene(
        std::string &errorReport
    ) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_PERSISTENCE_PERSISTENCE_CAPABILITY_HPP
