#ifndef ENGINE_MODULES_ASSET_ASSET_CAPABILITY_HPP
#define ENGINE_MODULES_ASSET_ASSET_CAPABILITY_HPP

// The published capability surface of the asset module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic.

#include <cstdint>
#include <string>

namespace engine
{

// Capability "asset": content import. The first (and for now only) source is
// glTF 2.0: the format the ecosystem standardizes on, and the one whose
// structure maps directly onto this engine's analytic scene contract.
//
// Mapping contract (documented in detail in the module README): each mesh
// primitive becomes ONE bounding-box scene object - the renderer rasterizes
// and traces analytic primitives (boxes/spheres), so mesh-accurate proxies
// are out of scope until a mesh pipeline exists. What the import gives a mod
// is real level data: node transforms, hierarchy, per-primitive world bounds
// and the glTF material factors (base color, metallic, roughness, emissive),
// all funneled through the scene registry so scripts can immediately address
// what arrived.
class AssetCapability
{
public:
    static constexpr const char *capabilityName = "asset";

    virtual ~AssetCapability() = default;

    // Imports one glTF 2.0 file (.gltf JSON; .glb containers are refused with
    // a precise message - the JSON form is the interchange form) into the
    // scene registry. Returns the number of objects spawned; on structural
    // failure returns 0 and fills errorReport with one precise message. A
    // partial import is possible (the registry caps at 32 live objects): the
    // count is what the caller checks.
    virtual std::uint32_t importGltfIntoScene(
        const std::string &path, std::string &errorReport
    ) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_ASSET_ASSET_CAPABILITY_HPP
