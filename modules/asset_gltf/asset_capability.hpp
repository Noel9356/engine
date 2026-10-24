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
// structure maps directly onto this engine's scene contract.
//
// Mapping contract (documented in detail in the module README): node
// transforms, hierarchy, per-primitive bounds and the glTF material factors
// (base color, metallic, roughness, emissive) are funneled through the scene
// registry so scripts can immediately address what arrived. Two fidelities
// exist: bounding proxies (each primitive becomes one bounding-box object -
// the honest analytic proxy) and full mesh (the primitive's triangles are
// registered in the scene registry and the object instances them; the
// renderer draws the triangles, the tracing consumers see the bbox proxy).
//
// Import fidelity. The bounding-proxy mapping is the v1 contract (each mesh
// primitive becomes one bounding-box object); full mesh import registers the
// primitive's actual triangles in the scene registry and spawns a mesh-type
// object instancing them. Both map node transforms and material factors the
// same way; the difference is only what the renderer draws vs what the
// tracing proxies close.
namespace asset_fidelity
{
inline constexpr std::uint32_t bounding_proxies = 0;
inline constexpr std::uint32_t full_mesh        = 1;
} // namespace asset_fidelity

class AssetCapability
{
public:
    static constexpr const char *capabilityName = "asset";

    virtual ~AssetCapability() = default;

    // Imports one glTF 2.0 file (.gltf JSON; .glb containers are refused with
    // a precise message - the JSON form is the interchange form) into the
    // scene registry, at the requested fidelity (asset_fidelity). Returns the
    // number of objects spawned; on structural failure returns 0 and fills
    // errorReport with one precise message. A partial import is possible (the
    // registry's configured object cap, and the mesh cap for full-mesh
    // imports): the count is what the caller checks.
    //
    // Buffer data (full-mesh fidelity): the importer reads EMBEDDED buffers -
    // a data URI ("data:application/octet-stream;base64,...") - and refuses
    // external .bin references with a precise message: the .gltf JSON form
    // stays self-contained (the module README documents the mapping).
    virtual std::uint32_t importGltfIntoScene(
        const std::string &path, std::uint32_t fidelity, std::string &errorReport
    ) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_ASSET_ASSET_CAPABILITY_HPP
