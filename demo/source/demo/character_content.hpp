// character_content.hpp - body part assets for the character customisation
// screen.
//
// This is the demo's content layer on top of the character module: it
// registers parameters, compatibility rules, rig fragments, parts, and the
// assembly profile through the character capability, and uploads the part
// meshes to the renderer. Every mesh is authored around its connector
// anchors: a part's origin is the point its parent connector aligns to (a
// head's "head.root" anchor is its neck ring, an arm's "shoulder.root" its
// shoulder ring). The module never sees geometry; it only stores metadata.
//
// Deformation follows the junction contract documented in
// modules/character/docs/customization_design.md: rings away from a part's
// origin never move by part scale - heads and the torso ship baked morphs
// (written by scripts/build_character_parts.py) whose interface deltas are
// exactly the JointTranslation binding offsets derived from the shared
// generated metrics header. Rings at the part origin (limbs, tail, wings)
// deform by part scale, which keeps them on their anchors by construction.

#ifndef ENGINE_DEMO_CHARACTER_CONTENT_HPP
#define ENGINE_DEMO_CHARACTER_CONTENT_HPP

#include "geometry.hpp"

#include <engine/character/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace demo
{

// Everything the screen needs to know about one registered body part asset
// on the rendering side (the module stores the anatomy side).
struct PartRenderInfo
{
    std::string                    displayName;
    std::string                    geometryId; // module-side geometry key
    std::string                    slot;       // "head", "tail", "wings"... for UI grouping
    engine::renderer::MeshMaterial material{};
};

// The parameter subset that drives baked morphs and the procedural neck.
// The screen fills this from the assembly before applyDeformation().
struct DeformationInput
{
    float bodyHeight = 1.0f;
    float bodyWidth  = 1.0f;
    float bodyMuscle = 1.0f;
    float headSize   = 1.0f;
    float headMuzzle = 1.0f;
};

// Body-part content for one character screen. Create before creating the
// assembly; the profile id of the default character is hybridProfileId().
class CharacterContent
{
public:
    // Registers all parameters, rules, rigs, parts, and the profile with the
    // capability, and uploads the part meshes. assetRoot is the demo assets
    // directory (downloaded part OBJs live under character_parts/). Returns
    // false (with a message on stderr) when registration failed.
    bool create(engine::character::CharacterCapability &character, const engine::renderer::RenderCapability &renderer,
        engine::renderer::RendererHandle rendererHandle, const std::string &assetRoot);

    void destroy(engine::character::CharacterCapability &character, const engine::renderer::RenderCapability &renderer,
        engine::renderer::RendererHandle rendererHandle);

    // Re-applies parameter-driven morphs and regenerates the procedural neck
    // piece, reuploading only meshes whose weights moved. Customization
    // geometry is rebuilt here - on parameter change, never per frame.
    void applyDeformation(const engine::character::CharacterCapability &character,
        engine::character::CharacterHandle characterHandle,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle,
        const DeformationInput &input);

    // The junction radius both sides of the neck derive from, for the current
    // body parameters (the head's neck_fit morph weight and the procedural
    // neck piece share this function).
    float neckRadius(const DeformationInput &input) const;

    // Mesh upload keyed by the part definition's geometryId.
    engine::renderer::MeshHandle meshForGeometry(const std::string &geometryId) const;

    const PartRenderInfo *renderInfoForPart(const std::string &partId) const;

    // All part ids registered for a UI slot (head variants, tails...).
    const std::vector<std::string> &partsInSlot(const std::string &slot) const;

    std::uint32_t triangleCountForGeometry(const std::string &geometryId) const;

private:
    // CPU geometry per geometryId: the rest-pose mesh (morph source), its
    // baked morphs, the morph weights currently on the GPU, and a scratch
    // mesh holding the deformed copy that gets uploaded.
    struct GeometryEntry
    {
        engine::renderer::MeshHandle handle = engine::renderer::invalidMesh;
        std::uint32_t                triangles = 0;
        MeshData                     rest;
        MeshData                     scratch;
        std::vector<std::string>     morphNames;
        std::vector<std::vector<glm::vec3>> morphDeltas;
        std::vector<std::uint32_t>   morphVertexIds; // expanded corner -> welded source vertex
        glm::vec3                    morphWeights{0.0f, 0.0f, 0.0f};
        bool                         uploaded = false;
    };

    // Loads a part OBJ (plus morph blocks) into the entry and uploads it.
    bool loadGeometry(const std::string &geometryId, const std::filesystem::path &path,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);

    // Copies source (welded-vertex-keyed) morph data, remapping delta ids to
    // the expanded corner order the loader produces.
    static void copyMorphs(const GeometryEntry &source, GeometryEntry &target);

    // Uploads the entry's scratch mesh, replacing the previous GPU mesh.
    bool uploadGeometry(const std::string &geometryId, const engine::renderer::RenderCapability &renderer,
        engine::renderer::RendererHandle rendererHandle);

    // Builds the procedural neck tube for the current neck radius.
    void rebuildNeckPiece(const engine::renderer::RenderCapability &renderer,
        engine::renderer::RendererHandle rendererHandle, float neckRadiusValue);

    // Runtime morph weight for one baked morph name.
    float weightForMorph(const std::string &name, const DeformationInput &input) const;

    std::unordered_map<std::string, GeometryEntry>            geometries_;
    std::unordered_map<std::string, PartRenderInfo>           partRenderInfo_;
    std::unordered_map<std::string, std::vector<std::string>> partsBySlot_;
    float lastNeckRadius_ = -1.0f;
};

// The default assembly profile shipped with the demo content.
constexpr const char *hybridProfileId = "hybrid_biped";

} // namespace demo

#endif
