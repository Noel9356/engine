// character_content.hpp - the demo's authored part/rig catalog.
//
// Owns the character module registry setup (parameters, compatibility rules,
// rigs, parts, assembly profiles), the geometry upload table for every part
// OBJ (including the baked morph blocks the extractor emits), the preview
// thumbnail table, and the change-time deformation entry points (morph
// weights and fur markings re-shade a part and re-upload it).

#ifndef ENGINE_DEMO_CHARACTER_CONTENT_HPP
#define ENGINE_DEMO_CHARACTER_CONTENT_HPP

#include "geometry.hpp"

#include <engine/character/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace demo
{

struct PartRenderInfo
{
    std::string displayName;
    std::string geometryId;
    std::string slot;
    engine::renderer::MeshMaterial material;
};

// A baked per-vertex morph block of one part OBJ ("m <name>" + "md" lines).
struct PartMorph
{
    std::string name;
    std::vector<glm::vec3> deltas; // one per rest vertex
};

class CharacterContent
{
public:
    // Registers every parameter, rule, rig, part, and profile, and uploads
    // all part geometries. assetRoot points at the demo assets directory.
    bool create(engine::character::CharacterCapability &character,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle,
        const std::string &assetRoot);

    void destroy(engine::character::CharacterCapability &character,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);

    // ---- geometry lookup ------------------------------------------------
    [[nodiscard]] engine::renderer::MeshHandle meshForGeometry(const std::string &geometryId) const;
    [[nodiscard]] const PartRenderInfo *renderInfoForPart(const std::string &partId) const;
    [[nodiscard]] const std::vector<std::string> &partsInSlot(const std::string &slot) const;
    [[nodiscard]] std::uint32_t triangleCountForGeometry(const std::string &geometryId) const;

    // ---- change-time deformation (morphs + markings) ---------------------
    // Applies the head morphs (muzzle.length / eyes.width weights) and the
    // fur marking mode to every affected geometry and re-uploads it. Called
    // only when a parameter changes, never per frame.
    void applyHeadMorphs(float muzzleWeight, float eyesWidthWeight);
    void applyMarkings(int markingMode, const float furTint[3]);

    // Per-part facts the assembly helpers need.
    [[nodiscard]] float legFootYForPart(const std::string &partId) const;
    [[nodiscard]] bool geometryBounds(const std::string &geometryId, float outCenter[3], float *outRadius) const;
    [[nodiscard]] bool partHasMorph(const std::string &geometryId, const char *morphName) const;

    // Preview thumbnails: renderer textures loaded from
    // assets/previews/<partId>.png (rendered once by the screen's preview
    // capture mode; see --part-previews).
    [[nodiscard]] engine::renderer::TextureHandle previewTextureForPart(const std::string &partId) const;
    bool loadPreviewTextures(const std::string &assetRoot);

    // Static profile ids shared with the screen.
    static constexpr const char *humanoidProfileId = "humanoid_biped";
    static constexpr const char *taurProfileId = "taur";
    static constexpr const char *quadrupedProfileId = "quadruped";

private:
    struct GeometryEntry
    {
        MeshData rest;
        std::vector<PartMorph> morphs;
        std::uint32_t triangles = 0;
        engine::renderer::MeshHandle handle = engine::renderer::invalidMesh;
    };

    bool loadGeometry(const std::string &geometryId, const std::filesystem::path &path,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);
    bool uploadGeometry(const std::string &geometryId,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);
    void mirrorAcrossX(const char *leftId, const char *rightId,
        const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);

    std::map<std::string, GeometryEntry> geometries_;
    std::map<std::string, PartRenderInfo> partRenderInfo_;
    std::map<std::string, std::vector<std::string>> partsBySlot_;
    std::map<std::string, engine::renderer::TextureHandle> previewTextures_;
    std::map<std::string, float> legFootY_;

    engine::renderer::RenderCapability renderCapability_;
    engine::renderer::RendererHandle rendererHandle_ = engine::renderer::invalidRenderer;
};

} // namespace demo

#endif // ENGINE_DEMO_CHARACTER_CONTENT_HPP
