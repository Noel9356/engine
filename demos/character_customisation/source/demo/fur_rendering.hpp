// fur_rendering.hpp - shell-fur asset + draw glue for the character demo.
//
// Bridges the fur module's content/policy capability (presets, procedural
// strand textures, screen-size LOD decisions) onto the renderer module's
// shell-fur draw path (RenderCapability::drawFur). One strand texture per
// preset is generated and uploaded once at startup; per frame the class
// evaluates the LOD decision for a part instance (angular size = part
// bounds radius / camera distance) and submits the shell stack after the
// part's opaque base mesh.
//
// The class degrades gracefully: without the fur capability (module absent)
// every draw becomes a no-op and the demo keeps its legacy vertex-tint fur.

#ifndef ENGINE_DEMO_FUR_RENDERING_HPP
#define ENGINE_DEMO_FUR_RENDERING_HPP

#include <engine/fur/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace demo
{

class FurRendering
{
public:
    // Generates and uploads one strand texture per preset (plush, shaggy,
    // velvety) when furCapability is present; with nullptr the class stays
    // disabled and everything below is a no-op. Returns false when the fur
    // capability exists but a texture failed to generate/upload (a renderer
    // problem worth reporting).
    bool create(const engine::renderer::RenderCapability &renderCapability,
        engine::renderer::RendererHandle rendererHandle, const engine::fur::FurCapability *furCapability);
    void destroy(const engine::renderer::RenderCapability &renderCapability,
        engine::renderer::RendererHandle rendererHandle);

    // CLI-driven knobs.
    void setEnabled(bool enabled)
    {
        enabled_ = enabled;
    }
    [[nodiscard]] bool enabled() const
    {
        return enabled_ && furCapability_ != nullptr && !textures_.empty();
    }
    void setPreset(engine::fur::FurPreset preset)
    {
        preset_ = preset;
    }
    [[nodiscard]] engine::fur::FurPreset preset() const
    {
        return preset_;
    }
    // 0 = the LOD evaluator decides; > 0 pins the layer count (debug/A/B).
    void setLayerOverride(std::uint32_t layers)
    {
        layerOverride_ = layers;
    }
    // Global strand-length multiplier on top of the per-part scale.
    void setLengthScale(float scale)
    {
        lengthScale_ = scale > 0.0f ? scale : 1.0f;
    }
    // Screen-size LOD on/off (debug: pin band 0 everywhere).
    void setLodEnabled(bool enabled)
    {
        lodEnabled_ = enabled;
    }

    // Per-part strand-length scale: ears are short-coated, tails shaggier.
    static float partLengthScale(const char *partId);

    // One preset's uploaded asset bundle.
    struct PresetFur
    {
        engine::fur::FurParameters  parameters{};
        engine::renderer::TextureHandle texture = engine::renderer::invalidTexture;
        std::string                     name;
    };

    [[nodiscard]] const std::vector<PresetFur> &presets() const
    {
        return textures_;
    }
    [[nodiscard]] const PresetFur *presetFur(engine::fur::FurPreset preset) const;

    // Draws the fur shell stack for one part instance. Call AFTER the
    // part's base mesh so the opaque surface provides the depth anchor.
    // geometryRadius is the part-local bounding radius (mesh units);
    // actorRadius (>= 0) overrides it with the whole-character world radius,
    // so every part of one actor shares the actor-level LOD decision (the
    // per-part metric under-drives the bands: individual parts are small
    // even when the actor fills the frame).
    void drawPartFur(engine::renderer::MeshHandle mesh, const float *modelMatrix,
        const engine::renderer::MeshMaterial &baseMaterial, const char *partId, float geometryRadius,
        float actorRadius, const float cameraPosition[3]);

    // Last LOD decision recorded by drawPartFur (QA/self-test introspection).
    [[nodiscard]] const engine::fur::FurLodDecision &lastDecision() const
    {
        return lastDecision_;
    }
    [[nodiscard]] float lastAngularSize() const
    {
        return lastAngularSize_;
    }

private:
    const engine::fur::FurCapability *furCapability_ = nullptr;
    const engine::renderer::RenderCapability *renderCapability_ = nullptr;
    engine::renderer::RendererHandle  rendererHandle_ = engine::renderer::invalidRenderer;

    std::vector<PresetFur>    textures_;
    engine::fur::FurPreset    preset_        = engine::fur::FurPreset::Plush;
    std::uint32_t             layerOverride_ = 0u;
    float                     lengthScale_   = 1.0f;
    bool                      enabled_       = true;
    bool                      lodEnabled_    = true;

    engine::fur::FurLodDecision lastDecision_{};
    float                       lastAngularSize_ = 0.0f;
};

} // namespace demo

#endif // ENGINE_DEMO_FUR_RENDERING_HPP
