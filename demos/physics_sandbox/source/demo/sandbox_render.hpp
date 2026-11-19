// sandbox_render.hpp - primitive mesh generation, per-fixed-step state
// capture for interpolation, and draw-command assembly for the physics
// sandbox demo. Rendering stays presentation-only: every transform comes
// from the physics capabilities' state copies.
#pragma once

#include <engine/physics_fur/capabilities.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_particles/capabilities.hpp>
#include <engine/physics_foliage/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>

#include "sandbox_scene.hpp"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace demo::physics_sandbox
{

// Render state captured for one body after every fixed step, so the render
// pass can interpolate between the last two simulated states.
struct BodyRenderState
{
    engine::physics_rigid::RigidTransform current;
    engine::physics_rigid::RigidTransform previous;
};

struct ClothRenderBuffers
{
    std::vector<double> currentPositions;
    std::vector<double> previousPositions;
    std::uint32_t       columns    = 0;
    std::uint32_t       rows       = 0;
    double              nodeRadius = 0.05;
};

class SandboxRenderer final
{
public:
    // Creates the primitive meshes (unit box, unit sphere, unit capsule,
    // cloth node sphere). Returns false when any upload fails.
    bool createMeshes(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer,
        std::string &outReport);

    void destroyMeshes(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer);

    // Generates the fur strand texture through the physics_fur capability and
    // uploads it; the fur ball's drawFur layer stack samples it. Safe to
    // call when the physics_fur capability is absent (the fur draw then skips).
    void prepareFur(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer,
        const engine::physics_fur::FurCapability *furCapability);

    // Rebuilds the tracked-state bookkeeping after a scene (re)build.
    void syncScene(const SceneDependencies &dependencies, const SceneHandles &handles,
        const engine::physics_cloth::ClothCapability *clothCapability);

    // Copies one fixed step's end state: the previous "current" becomes the
    // interpolation source and the fresh state becomes the target.
    void recordFixedStep(const SceneDependencies &dependencies, const SceneHandles &handles,
        const engine::physics_cloth::ClothCapability *clothCapability);

    // Assembles the draw commands for the interpolated pose at alpha in [0,1).
    void buildDrawCommands(const SceneHandles &handles, float alpha);

    // Extended-scene capture: rope nodes, fluid particles, particle states,
    // foliage bends, water tiles, foot targets, and jiggle bones. Call after
    // the fixed-update pass (no interpolation: the extended objects are
    // drawn at their latest state).
    void recordExtendedFixedStep(const SceneDependencies &dependencies, const SceneHandles &handles,
        double fixedDeltaSeconds);

    // Assembles the extended objects' instance matrices; call after
    // buildDrawCommands and before submit.
    void buildExtendedDrawCommands(const SceneDependencies &dependencies, const SceneHandles &handles, float alpha);

    // Submits the assembled commands between beginFrame/endFrame.
    void submit(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer);

    // Submits the extended objects' instanced draws (and the fur layer
    // stack) after submit().
    void submitExtended(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer,
        const SceneDependencies &dependencies, const SceneHandles &handles);

    std::uint32_t clothNodeInstanceCount() const
    {
        return clothNodeMatrices_.size();
    }
    std::uint32_t clothLinkInstanceCount() const
    {
        return clothLinkMatrices_.size();
    }

private:
    struct MeshEntry
    {
        engine::renderer::MeshHandle handle = engine::renderer::invalidMesh;
    };

    engine::renderer::MeshHandle cubeMesh_      = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle sphereMesh_    = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle capsuleMesh_   = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle clothNodeMesh_ = engine::renderer::invalidMesh;

    std::unordered_map<engine::physics_rigid::BodyHandle, std::size_t> bodyStateIndex_;
    std::vector<BodyRenderState>                                       bodyStates_;
    std::vector<engine::physics_rigid::ShapeDescription>               bodyShapes_;
    std::vector<engine::physics_rigid::BodyKind>                       bodyKinds_;

    std::vector<ClothRenderBuffers> clothBuffers_;

    // Extended-scene capture and instance buffers.
    struct RopeRenderBuffers
    {
        std::vector<double> positions;
        std::uint32_t       nodeCount = 0;
        double              nodeRadius = 0.07;
    };
    struct ParticleRenderBuffers
    {
        std::vector<engine::physics_particles::ParticleState> states;
    };
    struct FoliageRenderBuffers
    {
        std::vector<engine::physics_foliage::FoliageBend> bends;
    };
    std::vector<RopeRenderBuffers>     ropeBuffers_;
    std::vector<double>                fluidPositions_;
    std::array<ParticleRenderBuffers, ExtendedHandles::kEmitterSlotCount> particleBuffers_;
    FoliageRenderBuffers               foliageBends_;
    bool                               foliageValid_ = false;
    double                             waterClock_   = 0.0;
    float                              submitAlpha_  = 0.0f;

    using Matrix16 = std::array<float, 16>;

    std::vector<Matrix16>                            modelMatrices_;
    std::vector<engine::renderer::MeshMaterial>      materials_;
    std::vector<engine::renderer::RenderDrawCommand> drawCommands_;
    std::vector<Matrix16>                            clothNodeMatrices_;
    std::vector<Matrix16>                            clothLinkMatrices_;

    std::vector<Matrix16>                    waterTileMatrices_;
    std::vector<Matrix16>                    ropeNodeMatrices_;
    std::vector<Matrix16>                    ropeLinkMatrices_;
    std::vector<Matrix16>                    fluidMatrices_;
    std::vector<Matrix16>                    particleMatrices_[ExtendedHandles::kEmitterSlotCount];
    std::vector<Matrix16>                    foliageMatrices_;
    std::vector<Matrix16>                    footMatrices_;
    std::vector<Matrix16>                    jiggleMatrices_;

    engine::renderer::TextureHandle furTextureHandle_ = engine::renderer::invalidTexture;
    bool                            furTextureReady_  = false;
    engine::physics_rigid::BodyHandle furBallBodyHandle_ = engine::physics_rigid::invalidBody;
    bool                            furBallActive_     = false;
};

} // namespace demo::physics_sandbox
