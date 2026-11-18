// sandbox_render.hpp - primitive mesh generation, per-fixed-step state
// capture for interpolation, and draw-command assembly for the physics
// sandbox demo. Rendering stays presentation-only: every transform comes
// from the physics capabilities' state copies.
#pragma once

#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_cloth/capabilities.hpp>
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

    // Rebuilds the tracked-state bookkeeping after a scene (re)build.
    void syncScene(const SceneDependencies &dependencies, const SceneHandles &handles,
        const engine::physics_cloth::ClothCapability *clothCapability);

    // Copies one fixed step's end state: the previous "current" becomes the
    // interpolation source and the fresh state becomes the target.
    void recordFixedStep(const SceneDependencies &dependencies, const SceneHandles &handles,
        const engine::physics_cloth::ClothCapability *clothCapability);

    // Assembles the draw commands for the interpolated pose at alpha in [0,1).
    void buildDrawCommands(const SceneHandles &handles, float alpha);

    // Submits the assembled commands between beginFrame/endFrame.
    void submit(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle renderer);

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

    using Matrix16 = std::array<float, 16>;

    std::vector<Matrix16>                            modelMatrices_;
    std::vector<engine::renderer::MeshMaterial>      materials_;
    std::vector<engine::renderer::RenderDrawCommand> drawCommands_;
    std::vector<Matrix16>                            clothNodeMatrices_;
    std::vector<Matrix16>                            clothLinkMatrices_;
};

} // namespace demo::physics_sandbox
