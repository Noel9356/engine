#ifndef ENGINE_MODULES_RENDERER_MESH_GENERATION_HPP
#define ENGINE_MODULES_RENDERER_MESH_GENERATION_HPP

// Host side mesh generation for the two primitive shapes the scene supports.
// Pure functions over no Vulkan state: the renderer uploads the results
// through its staging path. Positions/normals are unit space; the model
// matrix places each object.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "scene_capability.hpp" // meshFloatsPerVertex: ONE definition of the
                                // shared vertex layout (the scene registry's
                                // mesh storage and this module's unit meshes
                                // interleave identically, so a registered
                                // mesh uploads exactly the bytes it carries).

namespace engine
{

// Interleaved vertex layout shared by both meshes: pos.xyz, normal.xyz
// (meshFloatsPerVertex floats per vertex; the scene pipeline's vertex input
// state mirrors it). The constant itself lives in scene_capability.hpp.

struct MeshData
{
    std::vector<float>         vertices; // meshFloatsPerVertex per vertex
    std::vector<std::uint32_t> indices;
};

// Unit cube: 24 vertices (per-face normals, corners at +-1) and 36 indices.
// modelMatrix scales by the primitive's HALF EXTENTS, so a scaled unit cube
// spans exactly the box the analytic SDF describes (signedDistanceBox) -
// rasterized geometry and traced geometry are the same size by construction.
MeshData buildBoxMesh();

// UV sphere, unit radius (position equals the normal).
MeshData buildSphereMesh(std::uint32_t slices, std::uint32_t stacks);

// The tessellation the renderer uses for every sphere object.
inline constexpr std::uint32_t sphereMeshSlices = 24;
inline constexpr std::uint32_t sphereMeshStacks = 12;

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_MESH_GENERATION_HPP
