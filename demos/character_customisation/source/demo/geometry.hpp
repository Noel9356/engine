// geometry.hpp - procedural mesh generation for the demo scene.
//
// CPU-side builders for the primitive meshes (box, sphere, rock, torus) that
// benchmark_scene.cpp uploads through the renderer capability. Vertex layout matches the
// renderer's fixed interleaved format (position, normal, color, uv).

#ifndef ENGINE_DEMO_GEOMETRY_HPP
#define ENGINE_DEMO_GEOMETRY_HPP

#include <engine/renderer/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace demo
{

// Interleaved vertex matching the renderer's default 11-float layout
// (renderer::meshVertexFloatCount): position, normal, color, uv.
struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
    glm::vec2 uv;
};

// CPU mesh: tightly packed vertices plus a 32-bit index buffer, ready to be
// turned into a renderer::MeshDescription for upload.
struct MeshData
{
    std::vector<Vertex>        vertices;
    std::vector<std::uint32_t> indices;

    [[nodiscard]] engine::renderer::MeshDescription description() const;
};

// makeBoxMesh: axis-aligned box from center and half extents (uvScale tiles the
// per-face UVs).
// makeSphereMesh: UV sphere; segments around Y, rings pole to pole, vertex
// color blended from equatorColor (low) to zenithColor (top).
// makeRockMesh: sphere displaced by layered sine noise for a jagged look;
// returns an empty mesh for invalid parameters.
// makeTorusMesh: torus around the Y axis from major/minor radii; returns an
// empty mesh for invalid parameters.
[[nodiscard]] MeshData makeBoxMesh(
    const glm::vec3 &center, const glm::vec3 &halfExtent, const glm::vec3 &color, float uvScale = 1.0f);
[[nodiscard]] MeshData makeSphereMesh(const glm::vec3 &center, float radius, std::uint32_t segments,
    std::uint32_t rings, const glm::vec3 &equatorColor, const glm::vec3 &zenithColor);
[[nodiscard]] MeshData makeRockMesh(
    float radius, std::uint32_t segments, std::uint32_t rings, const glm::vec3 &baseColor);
void appendTransformedMesh(MeshData &destination, const MeshData &source, const glm::mat4 &transform);

[[nodiscard]] MeshData makeTorusMesh(float majorRadius, float minorRadius, std::uint32_t majorSegments,
    std::uint32_t minorSegments, const glm::vec3 &color);

} // namespace demo

#endif
