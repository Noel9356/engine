#include "mesh_generation.hpp"

#include "scene_math.hpp"

#include <cmath>

namespace engine
{
namespace
{

// Unit cube face normals (+x, -x, +y, -y, +z, -z).
constexpr std::array<std::array<float, 3>, 6> boxFaceNormals = {{
    {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
    {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f},
}};

// Corner offsets per face so the winding keeps triangles outward facing.
constexpr std::array<std::array<std::array<float, 3>, 4>, 6> boxFaceCorners = {{
    {{{{1.0f, -1.0f, -1.0f}}, {{1.0f, 1.0f, -1.0f}}, {{1.0f, 1.0f, 1.0f}}, {{1.0f, -1.0f, 1.0f}}}},
    {{{{-1.0f, -1.0f, 1.0f}}, {{-1.0f, 1.0f, 1.0f}}, {{-1.0f, 1.0f, -1.0f}}, {{-1.0f, -1.0f, -1.0f}}}},
    {{{{-1.0f, 1.0f, -1.0f}}, {{-1.0f, 1.0f, 1.0f}}, {{1.0f, 1.0f, 1.0f}}, {{1.0f, 1.0f, -1.0f}}}},
    {{{{-1.0f, -1.0f, 1.0f}}, {{-1.0f, -1.0f, -1.0f}}, {{1.0f, -1.0f, -1.0f}}, {{1.0f, -1.0f, 1.0f}}}},
    {{{{-1.0f, -1.0f, 1.0f}}, {{1.0f, -1.0f, 1.0f}}, {{1.0f, 1.0f, 1.0f}}, {{-1.0f, 1.0f, 1.0f}}}},
    {{{{1.0f, -1.0f, -1.0f}}, {{-1.0f, -1.0f, -1.0f}}, {{-1.0f, 1.0f, -1.0f}}, {{1.0f, 1.0f, -1.0f}}}},
}};

constexpr std::size_t boxCornersPerFace = 4;
constexpr std::size_t boxFaceCount = 6;

} // namespace

MeshData buildBoxMesh()
{
    MeshData mesh;
    mesh.vertices.reserve(boxFaceCount * boxCornersPerFace * meshFloatsPerVertex);
    mesh.indices.reserve(boxFaceCount * 6);
    for (std::size_t face = 0; face < boxFaceCount; ++face)
    {
        for (const std::array<float, 3>& corner : boxFaceCorners[face])
        {
            mesh.vertices.push_back(corner[0]);
            mesh.vertices.push_back(corner[1]);
            mesh.vertices.push_back(corner[2]);
            mesh.vertices.push_back(boxFaceNormals[face][0]);
            mesh.vertices.push_back(boxFaceNormals[face][1]);
            mesh.vertices.push_back(boxFaceNormals[face][2]);
        }
        const std::uint32_t base = static_cast<std::uint32_t>(face * boxCornersPerFace);
        const std::array<std::uint32_t, 6> faceIndices = {base, base + 1u, base + 2u, base, base + 2u, base + 3u};
        mesh.indices.insert(mesh.indices.end(), faceIndices.begin(), faceIndices.end());
    }
    return mesh;
}

MeshData buildSphereMesh(std::uint32_t slices, std::uint32_t stacks)
{
    MeshData mesh;
    mesh.vertices.reserve((slices + 1) * (stacks + 1) * meshFloatsPerVertex);
    mesh.indices.reserve(slices * stacks * 6);
    for (std::uint32_t stack = 0; stack <= stacks; ++stack)
    {
        const float phi = pi * static_cast<float>(stack) / static_cast<float>(stacks);
        for (std::uint32_t slice = 0; slice <= slices; ++slice)
        {
            const float theta = twoPi * static_cast<float>(slice) / static_cast<float>(slices);
            const float x = std::sin(phi) * std::cos(theta);
            const float y = std::cos(phi);
            const float z = std::sin(phi) * std::sin(theta);
            // Position equals the unit normal: the sphere is unit radius.
            mesh.vertices.push_back(x);
            mesh.vertices.push_back(y);
            mesh.vertices.push_back(z);
            mesh.vertices.push_back(x);
            mesh.vertices.push_back(y);
            mesh.vertices.push_back(z);
        }
    }
    for (std::uint32_t stack = 0; stack < stacks; ++stack)
    {
        for (std::uint32_t slice = 0; slice < slices; ++slice)
        {
            const std::uint32_t row = stack * (slices + 1);
            const std::uint32_t a = row + slice;
            const std::uint32_t b = a + slices + 1;
            const std::array<std::uint32_t, 6> quad = {a, b, a + 1u, a + 1u, b, b + 1u};
            mesh.indices.insert(mesh.indices.end(), quad.begin(), quad.end());
        }
    }
    return mesh;
}

} // namespace engine
