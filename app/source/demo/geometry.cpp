// geometry.cpp - procedural mesh builders for the demo scene.
//
// All builders emit indexed triangles in the renderer's fixed 11-float vertex
// layout; degenerate parameter sets return empty meshes instead of throwing.

#include "geometry.hpp"

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace demo
{

// View of the vertex buffer for GPU upload. Relies on glm vectors being
// tightly packed so the 11-float-per-vertex layout the renderer assumes holds.
engine::render::MeshDescription MeshData::description() const
{
    return engine::render::MeshDescription{
        .vertices    = reinterpret_cast<const float *>(vertices.data()),
        .vertexCount = static_cast<std::uint32_t>(vertices.size()),
        .indices     = indices.data(),
        .indexCount  = static_cast<std::uint32_t>(indices.size())};
}

MeshData makeBoxMesh(const glm::vec3 &center, const glm::vec3 &halfExtent, const glm::vec3 &color, float uvScale)
{
    // One face = outward normal plus four corners wound counter-clockwise as
    // seen from outside, so the shared two-triangle index pattern is front-facing.
    struct Face
    {
        glm::vec3 normal;
        glm::vec3 corners[4];
    };

    const Face faces[6] = {
        {.normal = {1, 0, 0},
            .corners
            = {{center.x + halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z},
                {center.x + halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z}}},
        {.normal = {-1, 0, 0},
            .corners
            = {{center.x - halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x - halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z},
                {center.x - halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z},
                {center.x - halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z}}},
        {.normal = {0, 1, 0},
            .corners
            = {{center.x - halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z},
                {center.x - halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z}}},
        {.normal = {0, -1, 0},
            .corners
            = {{center.x - halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x + halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x + halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z},
                {center.x - halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z}}},
        {.normal = {0, 0, 1},
            .corners
            = {{center.x - halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z},
                {center.x + halfExtent.x, center.y - halfExtent.y, center.z + halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z},
                {center.x - halfExtent.x, center.y + halfExtent.y, center.z + halfExtent.z}}},
        {.normal = {0, 0, -1},
            .corners
            = {{center.x + halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x - halfExtent.x, center.y - halfExtent.y, center.z - halfExtent.z},
                {center.x - halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z},
                {center.x + halfExtent.x, center.y + halfExtent.y, center.z - halfExtent.z}}},
    };

    const glm::vec2 faceUvs[4] = {{0.0f, 1.0f}, {uvScale, 1.0f}, {uvScale, 0.0f}, {0.0f, 0.0f}};

    MeshData mesh;
    for (const Face &face : faces)
    {
        const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (int corner = 0; corner < 4; ++corner)
        {
            mesh.vertices.push_back(
                Vertex{.position = face.corners[corner], .normal = face.normal, .color = color, .uv = faceUvs[corner]});
        }
        mesh.indices.insert(mesh.indices.end(), {base + 0, base + 1, base + 2, base + 0, base + 2, base + 3});
    }
    return mesh;
}

MeshData makeSphereMesh(const glm::vec3 &center, float radius, std::uint32_t segments, std::uint32_t rings,
    const glm::vec3 &equatorColor, const glm::vec3 &zenithColor)
{
    // Vertices are stored pole first (index 0), then rings-1 latitude rings of
    // `segments` vertices, then the bottom pole; indexing below depends on it.
    MeshData mesh;

    mesh.vertices.push_back(Vertex{
        .position = center + glm::vec3(0, radius, 0), .normal = {0, 1, 0}, .color = zenithColor, .uv = {0.0f, 0.0f}});
    for (std::uint32_t ring = 1; ring < rings; ++ring)
    {
        const float phi        = glm::pi<float>() * static_cast<float>(ring) / static_cast<float>(rings);
        const float height     = std::cos(phi);
        const float ringRadius = std::sin(phi);

        const float     blend = std::clamp(height, 0.0f, 1.0f); // 0 at equator, 1 at top pole.
        const glm::vec3 color = glm::mix(equatorColor, zenithColor, blend);
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const float theta = 2.0f * glm::pi<float>() * static_cast<float>(segment) / static_cast<float>(segments);
            const glm::vec3 normal(std::cos(theta) * ringRadius, height, std::sin(theta) * ringRadius);
            mesh.vertices.push_back(
                Vertex{.position = center + normal * radius, .normal = normal, .color = color, .uv = {0.0f, 0.0f}});
        }
    }

    const std::uint32_t bottomPole = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(Vertex{
        .position = center - glm::vec3(0, radius, 0), .normal = {0, -1, 0}, .color = equatorColor, .uv = {0.0f, 0.0f}});

    const std::uint32_t firstRing = 1;
    for (std::uint32_t segment = 0; segment < segments; ++segment)
    {
        const std::uint32_t next = (segment + 1) % segments;
        mesh.indices.push_back(0);
        mesh.indices.push_back(firstRing + segment);
        mesh.indices.push_back(firstRing + next);
    }
    for (std::uint32_t ring = firstRing; ring + 1 < rings; ++ring)
    {
        const std::uint32_t ringA = firstRing + (ring - firstRing) * segments;
        const std::uint32_t ringB = ringA + segments;
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const std::uint32_t next = (segment + 1) % segments;
            mesh.indices.insert(mesh.indices.end(),
                {ringA + segment, ringB + segment, ringB + next, ringA + segment, ringB + next, ringA + next});
        }
    }
    const std::uint32_t lastRing = firstRing + (rings - firstRing - 1) * segments;
    for (std::uint32_t segment = 0; segment < segments; ++segment)
    {
        const std::uint32_t next = (segment + 1) % segments;
        mesh.indices.push_back(lastRing + segment);
        mesh.indices.push_back(bottomPole);
        mesh.indices.push_back(lastRing + next);
    }
    return mesh;
}

MeshData makeRockMesh(float radius, std::uint32_t segments, std::uint32_t rings, const glm::vec3 &baseColor)
{
    MeshData mesh;
    if (segments < 3 || rings < 2 || radius <= 0.0f)
    {
        return mesh;
    }

    // Same UV-sphere topology as makeSphereMesh, but each vertex is pushed along
    // its direction by layered sine noise (detailA/detailB) and tinted by a
    // height-based shade so the silhouette and lighting read as rock.
    mesh.vertices.reserve(2 + static_cast<std::size_t>(rings - 1) * segments);
    mesh.indices.reserve(static_cast<std::size_t>(segments) * (rings - 1) * 6);

    const auto appendVertex = [&](const glm::vec3 &direction, float u, float v)
    {
        const float detailA
            = std::sin(direction.x * 11.0f + direction.z * 7.0f) * std::sin(direction.y * 13.0f - direction.z * 5.0f);
        const float detailB
            = std::cos(direction.x * 23.0f + direction.y * 17.0f) * std::sin(direction.z * 19.0f - direction.y * 9.0f);
        const float     radialScale = 1.0f + detailA * 0.075f + detailB * 0.025f;
        const glm::vec3 normal      = glm::normalize(glm::vec3(direction.x * (1.0f + detailA * 0.10f),
            direction.y * (1.0f + detailA * 0.06f),
            direction.z * (1.0f + detailA * 0.10f)));
        const float     shade = 0.78f + 0.22f * std::clamp(direction.y * 0.5f + 0.5f, 0.0f, 1.0f) + detailA * 0.06f;
        mesh.vertices.push_back(Vertex{
            .position = direction * (radius * radialScale),
            .normal   = normal,
            .color    = baseColor * shade,
            .uv       = {u, v}});
    };

    appendVertex({0.0f, 1.0f, 0.0f}, 0.0f, 0.0f);
    for (std::uint32_t ring = 1; ring < rings; ++ring)
    {
        const float phi        = glm::pi<float>() * static_cast<float>(ring) / static_cast<float>(rings);
        const float y          = std::cos(phi);
        const float ringRadius = std::sin(phi);
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const float theta = 2.0f * glm::pi<float>() * static_cast<float>(segment) / static_cast<float>(segments);
            appendVertex({std::cos(theta) * ringRadius, y, std::sin(theta) * ringRadius},
                static_cast<float>(segment) / static_cast<float>(segments),
                static_cast<float>(ring) / static_cast<float>(rings));
        }
    }
    const std::uint32_t bottomPole = static_cast<std::uint32_t>(mesh.vertices.size());
    appendVertex({0.0f, -1.0f, 0.0f}, 0.0f, 1.0f);

    for (std::uint32_t segment = 0; segment < segments; ++segment)
    {
        const std::uint32_t next = (segment + 1) % segments;
        mesh.indices.insert(mesh.indices.end(), {0, 1 + segment, 1 + next});
    }
    for (std::uint32_t ring = 0; ring + 2 < rings; ++ring)
    {
        const std::uint32_t ringA = 1 + ring * segments;
        const std::uint32_t ringB = ringA + segments;
        for (std::uint32_t segment = 0; segment < segments; ++segment)
        {
            const std::uint32_t next = (segment + 1) % segments;
            mesh.indices.insert(mesh.indices.end(),
                {ringA + segment, ringB + segment, ringB + next, ringA + segment, ringB + next, ringA + next});
        }
    }
    const std::uint32_t lastRing = 1 + (rings - 2) * segments;
    for (std::uint32_t segment = 0; segment < segments; ++segment)
    {
        const std::uint32_t next = (segment + 1) % segments;
        mesh.indices.insert(mesh.indices.end(), {lastRing + segment, bottomPole, lastRing + next});
    }
    return mesh;
}

MeshData makeTorusMesh(float majorRadius, float minorRadius, std::uint32_t majorSegments, std::uint32_t minorSegments,
    const glm::vec3 &color)
{
    MeshData mesh;
    if (majorSegments < 3 || minorSegments < 3 || majorRadius <= 0.0f || minorRadius <= 0.0f)
    {
        return mesh;
    }

    // Standard torus parameterization: u around the ring (Y axis), v around the
    // tube; vertices are laid out row-major so quads connect adjacent rows.
    const std::size_t vertexCount = static_cast<std::size_t>(majorSegments) * minorSegments;
    mesh.vertices.reserve(vertexCount);
    mesh.indices.reserve(vertexCount * 6);
    for (std::uint32_t major = 0; major < majorSegments; ++major)
    {
        const float u    = 2.0f * glm::pi<float>() * static_cast<float>(major) / static_cast<float>(majorSegments);
        const float cosU = std::cos(u);
        const float sinU = std::sin(u);
        for (std::uint32_t minor = 0; minor < minorSegments; ++minor)
        {
            const float     v = 2.0f * glm::pi<float>() * static_cast<float>(minor) / static_cast<float>(minorSegments);
            const float     cosV = std::cos(v);
            const float     sinV = std::sin(v);
            const glm::vec3 normal(cosV * cosU, sinV, cosV * sinU);
            const glm::vec3 position((majorRadius + minorRadius * cosV) * cosU,
                minorRadius * sinV,
                (majorRadius + minorRadius * cosV) * sinU);
            const float     shade = 0.88f + 0.12f * (0.5f + 0.5f * std::sin(u * 3.0f + v * 2.0f));
            mesh.vertices.push_back(Vertex{
                .position = position,
                .normal   = normal,
                .color    = color * shade,
                .uv
                = {static_cast<float>(major) / static_cast<float>(majorSegments),
                    static_cast<float>(minor) / static_cast<float>(minorSegments)}});
        }
    }
    for (std::uint32_t major = 0; major < majorSegments; ++major)
    {
        const std::uint32_t nextMajor = (major + 1) % majorSegments;
        for (std::uint32_t minor = 0; minor < minorSegments; ++minor)
        {
            const std::uint32_t nextMinor = (minor + 1) % minorSegments;
            const std::uint32_t a         = major * minorSegments + minor;
            const std::uint32_t b         = nextMajor * minorSegments + minor;
            const std::uint32_t c         = nextMajor * minorSegments + nextMinor;
            const std::uint32_t d         = major * minorSegments + nextMinor;
            mesh.indices.insert(mesh.indices.end(), {a, d, b, b, d, c});
        }
    }
    return mesh;
}

} // namespace demo
