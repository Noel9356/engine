#include "gi_demo_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace demo::lumen_gi
{
using namespace engine;
namespace renderer = engine::renderer;
namespace lumen_gi = engine::lumen_gi;

// One axis-aligned quad (two triangles) with an outward normal.
void appendQuad(SceneMesh &mesh, const glm::vec3 &origin, const glm::vec3 &edgeU, const glm::vec3 &edgeV,
    const glm::vec3 &normal)
{
    const std::uint32_t base    = static_cast<std::uint32_t>(mesh.vertices.size());
    const glm::vec3     corners[4] = {origin, origin + edgeU, origin + edgeU + edgeV, origin + edgeV};
    for (const glm::vec3 &corner : corners)
    {
        mesh.vertices.push_back(SceneVertex{corner, normal, glm::vec3(1.0f), glm::vec2(0.0f)});
    }
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

// Axis-aligned box made of six outward-facing quads. Winding is CCW seen
// from outside (edgeU x edgeV = outward normal), matching the renderer's
// default back-face culling.
void appendBox(SceneMesh &mesh, const glm::vec3 &center, const glm::vec3 &halfExtent)
{
    const glm::vec3 minimum = center - halfExtent;
    const glm::vec3 maximum = center + halfExtent;
    const glm::vec3 ux(halfExtent.x * 2.0f, 0, 0);
    const glm::vec3 uy(0, halfExtent.y * 2.0f, 0);
    const glm::vec3 uz(0, 0, halfExtent.z * 2.0f);
    appendQuad(mesh, {minimum.x, minimum.y, maximum.z}, ux, uy, {0, 0, 1});    // front (+z)
    appendQuad(mesh, {maximum.x, minimum.y, minimum.z}, -ux, uy, {0, 0, -1});  // back (-z)
    appendQuad(mesh, {maximum.x, minimum.y, maximum.z}, -uz, uy, {1, 0, 0});   // right (+x)
    appendQuad(mesh, {minimum.x, minimum.y, minimum.z}, uz, uy, {-1, 0, 0});   // left (-x)
    appendQuad(mesh, {minimum.x, maximum.y, maximum.z}, ux, -uz, {0, 1, 0});   // top (+y)
    appendQuad(mesh, {minimum.x, minimum.y, minimum.z}, ux, uz, {0, -1, 0});   // bottom (-y)
}

void appendSphere(SceneMesh &mesh, const glm::vec3 &center, float radius, std::uint32_t slices, std::uint32_t stacks)
{
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
    for (std::uint32_t stack = 0; stack <= stacks; ++stack)
    {
        const float phi     = 3.141592654f * static_cast<float>(stack) / static_cast<float>(stacks);
        const float sinePhi = std::sin(phi);
        for (std::uint32_t slice = 0; slice <= slices; ++slice)
        {
            const float theta = 6.283185307f * static_cast<float>(slice) / static_cast<float>(slices);
            const glm::vec3 normal(sinePhi * std::cos(theta), std::cos(phi), sinePhi * std::sin(theta));
            SceneVertex vertex;
            vertex.position = center + normal * radius;
            vertex.normal   = normal;
            mesh.vertices.push_back(vertex);
        }
    }
    for (std::uint32_t stack = 0; stack < stacks; ++stack)
    {
        for (std::uint32_t slice = 0; slice < slices; ++slice)
        {
            const std::uint32_t row = base + stack * (slices + 1);
            mesh.indices.insert(mesh.indices.end(),
                {row + slice, row + slice + 1, row + slices + 1 + slice + 1, row + slice,
                    row + slices + 1 + slice + 1, row + slices + 1 + slice});
        }
    }
}

// ---- scenes -----------------------------------------------------------------

// Cornell-style dark room, 4 m wide / 3 m tall / 4 m deep, camera inside.
// Light reaches every surface only through the two emissive ceiling panels;
// the white occluder picks up red/green color bleeding from the side walls.
std::vector<SceneMesh> buildInteriorScene()
{
    std::vector<SceneMesh> scene;

    SceneMesh room;
    appendBox(room, {0.0f, -0.05f, 0.0f}, {2.2f, 0.05f, 2.2f});   // floor
    appendBox(room, {0.0f, 3.05f, 0.0f}, {2.2f, 0.05f, 2.2f});    // ceiling
    appendBox(room, {0.0f, 1.5f, -2.05f}, {2.2f, 1.5f, 0.05f});   // back wall
    appendBox(room, {0.0f, 1.5f, 2.05f}, {2.2f, 1.5f, 0.05f});    // front wall (behind camera)
    for (SceneVertex &vertex : room.vertices)
    {
        vertex.color = glm::vec3(0.9f);
    }
    room.material.baseColor[0] = 0.9f;
    room.material.baseColor[1] = 0.9f;
    room.material.baseColor[2] = 0.9f;
    room.giAlbedo[0]           = 0.9f;
    room.giAlbedo[1]           = 0.9f;
    room.giAlbedo[2]           = 0.9f;
    scene.push_back(std::move(room));

    SceneMesh leftWall;
    appendBox(leftWall, {-2.05f, 1.5f, 0.0f}, {0.05f, 1.5f, 2.2f});
    leftWall.material.baseColor[0] = 0.85f;   // deep red
    leftWall.material.baseColor[1] = 0.10f;
    leftWall.material.baseColor[2] = 0.10f;
    leftWall.giAlbedo[0]           = 0.85f;
    leftWall.giAlbedo[1]           = 0.10f;
    leftWall.giAlbedo[2]           = 0.10f;
    scene.push_back(std::move(leftWall));

    SceneMesh rightWall;
    appendBox(rightWall, {2.05f, 1.5f, 0.0f}, {0.05f, 1.5f, 2.2f});
    rightWall.material.baseColor[0] = 0.10f;  // deep green
    rightWall.material.baseColor[1] = 0.85f;
    rightWall.material.baseColor[2] = 0.10f;
    rightWall.giAlbedo[0]           = 0.10f;
    rightWall.giAlbedo[1]           = 0.85f;
    rightWall.giAlbedo[2]           = 0.10f;
    scene.push_back(std::move(rightWall));

    SceneMesh occluder;
    appendBox(occluder, {-0.45f, 0.6f, -0.5f}, {0.4f, 0.6f, 0.4f}); // lifted off the floor to avoid z-fighting
    for (SceneVertex &vertex : occluder.vertices)
    {
        vertex.color = glm::vec3(0.92f);
    }
    occluder.material.baseColor[0] = 0.92f;
    occluder.material.baseColor[1] = 0.92f;
    occluder.material.baseColor[2] = 0.92f;
    occluder.giAlbedo[0]           = 0.92f;
    occluder.giAlbedo[1]           = 0.92f;
    occluder.giAlbedo[2]           = 0.92f;
    scene.push_back(std::move(occluder));

    // Two emissive ceiling panels: the only light sources in the room.
    SceneMesh lamps;
    appendQuad(lamps, {-1.3f, 2.98f, -1.1f}, {1.0f, 0, 0}, {0, 0, 1.4f}, {0, -1, 0});
    appendQuad(lamps, {0.3f, 2.98f, -1.1f}, {1.0f, 0, 0}, {0, 0, 1.4f}, {0, -1, 0});
    lamps.material.emissiveColor[0] = 7.0f;
    lamps.material.emissiveColor[1] = 6.8f;
    lamps.material.emissiveColor[2] = 6.2f;
    lamps.giEmissive[0]             = 7.0f;
    lamps.giEmissive[1]             = 6.8f;
    lamps.giEmissive[2]             = 6.2f;
    scene.push_back(std::move(lamps));

    return scene;
}

// Sunlit outdoor block field: ground, colored blocks, unlit sky dome + sun
// disc (both excluded from GI tracing - the probe cache answers sky rays with
// the hemispheric skylight).
std::vector<SceneMesh> buildOutdoorScene()
{
    std::vector<SceneMesh> scene;

    SceneMesh ground;
    appendBox(ground, {0.0f, -0.05f, 0.0f}, {20.0f, 0.05f, 20.0f});
    ground.material.baseColor[0] = 0.55f;
    ground.material.baseColor[1] = 0.58f;
    ground.material.baseColor[2] = 0.52f;
    ground.giAlbedo[0]           = 0.55f;
    ground.giAlbedo[1]           = 0.58f;
    ground.giAlbedo[2]           = 0.52f;
    scene.push_back(std::move(ground));

    struct Block
    {
        glm::vec3 center;
        glm::vec3 halfExtent;
        glm::vec3 albedo;
    };
    const Block blocks[] = {
        {{-3.0f, 0.75f, -2.0f}, {0.75f, 0.75f, 0.75f}, {0.80f, 0.16f, 0.14f}},  // red cube
        {{3.0f, 1.5f, -1.0f}, {1.0f, 1.5f, 1.0f}, {0.16f, 0.70f, 0.24f}},       // green slab
        {{0.0f, 2.0f, -5.0f}, {0.5f, 2.0f, 0.5f}, {0.20f, 0.36f, 0.78f}},       // blue column
        {{-1.5f, 1.25f, -8.0f}, {1.25f, 1.25f, 1.25f}, {0.90f, 0.88f, 0.84f}},  // white cube
        {{4.5f, 0.6f, 2.5f}, {0.6f, 0.6f, 0.6f}, {0.85f, 0.45f, 0.10f}},        // orange cube
    };
    for (const Block &block : blocks)
    {
        SceneMesh mesh;
        appendBox(mesh, block.center, block.halfExtent);
        mesh.material.baseColor[0] = block.albedo.r;
        mesh.material.baseColor[1] = block.albedo.g;
        mesh.material.baseColor[2] = block.albedo.b;
        mesh.giAlbedo[0]           = block.albedo.r;
        mesh.giAlbedo[1]           = block.albedo.g;
        mesh.giAlbedo[2]           = block.albedo.b;
        scene.push_back(std::move(mesh));
    }

    // Unlit sky dome + sun disc: the renderer shows raw albedo/emission, GI
    // ignores them (excluded) and answers those rays with the skylight.
    SceneMesh skyDome;
    appendSphere(skyDome, {0.0f, 0.0f, 0.0f}, 60.0f, 48, 24);
    skyDome.material.unlit        = true;
    skyDome.material.baseColor[0] = 0.55f;
    skyDome.material.baseColor[1] = 0.65f;
    skyDome.material.baseColor[2] = 0.80f;
    skyDome.giFlagBits            = lumen_gi::giSurfaceFlagExcluded;
    scene.push_back(std::move(skyDome));

    SceneMesh sunDisc;
    const glm::vec3 sunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
    appendSphere(sunDisc, sunDirection * 55.0f, 2.2f, 24, 12);
    sunDisc.material.unlit            = true;
    sunDisc.material.emissiveColor[0] = 12.0f;
    sunDisc.material.emissiveColor[1] = 11.5f;
    sunDisc.material.emissiveColor[2] = 10.5f;
    sunDisc.giFlagBits                = lumen_gi::giSurfaceFlagExcluded;
    scene.push_back(std::move(sunDisc));

    return scene;
}

} // namespace demo::lumen_gi
