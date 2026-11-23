#include <engine/lumen_gi/capabilities.hpp>
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
// Three reflection surfaces exercise the lumen_reflection pipeline's bands:
// a polished-metal floor inlay (roughness 0.05 - the dedicated-ray band), a
// chrome sphere on it (roughness 0.15 - dedicated rays), and a brushed-metal
// back-wall panel (roughness 0.35 - the radiance-cache band between
// ray_band_max and max_trace). Three transparent/translucent props at the
// end of the list exercise the transmission paths end to end: a clear glass
// sphere (dedicated-band refraction through a solid), a frosted pane (the
// radiance-cache band plus a glowing, soft-shadowing translucent body), and
// a tinted ruby cube (colored transmission). The inlay is metallic on
// purpose: its F0 is
// the base color (0.72), so the mirrored room renders at full mirror
// strength instead of the 4% a dark dielectric would bounce back - a
// dielectric slab reads as a near-black rectangle at the camera's shallow
// floor angle, defeating the surface's purpose as a reflection test.
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
    room.name = "room";
    scene.push_back(std::move(room));

    SceneMesh leftWall;
    appendBox(leftWall, {-2.05f, 1.5f, 0.0f}, {0.05f, 1.5f, 2.2f});
    leftWall.material.baseColor[0] = 0.85f;   // deep red
    leftWall.material.baseColor[1] = 0.10f;
    leftWall.material.baseColor[2] = 0.10f;
    leftWall.giAlbedo[0]           = 0.85f;
    leftWall.giAlbedo[1]           = 0.10f;
    leftWall.giAlbedo[2]           = 0.10f;
    leftWall.name = "left wall";
    scene.push_back(std::move(leftWall));

    SceneMesh rightWall;
    appendBox(rightWall, {2.05f, 1.5f, 0.0f}, {0.05f, 1.5f, 2.2f});
    rightWall.material.baseColor[0] = 0.10f;  // deep green
    rightWall.material.baseColor[1] = 0.85f;
    rightWall.material.baseColor[2] = 0.10f;
    rightWall.giAlbedo[0]           = 0.10f;
    rightWall.giAlbedo[1]           = 0.85f;
    rightWall.giAlbedo[2]           = 0.10f;
    rightWall.name = "right wall";
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
    occluder.name = "occluder";
    scene.push_back(std::move(occluder));

    // Polished-metal floor inlay: a thin silver slab beside the occluder
    // (clear of it, so no z-fighting) whose near-zero roughness sits in the
    // dedicated reflection-ray band. Metallic 1 makes the specular F0 the
    // 0.72 base color - the inlay answers with a full-strength mirror image
    // of the room (walls, sphere, lamps). The GI albedo stays near zero: a
    // metal carries almost no diffuse, so the room's bounce lighting barely
    // notices the slab. Vertex colors stay white; both the renderer and the
    // lumen_reflection module multiply the base color by them.
    SceneMesh mirrorFloor;
    appendBox(mirrorFloor, {0.55f, 0.012f, -0.35f}, {0.45f, 0.012f, 0.9f});
    mirrorFloor.material.baseColor[0] = 0.72f;
    mirrorFloor.material.baseColor[1] = 0.73f;
    mirrorFloor.material.baseColor[2] = 0.75f;
    mirrorFloor.material.roughnessFactor = 0.05f;
    mirrorFloor.material.metallicFactor  = 1.0f;
    mirrorFloor.giAlbedo[0]           = 0.05f;
    mirrorFloor.giAlbedo[1]           = 0.05f;
    mirrorFloor.giAlbedo[2]           = 0.055f;
    mirrorFloor.rfRoughness           = 0.05f;
    mirrorFloor.rfMetallic            = 1.0f;
    mirrorFloor.name = "mirror floor";
    scene.push_back(std::move(mirrorFloor));

    // Chrome sphere sitting on the inlay: reflects the red/green walls, the
    // ceiling lamps, and the occluder back to the camera. Vertex colors stay
    // white so the 0.72 base color passes through both the renderer and the
    // reflection module's albedo (= base color x vertex color) untouched.
    // The GI albedo follows the same metal rule the mirror floor documents: a
    // metal carries almost no diffuse, so the world radiance cache must not
    // treat the sphere as a white diffuse ball (its brightness reaches the
    // room through the reflection field, not through the bounce lighting).
    SceneMesh chromeSphere;
    appendSphere(chromeSphere, {0.55f, 0.34f, -0.45f}, 0.34f, 48, 24);
    chromeSphere.material.baseColor[0] = 0.72f;
    chromeSphere.material.baseColor[1] = 0.73f;
    chromeSphere.material.baseColor[2] = 0.75f;
    chromeSphere.material.roughnessFactor = 0.15f;
    chromeSphere.material.metallicFactor  = 1.0f;
    chromeSphere.giAlbedo[0]           = 0.05f;
    chromeSphere.giAlbedo[1]           = 0.05f;
    chromeSphere.giAlbedo[2]           = 0.055f;
    chromeSphere.rfRoughness           = 0.15f;
    chromeSphere.rfMetallic            = 1.0f;
    chromeSphere.name = "chrome sphere";
    scene.push_back(std::move(chromeSphere));

    // Brushed-metal panel on the back wall: roughness inside the
    // radiance-cache band (no dedicated rays; the rough lobe resamples the
    // lumen_gi world cache).
    SceneMesh brushedPanel;
    appendBox(brushedPanel, {0.95f, 1.5f, -1.985f}, {0.55f, 0.85f, 0.012f});
    brushedPanel.material.baseColor[0] = 0.65f;
    brushedPanel.material.baseColor[1] = 0.6f;
    brushedPanel.material.baseColor[2] = 0.55f;
    brushedPanel.material.roughnessFactor = 0.35f;
    brushedPanel.material.metallicFactor  = 1.0f;
    brushedPanel.giAlbedo[0]           = 0.65f;
    brushedPanel.giAlbedo[1]           = 0.6f;
    brushedPanel.giAlbedo[2]           = 0.55f;
    brushedPanel.rfRoughness           = 0.35f;
    brushedPanel.rfMetallic            = 1.0f;
    brushedPanel.name = "brushed panel";
    scene.push_back(std::move(brushedPanel));

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
    lamps.name = "lamps";
    scene.push_back(std::move(lamps));

    // Transparent and translucent props exercising the transmission paths of
    // the renderer's transparent pass, lumen_reflection (refracted chains),
    // and lumen_gi (pass-through gathers, probe transmittance, softened
    // shadows). They are pushed last and drawn last: the renderer composites
    // transparent draws after the opaque scene (far to near within the
    // group), so the ordering here is the compositing order.
    //
    // Clear glass sphere: near-transparent dielectric whose refraction bends
    // the room behind it (the reflection field's transmitted share traces
    // through the medium with Snell's law). The GI albedo stays near zero
    // like the other dielectric-and-metal surfaces: what the room receives
    // from the sphere is its specular response, not a diffuse bounce.
    SceneMesh glassSphere;
    appendSphere(glassSphere, {-1.45f, 0.36f, -0.7f}, 0.36f, 48, 24);
    glassSphere.material.baseColor[0] = 0.96f;
    glassSphere.material.baseColor[1] = 0.98f;
    glassSphere.material.baseColor[2] = 0.97f;
    glassSphere.material.roughnessFactor = 0.06f;
    glassSphere.material.metallicFactor  = 0.0f;
    glassSphere.material.opacity           = 0.04f;
    glassSphere.material.indexOfRefraction = 1.5f;
    glassSphere.giAlbedo[0]           = 0.05f;
    glassSphere.giAlbedo[1]           = 0.05f;
    glassSphere.giAlbedo[2]           = 0.055f;
    glassSphere.rfRoughness           = 0.06f;
    glassSphere.rfMetallic            = 0.0f;
    glassSphere.name = "glass sphere";
    scene.push_back(std::move(glassSphere));

    // Frosted pane: a translucent slab whose rough surface blurs everything
    // behind it while its own body scatters lamp light (the GI albedo is
    // deliberately high: a translucent pane glows with the light it
    // scatters, and the shadow it casts stays soft).
    SceneMesh frostedPane;
    appendBox(frostedPane, {1.5f, 1.05f, -0.25f}, {0.02f, 0.5f, 0.4f});
    frostedPane.material.baseColor[0] = 0.92f;
    frostedPane.material.baseColor[1] = 0.88f;
    frostedPane.material.baseColor[2] = 0.82f;
    frostedPane.material.roughnessFactor = 0.3f;
    frostedPane.material.metallicFactor  = 0.0f;
    frostedPane.material.opacity           = 0.55f;
    frostedPane.material.indexOfRefraction = 1.45f;
    frostedPane.giAlbedo[0]           = 0.55f;
    frostedPane.giAlbedo[1]           = 0.52f;
    frostedPane.giAlbedo[2]           = 0.48f;
    frostedPane.rfRoughness           = 0.3f;
    frostedPane.rfMetallic            = 0.0f;
    frostedPane.name = "frosted pane";
    scene.push_back(std::move(frostedPane));

    // Ruby glass cube: tinted transparent solid - the refraction through it
    // picks up the color (the transmitted share is tinted by the base color),
    // and it casts a faint red-tinted soft shadow.
    SceneMesh rubyCube;
    appendBox(rubyCube, {0.02f, 0.13f, -1.35f}, {0.13f, 0.13f, 0.13f});
    rubyCube.material.baseColor[0] = 0.72f;
    rubyCube.material.baseColor[1] = 0.08f;
    rubyCube.material.baseColor[2] = 0.10f;
    rubyCube.material.roughnessFactor = 0.05f;
    rubyCube.material.metallicFactor  = 0.0f;
    rubyCube.material.opacity           = 0.15f;
    rubyCube.material.indexOfRefraction = 1.5f;
    rubyCube.giAlbedo[0]           = 0.05f;
    rubyCube.giAlbedo[1]           = 0.01f;
    rubyCube.giAlbedo[2]           = 0.015f;
    rubyCube.rfRoughness           = 0.05f;
    rubyCube.rfMetallic            = 0.0f;
    rubyCube.name = "ruby cube";
    scene.push_back(std::move(rubyCube));

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
    ground.name = "ground";
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

    // Chrome sphere on the open field: reflects the sky gradient, the
    // colored blocks, and the ground (the sun disc is excluded geometry -
    // its rays answer the skylight, the engine-wide rule the GI module
    // follows too).
    SceneMesh chromeSphere;
    appendSphere(chromeSphere, {2.2f, 0.55f, 2.0f}, 0.55f, 48, 24);
    chromeSphere.material.baseColor[0] = 0.8f;
    chromeSphere.material.baseColor[1] = 0.82f;
    chromeSphere.material.baseColor[2] = 0.85f;
    chromeSphere.material.roughnessFactor = 0.12f;
    chromeSphere.material.metallicFactor  = 1.0f;
    chromeSphere.giAlbedo[0]           = 0.8f;
    chromeSphere.giAlbedo[1]           = 0.82f;
    chromeSphere.giAlbedo[2]           = 0.85f;
    chromeSphere.rfRoughness           = 0.12f;
    chromeSphere.rfMetallic            = 1.0f;
    chromeSphere.name = "chrome sphere";
    scene.push_back(std::move(chromeSphere));

    // Polished water-like slab in the foreground: near-mirror dielectric.
    // Water keeps its 4% F0 on purpose - the grazing view angle drives the
    // Fresnel term up, and the sky it mirrors comes from the environment's
    // analytic skylight, which the sunlit profile calibrates to the visual
    // sky dome's radiance.
    SceneMesh waterSlab;
    appendBox(waterSlab, {-1.2f, 0.012f, 1.8f}, {1.2f, 0.012f, 1.0f});
    waterSlab.material.baseColor[0] = 0.04f;
    waterSlab.material.baseColor[1] = 0.05f;
    waterSlab.material.baseColor[2] = 0.06f;
    waterSlab.material.roughnessFactor = 0.08f;
    waterSlab.giAlbedo[0]           = 0.04f;
    waterSlab.giAlbedo[1]           = 0.05f;
    waterSlab.giAlbedo[2]           = 0.06f;
    waterSlab.rfRoughness           = 0.08f;
    waterSlab.rfMetallic            = 0.0f;
    waterSlab.name = "water slab";
    scene.push_back(std::move(waterSlab));

    // Crystal ball next to the water: a curved dielectric the sun shines
    // through, so the field shows focused caustics on the ground beside it
    // (the photon pass deposits them into the surface cache; the probes
    // carry the soft remainder). Kept clear and small so the effect reads as
    // light, not as a prop.
    SceneMesh crystalBall;
    appendSphere(crystalBall, {-3.6f, 0.45f, 0.9f}, 0.45f, 48, 24);
    crystalBall.material.baseColor[0] = 0.96f;
    crystalBall.material.baseColor[1] = 0.98f;
    crystalBall.material.baseColor[2] = 1.0f;
    // The renderer material carries the same near-mirror roughness the
    // reflection module traces with (rfRoughness below): a transparent
    // surface outside the traced roughness band composites through the
    // plain alpha blend, and a default-rough crystal ball would render as a
    // faint 5%-opacity smudge over the field's traced refraction response.
    crystalBall.material.roughnessFactor = 0.02f;
    crystalBall.material.opacity           = 0.05f;
    crystalBall.material.indexOfRefraction = 1.5f;
    crystalBall.giAlbedo[0]           = 0.96f;
    crystalBall.giAlbedo[1]           = 0.98f;
    crystalBall.giAlbedo[2]           = 1.0f;
    crystalBall.rfRoughness           = 0.02f;
    crystalBall.rfMetallic            = 0.0f;
    crystalBall.name = "crystal ball";
    scene.push_back(std::move(crystalBall));

    // Unlit sky dome + sun disc: the renderer shows raw albedo/emission, GI
    // ignores them (excluded) and answers those rays with the skylight.
    SceneMesh skyDome;
    appendSphere(skyDome, {0.0f, 0.0f, 0.0f}, 60.0f, 48, 24);
    skyDome.material.unlit        = true;
    skyDome.material.baseColor[0] = 0.55f;
    skyDome.material.baseColor[1] = 0.65f;
    skyDome.material.baseColor[2] = 0.80f;
    skyDome.giFlagBits            = lumen_gi::giSurfaceFlagExcluded;
    skyDome.name = "sky dome";
    scene.push_back(std::move(skyDome));

    SceneMesh sunDisc;
    const glm::vec3 sunDirection = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
    appendSphere(sunDisc, sunDirection * 55.0f, 2.2f, 24, 12);
    sunDisc.material.unlit            = true;
    sunDisc.material.emissiveColor[0] = 12.0f;
    sunDisc.material.emissiveColor[1] = 11.5f;
    sunDisc.material.emissiveColor[2] = 10.5f;
    sunDisc.giFlagBits                = lumen_gi::giSurfaceFlagExcluded;
    sunDisc.name = "sun disc";
    scene.push_back(std::move(sunDisc));

    return scene;
}

// Exhibition gallery: a 14 x 4.2 x 12 m hall with a raised platform and
// steps on the left, a half-height partition with a frosted glass pane across
// the middle, two columns, a polished-metal floor strip, and a reflecting
// pool under the main warm ceiling panel. The props cover every lighting and
// reflection band the two modules trace: chrome sphere and gold sphere
// (dedicated-ray metals), a glass sphere resting IN the pool and a ruby cube
// on the platform (transmission chains plus caustics under the emissive
// panels), a spinning chrome cube on the metal strip (dynamic surfaces),
// brushed-metal back-wall panels (the radiance-cache band), and stacked
// crates for multi-bounce color bleeding off the tinted side walls (cool
// blue left, warm terracotta right). All light comes from the emissive
// panels: one large warm panel over the pool, two cool cove strips along the
// ceiling edges, and a warm sconce on the back wall between the brushed
// panels - three color temperatures for the GI to mix.
std::vector<SceneMesh> buildGalleryScene()
{
    std::vector<SceneMesh> scene;

    auto pushBox = [&scene](const char *name, const glm::vec3 &center, const glm::vec3 &halfExtent,
        const glm::vec3 &albedo) -> SceneMesh &
    {
        SceneMesh mesh;
        appendBox(mesh, center, halfExtent);
        for (SceneVertex &vertex : mesh.vertices)
        {
            vertex.color = albedo;
        }
        mesh.material.baseColor[0] = albedo.r;
        mesh.material.baseColor[1] = albedo.g;
        mesh.material.baseColor[2] = albedo.b;
        mesh.giAlbedo[0]           = albedo.r;
        mesh.giAlbedo[1]           = albedo.g;
        mesh.giAlbedo[2]           = albedo.b;
        mesh.name = name;
        scene.push_back(std::move(mesh));
        return scene.back();
    };

    // Shell: floor, ceiling, back and front walls.
    pushBox("floor", {0.0f, -0.05f, 0.0f}, {7.0f, 0.05f, 6.0f}, glm::vec3(0.60f, 0.57f, 0.54f));
    pushBox("ceiling", {0.0f, 4.25f, 0.0f}, {7.0f, 0.05f, 6.0f}, glm::vec3(0.88f, 0.87f, 0.85f));
    pushBox("back wall", {0.0f, 2.1f, -6.05f}, {7.0f, 2.1f, 0.05f}, glm::vec3(0.82f, 0.80f, 0.77f));
    pushBox("front wall", {0.0f, 2.1f, 6.05f}, {7.0f, 2.1f, 0.05f}, glm::vec3(0.80f, 0.78f, 0.75f));
    // Tinted side walls (the GI bleeds both tints across the hall).
    pushBox("left wall", {-7.05f, 2.1f, 0.0f}, {0.05f, 2.1f, 6.0f}, glm::vec3(0.30f, 0.36f, 0.55f));
    pushBox("right wall", {7.05f, 2.1f, 0.0f}, {0.05f, 2.1f, 6.0f}, glm::vec3(0.62f, 0.42f, 0.30f));

    // Polished-metal floor strip across the hall's width: the full room
    // (lamps, spheres, walls) mirrored at the floor.
    {
        SceneMesh &strip = pushBox(
            "metal floor strip", {0.0f, 0.012f, -1.0f}, {7.0f, 0.012f, 0.28f}, glm::vec3(0.72f, 0.73f, 0.75f));
        strip.material.roughnessFactor = 0.05f;
        strip.material.metallicFactor  = 1.0f;
        strip.giAlbedo[0]           = 0.05f;
        strip.giAlbedo[1]           = 0.05f;
        strip.giAlbedo[2]           = 0.055f;
        strip.rfRoughness           = 0.05f;
        strip.rfMetallic            = 1.0f;
    }

    // Mid-hall half-height partition (two leaves with a gap the frosted pane
    // fills) and the columns flanking the back wall.
    pushBox("partition left", {-1.75f, 1.35f, -1.2f}, {1.75f, 1.35f, 0.1f}, glm::vec3(0.85f, 0.83f, 0.80f));
    pushBox("partition right", {2.9f, 1.35f, -1.2f}, {0.7f, 1.35f, 0.1f}, glm::vec3(0.85f, 0.83f, 0.80f));
    pushBox("column left", {-4.5f, 2.1f, -3.5f}, {0.35f, 2.1f, 0.35f}, glm::vec3(0.52f, 0.50f, 0.47f));
    pushBox("column right", {4.5f, 2.1f, -3.5f}, {0.35f, 2.1f, 0.35f}, glm::vec3(0.52f, 0.50f, 0.47f));

    // Left-back raised platform with three steps down toward the hall.
    pushBox("platform", {-4.2f, 0.35f, -4.2f}, {2.8f, 0.35f, 1.8f}, glm::vec3(0.66f, 0.63f, 0.58f));
    pushBox("step low", {-4.2f, 0.117f, -2.15f}, {1.4f, 0.117f, 0.25f}, glm::vec3(0.64f, 0.61f, 0.56f));
    pushBox("step mid", {-4.2f, 0.35f, -1.65f}, {1.4f, 0.35f, 0.25f}, glm::vec3(0.64f, 0.61f, 0.56f));
    pushBox("step high", {-4.2f, 0.583f, -1.15f}, {1.4f, 0.583f, 0.25f}, glm::vec3(0.64f, 0.61f, 0.56f));

    // Reflecting pool: a wide transmissive slab under the main ceiling panel.
    // The warm light passes through the water (and the glass sphere resting
    // in it) and the photon pass deposits the focused irradiance on the floor
    // texels beneath - the caustics showcase of the scene.
    {
        SceneMesh &pool = pushBox("pool water", {2.8f, 0.06f, -0.5f}, {1.6f, 0.06f, 1.5f}, glm::vec3(0.12f, 0.18f, 0.24f));
        pool.material.roughnessFactor = 0.06f;
        pool.material.metallicFactor  = 0.0f;
        pool.material.opacity           = 0.3f;
        pool.material.indexOfRefraction = 1.33f;
        pool.giAlbedo[0]           = 0.10f;
        pool.giAlbedo[1]           = 0.16f;
        pool.giAlbedo[2]           = 0.22f;
        pool.rfRoughness           = 0.06f;
        pool.rfMetallic            = 0.0f;
    }

    // Stacked crates on the right back corner.
    pushBox("crate large", {5.8f, 0.4f, -4.6f}, {0.4f, 0.4f, 0.4f}, glm::vec3(0.48f, 0.34f, 0.20f));
    pushBox("crate small", {5.8f, 1.05f, -4.6f}, {0.3f, 0.25f, 0.3f}, glm::vec3(0.42f, 0.30f, 0.18f));

    // Emissive panels: the hall's only light sources. One large warm panel
    // above the pool, two cool cove strips along the ceiling's side edges,
    // and a warm sconce on the back wall.
    SceneMesh lamps;
    appendQuad(lamps, {2.0f, 4.19f, -1.8f}, {1.6f, 0.0f, 0.0f}, {0.0f, 0.0f, 2.6f}, {0.0f, -1.0f, 0.0f});
    appendQuad(lamps, {-6.95f, 4.19f, -5.0f}, {0.3f, 0.0f, 0.0f}, {0.0f, 0.0f, 10.0f}, {0.0f, -1.0f, 0.0f});
    appendQuad(lamps, {6.65f, 4.19f, -5.0f}, {0.3f, 0.0f, 0.0f}, {0.0f, 0.0f, 10.0f}, {0.0f, -1.0f, 0.0f});
    appendQuad(lamps, {-0.9f, 2.25f, -5.93f}, {1.8f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 1.0f});
    lamps.material.emissiveColor[0] = 14.5f;
    lamps.material.emissiveColor[1] = 13.6f;
    lamps.material.emissiveColor[2] = 12.2f;
    lamps.giEmissive[0]             = 14.5f;
    lamps.giEmissive[1]             = 13.6f;
    lamps.giEmissive[2]             = 12.2f;
    lamps.name = "lamps";
    scene.push_back(std::move(lamps));

    // Brushed-metal back-wall panels flanking the sconce (the
    // radiance-cache band: no dedicated rays, the rough lobe resamples the
    // world cache with the emissive highlight capture).
    for (std::int32_t side = 0; side < 2; ++side)
    {
        SceneMesh panel;
        const float x = side == 0 ? -2.6f : 2.6f;
        appendBox(panel, {x, 2.3f, -5.94f}, {0.9f, 0.65f, 0.012f});
        panel.material.baseColor[0] = 0.66f;
        panel.material.baseColor[1] = 0.62f;
        panel.material.baseColor[2] = 0.58f;
        panel.material.roughnessFactor = 0.35f;
        panel.material.metallicFactor  = 1.0f;
        panel.giAlbedo[0]           = 0.66f;
        panel.giAlbedo[1]           = 0.62f;
        panel.giAlbedo[2]           = 0.58f;
        panel.rfRoughness           = 0.35f;
        panel.rfMetallic            = 1.0f;
        panel.name = side == 0 ? "brushed panel left" : "brushed panel right";
        scene.push_back(std::move(panel));
    }

    // Chrome sphere on the platform and a small gold sphere by the pool:
    // dedicated-ray metals at two color temperatures.
    {
        SceneMesh chromeSphere;
        appendSphere(chromeSphere, {-3.4f, 1.12f, -4.4f}, 0.42f, 48, 24);
        chromeSphere.material.baseColor[0] = 0.72f;
        chromeSphere.material.baseColor[1] = 0.73f;
        chromeSphere.material.baseColor[2] = 0.75f;
        chromeSphere.material.roughnessFactor = 0.15f;
        chromeSphere.material.metallicFactor  = 1.0f;
        chromeSphere.giAlbedo[0]           = 0.05f;
        chromeSphere.giAlbedo[1]           = 0.05f;
        chromeSphere.giAlbedo[2]           = 0.055f;
        chromeSphere.rfRoughness           = 0.15f;
        chromeSphere.rfMetallic            = 1.0f;
        chromeSphere.name = "chrome sphere";
        scene.push_back(std::move(chromeSphere));
    }
    {
        SceneMesh goldSphere;
        appendSphere(goldSphere, {5.6f, 0.22f, 1.8f}, 0.22f, 32, 16);
        goldSphere.material.baseColor[0] = 0.94f;
        goldSphere.material.baseColor[1] = 0.71f;
        goldSphere.material.baseColor[2] = 0.29f;
        goldSphere.material.roughnessFactor = 0.1f;
        goldSphere.material.metallicFactor  = 1.0f;
        goldSphere.giAlbedo[0]           = 0.05f;
        goldSphere.giAlbedo[1]           = 0.04f;
        goldSphere.giAlbedo[2]           = 0.02f;
        goldSphere.rfRoughness           = 0.1f;
        goldSphere.rfMetallic            = 1.0f;
        goldSphere.name = "gold sphere";
        scene.push_back(std::move(goldSphere));
    }

    // Spinning chrome cube on the metal strip: the dynamic-surface test piece
    // (renderer + lumen_gi + lumen_reflection all receive its per-frame
    // rotation), catching the moving room reflection.
    {
        SceneMesh chromeCube;
        appendBox(chromeCube, {0.0f, 0.31f, -1.0f}, {0.28f, 0.28f, 0.28f});
        chromeCube.material.baseColor[0] = 0.72f;
        chromeCube.material.baseColor[1] = 0.73f;
        chromeCube.material.baseColor[2] = 0.75f;
        chromeCube.material.roughnessFactor = 0.1f;
        chromeCube.material.metallicFactor  = 1.0f;
        chromeCube.giAlbedo[0]           = 0.05f;
        chromeCube.giAlbedo[1]           = 0.05f;
        chromeCube.giAlbedo[2]           = 0.055f;
        chromeCube.rfRoughness           = 0.1f;
        chromeCube.rfMetallic            = 1.0f;
        chromeCube.name = "chrome cube";
        scene.push_back(std::move(chromeCube));
    }

    // Transparent and translucent props, pushed last and drawn last.
    // Glass sphere resting in the pool: transmission chains under the warm
    // panel plus the caustics it focuses onto the floor beneath the water.
    {
        SceneMesh glassSphere;
        appendSphere(glassSphere, {1.6f, 0.46f, 0.4f}, 0.34f, 48, 24);
        glassSphere.material.baseColor[0] = 0.96f;
        glassSphere.material.baseColor[1] = 0.98f;
        glassSphere.material.baseColor[2] = 0.97f;
        glassSphere.material.roughnessFactor = 0.06f;
        glassSphere.material.metallicFactor  = 0.0f;
        glassSphere.material.opacity           = 0.04f;
        glassSphere.material.indexOfRefraction = 1.5f;
        glassSphere.giAlbedo[0]           = 0.05f;
        glassSphere.giAlbedo[1]           = 0.05f;
        glassSphere.giAlbedo[2]           = 0.055f;
        glassSphere.rfRoughness           = 0.06f;
        glassSphere.rfMetallic            = 0.0f;
        glassSphere.name = "glass sphere";
        scene.push_back(std::move(glassSphere));
    }
    // Frosted pane filling the partition's gap.
    {
        SceneMesh frostedPane;
        appendBox(frostedPane, {1.1f, 1.05f, -1.2f}, {1.1f, 1.05f, 0.025f});
        frostedPane.material.baseColor[0] = 0.92f;
        frostedPane.material.baseColor[1] = 0.88f;
        frostedPane.material.baseColor[2] = 0.82f;
        frostedPane.material.roughnessFactor = 0.3f;
        frostedPane.material.metallicFactor  = 0.0f;
        frostedPane.material.opacity           = 0.55f;
        frostedPane.material.indexOfRefraction = 1.45f;
        frostedPane.giAlbedo[0]           = 0.55f;
        frostedPane.giAlbedo[1]           = 0.52f;
        frostedPane.giAlbedo[2]           = 0.48f;
        frostedPane.rfRoughness           = 0.3f;
        frostedPane.rfMetallic            = 0.0f;
        frostedPane.name = "frosted pane";
        scene.push_back(std::move(frostedPane));
    }
    // Ruby glass cube on the platform next to the chrome sphere.
    {
        SceneMesh rubyCube;
        appendBox(rubyCube, {-4.6f, 0.83f, -3.6f}, {0.13f, 0.13f, 0.13f});
        rubyCube.material.baseColor[0] = 0.72f;
        rubyCube.material.baseColor[1] = 0.08f;
        rubyCube.material.baseColor[2] = 0.10f;
        rubyCube.material.roughnessFactor = 0.05f;
        rubyCube.material.metallicFactor  = 0.0f;
        rubyCube.material.opacity           = 0.15f;
        rubyCube.material.indexOfRefraction = 1.5f;
        rubyCube.giAlbedo[0]           = 0.05f;
        rubyCube.giAlbedo[1]           = 0.01f;
        rubyCube.giAlbedo[2]           = 0.015f;
        rubyCube.rfRoughness           = 0.05f;
        rubyCube.rfMetallic            = 0.0f;
        rubyCube.name = "ruby cube";
        scene.push_back(std::move(rubyCube));
    }

    return scene;
}

} // namespace demo::lumen_gi
