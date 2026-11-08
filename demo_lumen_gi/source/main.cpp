// main.cpp - the lumen_gi test application.
//
// A standalone harness for validating dynamic global illumination end to
// end: it builds small deterministic scenes, registers every mesh BOTH into
// the renderer (draw geometry) and into the lumen_gi module (GI surfaces),
// evaluates one GI frame per rendered frame, and exports screenshots of the
// composed image through the renderer's probe-grid indirect-lighting hook.
//
// Views:
//   --view interior  Cornell-style dark room: white walls, red/green side
//                    walls, two emissive ceiling panels, one occluder box.
//                    Sun disabled (profiles/config); all light is bounced.
//   --view outdoor   Sunlit ground plane with colored blocks, unlit sky dome
//                    and sun disc (excluded from GI tracing; probes fall
//                    back to the hemispheric skylight on miss).
//
// Useful flags: --frames N renders exactly N frames (deterministic, unlike
// the wall-clock --quit-after), --screenshot/--screenshot-frame export the
// rendered frame, --export-debug additionally writes the lumen_gi module's
// own debug views (final image, GI only, surface cache, probes, reference
// path trace) next to the screenshot for A/B validation.

#include <engine/application/application.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include "engine/application/application_host.hpp"
#include "engine/application/command_line.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{

using namespace engine;

namespace window = engine::window;

// ---- options ----------------------------------------------------------------

struct GiDemoOptions
{
    std::string   viewName         = "interior"; // --view interior|outdoor
    std::uint64_t frames           = 0;          // --frames: exact frame count (0 = default 30)
    std::string   screenshotPath;                // --screenshot: PPM export at screenshotFrame
    std::uint64_t screenshotFrame  = 30;         // --screenshot-frame
    std::string   exportDebugPrefix;             // --export-debug: lumen_gi debug views at screenshotFrame
    std::uint32_t statsEveryFrames = 30;         // --stats-every: print lumen_gi statistics cadence
};

const char *giOptionsUsage() noexcept
{
    return "GI demo options:\n"
           "  --view interior|outdoor  Scene to render (default interior).\n"
           "  --frames <n>             Render exactly n frames, then exit (deterministic).\n"
           "  --screenshot <file.ppm>  Export the rendered frame (hook output) as PPM.\n"
           "  --screenshot-frame <n>   Frame index for --screenshot (default 30).\n"
           "  --export-debug <prefix>  Also write lumen_gi debug views <prefix>_<view>.ppm\n"
           "                           (final, gionly, lumenscene, surfacecache, probes,\n"
           "                           reference) at the screenshot frame.\n"
           "  --stats-every <n>        Print lumen_gi statistics every n frames (default 30).\n";
}

bool parseGiOptions(const std::vector<std::string> &arguments, GiDemoOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const bool         hasValue = index + 1 < arguments.size();
        if (argument == "--view" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "interior" && value != "outdoor")
            {
                outError = "invalid view '" + value + "'; expected 'interior' or 'outdoor'";
                return false;
            }
            options.viewName = value;
        }
        else if (argument == "--frames" && hasValue)
        {
            options.frames = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--screenshot" && hasValue)
        {
            options.screenshotPath = arguments[++index];
        }
        else if (argument == "--screenshot-frame" && hasValue)
        {
            options.screenshotFrame = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--export-debug" && hasValue)
        {
            options.exportDebugPrefix = arguments[++index];
        }
        else if (argument == "--stats-every" && hasValue)
        {
            options.statsEveryFrames
                = static_cast<std::uint32_t>(std::strtoull(arguments[++index].c_str(), nullptr, 10));
        }
        else
        {
            outError = "unknown or incomplete option '" + argument + "'\n" + giOptionsUsage();
            return false;
        }
    }
    return true;
}

// ---- geometry ---------------------------------------------------------------

// Canonical 11-float interleaved vertex: position, normal, color, uv.
struct SceneVertex
{
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec3 color{1.0f};
    glm::vec2 uv{0.0f};
};

struct SceneMesh
{
    std::vector<SceneVertex>   vertices;
    std::vector<std::uint32_t> indices;

    renderer::MeshHandle      rendererMesh = renderer::invalidMesh;
    lumen_gi::GiSurfaceHandle giSurface    = lumen_gi::invalidGiSurface;
    renderer::MeshMaterial    material{};
    float                     giAlbedo[3]         = {1.0f, 1.0f, 1.0f};
    float                     giEmissive[3]       = {0.0f, 0.0f, 0.0f};
    float                     giEmissiveIntensity = 1.0f;
    std::uint32_t             giFlagBits          = 0;
};

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

} // namespace

int main(int argc, char **argv)
{
    std::cout << std::unitbuf;

    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo_lumen_gi: " << commandLineError << '\n'
                  << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << giOptionsUsage();
        return 0;
    }

    GiDemoOptions options;
    if (!parseGiOptions(commandLine.applicationArguments, options, commandLineError))
    {
        std::cerr << "demo_lumen_gi: " << commandLineError << "\n";
        return 2;
    }

    // Modules this harness needs: window + renderer draw the scene, pbr
    // supplies the sun/ambient environment, lumen_gi runs the GI pipeline.
    engine::ApplicationHostOptions hostOptions
        = engine::makeApplicationHostOptions(commandLine, {"window", "renderer", "pbr", "lumen_gi"});
    engine::ApplicationHost host;
    std::string             startupReport;
    if (commandLine.validateOnly)
    {
        const bool valid = host.validate(hostOptions, startupReport);
        std::cout << startupReport;
        std::cout << (valid ? "validate: ok\n" : "validate: failed\n");
        return valid ? 0 : 1;
    }
    if (!host.start(hostOptions, startupReport))
    {
        std::cerr << startupReport << "demo_lumen_gi: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::ParticipantContext &context = host.context();

    window::WindowCapability   &windowCapability = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability = context.acquireCapability<renderer::RenderCapability>();
    auto *giCapability = context.tryAcquireCapability<lumen_gi::LumenGiCapability>();
    if (giCapability == nullptr || giCapability->abi != lumen_gi::lumenGiCapabilityAbi)
    {
        std::cerr << "demo_lumen_gi: the lumen_gi capability is unavailable or its ABI differs; this harness "
                     "is pointless without it\n";
        host.stop();
        return 1;
    }
    const pbr::PbrCapability *pbrCapability = context.tryAcquireCapability<pbr::PbrCapability>();
    pbr::PbrEnvironment      environment{};
    const bool               hasEnvironment
        = pbrCapability != nullptr && pbrCapability->copyEnvironment(pbrCapability->self, &environment, nullptr);
    if (hasEnvironment)
    {
        std::cout << "demo_lumen_gi: pbr sun " << (environment.sun.enabled ? "enabled" : "disabled")
                  << " (intensity " << environment.sun.intensity << ")\n";
    }

    // Fixed deterministic camera per view.
    const glm::vec3 viewEye
        = options.viewName == "outdoor" ? glm::vec3(7.0f, 3.4f, 9.0f) : glm::vec3(0.0f, 1.5f, 1.7f);
    const glm::vec3 viewTarget
        = options.viewName == "outdoor" ? glm::vec3(-0.5f, 0.9f, -2.5f) : glm::vec3(0.0f, 1.1f, -1.2f);
    const float     fovRadians = 60.0f * 3.141592654f / 180.0f;
    const float     aspect     = 640.0f / 360.0f;
    const glm::mat4 view       = glm::lookAt(viewEye, viewTarget, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::mat4 projection = glm::perspective(fovRadians, aspect, 0.1f, 300.0f);
    const glm::mat4 viewProj   = projection * view;

    renderer::CameraMatrices cameraMatrices{};
    std::memcpy(cameraMatrices.view, glm::value_ptr(view), sizeof(cameraMatrices.view));
    std::memcpy(cameraMatrices.projection, glm::value_ptr(projection), sizeof(cameraMatrices.projection));
    cameraMatrices.position[0] = viewEye.x;
    cameraMatrices.position[1] = viewEye.y;
    cameraMatrices.position[2] = viewEye.z;

    window::WindowDescription  windowDescription{};
    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo_lumen_gi: the window module could not create a window\n";
        host.stop();
        return 1;
    }

    renderer::RenderError          renderError = renderer::RenderError::None;
    const renderer::RendererHandle rendererHandle
        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo_lumen_gi: the renderer module could not create a renderer\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    std::cout << "demo_lumen_gi: window backend '" << windowCapability.activeBackendName(windowCapability.self)
              << "', renderer backend '" << renderCapability.activeBackendName(renderCapability.self, rendererHandle)
              << "'\n";

    renderCapability.setViewport(renderCapability.self, rendererHandle, 640, 360);
    renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

    // Build the scene and register every mesh into BOTH consumers: the
    // renderer draws it, lumen_gi bounces light off it. Vertex attributes are
    // shared (one interleaved buffer, per-attribute strides), which mirrors
    // what a real engine integration would do.
    std::vector<SceneMesh> scene = options.viewName == "outdoor" ? buildOutdoorScene() : buildInteriorScene();
    std::cout << "demo_lumen_gi: view '" << options.viewName << "', " << scene.size() << " surfaces\n";

    constexpr std::uint32_t vertexStride = sizeof(SceneVertex);
    const glm::mat4         identityTransform(1.0f); // baked-in world transforms; no per-mesh model matrix
    std::vector<renderer::RenderDrawCommand> drawCommands;
    for (SceneMesh &mesh : scene)
    {
        std::array<renderer::VertexAttribute, 4> attributes = {{
            {renderer::VertexSemantic::Position, renderer::VertexFormat::Float32x3, 0},
            {renderer::VertexSemantic::Normal, renderer::VertexFormat::Float32x3, 12},
            {renderer::VertexSemantic::Color, renderer::VertexFormat::Float32x3, 24},
            {renderer::VertexSemantic::TexCoord0, renderer::VertexFormat::Float32x2, 36},
        }};

        renderer::MeshDescription description;
        description.vertices             = mesh.vertices.data();
        description.vertexCount          = static_cast<std::uint32_t>(mesh.vertices.size());
        description.indices              = mesh.indices.data();
        description.indexCount           = static_cast<std::uint32_t>(mesh.indices.size());
        description.indexFormat          = renderer::MeshIndexFormat::UInt32;
        description.vertexStrideBytes    = vertexStride;
        description.vertexAttributes     = attributes.data();
        description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());

        mesh.rendererMesh
            = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &renderError);
        if (mesh.rendererMesh == renderer::invalidMesh)
        {
            std::cerr << "demo_lumen_gi: mesh upload failed\n";
        }

        if (giCapability->createSurface != nullptr)
        {
            lumen_gi::GiSurfaceDescription surface{};
            surface.positions           = &mesh.vertices.front().position.x;
            surface.positionStrideBytes = vertexStride;
            surface.normals             = &mesh.vertices.front().normal.x;
            surface.normalStrideBytes   = vertexStride;
            surface.colors              = &mesh.vertices.front().color.x;
            surface.colorStrideBytes    = vertexStride;
            surface.indices             = mesh.indices.data();
            surface.vertexCount         = static_cast<std::uint32_t>(mesh.vertices.size());
            surface.indexCount          = static_cast<std::uint32_t>(mesh.indices.size());
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                surface.albedoColor[axis]   = mesh.giAlbedo[axis];
                surface.emissiveColor[axis] = mesh.giEmissive[axis];
            }
            surface.emissiveIntensity = mesh.giEmissiveIntensity;
            surface.flagBits          = mesh.giFlagBits;
            lumen_gi::GiError giError = lumen_gi::GiError::None;
            mesh.giSurface            = giCapability->createSurface(giCapability->self, &surface, &giError);
            if (mesh.giSurface == lumen_gi::invalidGiSurface)
            {
                std::cerr << "demo_lumen_gi: GI surface registration failed (error "
                          << static_cast<std::uint32_t>(giError) << ")\n";
            }
        }

        if (mesh.rendererMesh != renderer::invalidMesh)
        {
            drawCommands.push_back(renderer::RenderDrawCommand{
                .mesh = mesh.rendererMesh, .modelMatrix = glm::value_ptr(identityTransform), .material = &mesh.material});
        }
    }

    // The GI camera tracks the render camera; evaluate() runs one full
    // Lumen-style GI frame per rendered frame. Column-major view-projection
    // straight out of glm (OpenGL convention, same as CameraMatrices).
    const float viewProjection[16] = {
        viewProj[0][0], viewProj[0][1], viewProj[0][2], viewProj[0][3],
        viewProj[1][0], viewProj[1][1], viewProj[1][2], viewProj[1][3],
        viewProj[2][0], viewProj[2][1], viewProj[2][2], viewProj[2][3],
        viewProj[3][0], viewProj[3][1], viewProj[3][2], viewProj[3][3],
    };
    const float cameraPosition[3] = {viewEye.x, viewEye.y, viewEye.z};

    const std::uint64_t totalFrames     = options.frames != 0 ? options.frames : 30;
    const double        frameDeltaSeconds = 1.0 / 30.0;

    bool screenshotTaken = false;
    for (std::uint64_t frameIndex = 0; frameIndex < totalFrames; ++frameIndex)
    {
        if (giCapability->setView != nullptr)
        {
            giCapability->setView(giCapability->self, viewProjection, cameraPosition);
        }
        lumen_gi::GiError giError = lumen_gi::GiError::None;
        if (giCapability->evaluate != nullptr
            && !giCapability->evaluate(giCapability->self, frameDeltaSeconds, &giError))
        {
            std::cerr << "demo_lumen_gi: lumen_gi evaluate failed (error "
                      << static_cast<std::uint32_t>(giError) << ")\n";
            break;
        }

        renderCapability.beginFrame(renderCapability.self, rendererHandle, nullptr);
        renderCapability.drawMeshes(
            renderCapability.self, rendererHandle, drawCommands.data(), static_cast<std::uint32_t>(drawCommands.size()));
        renderCapability.endFrame(renderCapability.self, rendererHandle);

        if (options.statsEveryFrames != 0 && frameIndex % options.statsEveryFrames == 0
            && giCapability->statistics != nullptr)
        {
            lumen_gi::GiStatistics statistics{};
            if (giCapability->statistics(giCapability->self, &statistics))
            {
                std::cout << "demo_lumen_gi: frame " << frameIndex << " | surfaces " << statistics.surfaces
                          << ", cards " << statistics.cards << ", probes traced " << statistics.probeTracesThisFrame
                          << ", gather rays " << statistics.gatherRaysThisFrame << ", evaluate "
                          << statistics.lastEvaluateMilliseconds << " ms\n";
            }
        }

        if (!screenshotTaken && !options.screenshotPath.empty() && frameIndex == options.screenshotFrame)
        {
            // Headless runs export the window surface directly; Vulkan runs
            // schedule the export on the renderer's vulkan extension.
            auto *headlessWindowCapability = context.tryAcquireCapability<window::HeadlessWindowCapability>();
            if (headlessWindowCapability != nullptr
                && windowCapability.activeBackendName(windowCapability.self) == std::string("headless"))
            {
                window::WindowError exportError = window::WindowError::None;
                screenshotTaken                 = headlessWindowCapability->exportSurfacePpm(
                    headlessWindowCapability->self, windowHandle, options.screenshotPath.c_str(), &exportError);
            }
            if (!screenshotTaken)
            {
                auto *vulkanCapability = context.tryAcquireCapability<renderer::VulkanRenderCapability>();
                if (vulkanCapability != nullptr
                    && renderCapability.activeBackendName(renderCapability.self, rendererHandle)
                        == std::string("vulkan"))
                {
                    renderer::RenderError exportError = renderer::RenderError::None;
                    screenshotTaken                   = vulkanCapability->exportFramePpm(
                        vulkanCapability->self, rendererHandle, options.screenshotPath.c_str(), &exportError);
                    if (screenshotTaken)
                    {
                        std::cout << "demo_lumen_gi: screenshot scheduled on the vulkan backend (written at the "
                                     "next frame retirement)\n";
                    }
                }
            }
            if (screenshotTaken)
            {
                std::cout << "demo_lumen_gi: screenshot written to " << options.screenshotPath << "\n";
            }
            else
            {
                std::cerr << "demo_lumen_gi: screenshot export failed for backend '"
                          << windowCapability.activeBackendName(windowCapability.self) << "'\n";
            }

            // Optional module debug views for A/B validation against the
            // hook output captured in the screenshot above.
            if (!options.exportDebugPrefix.empty() && giCapability->debugExport != nullptr)
            {
                const lumen_gi::GiDebugView views[] = {lumen_gi::GiDebugView::FinalImage,
                    lumen_gi::GiDebugView::GiOnly,
                    lumen_gi::GiDebugView::LumenScene,
                    lumen_gi::GiDebugView::SurfaceCache,
                    lumen_gi::GiDebugView::ProbeIrradiance,
                    lumen_gi::GiDebugView::Reference};
                const char *viewNames[]     = {"final", "gionly", "lumenscene", "surfacecache", "probes", "reference"};
                for (std::uint32_t viewIndex = 0; viewIndex < 6; ++viewIndex)
                {
                    const std::string path = options.exportDebugPrefix + "_" + viewNames[viewIndex] + ".ppm";
                    lumen_gi::GiError exportGiError = lumen_gi::GiError::None;
                    if (!giCapability->debugExport(
                            giCapability->self, views[viewIndex], path.c_str(), &exportGiError))
                    {
                        std::cerr << "demo_lumen_gi: debug view export failed ("
                                  << static_cast<std::uint32_t>(exportGiError) << ")\n";
                    }
                }
            }
        }
    }

    if (giCapability->statistics != nullptr)
    {
        lumen_gi::GiStatistics finalStatistics{};
        if (giCapability->statistics(giCapability->self, &finalStatistics))
        {
            std::cout << "demo_lumen_gi: lumen_gi totals - frames " << finalStatistics.frameIndex << ", surfaces "
                      << finalStatistics.surfaces << ", cards " << finalStatistics.cards << ", card texels "
                      << finalStatistics.cardTexels << ", total rays " << finalStatistics.totalRaysTraced << "\n";
        }
    }

    for (SceneMesh &mesh : scene)
    {
        if (mesh.rendererMesh != renderer::invalidMesh)
        {
            renderCapability.destroyMesh(renderCapability.self, rendererHandle, mesh.rendererMesh);
        }
        if (mesh.giSurface != lumen_gi::invalidGiSurface && giCapability->destroySurface != nullptr)
        {
            giCapability->destroySurface(giCapability->self, mesh.giSurface);
        }
    }
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    host.stop();
    std::cout << "demo_lumen_gi: stopped cleanly after " << totalFrames << " frames\n";
    return 0;
}
