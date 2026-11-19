// main.cpp - the lumen_gi test application.
//
// A standalone harness for validating dynamic global illumination and
// reflections end to end: it builds small deterministic scenes, registers
// every mesh into the renderer (draw geometry), the lumen_gi module (GI
// surfaces), and the lumen_reflection module (reflection surfaces with their
// PBR specular parameters), evaluates one GI and one reflection frame per
// rendered frame, and exports screenshots of the composed image through the
// renderer's probe-grid indirect-lighting and reflection-field hooks.
//
// Views:
//   --view interior  Cornell-style dark room: white walls, red/green side
//                    walls, two emissive ceiling panels, one occluder box.
//                    Sun disabled (profiles/config); all light is bounced.
//   --view outdoor   Sunlit ground plane with colored blocks, unlit sky dome
//                    and sun disc (excluded from GI tracing; probes fall
//                    back to the hemispheric skylight on miss).
//
// The frame loop pumps window events every frame, so the OS window stays
// responsive even while a GI evaluate takes tens of milliseconds on the CPU.
// Without --frames the demo runs interactively until the window is closed or
// Escape is pressed; headless runs (and --frames N) stop after a fixed count.
//
// The interior scene adds transparent and translucent props (a clear glass
// sphere, a frosted pane, a tinted ruby cube) that exercise the
// transmission paths end to end: the renderer transparent pass,
// lumen_reflection refracted ray chains, and lumen_gi pass-through
// gathers, probe transmittance, and softened shadows. --self-test
// validates the whole chain headlessly (module statistics, snapshot
// versions, transmission counters, and the exported screenshot content).
//
// Useful flags: --frames N renders exactly N frames (deterministic),
// --screenshot/--screenshot-frame export the
// rendered frame, --export-debug additionally writes the lumen_gi module's
// own debug views (final image, GI only, surface cache, probes, reference
// path trace) and the lumen_reflection module's views (composite,
// reflections only, resolve mask, reference) next to the screenshot for A/B
// validation, --no-reflections disables the reflection module for the
// pre-hook image, and --spin rotates one scene object every frame (the
// interior occluder, the outdoor chrome sphere) so the dynamic-surface path
// is exercised. Screenshot exports that should show the fully converged
// lighting need enough frames for the radiance caches to settle: give a
// stationary scene at least 150-300 frames (see the validation numbers in
// modules/lumen_gi/docs/overview.md) instead of the 30-frame headless
// default, whose indirect term is still sharpening.

#include <engine/application/application.hpp>
#include "demo_harness.hpp"
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include "demo/gi_demo_support.hpp"
#include <engine/application/application_host.hpp>
#include <engine/application/command_line.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace
{
using namespace engine;
using namespace demo::lumen_gi;
using demo::harness::FrameTimingRecorder;
namespace window = engine::window;

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
    if (options.selfTest)
    {
        // Deterministic validation profile: the interior view (with its
        // transparent/translucent props), a fixed frame count, and a
        // screenshot at the last frame the checks below parse.
        options.viewName        = "interior";
        options.frames          = 24;
        options.screenshotFrame = 23;
        options.statsEveryFrames = 0;
        options.reflections     = true;
        options.screenshotPath  = "lumen_gi_selftest.ppm";
        std::cout << "demo_lumen_gi: self-test run (interior, 24 frames)\n";
    }

    // Modules this harness needs: window + renderer draw the scene, camera
    // drives the view, pbr supplies the sun/ambient environment, lumen_gi
    // runs the GI pipeline.
    engine::ApplicationHostOptions hostOptions
        = engine::makeApplicationHostOptions(commandLine,
        {"window", "renderer", "camera", "pbr", "lumen_gi", "lumen_reflection"});
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
    const std::int64_t maxFps = context.configurationIntegerInSection("engine", "max_fps").value_or(0);

    window::WindowCapability   &windowCapability = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability = context.acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability   &cameraCapability = context.acquireCapability<camera::CameraCapability>();
    auto *giCapability = context.tryAcquireCapability<lumen_gi::LumenGiCapability>();
    if (giCapability == nullptr || giCapability->abi != lumen_gi::lumenGiCapabilityAbi)
    {
        std::cerr << "demo_lumen_gi: the lumen_gi capability is unavailable or its ABI differs; this harness "
                     "is pointless without it\n";
        host.stop();
        return 1;
    }
    auto *reflectionCapability = context.tryAcquireCapability<lumen_reflection::LumenReflectionCapability>();
    if (options.reflections
        && (reflectionCapability == nullptr
            || reflectionCapability->abi != lumen_reflection::lumenReflectionCapabilityAbi))
    {
        std::cerr << "demo_lumen_gi: the lumen_reflection capability is unavailable or its ABI differs; "
                     "run with --no-reflections or enable the module\n";
        host.stop();
        return 1;
    }
    // The demo always runs the reflection module alongside lumen_gi unless
    // --no-reflections opts out of the A/B. The renderer-side composite has
    // its own gate; surface it loudly instead of rendering silently without
    // the field the module just traced.
    if (options.reflections)
    {
        const std::optional<bool> rendererReflectionGate
            = context.configurationBooleanInSection("renderer", "reflection_indirect_specular");
        if (rendererReflectionGate.has_value() && !rendererReflectionGate.value())
        {
            std::cerr << "demo_lumen_gi: warning - renderer.reflection_indirect_specular is disabled; the "
                         "renderer will not composite the lumen_reflection field this run\n";
        }
        std::cout << "demo_lumen_gi: reflections enabled - lumen_reflection module registered and evaluated "
                     "every frame (--no-reflections disables it for A/B)\n";
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

    // Camera pose per view; the engine camera module owns the matrices and
    // the interactive WASD/mouse-look controls (Fly mode).
    const glm::vec3 viewEye
        = options.viewName == "outdoor" ? glm::vec3(7.0f, 3.4f, 9.0f) : glm::vec3(0.0f, 1.5f, 1.7f);
    const glm::vec3 viewTarget
        = options.viewName == "outdoor" ? glm::vec3(-0.5f, 0.9f, -2.5f) : glm::vec3(0.0f, 1.1f, -1.2f);

    camera::CameraParameters cameraParameters{};
    cameraParameters.position[0]  = viewEye.x;
    cameraParameters.position[1]  = viewEye.y;
    cameraParameters.position[2]  = viewEye.z;
    cameraParameters.hasPosition   = true;
    cameraParameters.target[0]     = viewTarget.x;
    cameraParameters.target[1]     = viewTarget.y;
    cameraParameters.target[2]     = viewTarget.z;
    cameraParameters.hasTarget     = true;
    cameraParameters.fovYDegrees   = 60.0f;
    cameraParameters.nearZ         = 0.1f;
    cameraParameters.farZ          = 300.0f;
    camera::CameraError cameraError = camera::CameraError::None;
    const camera::CameraHandle cameraHandle
        = cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo_lumen_gi: the camera module could not create a camera\n";
        host.stop();
        return 1;
    }
    cameraCapability.setControlMode(cameraCapability.self, cameraHandle, camera::CameraControlMode::Fly);

    // --orbit sweeps the camera on a circular path. The interior circle is
    // centered in the room so the sweep stays inside the walls; the outdoor
    // view circles the default target in the open field. Both start from the
    // view's default pose.
    const glm::vec3 orbitCenter
        = options.viewName == "outdoor" ? viewTarget : glm::vec3(0.0f, 1.1f, 0.0f);
    const float orbitRadius       = std::max(0.5f,
        glm::length(glm::vec2(viewEye.x, viewEye.z) - glm::vec2(orbitCenter.x, orbitCenter.z)));
    const float orbitHeightOffset = viewEye.y - orbitCenter.y;
    double      orbitAngle        = std::atan2(viewEye.z - orbitCenter.z, viewEye.x - orbitCenter.x);

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
    const std::string windowBackendName = windowCapability.activeBackendName(windowCapability.self);
    std::cout << "demo_lumen_gi: window backend '" << windowBackendName << "', renderer backend '"
              << renderCapability.activeBackendName(renderCapability.self, rendererHandle) << "'\n";

    // First event slice now that the window is mapped: scene setup below runs
    // before the frame loop takes over the pumping.
    windowCapability.pollEvents(windowCapability.self);

    // Fly-camera mouse look wants a captured cursor; Left Alt releases it and
    // Escape quits (windowed backends only - the headless backend has no
    // cursor or keyboard).
    const bool windowedBackend = windowBackendName != std::string("headless");
    bool       cursorCaptured  = false;
    if (windowedBackend)
    {
        cursorCaptured = true;
        windowCapability.setCursorMode(windowCapability.self, windowHandle, window::CursorMode::Captured);
        std::cout << "demo_lumen_gi: controls - mouse looks, WASD moves, Space/Ctrl up/down, Shift sprints,\n"
                     "      arrows look, Left Alt toggles the cursor, Escape quits\n";
    }
    else
    {
        std::cout << "demo_lumen_gi: headless window - no keyboard/cursor input; use --orbit for a moving camera\n";
    }

    // Track the actual framebuffer (resizes keep this current through the
    // event drain below); the camera aspect and the GI view follow it.
    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        framebufferWidth  = 1920;
        framebufferHeight = 1080;
    }
    renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);

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
            surface.opacity           = mesh.material.opacity;
            surface.indexOfRefraction = mesh.material.indexOfRefraction;
            surface.flagBits          = mesh.giFlagBits;
            lumen_gi::GiError giError = lumen_gi::GiError::None;
            mesh.giSurface            = giCapability->createSurface(giCapability->self, &surface, &giError);
            if (mesh.giSurface == lumen_gi::invalidGiSurface)
            {
                std::cerr << "demo_lumen_gi: GI surface registration failed (error "
                          << static_cast<std::uint32_t>(giError) << ")\n";
            }
        }

        if (options.reflections && reflectionCapability->createSurface != nullptr)
        {
            lumen_reflection::ReflectionSurfaceDescription reflectionSurface{};
            reflectionSurface.positions           = &mesh.vertices.front().position.x;
            reflectionSurface.positionStrideBytes = vertexStride;
            reflectionSurface.normals             = &mesh.vertices.front().normal.x;
            reflectionSurface.normalStrideBytes   = vertexStride;
            reflectionSurface.colors              = &mesh.vertices.front().color.x;
            reflectionSurface.colorStrideBytes    = vertexStride;
            reflectionSurface.indices             = mesh.indices.data();
            reflectionSurface.vertexCount          = static_cast<std::uint32_t>(mesh.vertices.size());
            reflectionSurface.indexCount          = static_cast<std::uint32_t>(mesh.indices.size());
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                // The reflection module combines its surface albedo with the
                // vertex colors exactly like the renderer's base color, and
                // uses it as the specular F0 tint for metallic surfaces - so
                // the PBR base color (not the GI diffuse albedo) is the right
                // source here.
                reflectionSurface.albedoColor[axis]   = mesh.material.baseColor[axis];
                reflectionSurface.emissiveColor[axis] = mesh.giEmissive[axis];
            }
            reflectionSurface.emissiveIntensity = mesh.giEmissiveIntensity;
            reflectionSurface.roughness         = mesh.rfRoughness;
            reflectionSurface.metallic          = mesh.rfMetallic;
            reflectionSurface.opacity           = mesh.material.opacity;
            reflectionSurface.indexOfRefraction = mesh.material.indexOfRefraction;
            reflectionSurface.flagBits          = mesh.giFlagBits;
            lumen_reflection::ReflectionError reflectionError = lumen_reflection::ReflectionError::None;
            mesh.reflectionSurface = reflectionCapability->createSurface(
                reflectionCapability->self, &reflectionSurface, &reflectionError);
            if (mesh.reflectionSurface == lumen_reflection::invalidReflectionSurface)
            {
                std::cerr << "demo_lumen_gi: reflection surface registration failed (error "
                          << static_cast<std::uint32_t>(reflectionError) << ")\n";
            }
        }

        if (mesh.rendererMesh != renderer::invalidMesh)
        {
            drawCommands.push_back(renderer::RenderDrawCommand{
                .mesh = mesh.rendererMesh, .modelMatrix = glm::value_ptr(identityTransform), .material = &mesh.material});
        }
    }

    // --spin: the object that rotates around its own center every frame
    // (the interior occluder, the outdoor chrome sphere). Its per-frame model
    // matrix goes to the renderer's draw command and, through
    // updateSurfaceTransform, to the lumen_gi and lumen_reflection surfaces -
    // the dynamic-surface path the modules document (moving instances carry
    // their captured cards and re-light at the front of the queue). The
    // rotation center is the object's vertex bounds center, so the mesh
    // spins in place.
    const SceneMesh *spinTarget = nullptr;
    if (options.spin)
    {
        const char *spinName = options.viewName == "outdoor" ? "chrome sphere" : "occluder";
        for (const SceneMesh &mesh : scene)
        {
            if (mesh.name == spinName)
            {
                spinTarget = &mesh;
                break;
            }
        }
        if (spinTarget != nullptr)
        {
            std::cout << "demo_lumen_gi: --spin rotates the '" << spinTarget->name
                      << "' mesh every frame (renderer + GI + reflection transforms)\n";
        }
        else
        {
            std::cout << "demo_lumen_gi: --spin requested but no '" << spinName << "' mesh exists; ignoring\n";
        }
    }
    glm::vec3  spinCenter(0.0f);
    float      spinAngularStep = 0.05f;
    std::size_t spinMeshIndex  = 0;
    if (spinTarget != nullptr)
    {
        glm::vec3 boundsMin(std::numeric_limits<float>::max());
        glm::vec3 boundsMax(-std::numeric_limits<float>::max());
        for (const SceneVertex &vertex : spinTarget->vertices)
        {
            boundsMin = glm::min(boundsMin, vertex.position);
            boundsMax = glm::max(boundsMax, vertex.position);
        }
        spinCenter = 0.5f * (boundsMin + boundsMax);
        for (std::size_t index = 0; index < scene.size(); ++index)
        {
            if (&scene[index] == spinTarget)
            {
                spinMeshIndex = index;
                break;
            }
        }
    }

    // Run mode: --frames N pins an exact frame count (deterministic CI runs
    // and screenshot exports). Without it, a windowed backend runs
    // interactively until the OS window is closed or Escape is pressed;
    // headless backends (no close events) stop after 30 frames.
    const bool          runUntilClosed    = options.frames == 0 && windowedBackend;
    const std::uint64_t totalFrames       = options.frames != 0 ? options.frames : 30;
    const double        frameDeltaSeconds = 1.0 / 30.0;

    if (runUntilClosed)
    {
        std::cout << "demo_lumen_gi: running interactively - close the window or press Escape to stop\n";
    }
    if (!options.screenshotPath.empty() && !runUntilClosed && options.screenshotFrame >= totalFrames)
    {
        std::cout << "demo_lumen_gi: warning - --screenshot-frame " << options.screenshotFrame
                  << " is beyond this run of " << totalFrames << " frames; no screenshot will be taken\n";
    }

    bool          screenshotTaken = false;
    std::uint64_t framesRendered  = 0;
    // Self-test observation: per-frame maxima of the transmission counters,
    // and the snapshot versions the modules published.
    std::uint32_t selfTestMaxIndirectRays = 0;
    std::uint32_t selfTestMaxProbeTraces = 0;
    std::uint32_t selfTestMaxReflectionPixels = 0;
    std::uint32_t selfTestMaxTransmissionSegments = 0;
    std::uint64_t selfTestProbeVersion = 0;
    std::uint64_t selfTestFieldVersion = 0;
    std::array<window::WindowEvent, 64> eventBuffer;
    bool   haveLastCursor = false;
    double lastCursorX    = 0.0;
    double lastCursorY    = 0.0;
    auto   lastWallClock  = std::chrono::steady_clock::now();

    const bool presentedPacing = renderCapability.waitsForPresentation != nullptr
        && renderCapability.waitsForPresentation(renderCapability.self, rendererHandle);
    const bool pacerSuppressedByVsync = presentedPacing && maxFps > 0;
    const double effectiveTargetHz = pacerSuppressedByVsync ? 0.0 : static_cast<double>(std::max<std::int64_t>(maxFps, 0));
    const std::uint32_t pacerSpinWindowMicroseconds = effectiveTargetHz > 0.0 ? 200u : 0u;
    engine::FramePacer framePacer(
        {.targetFrameRateHz = effectiveTargetHz, .spinWindowMicroseconds = pacerSpinWindowMicroseconds});
    framePacer.reset();
    if (pacerSuppressedByVsync)
    {
        std::cout << "demo_lumen_gi: max_fps " << maxFps
                  << " left to vsync - presentation is already pacing the frame loop\n";
    }

    // Per-frame timing capture, shared with the planet demo (update =
    // events + camera + GI/reflection evaluate; upload = the draw-command
    // submission boundary; draw = the renderer frame).
    FrameTimingRecorder frameTiming;

    for (std::uint64_t frameIndex = 0; runUntilClosed || frameIndex < totalFrames; ++frameIndex)
    {
        frameTiming.beginFrame();
        // GLFW requires its event queue to be pumped from the main thread every frame; skipping this
        // is what makes a busy window show up as "not responding" to the window manager.
        windowCapability.pollEvents(windowCapability.self);

        // Drain and dispatch the queued events: close requests, resizes
        // (viewport + camera aspect + GI view all follow the framebuffer),
        // and the Left Alt cursor toggle.
        demo::harness::drainWindowEvents(windowCapability, windowHandle, eventBuffer,
            [&](const window::WindowEvent &event)
            {
                if (event.kind == window::WindowEventKind::CloseRequested)
                {
                    windowCapability.requestClose(windowCapability.self, windowHandle);
                }
                else if (event.kind == window::WindowEventKind::FramebufferResized)
                {
                    if (event.framebufferWidth != 0 && event.framebufferHeight != 0)
                    {
                        framebufferWidth  = event.framebufferWidth;
                        framebufferHeight = event.framebufferHeight;
                        renderCapability.setViewport(
                            renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);
                    }
                }
                else if (event.kind == window::WindowEventKind::KeyPressed
                    && event.key == window::Key::LeftAlt && windowedBackend)
                {
                    cursorCaptured = !cursorCaptured;
                    windowCapability.setCursorMode(windowCapability.self, windowHandle,
                        cursorCaptured ? window::CursorMode::Captured : window::CursorMode::Normal);
                    haveLastCursor = false;
                }
            });

        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            std::cout << "demo_lumen_gi: window close requested\n";
            break;
        }
        if (windowCapability.isKeyDown != nullptr
            && windowCapability.isKeyDown(windowCapability.self, windowHandle, window::Key::Escape))
        {
            std::cout << "demo_lumen_gi: escape pressed\n";
            break;
        }

        // Interactive runs advance the camera by wall-clock time; --frames
        // runs use the fixed 30 Hz step so the camera path is deterministic.
        const auto   now               = std::chrono::steady_clock::now();
        const double wallDeltaSeconds
            = std::min(0.25, std::chrono::duration<double>(now - lastWallClock).count());
        lastWallClock                = now;
        const double cameraDeltaSeconds = options.frames != 0 ? frameDeltaSeconds : wallDeltaSeconds;

        // Camera input: WASD + Space/Ctrl + arrows + Shift sprint, mouse deltas
        // while the cursor is captured.
        camera::CameraInputState cameraInput{};
        const auto keyDown = [&](window::Key key)
        { return windowCapability.isKeyDown(windowCapability.self, windowHandle, key); };
        cameraInput.moveForward    = static_cast<float>(keyDown(window::Key::W))
                                   - static_cast<float>(keyDown(window::Key::S));
        cameraInput.moveRight      = static_cast<float>(keyDown(window::Key::D))
                                   - static_cast<float>(keyDown(window::Key::A));
        cameraInput.moveUp         = static_cast<float>(keyDown(window::Key::Space))
                                   - static_cast<float>(keyDown(window::Key::LeftControl));
        cameraInput.lookHorizontal = static_cast<float>(keyDown(window::Key::Right))
                                   - static_cast<float>(keyDown(window::Key::Left));
        cameraInput.lookVertical   = static_cast<float>(keyDown(window::Key::Up))
                                   - static_cast<float>(keyDown(window::Key::Down));
        cameraInput.sprint         = keyDown(window::Key::LeftShift);
        if (cursorCaptured)
        {
            double cursorX = 0.0;
            double cursorY = 0.0;
            if (windowCapability.cursorPosition(windowCapability.self, windowHandle, &cursorX, &cursorY)
                && std::isfinite(cursorX) && std::isfinite(cursorY))
            {
                if (haveLastCursor)
                {
                    // Clamp the delta to survive platform cursor jumps.
                    double deltaX = cursorX - lastCursorX;
                    double deltaY = cursorY - lastCursorY;
                    demo::harness::clampCursorDelta(deltaX, deltaY, 200.0);
                    cameraInput.mouseDeltaX = static_cast<float>(deltaX);
                    cameraInput.mouseDeltaY = static_cast<float>(deltaY);
                }
                lastCursorX    = cursorX;
                lastCursorY    = cursorY;
                haveLastCursor = true;
            }
            else
            {
                haveLastCursor = false;
            }
        }
        else
        {
            haveLastCursor = false;
        }

        // --orbit overrides the pose each frame: a circular sweep around the
        // orbit center (deterministic per-frame step under --frames).
        if (options.orbit)
        {
            orbitAngle += options.frames != 0 ? 0.05 : 0.5 * cameraDeltaSeconds;
            const glm::vec3 orbitEye = orbitCenter
                + glm::vec3(orbitRadius * static_cast<float>(std::cos(orbitAngle)), orbitHeightOffset,
                    orbitRadius * static_cast<float>(std::sin(orbitAngle)));
            const float eye[3]    = {orbitEye.x, orbitEye.y, orbitEye.z};
            const float target[3] = {orbitCenter.x, orbitCenter.y, orbitCenter.z};
            const float up[3]     = {0.0f, 1.0f, 0.0f};
            cameraCapability.setLookAt(cameraCapability.self, cameraHandle, eye, target, up);
        }

        // --spin advances the object's rotation by the same deterministic
        // step under --frames (0.05 rad/frame, matching the --orbit step).
        // The transform lives for the whole frame iteration: the draw command
        // and both surface updates borrow its storage.
        glm::mat4 spinTransform(1.0f);
        if (spinTarget != nullptr)
        {
            const float angle = static_cast<float>(frameIndex) * spinAngularStep;
            spinTransform = glm::translate(glm::mat4(1.0f), spinCenter)
                * glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f))
                * glm::translate(glm::mat4(1.0f), -spinCenter);
            const float *spinMatrix = glm::value_ptr(spinTransform);
            drawCommands[spinMeshIndex].modelMatrix = spinMatrix;
            if (giCapability->updateSurfaceTransform != nullptr && scene[spinMeshIndex].giSurface != lumen_gi::invalidGiSurface)
            {
                giCapability->updateSurfaceTransform(giCapability->self, scene[spinMeshIndex].giSurface, spinMatrix);
            }
            if (options.reflections && reflectionCapability->updateSurfaceTransform != nullptr
                && scene[spinMeshIndex].reflectionSurface != lumen_reflection::invalidReflectionSurface)
            {
                reflectionCapability->updateSurfaceTransform(
                    reflectionCapability->self, scene[spinMeshIndex].reflectionSurface, spinMatrix);
            }
        }

        cameraCapability.update(cameraCapability.self, cameraHandle, &cameraInput, cameraDeltaSeconds);
        camera::ViewState viewState{};
        cameraCapability.viewState(
            cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);

        renderer::CameraMatrices cameraMatrices{};
        std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
        std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
        std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
        renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

        // The GI camera tracks the render camera; evaluate() runs one full
        // Lumen-style GI frame per rendered frame. Column-major view-projection
        // straight out of glm (OpenGL convention, same as CameraMatrices).
        const glm::mat4 viewProjectionMatrix
            = glm::make_mat4(viewState.projection) * glm::make_mat4(viewState.view);
        const float viewProjection[16] = {
            viewProjectionMatrix[0][0], viewProjectionMatrix[0][1], viewProjectionMatrix[0][2], viewProjectionMatrix[0][3],
            viewProjectionMatrix[1][0], viewProjectionMatrix[1][1], viewProjectionMatrix[1][2], viewProjectionMatrix[1][3],
            viewProjectionMatrix[2][0], viewProjectionMatrix[2][1], viewProjectionMatrix[2][2], viewProjectionMatrix[2][3],
            viewProjectionMatrix[3][0], viewProjectionMatrix[3][1], viewProjectionMatrix[3][2], viewProjectionMatrix[3][3],
        };
        const float cameraPosition[3] = {viewState.position[0], viewState.position[1], viewState.position[2]};
        if (giCapability->setView != nullptr)
        {
            giCapability->setView(giCapability->self, viewProjection, cameraPosition);
        }
        // The GI and reflection integrators advance by the same clock the
        // camera moved on: the fixed 30 Hz step under --frames (deterministic
        // runs), the clamped wall-clock delta in interactive runs so their
        // temporal convergence matches the camera's actual motion speed.
        const double evaluateDeltaSeconds = cameraDeltaSeconds;
        lumen_gi::GiError giError = lumen_gi::GiError::None;
        if (giCapability->evaluate != nullptr
            && !giCapability->evaluate(giCapability->self, evaluateDeltaSeconds, &giError))
        {
            std::cerr << "demo_lumen_gi: lumen_gi evaluate failed (error "
                      << static_cast<std::uint32_t>(giError) << ")\n";
            break;
        }

        // The reflection camera tracks the render camera (the field is sized
        // from the viewport the renderer composites onto).
        if (options.reflections && reflectionCapability->setView != nullptr)
        {
            reflectionCapability->setView(reflectionCapability->self, viewProjection, cameraPosition,
                framebufferWidth, framebufferHeight);
        }
        if (options.reflections && reflectionCapability->evaluate != nullptr)
        {
            lumen_reflection::ReflectionError reflectionError = lumen_reflection::ReflectionError::None;
            if (!reflectionCapability->evaluate(reflectionCapability->self, evaluateDeltaSeconds, &reflectionError))
            {
                std::cerr << "demo_lumen_gi: lumen_reflection evaluate failed (error "
                          << static_cast<std::uint32_t>(reflectionError) << ")\n";
                break;
            }
        }

        frameTiming.markUpdateDone();
        frameTiming.markUploadDone();
        renderCapability.beginFrame(renderCapability.self, rendererHandle, nullptr);
        renderCapability.drawMeshes(
            renderCapability.self, rendererHandle, drawCommands.data(), static_cast<std::uint32_t>(drawCommands.size()));
        renderCapability.endFrame(renderCapability.self, rendererHandle);
        frameTiming.markDrawDone();
        ++framesRendered;

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
                if (options.reflections && reflectionCapability->statistics != nullptr)
                {
                    lumen_reflection::ReflectionStatistics reflectionStatistics{};
                    if (reflectionCapability->statistics(reflectionCapability->self, &reflectionStatistics)
                        && reflectionStatistics.pixelsTracedThisFrame != 0)
                    {
                        std::cout << "demo_lumen_gi: reflection field " << reflectionStatistics.fieldWidth
                                  << "x" << reflectionStatistics.fieldHeight << " | rays "
                                  << reflectionStatistics.raysTracedThisFrame << " (screen "
                                  << reflectionStatistics.screenTraceHitsThisFrame << ", scene "
                                  << reflectionStatistics.sceneTraceHitsThisFrame << "), cache lookups "
                                  << reflectionStatistics.raysReusedCacheThisFrame << ", evaluate "
                                  << reflectionStatistics.lastEvaluateMilliseconds << " ms\n";
                    }
                }
            }
        }

        if (!screenshotTaken && !options.screenshotPath.empty() && frameIndex == options.screenshotFrame)
        {
            // Headless runs export the window surface directly; Vulkan runs
            // schedule the export on the renderer's vulkan extension.
            screenshotTaken = demo::harness::exportScreenshotPpm(context,
                windowCapability,
                windowHandle,
                renderCapability,
                rendererHandle,
                options.screenshotPath);
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
                if (options.reflections && reflectionCapability->debugExport != nullptr)
                {
                    const lumen_reflection::ReflectionDebugView reflectionViews[]
                        = {lumen_reflection::ReflectionDebugView::Composite,
                            lumen_reflection::ReflectionDebugView::ReflectionsOnly,
                            lumen_reflection::ReflectionDebugView::ResolveMask,
                            lumen_reflection::ReflectionDebugView::Reference};
                    const char *reflectionViewNames[]
                        = {"reflcomposite", "reflonly", "reflresolve", "reflreference"};
                    for (std::uint32_t viewIndex = 0; viewIndex < 4; ++viewIndex)
                    {
                        const std::string path
                            = options.exportDebugPrefix + "_" + reflectionViewNames[viewIndex] + ".ppm";
                        lumen_reflection::ReflectionError exportReflectionError
                            = lumen_reflection::ReflectionError::None;
                        if (!reflectionCapability->debugExport(
                                reflectionCapability->self, reflectionViews[viewIndex], path.c_str(),
                                &exportReflectionError))
                        {
                            std::cerr << "demo_lumen_gi: reflection debug view export failed ("
                                      << static_cast<std::uint32_t>(exportReflectionError) << ")\n";
                        }
                    }
                }
            }
        }
        if (options.selfTest)
        {
            if (giCapability->statistics != nullptr)
            {
                lumen_gi::GiStatistics observed{};
                if (giCapability->statistics(giCapability->self, &observed))
                {
                    selfTestMaxIndirectRays = std::max(selfTestMaxIndirectRays, observed.indirectRaysThisFrame);
                    selfTestMaxProbeTraces = std::max(selfTestMaxProbeTraces, observed.probeTracesThisFrame);
                }
            }
            if (options.reflections && reflectionCapability->statistics != nullptr)
            {
                lumen_reflection::ReflectionStatistics observed{};
                if (reflectionCapability->statistics(reflectionCapability->self, &observed))
                {
                    selfTestMaxReflectionPixels = std::max(selfTestMaxReflectionPixels, observed.pixelsTracedThisFrame);
                    selfTestMaxTransmissionSegments = std::max(selfTestMaxTransmissionSegments, observed.transmissionSegmentsThisFrame);
                }
            }
            if (giCapability->copyProbes != nullptr)
            {
                lumen_gi::GiProbeHeader header{};
                std::uint64_t version = 0;
                giCapability->copyProbes(giCapability->self, nullptr, 0, &header, &version);
                selfTestProbeVersion = std::max(selfTestProbeVersion, version);
            }
            if (options.reflections && reflectionCapability->copyField != nullptr)
            {
                lumen_reflection::ReflectionFieldHeader header{};
                std::uint64_t version = 0;
                reflectionCapability->copyField(reflectionCapability->self, nullptr, 0, &header, &version);
                selfTestFieldVersion = std::max(selfTestFieldVersion, version);
            }
        }
        (void)framePacer.pace(&host.kernel().profiler());
    }

    if (giCapability->statistics != nullptr)
    {
        lumen_gi::GiStatistics finalStatistics{};
        if (giCapability->statistics(giCapability->self, &finalStatistics))
        {
            std::cout << "demo_lumen_gi: lumen_gi totals - frames " << finalStatistics.frameIndex << " ("
                      << finalStatistics.framesSkippedStatic << " static-skipped), surfaces "
                      << finalStatistics.surfaces << ", cards " << finalStatistics.cards << ", card texels "
                      << finalStatistics.cardTexels << ", total rays " << finalStatistics.totalRaysTraced << "\n";
        }
    }

    if (options.selfTest)
    {
        struct Check { const char *name; bool pass; };
        // The exported screenshot must carry non-uniform content: parse the
        // P6 file and look at the luminance spread.
        bool screenshotContentOk = false;
        {
            std::ifstream screenshotFile(options.screenshotPath, std::ios::binary);
            std::string   magic;
            std::size_t   width = 0;
            std::size_t   height = 0;
            std::size_t   maximum = 0;
            if (screenshotFile && (screenshotFile >> magic) && magic == "P6" && (screenshotFile >> width >> height >> maximum)
                && screenshotFile.get() == '\n' && width > 0 && height > 0)
            {
                std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
                screenshotFile.read(reinterpret_cast<char *>(pixels.data()),
                    static_cast<std::streamsize>(pixels.size()));
                if (screenshotFile.gcount() == static_cast<std::streamsize>(pixels.size()))
                {
                    float sum = 0.0f;
                    float squaredSum = 0.0f;
                    for (const unsigned char channel : pixels)
                    {
                        const float value = static_cast<float>(channel) / 255.0f;
                        sum += value;
                        squaredSum += value * value;
                    }
                    const float count = static_cast<float>(pixels.size());
                    const float mean = sum / count;
                    const float variance = squaredSum / count - mean * mean;
                    screenshotContentOk = variance > 1.0e-4f && mean > 1.0e-3f;
                }
            }
        }
        lumen_gi::GiStatistics giTotals{};
        bool giTotalsOk = giCapability->statistics != nullptr
            && giCapability->statistics(giCapability->self, &giTotals) && giTotals.surfaces == scene.size()
            && giTotals.totalRaysTraced > 0;
        const Check checks[] = {
            {"frames rendered", framesRendered == 24},
            {"screenshot exported with content", screenshotTaken && screenshotContentOk},
            {"gi surfaces + rays", giTotalsOk},
            {"gi indirect gather rays ran", selfTestMaxIndirectRays > 0},
            {"gi probe traces ran", selfTestMaxProbeTraces > 0},
            {"gi probe snapshot published", selfTestProbeVersion > 0},
            {"reflection pixels traced", selfTestMaxReflectionPixels > 0},
            {"transmission segments walked", selfTestMaxTransmissionSegments > 0},
            {"reflection field published", selfTestFieldVersion > 0},
        };
        bool allPass = true;
        for (const Check &check : checks)
        {
            std::cout << "self-test: " << (check.pass ? "PASS " : "FAIL ") << check.name << "\n";
            allPass = allPass && check.pass;
        }
        std::cout << (allPass ? "self-test: PASS\n" : "self-test: FAIL\n");
        std::remove(options.screenshotPath.c_str());
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
        host.stop();
        return allPass ? 0 : 1;
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
        if (mesh.reflectionSurface != lumen_reflection::invalidReflectionSurface
            && reflectionCapability != nullptr && reflectionCapability->destroySurface != nullptr)
        {
            reflectionCapability->destroySurface(reflectionCapability->self, mesh.reflectionSurface);
        }
    }
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    host.stop();
    std::cout << frameTiming.summary();
    if (!options.frameTimesPath.empty())
    {
        if (frameTiming.writeCsv(options.frameTimesPath))
        {
            std::cout << "demo_lumen_gi: frame timings written to " << options.frameTimesPath << "\n";
        }
        else
        {
            std::cout << "demo_lumen_gi: could not write frame timings to " << options.frameTimesPath << "\n";
        }
    }
    std::cout << "demo_lumen_gi: stopped cleanly after " << framesRendered << " frames\n";
    return 0;
}
