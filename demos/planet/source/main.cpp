// main.cpp - the planet generation demo application.
//
// A standalone harness for the planet module: it creates one procedural
// planet through the "planet" capability, streams its LOD chunk meshes into
// the renderer every frame, and exposes screenshot exports and determinism
// self-tests so generation can be validated without any manual inspection.
//
// Views (deterministic camera placement from the planet's own sampling):
//   --view globe    whole planet from space: continents, oceans, ice caps.
//   --view coast    standing on a beach looking along the shoreline.
//   --view cliffs   terraced mountain country with overhanging cliff faces.
//   --view surface  rolling green hills under the sun.
//
// Useful flags: --seed N / --radius M override the planet description,
// --frames N renders exactly N frames (deterministic), --screenshot exports
// the rendered frame as PPM, --self-test verifies chunk generation is
// deterministic and independent of chunk build order, --stats-every prints
// planet statistics cadence, --caves enables the cave worm carving.

#include <engine/application/application.hpp>
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/planet.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include "demo/planet_demo_support.hpp"
#include "demo/planet_camera_input.hpp"
#include "demo/planet_chunk_renderer.hpp"
#include "engine/application/application_host.hpp"
#include "engine/application/command_line.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
using namespace engine;
using namespace demo::planet_demo;
namespace window = engine::window;
namespace renderer = engine::renderer;
namespace planet = engine::planet;

} // namespace

int main(int argc, char **argv)
{
    std::cout << std::unitbuf;

    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << '\n'
                  << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << planetOptionsUsage();
        return 0;
    }

    PlanetDemoOptions options;
    if (!parsePlanetOptions(commandLine.applicationArguments, options, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << "\n";
        return 2;
    }

    engine::ApplicationHostOptions hostOptions
        = engine::makeApplicationHostOptions(commandLine, {"window", "renderer", "camera", "pbr", "planet"});
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
        std::cerr << startupReport << "demo_planet: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::ParticipantContext &context = host.context();
    const std::int64_t maxFps = context.configurationIntegerInSection("engine", "max_fps").value_or(0);

    window::WindowCapability   &windowCapability  = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability  = context.acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability   &cameraCapability  = context.acquireCapability<camera::CameraCapability>();
    auto *planetCapability = context.tryAcquireCapability<planet::PlanetCapability>();
    if (planetCapability == nullptr || planetCapability->abi != planet::planetCapabilityAbi)
    {
        std::cerr << "demo_planet: the planet capability is unavailable or its ABI differs; this demo is pointless "
                     "without it\n";
        host.stop();
        return 1;
    }
    const pbr::PbrCapability *pbrCapability = context.tryAcquireCapability<pbr::PbrCapability>();
    pbr::PbrEnvironment       environment{};
    if (pbrCapability != nullptr && pbrCapability->copyEnvironment(pbrCapability->self, &environment, nullptr))
    {
        std::cout << "demo_planet: pbr sun " << (environment.sun.enabled ? "enabled" : "disabled") << " (intensity "
                  << environment.sun.intensity << ")\n";
    }

    // ---- planet creation ---------------------------------------------------------
    planet::PlanetDescription description{};
    description.seed         = options.seed;
    description.radiusMeters = options.radius;
    const planet::PlanetHandle planetHandle = planetCapability->createPlanet(planetCapability->self, &description, nullptr);
    if (planetHandle == planet::invalidPlanet)
    {
        std::cerr << "demo_planet: the planet module rejected the planet description\n";
        host.stop();
        return 1;
    }
    planet::PlanetGenerationParams params{};
    planetCapability->copyGenerationParams(planetCapability->self, planetHandle, &params);
    if (options.caves)
    {
        std::cout << "demo_planet: --caves currently only toggles through configuration "
                     "(--set planet.caves_enabled=true); continuing without caves\n";
    }
    std::cout << "demo_planet: planet seed " << params.seed << ", radius " << params.radiusMeters << " m, octree depth "
              << params.derivedMaxDepth << ", leaf voxel " << params.derivedVoxelMeters << " m\n";

    // ---- self-test short-circuit ----------------------------------------------------
    if (options.selfTest)
    {
        const SelfTestResult test = runSelfTest(*planetCapability, planetCapability->self, options);
        std::cout << test.report;
        planetCapability->destroyPlanet(planetCapability->self, planetHandle);
        host.stop();
        return test.pass ? 0 : 1;
    }

    // ---- camera placement per view ----------------------------------------------------
    const glm::dvec3 planetCenter(params.centerMeters[0], params.centerMeters[1], params.centerMeters[2]);
    const double     planetRadius = params.radiusMeters;

    glm::dvec3 eye      = planetCenter + glm::dvec3(0.0, 0.0, planetRadius * 3.0);
    glm::dvec3 target   = planetCenter;
    double     nearZ    = planetRadius * 0.01;
    double     farZ     = planetRadius * 12.0;

    if (options.viewName == "globe")
    {
        const glm::dvec3 direction = glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        eye                        = planetCenter + direction * planetRadius * 3.1;
        target                     = planetCenter;
    }
    else if (options.viewName == "orbit")
    {
        // Mid-altitude view: the planet fills the frame, continents and
        // mountain ranges are readable - the scale between the globe and the
        // surface where range structure (or its absence) shows most clearly.
        const glm::dvec3 direction = glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        eye                        = planetCenter + direction * planetRadius * 1.6;
        target                     = planetCenter;
    }
    else
    {
        // Surface views: find a deterministic spot matching the view's
        // terrain predicate, stand eyeHeight above the ground, look along the
        // horizon toward the view's focus.
        Viewpoint viewpoint;
        const auto elevationWindow = [](double low, double high)
        {
            return [low, high](double elevation, engine::planet::Biome, double, const planet::PlanetGenerationParams &)
            { return elevation >= low && elevation <= high; };
        };
        // Warm-land windows: the biome check alone is not enough - the biome
        // colors BLEND into snow below the climate snow line (and toward the
        // freeze line), so a spot the enum calls "taiga" can still render
        // half-white. Require strictly warm air and a spot clearly below the
        // blended snow band so showcase views land on green/sandy terrain.
        const auto warmLandWindow = [](double low, double high)
        {
            return [low, high](double elevation, engine::planet::Biome biome, double temperature01,
                const planet::PlanetGenerationParams &params)
            {
                if (elevation < low || elevation > high)
                {
                    return false;
                }
                if (biome == planet::Biome::Snow || biome == planet::Biome::Tundra || temperature01 <= 0.45)
                {
                    return false;
                }
                const double snowLine = params.snowLineMeters * (0.25 + 0.75 * temperature01);
                const double snowBand = params.snowLineMeters * params.elevationBlendFraction;
                return elevation < snowLine - snowBand;
            };
        };
        bool found = false;
        // View placement: biome-aware, climate-aware and sightline-checked so
        // the view shows landscape rather than the nearest slope wall.
        const planet::Vec3 sunDirection = environment.sun.enabled
            ? planet::Vec3{static_cast<double>(environment.sun.direction[0]),
                static_cast<double>(environment.sun.direction[1]),
                static_cast<double>(environment.sun.direction[2])}
            : planet::Vec3{0.0, 0.0, 0.0};

        if (options.viewName == "coast")
        {
            // A warm beach seen from just above the shoreline; the sea may
            // (should) enter the frame. The window reaches 45 m: the coastal
            // flattening damps most shoreline terrain toward the waterline,
            // so a 2-18 m window nearly only ever matched tiny islets whose
            // land hides under the camera.
            found = pickViewpoint(*planetCapability, planetHandle, warmLandWindow(2.0, 45.0), ProbePolicy::Coastline, 120.0,
                sunDirection, viewpoint);
        }
        else if (options.viewName == "cliffs")
        {
            // Below the snow blend, at any elevation where terraced cliff
            // faces and overhangs live (the snow blend rides the climate
            // line, so the window adapts to temperature). Higher country than
            // the other views: rock faces and strata need real relief.
            found = pickViewpoint(*planetCapability, planetHandle, warmLandWindow(250.0, 800.0), ProbePolicy::Mountain, 140.0,
                sunDirection, viewpoint);
        }
        else // surface
        {
            // Rolling green hills: the SPOT itself must be a genuinely green
            // biome (shrubland is excluded - its pale khaki albedo photographs
            // as washed-out empty country), above the coastal-flatten band
            // (its planed-off terrain photographs as an empty sheet) and
            // below the snow blend.
            const auto warmGreenLandWindow = [warmLandWindow](double low, double high)
            {
                auto warm = warmLandWindow(low, high);
                return [warm](double elevation, engine::planet::Biome biome, double temperature01,
                    const planet::PlanetGenerationParams &params)
                {
                    // TropicalRainforest is excluded: its canopy palette is the
                    // darkest row (linear albedo ~0.09-0.13) and a front-lit
                    // frame over it still photographs as a murky near-black
                    // sheet under the demo's linear exposure.
                    const bool green = biome == planet::Biome::Grassland
                        || biome == planet::Biome::TemperateForest || biome == planet::Biome::SeasonalForest
                        || biome == planet::Biome::TemperateRainforest
                        || biome == planet::Biome::Taiga;
                    return warm(elevation, biome, temperature01, params) && green;
                };
            };
            found = pickViewpoint(*planetCapability, planetHandle, warmGreenLandWindow(60.0, 200.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Very flat or very mountainous seeds may lack a match; fall back
            // to any land-ish point rather than failing the demo.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Coastline, 110.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Last resort: the coastline gate (land AND water inside the
            // sight fan) can fail on pathological seeds; an inland land view
            // always beats an error exit.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            std::cerr << "demo_planet: no usable viewpoint found for view '" << options.viewName << "'\n";
            planetCapability->destroyPlanet(planetCapability->self, planetHandle);
            host.stop();
            return 1;
        }

        const glm::dvec3 direction(viewpoint.direction.x, viewpoint.direction.y, viewpoint.direction.z);
        const glm::dvec3 position = planetCenter + direction * viewpoint.surfaceRadius;
        {
            // Degenerate or non-finite tangent: fall back to the world-up
            // cross product below.
            const planet::Vec3 &stored = viewpoint.lookTangent;
            const bool usable = std::isfinite(stored.x) && std::isfinite(stored.y) && std::isfinite(stored.z)
                && planet::length(stored) > 0.5;
            if (!usable)
            {
                viewpoint.hasLookTangent = false;
            }
        }
        // Tangent frame at the spot: coast views get the waterward direction
        // the picker chose; everything else uses the world-up cross tangent.
        const glm::dvec3 up       = direction;
        glm::dvec3       tangent  = viewpoint.hasLookTangent
            ? glm::dvec3(viewpoint.lookTangent.x, viewpoint.lookTangent.y, viewpoint.lookTangent.z)
            : glm::normalize(glm::cross(up, glm::dvec3(0.0, 1.0, 0.0)));
        if (viewpoint.hasLookTangent)
        {
            tangent = glm::normalize(tangent);
        }
        else if (glm::length(tangent) < 0.5)
        {
            tangent = glm::normalize(glm::cross(up, glm::dvec3(1.0, 0.0, 0.0)));
        }

        // Camera eye heights MATCH the picker's altitudes: the sightline wall
        // caps are calibrated at exactly these heights, and the old mismatch
        // (camera at 220/420 m after the caps cleared 110/140 m) let walls
        // the caps never saw fill the frame.
        const double eyeHeight = options.viewName == "cliffs" ? 140.0 : (options.viewName == "surface" ? 140.0 : 120.0);
        eye                    = position + up * eyeHeight;
        const double lookAheadDistance = options.viewName == "cliffs" ? 700.0 : (options.viewName == "surface" ? 420.0 : 320.0);
        // Exact pitch aim below the local horizontal - no terrain
        // re-projection (when the ground ahead rises, a re-projected target
        // snaps onto the hillside wall and fills the frame with slope) and
        // no level tangent aim (it put the horizon mid-frame, the shoreline
        // out of it, and half the frame black sky). Per-view pitch centers
        // the subject: the coast shoreline sits ~15 deg below a 120 m eye at
        // 320 m; surface ground at 420 m sits ~15.5 deg below a 140 m eye;
        // the cliffs view aims near level AT the terraced wall the picker
        // located (the wall rises ahead and fills the center frame).
        const double aimPitchRadians = options.viewName == "cliffs" ? 0.08
            : (options.viewName == "surface" ? 0.27 : 0.26); // 4.6 / 15.5 / 14.9 deg
        target = eye + (tangent * std::cos(aimPitchRadians) - up * std::sin(aimPitchRadians)) * lookAheadDistance;
        // Ground views sit close to the terrain: a near plane scaled from
        // the eye height (55-105 m) clipped everything inside it. 15 m keeps
        // near clipping out of the frame while leaving depth precision for
        // the 120 km far plane.
        nearZ    = 15.0;
        farZ     = planetRadius * 2.0;
        // Open-framing guarantee: whatever the ground ahead does, the view
        // pitch stays in a band that keeps the horizon in frame (deep enough
        // to keep landscape in the lower half, shallow enough to keep sky
        // above it). Cliff views may pitch deeper - the risers below the eye
        // are the showcase. (The exact-pitch aim above already lands inside
        // this band; the clamp is a safety net.)
        const double maxPitchRadians = options.viewName == "cliffs" ? 0.52359877559829882 : 0.38397243543875248; // 30 deg / 22 deg
        clampCameraPitch(eye, up, target, 0.08726646259971647, maxPitchRadians); // >= 5 deg down
        std::cout << "demo_planet: view pose - eye (" << eye.x << ", " << eye.y << ", " << eye.z << ") radius "
                  << glm::length(eye - planetCenter) << ", target (" << target.x << ", " << target.y << ", "
                  << target.z << ")\n";
    }

    // ---- window / renderer / camera ------------------------------------------------
    window::WindowDescription  windowDescription{};
    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo_planet: the window module could not create a window\n";
        host.stop();
        return 1;
    }
    renderer::RenderError          renderError = renderer::RenderError::None;
    const renderer::RendererHandle rendererHandle
        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo_planet: the renderer module could not create a renderer\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    const std::string windowBackendName = windowCapability.activeBackendName(windowCapability.self);
    std::cout << "demo_planet: window backend '" << windowBackendName << "', renderer backend '"
              << renderCapability.activeBackendName(renderCapability.self, rendererHandle) << "'\n";
    windowCapability.pollEvents(windowCapability.self);

    const bool windowedBackend = windowBackendName != std::string("headless");
    if (windowedBackend)
    {
        std::cout << "demo_planet: controls - mouse looks, WASD moves, Space/Ctrl up/down, Shift sprints,\n"
                     "      arrows look, Left Alt toggles the cursor, Escape quits\n";
        if (options.orbit)
        {
            std::cout << "demo_planet: --orbit drives the camera automatically; disable it for manual movement\n";
        }
    }

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        framebufferWidth  = 960;
        framebufferHeight = 540;
    }
    renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);

    const bool macroView = options.viewName == "globe" || options.viewName == "orbit";
    glm::dvec3 initialCameraUp = macroView ? glm::dvec3(0.0, 1.0, 0.0)
                                          : glm::normalize(eye - planetCenter);
    if (macroView
        && glm::abs(glm::dot(glm::normalize(eye - planetCenter), glm::dvec3(0.0, 1.0, 0.0))) > 0.99)
    {
        // Avoid a singular look-at basis when the globe view starts near a pole.
        initialCameraUp = glm::normalize(glm::dvec3(0.05, 0.9987, 0.0));
    }

    // Camera movement scales with the view: close terrain uses walking/flying
    // speeds, while globe/orbit views need larger world-unit steps.
    const float cameraMoveSpeed = macroView
        ? std::clamp(static_cast<float>(planetRadius * (options.viewName == "globe" ? 0.05 : 0.025)), 250.0f, 50000.0f)
        : std::clamp(static_cast<float>(planetRadius * 0.001), 12.0f, 300.0f);

    const camera::CameraHandle cameraHandle = [&]()
    {
        camera::CameraParameters cameraParameters{};
        cameraParameters.position[0] = static_cast<float>(eye.x);
        cameraParameters.position[1] = static_cast<float>(eye.y);
        cameraParameters.position[2] = static_cast<float>(eye.z);
        cameraParameters.hasPosition  = true;
        cameraParameters.target[0]    = static_cast<float>(target.x);
        cameraParameters.target[1]    = static_cast<float>(target.y);
        cameraParameters.target[2]    = static_cast<float>(target.z);
        cameraParameters.hasTarget    = true;
        cameraParameters.up[0]        = static_cast<float>(initialCameraUp.x);
        cameraParameters.up[1]        = static_cast<float>(initialCameraUp.y);
        cameraParameters.up[2]        = static_cast<float>(initialCameraUp.z);
        cameraParameters.hasUp        = true;
        cameraParameters.fovYDegrees  = macroView ? 42.0f : 65.0f;
        cameraParameters.nearZ        = static_cast<float>(nearZ);
        cameraParameters.farZ         = static_cast<float>(farZ);
        cameraParameters.moveSpeed    = cameraMoveSpeed;
        cameraParameters.lookSpeed    = 90.0f;
        cameraParameters.mouseSensitivity = 0.11f;
        cameraParameters.pitchLimitDegrees = 89.0f;
        camera::CameraError cameraError = camera::CameraError::None;
        return cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    }();
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo_planet: the camera module could not create a camera\n";
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }

    cameraCapability.setControlMode(cameraCapability.self, cameraHandle, camera::CameraControlMode::Fly);
    PlanetCameraInput cameraInput(windowCapability, windowHandle, windowedBackend, !options.orbit);

    // ---- run mode -------------------------------------------------------------------
    const bool          runUntilClosed = options.frames == 0 && windowedBackend;
    const std::uint64_t totalFrames    = options.frames != 0 ? options.frames : 90;
    const double        frameDelta     = 1.0 / 30.0;
    if (options.screenshotFrame == 0)
    {
        options.screenshotFrame = totalFrames - 1;
    }

    std::cout << "demo_planet: view '" << options.viewName << "', " << totalFrames << " frames ("
              << options.warmupFrames << " warmup)\n";

    // GPU chunk residency and draw-command storage are isolated from the
    // application loop; this keeps planet streaming and camera control from
    // accumulating in one demo-level coordinator.
    PlanetChunkRenderer chunkRenderer(*planetCapability, planetHandle, renderCapability, rendererHandle, params);

    // ---- frame loop -------------------------------------------------------------------
    std::uint64_t framesRendered = 0;
    bool          screenshotTaken = false;
    double        orbitAngle      = 0.0;
    std::array<window::WindowEvent, 64> eventBuffer;
    auto lastWallClock = std::chrono::steady_clock::now();

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
        std::cout << "demo_planet: max_fps " << maxFps
                  << " left to vsync - presentation is already pacing the frame loop\n";
    }

    for (std::uint64_t frameIndex = 0; runUntilClosed || frameIndex < totalFrames; ++frameIndex)
    {
        windowCapability.pollEvents(windowCapability.self);

        std::uint32_t eventCount = 0;
        do
        {
            eventCount = windowCapability.popEvents(windowCapability.self, windowHandle, eventBuffer.data(),
                static_cast<std::uint32_t>(eventBuffer.size()));
            for (std::uint32_t eventIndex = 0; eventIndex < eventCount; ++eventIndex)
            {
                const window::WindowEvent &event = eventBuffer[eventIndex];
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
                        renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);
                    }
                }
                else
                {
                    cameraInput.handleWindowEvent(event);
                }
            }
        }
        while (eventCount == static_cast<std::uint32_t>(eventBuffer.size()));

        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            std::cout << "demo_planet: window close requested\n";
            break;
        }
        if (windowCapability.isKeyDown != nullptr
            && windowCapability.isKeyDown(windowCapability.self, windowHandle, window::Key::Escape))
        {
            std::cout << "demo_planet: escape pressed\n";
            break;
        }

        const auto   now               = std::chrono::steady_clock::now();
        const double wallDeltaSeconds = std::min(0.25, std::chrono::duration<double>(now - lastWallClock).count());
        lastWallClock                 = now;
        const double deltaSeconds     = options.frames != 0 ? frameDelta : wallDeltaSeconds;

        camera::CameraInputState cameraSample{};
        if (options.orbit)
        {
            // Deterministic automatic orbit mode remains available for captures
            // and headless runs; normal windowed runs use live keyboard/mouse input.
            orbitAngle += deltaSeconds * (options.viewName == "globe" ? 0.05 : 0.10);
            if (options.viewName == "globe")
            {
                const glm::dvec3 axis(0.0, 1.0, 0.0);
                const glm::dvec3 offset = eye - planetCenter;
                const double orbitRadius = glm::length(offset);
                const glm::dvec3 rotated = glm::rotate(glm::dmat4(1.0), orbitAngle, axis) * glm::dvec4(offset, 1.0);
                eye = planetCenter + glm::normalize(rotated) * orbitRadius;
                target = planetCenter;
            }
            else
            {
                const glm::dvec3 localUp = glm::normalize(eye - planetCenter);
                const glm::dvec3 offset = eye - target;
                const glm::dvec3 rotated = glm::rotate(glm::dmat4(1.0), orbitAngle, localUp) * glm::dvec4(offset, 1.0);
                eye = target + rotated;
            }

            const glm::dvec3 lookUp = macroView ? glm::dvec3(0.0, 1.0, 0.0)
                                                : glm::normalize(eye - planetCenter);
            const float poseEye[3] = {static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z)};
            const float poseTarget[3] = {
                static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z)};
            float poseUp[3] = {static_cast<float>(lookUp.x), static_cast<float>(lookUp.y), static_cast<float>(lookUp.z)};
            if (macroView
                && glm::abs(glm::dot(glm::normalize(eye - planetCenter), glm::dvec3(0.0, 1.0, 0.0))) > 0.99)
            {
                poseUp[0] = 0.05f;
                poseUp[1] = 0.9987f;
                poseUp[2] = 0.0f;
            }
            cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, poseUp);
        }
        else
        {
            cameraSample = cameraInput.sample();
        }
        cameraCapability.update(cameraCapability.self, cameraHandle, &cameraSample, deltaSeconds);

        camera::ViewState viewState{};
        cameraCapability.viewState(cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);
        renderer::CameraMatrices cameraMatrices{};
        std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
        std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
        std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
        renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

        // Advance the planet LOD stream for this eye and refresh GPU meshes.
        // During warmup frames the renderer stays idle: every worker builds
        // chunks at full speed instead of competing with rasterizer tile
        // tasks, so deep LOD views start their first RENDERED frame with the
        // visible set already complete (no black-hole screenshots).
        const double eyeWorld[3] = {viewState.position[0], viewState.position[1], viewState.position[2]};
        planetCapability->updatePlanet(planetCapability->self, planetHandle, eyeWorld);
        const bool warming = frameIndex < static_cast<std::uint64_t>(options.warmupFrames);
        if (warming)
        {
            continue;
        }
        chunkRenderer.updateVisibleChunks();
        chunkRenderer.rebuildDrawCommands();
        const auto &drawCommands = chunkRenderer.drawCommands();

        renderCapability.beginFrame(renderCapability.self, rendererHandle, nullptr);
        renderCapability.drawMeshes(renderCapability.self, rendererHandle, drawCommands.data(),
            static_cast<std::uint32_t>(drawCommands.size()));
        renderCapability.endFrame(renderCapability.self, rendererHandle);
        ++framesRendered;
        if (framesRendered % 100 == 0)
        {
            renderer::FrameStatistics frameStats{};
            renderCapability.frameStatistics(renderCapability.self, rendererHandle, &frameStats);
            std::cout << "demo_planet: raster stats - submitted " << frameStats.trianglesSubmitted
                      << ", rasterized " << frameStats.trianglesRasterized << "\n";
        }

        if (options.statsEveryFrames != 0 && frameIndex % options.statsEveryFrames == 0)
        {
            planet::PlanetStatistics statistics{};
            if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &statistics))
            {
                std::cout << "demo_planet: frame " << frameIndex << " | chunks visible " << statistics.chunksVisible
                          << ", ready " << statistics.chunksReady << ", building " << statistics.chunksBuilding
                          << ", cached " << statistics.chunksReadyTotal << ", triangles " << statistics.triangleCount
                          << ", depth " << statistics.octreeDepthUsed
                          << ", update " << statistics.lastUpdateMilliseconds << " ms\n";
            }
        }

        if (!screenshotTaken && !options.screenshotPath.empty() && frameIndex == options.screenshotFrame)
        {
            auto *headlessWindowCapability = context.tryAcquireCapability<window::HeadlessWindowCapability>();
            if (headlessWindowCapability != nullptr && windowBackendName == std::string("headless"))
            {
                window::WindowError exportError = window::WindowError::None;
                screenshotTaken                 = headlessWindowCapability->exportSurfacePpm(
                    headlessWindowCapability->self, windowHandle, options.screenshotPath.c_str(), &exportError);
            }
            if (!screenshotTaken)
            {
                auto *vulkanCapability = context.tryAcquireCapability<renderer::VulkanRenderCapability>();
                if (vulkanCapability != nullptr
                    && renderCapability.activeBackendName(renderCapability.self, rendererHandle) == std::string("vulkan"))
                {
                    renderer::RenderError exportError = renderer::RenderError::None;
                    screenshotTaken                   = vulkanCapability->exportFramePpm(
                        vulkanCapability->self, rendererHandle, options.screenshotPath.c_str(), &exportError);
                }
            }
            if (screenshotTaken)
            {
                std::cout << "demo_planet: screenshot written to " << options.screenshotPath << "\n";
            }
            else
            {
                std::cerr << "demo_planet: screenshot export failed for backend '" << windowBackendName << "'\n";
            }
        }
        (void)framePacer.pace(&host.kernel().profiler());
    }

    // ---- teardown -----------------------------------------------------------------------
    chunkRenderer.destroy();
    planet::PlanetStatistics finalStatistics{};
    if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &finalStatistics))
    {
        std::cout << "demo_planet: totals - chunks cached " << finalStatistics.chunksReadyTotal << ", triangles "
                  << finalStatistics.triangleCount << ", vertices " << finalStatistics.verticesTotal << "\n";
    }
    planetCapability->destroyPlanet(planetCapability->self, planetHandle);
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    host.stop();
    std::cout << "demo_planet: stopped cleanly after " << framesRendered << " frames\n";
    return 0;
}
