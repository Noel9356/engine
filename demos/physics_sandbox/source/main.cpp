// main.cpp - the physics sandbox demo application.
//
// An interactive harness for the physics_rigid, physics_articulated, and
// physics_cloth modules: one rigid world holds static scenery, dynamic
// primitives, an articulated chain and a drive-controlled pendulum figure,
// and cloth sheets, all stepped on a fixed-timestep clock with render-side
// interpolation. The cloth sheets render as continuous fabric surfaces: the
// simulated particle grid doubles as the render mesh, refreshed each frame
// through the renderer's dynamic vertex update and drawn double-sided with
// the fabric sheen material.
//
// Interaction (windowed runs):
//   Left mouse       grab and drag the body or cloth particle under the cursor
//   Mouse + Q/E      orbit and zoom the camera (cursor capture on start)
//   1 / 2 / 3        spawn a box / sphere / capsule in front of the camera
//   B                radial impulse blast at the camera target
//   W                toggle cloth wind
//   R                reset the scene
//   P or Space       pause / resume the simulation
//   V                cycle the physics fixed rate (30 / 60 / 120 Hz)
//   Escape           quit
//
// Useful flags: --frames N renders exactly N frames (headless captures),
// --screenshot PATH exports a frame as PPM, --scripted runs the deterministic
// capture timeline (spawns, impulses, paddle sweep, wind), --warmup-seconds
// pre-simulates before the first rendered frame, and --self-test runs the
// determinism and frame-rate independence battery without a renderer.

#include <engine/application/application_host.hpp>
#include <engine/application/command_line.hpp>
#include <engine/application/frame_clock.hpp>
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/physics_fur/capabilities.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_character/capabilities.hpp>
#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_foliage/capabilities.hpp>
#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_particles/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/physics_rope/capabilities.hpp>
#include <engine/physics_water/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include "demo/sandbox_extended.hpp"
#include "demo/sandbox_input.hpp"
#include "demo/sandbox_options.hpp"
#include "demo/sandbox_render.hpp"
#include "demo/sandbox_scene.hpp"
#include "demo/sandbox_selftest.hpp"
#include "demo_harness.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{

using namespace engine;
namespace physics_rigid       = engine::physics_rigid;
namespace physics_articulated = engine::physics_articulated;
namespace physics_cloth       = engine::physics_cloth;
namespace physics_fields      = engine::physics_fields;
namespace physics_rope        = engine::physics_rope;
namespace physics_water       = engine::physics_water;
namespace physics_fluid       = engine::physics_fluid;
namespace physics_particles   = engine::physics_particles;
namespace physics_foliage     = engine::physics_foliage;
namespace physics_character   = engine::physics_character;
namespace physics_scheduling  = engine::physics_scheduling;
namespace renderer            = engine::renderer;
namespace window              = engine::window;
namespace camera              = engine::camera;
namespace math                = engine::physics_rigid::math;

using demo::physics_sandbox::SceneDependencies;
using demo::physics_sandbox::SceneHandles;

struct GrabState
{
    physics_rigid::BodyHandle body = physics_rigid::invalidBody;
    math::Vec3                localAnchor;
    double                    distance = 0.0;

    physics_cloth::ClothHandle cloth         = physics_cloth::invalidCloth;
    std::uint32_t              clothParticle = 0;

    bool active = false;
};

// Point-to-ray distance for cloth particle picking.
double pointRayDistance(const math::Vec3 &point, const math::Vec3 &origin, const math::Vec3 &direction)
{
    const math::Vec3 delta = point - origin;
    const double     along = math::dot(delta, direction);
    if (along < 0.0)
    {
        return math::length(delta);
    }
    return math::length(delta - direction * along);
}

} // namespace

int main(int argc, char **argv)
{
    std::cout << std::unitbuf;

    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo_physics_sandbox: " << commandLineError << '\n'
                  << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << demo::physics_sandbox::sandboxOptionsUsage();
        return 0;
    }

    demo::physics_sandbox::SandboxOptions options;
    if (!demo::physics_sandbox::parseSandboxOptions(commandLine.applicationArguments, options, commandLineError))
    {
        std::cerr << "demo_physics_sandbox: " << commandLineError << '\n';
        return 2;
    }

    // Self-test runs without the presentation stack: only the physics
    // modules are required.
    std::vector<std::string> requiredCapabilities;
    if (options.selfTest)
    {
        requiredCapabilities = {"physics_rigid", "physics_articulated", "physics_cloth", "physics_fields",
            "physics_rope", "physics_water", "physics_fluid", "physics_particles", "physics_foliage",
            "physics_character", "physics_scheduling"};
    }
    else
    {
        requiredCapabilities = {"window", "renderer", "camera", "pbr", "physics_rigid", "physics_articulated",
            "physics_cloth", "physics_fields", "physics_rope", "physics_water", "physics_fluid",
            "physics_particles", "physics_foliage", "physics_character", "physics_scheduling", "physics_fur"};
    }
    if (options.fixedHz > 0.0)
    {
        commandLine.configurationOverrides.push_back("physics_rigid.fixed_rate_hz=" + std::to_string(options.fixedHz));
        commandLine.configurationOverrides.push_back("physics_cloth.fixed_rate_hz=" + std::to_string(options.fixedHz));
        commandLine.configurationOverrides.push_back("physics_rope.fixed_rate_hz=" + std::to_string(options.fixedHz));
        commandLine.configurationOverrides.push_back("physics_fluid.fixed_rate_hz=" + std::to_string(options.fixedHz));
        commandLine.configurationOverrides.push_back("physics_particles.fixed_rate_hz=" + std::to_string(options.fixedHz));
        commandLine.configurationOverrides.push_back("physics_foliage.fixed_rate_hz=" + std::to_string(options.fixedHz));
    }

    engine::ApplicationHostOptions hostOptions = engine::makeApplicationHostOptions(commandLine, requiredCapabilities);
    engine::ApplicationHost        host;
    std::string                    startupReport;
    if (commandLine.validateOnly)
    {
        const bool valid = host.validate(hostOptions, startupReport);
        std::cout << startupReport;
        std::cout << (valid ? "validate: ok\n" : "validate: failed\n");
        return valid ? 0 : 1;
    }
    if (!host.start(hostOptions, startupReport))
    {
        std::cerr << startupReport << "demo_physics_sandbox: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::ParticipantContext &context = host.context();

    SceneDependencies dependencies;
    dependencies.rigid = context.tryAcquireCapability<physics_rigid::PhysicsRigidCapability>();
    if (dependencies.rigid != nullptr && dependencies.rigid->abi != physics_rigid::physicsRigidCapabilityAbi)
    {
        dependencies.rigid = nullptr;
    }
    if (!options.noArticulation)
    {
        dependencies.articulation = context.tryAcquireCapability<physics_articulated::ArticulatedCapability>();
        if (dependencies.articulation != nullptr
            && dependencies.articulation->abi != physics_articulated::physicsArticulatedCapabilityAbi)
        {
            dependencies.articulation = nullptr;
        }
    }
    if (!options.noCloth)
    {
        dependencies.cloth = context.tryAcquireCapability<physics_cloth::ClothCapability>();
        if (dependencies.cloth != nullptr && dependencies.cloth->abi != physics_cloth::physicsClothCapabilityAbi)
        {
            dependencies.cloth = nullptr;
        }
    }
    {
        dependencies.fields = context.tryAcquireCapability<physics_fields::FieldsCapability>();
        if (dependencies.fields != nullptr && dependencies.fields->abi != physics_fields::physicsFieldsCapabilityAbi)
        {
            dependencies.fields = nullptr;
        }
        dependencies.rope = context.tryAcquireCapability<physics_rope::RopeCapability>();
        if (dependencies.rope != nullptr && dependencies.rope->abi != physics_rope::physicsRopeCapabilityAbi)
        {
            dependencies.rope = nullptr;
        }
        dependencies.water = context.tryAcquireCapability<physics_water::WaterCapability>();
        if (dependencies.water != nullptr && dependencies.water->abi != physics_water::physicsWaterCapabilityAbi)
        {
            dependencies.water = nullptr;
        }
        dependencies.fluid = context.tryAcquireCapability<physics_fluid::FluidCapability>();
        if (dependencies.fluid != nullptr && dependencies.fluid->abi != physics_fluid::physicsFluidCapabilityAbi)
        {
            dependencies.fluid = nullptr;
        }
        dependencies.particles = context.tryAcquireCapability<physics_particles::ParticlesCapability>();
        if (dependencies.particles != nullptr
            && dependencies.particles->abi != physics_particles::physicsParticlesCapabilityAbi)
        {
            dependencies.particles = nullptr;
        }
        dependencies.foliage = context.tryAcquireCapability<physics_foliage::FoliageCapability>();
        if (dependencies.foliage != nullptr && dependencies.foliage->abi != physics_foliage::physicsFoliageCapabilityAbi)
        {
            dependencies.foliage = nullptr;
        }
        dependencies.character = context.tryAcquireCapability<physics_character::CharacterCapability>();
        if (dependencies.character != nullptr
            && dependencies.character->abi != physics_character::physicsCharacterCapabilityAbi)
        {
            dependencies.character = nullptr;
        }
        dependencies.scheduling = context.tryAcquireCapability<physics_scheduling::SchedulingCapability>();
        if (dependencies.scheduling != nullptr
            && dependencies.scheduling->abi != physics_scheduling::physicsSchedulingCapabilityAbi)
        {
            dependencies.scheduling = nullptr;
        }
    }
    const engine::physics_fur::FurCapability *furCapability = context.tryAcquireCapability<engine::physics_fur::FurCapability>();
    if (furCapability != nullptr && furCapability->abi != engine::physics_fur::physicsFurCapabilityAbi)
    {
        furCapability = nullptr;
    }
    if (dependencies.rigid == nullptr)
    {
        std::cerr << "demo_physics_sandbox: the physics_rigid capability is unavailable or its ABI differs\n";
        host.stop();
        return 1;
    }

    const demo::physics_sandbox::SceneConfig sceneConfig = demo::physics_sandbox::readSceneConfig(context);
    const double                             fixedHz
        = options.fixedHz > 0.0
            ? options.fixedHz
            : context.configurationFloatingInSection("physics_sandbox", "fixed_rate_hz").value_or(60.0);
    const double fixedDelta = 1.0 / fixedHz;

    // ---- self-test short-circuit ---------------------------------------------
    if (options.selfTest)
    {
        const demo::physics_sandbox::SelfTestResult test
            = demo::physics_sandbox::runSelfTest(dependencies, sceneConfig);
        std::cout << test.report;
        const demo::physics_sandbox::SelfTestResult extendedTest
            = demo::physics_sandbox::runSelfTestExtended(dependencies, sceneConfig);
        std::cout << extendedTest.report;
        const bool allPass = test.pass && extendedTest.pass;
        std::cout << (allPass ? "self-test: PASS\n" : "self-test: FAIL\n");
        host.stop();
        return allPass ? 0 : 1;
    }

    const std::int64_t maxFps = context.configurationIntegerInSection("engine", "max_fps").value_or(0);

    window::WindowCapability   &windowCapability = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability = context.acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability   &cameraCapability = context.acquireCapability<camera::CameraCapability>();

    // ---- window and renderer ---------------------------------------------------
    window::WindowDescription windowDescription;
    windowDescription.title                = "physics sandbox";
    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo_physics_sandbox: window creation failed (error " << static_cast<std::uint32_t>(windowError)
                  << ")\n";
        host.stop();
        return 1;
    }
    const std::string windowBackendName = windowCapability.activeBackendName(windowCapability.self);
    const bool        windowedBackend   = windowBackendName != "headless";

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        framebufferWidth  = 960;
        framebufferHeight = 540;
    }
    renderCapability.setViewport(renderCapability.self, renderer::invalidRenderer, framebufferWidth, framebufferHeight);

    renderer::RenderError          renderError = renderer::RenderError::None;
    const renderer::RendererHandle rendererHandle
        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo_physics_sandbox: renderer creation failed (error " << static_cast<std::uint32_t>(renderError)
                  << ")\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }

    camera::CameraParameters cameraParameters;
    cameraParameters.position[0]           = 15.0f;
    cameraParameters.position[1]           = 8.0f;
    cameraParameters.position[2]           = 15.0f;
    cameraParameters.target[0]             = 0.0f;
    cameraParameters.target[1]             = 1.8f;
    cameraParameters.target[2]             = 0.0f;
    cameraParameters.up[0]                 = 0.0f;
    cameraParameters.up[1]                 = 1.0f;
    cameraParameters.up[2]                 = 0.0f;
    cameraParameters.fovYDegrees           = 55.0f;
    cameraParameters.nearZ                 = 0.1f;
    cameraParameters.farZ                  = 220.0f;
    camera::CameraError        cameraError = camera::CameraError::None;
    const camera::CameraHandle cameraHandle
        = cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo_physics_sandbox: camera creation failed\n";
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    cameraCapability.setControlMode(cameraCapability.self,
        cameraHandle,
        options.cameraMode == "free" ? camera::CameraControlMode::Fly : camera::CameraControlMode::Orbit);
    demo::physics_sandbox::SandboxInput sandboxInput(windowCapability, windowHandle, windowedBackend);
    if (options.cameraMode == "fixed")
    {
        cameraCapability.setControlMode(cameraCapability.self, cameraHandle, camera::CameraControlMode::Fixed);
    }

    // ---- scene -----------------------------------------------------------------
    SceneHandles scene;
    std::string  sceneReport;
    if (!demo::physics_sandbox::buildScene(dependencies, sceneConfig, options.sceneName, scene, sceneReport))
    {
        std::cerr << "demo_physics_sandbox: scene build failed\n" << sceneReport;
        cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    std::cout << "demo_physics_sandbox: scene '" << options.sceneName << "' built: " << scene.bodies.size()
              << " bodies, " << scene.figures.size() << " figures, " << scene.cloths.size() << " cloth sheets\n";

    // Overlap trigger around the action zone: the scripted spawns, blasts,
    // and the paddle sweep all cross it, so the stats line reports live
    // enter/exit transitions from the trigger surface.
    physics_rigid::TriggerHandle zoneTrigger = physics_rigid::invalidTrigger;
    std::uint64_t                triggerEnters = 0;
    std::uint64_t                triggerExits  = 0;
    {
        physics_rigid::TriggerDescription zoneDescription;
        zoneDescription.shape.kind              = physics_rigid::ShapeKind::Box;
        zoneDescription.shape.boxHalfExtents[0] = 2.5;
        zoneDescription.shape.boxHalfExtents[1] = 1.5;
        zoneDescription.shape.boxHalfExtents[2] = 2.5;
        zoneDescription.transform.position      = physics_rigid::Vec3{0.0, 1.5, 0.0};
        zoneDescription.userTag                 = 1; // the action zone
        physics_rigid::PhysicsError triggerError = physics_rigid::PhysicsError::None;
        zoneTrigger = dependencies.rigid->createTrigger(
            dependencies.rigid->self, scene.world, &zoneDescription, &triggerError);
        if (zoneTrigger != physics_rigid::invalidTrigger)
        {
            std::cout << "demo_physics_sandbox: action-zone trigger created (handle " << zoneTrigger << ")\n";
        }
    }
    for (std::size_t figureIndex = 0; figureIndex < scene.figures.size(); ++figureIndex)
    {
        physics_rigid::BodyHandle       rootLink = physics_rigid::invalidBody;
        physics_articulated::FigureInfo figureInfo{};
        if (dependencies.articulation != nullptr
            && dependencies.articulation->figureInfo(
                dependencies.articulation->self, scene.figures[figureIndex], &figureInfo)
            && dependencies.articulation->linkBodyHandle(
                dependencies.articulation->self, scene.figures[figureIndex], 0, &rootLink))
        {
            physics_rigid::BodyState rootState{};
            dependencies.rigid->copyBodyState(dependencies.rigid->self, scene.world, rootLink, &rootState);
            std::cout << "demo_physics_sandbox: figure " << figureIndex << ": " << figureInfo.linkCount << " links, "
                      << figureInfo.jointCount << " joints, root at (" << rootState.transform.position.x << ", "
                      << rootState.transform.position.y << ", " << rootState.transform.position.z << ")\n";
        }
    }

    demo::physics_sandbox::SandboxRenderer sandboxRenderer;
    std::string                            meshReport;
    if (!sandboxRenderer.createMeshes(renderCapability, rendererHandle, meshReport))
    {
        std::cerr << "demo_physics_sandbox: " << meshReport;
        demo::physics_sandbox::destroyScene(dependencies, scene);
        cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    sandboxRenderer.syncScene(renderCapability, rendererHandle, dependencies, scene, dependencies.cloth);
    sandboxRenderer.prepareFur(renderCapability, rendererHandle, furCapability);

    // ---- warmup ------------------------------------------------------------------
    double simulatedSeconds = 0.0;
    if (options.warmupSeconds > 0.0)
    {
        const std::uint32_t warmupSteps = static_cast<std::uint32_t>(options.warmupSeconds / fixedDelta);
        for (std::uint32_t step = 0; step < warmupSteps; ++step)
        {
            if (options.scripted)
            {
                demo::physics_sandbox::updateScriptedTimeline(
                    dependencies, sceneConfig, scene, simulatedSeconds, fixedDelta);
                demo::physics_sandbox::updateExtendedScriptedTimeline(
                    dependencies, scene, simulatedSeconds, fixedDelta);
                for (const physics_articulated::FigureHandle figure : scene.figures)
                {
                    if (dependencies.articulation != nullptr)
                    {
                        dependencies.articulation->updateDrives(dependencies.articulation->self, figure, fixedDelta);
                    }
                }
            }
            demo::physics_sandbox::stepExtendedBeforeRigid(dependencies, scene, simulatedSeconds, fixedDelta);
            dependencies.rigid->step(dependencies.rigid->self, scene.world, fixedDelta);
            if (dependencies.cloth != nullptr)
            {
                for (const physics_cloth::ClothHandle cloth : scene.cloths)
                {
                    dependencies.cloth->step(dependencies.cloth->self, cloth, fixedDelta);
                }
            }
            demo::physics_sandbox::stepExtendedAfterRigid(dependencies, scene, simulatedSeconds, fixedDelta);
            sandboxRenderer.recordFixedStep(dependencies, scene, dependencies.cloth);
            sandboxRenderer.recordExtendedFixedStep(dependencies, scene, fixedDelta);
            simulatedSeconds += fixedDelta;
        }
        std::cout << "demo_physics_sandbox: warmup simulated " << simulatedSeconds << " s (" << warmupSteps
                  << " fixed steps)\n";
    }

    // ---- run mode -----------------------------------------------------------------
    const bool          runUntilClosed = options.frames == 0 && windowedBackend;
    const std::uint64_t totalFrames    = options.frames != 0 ? options.frames : 120;
    if (options.screenshotFrame == demo::physics_sandbox::SandboxOptions::kScreenshotFrameUnset
        && !options.screenshotPath.empty())
    {
        options.screenshotFrame = totalFrames - 1;
    }
    std::cout << "demo_physics_sandbox: physics at " << fixedHz << " Hz, " << totalFrames << " frames ("
              << (options.scripted ? "scripted" : "static") << ")\n";

    FrameClockOptions clockOptions;
    clockOptions.fixedUpdateHz           = fixedHz;
    clockOptions.maxFrameDeltaSeconds    = 0.25;
    clockOptions.maxFixedUpdatesPerFrame = 8;
    FrameClock frameClock(clockOptions);
    frameClock.reset();

    FramePacerOptions pacerOptions;
    pacerOptions.targetFrameRateHz = maxFps > 0 ? static_cast<double>(maxFps) : 0.0;
    FramePacer framePacer(pacerOptions);
    framePacer.reset();

    bool          paused            = options.pause;
    double        windStrength      = 0.0;
    double        fixedRateCycle[3] = {30.0, 60.0, 120.0};
    std::uint32_t fixedRateIndex    = 0;
    for (std::uint32_t index = 0; index < 3; ++index)
    {
        if (std::abs(fixedRateCycle[index] - fixedHz) < 1.0e-9)
        {
            fixedRateIndex = index;
        }
    }

    GrabState                          grab;
    std::vector<window::WindowEvent>   eventBuffer(64);
    bool                               screenshotTaken = false;
    bool                               quitRequested   = false;
    demo::harness::FrameTimingRecorder frameTiming;
    const auto                         runStartClock = FrameClock::Clock::now();

    // ---- frame loop ------------------------------------------------------------------
    for (std::uint64_t frameIndex = 0; runUntilClosed || frameIndex < totalFrames; ++frameIndex)
    {
        frameTiming.beginFrame();
        windowCapability.pollEvents(windowCapability.self);
        demo::harness::drainWindowEvents(windowCapability,
            windowHandle,
            eventBuffer,
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
                else
                {
                    sandboxInput.handleWindowEvent(event);
                }
            });
        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            std::cout << "demo_physics_sandbox: window close requested\n";
            break;
        }

        for (const demo::physics_sandbox::SandboxAction action : sandboxInput.drainActions())
        {
            switch (action)
            {
            case demo::physics_sandbox::SandboxAction::SpawnBox:
            case demo::physics_sandbox::SandboxAction::SpawnSphere:
            case demo::physics_sandbox::SandboxAction::SpawnCapsule: {
                camera::CameraPose pose{};
                if (cameraCapability.cameraPose(cameraCapability.self, cameraHandle, &pose))
                {
                    const double posePosition[3] = {pose.position[0], pose.position[1], pose.position[2]};
                    const double poseForward[3]  = {pose.forward[0], pose.forward[1], pose.forward[2]};
                    demo::physics_sandbox::PrimitiveKind kind
                        = action == demo::physics_sandbox::SandboxAction::SpawnBox
                            ? demo::physics_sandbox::PrimitiveKind::Box
                        : action == demo::physics_sandbox::SandboxAction::SpawnSphere
                            ? demo::physics_sandbox::PrimitiveKind::Sphere
                            : demo::physics_sandbox::PrimitiveKind::Capsule;
                    demo::physics_sandbox::spawnPrimitive(
                        dependencies, sceneConfig, scene, kind, posePosition, poseForward);
                    sandboxRenderer.syncScene(renderCapability, rendererHandle, dependencies, scene, dependencies.cloth);
                }
                break;
            }
            case demo::physics_sandbox::SandboxAction::Blast: {
                const double origin[3] = {0.0, 1.2, 0.0};
                demo::physics_sandbox::applyBlast(dependencies, sceneConfig, scene, origin);
                break;
            }
            case demo::physics_sandbox::SandboxAction::ToggleWind: {
                windStrength = windStrength > 0.0 ? 0.0 : sceneConfig.windStrength;
                if (dependencies.cloth != nullptr)
                {
                    const double wind[3] = {windStrength, windStrength * 0.2, windStrength * 0.4};
                    for (const physics_cloth::ClothHandle cloth : scene.cloths)
                    {
                        dependencies.cloth->setWindAcceleration(dependencies.cloth->self, cloth, wind);
                    }
                }
                std::cout << "demo_physics_sandbox: cloth wind " << (windStrength > 0.0 ? "on" : "off") << "\n";
                break;
            }
            case demo::physics_sandbox::SandboxAction::ResetScene:
                demo::physics_sandbox::destroyScene(dependencies, scene);
                if (!demo::physics_sandbox::buildScene(
                        dependencies, sceneConfig, options.sceneName, scene, sceneReport))
                {
                    std::cerr << "demo_physics_sandbox: scene rebuild failed\n" << sceneReport;
                    quitRequested = true;
                    break;
                }
                sandboxRenderer.syncScene(renderCapability, rendererHandle, dependencies, scene, dependencies.cloth);
                grab             = GrabState{};
                simulatedSeconds = 0.0;
                std::cout << "demo_physics_sandbox: scene reset\n";
                break;
            case demo::physics_sandbox::SandboxAction::TogglePause:
                paused = !paused;
                std::cout << "demo_physics_sandbox: " << (paused ? "paused" : "resumed") << "\n";
                break;
            case demo::physics_sandbox::SandboxAction::CycleFixedRate: {
                fixedRateIndex             = (fixedRateIndex + 1) % 3;
                const double nextRate      = fixedRateCycle[fixedRateIndex];
                clockOptions.fixedUpdateHz = nextRate;
                frameClock                 = FrameClock(clockOptions);
                frameClock.reset();
                std::cout << "demo_physics_sandbox: physics rate switched to " << nextRate << " Hz (the same "
                          << "wall clock now advances the simulation with a different fixed step)\n";
                break;
            }
            case demo::physics_sandbox::SandboxAction::Quit:
                quitRequested = true;
                break;
            case demo::physics_sandbox::SandboxAction::None:
            default:
                break;
            }
            if (quitRequested)
            {
                break;
            }
        }
        if (quitRequested)
        {
            std::cout << "demo_physics_sandbox: quit requested\n";
            break;
        }

        // Grab picking runs at frame rate; the spring itself runs per fixed
        // step inside the physics loop.
        if (sandboxInput.grabPressed())
        {
            float rayOrigin[3]    = {0.0f, 0.0f, 0.0f};
            float rayDirection[3] = {0.0f, 0.0f, 1.0f};
            if (cameraCapability.viewportToWorldRay(cameraCapability.self,
                    cameraHandle,
                    framebufferWidth,
                    framebufferHeight,
                    static_cast<float>(sandboxInput.cursorX()),
                    static_cast<float>(sandboxInput.cursorY()),
                    rayOrigin,
                    rayDirection))
            {
                bool grabbed = false;
                if (dependencies.rigid != nullptr)
                {
                    const double          pickOrigin[3]    = {rayOrigin[0], rayOrigin[1], rayOrigin[2]};
                    const double          pickDirection[3] = {rayDirection[0], rayDirection[1], rayDirection[2]};
                    physics_rigid::RayHit hit{};
                    if (dependencies.rigid->rayCast(
                            dependencies.rigid->self, scene.world, pickOrigin, pickDirection, 120.0, &hit)
                        && hit.body != physics_rigid::invalidBody)
                    {
                        physics_rigid::BodyState state{};
                        if (dependencies.rigid->copyBodyState(dependencies.rigid->self, scene.world, hit.body, &state))
                        {
                            grab.body     = hit.body;
                            grab.distance = hit.distance;
                            const math::Vec3 worldAnchor{hit.point.x, hit.point.y, hit.point.z};
                            grab.localAnchor
                                = math::inverseRotate(state.transform.rotation, worldAnchor - state.transform.position);
                            grab.cloth  = physics_cloth::invalidCloth;
                            grab.active = true;
                            grabbed     = true;
                        }
                    }
                }
                if (!grabbed && dependencies.cloth != nullptr)
                {
                    // Cloth picking: nearest particle within the grab radius
                    // of the camera ray.
                    const math::Vec3 origin{rayOrigin[0], rayOrigin[1], rayOrigin[2]};
                    const math::Vec3 direction
                        = math::normalized(math::Vec3{rayDirection[0], rayDirection[1], rayDirection[2]});
                    double bestDistance = 0.45;
                    bool   found        = false;
                    for (const physics_cloth::ClothHandle cloth : scene.cloths)
                    {
                        physics_cloth::ClothInfo info{};
                        if (!dependencies.cloth->clothInfo(dependencies.cloth->self, cloth, &info))
                        {
                            continue;
                        }
                        std::vector<double> positions(static_cast<std::size_t>(info.particleCount) * 3);
                        if (!dependencies.cloth->copyParticlePositions(
                                dependencies.cloth->self, cloth, positions.data(), info.particleCount * 3u, nullptr))
                        {
                            continue;
                        }
                        for (std::uint32_t particle = 0; particle < info.particleCount; ++particle)
                        {
                            const math::Vec3 point{
                                positions[particle * 3], positions[particle * 3 + 1], positions[particle * 3 + 2]};
                            const double distance = pointRayDistance(point, origin, direction);
                            if (distance < bestDistance)
                            {
                                bestDistance       = distance;
                                grab.cloth         = cloth;
                                grab.clothParticle = particle;
                                grab.distance      = math::length(point - origin);
                                found              = true;
                            }
                        }
                    }
                    if (found)
                    {
                        physics_cloth::ClothParticleDrag drag;
                        drag.particleIndex       = grab.clothParticle;
                        drag.stiffness           = 380.0;
                        drag.damping             = 9.0;
                        drag.maximumAcceleration = 420.0;
                        dependencies.cloth->beginParticleDrag(dependencies.cloth->self, grab.cloth, &drag);
                        grab.body   = physics_rigid::invalidBody;
                        grab.active = true;
                    }
                }
            }
        }
        if (sandboxInput.grabReleased())
        {
            if (grab.active && grab.cloth != physics_cloth::invalidCloth && dependencies.cloth != nullptr)
            {
                dependencies.cloth->endParticleDrag(dependencies.cloth->self, grab.cloth);
            }
            grab = GrabState{};
        }
        sandboxInput.clearGrabEdges();

        // The FrameClock consumes time: real wall clock in windowed runs, a
        // synthetic 30 Hz clock in headless captures so they are repeatable.
        FrameTiming timing;
        if (options.frames != 0 && !windowedBackend)
        {
            const auto syntheticNow = runStartClock
                                    + std::chrono::duration_cast<FrameClock::Clock::duration>(
                                        std::chrono::duration<double>(static_cast<double>(frameIndex + 1) / 30.0));
            timing                  = frameClock.advance(syntheticNow);
        }
        else
        {
            timing = frameClock.advance();
        }

        // ---- fixed updates -------------------------------------------------------
        std::uint64_t stepsExecuted = 0;
        if (!paused)
        {
            for (std::uint32_t step = 0; step < timing.fixedUpdateCount; ++step)
            {
                if (options.scripted)
                {
                    demo::physics_sandbox::updateScriptedTimeline(
                        dependencies, sceneConfig, scene, simulatedSeconds, fixedDelta);
                    demo::physics_sandbox::updateExtendedScriptedTimeline(
                        dependencies, scene, simulatedSeconds, fixedDelta);
                }
                if (options.sceneName == "lod" && sceneConfig.driveSimulationLod)
                {
                    // Warmup runs before the first rendered frame; the
                    // camera still reports its creation pose here.
                    camera::CameraPose lodCamera{};
                    if (cameraCapability.cameraPose(cameraCapability.self, cameraHandle, &lodCamera))
                    {
                        const double lodCameraPosition[3]
                            = {lodCamera.position[0], lodCamera.position[1], lodCamera.position[2]};
                        demo::physics_sandbox::driveSimulationLod(dependencies, scene, sceneConfig, lodCameraPosition);
                    }
                }
                if (options.sceneName == "lod" && sceneConfig.driveSimulationLod)
                {
                    camera::CameraPose lodCamera{};
                    if (cameraCapability.cameraPose(cameraCapability.self, cameraHandle, &lodCamera))
                    {
                        const double lodCameraPosition[3]
                            = {lodCamera.position[0], lodCamera.position[1], lodCamera.position[2]};
                        demo::physics_sandbox::driveSimulationLod(dependencies, scene, sceneConfig, lodCameraPosition);
                    }
                }
                // Grab spring for rigid bodies: forces persist exactly one
                // step, so re-apply each fixed update toward the current
                // pointer ray.
                if (grab.active && grab.body != physics_rigid::invalidBody)
                {
                    float rayOrigin[3]    = {0.0f, 0.0f, 0.0f};
                    float rayDirection[3] = {0.0f, 0.0f, 1.0f};
                    if (cameraCapability.viewportToWorldRay(cameraCapability.self,
                            cameraHandle,
                            framebufferWidth,
                            framebufferHeight,
                            static_cast<float>(sandboxInput.cursorX()),
                            static_cast<float>(sandboxInput.cursorY()),
                            rayOrigin,
                            rayDirection))
                    {
                        physics_rigid::BodyState state{};
                        if (dependencies.rigid->copyBodyState(dependencies.rigid->self, scene.world, grab.body, &state))
                        {
                            const math::Vec3 origin{rayOrigin[0], rayOrigin[1], rayOrigin[2]};
                            const math::Vec3 direction
                                = math::normalized(math::Vec3{rayDirection[0], rayDirection[1], rayDirection[2]});
                            const math::Vec3 target = origin + direction * grab.distance;
                            const math::Vec3 worldAnchor
                                = state.transform.position + math::rotate(state.transform.rotation, grab.localAnchor);
                            const math::Vec3 anchorVelocity
                                = state.linearVelocity
                                + math::cross(state.angularVelocity, worldAnchor - state.transform.position);
                            math::Vec3   force          = (target - worldAnchor) * sceneConfig.grabStiffness
                                                        - anchorVelocity * sceneConfig.grabDamping;
                            const double forceMagnitude = math::length(force);
                            if (forceMagnitude > sceneConfig.grabMaximumForce)
                            {
                                force = force * (sceneConfig.grabMaximumForce / forceMagnitude);
                            }
                            const double forceArray[3]  = {force.x, force.y, force.z};
                            const double anchorArray[3] = {worldAnchor.x, worldAnchor.y, worldAnchor.z};
                            dependencies.rigid->applyForceAtPoint(
                                dependencies.rigid->self, scene.world, grab.body, forceArray, anchorArray);
                        }
                    }
                }
                if (grab.active && grab.cloth != physics_cloth::invalidCloth && dependencies.cloth != nullptr)
                {
                    float rayOrigin[3]    = {0.0f, 0.0f, 0.0f};
                    float rayDirection[3] = {0.0f, 0.0f, 1.0f};
                    if (cameraCapability.viewportToWorldRay(cameraCapability.self,
                            cameraHandle,
                            framebufferWidth,
                            framebufferHeight,
                            static_cast<float>(sandboxInput.cursorX()),
                            static_cast<float>(sandboxInput.cursorY()),
                            rayOrigin,
                            rayDirection))
                    {
                        const math::Vec3 origin{rayOrigin[0], rayOrigin[1], rayOrigin[2]};
                        const math::Vec3 direction
                            = math::normalized(math::Vec3{rayDirection[0], rayDirection[1], rayDirection[2]});
                        const math::Vec3 target         = origin + direction * grab.distance;
                        const double     targetArray[3] = {target.x, target.y, target.z};
                        dependencies.cloth->setParticleDragTarget(dependencies.cloth->self, grab.cloth, targetArray);
                    }
                }
                for (const physics_articulated::FigureHandle figure : scene.figures)
                {
                    if (dependencies.articulation != nullptr)
                    {
                        dependencies.articulation->updateDrives(dependencies.articulation->self, figure, fixedDelta);
                    }
                }
                demo::physics_sandbox::stepExtendedBeforeRigid(dependencies, scene, simulatedSeconds, fixedDelta);
                dependencies.rigid->step(dependencies.rigid->self, scene.world, fixedDelta);
                if (dependencies.cloth != nullptr)
                {
                    for (const physics_cloth::ClothHandle cloth : scene.cloths)
                    {
                        dependencies.cloth->step(dependencies.cloth->self, cloth, fixedDelta);
                    }
                }
                demo::physics_sandbox::stepExtendedAfterRigid(dependencies, scene, simulatedSeconds, fixedDelta);
                // Drain the trigger queue: transitions accumulate per step.
                if (zoneTrigger != physics_rigid::invalidTrigger)
                {
                    std::uint32_t pendingEvents = 0;
                    bool          overflowed    = false;
                    if (dependencies.rigid->triggerEventCount(
                            dependencies.rigid->self, scene.world, &pendingEvents, &overflowed) && pendingEvents > 0)
                    {
                        std::vector<physics_rigid::TriggerEvent> events(pendingEvents);
                        std::uint32_t copied = 0;
                        if (dependencies.rigid->copyTriggerEvents(
                                dependencies.rigid->self, scene.world, 0, events.data(), pendingEvents, &copied))
                        {
                            for (const physics_rigid::TriggerEvent &event : events)
                            {
                                if (event.trigger != zoneTrigger)
                                {
                                    continue;
                                }
                                if (event.kind == physics_rigid::TriggerEventKind::Enter)
                                {
                                    ++triggerEnters;
                                }
                                else
                                {
                                    ++triggerExits;
                                }
                            }
                        }
                    }
                    dependencies.rigid->clearTriggerEvents(dependencies.rigid->self, scene.world);
                }
                sandboxRenderer.recordFixedStep(dependencies, scene, dependencies.cloth);
                sandboxRenderer.recordExtendedFixedStep(dependencies, scene, fixedDelta);
                simulatedSeconds += fixedDelta;
                ++stepsExecuted;
            }
        }

        frameTiming.markUpdateDone();

        // ---- camera and draw -------------------------------------------------------
        if (options.cameraMode == "orbit" && runUntilClosed == false && options.frames != 0 && !windowedBackend)
        {
            // Deterministic headless orbit: a constant pixel delta per frame.
            camera::CameraInputState orbitInput;
            orbitInput.mouseDeltaX = 2.4f;
            cameraCapability.update(cameraCapability.self, cameraHandle, &orbitInput, 1.0 / 30.0);
        }
        else
        {
            const camera::CameraInputState cameraSample = sandboxInput.sampleCamera();
            cameraCapability.update(
                cameraCapability.self, cameraHandle, &cameraSample, std::min(timing.deltaSeconds, 0.25));
        }

        camera::ViewState viewState{};
        cameraCapability.viewState(
            cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);
        renderer::CameraMatrices cameraMatrices{};
        std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
        std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
        std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
        renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

        const float               clearColor[4] = {0.10f, 0.12f, 0.16f, 1.0f};
        renderer::FrameParameters frameParameters{};
        std::memcpy(frameParameters.clearColor, clearColor, sizeof(frameParameters.clearColor));

        sandboxRenderer.buildDrawCommands(scene, static_cast<float>(timing.interpolationAlpha));
        sandboxRenderer.buildExtendedDrawCommands(dependencies, scene, static_cast<float>(timing.interpolationAlpha));
        frameTiming.markUploadDone();

        renderCapability.beginFrame(renderCapability.self, rendererHandle, &frameParameters);
        sandboxRenderer.submit(renderCapability, rendererHandle);
        sandboxRenderer.submitExtended(renderCapability, rendererHandle, dependencies, scene);
        renderCapability.endFrame(renderCapability.self, rendererHandle);
        frameTiming.markDrawDone();

        const std::uint64_t framesRendered = frameIndex + 1;
        if (options.statsEveryFrames != 0 && framesRendered % options.statsEveryFrames == 0)
        {
            physics_rigid::WorldStatistics statistics{};
            if (dependencies.rigid->worldStatistics(dependencies.rigid->self, scene.world, &statistics))
            {
                std::cout << "demo_physics_sandbox: frame " << frameIndex << " | bodies " << statistics.bodyCount
                          << " (awake " << statistics.awakeBodyCount << ", sleeping " << statistics.sleepingBodyCount
                          << ", lod-reduced " << statistics.lodReducedBodyCount << "), contacts "
                          << statistics.contactPairCount << " pairs / " << statistics.contactPointCount
                          << " points, joints " << statistics.jointCount << ", triggers +" << triggerEnters << "/-"
                          << triggerExits << ", step " << statistics.lastStepMilliseconds << " ms, steps this frame "
                          << stepsExecuted << "\n";
            }
            if (dependencies.cloth != nullptr && !scene.cloths.empty())
            {
                physics_cloth::ClothStatistics clothStatistics{};
                if (dependencies.cloth->clothStatistics(
                        dependencies.cloth->self, scene.cloths.front(), &clothStatistics))
                {
                    std::cout << "demo_physics_sandbox: cloth contacts " << clothStatistics.lastContactCount
                              << ", step " << clothStatistics.lastStepMilliseconds << " ms\n";
                }
            }
            if (options.dumpBodies)
            {
                for (const demo::physics_sandbox::SceneBody &sceneBody : scene.bodies)
                {
                    physics_rigid::BodyState state{};
                    if (dependencies.rigid->copyBodyState(
                            dependencies.rigid->self, scene.world, sceneBody.body, &state))
                    {
                        std::cout << "  body " << sceneBody.body << " at (" << state.transform.position.x << ", "
                                  << state.transform.position.y << ", " << state.transform.position.z << ")\n";
                    }
                }
                if (dependencies.articulation != nullptr)
                {
                    for (std::size_t figureIndex = 0; figureIndex < scene.figures.size(); ++figureIndex)
                    {
                        for (std::uint32_t linkIndex = 1; linkIndex < 4; ++linkIndex)
                        {
                            physics_articulated::FigureJointState jointState{};
                            if (dependencies.articulation->copyJointState(dependencies.articulation->self,
                                    scene.figures[figureIndex],
                                    linkIndex,
                                    &jointState))
                            {
                                std::cout << "  figure " << figureIndex << " joint " << linkIndex << " angle "
                                          << jointState.angleRadians << " swing " << jointState.swingRadians << "\n";
                            }
                        }
                    }
                }
            }
        }

        if (!screenshotTaken && !options.screenshotPath.empty() && framesRendered == options.screenshotFrame + 1)
        {
            screenshotTaken = demo::harness::exportScreenshotPpm(
                context, windowCapability, windowHandle, renderCapability, rendererHandle, options.screenshotPath);
            if (screenshotTaken)
            {
                std::cout << "demo_physics_sandbox: screenshot written to " << options.screenshotPath << "\n";
            }
            else
            {
                std::cerr << "demo_physics_sandbox: screenshot export failed for backend '" << windowBackendName
                          << "'\n";
            }
        }

        (void)framePacer.pace(&host.kernel().profiler());
    }

    // ---- teardown ---------------------------------------------------------------------
    std::cout << frameTiming.summary();
    if (!options.frameTimesPath.empty())
    {
        if (frameTiming.writeCsv(options.frameTimesPath))
        {
            std::cout << "demo_physics_sandbox: frame timings written to " << options.frameTimesPath << "\n";
        }
        else
        {
            std::cerr << "demo_physics_sandbox: could not write frame timings to " << options.frameTimesPath << "\n";
        }
    }
    sandboxRenderer.destroyMeshes(renderCapability, rendererHandle);
    demo::physics_sandbox::destroyScene(dependencies, scene);
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    host.stop();
    std::cout << "demo_physics_sandbox: stopped cleanly\n";
    return 0;
}
