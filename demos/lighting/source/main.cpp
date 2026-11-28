// demo_lighting: the engine's lighting pipeline in one scene - the pbr sun
// with its shadow maps, the direct_lighting registry's point lights, the
// radiance_cache module's probe grid and screen-space gather, and the
// reflection_field module's traced specular - over a deterministic scatter
// of thousands of static and animated instances. The per-frame lighting
// statistics the demo prints are the point: every term updates under a
// per-frame budget, so the frame cost stays flat no matter how large the
// registered scene is.
//
// The demo takes no command-line arguments: interactive runs use the keys
// (mouse look, WASD + Space/Ctrl movement, Shift sprint, 1..3 camera
// framings, F12 screenshot, Escape quits), and the headless/capture hooks
// ride DEMO_* environment variables (DEMO_VIEW selects the framing,
// DEMO_FRAMES/DEMO_WARMUP bound a run, DEMO_SCREENSHOT/DEMO_FRAME_TIMES
// export artifacts, DEMO_ORBIT drives the automated camera path,
// DEMO_SELF_TEST runs the registration audit; DEMO_OBJECTS/DEMO_DYNAMIC/
// DEMO_LIGHTS/DEMO_SEED/DEMO_FIELD/DEMO_ORIGIN shape the scene and
// DEMO_SHOWCASE toggles the reflection/refraction cluster). The window and
// renderer backends follow the hardware (GLFW + Vulkan when present, the
// CPU backends otherwise).

#include "lighting_demo_options.hpp"
#include "lighting_scene.hpp"

#include "demo_environment.hpp"

#include "demo_camera_input.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "demo_harness.hpp"

#include <engine/application/application_host.hpp>
#include <engine/application/command_line.hpp>
#include <chrono>
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/reflection_field/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

int main()
{
    using namespace demo;
    using namespace demo::lighting_demo;

    // The demo takes no command-line arguments (see the header comment): the
    // headless and capture hooks ride DEMO_* environment variables, and
    // interactive runs configure everything through the keys.
    LightingDemoOptions options;
    std::string         optionsError;
    if (!lightingDemoOptionsFromEnvironment(options, optionsError))
    {
        std::cerr << "demo_lighting: " << optionsError << "\n";
        return 2;
    }
    std::vector<std::string> profilePathList;
    std::vector<std::string> stagedOverrides;
    if (!demo::environment::readProfilePaths(profilePathList, optionsError)
        || !demo::environment::readConfigurationOverrides(stagedOverrides, optionsError))
    {
        std::cerr << "demo_lighting: " << optionsError << "\n";
        return 2;
    }

    // The demo's requirements: the window/renderer/camera trio every demo
    // needs, plus the whole lighting stack. Optional capabilities (scripting,
    // audio) stay out of the demand set.
    engine::ApplicationHostOptions hostOptions;
    hostOptions.requiredCapabilities
        = {"window", "renderer", "camera", "pbr", "direct_lighting", "radiance_cache", "reflection_field"};
    for (const std::string &profile : profilePathList)
    {
        hostOptions.configurationProfileFiles.push_back(profile);
    }
    // The scene registers one GI and one reflection surface per instance plus
    // the ground quadrants and the showcase cluster, so the registries must
    // hold the full scatter: the demo stages the same floors its own
    // configuration carries, and a launch that lands on a configuration
    // without the capacity keys (a foreign or hand-rolled engine.toml) keeps
    // working instead of failing the first registration.
    hostOptions.configurationOverrides.push_back("radiance_cache.scene.max_surfaces=8192");
    hostOptions.configurationOverrides.push_back("reflection_field.scene.max_surfaces=8192");
    hostOptions.configurationOverrides.insert(hostOptions.configurationOverrides.end(), stagedOverrides.begin(),
        stagedOverrides.end());

    // The scene anchor: the whole renderable world composes relative to this
    // ground-plane offset, so the identical scene can be placed anywhere on
    // a planetary surface (the anchor-relative frame below keeps every
    // float32 the lighting modules see small no matter how far out the
    // anchor sits).
    const glm::dvec2 sceneAnchor(options.originMeters[0], options.originMeters[1]);

    engine::ApplicationHost host;
    std::string             startupReport;
    if (!host.start(hostOptions, startupReport))
    {
        std::cerr << startupReport << "demo_lighting: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }
    {
        namespace window          = engine::window;
        namespace renderer        = engine::renderer;
        namespace camera          = engine::camera;
        namespace pbr             = engine::pbr;
        namespace radiance_cache  = engine::radiance_cache;
        namespace reflection_field = engine::reflection_field;
        namespace direct_lighting = engine::direct_lighting;

        auto                        &context = host.context();
        window::WindowCapability    &windowCapability = context.acquireCapability<window::WindowCapability>();
        renderer::RenderCapability  &renderCapability = context.acquireCapability<renderer::RenderCapability>();
        camera::CameraCapability    &cameraCapability = context.acquireCapability<camera::CameraCapability>();
        pbr::PbrCapability          &pbrCapability    = context.acquireCapability<pbr::PbrCapability>();
        radiance_cache::RadianceCacheCapability &giCapability
            = context.acquireCapability<radiance_cache::RadianceCacheCapability>();
        reflection_field::ReflectionFieldCapability &reflectionCapability
            = context.acquireCapability<reflection_field::ReflectionFieldCapability>();
        direct_lighting::DirectLightingCapability &directLightingCapability
            = context.acquireCapability<direct_lighting::DirectLightingCapability>();
        if (windowCapability.abi != window::windowCapabilityAbi
            || renderCapability.abi != renderer::renderCapabilityAbi
            || cameraCapability.abi != camera::cameraCapabilityAbi
            || pbrCapability.abi != pbr::pbrCapabilityAbi
            || giCapability.abi != radiance_cache::radianceCacheCapabilityAbi
            || reflectionCapability.abi != reflection_field::reflectionFieldCapabilityAbi
            || directLightingCapability.abi != direct_lighting::directLightingCapabilityAbi)
        {
            std::cerr << "demo_lighting: capability ABI mismatch\n";
            host.stop();
            return 1;
        }

        // ---- window + renderer + camera ---------------------------------
        window::WindowDescription windowDescription;
        windowDescription.title  = "engine lighting demo";
        windowDescription.width  = 1280;
        windowDescription.height = 720;
        window::WindowError windowError = window::WindowError::None;
        const window::WindowHandle windowHandle
            = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
        if (windowHandle == window::invalidWindow)
        {
            std::cerr << "demo_lighting: window creation failed\n";
            host.stop();
            return 1;
        }
        windowCapability.pollEvents(windowCapability.self);

        renderer::RenderError renderError = renderer::RenderError::None;
        const renderer::RendererHandle rendererHandle
            = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
        if (rendererHandle == renderer::invalidRenderer)
        {
            std::cerr << "demo_lighting: renderer creation failed\n";
            windowCapability.destroyWindow(windowCapability.self, windowHandle);
            host.stop();
            return 1;
        }
        const std::string backendName = renderCapability.activeBackendName(renderCapability.self, rendererHandle);

        camera::CameraParameters cameraParameters;
        camera::CameraError      cameraError = camera::CameraError::None;
        const camera::CameraHandle cameraHandle
            = cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
        // The camera works in the ANCHOR-RELATIVE frame: the anchor itself
        // never enters its numbers, so the pose stays float-exact at any
        // scene placement.
        cameraCapability.setProjection(cameraCapability.self, cameraHandle, camera::ProjectionType::Perspective,
            58.0f, 0.1f, 900.0f);
        cameraCapability.setLookAt(cameraCapability.self, cameraHandle,
            glm::value_ptr(glm::vec3(34.0f, 16.0f, 34.0f)), glm::value_ptr(glm::vec3(0.0f, 2.0f, 0.0f)),
            glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));

        // ---- the scene ---------------------------------------------------
        LightingSceneParameters sceneParameters;
        sceneParameters.seed        = options.seed;
        sceneParameters.objectCount = options.objectCount;
        sceneParameters.dynamicShare = options.dynamicShare;
        sceneParameters.lightCount  = options.lightCount;
        sceneParameters.fieldMeters = options.fieldMeters;
        sceneParameters.showcaseEnabled = options.showcaseEnabled;
        // The scene holds renderer meshes and module registrations; its
        // lifetime must end before the renderer, the window and the host
        // below tear those owners down (a unique_ptr pins that order in
        // both the self-test and the frame-loop exits).
        std::unique_ptr<LightingScene> scene
            = std::make_unique<LightingScene>(renderCapability, rendererHandle, sceneParameters);
        scene->bindLightingModules(&giCapability, &reflectionCapability, &directLightingCapability);
        if (!scene->initialize(engine::executableDirectory() / "assets" / "textures"))
        {
            std::cerr << "demo_lighting: scene initialization failed\n";
            cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
            renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
            windowCapability.destroyWindow(windowCapability.self, windowHandle);
            host.stop();
            return 1;
        }

        // The sky backdrop bakes from the live pbr environment, so the dome
        // and the shading share one sun and one hemisphere sky.
        if (const pbr::PbrEnvironment *environment = pbrCapability.environment(pbrCapability.self))
        {
            scene->buildSkyBackdrop(*environment);
        }

        const LightingSceneCounts counts = scene->counts();
        std::cout << "demo_lighting: backend '" << backendName << "', scene anchor ("
                  << sceneAnchor.x << ", " << sceneAnchor.y << ") m\n";
        std::cout << "demo_lighting: " << counts.objects << " objects (" << counts.dynamicObjects
                  << " dynamic), " << counts.groundTiles << " ground tiles, " << counts.lights
                  << " point lights, " << counts.triangles << " triangles, " << counts.giSurfaces
                  << " GI surfaces, " << counts.reflectionSurfaces << " reflection surfaces";
        if (counts.showcaseSurfaces != 0)
        {
            std::cout << ", showcase " << counts.showcaseSurfaces << " surfaces ("
                      << counts.transmissiveSurfaces << " transmissive: reflection, Snell refraction"
                      << " and the water/glass caustics)";
        }
        std::cout << "\n";

        // The resolved per-frame budgets the lighting systems run with: the
        // "low-cost" half of the demo's claim, printed once at startup.
        renderer::DeviceProfile deviceProfile{};
        if (renderCapability.deviceProfile(renderCapability.self, rendererHandle, &deviceProfile))
        {
            radiance_cache::GiProfile giProfile;
            if (giCapability.resolveProfile(giCapability.self, &deviceProfile, &giProfile))
            {
                std::cout << "demo_lighting: GI budget - " << giProfile.captureTexelsPerFrame
                          << " capture texels, " << giProfile.directTexelsPerFrame << " direct texels, "
                          << giProfile.indirectTexelsPerFrame << " indirect texels, "
                          << giProfile.probeUpdatesPerFrame << " probe updates per frame\n";
            }
            reflection_field::ReflectionProfile reflectionProfile;
            if (reflectionCapability.resolveProfile(reflectionCapability.self, &deviceProfile, &reflectionProfile))
            {
                std::cout << "demo_lighting: reflection budget - 1/" << reflectionProfile.screenScale
                          << " field scale, " << reflectionProfile.raysPerPixel << " rays per pixel\n";
            }
            direct_lighting::DirectLightingProfile lightProfile;
            if (directLightingCapability.resolveProfile(directLightingCapability.self, &deviceProfile, &lightProfile))
            {
                std::cout << "demo_lighting: direct light budget - " << lightProfile.maxLights
                          << " lights per draw\n";
            }
        }

        if (options.selfTest)
        {
            const std::uint32_t failures = scene->selfTest();
            scene.reset();
            cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
            renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
            windowCapability.destroyWindow(windowCapability.self, windowHandle);
            host.stop();
            return failures == 0 ? 0 : 1;
        }

        // ---- the frame loop ----------------------------------------------
        // Headless runs pace a fixed frame delta (deterministic captures)
        // and never wall-clock throttle; windowed runs use the wall clock
        // (clamped like the planet demo) and pace to the configured frame
        // limit (engine.max_fps, 60 by default) whenever the presentation
        // path itself does not already pace the loop.
        const double frameDelta = 1.0 / 30.0;
        auto          lastWallClock = std::chrono::steady_clock::now();
        const std::int64_t maxFps
            = context.configurationIntegerInSection("engine", "max_fps").value_or(60);
        const bool headless = backendName.find("headless") != std::string::npos;
        const bool presentedPacing = renderCapability.waitsForPresentation != nullptr
            && renderCapability.waitsForPresentation(renderCapability.self, rendererHandle);
        engine::FramePacer framePacer;
        harness::FrameTimingRecorder frameTiming;
        frameTiming.setAutoFlushPath(options.frameTimesPath);
        std::vector<window::WindowEvent> eventBuffer(64);

        // The frame limiter resolves after the window exists (the display
        // query needs it). engine.max_fps > 0 caps the loop at that rate and
        // 0 caps it at the display's refresh rate; headless capture runs
        // stay unthrottled (see applyFramePaceTarget).
        double monitorRefreshHz = 0.0;
        if (windowCapability.getWindowAttributes != nullptr)
        {
            window::WindowAttributes windowAttributes;
            if (windowCapability.getWindowAttributes(windowCapability.self, windowHandle, &windowAttributes))
            {
                monitorRefreshHz = windowAttributes.monitorRefreshRateHz;
            }
        }
        // The resolved pace target (engine.max_fps contract); the pacer is a
        // no-op while the target is 0 (headless capture runs, or vsync
        // already pacing the loop at the display rate).
        const double effectiveTargetHz = harness::applyFramePaceTarget(
            framePacer, !headless, std::max<std::int64_t>(maxFps, 0), presentedPacing, monitorRefreshHz,
            "demo_lighting");
        (void)effectiveTargetHz;

        std::uint32_t framesRendered = 0;
        const std::uint32_t totalFrames = headless ? options.frames : 0u;
        bool          screenshotTaken = options.screenshotPath.empty();
        const std::uint32_t screenshotFrame
            = options.screenshotFrame == 0xffffffffu ? totalFrames - 1u : options.screenshotFrame;
        double        simSeconds   = 0.0;
        const bool    windowedRun  = !headless;
        std::uint64_t frameIndex   = 0;
        // The active camera framing: the startup DEMO_VIEW choice, then
        // whatever the 1..3 keys switch to (the framings share the scene;
        // only the camera path changes).
        std::string activeView = options.view;
        bool        screenshotQueued = false;
        std::uint64_t screenshotIndex = 0;
        if (windowedRun)
        {
            std::cout << "demo_lighting: controls - mouse looks (LeftAlt releases the cursor), WASD walks,\n"
                         "      Space rises, Ctrl descends, Shift sprints, 1..3 switch framings (field/showcase/pool),\n"
                         "      F12 saves a screenshot, Escape quits\n";
        }

        // The shared interactive-camera input (cursor capture + raw mouse
        // deltas through the camera module's degrees-per-pixel look path).
        demo::harness::DemoCameraInput cameraInput(
            windowCapability, windowHandle, windowedRun, windowedRun);
        // The 1..3 keys re-frame the free camera: the next update applies the
        // view's pose once through setLookAt, then manual flight continues.
        bool applyViewPose = true;
        // Rolling wall-clock frame rate for the periodic stats line.
        auto        fpsWindowStart    = std::chrono::steady_clock::now();
        std::uint64_t fpsWindowFrames = 0;

        for (; windowedRun || framesRendered < totalFrames + options.warmupFrames; ++frameIndex)
        {
            if (windowCapability.shouldClose(windowCapability.self, windowHandle))
            {
                break;
            }
            frameTiming.beginFrame();
            windowCapability.pollEvents(windowCapability.self);
            harness::drainWindowEvents(windowCapability, windowHandle, eventBuffer,
                [&](const window::WindowEvent &event)
                {
                    // The in-app controls beyond camera movement: F12 saves
                    // a timestamped screenshot, 1..3 switch the camera
                    // framing, LeftAlt toggles the cursor capture.
                    cameraInput.handleWindowEvent(event);
                    if (event.kind == window::WindowEventKind::KeyPressed)
                    {
                        if (event.key == window::Key::F12)
                        {
                            screenshotQueued = true;
                        }
                        else if (event.key == window::Key::Num1)
                        {
                            activeView      = "field";
                            applyViewPose   = true;
                        }
                        else if (event.key == window::Key::Num2)
                        {
                            activeView      = "showcase";
                            applyViewPose   = true;
                        }
                        else if (event.key == window::Key::Num3)
                        {
                            activeView      = "pool";
                            applyViewPose   = true;
                        }
                    }
                });

            if (windowCapability.isKeyDown != nullptr
                && windowCapability.isKeyDown(windowCapability.self, windowHandle, window::Key::Escape))
            {
                std::cout << "demo_lighting: escape pressed\n";
                break;
            }

            // F12 exports the frame this update renders (a headless window
            // exports its surface directly; a Vulkan renderer defers the
            // readback to that frame's retirement) as a timestamped PNG in
            // the working directory.
            if (screenshotQueued)
            {
                screenshotQueued = false;
                std::string writtenPath;
                const std::string request
                    = demo::environment::timestampedScreenshotPath("screenshot_lighting", screenshotIndex++);
                if (harness::exportScreenshot(context, windowCapability, windowHandle, renderCapability,
                        rendererHandle, request, writtenPath))
                {
                    std::cout << "demo_lighting: F12 screenshot written to " << writtenPath << "\n";
                }
                else
                {
                    std::cerr << "demo_lighting: F12 screenshot export failed for backend '" << backendName
                              << "'\n";
                }
            }

            const auto   now             = std::chrono::steady_clock::now();
            const double wallDeltaSeconds = std::min(0.25, std::chrono::duration<double>(now - lastWallClock).count());
            lastWallClock                = now;
            const double deltaSeconds    = headless ? frameDelta : wallDeltaSeconds;

            // The camera path per view: 'field' sweeps the whole plaza on a
            // slow circle (the capture path); 'showcase' and 'pool' frame
            // the reflecting pool - a fixed three-quarter pose and a low
            // across-the-water pose whose gaze runs toward the sun, so
            // the glints, the refraction and the caustics all read in one
            // frame. DEMO_ORBIT (or any headless run) animates whichever
            // framing is active; windowed runs keep the free camera.
            if (options.orbit || headless)
            {
                const glm::dvec3 focus = scene->showcaseFocus();
                glm::vec3       eye;
                glm::vec3       target;
                double          sweepRadians = 0.0;
                double          sweepRadius  = 0.0;
                if (activeView == "showcase")
                {
                    eye   = glm::vec3(static_cast<float>(focus.x) + 11.0f, 4.6f, static_cast<float>(focus.z) + 15.0f);
                    target = glm::vec3(static_cast<float>(focus.x), 0.8f, static_cast<float>(focus.z));
                    sweepRadians = 0.16;
                    sweepRadius  = 3.5;
                }
                else if (activeView == "pool")
                {
                    // The classic pool photograph: a LOW three-quarter pose
                    // along the pool's long axis, on the sun side. The
                    // Fresnel curve does the composition - the near water
                    // sits at a steep enough incidence to transmit (the pale
                    // floor reads through), the far water approaches grazing
                    // and turns to a dusk-sky mirror carrying the far coping's
                    // dark rim and the glowing shapes' reflections, and the
                    // transition rides the length of the pool instead of a
                    // flat sheet (a fully raised pose flattens both terms:
                    // the transmission reads as a uniform tint and the
                    // reflections fall below the sheet's own brightness).
                    eye   = glm::vec3(static_cast<float>(focus.x) + 0.8f, 2.2f, static_cast<float>(focus.z) + 9.6f);
                    target = glm::vec3(static_cast<float>(focus.x) - 1.2f, 0.3f, static_cast<float>(focus.z) - 3.5f);
                    sweepRadians = 0.04;
                    sweepRadius  = 0.8;
                }
                else
                {
                    const double angle = 0.55 + simSeconds * 0.05;
                    const double radius = 46.0;
                    eye = glm::vec3(static_cast<float>(std::cos(angle) * radius), 15.0f,
                    static_cast<float>(std::sin(angle) * radius));
                    target = glm::vec3(0.0f, 2.5f, 0.0f);
                }
                if (sweepRadius > 0.0)
                {
                    // A gentle arc around the view's own eye position keeps
                    // the framing while the light orbits move the caustics.
                    const double sweep = 0.5 + simSeconds * 0.03;
                    eye.x += static_cast<float>(std::cos(sweep) * sweepRadians * sweepRadius);
                    eye.z += static_cast<float>(std::sin(sweep) * sweepRadians * sweepRadius);
                }
                cameraCapability.setLookAt(cameraCapability.self, cameraHandle, glm::value_ptr(eye),
                glm::value_ptr(target), glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));
            }
            else
            {
                // Full manual flight: the shared input sample carries the
                // WASD/Space/Ctrl axes plus the raw cursor deltas (the camera
                // module's degrees-per-pixel look path), Shift sprints. A
                // 1..3 key press re-frames the camera first through the
                // view's own pose, then manual flight continues from there.
                if (applyViewPose)
                {
                    applyViewPose = false;
                    const glm::dvec3 focus = scene->showcaseFocus();
                    glm::vec3       eye;
                    glm::vec3       target;
                    if (activeView == "showcase")
                    {
                        eye    = glm::vec3(static_cast<float>(focus.x) + 11.0f, 4.6f, static_cast<float>(focus.z) + 15.0f);
                        target = glm::vec3(static_cast<float>(focus.x), 0.8f, static_cast<float>(focus.z));
                    }
                    else if (activeView == "pool")
                    {
                        eye    = glm::vec3(static_cast<float>(focus.x) + 0.8f, 2.2f, static_cast<float>(focus.z) + 9.6f);
                        target = glm::vec3(
                            static_cast<float>(focus.x) - 1.2f, 0.3f, static_cast<float>(focus.z) - 3.5f);
                    }
                    else
                    {
                        // The field framing's own orbit pose at time zero
                        // (angle 0.55, radius 46 - the automated path's
                        // opening frame).
                        eye    = glm::vec3(39.2f, 15.0f, 24.0f);
                        target = glm::vec3(0.0f, 2.5f, 0.0f);
                    }
                    cameraCapability.setLookAt(cameraCapability.self, cameraHandle, glm::value_ptr(eye),
                        glm::value_ptr(target), glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));
                }
                const camera::CameraInputState input = cameraInput.sample();
                cameraCapability.update(cameraCapability.self, cameraHandle, &input, deltaSeconds);
            }

            simSeconds += deltaSeconds;

            // The lighting frame: both modules evaluate under their own
            // per-frame budgets (the camera and view are anchor-relative,
            // the same frame every renderable lives in).
            camera::ViewState viewState;
            {
                std::uint32_t windowWidth  = 0;
                std::uint32_t windowHeight = 0;
                windowCapability.getSize(windowCapability.self, windowHandle, &windowWidth, &windowHeight);
                if (windowWidth == 0 || windowHeight == 0)
                {
                    windowWidth  = 1280;
                    windowHeight = 720;
                }
                cameraCapability.viewState(cameraCapability.self, cameraHandle, windowWidth, windowHeight, &viewState);
                renderCapability.setViewport(renderCapability.self, rendererHandle, windowWidth, windowHeight);
                // The renderer shades from the same matrices the lighting
                // modules evaluate with (one camera for every consumer).
                renderer::CameraMatrices cameraMatrices{};
                std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
                std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
                std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
                renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);
                giCapability.setView(giCapability.self, viewState.viewProjection, viewState.position);
                reflectionCapability.setView(reflectionCapability.self, viewState.viewProjection, viewState.position,
                    windowWidth, windowHeight);
            }
            // The scene poses AFTER the camera state is read (the sky dome
            // re-centers on this frame's eye) and BEFORE the modules
            // evaluate (the dynamic transforms it pushes land in the same
            // evaluate).
            scene->update(simSeconds, glm::dvec3(viewState.position[0], viewState.position[1], viewState.position[2]));
            radiance_cache::GiError giError = radiance_cache::GiError::None;
            giCapability.evaluate(giCapability.self, deltaSeconds, &giError);
            reflection_field::ReflectionError reflectionError = reflection_field::ReflectionError::None;
            reflectionCapability.evaluate(reflectionCapability.self, deltaSeconds, &reflectionError);
            frameTiming.markUpdateDone();

            const bool warmupFrame = headless && framesRendered < options.warmupFrames;
            if (warmupFrame)
            {
                // Converge-only frame: the lighting budgets fill their caches
                // (the evaluate calls above already ran); skip the draw.
                ++framesRendered;
                (void)framePacer.pace(&host.kernel().profiler());
                continue;
            }

            renderer::FrameParameters frameParameters{};
            frameParameters.clearColor[0] = 0.012f;
            frameParameters.clearColor[1] = 0.014f;
            frameParameters.clearColor[2] = 0.028f;
            frameParameters.clearColor[3] = 1.0f;
            const auto &commands = scene->drawCommands();
            frameTiming.markUploadDone();
            renderCapability.beginFrame(renderCapability.self, rendererHandle, &frameParameters);
            renderCapability.drawMeshes(renderCapability.self, rendererHandle, commands.data(),
                static_cast<std::uint32_t>(commands.size()));
            renderer::FrameStatistics frameStats{};
            renderCapability.frameStatistics(renderCapability.self, rendererHandle, &frameStats);
            if (frameStats.gpuTimingAvailable)
            {
                frameTiming.recordGpuMilliseconds(frameStats.gpuFrameMilliseconds);
            }
            renderCapability.endFrame(renderCapability.self, rendererHandle);
            frameTiming.markDrawDone();

            // The rendered frame's 0-based index is framesRendered -
            // warmupFrames here (the counter increments at the loop tail).
            if (!screenshotTaken && static_cast<std::uint32_t>(framesRendered - options.warmupFrames)
                    == screenshotFrame)
            {
                std::string writtenPath;
                screenshotTaken = harness::exportScreenshot(context, windowCapability, windowHandle, renderCapability,
                    rendererHandle, options.screenshotPath, writtenPath);
                if (screenshotTaken)
                {
                    std::cout << "demo_lighting: screenshot written to " << writtenPath << "\n";
                }
                else
                {
                    std::cerr << "demo_lighting: screenshot export failed for backend '" << backendName << "'\n";
                }
            }

            const std::uint32_t renderedFrame = static_cast<std::uint32_t>(framesRendered - options.warmupFrames);
            ++fpsWindowFrames;
            if (options.statsEveryFrames != 0 && renderedFrame > 0 && renderedFrame % options.statsEveryFrames == 0)
            {
                radiance_cache::GiStatistics giStatistics;
                reflection_field::ReflectionStatistics reflectionStatistics;
                const bool giValid         = giCapability.statistics(giCapability.self, &giStatistics);
                const bool reflectionValid = reflectionCapability.statistics(reflectionCapability.self,
                    &reflectionStatistics);
                // The wall-clock frame rate over the stats window: with the
                // frame limiter active this reads the configured limit
                // (engine.max_fps, 60 by default) whenever the frame cost
                // stays under one frame interval.
                const auto   now        = std::chrono::steady_clock::now();
                const double windowSeconds = std::chrono::duration<double>(now - fpsWindowStart).count();
                const double framesPerSecond
                    = windowSeconds > 0.0 ? static_cast<double>(fpsWindowFrames) / windowSeconds : 0.0;
                fpsWindowStart  = now;
                fpsWindowFrames = 0;
                char fpsText[16];
                std::snprintf(fpsText, sizeof(fpsText), "%.1f", framesPerSecond);
                std::cout << "demo_lighting: frame " << renderedFrame << " | " << fpsText << " fps";
                if (giValid)
                {
                    std::cout << " | GI " << giStatistics.lastEvaluateMilliseconds << " ms (surfaces "
                              << giStatistics.surfaces << ", cards " << giStatistics.cards << ", skipped "
                              << giStatistics.framesSkippedStatic << ", texels cap/dir/ind "
                              << giStatistics.captureTexelsThisFrame << "/" << giStatistics.directTexelsThisFrame
                              << "/" << giStatistics.indirectTexelsThisFrame << ", probe rays "
                              << giStatistics.probeTracesThisFrame << ", gather rays "
                              << giStatistics.gatherRaysThisFrame
                              << ")";
                }
                if (reflectionValid)
                {
                    std::cout << " | reflections " << reflectionStatistics.lastEvaluateMilliseconds << " ms (rays "
                              << reflectionStatistics.raysTracedThisFrame << ", reused "
                              << reflectionStatistics.raysReusedCacheThisFrame << ", skipped "
                              << reflectionStatistics.framesSkippedStatic << ")";
                }
                std::cout << "\n";
            }
            ++framesRendered;
            (void)framePacer.pace(&host.kernel().profiler());
        }

        std::cout << frameTiming.summary();
        if (!options.frameTimesPath.empty())
        {
            if (frameTiming.writeCsv(options.frameTimesPath))
            {
                std::cout << "demo_lighting: frame timings written to " << options.frameTimesPath << "\n";
            }
        }

        scene.reset();
        cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
    }
    host.stop();
    return 0;
}
