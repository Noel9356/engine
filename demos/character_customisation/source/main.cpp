// main.cpp - the engine demo application.
//
// Starts the engine with the modules the demo needs and acquires their
// capabilities. --view character (the default) shows the character
// customisation screen built on the character module; --view benchmark|sun
// assembles the benchmark 3D scene (textured room, sky dome, sun disc,
// static draw batches, moving instances, dragon models). The frame loop runs
// camera input for the active view and drives the main_menu Lua plugin through
// the C++ bridge in demo/menu_bridge on the benchmark views.

#include <engine/application/application.hpp>
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/character/capabilities.hpp>
#include <engine/file_asset_store/capabilities.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/models/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/draw_sort.hpp>
#include <engine/script_runtime/capabilities.hpp>
#include <engine/ui/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include "demo/benchmark_scene.hpp"
#include "demo/camera_bindings.hpp"
#include "demo/character_screen.hpp"
#include "demo/diagnostics.hpp"
#include "demo/frame_stats.hpp"
#include "demo/menu_bridge.hpp"
#include "demo/options.hpp"
#include "engine/application/application_host.hpp"
#include "engine/application/command_line.hpp"
#include "engine/application/frame_clock.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"
#include "engine/kernel/profiling.hpp"
#include "engine/renderer/device_profile.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"

#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using demo::DemoCameraKeyBindings;
using demo::keyMaskBit;
using demo::loadCameraKeyBindings;
using demo::windowKeyName;
using demo::cameraErrorName;
using demo::capabilityAbiMatches;
using demo::deviceClassName;
using demo::DynamicObject;
using demo::StaticBatch;
using demo::renderErrorName;
using demo::renderGraphErrorName;
using demo::windowErrorName;

namespace window       = engine::window;
namespace render_graph = engine::render_graph;

namespace pbr    = engine::pbr;
namespace camera = engine::camera;

// --screenshot writes its PPM export after the scene and menu have warmed up
// but before the loop can quit early; --screenshot-frame moves the export to
// another frame (the Options default in options.hpp mirrors this choice).

} // namespace

int main(int argc, char **argv)
{
    // Flush stdout after every insertion: startup progress and stall notices
    // must be visible the moment they print, even when stdout is a pipe or a
    // redirected file (a hung run otherwise shows a stale tail and the last
    // visible line can be minutes behind where the process actually is).
    // Startup output volume is tiny, so the extra flushes cost nothing.
    std::cout << std::unitbuf;

    // Engine-owned flags are parsed first; the rest belong to the demo (see
    // demo::parseOptions) or trigger a validate-only dry run of the host.
    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo: " << commandLineError << '\n' << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << demo::optionsUsage();
        return 0;
    }

    demo::Options demoOptions;
    if (!demo::parseOptions(commandLine.applicationArguments, demoOptions, commandLineError))
    {
        std::cerr << "demo: " << commandLineError << "\n";
        return 2;
    }

    // Modules loaded by the kernel for this demo; pbr, character and
    // render_graph are probed optionally below.
    engine::ApplicationHostOptions hostOptions = engine::makeApplicationHostOptions(
        commandLine, {"window", "renderer", "render_graph", "camera", "models", "ui", "character", "script_runtime"});
    // The character customisation sample includes a small script plugin that
    // exercises the public character Lua bridge. Keep it in this application's
    // default set, while preserving explicit --active-plugin/--no-plugins
    // command-line choices from the caller.
    if (!hostOptions.activePlugins.has_value())
    {
        hostOptions.activePlugins = std::vector<std::string>{"main_menu", "character_lab"};
    }
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
        std::cerr << startupReport << "demo: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::EngineKernel &kernel     = host.kernel();
    auto                 *appContext = &host.context();

    const std::filesystem::path executableDirectory = engine::executableDirectory();

    // Assets live next to the executable in installed builds; fall back to the
    // working directory for in-tree runs.
    std::filesystem::path assetRoot = executableDirectory / "assets";
    if (!std::filesystem::exists(assetRoot))
    {
        const std::filesystem::path workingDirectoryAssets("assets");
        if (std::filesystem::exists(workingDirectoryAssets))
        {
            assetRoot = workingDirectoryAssets;
        }
    }
    window::WindowCapability         &windowCapability = appContext->acquireCapability<window::WindowCapability>();
    renderer::RenderCapability       &renderCapability = appContext->acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability         &cameraCapability = appContext->acquireCapability<camera::CameraCapability>();
    engine::models::ModelsCapability &modelsCapability
        = appContext->acquireCapability<engine::models::ModelsCapability>();
    ui::UiCapability &uiCapability = appContext->acquireCapability<ui::UiCapability>();
    // The character module hosts the character customisation screen; it is
    // optional at the engine level (a build without it falls back to the
    // benchmark views) but required by --view character.
    auto *characterCapability = appContext->tryAcquireCapability<engine::character::CharacterCapability>();
    if (characterCapability != nullptr
        && !capabilityAbiMatches("character", characterCapability->abi, engine::character::characterCapabilityAbi))
    {
        characterCapability = nullptr;
    }
    const bool characterView = demoOptions.viewName == "character";
    if (characterView && characterCapability == nullptr)
    {
        std::cerr << "demo: --view character needs the character module, which is not loaded\n";
        host.stop();
        return 1;
    }
    // pbr is optional: without it the demo falls back to its built-in sun and
    // skips the environment block below.
    const pbr::PbrCapability *pbrCapability = appContext->tryAcquireCapability<pbr::PbrCapability>();
    // render_graph is optional too: the demo schedules its per-frame CPU work
    // through it when present and falls back to inline updates otherwise.
    auto *frameGraphCapability = appContext->tryAcquireCapability<render_graph::RenderGraphCapability>();
    if (frameGraphCapability != nullptr
        && !capabilityAbiMatches("render_graph", frameGraphCapability->abi, render_graph::renderGraphCapabilityAbi))
    {
        frameGraphCapability = nullptr;
    }

    // Bail out (releasing the host) if any acquired capability's ABI differs
    // from the one the demo compiled against; extended backend capabilities are
    // checked only when present.
    if (!capabilityAbiMatches("window", windowCapability.abi, window::windowCapabilityAbi)
        || !capabilityAbiMatches("renderer", renderCapability.abi, renderer::renderCapabilityAbi)
        || !capabilityAbiMatches("camera", cameraCapability.abi, camera::cameraCapabilityAbi)
        || !capabilityAbiMatches("models", modelsCapability.abi, engine::models::modelsCapabilityAbi)
        || !capabilityAbiMatches("ui", uiCapability.abi, ui::uiCapabilityAbi)
        || (pbrCapability != nullptr && !capabilityAbiMatches("pbr", pbrCapability->abi, pbr::pbrCapabilityAbi)))
    {
        host.stop();
        return 1;
    }

    // Optional backend extensions: GLFW native handles, the headless surface,
    // and Vulkan-specific frame export/info queries.
    auto *glfwCapability           = appContext->tryAcquireCapability<window::GlfwWindowCapability>();
    auto *headlessWindowCapability = appContext->tryAcquireCapability<window::HeadlessWindowCapability>();
    auto *vulkanCapability         = appContext->tryAcquireCapability<renderer::VulkanRenderCapability>();
    // Optional CPU-side eviction lever for the GPU memory-budget feedback
    // below; absent when the file_asset_store module is not loaded.
    auto *fileAssetStoreCapability
        = appContext->tryAcquireCapability<engine::file_asset_store::FileAssetStoreCapability>();
    if ((glfwCapability != nullptr
            && !capabilityAbiMatches("window.glfw", glfwCapability->abi, window::windowCapabilityAbi))
        || (headlessWindowCapability != nullptr
            && !capabilityAbiMatches("window.headless", headlessWindowCapability->abi, window::windowCapabilityAbi))
        || (vulkanCapability != nullptr
            && !capabilityAbiMatches("renderer.vulkan", vulkanCapability->abi, renderer::renderCapabilityAbi))
        || (fileAssetStoreCapability != nullptr
            && !capabilityAbiMatches("file_asset_store",
                fileAssetStoreCapability->abi,
                engine::file_asset_store::fileAssetStoreCapabilityAbi)))
    {
        host.stop();
        return 1;
    }

    // pbr supplies the lighting environment when available; its sun replaces
    // the demo defaults below.
    pbr::PbrEnvironment pbrEnvironment{};
    const bool          hasPbrEnvironment
        = pbrCapability != nullptr && pbrCapability->copyEnvironment(pbrCapability->self, &pbrEnvironment, nullptr);

    const window::WindowDescription windowDescription{};

    // The renderer's auto policy probes the host GPU inside createRenderer;
    // on a machine with a broken or slow Vulkan ICD that probe can take a
    // while, so say where startup is before the window and renderer come up.
    std::cout << "demo: creating window and renderer (probing the GPU once)...\n" << std::flush;

    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo: the window module could not create a window (" << windowErrorName(windowError) << ")\n";
        host.stop();
        return 1;
    }

    renderer::RenderError renderError = renderer::RenderError::None;
    // Renderer creation is off-thread and the window keeps being pumped while
    // it runs. Rationale: VulkanBackend::create compiles the graphics
    // pipelines inside one uninterruptible driver call, which on slower GPUs
    // and VM graphics stacks takes tens of seconds to minutes. With the
    // previous single-threaded startup the window existed but its event loop
    // was never reached during that time, so the OS flagged it "not
    // responding" and users closed the demo mid-compilation. GLFW requires
    // event polling on the main thread, so instead the heavy call moved off
    // it: this thread only waits and pumps; the render thread does the rest.
    // (Vulkan has no main-thread restriction, and no GLFW call happens inside
    // renderer creation - the surface was already created above.)
    std::atomic<bool>        rendererCreationDone{false};
    bool                     rendererCreationThrew = false;
    renderer::RendererHandle createdRenderer       = renderer::invalidRenderer;
    {
        std::thread creator(
            [&]()
            {
                try
                {
                    createdRenderer
                        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
                }
                catch (...)
                {
                    rendererCreationThrew = true;
                }
                rendererCreationDone.store(true, std::memory_order_release);
            });
        if (windowCapability.activeBackendName(windowCapability.self) != std::string("headless"))
        {
            std::cout << "demo: initializing the renderer (first launch compiles shaders; this can take a while on\n"
                         "      slower GPUs and inside VMs - the window stays responsive while it works)...\n"
                      << std::flush;
        }
        while (!rendererCreationDone.load(std::memory_order_acquire))
        {
            windowCapability.pollEvents(windowCapability.self);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        creator.join();
    }
    const renderer::RendererHandle rendererHandle = createdRenderer;
    if (rendererCreationThrew)
    {
        std::cerr << "demo: the renderer module crashed during creation\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo: the renderer module could not create a renderer (" << renderErrorName(renderError) << ")\n";
        if (renderError == renderer::RenderError::BackendUnavailable
            && windowCapability.activeBackendName(windowCapability.self) == std::string("glfw"))
        {
            // An on-screen window can only be presented to by the Vulkan
            // backend, so this failure with a GLFW window means no usable
            // Vulkan device/driver was found. Say what to do next instead of
            // leaving a bare error code.
            std::cerr << "demo: on-screen rendering needs a Vulkan-capable GPU and driver, and none was\n"
                         "      available. Install or enable a Vulkan driver (for example mesa-vulkan-drivers\n"
                         "      or your laptop vendor's driver), or run offscreen by setting both\n"
                         "      [window] backend = \"headless\" and [renderer] backend = \"headless\" in the profile\n"
                         "      (headless demo runs stop themselves after 10s unless --quit-after is given).\n";
        }
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }

    std::cout << "demo: window backend '" << windowCapability.activeBackendName(windowCapability.self)
              << "', renderer backend '" << renderCapability.activeBackendName(renderCapability.self, rendererHandle)
              << "'\n";
    std::cout << "demo: renderer device "
              << deviceClassName(renderCapability.deviceClass(renderCapability.self, rendererHandle));
    if (renderCapability.deviceName(renderCapability.self, rendererHandle) != nullptr)
    {
        std::cout << " '" << renderCapability.deviceName(renderCapability.self, rendererHandle) << "'";
    }
    std::cout << "\n";
    if (hasPbrEnvironment)
    {
        std::cout << "demo: pbr sun (" << pbrEnvironment.sun.direction[0] << ", " << pbrEnvironment.sun.direction[1]
                  << ", " << pbrEnvironment.sun.direction[2] << ") intensity " << pbrEnvironment.sun.intensity << "\n";
    }
    if (glfwCapability != nullptr)
    {
        const char                 *glfwVersion = glfwCapability->glfwVersion(glfwCapability->self);
        window::NativeWindowHandles nativeHandles;
        if (glfwVersion != nullptr && glfwCapability->nativeHandles(glfwCapability->self, windowHandle, &nativeHandles))
        {
            std::cout << "demo: window.glfw " << glfwVersion << ", native window "
                      << (nativeHandles.win32Hwnd != nullptr ? "win32" : "x11") << "\n";
        }
    }
    if (vulkanCapability != nullptr)
    {
        renderer::RendererInfoStrings info;
        if (vulkanCapability->infoStrings(vulkanCapability->self, rendererHandle, &info))
        {
            std::cout << "demo: renderer.vulkan " << info.version << " (" << info.renderer << ", " << info.vendor
                      << ")\n";
        }
    }

    if (characterView)
    {
        std::cout << "demo: controls - left-drag orbits the character, Space/Ctrl zoom, "
                     "arrow keys orbit\n"
                     "      the Character Studio panel edits parts, parameters, and recipes\n";
    }
    else
    {
        std::cout << "demo: controls - mouse looks, WASD moves, Space up, Ctrl down, "
                     "Shift sprints,\n"
                     "      Esc toggles menu, Left Alt toggles cursor\n";
    }

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);

    std::cout << "demo: assets at '" << assetRoot.string() << "'\n";
    demo::BenchmarkScene benchmarkScene;
    demo::createBenchmarkScene(benchmarkScene, renderCapability, rendererHandle, modelsCapability, windowCapability,
        assetRoot, characterView, hasPbrEnvironment ? &pbrEnvironment : nullptr);

    glm::vec3 sunDirectionForView(0.45f, 0.62f, 0.35f);
    if (hasPbrEnvironment)
    {
        sunDirectionForView = glm::normalize(glm::make_vec3(pbrEnvironment.sun.direction));
    }

    // --view selects the starting viewpoint: default overview of the grid or a
    // close-up angled toward the sun disc. The character screen re-aims the
    // camera itself (orbit around the assembled character).
    glm::vec3 viewEye(0.0f, 13.0f, 39.0f);
    glm::vec3 viewTarget(0.0f, 0.0f, 0.0f);
    if (characterView)
    {
        viewEye    = glm::vec3(1.15f, 1.65f, 2.6f);
        viewTarget = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    else if (demoOptions.viewName == "sun")
    {
        viewEye    = glm::vec3(7.5f, 8.0f, 10.5f);
        viewTarget = sunDirectionForView * 55.0f;
    }

    camera::CameraParameters cameraParameters{
        .position    = {viewEye.x, viewEye.y, viewEye.z},
        .target      = {viewTarget.x, viewTarget.y, viewTarget.z},
        .up          = {0.0f, 1.0f, 0.0f},
        .fovYDegrees = 0.0f,
        .nearZ       = 0.25f,
        .farZ        = 2000.0f,
    };
    camera::CameraError        cameraError = camera::CameraError::None;
    const camera::CameraHandle cameraHandle
        = cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo: the camera module could not create a camera (" << cameraErrorName(cameraError) << ")\n";
        host.stop();
        return 1;
    }
    if (!demoOptions.cameraMode.empty())
    {
        const camera::CameraControlMode mode = demoOptions.cameraMode == "fly"   ? camera::CameraControlMode::Fly
                                             : demoOptions.cameraMode == "orbit" ? camera::CameraControlMode::Orbit
                                                                                 : camera::CameraControlMode::Fixed;
        cameraCapability.setControlMode(cameraCapability.self, cameraHandle, mode);
    }

    // Capture the cursor (fly mode) unless orbit/fixed was requested; the
    // window config is consulted when the command line left it empty. The
    // character screen keeps the cursor visible for its UI panel.
    const auto configuredCameraMode = appContext->configurationStringInSection("camera", "mode");
    bool       cursorCaptured       = characterView ? false
                                    : demoOptions.cameraMode.empty()
                                        ? (!configuredCameraMode.has_value()
                                              || (*configuredCameraMode != "orbit" && *configuredCameraMode != "fixed"))
                                        : demoOptions.cameraMode == "fly";
    windowCapability.setCursorMode(windowCapability.self,
        windowHandle,
        cursorCaptured ? window::CursorMode::Captured : window::CursorMode::Normal);
    const DemoCameraKeyBindings cameraKeys = loadCameraKeyBindings(*appContext);
    // Mask of every key the camera controls use, queried in one keyStateMask
    // call per frame instead of thirteen.
    const std::uint64_t cameraKeyMask
        = keyMaskBit(cameraKeys.moveForward) | keyMaskBit(cameraKeys.moveBackward) | keyMaskBit(cameraKeys.moveLeft)
        | keyMaskBit(cameraKeys.moveRight) | keyMaskBit(cameraKeys.moveUp) | keyMaskBit(cameraKeys.moveDown)
        | keyMaskBit(cameraKeys.orbitZoomIn) | keyMaskBit(cameraKeys.orbitZoomOut) | keyMaskBit(cameraKeys.sprint)
        | keyMaskBit(cameraKeys.lookLeft) | keyMaskBit(cameraKeys.lookRight) | keyMaskBit(cameraKeys.lookUp)
        | keyMaskBit(cameraKeys.lookDown);
    bool   haveLastCameraCursor = false;
    double lastCameraCursorX    = 0.0;
    double lastCameraCursorY    = 0.0;

    // quit_after: the command line (--quit-after) wins over the
    // [engine] quit_after_seconds configuration; <= 0 disables the timeout.
    constexpr double            headlessDefaultQuitAfterSeconds = 10.0;
    const std::int64_t          maxFps = appContext->configurationIntegerInSection("engine", "max_fps").value_or(0);
    const std::optional<double> configuredQuitAfterSeconds
        = appContext->configurationFloatingInSection("engine", "quit_after_seconds");
    const bool quitAfterExplicit = demoOptions.quitAfterSeconds >= 0.0 || configuredQuitAfterSeconds.has_value();
    double     effectiveQuitAfterSeconds
        = demoOptions.quitAfterSeconds >= 0.0 ? demoOptions.quitAfterSeconds : configuredQuitAfterSeconds.value_or(0.0);
    // A headless window has no close button and never reports a close request,
    // so an unbounded run would spin silently forever (the terminal looks
    // frozen). When neither the command line nor the configuration set a quit
    // condition, fall back to a bounded headless run and say so.
    if (!quitAfterExplicit && effectiveQuitAfterSeconds <= 0.0
        && windowCapability.activeBackendName(windowCapability.self) == std::string("headless"))
    {
        effectiveQuitAfterSeconds = headlessDefaultQuitAfterSeconds;
        std::cout << "demo: headless window backend has no close button; auto-quit after "
                  << headlessDefaultQuitAfterSeconds
                  << "s (override with --quit-after or [engine] quit_after_seconds)\n";
    }

    const renderer::MeshMaterial floorMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = benchmarkScene.floorAlbedo,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = benchmarkScene.floorMetalRough,
        .normalTexture            = benchmarkScene.floorNormal,
        .normalScale              = 0.9f,
        .occlusionTexture         = benchmarkScene.floorOcclusion,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = false,
    };
    const renderer::MeshMaterial rockMaterial{
        .baseColor                = {0.78f, 0.84f, 0.88f},
        .baseColorTexture         = renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 0.82f,
        .metallicRoughnessTexture = renderer::invalidTexture,
        .normalTexture            = renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = false,
    };
    const renderer::MeshMaterial torusMaterial{
        .baseColor                = {0.95f, 0.70f, 0.42f},
        .baseColorTexture         = renderer::invalidTexture,
        .metallicFactor           = 0.82f,
        .roughnessFactor          = 0.24f,
        .metallicRoughnessTexture = renderer::invalidTexture,
        .normalTexture            = renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = false,
    };
    const renderer::MeshMaterial dragonMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 0.78f,
        .metallicRoughnessTexture = renderer::invalidTexture,
        .normalTexture            = renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = false,
    };
    const renderer::MeshMaterial skyMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = renderer::invalidTexture,
        .normalTexture            = renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = true,
    };
    const renderer::MeshMaterial sunMaterial{
        .baseColor                = {benchmarkScene.sunDiscRadiance.r, benchmarkScene.sunDiscRadiance.g, benchmarkScene.sunDiscRadiance.b},
        .baseColorTexture         = renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = renderer::invalidTexture,
        .normalTexture            = renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = renderer::invalidTexture,
        .unlit                    = true,
    };

    // Sky and sun are drawn first and unsorted; the floor/batch draws from
    // sortableStart on are sorted (default: material, then mesh) so the
    // renderer can batch runs of equal materials into instanced draws.
    // Benchmark-only: the character screen submits its own draw list.
    glm::mat4                                skyTransform(1.0f);
    glm::mat4                                sunTransform(1.0f);
    std::vector<renderer::RenderDrawCommand> drawCommands;
    if (!characterView)
    {
        const glm::mat4 identityTransform(1.0f);
        drawCommands.reserve(3 + benchmarkScene.staticBatches.size());
        const auto addDraw
            = [&drawCommands](
                  renderer::MeshHandle mesh, const glm::mat4 &transform, const renderer::MeshMaterial &material)
        {
            if (mesh != renderer::invalidMesh)
            {
                drawCommands.push_back({.mesh = mesh, .modelMatrix = glm::value_ptr(transform), .material = &material});
            }
        };
        addDraw(benchmarkScene.skyDomeMesh, skyTransform, skyMaterial);
        addDraw(benchmarkScene.sunDiscMesh, sunTransform, sunMaterial);
        const std::size_t sortableStart = drawCommands.size();
        addDraw(benchmarkScene.floorMesh, identityTransform, floorMaterial);
        for (const StaticBatch &batch : benchmarkScene.staticBatches)
        {
            addDraw(batch.mesh, identityTransform, batch.modelType == 0 ? rockMaterial : torusMaterial);
        }
        if (drawCommands.size() > sortableStart)
        {
            renderer::sortRenderDrawCommands(
                std::span<renderer::RenderDrawCommand>(drawCommands).subspan(sortableStart));
        }
    }

    // Fixed-update pacing comes from the [engine] section; the clock clamps
    // large deltas and caps catch-up steps per frame.
    const double configuredFixedUpdateHz
        = appContext->configurationFloatingInSection("engine", "fixed_update_hz").value_or(0.0);
    const double configuredMaxFrameDelta
        = appContext->configurationFloatingInSection("engine", "max_frame_delta_seconds").value_or(0.25);
    const std::int64_t configuredMaxFixedSteps
        = appContext->configurationIntegerInSection("engine", "max_fixed_update_steps").value_or(4);

    // The main_menu plugin is a Lua script hosted by the script_runtime module.
    // The bridge below gives the hosted runtime the drawing functions it needs;
    // everything else (visibility, statistics) lives in the Lua script.
    auto *scriptCapability = appContext->tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
    if (scriptCapability != nullptr
        && !demo::capabilityAbiMatches(
            "script_runtime", scriptCapability->abi, script_runtime::scriptRuntimeCapabilityAbi))
    {
        scriptCapability = nullptr;
    }

    demo::MenuBridge                    menuBridge;
    script_runtime::ScriptRuntimeHandle menuRuntime = script_runtime::invalidScriptRuntime;
    bool                                menuReady   = false;
    if (scriptCapability == nullptr)
    {
        std::cerr << "demo: script_runtime capability unavailable; the main_menu "
                     "plugin is disabled\n";
    }
    else
    {
        menuRuntime = scriptCapability->hostedRuntime(scriptCapability->self, "main_menu");
        if (menuRuntime == script_runtime::invalidScriptRuntime)
        {
            std::cerr << "demo: the main_menu script plugin is not hosted; the menu is "
                         "disabled\n";
        }
        else if (!menuBridge.create(uiCapability, renderCapability, rendererHandle))
        {
            std::cerr << "demo: main_menu bridge could not create its UI context or "
                         "font texture\n";
        }
        else if (!menuBridge.registerNatives(*scriptCapability, menuRuntime))
        {
            std::cerr << "demo: main_menu bridge could not register its drawing "
                         "functions\n";
            menuBridge.destroy();
        }
        else
        {
            menuReady = true;
        }
    }

    // The character customisation screen (its own UI context, content, and
    // assembly on top of the character module).
    demo::CharacterScreen characterScreen;
    bool                  characterScreenReady = false;
    bool                  characterTestPassed  = true;
    if (characterView)
    {
        characterScreenReady = characterScreen.create(windowCapability,
            windowHandle,
            renderCapability,
            rendererHandle,
            cameraCapability,
            cameraHandle,
            uiCapability,
            *characterCapability,
            assetRoot.string());
        if (!characterScreenReady)
        {
            std::cerr << "demo: the character screen failed to initialise\n";
            menuBridge.destroy();
            cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
            renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
            windowCapability.destroyWindow(windowCapability.self, windowHandle);
            host.stop();
            return 1;
        }
        if (demoOptions.characterTest)
        {
            characterTestPassed = characterScreen.runSelfTest();
            if (demoOptions.screenshotPath.empty())
            {
                // Pure test run: leave the loop almost immediately.
                effectiveQuitAfterSeconds
                    = effectiveQuitAfterSeconds > 0.0 ? std::min(effectiveQuitAfterSeconds, 0.25) : 0.25;
            }
        }
        if (!demoOptions.characterDrive.empty())
        {
            if (!characterScreen.applyDrivePreset(demoOptions.characterDrive))
            {
                std::cerr << "demo: character drive preset failed\n";
                menuBridge.destroy();
                cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
                renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
                windowCapability.destroyWindow(windowCapability.self, windowHandle);
                host.stop();
                return 1;
            }
        }
        if (demoOptions.cameraFramingSet)
        {
            characterScreen.configureCamera(demoOptions.cameraYawDegrees, demoOptions.cameraPitchDegrees,
                demoOptions.orbitDistance, demoOptions.cameraTargetY);
            characterScreen.setUserCameraFraming(true);
        }
        if (demoOptions.hidePanel)
        {
            characterScreen.setPanelVisible(false);
        }
        std::cout << "demo: character screen ready (drag to orbit, panel on the right)\n";
    }

    engine::FrameClock frameClock(engine::FrameClockOptions{
        .maxFrameDeltaSeconds = configuredMaxFrameDelta,
        .fixedUpdateHz        = configuredFixedUpdateHz,
        .maxFixedUpdatesPerFrame
        = static_cast<std::uint32_t>(std::clamp<std::int64_t>(configuredMaxFixedSteps, 1, 256))});

    std::uint64_t          frameIndex = 0;
    std::vector<glm::mat4> dynamicRockTransforms;
    std::vector<glm::mat4> dynamicTorusTransforms;
    dynamicRockTransforms.reserve(benchmarkScene.dynamicObjects.size());
    dynamicTorusTransforms.reserve(benchmarkScene.dynamicObjects.size());
    // Six dragons on display; the last two slowly wander along a Lissajous
    // path and turn, the rest stand still.
    std::vector<glm::mat4> dragonTransforms;
    dragonTransforms.reserve(6);
    const std::array<glm::vec3, 6> dragonPositions
        = {glm::vec3(-17.0f, 0.0f, -20.0f),
            glm::vec3(-6.0f, 0.0f, -20.0f),
            glm::vec3(6.0f, 0.0f, -20.0f),
            glm::vec3(17.0f, 0.0f, -20.0f),
            glm::vec3(-11.0f, 0.0f, 3.0f),
            glm::vec3(11.0f, 0.0f, 3.0f)};
    for (std::size_t index = 0; index < dragonPositions.size(); ++index)
    {
        const float yaw = index < 4 ? 0.0f : (index == 4 ? -0.22f : 0.22f);
        dragonTransforms.push_back(glm::translate(glm::mat4(1.0f), dragonPositions[index])
                                   * glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 1.0f, 0.0f))
                                   * glm::scale(glm::mat4(1.0f), glm::vec3(0.82f)));
    }

    // --- render_graph consumer ---
    // The demo's per-frame CPU work (moving instances, wandering dragons) is
    // scheduled through the render_graph capability: three independent passes
    // each write one transient scratch resource, compile() plans the graph
    // once, and every frame execute() runs the callbacks on the kernel task
    // scheduler. The three lambdas below are the single source of truth for
    // the update math; when the graph is unavailable (capability absent, ABI
    // mismatch, compile/execute failure) they are called inline instead.
    struct FrameGraphTasks
    {
        std::function<void()> updateRocks;
        std::function<void()> updateTori;
        std::function<void()> updateDragons;
    };

    float           animationTime = 0.0f; // refreshed every frame; the lambdas read it live
    FrameGraphTasks frameGraphTasks;
    frameGraphTasks.updateRocks = [&]()
    {
        dynamicRockTransforms.clear();
        for (DynamicObject &object : benchmarkScene.dynamicObjects)
        {
            if (object.modelType != 0)
            {
                continue;
            }
            const float phase    = animationTime * object.angularSpeed + object.phase;
            glm::vec3   position = object.basePosition;
            position.x += std::sin(phase * 0.55f) * 0.34f;
            position.y += std::sin(phase * 1.7f) * 0.18f;
            position.z += std::cos(phase * 0.47f) * 0.34f;
            object.transform = glm::translate(glm::mat4(1.0f), position)
                             * glm::rotate(glm::mat4(1.0f), phase, glm::vec3(0, 1, 0))
                             * glm::scale(glm::mat4(1.0f), glm::vec3(object.scale));
            dynamicRockTransforms.push_back(object.transform);
        }
    };
    frameGraphTasks.updateTori = [&]()
    {
        dynamicTorusTransforms.clear();
        for (DynamicObject &object : benchmarkScene.dynamicObjects)
        {
            if (object.modelType == 0)
            {
                continue;
            }
            const float phase    = animationTime * object.angularSpeed + object.phase;
            glm::vec3   position = object.basePosition;
            position.x += std::sin(phase * 0.55f) * 0.34f;
            position.y += std::sin(phase * 1.7f) * 0.18f;
            position.z += std::cos(phase * 0.47f) * 0.34f;
            object.transform = glm::translate(glm::mat4(1.0f), position)
                             * glm::rotate(glm::mat4(1.0f), phase, glm::vec3(0, 1, 0))
                             * glm::scale(glm::mat4(1.0f), glm::vec3(object.scale));
            dynamicTorusTransforms.push_back(object.transform);
        }
    };
    // Dragons 4 and 5 wander: position on a slow Lissajous-like path, yaw
    // drifts linearly with time.
    frameGraphTasks.updateDragons = [&]()
    {
        dragonTransforms[4]
            = glm::translate(glm::mat4(1.0f),
                  dragonPositions[4]
                      + glm::vec3(std::sin(animationTime * 0.37f) * 1.2f, 0.0f, std::cos(animationTime * 0.29f) * 0.9f))
            * glm::rotate(glm::mat4(1.0f), -0.22f + animationTime * 0.12f, glm::vec3(0, 1, 0))
            * glm::scale(glm::mat4(1.0f), glm::vec3(0.82f));
        dragonTransforms[5] = glm::translate(glm::mat4(1.0f),
                                  dragonPositions[5]
                                      + glm::vec3(std::sin(animationTime * 0.31f + 1.2f) * 1.2f,
                                          0.0f,
                                          std::cos(animationTime * 0.41f + 0.8f) * 0.9f))
                            * glm::rotate(glm::mat4(1.0f), 0.22f + animationTime * 0.1f, glm::vec3(0, 1, 0))
                            * glm::scale(glm::mat4(1.0f), glm::vec3(0.82f));
    };

    // Pass dispatch: the capability hands back the caller's original pass
    // index, which selects the matching update lambda.
    const render_graph::RenderGraphExecuteCallback frameGraphDispatch = [](void *userData, std::uint32_t passIndex)
    {
        auto *tasks = static_cast<FrameGraphTasks *>(userData);
        switch (passIndex)
        {
        case 0:
            tasks->updateRocks();
            break;
        case 1:
            tasks->updateTori();
            break;
        case 2:
            tasks->updateDragons();
            break;
        default:
            break;
        }
    };

    // Graph description, constant for the whole run: one pass per update
    // group writing its own scratch resource. Sizes represent the packed
    // instance matrices the passes produce; a minimum of one transform keeps
    // the graph well-formed even when a group is empty.
    std::size_t rockInstanceCount  = 0;
    std::size_t torusInstanceCount = 0;
    for (const DynamicObject &object : benchmarkScene.dynamicObjects)
    {
        ++(object.modelType == 0 ? rockInstanceCount : torusInstanceCount);
    }
    const std::uint64_t                        transformStride = sizeof(glm::mat4);
    const render_graph::RenderGraphResourceUse frameGraphUses[3]
        = {{.resourceIndex = 0, .access = render_graph::ResourceAccess::Write},
            {.resourceIndex = 1, .access = render_graph::ResourceAccess::Write},
            {.resourceIndex = 2, .access = render_graph::ResourceAccess::Write}};
    render_graph::RenderGraphPassDescription frameGraphPasses[3]{};
    for (std::uint32_t pass = 0; pass < 3; ++pass)
    {
        frameGraphPasses[pass].uses     = &frameGraphUses[pass];
        frameGraphPasses[pass].useCount = 1;
    }
    const render_graph::RenderGraphResourceDescription frameGraphResources[3]
        = {{.sizeBytes             = std::max<std::size_t>(rockInstanceCount, 1u) * transformStride,
               .alignmentBytes     = 256,
               .compatibilityClass = 0,
               .transient          = true},
            {.sizeBytes             = std::max<std::size_t>(torusInstanceCount, 1u) * transformStride,
                .alignmentBytes     = 256,
                .compatibilityClass = 0,
                .transient          = true},
            {.sizeBytes             = dragonTransforms.size() * transformStride,
                .alignmentBytes     = 256,
                .compatibilityClass = 0,
                .transient          = true}};

    bool          frameGraphReady = false;
    std::uint32_t frameGraphOrder[3]{};
    std::uint32_t frameGraphSlots[3]{};
    if (frameGraphCapability != nullptr && !characterView)
    {
        // Device-tuned limits: resolveProfile scales the configured settings
        // for the active renderer and setSettings applies them (clearing the
        // plan cache) before the one-time compile below.
        const render_graph::RenderGraphSettings *currentSettings
            = frameGraphCapability->settings(frameGraphCapability->self);
        renderer::DeviceProfile          deviceProfile{};
        render_graph::RenderGraphProfile scaledProfile{};
        if (currentSettings != nullptr
            && renderCapability.deviceProfile(renderCapability.self, rendererHandle, &deviceProfile)
            && frameGraphCapability->resolveProfile(frameGraphCapability->self, &deviceProfile, &scaledProfile))
        {
            render_graph::RenderGraphSettings applied = *currentSettings;
            applied.maxPasses                         = scaledProfile.maxPasses;
            applied.maxResources                      = scaledProfile.maxResources;
            applied.transientMemoryBudgetBytes        = scaledProfile.transientMemoryBudgetBytes;
            applied.transientAliasing                 = scaledProfile.transientAliasing;
            if (frameGraphCapability->setSettings(frameGraphCapability->self, &applied))
            {
                std::cout << "demo: render_graph profile applied: maxPasses " << applied.maxPasses << ", maxResources "
                          << applied.maxResources << ", transient budget "
                          << applied.transientMemoryBudgetBytes / (1024ull * 1024ull) << " MB\n";
            }
        }

        render_graph::RenderGraphCompileResult compileResult{};
        render_graph::RenderGraphError         graphError = render_graph::RenderGraphError::None;
        if (frameGraphCapability->compile(frameGraphCapability->self,
                frameGraphPasses,
                3,
                frameGraphResources,
                3,
                frameGraphOrder,
                3,
                frameGraphSlots,
                3,
                &compileResult,
                &graphError))
        {
            frameGraphReady = true;
            std::cout << "demo: render_graph frame updates: 3 passes, 3 transient "
                         "scratch resources -> "
                      << compileResult.transientSlotCount << " physical slot(s), " << compileResult.transientBytes
                      << " bytes, execution order " << frameGraphOrder[0] << " -> " << frameGraphOrder[1] << " -> "
                      << frameGraphOrder[2] << "\n";
        }
        else
        {
            std::cerr << "demo: render_graph compile failed (" << renderGraphErrorName(graphError)
                      << "); frame updates run inline\n";
        }
    }
    else if (!characterView)
    {
        std::cout << "demo: render_graph capability unavailable; frame updates run "
                     "inline\n";
    }

    bool                                screenshotTaken = false;
    std::array<window::WindowEvent, 64> eventBuffer{};
    // Double-throttle guard: a FIFO swapchain already blocks in present until
    // the next vblank, so the loop is paced to the display refresh by the
    // presentation engine. Running FramePacer's wall-clock deadline chain on
    // top of that throttles the same frame twice. The two clocks are never in
    // lock-step - the steady_clock deadline and the display's vblank are
    // independent, and the presentation engine only quantises to refresh
    // boundaries - so whenever the pacer's deadline lands a hair past the
    // vblank the frame is held a whole extra refresh period. That is the
    // recurring 2x-period spike at P99 (~31 ms against a 60 Hz display) when
    // using the full 1200us spin window. With a reduced spin window (200us)
    // and vsync disabled (recommended for lowest P99), the pacer operates
    // without double-throttle interference.
    //
    // When the target rate is at or above the display refresh the pacer has
    // nothing to add, so it is disabled and presentation does the pacing.
    // A target below the refresh still needs the pacer (it is throttling
    // harder than vsync would). Unknown/legacy modules return false here and
    // keep the previous behaviour.
    const bool presentedPacing
        = renderCapability.waitsForPresentation != nullptr
        && renderCapability.waitsForPresentation(renderCapability.self, rendererHandle);
    const bool pacerSuppressedByVsync = presentedPacing && maxFps > 0;
    const double effectiveTargetHz    = pacerSuppressedByVsync ? 0.0 : static_cast<double>(std::max<std::int64_t>(maxFps, 0));

    // Paced runs sleep to just before the deadline and spin the remainder:
    // OS sleep granularity (1-4 ms Windows, ~50 us timer slack on Linux)
    // otherwise lands directly in the frame-time P99. Uncapped runs skip
    // pacing entirely, so the spin window costs nothing there.
    // Reduced spin window minimizes CPU overhead while still protecting
    // against sleep granularity-induced P99 spikes.
    const std::uint32_t pacerSpinWindowMicroseconds = effectiveTargetHz > 0.0 ? 200u : 0u;
    engine::FramePacer  framePacer(
        {.targetFrameRateHz = effectiveTargetHz, .spinWindowMicroseconds = pacerSpinWindowMicroseconds});
    framePacer.reset();
    if (pacerSuppressedByVsync)
    {
        // Surface the decision: a run whose frame rate is paced by the
        // presentation engine should not look like a misconfigured max_fps.
        std::cout << "demo: max_fps " << maxFps
                  << " left to vsync - the swapchain already paces to the display refresh, so FramePacer is off "
                     "(raise the display rate or disable vsync to cap harder)\n";
    }
    demo::FrameTimeRecorder frameTimeRecorder;

    // A windowed frame loop is silent on the terminal; say how to exit.
    const bool headlessWindowBackend
        = windowCapability.activeBackendName(windowCapability.self) == std::string("headless");
    if (!headlessWindowBackend)
    {
        std::cout << "demo: running - close the window to quit\n" << std::flush;
    }

    // Edge state for the GPU memory-budget feedback (set below; the demo
    // reacts to pressure transitions, not to every frame's state).
    bool assetPressurePurgeActive = false;
    // Presentation-stall visibility: if the renderer stops completing frames
    // (minimized window, stalled presentation engine) the terminal would
    // otherwise sit silent while the process is alive and burning cycles.
    std::uint64_t presentedFrames        = 0;
    double        lastPresentedSeconds   = 0.0;
    double        lastStallNoticeSeconds = 0.0;

    // Main frame loop: pump events, advance the frame clock, update camera and
    // dynamic objects, draw, then handle screenshot/quit bookkeeping.
    while (true)
    {
        frameTimeRecorder.beginFrame();
        windowCapability.pollEvents(windowCapability.self);

        std::uint32_t eventCount = 0;
        // Drain the event queue in buffer-sized batches before the frame work.
        do
        {
            eventCount = windowCapability.popEvents(windowCapability.self,
                windowHandle,
                eventBuffer.data(),
                static_cast<std::uint32_t>(eventBuffer.size()));
            for (std::uint32_t eventIndex = 0; eventIndex < eventCount; ++eventIndex)
            {
                const window::WindowEvent &event = eventBuffer[eventIndex];
                if (characterScreenReady)
                {
                    characterScreen.handleWindowEvent(event);
                }
                if (event.kind == window::WindowEventKind::CloseRequested)
                {
                    windowCapability.requestClose(windowCapability.self, windowHandle);
                }
                else if (event.kind == window::WindowEventKind::FramebufferResized)
                {
                    renderCapability.setViewport(
                        renderCapability.self, rendererHandle, event.framebufferWidth, event.framebufferHeight);
                }
                else if (event.kind == window::WindowEventKind::KeyPressed)
                {
                    // Script plugins listen for key presses on the event bus; the
                    // demo itself only reacts to the cursor toggle.
                    kernel.eventBus().publish(engine::WindowKeyEvent{
                        .keyCode = static_cast<std::uint32_t>(event.key), .keyName = windowKeyName(event.key)});
                    if (event.key == window::Key::LeftAlt)
                    {
                        cursorCaptured = !cursorCaptured;
                        windowCapability.setCursorMode(windowCapability.self,
                            windowHandle,
                            cursorCaptured ? window::CursorMode::Captured : window::CursorMode::Normal);
                        haveLastCameraCursor = false;
                    }
                }
            }
        }
        while (eventCount == static_cast<std::uint32_t>(eventBuffer.size()));

        // Plugin callbacks run in these dispatches and may throw; a failing
        // handler must not take the demo down.
        try
        {
            kernel.eventBus().flushDeferredEvents();
        }
        catch (const std::exception &error)
        {
            std::cerr << "demo: deferred event handler failed: " << error.what() << "\n";
        }
        try
        {
            kernel.taskQueue().tick();
        }
        catch (const std::exception &error)
        {
            std::cerr << "demo: scheduled task failed: " << error.what() << "\n";
        }

        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            break;
        }

        // Advance the clock first so fixed updates are published before the
        // variable-step render work below.
        const auto now = std::chrono::steady_clock::now();

        const engine::FrameTiming frameTiming    = frameClock.advance(now);
        const double              deltaSeconds   = frameTiming.deltaSeconds;
        const double              elapsedSeconds = frameTiming.elapsedSeconds;
        frameIndex                               = frameTiming.frameIndex;
        frameClock.publishFixedUpdates(kernel, frameTiming);

        windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
        // Benchmark views assemble camera input from the configured keys
        // (signed -1/+1 axes) plus mouse deltas captured since the previous
        // frame. The character screen drives the camera itself.
        camera::CameraInputState cameraInput{};
        const std::uint64_t      cameraKeysDown
            = cameraKeyMask != 0 ? windowCapability.keyStateMask(windowCapability.self, windowHandle, cameraKeyMask)
                                 : 0;
        const auto keyDown      = [cameraKeysDown](window::Key key) { return (cameraKeysDown & keyMaskBit(key)) != 0; };
        cameraInput.moveForward = static_cast<float>(keyDown(cameraKeys.moveForward))
                                - static_cast<float>(keyDown(cameraKeys.moveBackward));
        cameraInput.moveRight
            = static_cast<float>(keyDown(cameraKeys.moveRight)) - static_cast<float>(keyDown(cameraKeys.moveLeft));
        cameraInput.moveUp
            = static_cast<float>(keyDown(cameraKeys.moveUp)) - static_cast<float>(keyDown(cameraKeys.moveDown));
        cameraInput.lookHorizontal
            = static_cast<float>(keyDown(cameraKeys.lookRight)) - static_cast<float>(keyDown(cameraKeys.lookLeft));
        cameraInput.lookVertical
            = static_cast<float>(keyDown(cameraKeys.lookUp)) - static_cast<float>(keyDown(cameraKeys.lookDown));
        cameraInput.zoom   = static_cast<float>(keyDown(cameraKeys.orbitZoomIn))
                           - static_cast<float>(keyDown(cameraKeys.orbitZoomOut));
        cameraInput.sprint = keyDown(cameraKeys.sprint);
        // Mouse look only while the cursor is captured; deltas are clamped to
        // guard against garbage cursor jumps from the platform.
        if (cursorCaptured)
        {
            double cursorX = 0.0;
            double cursorY = 0.0;
            if (windowCapability.cursorPosition(windowCapability.self, windowHandle, &cursorX, &cursorY)
                && std::isfinite(cursorX) && std::isfinite(cursorY))
            {
                if (haveLastCameraCursor)
                {
                    cameraInput.mouseDeltaX
                        = static_cast<float>(std::clamp(cursorX - lastCameraCursorX, -10000.0, 10000.0));
                    cameraInput.mouseDeltaY
                        = static_cast<float>(std::clamp(cursorY - lastCameraCursorY, -10000.0, 10000.0));
                }
                lastCameraCursorX    = cursorX;
                lastCameraCursorY    = cursorY;
                haveLastCameraCursor = true;
            }
            else
            {
                haveLastCameraCursor = false;
            }
        }
        else
        {
            haveLastCameraCursor = false;
        }
        if (!characterScreenReady)
        {
            cameraCapability.update(cameraCapability.self, cameraHandle, &cameraInput, deltaSeconds);
            camera::ViewState viewState;
            cameraCapability.viewState(
                cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);
            renderer::CameraMatrices cameraMatrices;
            std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
            std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
            std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
            renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

            // The sky dome and sun disc are infinitely far: re-center them on the
            // camera each frame so they can never be reached.
            const glm::vec3 cameraPosition = glm::make_vec3(viewState.position);
            skyTransform                   = glm::translate(glm::mat4(1.0f), cameraPosition);
            sunTransform                   = skyTransform;
        }

        // Per-frame CPU work: through the render_graph capability when it is
        // ready (the callbacks run on the task scheduler and block here), or
        // inline in dependency order otherwise. animationTime feeds both
        // paths; a failed execute falls back to a full inline pass, which is
        // safe because every update recomputes its outputs from scratch.
        animationTime = static_cast<float>(elapsedSeconds);
        if (characterScreenReady)
        {
            // (benchmark-only CPU passes skipped on the character screen)
        }
        else if (frameGraphReady)
        {
            render_graph::RenderGraphError graphError = render_graph::RenderGraphError::None;
            if (!frameGraphCapability->execute(frameGraphCapability->self,
                    frameGraphPasses,
                    3,
                    frameGraphResources,
                    3,
                    frameGraphDispatch,
                    &frameGraphTasks,
                    &graphError))
            {
                std::cerr << "demo: render_graph execute failed (" << renderGraphErrorName(graphError)
                          << "); frame updates fall back inline\n";
                frameGraphReady = false;
            }
        }
        if (!frameGraphReady && !characterScreenReady)
        {
            frameGraphTasks.updateRocks();
            frameGraphTasks.updateTori();
            frameGraphTasks.updateDragons();
        }

        renderCapability.beginFrame(renderCapability.self, rendererHandle, nullptr);
        if (characterScreenReady)
        {
            // The screen draws stage, character, and its UI panel, and
            // submits the overlay before endFrame.
            characterScreen.frame(deltaSeconds, elapsedSeconds);
        }
        else
        {
            // Static batches in one drawMeshes call; moving objects as instanced
            // draws (one per mesh, transform list packed contiguously).
            renderCapability.drawMeshes(renderCapability.self,
                rendererHandle,
                drawCommands.data(),
                static_cast<std::uint32_t>(drawCommands.size()));
            if (benchmarkScene.rockMesh != renderer::invalidMesh && !dynamicRockTransforms.empty())
            {
                renderCapability.drawMeshInstances(renderCapability.self,
                    rendererHandle,
                    benchmarkScene.rockMesh,
                    glm::value_ptr(dynamicRockTransforms.front()),
                    static_cast<std::uint32_t>(dynamicRockTransforms.size()),
                    &rockMaterial);
            }
            if (benchmarkScene.torusMesh != renderer::invalidMesh && !dynamicTorusTransforms.empty())
            {
                renderCapability.drawMeshInstances(renderCapability.self,
                    rendererHandle,
                    benchmarkScene.torusMesh,
                    glm::value_ptr(dynamicTorusTransforms.front()),
                    static_cast<std::uint32_t>(dynamicTorusTransforms.size()),
                    &torusMaterial);
            }
            if (!benchmarkScene.dragonMeshes.empty())
            {
                for (const renderer::MeshHandle dragonMesh : benchmarkScene.dragonMeshes)
                {
                    renderCapability.drawMeshInstances(renderCapability.self,
                        rendererHandle,
                        dragonMesh,
                        glm::value_ptr(dragonTransforms.front()),
                        static_cast<std::uint32_t>(dragonTransforms.size()),
                        &dragonMaterial);
                }
            }
        } // benchmark draw branch
        if (menuReady && !characterScreenReady)
        {
            std::uint32_t displayWidth  = 0;
            std::uint32_t displayHeight = 0;
            windowCapability.getSize(windowCapability.self, windowHandle, &displayWidth, &displayHeight);
            const float framebufferScaleX
                = displayWidth != 0 ? static_cast<float>(framebufferWidth) / static_cast<float>(displayWidth) : 1.0f;
            const float framebufferScaleY
                = displayHeight != 0 ? static_cast<float>(framebufferHeight) / static_cast<float>(displayHeight) : 1.0f;

            // The script reads the frame geometry from these globals, then draws
            // its window through the menu_* bridge functions.
            scriptCapability->setGlobalNumber(
                scriptCapability->self, menuRuntime, "menu_display_width", static_cast<double>(displayWidth));
            scriptCapability->setGlobalNumber(
                scriptCapability->self, menuRuntime, "menu_display_height", static_cast<double>(displayHeight));
            scriptCapability->setGlobalNumber(
                scriptCapability->self, menuRuntime, "menu_framebuffer_scale_x", framebufferScaleX);
            scriptCapability->setGlobalNumber(
                scriptCapability->self, menuRuntime, "menu_framebuffer_scale_y", framebufferScaleY);
            scriptCapability->setGlobalNumber(scriptCapability->self, menuRuntime, "menu_delta_seconds", deltaSeconds);

            std::array<char, 256>       scriptError{};
            script_runtime::ScriptError scriptError_ = script_runtime::ScriptError::None;
            // The script_runtime API reports runtime failures through the error
            // code and error text; the boolean return only excludes usage errors.
            const bool invoked = scriptCapability->callFunction(scriptCapability->self,
                menuRuntime,
                "on_draw",
                &scriptError_,
                scriptError.data(),
                static_cast<std::uint32_t>(scriptError.size()),
                nullptr);
            if (!invoked || scriptError_ != script_runtime::ScriptError::None)
            {
                // A failing draw must not repeat every frame; the menu stays
                // disabled and the error is reported once.
                std::cerr << "demo: main_menu on_draw failed (" << static_cast<std::uint32_t>(scriptError_)
                          << "): " << scriptError.data() << "\n";
                menuReady = false;
            }
        }
        renderCapability.endFrame(renderCapability.self, rendererHandle);

        // GPU memory-budget feedback into the CPU asset policy (P99 hybrid
        // round, H3): the renderer tracker enters its pressure band at 7/8 of
        // the device-local budget and leaves at 3/4 (hysteresis). On the
        // enter edge the demo purges unreferenced cached assets - the CPU
        // cache is the only unbounded pool a resident can shrink before the
        // driver starts paging - and both edges are logged once per episode.
        renderer::FrameStatistics frameStatistics;
        renderCapability.frameStatistics(renderCapability.self, rendererHandle, &frameStatistics);
        if (frameStatistics.framesRendered != presentedFrames)
        {
            presentedFrames      = frameStatistics.framesRendered;
            lastPresentedSeconds = elapsedSeconds;
        }
        else if (!headlessWindowBackend && elapsedSeconds - lastPresentedSeconds >= 2.0
                 && elapsedSeconds - lastStallNoticeSeconds >= 5.0)
        {
            std::cout << "demo: no frames presented for "
                      << static_cast<long long>(elapsedSeconds - lastPresentedSeconds)
                      << "s (window minimized or presentation stalled) - still running, close the window to quit\n"
                      << std::flush;
            lastStallNoticeSeconds = elapsedSeconds;
        }
        const bool deviceMemoryPressure = frameStatistics.deviceLocalMemoryPressure;
        if (deviceMemoryPressure != assetPressurePurgeActive)
        {
            assetPressurePurgeActive = deviceMemoryPressure;
            if (deviceMemoryPressure && fileAssetStoreCapability != nullptr)
            {
                const std::uint64_t freedBytes
                    = fileAssetStoreCapability->purgeUnusedAssets(fileAssetStoreCapability->self, 0);
                std::cout << "demo: device-local memory pressure (" << frameStatistics.deviceLocalUsageBytes << "/"
                          << frameStatistics.deviceLocalBudgetBytes << " bytes): purged " << freedBytes
                          << " evictable asset cache bytes\n";
            }
            else if (deviceMemoryPressure)
            {
                std::cout << "demo: device-local memory pressure (" << frameStatistics.deviceLocalUsageBytes << "/"
                          << frameStatistics.deviceLocalBudgetBytes << " bytes): no asset store loaded\n";
            }
            else
            {
                std::cout << "demo: device-local memory pressure left\n";
            }
        }

        // One-shot screenshot at the configured frame: the Vulkan backend can
        // export the frame directly; otherwise only the headless window backend
        // can export its surface. Other backends cannot export.
        if (!screenshotTaken && !demoOptions.screenshotPath.empty() && frameIndex == demoOptions.screenshotFrame)
        {
            if (vulkanCapability != nullptr
                && renderCapability.activeBackendName(renderCapability.self, rendererHandle) == std::string("vulkan"))
            {
                renderer::RenderError exportError = renderer::RenderError::None;

                screenshotTaken = vulkanCapability->exportFramePpm(
                    vulkanCapability->self, rendererHandle, demoOptions.screenshotPath.c_str(), &exportError);
                if (!screenshotTaken)
                {
                    std::cerr << "demo: screenshot export failed (" << renderErrorName(exportError) << ")\n";
                }
            }
            else if (headlessWindowCapability != nullptr
                     && windowCapability.activeBackendName(windowCapability.self) == std::string("headless"))
            {
                window::WindowError exportError = window::WindowError::None;
                screenshotTaken                 = headlessWindowCapability->exportSurfacePpm(
                    headlessWindowCapability->self, windowHandle, demoOptions.screenshotPath.c_str(), &exportError);
                if (!screenshotTaken)
                {
                    std::cerr << "demo: screenshot export failed (" << windowErrorName(exportError) << ")\n";
                }
            }
        }

        // Publish the frame event last so plugins observe the fully rendered state.
        frameClock.publishFrameEvent(kernel, frameTiming);
        ++frameIndex;

        if (effectiveQuitAfterSeconds > 0.0 && elapsedSeconds >= effectiveQuitAfterSeconds)
        {
            break;
        }
        // No-op when max_fps is 0 (uncapped).
        (void)framePacer.pace(&kernel.profiler());
    }

    // The --character-test run reports failure through the exit code.
    if (demoOptions.characterTest && characterView && !characterTestPassed)
    {
        std::cerr << "demo: character test FAILED\n";
    }

    // Headless runs that exited before the screenshot frame get one last
    // surface export so --quit-after can still produce an image.
    if (!screenshotTaken && !demoOptions.screenshotPath.empty() && headlessWindowCapability != nullptr
        && windowCapability.activeBackendName(windowCapability.self) == std::string("headless"))
    {
        window::WindowError exportError = window::WindowError::None;
        screenshotTaken                 = headlessWindowCapability->exportSurfacePpm(
            headlessWindowCapability->self, windowHandle, demoOptions.screenshotPath.c_str(), &exportError);
    }

    // Frame-time percentile summary (P50/P95/P99/max) for the session; this is
    // the stutter metric, printed before teardown so it covers every frame.
    frameTimeRecorder.printSummary();

    // Per-stage profile: attributes the frame-time tail to a specific wait
    // (slot retirement, swapchain acquire/present, queue submit, draw sort)
    // instead of leaving it to guesswork. Only printed when profiling is on.
    if (kernel.profiler().enabled())
    {
        std::fprintf(stderr, "demo: stage timings ms (count/p50/p99/max):\n");
        for (std::uint8_t metricIndex = 0; metricIndex < static_cast<std::uint8_t>(engine::PerformanceMetric::Count);
             ++metricIndex)
        {
            const auto metric   = static_cast<engine::PerformanceMetric>(metricIndex);
            const auto snapshot = kernel.profiler().snapshot(metric);
            if (snapshot.sampleCount == 0)
                continue;
            std::fprintf(stderr,
                "  %-20s n=%-8llu p50=%.3f p99=%.3f max=%.3f\n",
                engine::performanceMetricName(metric),
                static_cast<unsigned long long>(snapshot.sampleCount),
                static_cast<double>(snapshot.p50Nanoseconds) / 1.0e6,
                static_cast<double>(snapshot.p99Nanoseconds) / 1.0e6,
                static_cast<double>(snapshot.maximumNanoseconds) / 1.0e6);
        }
    }

    // Teardown in reverse creation order: meshes, textures, menu bridge,
    // camera, renderer, window, then the engine host itself.
    renderer::FrameStatistics statistics;
    renderCapability.frameStatistics(renderCapability.self, rendererHandle, &statistics);
    std::cout << "demo: renderer rendered " << statistics.framesRendered << " frames, "
              << statistics.trianglesRasterized << "/" << statistics.trianglesSubmitted << " triangles rasterized\n";

    demo::destroyBenchmarkScene(benchmarkScene, renderCapability, rendererHandle);
    characterScreen.destroy();
    menuBridge.destroy();
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);

    host.stop();
    std::cout << "demo: stopped cleanly after " << frameIndex << " frames\n";
    return (demoOptions.characterTest && characterView && !characterTestPassed) ? 1 : 0;
}
