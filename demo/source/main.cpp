// main.cpp - the engine demo application.
//
// Starts the engine with the modules the demo needs and acquires their
// capabilities. --view character (the default) shows the character
// customisation screen built on the character module; --view benchmark|sun
// assembles the benchmark 3D scene (textured room, sky dome, sun disc,
// static draw batches, moving instances, dragon models). The frame loop runs
// camera input for the active view and drives the main_menu Lua mod through
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

#include "demo/character_screen.hpp"
#include "demo/diagnostics.hpp"
#include "demo/frame_stats.hpp"
#include "demo/geometry.hpp"
#include "demo/menu_bridge.hpp"
#include "demo/options.hpp"
#include "engine/application/application_host.hpp"
#include "engine/application/command_line.hpp"
#include "engine/application/frame_clock.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"
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

using demo::cameraErrorName;
using demo::capabilityAbiMatches;
using demo::deviceClassName;
using demo::makeBoxMesh;
using demo::makeSphereMesh;
using demo::MeshData;
using demo::renderErrorName;
using demo::renderGraphErrorName;
using demo::windowErrorName;

namespace window       = engine::window;
namespace render_graph = engine::render_graph;

// Keyboard bindings for the camera controls. Defaults are WASD-style; every
// field can be overridden from a matching "*_key" entry in the [camera] config
// section (see loadCameraKeyBindings below).
struct DemoCameraKeyBindings
{
    window::Key moveForward  = window::Key::W;
    window::Key moveBackward = window::Key::S;
    window::Key moveLeft     = window::Key::A;
    window::Key moveRight    = window::Key::D;
    window::Key moveUp       = window::Key::Space;
    window::Key moveDown     = window::Key::LeftControl;
    window::Key orbitZoomIn  = window::Key::Space;
    window::Key orbitZoomOut = window::Key::LeftControl;
    window::Key sprint       = window::Key::LeftShift;
    window::Key lookLeft     = window::Key::Left;
    window::Key lookRight    = window::Key::Right;
    window::Key lookUp       = window::Key::Up;
    window::Key lookDown     = window::Key::Down;
};

// Bit position of a key in the 64-bit key-state mask returned by
// keyStateMask(); returns 0 for keys outside the representable range.
std::uint64_t keyMaskBit(window::Key key) noexcept
{
    const auto value = static_cast<std::uint32_t>(key);
    return value > 0 && value < 64 ? (std::uint64_t{1} << value) : 0;
}

using WindowKeyEntry = std::pair<const char *, window::Key>;

// Canonical lowercase names for window keys. The demo publishes these names in
// WindowKeyEvent, so script mods see the same spelling that configuration
// parsing accepts.
constexpr WindowKeyEntry windowKeyNames[]
    = {{"unknown", window::Key::Unknown},
        {"escape", window::Key::Escape},
        {"space", window::Key::Space},
        {"w", window::Key::W},
        {"a", window::Key::A},
        {"s", window::Key::S},
        {"d", window::Key::D},
        {"left", window::Key::Left},
        {"right", window::Key::Right},
        {"up", window::Key::Up},
        {"down", window::Key::Down},
        {"left_shift", window::Key::LeftShift},
        {"left_control", window::Key::LeftControl},
        {"q", window::Key::Q},
        {"e", window::Key::E},
        {"r", window::Key::R},
        {"f", window::Key::F},
        {"z", window::Key::Z},
        {"x", window::Key::X},
        {"c", window::Key::C},
        {"v", window::Key::V},
        {"b", window::Key::B},
        {"n", window::Key::N},
        {"m", window::Key::M},
        {"t", window::Key::T},
        {"g", window::Key::G},
        {"y", window::Key::Y},
        {"u", window::Key::U},
        {"i", window::Key::I},
        {"o", window::Key::O},
        {"p", window::Key::P},
        {"j", window::Key::J},
        {"k", window::Key::K},
        {"l", window::Key::L},
        {"h", window::Key::H},
        {"num0", window::Key::Num0},
        {"num1", window::Key::Num1},
        {"num2", window::Key::Num2},
        {"num3", window::Key::Num3},
        {"num4", window::Key::Num4},
        {"num5", window::Key::Num5},
        {"num6", window::Key::Num6},
        {"num7", window::Key::Num7},
        {"num8", window::Key::Num8},
        {"num9", window::Key::Num9},
        {"tab", window::Key::Tab},
        {"enter", window::Key::Enter},
        {"backspace", window::Key::Backspace},
        {"left_alt", window::Key::LeftAlt},
        {"right_shift", window::Key::RightShift},
        {"right_control", window::Key::RightControl},
        {"right_alt", window::Key::RightAlt}};

// Reverse lookup used when publishing WindowKeyEvent; the names match the
// spelling parseWindowKey accepts from configuration.
const char *windowKeyName(window::Key key) noexcept
{
    for (const WindowKeyEntry &entry : windowKeyNames)
        if (entry.second == key)
            return entry.first;
    return "unknown";
}

// Parses a lowercase key name from configuration; returns false if unknown.
bool parseWindowKey(const std::string &name, window::Key &out) noexcept
{
    for (const WindowKeyEntry &entry : windowKeyNames)
        if (name == entry.first)
        {
            out = entry.second;
            return true;
        }
    return false;
}

// Loads the [camera] "*_key" overrides for every binding. Unknown key names
// are reported on stderr and keep the WASD-style default.
DemoCameraKeyBindings loadCameraKeyBindings(engine::ParticipantContext &context)
{
    DemoCameraKeyBindings                                                               result{};
    const std::array<std::pair<const char *, window::Key DemoCameraKeyBindings::*>, 13> fields
        = {{{"move_forward_key", &DemoCameraKeyBindings::moveForward},
            {"move_backward_key", &DemoCameraKeyBindings::moveBackward},
            {"move_left_key", &DemoCameraKeyBindings::moveLeft},
            {"move_right_key", &DemoCameraKeyBindings::moveRight},
            {"move_up_key", &DemoCameraKeyBindings::moveUp},
            {"move_down_key", &DemoCameraKeyBindings::moveDown},
            {"orbit_zoom_in_key", &DemoCameraKeyBindings::orbitZoomIn},
            {"orbit_zoom_out_key", &DemoCameraKeyBindings::orbitZoomOut},
            {"sprint_key", &DemoCameraKeyBindings::sprint},
            {"look_left_key", &DemoCameraKeyBindings::lookLeft},
            {"look_right_key", &DemoCameraKeyBindings::lookRight},
            {"look_up_key", &DemoCameraKeyBindings::lookUp},
            {"look_down_key", &DemoCameraKeyBindings::lookDown}}};
    for (const auto &[name, member] : fields)
    {
        const auto configured = context.configurationStringInSection("camera", name);
        if (!configured)
            continue;
        window::Key parsed = window::Key::Unknown;
        if (parseWindowKey(*configured, parsed))
            result.*member = parsed;
        else
            std::fprintf(stderr, "demo: ignoring unknown [camera] %s '%s'\n", name, configured->c_str());
    }
    return result;
}
namespace pbr    = engine::pbr;
namespace camera = engine::camera;

// --screenshot writes its PPM export after the scene and menu have warmed up
// but before the loop can quit early; --screenshot-frame moves the export to
// another frame (the Options default in options.hpp mirrors this choice).

// Ground plane: a thin box extending roomHalfExtent on X/Z.
constexpr float roomHalfExtent = 36.0f;

// Sky dome and sun disc radii/positions (world units). Both sit inside the
// camera far plane (2000) and are re-centered on the camera every frame.
constexpr float skyDomeRadius = 1990.0f;
constexpr float sunDistance   = 1940.0f;
constexpr float sunRadius     = 50.0f;

// Appends a transformed copy of source into destination. Positions go through
// the full matrix, normals through its upper-left 3x3 (assuming rigid
// transforms), indices are rebased. Throws std::length_error if the combined
// mesh would no longer fit 32-bit indices.
void appendTransformedMesh(MeshData &destination, const MeshData &source, const glm::mat4 &transform)
{
    const std::size_t maxVertexCount = std::numeric_limits<std::uint32_t>::max();
    const std::size_t maxIndexCount  = std::numeric_limits<std::uint32_t>::max();
    if (source.vertices.size() > maxVertexCount || destination.vertices.size() > maxVertexCount - source.vertices.size()
        || source.indices.size() > maxIndexCount || destination.indices.size() > maxIndexCount - source.indices.size())
    {
        throw std::length_error("static benchmark mesh exceeds 32-bit mesh indexing");
    }

    const auto        baseVertex = static_cast<std::uint32_t>(destination.vertices.size());
    const glm::mat3   normalTransform(transform);
    const std::size_t requiredVertices = destination.vertices.size() + source.vertices.size();
    const std::size_t requiredIndices  = destination.indices.size() + source.indices.size();
    if (requiredVertices > destination.vertices.capacity())
    {
        destination.vertices.reserve(std::max(requiredVertices, destination.vertices.capacity() * 2));
    }
    if (requiredIndices > destination.indices.capacity())
    {
        destination.indices.reserve(std::max(requiredIndices, destination.indices.capacity() * 2));
    }
    for (const demo::Vertex &sourceVertex : source.vertices)
    {
        demo::Vertex vertex = sourceVertex;
        vertex.position     = glm::vec3(transform * glm::vec4(sourceVertex.position, 1.0f));
        vertex.normal       = glm::normalize(normalTransform * sourceVertex.normal);
        destination.vertices.push_back(vertex);
    }
    for (const std::uint32_t index : source.indices)
    {
        destination.indices.push_back(baseVertex + index);
    }
}

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
    const auto assetPath = [&assetRoot](const char *relativeName) { return (assetRoot / relativeName).string(); };

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
    // Scene setup below uploads hundreds of megabytes of mesh/texture data and
    // can take seconds on slow storage or drivers. Pumping the window in
    // between keeps the compositor fed so the OS never marks the window
    // unresponsive during startup; queued events are dispatched by the main
    // loop as usual.
    const auto pumpWindowEvents = [&]() { windowCapability.pollEvents(windowCapability.self); };
    // Load every floor texture up front; failures log and yield invalidTexture,
    // which the renderer treats as "no texture". The character screen builds
    // its own untextured stage, so this is benchmark-only.
    renderer::TextureHandle floorAlbedo     = renderer::invalidTexture;
    renderer::TextureHandle floorNormal     = renderer::invalidTexture;
    renderer::TextureHandle floorMetalRough = renderer::invalidTexture;
    renderer::TextureHandle floorOcclusion  = renderer::invalidTexture;
    if (!characterView)
    {
        std::cout << "demo: loading floor textures (albedo, normal, metal-roughness, occlusion)...\n";
        auto loadTexture = [&](const char *path)
        {
            const renderer::TextureHandle handle
                = renderCapability.createTextureFromFile(renderCapability.self, rendererHandle, path, &renderError);
            if (handle == renderer::invalidTexture)
            {
                std::cerr << "demo: texture '" << path << "' failed (" << renderErrorName(renderError) << ")\n";
            }
            return handle;
        };
        floorAlbedo = loadTexture(assetPath("textures/floor_tiles.png").c_str());
        pumpWindowEvents();
        floorNormal = loadTexture(assetPath("textures/floor_tiles_normal.png").c_str());
        pumpWindowEvents();
        floorMetalRough = loadTexture(assetPath("textures/floor_tiles_mr.png").c_str());
        pumpWindowEvents();
        floorOcclusion = loadTexture(assetPath("textures/floor_tiles_ao.png").c_str());
        pumpWindowEvents();
        std::cout << "demo: floor textures loaded\n";
    }

    glm::vec3 sunDiscColor(1.0f, 0.9f, 0.7f);
    glm::vec3 sunDirection(0.45f, 0.62f, 0.35f);
    float     sunIntensity = 3.0f;
    if (hasPbrEnvironment)
    {
        sunDirection = glm::make_vec3(pbrEnvironment.sun.direction);
        sunIntensity = pbrEnvironment.sun.intensity;
        sunDiscColor = glm::make_vec3(pbrEnvironment.sun.color);
    }
    sunDirection = glm::normalize(sunDirection);
    // The sun disc is unlit; boost its base color so it reads as emissive.
    const glm::vec3 sunDiscRadiance = sunDiscColor * (sunIntensity * 1.5f + 0.6f);

    // Uploads CPU mesh data via the renderer capability; failures log and leave
    // the outHandle invalid, which every draw path skips.
    auto uploadMesh = [&](const MeshData &mesh, renderer::MeshHandle &outHandle)
    {
        const renderer::MeshDescription description = mesh.description();
        outHandle = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &renderError);
        if (outHandle == renderer::invalidMesh)
        {
            std::cerr << "demo: renderer module rejected a scene mesh (" << renderErrorName(renderError) << ")\n";
        }
    };

    // Inigo model + benchmark environment: only the benchmark views load
    // them; the character screen builds its own stage and body parts.
    std::vector<renderer::MeshHandle> dragonMeshes;
    engine::models::ModelStatistics   dragonStatistics{};
    renderer::MeshHandle              floorMesh   = renderer::invalidMesh;
    renderer::MeshHandle              skyDomeMesh = renderer::invalidMesh;
    renderer::MeshHandle              sunDiscMesh = renderer::invalidMesh;
    if (!characterView)
    {
        engine::models::ModelLoadOptions dragonLoadOptions{
            .normalMode         = engine::models::ModelNormalMode::GenerateSmooth,
            .importVertexColors = true,
            .weldVertices       = true,
            .importMaterials    = false};
        engine::models::ModelLoadError dragonLoadError = engine::models::ModelLoadError::None;
        const std::string              dragonPath      = assetPath("models/inigo_the_dragon/dragon_meshed.obj");
        std::cout << "demo: importing the Inigo model (weld + smooth normals)...\n";
        const engine::models::ModelHandle dragonModel = modelsCapability.loadModelWithOptions(
            modelsCapability.self, dragonPath.c_str(), &dragonLoadOptions, &dragonLoadError);
        // One GPU mesh per imported part; parts that fail to upload are skipped.
        if (dragonModel == engine::models::invalidModel)
        {
            std::cerr << "demo: Inigo model import failed (" << static_cast<std::uint32_t>(dragonLoadError) << ")\n";
        }
        else
        {
            (void)modelsCapability.statistics(modelsCapability.self, dragonModel, &dragonStatistics);
            const std::uint32_t dragonPartCount = modelsCapability.partCount(modelsCapability.self, dragonModel);
            dragonMeshes.reserve(dragonPartCount);
            for (std::uint32_t partIndex = 0; partIndex < dragonPartCount; ++partIndex)
            {
                renderer::MeshDescription description;
                if (!modelsCapability.part(modelsCapability.self, dragonModel, partIndex, &description))
                {
                    continue;
                }
                const renderer::MeshHandle handle
                    = renderCapability.createMesh(renderCapability.self, rendererHandle, &description, &renderError);
                pumpWindowEvents();
                if (handle == renderer::invalidMesh)
                {
                    std::cerr << "demo: Inigo mesh upload failed (" << renderErrorName(renderError) << ")\n";
                    continue;
                }
                dragonMeshes.push_back(handle);
            }
            modelsCapability.destroyModel(modelsCapability.self, dragonModel);
            std::cout << "demo: loaded Inigo model " << dragonStatistics.vertexCount << " vertices, "
                      << dragonStatistics.indexCount / 3 << " triangles, " << dragonMeshes.size()
                      << " GPU mesh parts\n";
        }

        // Static environment geometry: floor box, sky dome, sun disc.
        std::cout << "demo: uploading environment meshes (floor, sky dome, sun disc)...\n";
        const MeshData floorGeometry
            = makeBoxMesh({0.0f, -0.05f, 0.0f}, {roomHalfExtent, 0.05f, roomHalfExtent}, {1, 1, 1}, roomHalfExtent);
        uploadMesh(floorGeometry, floorMesh);
        const MeshData skyGeometry
            = makeSphereMesh({0.0f, 0.0f, 0.0f}, skyDomeRadius, 48, 24, {0.72f, 0.78f, 0.86f}, {0.30f, 0.48f, 0.78f});
        uploadMesh(skyGeometry, skyDomeMesh);
        const MeshData sunGeometry
            = makeSphereMesh(sunDirection * sunDistance, sunRadius, 24, 12, sunDiscRadiance, sunDiscRadiance);
        uploadMesh(sunGeometry, sunDiscMesh);
    }

    // Benchmark grid: gridWidth^2 objects, spatially grouped into chunkWidth x
    // chunkWidth chunks. Each chunk gets one merged batch per model type (rock/
    // torus), so the static scene costs batchCount draw calls instead of one
    // per object. Benchmark-only: the character screen does not need it, but
    // the handles/containers stay live at function scope for the frame loop
    // and teardown.
    MeshData             rockGeometry{};
    MeshData             torusGeometry{};
    renderer::MeshHandle rockMesh  = renderer::invalidMesh;
    renderer::MeshHandle torusMesh = renderer::invalidMesh;
    struct StaticBatch
    {
        renderer::MeshHandle mesh      = renderer::invalidMesh;
        std::uint32_t        modelType = 0; // 0 = rock, 1 = torus; selects the material.
    };
    struct DynamicObject
    {
        std::uint32_t modelType = 0; // 0 = rock, 1 = torus.
        glm::vec3     basePosition{0.0f};
        float         scale        = 1.0f;
        float         phase        = 0.0f; // Per-object phase offset for the wander motion.
        float         angularSpeed = 1.0f; // Radians/second of the wander/rotation phase.
        glm::mat4     transform{1.0f};
    };
    std::vector<StaticBatch>   staticBatches;
    std::vector<DynamicObject> dynamicObjects;
    if (!characterView)
    {
        constexpr std::uint32_t gridWidth            = 10;
        constexpr std::uint32_t chunkWidth           = 3;
        constexpr std::uint32_t chunksPerAxis        = (gridWidth + chunkWidth - 1) / chunkWidth;
        constexpr std::uint32_t batchCount           = chunksPerAxis * chunksPerAxis * 2;
        constexpr std::uint32_t benchmarkObjectCount = gridWidth * gridWidth;
        constexpr float         objectSpacing        = 6.2f;

        std::cout << "demo: building benchmark meshes (rock 160x80, torus 128x48)...\n";
        rockGeometry  = demo::makeRockMesh(1.0f, 160, 80, {0.48f, 0.56f, 0.62f});
        torusGeometry = demo::makeTorusMesh(1.0f, 0.32f, 128, 48, {0.82f, 0.43f, 0.16f});
        uploadMesh(rockGeometry, rockMesh);
        pumpWindowEvents();
        uploadMesh(torusGeometry, torusMesh);
        pumpWindowEvents();

        // Deterministic layout: checkerboard model assignment, hashed scale/yaw,
        // grid-centered positions. Every 5th object becomes a moving instance; the
        // rest are baked into its chunk's batch geometry.
        std::cout << "demo: merging " << benchmarkObjectCount << " static objects into " << batchCount
                  << " spatial batches...\n";
        std::vector<MeshData> staticBatchGeometry(batchCount);
        dynamicObjects.reserve(benchmarkObjectCount / 5);
        std::uint32_t staticObjectCount = 0;
        for (std::uint32_t row = 0; row < gridWidth; ++row)
        {
            for (std::uint32_t column = 0; column < gridWidth; ++column)
            {
                const std::uint32_t modelType = (row + column) & 1u;
                const float         scale     = 0.82f + static_cast<float>((row * 3 + column * 5) % 4) * 0.11f;
                const float         yaw       = static_cast<float>((row * 7 + column * 11) % 31) * 0.17f;
                const glm::vec3     position((static_cast<float>(column) - 4.5f) * objectSpacing,
                    (modelType == 0 ? 0.94f : 0.70f) * scale,
                    (static_cast<float>(row) - 4.5f) * objectSpacing);
                const glm::mat4     transform = glm::translate(glm::mat4(1.0f), position)
                                              * glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0, 1, 0))
                                              * glm::scale(glm::mat4(1.0f), glm::vec3(scale));

                if (((row * gridWidth + column) % 5u) == 0u)
                {
                    dynamicObjects.push_back(DynamicObject{
                        .modelType    = modelType,
                        .basePosition = position,
                        .scale        = scale,
                        .phase        = static_cast<float>(row * gridWidth + column) * 0.37f,
                        .angularSpeed = 0.25f + static_cast<float>((row + column) % 4) * 0.13f,
                        .transform    = transform});
                    continue;
                }

                const std::uint32_t chunkX     = column / chunkWidth;
                const std::uint32_t chunkZ     = row / chunkWidth;
                const std::uint32_t chunkIndex = chunkZ * chunksPerAxis + chunkX;
                const std::uint32_t batchIndex = chunkIndex * 2 + modelType;
                appendTransformedMesh(
                    staticBatchGeometry[batchIndex], modelType == 0 ? rockGeometry : torusGeometry, transform);
                ++staticObjectCount;
            }
        }

        // Upload the merged batches, then release all CPU-side geometry: batches
        // own their vertices now, and the per-object data is no longer needed.
        std::cout << "demo: uploading benchmark batches to the GPU...\n";
        staticBatches.reserve(batchCount);
        for (std::uint32_t batchIndex = 0; batchIndex < batchCount; ++batchIndex)
        {
            MeshData &geometry = staticBatchGeometry[batchIndex];
            if (geometry.vertices.empty())
            {
                continue;
            }
            StaticBatch batch{.modelType = batchIndex & 1u};
            uploadMesh(geometry, batch.mesh);
            pumpWindowEvents();
            if (batch.mesh != renderer::invalidMesh)
            {
                staticBatches.push_back(batch);
            }
            geometry = MeshData{};
        }
        staticBatchGeometry.clear();
        staticBatchGeometry.shrink_to_fit();
        const std::uint64_t rockTriangleCount  = rockGeometry.indices.size() / 3;
        const std::uint64_t torusTriangleCount = torusGeometry.indices.size() / 3;
        const std::uint64_t benchmarkTriangleCount
            = (benchmarkObjectCount / 2) * (rockTriangleCount + torusTriangleCount);
        std::cout << "demo: benchmark scene " << benchmarkObjectCount << " high-detail objects (" << staticObjectCount
                  << " static in " << staticBatches.size() << " spatial batches, " << dynamicObjects.size()
                  << " moving), " << benchmarkTriangleCount << " model triangles (rock=" << rockTriangleCount
                  << ", torus=" << torusTriangleCount << ")\n";
        rockGeometry  = MeshData{};
        torusGeometry = MeshData{};
    }

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
        .baseColorTexture         = floorAlbedo,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = floorMetalRough,
        .normalTexture            = floorNormal,
        .normalScale              = 0.9f,
        .occlusionTexture         = floorOcclusion,
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
        .baseColor                = {sunDiscRadiance.r, sunDiscRadiance.g, sunDiscRadiance.b},
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
        drawCommands.reserve(3 + staticBatches.size());
        const auto addDraw
            = [&drawCommands](
                  renderer::MeshHandle mesh, const glm::mat4 &transform, const renderer::MeshMaterial &material)
        {
            if (mesh != renderer::invalidMesh)
            {
                drawCommands.push_back({.mesh = mesh, .modelMatrix = glm::value_ptr(transform), .material = &material});
            }
        };
        addDraw(skyDomeMesh, skyTransform, skyMaterial);
        addDraw(sunDiscMesh, sunTransform, sunMaterial);
        const std::size_t sortableStart = drawCommands.size();
        addDraw(floorMesh, identityTransform, floorMaterial);
        for (const StaticBatch &batch : staticBatches)
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

    // The main_menu mod is a Lua script hosted by the script_runtime module.
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
                     "mod is disabled\n";
    }
    else
    {
        menuRuntime = scriptCapability->hostedRuntime(scriptCapability->self, "main_menu");
        if (menuRuntime == script_runtime::invalidScriptRuntime)
        {
            std::cerr << "demo: the main_menu script mod is not hosted; the menu is "
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
            characterScreen.configureCamera(
                demoOptions.cameraYawDegrees, demoOptions.cameraPitchDegrees, demoOptions.orbitDistance);
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
    dynamicRockTransforms.reserve(dynamicObjects.size());
    dynamicTorusTransforms.reserve(dynamicObjects.size());
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
        for (DynamicObject &object : dynamicObjects)
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
        for (DynamicObject &object : dynamicObjects)
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
    for (const DynamicObject &object : dynamicObjects)
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
    // Paced runs sleep to just before the deadline and spin the remainder:
    // OS sleep granularity (1-4 ms Windows, ~50 us timer slack on Linux)
    // otherwise lands directly in the frame-time P99. Uncapped runs skip
    // pacing entirely, so the spin window costs nothing there.
    const std::uint32_t pacerSpinWindowMicroseconds = maxFps > 0 ? 1200u : 0u;
    engine::FramePacer  framePacer(
        {.targetFrameRateHz         = static_cast<double>(std::max<std::int64_t>(maxFps, 0)),
            .spinWindowMicroseconds = pacerSpinWindowMicroseconds});
    framePacer.reset();
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
                    // Script mods listen for key presses on the event bus; the
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

        // Mod callbacks run in these dispatches and may throw; a failing
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
            if (rockMesh != renderer::invalidMesh && !dynamicRockTransforms.empty())
            {
                renderCapability.drawMeshInstances(renderCapability.self,
                    rendererHandle,
                    rockMesh,
                    glm::value_ptr(dynamicRockTransforms.front()),
                    static_cast<std::uint32_t>(dynamicRockTransforms.size()),
                    &rockMaterial);
            }
            if (torusMesh != renderer::invalidMesh && !dynamicTorusTransforms.empty())
            {
                renderCapability.drawMeshInstances(renderCapability.self,
                    rendererHandle,
                    torusMesh,
                    glm::value_ptr(dynamicTorusTransforms.front()),
                    static_cast<std::uint32_t>(dynamicTorusTransforms.size()),
                    &torusMaterial);
            }
            if (!dragonMeshes.empty())
            {
                for (const renderer::MeshHandle dragonMesh : dragonMeshes)
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

        // Publish the frame event last so mods observe the fully rendered state.
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

    // Teardown in reverse creation order: meshes, textures, menu bridge,
    // camera, renderer, window, then the engine host itself.
    renderer::FrameStatistics statistics;
    renderCapability.frameStatistics(renderCapability.self, rendererHandle, &statistics);
    std::cout << "demo: renderer rendered " << statistics.framesRendered << " frames, "
              << statistics.trianglesRasterized << "/" << statistics.trianglesSubmitted << " triangles rasterized\n";

    renderCapability.destroyMesh(renderCapability.self, rendererHandle, floorMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, skyDomeMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, sunDiscMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, rockMesh);
    renderCapability.destroyMesh(renderCapability.self, rendererHandle, torusMesh);
    for (const renderer::MeshHandle dragonMesh : dragonMeshes)
    {
        renderCapability.destroyMesh(renderCapability.self, rendererHandle, dragonMesh);
    }
    for (const StaticBatch &batch : staticBatches)
    {
        renderCapability.destroyMesh(renderCapability.self, rendererHandle, batch.mesh);
    }
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, floorAlbedo);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, floorNormal);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, floorMetalRough);
    renderCapability.destroyTexture(renderCapability.self, rendererHandle, floorOcclusion);
    characterScreen.destroy();
    menuBridge.destroy();
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);

    host.stop();
    std::cout << "demo: stopped cleanly after " << frameIndex << " frames\n";
    return (demoOptions.characterTest && characterView && !characterTestPassed) ? 1 : 0;
}
