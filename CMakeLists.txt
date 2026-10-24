cmake_minimum_required(VERSION 3.24)

project(engine VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Native mods and module plugins are shared libraries that link the kernel
# dynamically; every engine object must therefore be position independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Platform preprocessor setup -------------------------------------------------
# windows.h must keep min/max as functions and stay lean. Defining this once
# for the whole tree replaces per-file defines, which clash with MinGW's
# libstdc++: its <filesystem> and <string> already define NOMINMAX, so a later
# #define in a source file produces a redefinition warning on every build.
if(WIN32)
    add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# ---- Options: runtime module selection -----------------------------------------
# Every module can be excluded from the build; its plugin binary is then not
# produced, and a run whose mods demand a capability only it provides fails
# startup with a precise diagnostic naming the consumer.

option(ENGINE_BUILD_MODULE_BVH "Include the bvh (binary median-split hierarchy) module in the build" ON)
option(ENGINE_BUILD_MODULE_LOGGING_STDOUT "Include the logging (stdout/file provider) module in the build" ON)
option(ENGINE_BUILD_MODULE_WINDOW_GLFW "Include the window module in the build" ON)
option(ENGINE_BUILD_MODULE_INPUT_GLFW "Include the input module in the build" ON)
option(ENGINE_BUILD_MODULE_RENDERER_VULKAN "Include the renderer module in the build" ON)
option(ENGINE_BUILD_MODULE_SCREENSHOT_PNG "Include the screenshot module in the build" ON)
option(ENGINE_BUILD_MODULE_SCENE_REGISTRY "Include the scene (runtime object registry) module in the build" ON)
option(ENGINE_BUILD_MODULE_ANIMATION_TWEEN "Include the animation (tween drivers) module in the build" ON)
option(ENGINE_BUILD_MODULE_ASSET_GLTF "Include the asset (glTF import) module in the build" ON)
option(ENGINE_BUILD_MODULE_PHYSICS_BUILTIN "Include the physics (builtin rigid body provider) module in the build" ON)
option(ENGINE_BUILD_MODULE_AUDIO_MINIAUDIO "Include the audio (miniaudio provider) module in the build" ON)
option(ENGINE_BUILD_MODULE_PERSISTENCE_TOML "Include the persistence (registry save/load) module in the build" ON)
option(ENGINE_BUILD_MODULE_UI_IMGUI "Include the ui (debug overlay) module in the build" ON)
option(ENGINE_BUILD_MODULE_NET_TCP "Include the net (tcp transport) module in the build" ON)
option(ENGINE_BUILD_MODULE_HOT_RELOAD_WATCHER "Include the hot reload (filesystem watcher) module in the build" ON)
option(ENGINE_BUILD_MODULE_LUA_SCRIPTING "Include the lua scripting module in the build" ON)
option(ENGINE_BUILD_MODULE_PBR "Include the pbr lighting technique module in the build" ON)
option(ENGINE_BUILD_MODULE_RADIANCE_CASCADES "Include the radiance cascades lighting technique module in the build" ON)
option(ENGINE_BUILD_MODULE_RAY_REFLECTIONS "Include the ray traced reflections lighting technique module in the build" ON)
option(ENGINE_BUILD_DEMO "Build the demo host" ON)

# ---- Runtime layout -------------------------------------------------------------
# Every engine runtime artifact lands in build/runtime/: the executable, the
# kernel shared library, the GLFW runtime the window and input plugins share,
# and the modules/ plugin directory. These two set() calls MUST run BEFORE
# include(engine_dependencies): a directory-scope set() writes the scope at
# the moment it executes, and the dependency fetch's add_subdirectory
# snapshots that scope when it runs. After the include, the fetched GLFW
# shared runtime keeps its default location under _deps/ and the runtime
# layout assembles without it - the window and input plugins then fail to
# load with an unsatisfied glfw dependency.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(engine_dependencies)

# Threading support for the kernel's async task dispatcher (one worker
# thread per dispatcher; the frame loop and the bus stay single threaded).
find_package(Threads REQUIRED)
# The portability guard for the modules' published capability headers; every
# module's CMakeLists calls it for the headers it publishes.
include(capability_header_guard)

# ---- Engine targets ---------------------------------------------------------------
# The tree is flat by design: kernel/, modules/<name>/, host/ and mods/ are
# siblings at the root. A module directory is self-contained (sources, its
# published capability headers, shaders); a mod directory is a runtime package
# (mod.toml plus entry sources). There is no central capability directory: a
# capability's types are published by the module that provides it, and
# consumers include that header directly.

add_subdirectory(kernel)

if(ENGINE_BUILD_MODULE_BVH)
    add_subdirectory(modules/bvh)
endif()
if(ENGINE_BUILD_MODULE_LOGGING_STDOUT)
    add_subdirectory(modules/logging_stdout)
endif()
if(ENGINE_BUILD_MODULE_WINDOW_GLFW)
    add_subdirectory(modules/window_glfw)
endif()
if(ENGINE_BUILD_MODULE_INPUT_GLFW)
    add_subdirectory(modules/input_glfw)
endif()
if(ENGINE_BUILD_MODULE_RENDERER_VULKAN)
    add_subdirectory(modules/renderer_vulkan)
endif()
if(ENGINE_BUILD_MODULE_SCREENSHOT_PNG)
    add_subdirectory(modules/screenshot_png)
endif()
if(ENGINE_BUILD_MODULE_SCENE_REGISTRY)
    add_subdirectory(modules/scene_registry)
endif()
if(ENGINE_BUILD_MODULE_ANIMATION_TWEEN)
    add_subdirectory(modules/animation_tween)
endif()
if(ENGINE_BUILD_MODULE_ASSET_GLTF)
    add_subdirectory(modules/asset_gltf)
endif()
if(ENGINE_BUILD_MODULE_PHYSICS_BUILTIN)
    add_subdirectory(modules/physics_builtin)
endif()
if(ENGINE_BUILD_MODULE_AUDIO_MINIAUDIO)
    add_subdirectory(modules/audio_miniaudio)
endif()
if(ENGINE_BUILD_MODULE_PERSISTENCE_TOML)
    add_subdirectory(modules/persistence_toml)
endif()
if(ENGINE_BUILD_MODULE_UI_IMGUI)
    add_subdirectory(modules/ui_imgui)
endif()
if(ENGINE_BUILD_MODULE_NET_TCP)
    add_subdirectory(modules/net_tcp)
endif()
if(ENGINE_BUILD_MODULE_HOT_RELOAD_WATCHER)
    add_subdirectory(modules/hot_reload_watcher)
endif()
if(ENGINE_BUILD_MODULE_LUA_SCRIPTING)
    add_subdirectory(modules/lua_scripting)
endif()
if(ENGINE_BUILD_MODULE_PBR)
    add_subdirectory(modules/pbr)
endif()
if(ENGINE_BUILD_MODULE_RADIANCE_CASCADES)
    add_subdirectory(modules/radiance_cascades)
endif()
if(ENGINE_BUILD_MODULE_RAY_REFLECTIONS)
    add_subdirectory(modules/ray_reflections)
endif()

if(ENGINE_BUILD_DEMO)
    add_subdirectory(host)
endif()

# ---- Runtime layout -----------------------------------------------------------
# Mod packages are assembled into runtime/mods/<id>/: the manifest always, the
# package's shipped defaults (config.toml) when present, every script file for
# script mods (native libraries are placed there by the mod's own CMakeLists
# through LIBRARY_OUTPUT_DIRECTORY), and the package's content assets
# (assets/) when it ships any - the engine itself ships no content.

if(ENGINE_BUILD_DEMO)
    set(ENGINE_MOD_PACKAGE_COMMANDS)
    file(GLOB ENGINE_MOD_PACKAGE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/mods/*")
    foreach(modPackageDirectory IN LISTS ENGINE_MOD_PACKAGE_DIRS)
        if(NOT EXISTS "${modPackageDirectory}/mod.toml")
            continue()
        endif()
        get_filename_component(modPackageId "${modPackageDirectory}" NAME)
        list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
             COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     "${modPackageDirectory}/mod.toml"
                     "${CMAKE_BINARY_DIR}/runtime/mods/${modPackageId}/mod.toml")
        if(EXISTS "${modPackageDirectory}/config.toml")
            list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
                 COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${modPackageDirectory}/config.toml"
                         "${CMAKE_BINARY_DIR}/runtime/mods/${modPackageId}/config.toml")
        endif()
        if(EXISTS "${modPackageDirectory}/scene.toml")
            # The world content the manifest may declare through its scene
            # key. The manifest accepts any package-relative path, but the
            # conventional name is scene.toml and this rule copies exactly
            # that; a package using a different name brings the file along
            # through its own packaging (native mod CMake rules or assets/).
            list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
                 COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${modPackageDirectory}/scene.toml"
                         "${CMAKE_BINARY_DIR}/runtime/mods/${modPackageId}/scene.toml")
        endif()
        file(GLOB modPackageScripts "${modPackageDirectory}/*.lua")
        foreach(modScript IN LISTS modPackageScripts)
            list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
                 COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${modScript}"
                         "${CMAKE_BINARY_DIR}/runtime/mods/${modPackageId}/")
        endforeach()
        if(EXISTS "${modPackageDirectory}/assets")
            list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
                 COMMAND ${CMAKE_COMMAND} -E copy_directory
                         "${modPackageDirectory}/assets"
                         "${CMAKE_BINARY_DIR}/runtime/mods/${modPackageId}/assets")
        endif()
    endforeach()

    # Shipped module defaults: a module directory may carry a module.toml; the
    # kernel scans runtime/config/modules/<name>/module.toml and merges it as
    # the lowest configuration layer (docs/configuration.md, "The layer
    # stack"). No in-tree module ships one today; the rule keeps the runtime
    # layout correct the day one does.
    file(GLOB ENGINE_MODULE_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/modules/*/module.toml")
    foreach(moduleDefaultsFile IN LISTS ENGINE_MODULE_DEFAULTS)
        get_filename_component(moduleDefaultsDirectory "${moduleDefaultsFile}" DIRECTORY)
        get_filename_component(moduleDefaultsName "${moduleDefaultsDirectory}" NAME)
        list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
             COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     "${moduleDefaultsFile}"
                     "${CMAKE_BINARY_DIR}/runtime/config/modules/${moduleDefaultsName}/module.toml")
    endforeach()

    add_custom_target(engine_runtime_layout ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/config" "${CMAKE_BINARY_DIR}/runtime/config"
        ${ENGINE_MOD_PACKAGE_COMMANDS}
        COMMENT "assembling the runtime layout (config, mod packages with their content assets)")
endif()
