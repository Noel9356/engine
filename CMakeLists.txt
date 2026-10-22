cmake_minimum_required(VERSION 3.24)

project(engine VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Native mods are shared libraries that link the kernel statically; every
# engine object must therefore be position independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Platform preprocessor setup -------------------------------------------------
# windows.h must keep min/max as functions and stay lean. Defining this once
# for the whole tree replaces per-file defines, which clash with MinGW's
# libstdc++: its <filesystem> and <string> already define NOMINMAX, so a later
# #define in a source file produces a redefinition warning on every build.
if(WIN32)
    add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# ---- Options: compile time module selection ------------------------------------
# Every module can be excluded from the build; the runtime configuration must
# then not list it either, and the startup report names any mismatch.

option(ENGINE_BUILD_MODULE_LOGGING "Include the logging module in the build" ON)
option(ENGINE_BUILD_MODULE_WINDOW "Include the window module in the build" ON)
option(ENGINE_BUILD_MODULE_INPUT "Include the input module in the build" ON)
option(ENGINE_BUILD_MODULE_RENDERER "Include the renderer module in the build" ON)
option(ENGINE_BUILD_MODULE_SCREENSHOT "Include the screenshot module in the build" ON)
option(ENGINE_BUILD_MODULE_LUA_SCRIPTING "Include the lua scripting module in the build" ON)
option(ENGINE_BUILD_MODULE_PBR "Include the pbr lighting technique module in the build" ON)
option(ENGINE_BUILD_MODULE_RADIANCE_CASCADES "Include the radiance cascades lighting technique module in the build" ON)
option(ENGINE_BUILD_MODULE_RAY_REFLECTIONS "Include the ray traced reflections lighting technique module in the build" ON)
option(ENGINE_BUILD_DEMO "Build the demo host" ON)
option(ENGINE_BUILD_TESTS "Build the kernel and module tests" ON)

# ---- Dependencies and generated files -------------------------------------------

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(engine_dependencies)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")

set(ENGINE_GENERATED_INCLUDE_DIR "${CMAKE_BINARY_DIR}/generated")
configure_file(cmake/engine_module_availability.h.in "${ENGINE_GENERATED_INCLUDE_DIR}/engine_module_availability.h" @ONLY)

# ---- Engine targets ---------------------------------------------------------------
# The tree is flat by design: kernel/, modules/<name>/, host/ and mods/ are
# siblings at the root. A module directory is self-contained (sources, headers,
# shaders); a mod directory is a runtime package (mod.toml plus entry sources).

add_subdirectory(kernel)

if(ENGINE_BUILD_MODULE_LOGGING)
    add_subdirectory(modules/logging)
endif()
if(ENGINE_BUILD_MODULE_WINDOW)
    add_subdirectory(modules/window)
endif()
if(ENGINE_BUILD_MODULE_INPUT)
    add_subdirectory(modules/input)
endif()
if(ENGINE_BUILD_MODULE_RENDERER)
    add_subdirectory(modules/renderer)
endif()
if(ENGINE_BUILD_MODULE_SCREENSHOT)
    add_subdirectory(modules/screenshot)
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

if(ENGINE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
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
