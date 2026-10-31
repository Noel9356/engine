cmake_minimum_required(VERSION 3.24)

project(engine VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(WIN32)
    add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime")

if(APPLE)
    set(ENGINE_PLUGIN_RPATH_BUILD "@loader_path;@loader_path/..")
    set(ENGINE_PLUGIN_RPATH_INSTALL "@loader_path;@loader_path/..")
else()
    set(ENGINE_PLUGIN_RPATH_BUILD "$ORIGIN;$ORIGIN/..")
    set(ENGINE_PLUGIN_RPATH_INSTALL "$ORIGIN:$ORIGIN/..")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(engine_dependencies)

option(ENGINE_ENABLE_LTO "Enable link-time / interprocedural optimization (opt-in)" OFF)

if(ENGINE_ENABLE_LTO)
    if(WIN32)
        message(STATUS "engine: ENGINE_ENABLE_LTO=ON is not applied on Windows (the kernel's symbol-export mechanism does not compose with whole-program optimization)")
    elseif(CMAKE_BUILD_TYPE MATCHES "Release|MinSizeRel|RelWithDebInfo")
        include(CheckIPOSupported)
        check_ipo_supported(RESULT engineIpoSupported OUTPUT engineIpoError)
        if(engineIpoSupported)
            set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
            message(STATUS "engine: link-time optimization enabled for this build")
        else()
            message(FATAL_ERROR "engine: ENGINE_ENABLE_LTO=ON but this toolchain does not support IPO (${engineIpoError})")
        endif()
    endif()
endif()

find_package(Threads REQUIRED)
include(capability_header_guard)

add_subdirectory(kernel)

file(GLOB ENGINE_MODULE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/modules/*")
list(SORT ENGINE_MODULE_DIRECTORIES)

option(ENGINE_BUILD_DEMO "Build the demo app" ON)

foreach(moduleDirectory IN LISTS ENGINE_MODULE_DIRECTORIES)
    if(EXISTS "${moduleDirectory}/CMakeLists.txt")
        add_subdirectory("${moduleDirectory}")
    endif()
endforeach()

if(ENGINE_BUILD_DEMO)
    add_subdirectory(app)
endif()

if(ENGINE_BUILD_DEMO)
    file(GLOB ENGINE_NATIVE_MOD_PROJECTS "${CMAKE_CURRENT_SOURCE_DIR}/mods/*/CMakeLists.txt")
    foreach(nativeModProject IN LISTS ENGINE_NATIVE_MOD_PROJECTS)
        get_filename_component(nativeModDirectory "${nativeModProject}" DIRECTORY)
        add_subdirectory("${nativeModDirectory}")
    endforeach()

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

    file(GLOB ENGINE_MODULE_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/modules/*/config.toml")
    foreach(moduleDefaultsFile IN LISTS ENGINE_MODULE_DEFAULTS)
        get_filename_component(moduleDefaultsDirectory "${moduleDefaultsFile}" DIRECTORY)
        get_filename_component(moduleDefaultsName "${moduleDefaultsDirectory}" NAME)
        list(APPEND ENGINE_MOD_PACKAGE_COMMANDS
             COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     "${moduleDefaultsFile}"
                     "${CMAKE_BINARY_DIR}/runtime/config/modules/${moduleDefaultsName}/config.toml")
    endforeach()

    add_custom_target(engine_runtime_layout ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/config" "${CMAKE_BINARY_DIR}/runtime/config"
        ${ENGINE_MOD_PACKAGE_COMMANDS}
        COMMENT "assembling the runtime layout (config, mod packages with their content assets)")
endif()
