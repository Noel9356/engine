# Third party dependencies for the engine.
#
# Acquisition strategy, per dependency, in order of preference:
#
#   1. Already available target: a superproject or a previous include() may
#      have provided the dependency; its target is reused and nothing is
#      fetched. System packages found through find_package produce the same
#      effect: GLFW 3.4+, Catch2 3.16+ and a complete Vulkan SDK (headers
#      including vulkan.hpp plus the loader) are probed explicitly below.
#   2. Vendored copy under third_party/<name>: a drop-in source tree, either a
#      full CMake project (has CMakeLists.txt, added as a subdirectory) or a
#      plain header/source layout (no CMakeLists.txt, wrapped by an engine
#      owned interface target). The directory does not exist in a fresh
#      checkout; create it only when vendoring.
#   3. Previously populated build tree: a <name>-src directory under _deps
#      left by an earlier configure is reused; no network access.
#   4. Pinned download, only when ENGINE_ALLOW_FETCHCONTENT is ON. Git
#      dependencies are pinned to release tags; URL downloads are verified
#      against a SHA256 hash. With ENGINE_ALLOW_FETCHCONTENT=OFF the configure
#      step fails with a precise message instead of touching the network.
#
# FetchContent declarations use the dependency's plain name (glfw, lua, stb,
# ...), so the populated trees are _deps/<name>-src and the built-in override
# is FETCHCONTENT_SOURCE_DIR_<NAME> (for example
# FETCHCONTENT_SOURCE_DIR_GLFW=/path/to/glfw). Engine owned wrapper targets
# keep the engine_ prefix (engine_lua_library, engine_sol2_bindings, ...) to
# avoid colliding with dependency or superproject target names.
#
# Every resolution appends one line to the dependency origin report printed at
# the end of this file, so a configure log always answers "what came from
# where, and what was (not) downloaded".

include(FetchContent)

set(ENGINE_DEPENDENCY_ORIGINS "" CACHE INTERNAL "one <name>: <origin> entry per resolved dependency")

function(engine_report_dependency_origin dependencyName originDescription)
    if(ENGINE_DEPENDENCY_ORIGINS)
        set(ENGINE_DEPENDENCY_ORIGINS
            "${ENGINE_DEPENDENCY_ORIGINS};${dependencyName}: ${originDescription}"
            CACHE INTERNAL "one <name>: <origin> entry per resolved dependency")
    else()
        set(ENGINE_DEPENDENCY_ORIGINS
            "${dependencyName}: ${originDescription}"
            CACHE INTERNAL "one <name>: <origin> entry per resolved dependency")
    endif()
endfunction()

option(ENGINE_ALLOW_FETCHCONTENT
       "Allow downloading pinned dependencies when they are not already available" ON)

set(FETCHCONTENT_QUIET OFF)

set(ENGINE_THIRD_PARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party")

# Downloads one file and verifies its SHA256 hash. If the TLS handshake fails
# (MinGW CMake bundles curl/GnuTLS without a CA trust store), the download is
# retried without certificate verification: the hash check still guarantees
# integrity, which is why this helper must only be used for hash checked pins.
function(engine_download_hash_checked url destination sha256 dependencyName)
    if(EXISTS "${destination}")
        return()
    endif()
    if(NOT ENGINE_ALLOW_FETCHCONTENT)
        message(FATAL_ERROR
                "dependency '${dependencyName}' is not available: no cached copy at ${destination} "
                "and ENGINE_ALLOW_FETCHCONTENT is OFF (offline builds need vendored dependencies, "
                "system packages, or one earlier configure with fetching enabled)")
    endif()
    file(DOWNLOAD "${url}" "${destination}" STATUS engineDownloadStatus)
    list(GET engineDownloadStatus 0 engineDownloadError)
    if(engineDownloadError)
        list(GET engineDownloadStatus 1 engineDownloadMessage)
        message(WARNING
                "downloading ${url} failed (${engineDownloadMessage}); retrying without TLS "
                "certificate verification. Integrity is not at risk: the file is rejected unless "
                "its SHA256 hash matches the pinned value.")
        file(DOWNLOAD "${url}" "${destination}" STATUS engineDownloadStatus TLS_VERIFY 0)
        list(GET engineDownloadStatus 0 engineDownloadError)
        if(engineDownloadError)
            list(GET engineDownloadStatus 1 engineDownloadMessage)
            message(FATAL_ERROR "downloading ${url} failed: ${engineDownloadMessage}")
        endif()
    endif()
    file(SHA256 "${destination}" engineDownloadActualHash)
    if(NOT engineDownloadActualHash STREQUAL sha256)
        file(REMOVE "${destination}")
        message(FATAL_ERROR
                "downloaded ${url} has SHA256 ${engineDownloadActualHash}, expected ${sha256}")
    endif()
endfunction()

# Resolves one dependency. The call guarantees afterwards that either the
# target <targetName> exists, or (for header only dependencies without a CMake
# project) that <dependencyName>_SOURCE_DIR points at a usable source
# directory in the caller's scope. All arguments after <targetName> are
# forwarded to FetchContent_Declare and only used on the fetch path.
# PIN_LABEL overrides the pin description shown in the origin report (used
# when the declared URL is a pre-downloaded local file).
function(engine_acquire_dependency dependencyName targetName)
    if(TARGET ${targetName})
        message(STATUS "engine dependency '${dependencyName}': target '${targetName}' already available, not fetching")
        engine_report_dependency_origin(${dependencyName}
            "already available (existing target from a superproject or find_package)")
        return()
    endif()

    set(vendoredDirectory "${ENGINE_THIRD_PARTY_DIR}/${dependencyName}")
    if(EXISTS "${vendoredDirectory}/CMakeLists.txt")
        message(STATUS "engine dependency '${dependencyName}': using vendored project ${vendoredDirectory}")
        engine_report_dependency_origin(${dependencyName} "vendored project third_party/${dependencyName}")
        add_subdirectory("${vendoredDirectory}" "${CMAKE_BINARY_DIR}/vendored/${dependencyName}" EXCLUDE_FROM_ALL)
        if(NOT TARGET ${targetName})
            message(FATAL_ERROR
                    "vendored dependency '${dependencyName}' did not define the expected target '${targetName}'")
        endif()
        return()
    elseif(EXISTS "${vendoredDirectory}")
        message(STATUS "engine dependency '${dependencyName}': using vendored sources ${vendoredDirectory}")
        engine_report_dependency_origin(${dependencyName} "vendored sources third_party/${dependencyName}")
        set(${dependencyName}_SOURCE_DIR "${vendoredDirectory}" PARENT_SCOPE)
        return()
    endif()

    # An already populated <name>-src inside this build tree is reused as-is:
    # FetchContent never re-downloads it, which is what makes the very first
    # online configure the only network access this tree will ever need.
    cmake_parse_arguments(PARSE_ARGV 2 engineAcquire
                          "" "PIN_LABEL;GIT_TAG;GIT_SHALLOW;SOURCE_SUBDIR;URL_HASH" "URL;GIT_REPOSITORY")
    if(engineAcquire_GIT_REPOSITORY)
        set(engineAcquirePin "${engineAcquire_GIT_REPOSITORY} at tag ${engineAcquire_GIT_TAG}")
    elseif(engineAcquire_PIN_LABEL)
        set(engineAcquirePin "${engineAcquire_PIN_LABEL}")
    else()
        set(engineAcquirePin "${engineAcquire_URL}")
    endif()
    if(EXISTS "${FETCHCONTENT_BASE_DIR}/${dependencyName}-src")
        set(engineAcquireOrigin "fetched sources reused from this build tree (no download)")
        message(STATUS "engine dependency '${dependencyName}': reusing already populated sources in this build tree")
    else()
        set(engineAcquireOrigin "downloaded (pinned ${engineAcquirePin})")
        message(STATUS "engine dependency '${dependencyName}': fetching pinned sources")
    endif()

    # Offline guard: populated sources or an already cached local URL file are
    # usable without the network; anything else is a precise failure.
    set(engineAcquireOfflineSourceExists FALSE)
    if(engineAcquire_URL AND EXISTS "${engineAcquire_URL}")
        set(engineAcquireOfflineSourceExists TRUE)
    endif()
    if(NOT ENGINE_ALLOW_FETCHCONTENT
       AND NOT EXISTS "${FETCHCONTENT_BASE_DIR}/${dependencyName}-src"
       AND NOT engineAcquireOfflineSourceExists)
        message(FATAL_ERROR
                "dependency '${dependencyName}' is not available: target '${targetName}' does not exist, "
                "third_party/${dependencyName} is not vendored, this build tree has no previously populated "
                "sources, and ENGINE_ALLOW_FETCHCONTENT is OFF (offline builds need vendored dependencies, "
                "system packages, or one earlier configure with fetching enabled)")
    endif()

    engine_report_dependency_origin(${dependencyName} "${engineAcquireOrigin}")
    # Forward everything except the PIN_LABEL display-only keyword pair to
    # FetchContent: unknown keywords would be swallowed by ExternalProject's
    # multi-value options and corrupt the dependency's arguments.
    set(engineAcquireForwardArgs "${ARGN}")
    list(FIND engineAcquireForwardArgs "PIN_LABEL" engineAcquirePinLabelIndex)
    if(NOT engineAcquirePinLabelIndex EQUAL -1)
        math(EXPR engineAcquirePinLabelValueIndex "${engineAcquirePinLabelIndex} + 1")
        list(REMOVE_AT engineAcquireForwardArgs
             "${engineAcquirePinLabelIndex}" "${engineAcquirePinLabelValueIndex}")
    endif()
    FetchContent_Declare(${dependencyName} ${engineAcquireForwardArgs})
    FetchContent_MakeAvailable(${dependencyName})

    # FetchContent defines <name>_SOURCE_DIR in the caller's scope; forward it
    # out of this function so wrapping code sees one uniform variable.
    foreach(scopeVariable SOURCE_DIR BINARY_DIR)
        if(DEFINED ${dependencyName}_${scopeVariable})
            set(${dependencyName}_${scopeVariable} "${${dependencyName}_${scopeVariable}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

# ---- Vulkan (headers, C++ bindings, loader) -----------------------------------
# A complete system install (the Vulkan SDK ships matching vulkan.h and
# vulkan.hpp, which come from the same Vulkan-Headers release) is used as-is:
# no download. Only a missing or incomplete install falls back to the pinned
# Vulkan-Headers + Vulkan-Hpp pair, fetched at one tag so the two can never
# skew. Set ENGINE_FORCE_PINNED_VULKAN=ON to ignore the system SDK.

option(ENGINE_FORCE_PINNED_VULKAN
       "Ignore the system Vulkan SDK and use the pinned Vulkan-Headers + Vulkan-Hpp sources" OFF)

set(ENGINE_VULKAN_FROM_SYSTEM FALSE)
if(NOT ENGINE_FORCE_PINNED_VULKAN)
    find_package(Vulkan QUIET)
    if(Vulkan_FOUND AND EXISTS "${Vulkan_INCLUDE_DIRS}/vulkan/vulkan.hpp")
        set(ENGINE_VULKAN_FROM_SYSTEM TRUE)
    elseif(Vulkan_FOUND)
        message(STATUS "system Vulkan SDK found but vulkan.hpp is missing; using the pinned Vulkan-Headers + Vulkan-Hpp pair")
    endif()
endif()

if(ENGINE_VULKAN_FROM_SYSTEM)
    message(STATUS "engine dependency 'vulkan': using system Vulkan SDK headers at ${Vulkan_INCLUDE_DIRS}")
    engine_report_dependency_origin(vulkan "already available (Vulkan SDK: ${Vulkan_INCLUDE_DIRS})")
    # FindVulkan provides Vulkan::Headers (used by the kernel) and
    # Vulkan::Vulkan (headers plus loader, which static dispatch in vulkan-hpp
    # needs).
    if(NOT TARGET engine_vulkan_hpp_bindings)
        add_library(engine_vulkan_hpp_bindings INTERFACE)
        target_link_libraries(engine_vulkan_hpp_bindings INTERFACE Vulkan::Vulkan)
    endif()
else()
    engine_acquire_dependency(vulkan_headers Vulkan::Headers
        GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
        GIT_TAG v1.4.365
        GIT_SHALLOW TRUE
    )

    engine_acquire_dependency(vulkan_hpp engine_vulkan_hpp_bindings
        GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Hpp.git
        GIT_TAG v1.4.365
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR cmake_placeholder # populate only: no add_subdirectory
    )

    if(NOT TARGET engine_vulkan_hpp_bindings)
        add_library(engine_vulkan_hpp_bindings INTERFACE)
        target_include_directories(engine_vulkan_hpp_bindings INTERFACE "${vulkan_hpp_SOURCE_DIR}")
        target_link_libraries(engine_vulkan_hpp_bindings INTERFACE Vulkan::Headers)

        # Static dispatch in vulkan-hpp calls the loader entry points directly,
        # so the loader library must be linked. find_library honors
        # CMAKE_PREFIX_PATH and system paths; the Vulkan SDK provides
        # vulkan-1.lib on Windows.
        find_library(ENGINE_VULKAN_LOADER_LIBRARY
                     NAMES vulkan vulkan-1
                     HINTS "$ENV{VULKAN_SDK}/Lib" "$ENV{VULKAN_SDK}/lib")
        if(NOT ENGINE_VULKAN_LOADER_LIBRARY)
            # Some Linux systems ship only the versioned loader; accept it as a
            # last resort so linking does not require the development symlink.
            find_file(ENGINE_VULKAN_LOADER_FILE
                      NAMES libvulkan.so.1 libvulkan.1.so
                      PATHS /usr/lib/x86_64-linux-gnu /usr/lib64 /usr/lib
                      )
            if(ENGINE_VULKAN_LOADER_FILE)
                set(ENGINE_VULKAN_LOADER_LIBRARY "${ENGINE_VULKAN_LOADER_FILE}")
            endif()
        endif()
        if(NOT ENGINE_VULKAN_LOADER_LIBRARY)
            message(FATAL_ERROR
                    "Vulkan loader not found: install libvulkan-dev (Linux) or the Vulkan SDK (Windows), "
                    "or pass -DCMAKE_PREFIX_PATH pointing at a prefix containing the loader")
        endif()
        message(STATUS "Vulkan loader: ${ENGINE_VULKAN_LOADER_LIBRARY}")
        target_link_libraries(engine_vulkan_hpp_bindings INTERFACE "${ENGINE_VULKAN_LOADER_LIBRARY}")
    endif()
endif()

# ---- GLFW (window and input backends) ----------------------------------------
# A system GLFW 3.4+ is probed first; the fetched copy is the fallback.

if(NOT TARGET glfw)
    find_package(glfw3 3.4 CONFIG QUIET)
endif()

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

engine_acquire_dependency(glfw glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)

# ---- Lua 5.4 (script mods) ----------------------------------------------------
# Not probed through find_package: sol2 requires Lua 5.4 exactly and a system
# Lua is frequently 5.1/5.3. Lua ships no CMake project, so the pinned tarball
# is downloaded (hash checked, with the TLS fallback below), cached in the
# build tree, and the library is built from its sources directly. lua.c, luac.c
# and onelua.c are interpreter front ends, not library members.

set(ENGINE_LUA_URL "https://www.lua.org/ftp/lua-5.4.8.tar.gz")
set(ENGINE_LUA_SHA256 "4f18ddae154e793e46eeab727c59ef1c0c0c2b744e7b94219710d76f530629ae")
set(ENGINE_LUA_TARBALL "${CMAKE_BINARY_DIR}/deps/lua-5.4.8.tar.gz")

if(NOT TARGET engine_lua_library
   AND NOT EXISTS "${ENGINE_THIRD_PARTY_DIR}/lua/CMakeLists.txt"
   AND NOT EXISTS "${ENGINE_THIRD_PARTY_DIR}/lua/src/lua.h"
   AND NOT EXISTS "${FETCHCONTENT_BASE_DIR}/lua-src")
    engine_download_hash_checked("${ENGINE_LUA_URL}" "${ENGINE_LUA_TARBALL}" "${ENGINE_LUA_SHA256}" lua)
endif()

engine_acquire_dependency(lua engine_lua_library
    URL "${ENGINE_LUA_TARBALL}"
    URL_HASH SHA256=${ENGINE_LUA_SHA256}
    PIN_LABEL "${ENGINE_LUA_URL} (SHA256 checked)"
    SOURCE_SUBDIR cmake_placeholder # populate only: no add_subdirectory
)

if(NOT TARGET engine_lua_library)
    file(GLOB ENGINE_LUA_SOURCES "${lua_SOURCE_DIR}/src/*.c")
    list(FILTER ENGINE_LUA_SOURCES EXCLUDE REGEX ".*/(lua|luac|onelua)\\.c$")
    add_library(engine_lua_library STATIC ${ENGINE_LUA_SOURCES})
    target_include_directories(engine_lua_library PUBLIC "${lua_SOURCE_DIR}/src")
endif()

# ---- sol2 (Lua bindings, header only) -----------------------------------------

engine_acquire_dependency(sol2 engine_sol2_bindings
    GIT_REPOSITORY https://github.com/ThePhD/sol2.git
    GIT_TAG v3.5.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR cmake_placeholder # populate only: no add_subdirectory
)

if(NOT TARGET engine_sol2_bindings)
    add_library(engine_sol2_bindings INTERFACE)
    target_include_directories(engine_sol2_bindings INTERFACE "${sol2_SOURCE_DIR}/include")
    target_link_libraries(engine_sol2_bindings INTERFACE engine_lua_library)
    target_compile_definitions(engine_sol2_bindings INTERFACE SOL_ALL_SAFETIES_ON=1)
endif()

# ---- toml++ (configuration and mod manifests, header only) ---------------------

engine_acquire_dependency(tomlplusplus engine_tomlplusplus_bindings
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR cmake_placeholder # populate only: no add_subdirectory
)

if(NOT TARGET engine_tomlplusplus_bindings)
    add_library(engine_tomlplusplus_bindings INTERFACE)
    # The repository keeps the headers in a "toml++" directory at its root;
    # the include root is therefore the source directory itself.
    target_include_directories(engine_tomlplusplus_bindings INTERFACE "${tomlplusplus_SOURCE_DIR}")
endif()

# ---- stb (PNG writing for the screenshot module, image loading for the
# renderer's texture sets) ---------------------------------------------------------
# Two pinned single files from the same stb commit: vendored copy first, then
# a hash checked download that only happens once per build tree.

set(ENGINE_STB_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
set(ENGINE_STB_DIR "${CMAKE_BINARY_DIR}/deps/stb")

set(ENGINE_STB_IMAGE_WRITE_SHA256 "cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05")
set(ENGINE_STB_IMAGE_WRITE_URL "https://raw.githubusercontent.com/nothings/stb/${ENGINE_STB_COMMIT}/stb_image_write.h")
set(ENGINE_STB_IMAGE_SHA256 "594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3")
set(ENGINE_STB_IMAGE_URL "https://raw.githubusercontent.com/nothings/stb/${ENGINE_STB_COMMIT}/stb_image.h")

if(EXISTS "${ENGINE_THIRD_PARTY_DIR}/stb/stb_image_write.h")
    set(ENGINE_STB_DIR "${ENGINE_THIRD_PARTY_DIR}/stb")
    message(STATUS "engine dependency 'stb': using vendored sources ${ENGINE_STB_DIR}")
    engine_report_dependency_origin(stb "vendored sources third_party/stb")
elseif(EXISTS "${ENGINE_STB_DIR}/stb_image_write.h")
    engine_report_dependency_origin(stb "cached single header reused from build/deps/stb (no download)")
else()
    engine_download_hash_checked("${ENGINE_STB_IMAGE_WRITE_URL}" "${ENGINE_STB_DIR}/stb_image_write.h"
                                 "${ENGINE_STB_IMAGE_WRITE_SHA256}" stb)
    engine_report_dependency_origin(stb "single header downloaded (SHA256 checked, cached in build/deps/stb)")
endif()

if(NOT EXISTS "${ENGINE_STB_DIR}/stb_image.h")
    engine_download_hash_checked("${ENGINE_STB_IMAGE_URL}" "${ENGINE_STB_DIR}/stb_image.h"
                                 "${ENGINE_STB_IMAGE_SHA256}" stb_image)
endif()

if(NOT TARGET engine_stb_bindings)
    add_library(engine_stb_bindings INTERFACE)
    # SYSTEM: stb is a third-party single header; its warnings must not leak
    # into the engine's warning-clean build.
    target_include_directories(engine_stb_bindings SYSTEM INTERFACE "${ENGINE_STB_DIR}")
endif()

# ---- Catch2 (tests) -------------------------------------------------------------
# A system Catch2 3.x is probed first; the fetched copy is the fallback. Only
# needed when ENGINE_BUILD_TESTS is ON.

if(NOT TARGET Catch2::Catch2WithMain)
    find_package(Catch2 3.16.1 CONFIG QUIET)
endif()

set(CATCH_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CATCH_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
set(CATCH_INSTALL_EXTRAS OFF CACHE BOOL "" FORCE)

engine_acquire_dependency(catch2 Catch2::Catch2WithMain
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.16.1
    GIT_SHALLOW TRUE
)

# ---- Origin report -------------------------------------------------------------
# One consolidated configure-time summary: for every third party dependency,
# where it came from and whether the network was involved. Air-gapped or
# reproducible builds can be checked by grepping the configure log for
# "downloaded".

list(REMOVE_DUPLICATES ENGINE_DEPENDENCY_ORIGINS)
message(STATUS "engine dependency origins:")
foreach(engineOriginEntry IN LISTS ENGINE_DEPENDENCY_ORIGINS)
    message(STATUS "  ${engineOriginEntry}")
endforeach()
