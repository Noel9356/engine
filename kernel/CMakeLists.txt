include(engine_fetch_dependency)

engine_fetch_dependency(
    tomlplusplus
    OWNER
    kernel
    GIT_REPOSITORY
    https://github.com/marzer/tomlplusplus.git
    GIT_TAG
    v3.4.0
    HEADER_ONLY
    INCLUDE_SUBDIRS
    include
    include/toml++
)

add_library(
    engine_kernel
    SHARED
    source/capability_provider_index.cpp
    source/participant_context.cpp
    source/configuration.cpp
    source/dependency_resolver.cpp
    source/dynamic_library.cpp
    source/engine_kernel.cpp
    source/engine_kernel/configuration.cpp
    source/engine_kernel/discovery.cpp
    source/engine_kernel/module_loading.cpp
    source/engine_kernel/lifecycle.cpp
    source/executable_directory.cpp
    source/game_mod_discovery.cpp
    source/game_mod_manifest.cpp
    source/module_manifest.cpp
    source/module_descriptor_cache.cpp
    source/trust_gate.cpp
    source/sha256.cpp
)

set_target_properties(
    engine_kernel
    PROPERTIES PREFIX
               "lib"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime"
               LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime"
               WINDOWS_EXPORT_ALL_SYMBOLS
               ON
)

target_include_directories(
    engine_kernel
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_kernel
    PRIVATE engine_dep_tomlplusplus
    PRIVATE Threads::Threads
)

if(MSVC)
    target_compile_options(
        engine_kernel
        PUBLIC /W4
               /permissive-
    )
else()
    target_compile_options(
        engine_kernel
        PUBLIC -Wall
               -Wextra
               -Wpedantic
    )
endif()
