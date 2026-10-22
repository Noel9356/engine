# The engine kernel: capability contracts, event bus, dependency resolution,
# permission system, configuration and lifecycle. Nothing here knows about
# windows, rendering or scripting - including its headers, which stay free of
# graphics, windowing and platform libraries by the contract purity guard below.

add_library(engine_kernel STATIC
    source/component_context.cpp
    source/configuration.cpp
    source/dependency_resolver.cpp
    source/dynamic_library.cpp
    source/engine_kernel.cpp
    source/filesystem_utilities.cpp
    source/mod_discovery.cpp
    source/mod_manifest.cpp
)

target_include_directories(engine_kernel
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ENGINE_GENERATED_INCLUDE_DIR}"
)

target_link_libraries(engine_kernel
    PRIVATE engine_tomlplusplus_bindings
)

# ---- Contract purity guard -------------------------------------------------------
# Capability contracts are the mod-facing surface: implementation-, platform-
# and vendor-agnostic (mods/README.md, section "Portability"). This configure-
# time check keeps that promise mechanical: a contract that starts including a
# graphics, windowing or platform header fails the configure with a precise
# message instead of silently coupling mods to one renderer implementation.
file(GLOB ENGINE_CONTRACT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/engine/capabilities/*.hpp")
foreach(engineContractHeader ${ENGINE_CONTRACT_HEADERS})
    file(READ "${engineContractHeader}" engineContractContent)
    if(engineContractContent MATCHES "#[ \t]*include[ \t]*[<\"](vulkan/|GLFW/|GL/|X11/|windows\\.h)")
        message(FATAL_ERROR
                "capability contract '${engineContractHeader}' includes an implementation, platform or vendor "
                "specific header. Contracts must stay implementation-, platform- and vendor-agnostic; "
                "implementations live in modules behind opaque handles. See mods/README.md, section Portability.")
    endif()
endforeach()

if(MSVC)
    target_compile_options(engine_kernel PUBLIC /W4 /permissive-)
else()
    target_compile_options(engine_kernel PUBLIC -Wall -Wextra -Wpedantic)
endif()

# The kernel links no platform UI libraries; it is loader, TOML, filesystem and
# event plumbing only.
