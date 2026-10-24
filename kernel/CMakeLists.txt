# The engine kernel: event bus, dependency resolution, permission system,
# configuration and lifecycle. Nothing here knows about windows, rendering or
# scripting, and nothing here knows what any capability means: every
# capability's published header lives in its providing module's directory,
# which the kernel does not include - the kernel only matches opaque
# capability name strings between consumer requirements and provider
# declarations.

add_library(engine_kernel SHARED
    source/component_context.cpp
    source/configuration.cpp
    source/dependency_resolver.cpp
    source/dynamic_library.cpp
    source/engine_kernel.cpp
    source/filesystem_utilities.cpp
    source/mod_discovery.cpp
    source/mod_manifest.cpp
)

# The kernel is the one shared library every module plugin binds to: one
# process, one kernel instance. The DLL lives next to
# the engine executable (the first entry of the Windows DLL search order, so
# plugins resolve it without any path configuration); on POSIX the plugins
# reach it through their $ORIGIN/.. RPATH.
set_target_properties(engine_kernel PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime"
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

target_include_directories(engine_kernel
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(engine_kernel
    PRIVATE engine_tomlplusplus_bindings
    PRIVATE Threads::Threads
)

if(MSVC)
    target_compile_options(engine_kernel PUBLIC /W4 /permissive-)
else()
    target_compile_options(engine_kernel PUBLIC -Wall -Wextra -Wpedantic)
endif()

# The kernel links no platform UI libraries; it is loader, TOML, filesystem and
# event plumbing only.
