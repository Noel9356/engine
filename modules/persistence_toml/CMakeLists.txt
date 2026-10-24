add_library(engine_module_persistence_toml SHARED
    module_plugin.cpp
    persistence.cpp
)

target_link_libraries(engine_module_persistence_toml
    PUBLIC engine_kernel
    PUBLIC engine_tomlplusplus_bindings
    PUBLIC engine_capability_persistence
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_scene
)

# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
set_target_properties(engine_module_persistence_toml PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_persistence_toml PRIVATE -Wl,--export-all-symbols)
endif()

# The published capability surface of this module: a header-only INTERFACE
# target. Consumers link it to include persistence_capability.hpp WITHOUT
# linking this module's implementation.
add_library(engine_capability_persistence INTERFACE)
target_include_directories(engine_capability_persistence INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# The portability guard: the published header must stay implementation-,
# platform- and vendor-agnostic (see cmake/capability_header_guard.cmake).
engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/persistence_capability.hpp")
