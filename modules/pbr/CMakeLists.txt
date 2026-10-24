add_library(engine_module_pbr SHARED
    module_plugin.cpp
    pbr.cpp
)

target_link_libraries(engine_module_pbr
    PUBLIC engine_kernel
    PUBLIC engine_capability_pbr
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_renderer
    PRIVATE engine_module_renderer_vulkan
    PRIVATE engine_vulkan_hpp_bindings
)

if(ENGINE_BUILD_DEMO)
    include("${CMAKE_SOURCE_DIR}/cmake/shader_compilation.cmake")
    engine_compile_module_shaders(TARGET engine_module_pbr
                                  SOURCES shaders/pbr_lighting.comp
                                  INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/modules/renderer_vulkan/shaders/common")
endif()


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_pbr.dll
# on Windows, libengine_module_pbr.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_pbr PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
# MinGW: one explicit __declspec(dllexport) (the plugin entry point) disables
# the linker's all-symbol auto-export, and WINDOWS_EXPORT_ALL_SYMBOLS above is
# implemented only for MS-compatible toolchains; this flag publishes the
# module's C++ surface to the other plugins' import libraries (the lighting
# techniques reach RendererModule::attachLightingTechnique directly).
if(MINGW)
    target_link_options(engine_module_pbr PRIVATE -Wl,--export-all-symbols)
endif()


# The published capability surface of this module: a header-only INTERFACE
# target. Consumers link it to include lighting_capability.hpp WITHOUT linking this module's
# implementation - the capability graph stays explicit in the target links,
# and no consumer binary depends on another module's symbols at load time.
add_library(engine_capability_pbr INTERFACE)
target_include_directories(engine_capability_pbr INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# The portability guard: the published header must stay implementation-,
# platform- and vendor-agnostic (see cmake/capability_header_guard.cmake).
engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/lighting_capability.hpp")
