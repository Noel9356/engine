add_library(engine_module_radiance_cascades SHARED
    module_plugin.cpp
    radiance_cascades.cpp
)

target_link_libraries(engine_module_radiance_cascades
    PUBLIC engine_kernel
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_renderer
    PUBLIC engine_capability_pbr
    PRIVATE engine_module_renderer_vulkan
    PRIVATE engine_vulkan_hpp_bindings
)

if(ENGINE_BUILD_DEMO)
    include("${CMAKE_SOURCE_DIR}/cmake/shader_compilation.cmake")
    engine_compile_module_shaders(TARGET engine_module_radiance_cascades
                                  SOURCES shaders/rc_march.comp
                                          shaders/rc_compose.comp
                                          shaders/rc_gather.comp
                                          shaders/rc_denoise.comp
                                  INCLUDE_DIRS shaders/common
                                            "${CMAKE_SOURCE_DIR}/modules/renderer_vulkan/shaders/common")
endif()


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_radiance_cascades.dll
# on Windows, libengine_module_radiance_cascades.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_radiance_cascades PROPERTIES
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
    target_link_options(engine_module_radiance_cascades PRIVATE -Wl,--export-all-symbols)
endif()


# This module provides a demand marker capability ("indirect_diffuse") and no
# call interface of its own: its implementation surface is the lighting
# technique pass it attaches to the renderer (modules/renderer_vulkan/
# render_interface.hpp, an in-tree integration that is deliberately not a
# capability). Consumers demand the marker; nothing includes a header here,
# so no header-only include target exists.
