add_library(engine_module_radiance_cascades SHARED
    module_plugin.cpp
    radiance_cascades.cpp
)

target_link_libraries(engine_module_radiance_cascades
    PUBLIC engine_kernel
    PRIVATE engine_module_renderer
    PRIVATE engine_vulkan_hpp_bindings
)

target_include_directories(engine_module_radiance_cascades PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(ENGINE_BUILD_DEMO)
    include("${CMAKE_SOURCE_DIR}/cmake/shader_compilation.cmake")
    engine_compile_module_shaders(TARGET engine_module_radiance_cascades
                                  SOURCES shaders/rc_march.comp
                                          shaders/rc_compose.comp
                                          shaders/rc_gather.comp
                                          shaders/rc_denoise.comp
                                  INCLUDE_DIRS shaders/common
                                            "${CMAKE_SOURCE_DIR}/modules/renderer/shaders/common")
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
