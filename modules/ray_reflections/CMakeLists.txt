add_library(engine_module_ray_reflections SHARED
    module_plugin.cpp
    ray_reflections.cpp
)

target_link_libraries(engine_module_ray_reflections
    PUBLIC engine_kernel
    PUBLIC engine_contracts
    PRIVATE engine_module_renderer
    PRIVATE engine_vulkan_hpp_bindings
)

# The module attaches its pass through the renderer's in-tree interface.
target_include_directories(engine_module_ray_reflections PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(ENGINE_BUILD_DEMO)
    include("${CMAKE_SOURCE_DIR}/cmake/shader_compilation.cmake")
    engine_compile_module_shaders(TARGET engine_module_ray_reflections
                                  SOURCES shaders/ray_reflections.comp
                                  INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/modules/renderer/shaders/common")
endif()


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_ray_reflections.dll
# on Windows, libengine_module_ray_reflections.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_ray_reflections PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
