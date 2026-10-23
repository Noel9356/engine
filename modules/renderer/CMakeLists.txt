# One state struct (renderer_state.hpp, module-private), several translation
# units split by concern: device/swapchain creation, scene resources, pipelines,
# frame recording, and the module lifecycle.
add_library(engine_module_renderer SHARED
    module_plugin.cpp
    renderer.cpp
    renderer_device.cpp
    renderer_scene.cpp
    renderer_pipelines.cpp
    renderer_frames.cpp
    scene_description.cpp
    mesh_generation.cpp
)

target_link_libraries(engine_module_renderer
    PUBLIC engine_kernel
    PRIVATE engine_vulkan_hpp_bindings
    PRIVATE engine_stb_bindings
)

# The host composition root includes the module header directly; lighting
# modules include the pass interface the same way.
target_include_directories(engine_module_renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# The renderer owns the scene rasterization and composite shaders. The shared
# SDF/PBR include directory is consumed by the lighting modules through
# INCLUDE_DIRS, so one geometry truth backs rasterization and tracing.
if(ENGINE_BUILD_DEMO)
    include("${CMAKE_SOURCE_DIR}/cmake/shader_compilation.cmake")
    engine_compile_module_shaders(TARGET engine_module_renderer
                                  SOURCES shaders/scene.vert
                                          shaders/scene.frag
                                          shaders/scene_composite.vert
                                          shaders/scene_composite.frag
                                  INCLUDE_DIRS shaders/common)
endif()


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_renderer.dll
# on Windows, libengine_module_renderer.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_renderer PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
