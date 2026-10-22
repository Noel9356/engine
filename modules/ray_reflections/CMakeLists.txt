add_library(engine_module_ray_reflections STATIC
    ray_reflections.cpp
)

target_link_libraries(engine_module_ray_reflections
    PUBLIC engine_kernel
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
