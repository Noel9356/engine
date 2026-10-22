add_library(engine_module_radiance_cascades STATIC
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
