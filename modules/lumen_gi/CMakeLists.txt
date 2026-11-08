add_library(
    engine_lumen_gi_capabilities
    INTERFACE
)
target_include_directories(
    engine_lumen_gi_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_lumen_gi_capabilities
    INTERFACE engine_renderer_types
              engine_dep_glm
)
add_library(
    engine_module_lumen_gi
    MODULE
    source/lumen_gi.cpp
    source/lumen_gi_scene.cpp
    source/lumen_gi_lighting.cpp
    source/lumen_gi_debug.cpp
    source/lumen_gi_debug_views.cpp
)
target_link_libraries(
    engine_module_lumen_gi
    PRIVATE engine_kernel
            engine_lumen_gi_capabilities
            engine_pbr_capabilities
            engine_direct_lighting_capabilities
            engine_dep_glm
)
set_target_properties(
    engine_module_lumen_gi
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_gi"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_gi"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_gi"
)
add_custom_command(
    TARGET engine_module_lumen_gi
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_lumen_gi>/module.toml"
    COMMENT "placing the lumen_gi module descriptor next to its library"
)
