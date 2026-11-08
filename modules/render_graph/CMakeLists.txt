add_library(
    engine_render_graph_capabilities
    INTERFACE
)
target_include_directories(
    engine_render_graph_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(
    engine_render_graph_capabilities
    INTERFACE engine_renderer_capabilities
)

add_library(
    engine_render_graph_plan
    STATIC
    source/render_graph_compiler.cpp
)
target_link_libraries(
    engine_render_graph_plan
    PUBLIC engine_render_graph_capabilities
)

add_library(
    engine_module_render_graph
    MODULE
    source/render_graph.cpp
)
target_link_libraries(
    engine_module_render_graph
    PRIVATE engine_kernel
            engine_render_graph_capabilities
            engine_render_graph_plan
)
set_target_properties(
    engine_module_render_graph
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render_graph"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render_graph"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render_graph"
)
add_custom_command(
    TARGET engine_module_render_graph
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_render_graph>/module.toml"
    COMMENT "placing the render_graph module descriptor next to its library"
)
