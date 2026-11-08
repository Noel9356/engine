add_library(
    engine_direct_lighting_capabilities
    INTERFACE
)
target_include_directories(
    engine_direct_lighting_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_direct_lighting_capabilities
    INTERFACE engine_renderer_types
)
add_library(
    engine_module_direct_lighting
    MODULE
    source/direct_lighting.cpp
)
target_link_libraries(
    engine_module_direct_lighting
    PRIVATE engine_kernel
            engine_direct_lighting_capabilities
)
set_target_properties(
    engine_module_direct_lighting
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/direct_lighting"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/direct_lighting"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/direct_lighting"
)
add_custom_command(
    TARGET engine_module_direct_lighting
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_direct_lighting>/module.toml"
    COMMENT "placing the direct_lighting module descriptor next to its library"
)
