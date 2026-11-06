add_library(
    engine_pbr_capabilities
    INTERFACE
)
target_include_directories(
    engine_pbr_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(
    engine_pbr_capabilities
    INTERFACE engine_model_types
              engine_render_types
)

add_library(
    engine_module_pbr
    MODULE
    source/pbr.cpp
)

target_compile_definitions(
    engine_module_pbr
    PRIVATE ENGINE_PBR_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_pbr
    PRIVATE engine_kernel
            engine_pbr_capabilities
            engine_model_types
            engine_render_types
)

set_target_properties(
    engine_module_pbr
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/pbr"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/pbr"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/pbr"
)

add_custom_command(
    TARGET engine_module_pbr
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_pbr>/module.toml"
    COMMENT "placing the pbr module descriptor next to its library"
)
