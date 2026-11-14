add_library(
    engine_fur_capabilities
    INTERFACE
)
target_include_directories(
    engine_fur_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_fur
    MODULE
    source/fur.cpp
    source/fur_texture.cpp
    source/fur_lod.cpp
)

target_compile_definitions(
    engine_module_fur
    PRIVATE ENGINE_FUR_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_fur
    PRIVATE engine_kernel
            engine_fur_capabilities
)

set_target_properties(
    engine_module_fur
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/fur"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/fur"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/fur"
)

add_custom_command(
    TARGET engine_module_fur
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_fur>/module.toml"
    COMMENT "placing the fur module descriptor next to its library"
)
