add_library(
    engine_physics_fur_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_fur_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_physics_fur
    MODULE
    source/physics_fur.cpp
    source/strand_texture.cpp
    source/lod_bands.cpp
)

target_compile_definitions(
    engine_module_physics_fur
    PRIVATE ENGINE_PHYSICS_FUR_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_physics_fur
    PRIVATE engine_kernel
            engine_physics_fur_capabilities
)

set_target_properties(
    engine_module_physics_fur
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fur"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fur"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fur"
)

add_custom_command(
    TARGET engine_module_physics_fur
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_fur>/module.toml"
    COMMENT "placing the physics_fur module descriptor next to its library"
)
