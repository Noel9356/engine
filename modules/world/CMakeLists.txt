add_library(
    engine_world_capabilities
    INTERFACE
)
target_include_directories(
    engine_world_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_world_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_ecs_capabilities
    INTERFACE
)
target_link_libraries(
    engine_ecs_capabilities
    INTERFACE engine_world_capabilities
)
add_library(
    engine_entity_world_capabilities
    INTERFACE
)
target_link_libraries(
    engine_entity_world_capabilities
    INTERFACE engine_world_capabilities
)

add_library(
    engine_module_world
    MODULE
    source/world.cpp
    source/ecs.cpp
    source/ecs_storage.cpp
    source/entity_world.cpp
)
target_compile_definitions(
    engine_module_world
    PRIVATE ENGINE_WORLD_EMBEDDED=1
            ENGINE_ECS_MODULE_VERSION="0.1.0"
            ENGINE_ENTITY_WORLD_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_world
    PRIVATE engine_kernel
            engine_world_capabilities
            engine_ecs_capabilities
            engine_entity_world_capabilities
)
set_target_properties(
    engine_module_world
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/world"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/world"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/world"
)
add_custom_command(
    TARGET engine_module_world
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_world>/module.toml"
    COMMENT "placing the world module descriptor next to its library"
)

add_custom_target(
    engine_module_ecs
    DEPENDS engine_module_world
)
add_custom_target(
    engine_module_entity_world
    DEPENDS engine_module_world
)
