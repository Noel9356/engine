add_library(
    engine_physics_character_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_character_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_character_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_character
    MODULE
    source/physics_character.cpp
)
target_compile_definitions(
    engine_module_physics_character
    PRIVATE ENGINE_PHYSICS_CHARACTER_EMBEDDED=1
            ENGINE_PHYSICS_CHARACTER_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_character
    PRIVATE engine_kernel
            engine_physics_character_capabilities
            engine_physics_rigid_capabilities
            engine_physics_articulated_capabilities
)
set_target_properties(
    engine_module_physics_character
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_character"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_character"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_character"
)
add_custom_command(
    TARGET engine_module_physics_character
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_character>/module.toml"
    COMMENT "placing the physics_character module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_character_capability
    DEPENDS engine_module_physics_character
)
