add_library(
    engine_physics_rope_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_rope_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_rope_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_rope
    MODULE
    source/physics_rope.cpp
)
target_compile_definitions(
    engine_module_physics_rope
    PRIVATE ENGINE_PHYSICS_ROPE_EMBEDDED=1
            ENGINE_PHYSICS_ROPE_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_rope
    PRIVATE engine_kernel
            engine_physics_rope_capabilities
            engine_physics_rigid_capabilities
)
set_target_properties(
    engine_module_physics_rope
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rope"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rope"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rope"
)
add_custom_command(
    TARGET engine_module_physics_rope
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_rope>/module.toml"
    COMMENT "placing the physics_rope module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_rope_capability
    DEPENDS engine_module_physics_rope
)
