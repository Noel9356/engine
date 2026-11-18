add_library(
    engine_physics_fields_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_fields_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_fields_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_fields
    MODULE
    source/physics_fields.cpp
)
target_compile_definitions(
    engine_module_physics_fields
    PRIVATE ENGINE_PHYSICS_FIELDS_EMBEDDED=1
            ENGINE_PHYSICS_FIELDS_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_fields
    PRIVATE engine_kernel
            engine_physics_fields_capabilities
            engine_physics_rigid_capabilities
)
set_target_properties(
    engine_module_physics_fields
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fields"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fields"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fields"
)
add_custom_command(
    TARGET engine_module_physics_fields
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_fields>/module.toml"
    COMMENT "placing the physics_fields module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_fields_capability
    DEPENDS engine_module_physics_fields
)
