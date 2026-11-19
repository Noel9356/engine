add_library(
    engine_physics_fluid_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_fluid_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_fluid_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_fluid
    MODULE
    source/physics_fluid.cpp
)
target_compile_definitions(
    engine_module_physics_fluid
    PRIVATE ENGINE_PHYSICS_FLUID_EMBEDDED=1
            ENGINE_PHYSICS_FLUID_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_fluid
    PRIVATE engine_kernel
            engine_physics_fluid_capabilities
            engine_physics_rigid_capabilities
            engine_util_types
)
set_target_properties(
    engine_module_physics_fluid
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fluid"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fluid"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_fluid"
)
add_custom_command(
    TARGET engine_module_physics_fluid
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_fluid>/module.toml"
    COMMENT "placing the physics_fluid module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_fluid_capability
    DEPENDS engine_module_physics_fluid
)
