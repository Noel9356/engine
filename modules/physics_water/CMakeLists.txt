add_library(
    engine_physics_water_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_water_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_water_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_water
    MODULE
    source/physics_water.cpp
)
target_compile_definitions(
    engine_module_physics_water
    PRIVATE ENGINE_PHYSICS_WATER_EMBEDDED=1
            ENGINE_PHYSICS_WATER_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_water
    PRIVATE engine_kernel
            engine_physics_water_capabilities
            engine_physics_rigid_capabilities
            engine_util_types
)
set_target_properties(
    engine_module_physics_water
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_water"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_water"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_water"
)
add_custom_command(
    TARGET engine_module_physics_water
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_water>/module.toml"
    COMMENT "placing the physics_water module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_water_capability
    DEPENDS engine_module_physics_water
)
