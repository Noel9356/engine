add_library(
    engine_physics_cloth_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_cloth_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_cloth_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_cloth
    MODULE
    source/physics_cloth.cpp
)
target_compile_definitions(
    engine_module_physics_cloth
    PRIVATE ENGINE_PHYSICS_CLOTH_EMBEDDED=1
            ENGINE_PHYSICS_CLOTH_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_cloth
    PRIVATE engine_kernel
            engine_physics_cloth_capabilities
            engine_physics_rigid_capabilities
)
set_target_properties(
    engine_module_physics_cloth
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_cloth"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_cloth"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_cloth"
)
add_custom_command(
    TARGET engine_module_physics_cloth
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_cloth>/module.toml"
    COMMENT "placing the physics_cloth module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_cloth_capability
    DEPENDS engine_module_physics_cloth
)
