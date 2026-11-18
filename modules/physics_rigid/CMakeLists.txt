add_library(
    engine_physics_rigid_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_rigid_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_rigid_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_rigid
    MODULE
    source/broadphase.cpp
    source/physics_rigid.cpp
    source/shapes.cpp
    source/solver.cpp
    source/world.cpp
)
target_compile_definitions(
    engine_module_physics_rigid
    PRIVATE ENGINE_PHYSICS_RIGID_EMBEDDED=1
            ENGINE_PHYSICS_RIGID_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_rigid
    PRIVATE engine_kernel
            engine_physics_rigid_capabilities
)
set_target_properties(
    engine_module_physics_rigid
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rigid"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rigid"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_rigid"
)
add_custom_command(
    TARGET engine_module_physics_rigid
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_rigid>/module.toml"
    COMMENT "placing the physics_rigid module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_rigid_capability
    DEPENDS engine_module_physics_rigid
)
