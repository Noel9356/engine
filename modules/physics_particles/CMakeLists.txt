add_library(
    engine_physics_particles_capabilities
    INTERFACE
)
target_include_directories(
    engine_physics_particles_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_physics_particles_capabilities
    INTERFACE cxx_std_20
)

add_library(
    engine_module_physics_particles
    MODULE
    source/physics_particles.cpp
)
target_compile_definitions(
    engine_module_physics_particles
    PRIVATE ENGINE_PHYSICS_PARTICLES_EMBEDDED=1
            ENGINE_PHYSICS_PARTICLES_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_physics_particles
    PRIVATE engine_kernel
            engine_physics_particles_capabilities
            engine_physics_rigid_capabilities
)
set_target_properties(
    engine_module_physics_particles
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_particles"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_particles"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/physics_particles"
)
add_custom_command(
    TARGET engine_module_physics_particles
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_particles>/module.toml"
    COMMENT "placing the physics_particles module descriptor next to its library"
)

add_custom_target(
    engine_module_physics_particles_capability
    DEPENDS engine_module_physics_particles
)
