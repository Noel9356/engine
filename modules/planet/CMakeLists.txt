add_library(
    engine_planet_capabilities
    INTERFACE
)
target_include_directories(
    engine_planet_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# Core object library: the pure generation code shared between the runtime
# module (.so loaded by the kernel) and host applications that build chunks
# directly (the demo's determinism self-test). OBJECT libraries may be linked
# into MODULE targets, unlike MODULE targets themselves.
add_library(
    engine_planet_core
    OBJECT
    source/noise.cpp
    source/params.cpp
    source/climate.cpp
    source/density.cpp
    source/meshing.cpp
    source/octree.cpp
    source/planet.cpp
)
target_link_libraries(
    engine_planet_core
    PRIVATE engine_kernel
            engine_planet_capabilities
)
set_target_properties(
    engine_planet_core
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)

add_library(
    engine_module_planet
    MODULE
    $<TARGET_OBJECTS:engine_planet_core>
    source/planet_module.cpp
)
target_link_libraries(
    engine_module_planet
    PRIVATE engine_kernel
            engine_planet_capabilities
)
set_target_properties(
    engine_module_planet
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/planet"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/planet"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/planet"
)
add_custom_command(
    TARGET engine_module_planet
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_planet>/module.toml"
    COMMENT "placing the planet module descriptor next to its library"
)
