add_library(engine_physics_scheduling_capabilities INTERFACE)
target_include_directories(engine_physics_scheduling_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(engine_physics_scheduling_capabilities INTERFACE cxx_std_20)

add_library(engine_module_physics_scheduling MODULE source/physics_scheduling.cpp)
target_compile_definitions(engine_module_physics_scheduling
    PRIVATE ENGINE_PHYSICS_SCHEDULING_EMBEDDED=1
            ENGINE_PHYSICS_SCHEDULING_MODULE_VERSION="0.1.0")
target_link_libraries(engine_module_physics_scheduling
    PRIVATE engine_util_types
            engine_kernel
            engine_physics_scheduling_capabilities)
set_target_properties(engine_module_physics_scheduling PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules/physics_scheduling"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules/physics_scheduling"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules/physics_scheduling")
add_custom_command(TARGET engine_module_physics_scheduling POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_physics_scheduling>/module.toml"
    COMMENT "placing the physics_scheduling module descriptor next to its library")
add_custom_target(engine_module_physics_scheduling_capability
    DEPENDS engine_module_physics_scheduling)
