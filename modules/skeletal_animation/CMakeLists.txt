add_library(
    engine_skeletal_animation_capabilities
    INTERFACE
)
target_include_directories(
    engine_skeletal_animation_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_skeletal_animation
    MODULE
    source/skeletal_animation.cpp
    source/sampler.cpp
    source/event_collector.cpp
    source/blending.cpp
    source/lua_bridge.cpp
)
target_compile_definitions(
    engine_module_skeletal_animation
    PRIVATE ENGINE_SKELETAL_ANIMATION_MODULE_VERSION="0.1.0"
)
target_include_directories(
    engine_module_skeletal_animation
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../script_runtime/include"
)

target_link_libraries(
    engine_module_skeletal_animation
    PRIVATE engine_kernel
            engine_skeletal_animation_capabilities
            engine_util_types
)

set_target_properties(
    engine_module_skeletal_animation
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/skeletal_animation"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/skeletal_animation"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/skeletal_animation"
)

add_custom_command(
    TARGET engine_module_skeletal_animation
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_skeletal_animation>/module.toml"
    COMMENT "placing the skeletal_animation module descriptor next to its library"
)
