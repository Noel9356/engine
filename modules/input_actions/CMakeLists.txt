add_library(
    engine_input_actions_capabilities
    INTERFACE
)
target_include_directories(
    engine_input_actions_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(
    engine_input_actions_capabilities
    INTERFACE engine_window_capabilities
)

add_library(
    engine_module_input_actions
    MODULE
    source/input_actions.cpp
    source/device_state.cpp
    source/context_registry.cpp
    source/window_adapter.cpp
    source/frame_evaluator.cpp
    source/lua_bridge.cpp
)
target_compile_definitions(
    engine_module_input_actions
    PRIVATE ENGINE_INPUT_ACTIONS_MODULE_VERSION="0.1.0"
)
target_include_directories(
    engine_module_input_actions
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../script_runtime/include"
)

target_link_libraries(
    engine_module_input_actions
    PRIVATE engine_kernel
            engine_input_actions_capabilities
            engine_window_capabilities
)

set_target_properties(
    engine_module_input_actions
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/input_actions"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/input_actions"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/input_actions"
)

add_custom_command(
    TARGET engine_module_input_actions
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_input_actions>/module.toml"
    COMMENT "placing the input_actions module descriptor next to its library"
)
