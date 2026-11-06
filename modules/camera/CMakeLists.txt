include(engine_fetch_dependency)

engine_fetch_dependency(
    glm
    OWNER
    camera
    GIT_REPOSITORY
    https://github.com/g-truc/glm.git
    GIT_TAG
    1.0.1
    HEADER_ONLY
)

add_library(
    engine_camera_capabilities
    INTERFACE
)
target_include_directories(
    engine_camera_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_camera_capabilities
    INTERFACE engine_dep_glm
)

add_library(
    engine_module_camera
    MODULE
    source/camera.cpp
)

target_compile_definitions(
    engine_module_camera
    PRIVATE ENGINE_CAMERA_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_camera
    PRIVATE engine_kernel
            engine_camera_capabilities
            engine_dep_glm
)

set_target_properties(
    engine_module_camera
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/camera"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/camera"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/camera"
)

add_custom_command(
    TARGET engine_module_camera
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_camera>/module.toml"
    COMMENT "placing the camera module descriptor next to its library"
)
