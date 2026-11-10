include(engine_fetch_dependency)

add_library(
    engine_window_capabilities
    INTERFACE
)
target_include_directories(
    engine_window_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

option(
    ENGINE_WINDOW_ENABLE_GLFW
    "Build the window module with the GLFW desktop backend"
    ON
)

if(ANDROID)

    set(ENGINE_WINDOW_ENABLE_GLFW
        OFF
    )
endif()

if(ENGINE_WINDOW_ENABLE_GLFW)

    set(GLFW_BUILD_DOCS
        OFF
        CACHE BOOL
              ""
              FORCE
    )
    set(GLFW_BUILD_TESTS
        OFF
        CACHE BOOL
              ""
              FORCE
    )
    set(GLFW_BUILD_EXAMPLES
        OFF
        CACHE BOOL
              ""
              FORCE
    )
    set(GLFW_INSTALL
        OFF
        CACHE BOOL
              ""
              FORCE
    )

    option(
        ENGINE_GLFW_ENABLE_WAYLAND
        "Build GLFW with the Wayland backend"
        OFF
    )
    set(GLFW_BUILD_WAYLAND
        "${ENGINE_GLFW_ENABLE_WAYLAND}"
        CACHE BOOL
              ""
              FORCE
    )

    engine_fetch_dependency(
        glfw
        OWNER
        window
        GIT_REPOSITORY
        https://github.com/glfw/glfw.git
        GIT_TAG
        3.4
        MAKE_AVAILABLE
    )

    set_target_properties(
        glfw
        PROPERTIES POSITION_INDEPENDENT_CODE
                   ON
    )
endif()

add_library(
    engine_module_window
    MODULE
    source/window.cpp
    source/glfw_backend.cpp
    source/headless_window_backend.cpp
)

target_compile_definitions(
    engine_module_window
    PRIVATE ENGINE_WINDOW_MODULE_VERSION="0.1.0"
)

option(
    ENGINE_WINDOW_HEADLESS_VULKAN
    "Headless window backend reports a Vulkan surface (VK_EXT_headless_surface)"
    OFF
)
if(ENGINE_WINDOW_HEADLESS_VULKAN)
    target_compile_definitions(
        engine_module_window
        PRIVATE ENGINE_WINDOW_HEADLESS_VULKAN=1
    )
endif()

target_link_libraries(
    engine_module_window
    PRIVATE engine_kernel
            engine_window_capabilities
            engine_util_types
)

if(ENGINE_WINDOW_ENABLE_GLFW
   AND UNIX
   AND NOT
       APPLE
)
    find_package(
        X11
        REQUIRED
    )
    target_include_directories(
        engine_module_window
        PRIVATE ${X11_INCLUDE_DIR}
    )
endif()

target_include_directories(
    engine_module_window
    PRIVATE ${CMAKE_INCLUDE_PATH}
)

if(ENGINE_WINDOW_ENABLE_GLFW)
    target_compile_definitions(
        engine_module_window
        PRIVATE ENGINE_WINDOW_BACKEND_GLFW=1
    )
    target_link_libraries(
        engine_module_window
        PRIVATE glfw
    )
endif()

set_target_properties(
    engine_module_window
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/window"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/window"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/window"
)

add_custom_command(
    TARGET engine_module_window
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_window>/module.toml"
    COMMENT "placing the window module descriptor next to its library"
)
