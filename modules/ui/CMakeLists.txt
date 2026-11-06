include(engine_fetch_dependency)

option(
    ENGINE_UI_ENABLE_IMGUI
    "Build the UI module with Dear ImGui core"
    ON
)

if(ENGINE_UI_ENABLE_IMGUI)
    engine_fetch_dependency(
        imgui
        OWNER
        ui
        GIT_REPOSITORY
        https://github.com/ocornut/imgui.git
        GIT_TAG
        v1.91.9b
    )

    add_library(
        engine_ui_imgui
        STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    )
    target_include_directories(
        engine_ui_imgui
        PUBLIC "${imgui_SOURCE_DIR}"
    )
    set_target_properties(
        engine_ui_imgui
        PROPERTIES POSITION_INDEPENDENT_CODE
                   ON
    )
endif()

add_library(
    engine_ui_capabilities
    INTERFACE
)
target_include_directories(
    engine_ui_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_ui
    MODULE
    source/ui.cpp
)

target_compile_definitions(
    engine_module_ui
    PRIVATE ENGINE_UI_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_ui
    PRIVATE engine_kernel
            engine_ui_capabilities
            Threads::Threads
)

if(ENGINE_UI_ENABLE_IMGUI)
    target_compile_definitions(
        engine_module_ui
        PRIVATE ENGINE_UI_BACKEND_IMGUI=1
    )
    target_link_libraries(
        engine_module_ui
        PRIVATE engine_ui_imgui
    )
endif()

set_target_properties(
    engine_module_ui
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/ui"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/ui"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/ui"
)

add_custom_command(
    TARGET engine_module_ui
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_ui>/module.toml"
    COMMENT "placing the UI module descriptor next to its library"
)
