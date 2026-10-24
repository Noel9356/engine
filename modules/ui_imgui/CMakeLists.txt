add_library(engine_module_ui_imgui SHARED
    module_plugin.cpp
    ui.cpp
    imgui_backend.cpp
)

target_link_libraries(engine_module_ui_imgui
    PUBLIC engine_kernel
    PUBLIC engine_capability_ui
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_window
    PUBLIC engine_capability_input
    PUBLIC engine_capability_renderer
    PUBLIC engine_capability_scene
    PRIVATE engine_imgui_bindings
    PRIVATE engine_capability_persistence
)

# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
set_target_properties(engine_module_ui_imgui PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_ui_imgui PRIVATE -Wl,--export-all-symbols)
endif()

# The published capability surface of this module: a header-only INTERFACE
# target. Consumers link it to include ui_capability.hpp WITHOUT linking this
# module's implementation.
add_library(engine_capability_ui INTERFACE)
target_include_directories(engine_capability_ui INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# The portability guard: the published header must stay implementation-,
# platform- and vendor-agnostic (see cmake/capability_header_guard.cmake).
engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/ui_capability.hpp")
