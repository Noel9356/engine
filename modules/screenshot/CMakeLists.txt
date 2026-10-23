add_library(engine_module_screenshot SHARED
    module_plugin.cpp
    screenshot.cpp
)

target_compile_definitions(engine_module_screenshot PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)

target_link_libraries(engine_module_screenshot
    PUBLIC engine_kernel
    PUBLIC engine_contracts
    PRIVATE engine_stb_bindings
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_screenshot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_screenshot.dll
# on Windows, libengine_module_screenshot.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_screenshot PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
