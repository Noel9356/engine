add_library(engine_module_input SHARED
    module_plugin.cpp
    input.cpp
)

target_compile_definitions(engine_module_input PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(engine_module_input
    PUBLIC engine_kernel
    PRIVATE glfw
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_input PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_input.dll
# on Windows, libengine_module_input.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_input PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
