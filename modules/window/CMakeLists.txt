add_library(engine_module_window SHARED
    module_plugin.cpp
    window.cpp
)

target_compile_definitions(engine_module_window PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(engine_module_window
    PUBLIC engine_kernel
    PUBLIC engine_capability_window
    PUBLIC engine_capability_logging
    PRIVATE glfw
    # The window module creates Vulkan presentation surfaces (glfwCreateWindowSurface):
    # it needs the Vulkan C headers. Headers only, PRIVATE - the capability contract
    # stays implementation agnostic and carries no Vulkan types.
    PRIVATE Vulkan::Headers
)



# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_window.dll
# on Windows, libengine_module_window.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_window PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)


# The published capability surface of this module: a header-only INTERFACE
# target. Consumers link it to include window_capability.hpp WITHOUT linking this module's
# implementation - the capability graph stays explicit in the target links,
# and no consumer binary depends on another module's symbols at load time.
add_library(engine_capability_window INTERFACE)
target_include_directories(engine_capability_window INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# The portability guard: the published header must stay implementation-,
# platform- and vendor-agnostic (see cmake/capability_header_guard.cmake).
engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/window_capability.hpp")
