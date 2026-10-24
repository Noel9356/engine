# The module's own build switch, declared here so everything about the module
# lives in its directory: the root build discovers this CMakeLists and the
# option arrives with it. OFF keeps every target defined - consumers still
# compile against the published capability header below, and an in-tree
# consumer that links the implementation still pulls it - but removes the
# plugin binary from the default build; a run whose mods then demand a
# capability only this module provides fails startup with a precise
# diagnostic naming the consumer, never a link error.
option(ENGINE_BUILD_MODULE_WINDOW_GLFW "Include the window_glfw module in the build" ON)
# The published capability surface of this module: a header-only INTERFACE
# target. Consumers link it to include window_capability.hpp WITHOUT linking this module's
# implementation - the capability graph stays explicit in the target links,
# and no consumer binary depends on another module's symbols at load time.
add_library(engine_capability_window INTERFACE)
target_include_directories(engine_capability_window INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# The portability guard: the published header must stay implementation-,
# platform- and vendor-agnostic (see cmake/capability_header_guard.cmake).
engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/window_capability.hpp")

add_library(engine_module_window_glfw SHARED
    module_plugin.cpp
    window.cpp
)

target_compile_definitions(engine_module_window_glfw PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(engine_module_window_glfw
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
# PREFIX keeps one naming scheme on every platform (libengine_module_window_glfw.dll
# on Windows, libengine_module_window_glfw.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_window_glfw PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
# MinGW: one explicit __declspec(dllexport) (the plugin entry point) disables
# the linker's all-symbol auto-export, and WINDOWS_EXPORT_ALL_SYMBOLS above is
# implemented only for MS-compatible toolchains; this flag publishes the
# module's C++ surface to the other plugins' import libraries (the lighting
# techniques reach RendererModule::attachLightingTechnique directly).
if(MINGW)
    target_link_options(engine_module_window_glfw PRIVATE -Wl,--export-all-symbols)
endif()

# Excluded from the default build when the option is OFF: the target stays
# defined for consumers, but nothing builds the plugin binary unless an
# in-tree consumer actually links it (the runtime then simply does not see
# the plugin, and a demand for its capability fails startup precisely).
if(NOT ENGINE_BUILD_MODULE_WINDOW_GLFW)
    set_target_properties(engine_module_window_glfw PROPERTIES EXCLUDE_FROM_ALL ON)
endif()
