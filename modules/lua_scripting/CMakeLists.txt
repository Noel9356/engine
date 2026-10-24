# The module's own build switch, declared here so everything about the module
# lives in its directory: the root build discovers this CMakeLists and the
# option arrives with it. OFF keeps every target defined - consumers still
# compile against the published capability header below, and an in-tree
# consumer that links the implementation still pulls it - but removes the
# plugin binary from the default build; a run whose mods then demand a
# capability only this module provides fails startup with a precise
# diagnostic naming the consumer, never a link error.
option(ENGINE_BUILD_MODULE_LUA_SCRIPTING "Include the lua_scripting module in the build" ON)

add_library(engine_module_lua_scripting SHARED
    module_plugin.cpp
    lua_scripting.cpp
)

target_link_libraries(engine_module_lua_scripting
    PUBLIC engine_kernel
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_window
    PUBLIC engine_capability_input
    PUBLIC engine_capability_renderer
    PUBLIC engine_capability_screenshot
    PUBLIC engine_capability_scene
    PUBLIC engine_capability_pbr
    PUBLIC engine_capability_animation
    PUBLIC engine_capability_asset
    PUBLIC engine_capability_physics
    PUBLIC engine_capability_audio
    PUBLIC engine_capability_ui
    PUBLIC engine_capability_persistence
    PUBLIC engine_capability_hot_reload
    PRIVATE engine_sol2_bindings
)


# Module plugin deployment: the binary lands in runtime/modules/, the directory
# the kernel scans for plugins (see engine_kernel.cpp loadModulePlugins).
# PREFIX keeps one naming scheme on every platform (libengine_module_lua_scripting.dll
# on Windows, libengine_module_lua_scripting.so on POSIX); the $ORIGIN entries resolve
# the sibling plugins and $ORIGIN/.. the kernel and GLFW runtimes in runtime/;
# WINDOWS_EXPORT_ALL_SYMBOLS publishes the module's C++ surface to the other
# plugins that consume it across the boundary.
set_target_properties(engine_module_lua_scripting PROPERTIES
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
    target_link_options(engine_module_lua_scripting PRIVATE -Wl,--export-all-symbols)
endif()


# This module provides the "scripting.lua" capability but publishes no C++
# capability header: its consumer surface is the Lua bridge installed into
# every script state (mods/README.md), and the kernel reaches the hosting
# surface through the kernel-owned ScriptModHost interface. Nothing here
# needs a header-only include target.

# Excluded from the default build when the option is OFF: the target stays
# defined for consumers, but nothing builds the plugin binary unless an
# in-tree consumer actually links it (the runtime then simply does not see
# the plugin, and a demand for its capability fails startup precisely).
if(NOT ENGINE_BUILD_MODULE_LUA_SCRIPTING)
    set_target_properties(engine_module_lua_scripting PROPERTIES EXCLUDE_FROM_ALL ON)
endif()
