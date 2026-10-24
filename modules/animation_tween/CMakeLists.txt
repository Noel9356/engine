# The module's own build switch, declared here so everything about the module
# lives in its directory: the root build discovers this CMakeLists and the
# option arrives with it. OFF keeps every target defined - consumers still
# compile against the published capability header below, and an in-tree
# consumer that links the implementation still pulls it - but removes the
# plugin binary from the default build; a run whose mods then demand a
# capability only this module provides fails startup with a precise
# diagnostic naming the consumer, never a link error.
option(ENGINE_BUILD_MODULE_ANIMATION_TWEEN "Include the animation_tween module in the build" ON)

add_library(engine_module_animation_tween SHARED
    module_plugin.cpp
    animation.cpp
)

target_link_libraries(engine_module_animation_tween
    PUBLIC engine_kernel
    PUBLIC engine_capability_animation
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_scene
    PUBLIC engine_capability_pbr
    PUBLIC engine_capability_renderer
)

set_target_properties(engine_module_animation_tween PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_animation_tween PRIVATE -Wl,--export-all-symbols)
endif()

add_library(engine_capability_animation INTERFACE)
target_include_directories(engine_capability_animation INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/animation_capability.hpp")

# Excluded from the default build when the option is OFF: the target stays
# defined for consumers, but nothing builds the plugin binary unless an
# in-tree consumer actually links it (the runtime then simply does not see
# the plugin, and a demand for its capability fails startup precisely).
if(NOT ENGINE_BUILD_MODULE_ANIMATION_TWEEN)
    set_target_properties(engine_module_animation_tween PROPERTIES EXCLUDE_FROM_ALL ON)
endif()
