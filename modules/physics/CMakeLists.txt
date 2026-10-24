add_library(engine_module_physics SHARED
    module_plugin.cpp
    physics.cpp
)

target_link_libraries(engine_module_physics
    PUBLIC engine_kernel
    PUBLIC engine_capability_physics
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_scene
    PUBLIC engine_capability_renderer
)

set_target_properties(engine_module_physics PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_physics PRIVATE -Wl,--export-all-symbols)
endif()

add_library(engine_capability_physics INTERFACE)
target_include_directories(engine_capability_physics INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/physics_capability.hpp")
