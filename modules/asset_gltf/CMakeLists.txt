add_library(engine_module_asset_gltf SHARED
    module_plugin.cpp
    asset.cpp
    gltf_json.cpp
    gltf_import.cpp
)

target_link_libraries(engine_module_asset_gltf
    PUBLIC engine_kernel
    PUBLIC engine_capability_asset
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_scene
)

set_target_properties(engine_module_asset_gltf PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_asset_gltf PRIVATE -Wl,--export-all-symbols)
endif()

add_library(engine_capability_asset INTERFACE)
target_include_directories(engine_capability_asset INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/asset_capability.hpp")
