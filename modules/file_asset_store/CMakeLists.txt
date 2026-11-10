add_library(
    engine_file_asset_store_capabilities
    INTERFACE
)
target_include_directories(
    engine_file_asset_store_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_asset_catalog
    STATIC
    source/asset_catalog.cpp
)
target_include_directories(
    engine_asset_catalog
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_asset_catalog
    PUBLIC engine_kernel
)
target_compile_features(
    engine_asset_catalog
    PUBLIC cxx_std_20
)

add_library(
    engine_module_file_asset_store
    MODULE
    source/file_asset_store.cpp
)
target_compile_definitions(
    engine_module_file_asset_store
    PRIVATE ENGINE_FILE_ASSET_STORE_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_file_asset_store
    PRIVATE engine_file_asset_store_capabilities
            engine_asset_catalog
            engine_util_types
            Threads::Threads
)

set_target_properties(
    engine_module_file_asset_store
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/file_asset_store"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/file_asset_store"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/file_asset_store"
)

add_custom_command(
    TARGET engine_module_file_asset_store
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_file_asset_store>/module.toml"
    COMMENT "placing the file_asset_store module descriptor next to its library"
)
