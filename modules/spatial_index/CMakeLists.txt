add_library(
    engine_module_spatial_index
    INTERFACE
)
target_include_directories(
    engine_module_spatial_index
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_module_spatial_index
    INTERFACE cxx_std_20
)
add_library(
    engine_spatial_index
    ALIAS
    engine_module_spatial_index
)
