add_library(
    engine_render_types
    INTERFACE
)
target_include_directories(
    engine_render_types
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_render_types
    INTERFACE cxx_std_20
)

add_library(
    engine_model_types
    INTERFACE
)
target_include_directories(
    engine_model_types
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_model_types
    INTERFACE cxx_std_20
)
