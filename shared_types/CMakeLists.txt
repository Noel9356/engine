add_library(
    engine_renderer_types
    INTERFACE
)
target_include_directories(
    engine_renderer_types
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_renderer_types
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

add_library(
    engine_util_types
    INTERFACE
)
target_include_directories(
    engine_util_types
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(
    engine_util_types
    INTERFACE cxx_std_20
)
