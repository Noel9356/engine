add_library(
    engine_reflection_field_capabilities
    INTERFACE
)
target_include_directories(
    engine_reflection_field_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_reflection_field_capabilities
    INTERFACE engine_renderer_types
              engine_dep_glm
)
add_library(
    engine_module_reflection_field
    MODULE
    source/reflection_field.cpp
    source/reflection_field_scene.cpp
    source/reflection_field_pipeline.cpp
    source/reflection_field_denoise.cpp
    source/reflection_field_debug.cpp
)
target_link_libraries(
    engine_module_reflection_field
    PRIVATE engine_kernel
            engine_reflection_field_capabilities
            engine_radiance_cache_capabilities
            engine_pbr_capabilities
            engine_direct_lighting_capabilities
            engine_dep_glm
)
# The ray and denoise passes are sqrt/division heavy: the same GLM intrinsics
# and math options the renderer module compiles with let the auto-vectorizer
# reach the hot loops (errno-bound square roots block it otherwise).
if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_definitions(
        engine_module_reflection_field
        PRIVATE GLM_FORCE_DEFAULT
                GLM_FORCE_INTRINSICS_V
    )
    target_compile_options(
        engine_module_reflection_field
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno;-funroll-loops>
    )
endif()
set_target_properties(
    engine_module_reflection_field
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/reflection_field"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/reflection_field"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/reflection_field"
)
add_custom_command(
    TARGET engine_module_reflection_field
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_reflection_field>/module.toml"
    COMMENT "placing the reflection_field module descriptor next to its library"
)
