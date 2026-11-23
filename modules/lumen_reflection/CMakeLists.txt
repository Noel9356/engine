add_library(
    engine_lumen_reflection_capabilities
    INTERFACE
)
target_include_directories(
    engine_lumen_reflection_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_lumen_reflection_capabilities
    INTERFACE engine_renderer_types
              engine_dep_glm
)
add_library(
    engine_module_lumen_reflection
    MODULE
    source/lumen_reflection.cpp
    source/lumen_reflection_scene.cpp
    source/lumen_reflection_pipeline.cpp
    source/lumen_reflection_denoise.cpp
    source/lumen_reflection_debug.cpp
)
target_link_libraries(
    engine_module_lumen_reflection
    PRIVATE engine_kernel
            engine_lumen_reflection_capabilities
            engine_lumen_gi_capabilities
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
        engine_module_lumen_reflection
        PRIVATE GLM_FORCE_DEFAULT
                GLM_FORCE_INTRINSICS_V
    )
endif()
if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_options(
        engine_module_lumen_reflection
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno;-funroll-loops>
    )
endif()
set_target_properties(
    engine_module_lumen_reflection
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_reflection"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_reflection"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/lumen_reflection"
)
add_custom_command(
    TARGET engine_module_lumen_reflection
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_lumen_reflection>/module.toml"
    COMMENT "placing the lumen_reflection module descriptor next to its library"
)
