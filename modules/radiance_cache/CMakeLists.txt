add_library(
    engine_radiance_cache_capabilities
    INTERFACE
)
target_include_directories(
    engine_radiance_cache_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_radiance_cache_capabilities
    INTERFACE engine_renderer_types
              engine_dep_glm
)
add_library(
    engine_module_radiance_cache
    MODULE
    source/radiance_cache.cpp
    source/radiance_cache_scene.cpp
    source/radiance_cache_lighting.cpp
    source/radiance_cache_pipeline.cpp
    source/radiance_cache_caustics.cpp
    source/radiance_cache_gather.cpp
    source/radiance_cache_debug_views.cpp
)
target_link_libraries(
    engine_module_radiance_cache
    PRIVATE engine_kernel
            engine_radiance_cache_capabilities
            engine_pbr_capabilities
            engine_direct_lighting_capabilities
            engine_dep_glm
)
# The gather, capture, and probe passes are sqrt/division heavy: the same GLM
# intrinsics and math options the renderer module compiles with let the
# auto-vectorizer reach the hot loops (errno-bound square roots block it
# otherwise).
if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_definitions(
        engine_module_radiance_cache
        PRIVATE GLM_FORCE_DEFAULT
                GLM_FORCE_INTRINSICS_V
    )
endif()
if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_options(
        engine_module_radiance_cache
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno;-funroll-loops>
    )
endif()
set_target_properties(
    engine_module_radiance_cache
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/radiance_cache"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/radiance_cache"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/radiance_cache"
)
add_custom_command(
    TARGET engine_module_radiance_cache
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_radiance_cache>/module.toml"
    COMMENT "placing the radiance_cache module descriptor next to its library"
)
