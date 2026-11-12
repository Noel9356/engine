include(engine_fetch_dependency)

add_library(
    engine_models_capabilities
    INTERFACE
)
target_include_directories(
    engine_models_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_models_capabilities
    INTERFACE engine_model_types
              engine_renderer_types
)

engine_fetch_dependency(
    tinyobjloader
    OWNER
    models
    GIT_REPOSITORY
    https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG
    v2.0.0rc13
    HEADER_ONLY
)

engine_fetch_dependency(
    cgltf
    OWNER
    models
    GIT_REPOSITORY
    https://github.com/jkuhlmann/cgltf.git
    GIT_TAG
    v1.15
    HEADER_ONLY
)

set(MESHOPT_BUILD_DEMO
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(MESHOPT_BUILD_GLTFPACK
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(MESHOPT_INSTALL
    OFF
    CACHE BOOL
          ""
          FORCE
)
engine_fetch_dependency(
    meshoptimizer
    OWNER
    models
    GIT_REPOSITORY
    https://github.com/zeux/meshoptimizer.git
    GIT_TAG
    v1.3
    MAKE_AVAILABLE
)

add_library(
    engine_module_models
    MODULE
    source/core/models.cpp
    source/importers/obj_importer.cpp
    source/importers/stl_importer.cpp
    source/importers/ply_importer.cpp
    source/importers/off_importer.cpp
    source/importers/gltf_importer.cpp
    source/importers/cgltf_implementation.cpp
    source/optimization/model_optimizer.cpp
)

target_include_directories(
    engine_module_models
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source/internal"
)

target_compile_definitions(
    engine_module_models
    PRIVATE ENGINE_MODELS_MODULE_VERSION="0.1.0"
)

target_link_libraries(
    engine_module_models
    PRIVATE engine_kernel
            engine_models_capabilities
            engine_model_types
            engine_renderer_types
            engine_dep_tinyobjloader
            engine_dep_cgltf
            meshoptimizer
)

set_target_properties(
    engine_module_models
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/models"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/models"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/models"
)

add_custom_command(
    TARGET engine_module_models
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_models>/module.toml"
    COMMENT "placing the models module descriptor next to its library"
)
