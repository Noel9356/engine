# The demo's own configuration keys (source/main.cpp) parse the demo's
# engine.toml with the same TOML library the kernel reads configuration
# through; the dependency coordinator deduplicates this declaration with the
# kernel's identical pin.
engine_fetch_dependency(
    tomlplusplus
    OWNER demo_lighting
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    HEADER_ONLY
    INCLUDE_SUBDIRS
    include
    include/toml++
)

engine_add_application(
    demo_lighting
    SOURCES
    source/main.cpp
    source/lighting_demo_options.cpp
    source/lighting_scene.cpp
    ../common/demo_harness.cpp
    ../common/demo_environment.cpp
    INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/../common"
    CAPABILITIES
    window
    renderer
    camera
    pbr
    direct_lighting
    radiance_cache
    reflection_field
    LIBRARIES
    engine_dep_glm
    CONFIG_DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/config"
)
