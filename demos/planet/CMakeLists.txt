# The demo's own [planet] configuration forwarding (source/main.cpp) parses
# the demo's engine.toml and its profiles with the same TOML library the
# kernel reads configuration through; the dependency coordinator deduplicates
# this declaration with the kernel's identical pin.
engine_fetch_dependency(
    tomlplusplus
    OWNER demo_planet
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    HEADER_ONLY
    INCLUDE_SUBDIRS
    include
    include/toml++
)

engine_add_application(
    demo_planet
    SOURCES
    source/main.cpp
    source/planet_demo_options.cpp
    source/planet_camera_input.cpp
    source/planet_chunk_renderer.cpp
    source/planet_demo_tests.cpp
    source/planet_benchmark.cpp
    source/planet_scatter.cpp
    source/planet_viewpoints.cpp
    source/planet_walk_controller.cpp
    ../common/demo_harness.cpp
    INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/../common"
    CAPABILITIES
    window
    renderer
    camera
    pbr
    planet
    LIBRARIES
    engine_dep_glm
    # [planet] configuration forwarding parses the demo's TOML directly.
    engine_dep_tomlplusplus
    # The determinism self-test constructs a bare Planet to exercise
    # conforming-mask builds (the capability hashChunk has no mask argument),
    # which needs the module core's symbols at link time. The core is an
    # OBJECT library (the runtime .so is built from the same objects).
    engine_planet_core
    CONFIG_DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/config"
)

