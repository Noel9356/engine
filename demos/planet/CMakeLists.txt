engine_add_application(
    demo_planet
    SOURCES
    source/main.cpp
    source/demo/planet_demo_options.cpp
    source/demo/planet_camera_input.cpp
    source/demo/planet_chunk_renderer.cpp
    source/demo/planet_demo_tests.cpp
    source/demo/planet_benchmark.cpp
    source/demo/planet_viewpoints.cpp
    source/demo/planet_walk_controller.cpp
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
    # The determinism self-test constructs a bare Planet to exercise
    # conforming-mask builds (the capability hashChunk has no mask argument),
    # which needs the module core's symbols at link time. The core is an
    # OBJECT library (the runtime .so is built from the same objects).
    engine_planet_core
    CONFIG_DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/config"
)
