engine_add_application(
    demo_planet
    SOURCES
    source/main.cpp
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
