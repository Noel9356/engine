# Kernel and module tests. These run everywhere, including headless machines:
# the Catch2 suite never opens a window or creates a Vulkan instance.

add_executable(engine_tests
    event_bus_tests.cpp
    dependency_resolver_tests.cpp
    permission_system_tests.cpp
    kernel_integration_tests.cpp
    mod_manifest_tests.cpp
    configuration_tests.cpp
    frame_sync_policy_tests.cpp
    camera_controller_tests.cpp
)

# Native mod loading fixture: a real shared library the kernel discovers and
# dlopens from a temporary mod package. It replaces the coverage the shipped
# example mods used to provide (the engine ships content packages only), and
# lives under tests/ so it never appears in a runtime mods directory.
add_library(native_mod_fixture MODULE
    native_mod_fixture/fixture_mod.cpp
)
set_target_properties(native_mod_fixture PROPERTIES
    OUTPUT_NAME native_mod_fixture
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-artifacts/native_mod_fixture"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-artifacts/native_mod_fixture"
)
if(WIN32)
    # The manifest's entry is a library base name; keep the canonical
    # <entry>.dll spelling on every Windows toolchain (see mods/README.md).
    set_target_properties(native_mod_fixture PROPERTIES PREFIX "")
endif()
target_link_libraries(native_mod_fixture PRIVATE engine_kernel)
target_compile_definitions(engine_tests PRIVATE
    ENGINE_NATIVE_MOD_FIXTURE_LIBRARY="$<TARGET_FILE:native_mod_fixture>")
add_dependencies(engine_tests native_mod_fixture)

# frame_sync_policy.hpp lives with the renderer module: it is header-only and
# Vulkan-free, so the unit suite includes it without linking module objects.
target_include_directories(engine_tests PRIVATE "${CMAKE_SOURCE_DIR}/modules/renderer")

target_link_libraries(engine_tests
    PRIVATE engine_kernel
    PRIVATE Catch2::Catch2WithMain
)

add_test(NAME engine_tests COMMAND engine_tests)

# Headless smoke test: boots the assembled runtime (window, surface, swapchain,
# pipeline, frame loop, mods) and renders a fixed number of frames, failing on
# crashes, thrown exceptions and graphics backend validation errors. This is
# the integration layer the unit tests deliberately do not cover: it needs a
# display (its own Xvfb when DISPLAY is unset) and a Vulkan ICD, so it is only
# registered when the demo host is part of the build; on machines without a
# Vulkan loader the run fails fast with a precise message.
if(ENGINE_BUILD_DEMO AND UNIX)
    add_test(NAME engine_headless_smoke
             COMMAND /bin/sh "${CMAKE_SOURCE_DIR}/scripts/run_headless_smoke.sh"
                     "$<TARGET_FILE:engine>" 30 config/smoke.toml)
    set_tests_properties(engine_headless_smoke PROPERTIES TIMEOUT 300)

    # Light leak test: a sealed room with one emissive panel inside and a
    # window opening; the engine itself evaluates the captured frame against
    # the [leak_test] regions (exit 4 = leak or dead GI). This is the
    # automated guard for the no-light-leak guarantee of the radiance
    # cascades pass (docs/robustness.md).
    add_test(NAME engine_leak_check
             COMMAND /bin/sh "${CMAKE_SOURCE_DIR}/scripts/run_leak_check.sh"
                     "$<TARGET_FILE:engine>")
    set_tests_properties(engine_leak_check PROPERTIES TIMEOUT 300)
endif()
