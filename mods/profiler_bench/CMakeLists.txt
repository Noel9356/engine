# The profiler_bench native mod builds itself: the root CMakeLists discovers
# every mods/<id>/CMakeLists.txt and adds it as a subdirectory, so shipping
# this package means creating the directory - no shared build file lists mods.
# The target links the kernel plus the published capability headers of the two
# capabilities it consumes (header-only INTERFACE targets; no provider binary
# appears on the link line) and lands the library inside the package's runtime
# directory, where the kernel's native mod loader looks for it
# (lib<entry>.so / <entry>.dll, see mod_discovery.cpp loadNativeModEntry).

add_library(engine_mod_profiler_bench SHARED
    profiler_bench.cpp
)

target_link_libraries(engine_mod_profiler_bench
    PUBLIC engine_kernel
    PUBLIC engine_capability_logging
    PUBLIC engine_capability_profiler
)

set_target_properties(engine_mod_profiler_bench PROPERTIES
    PREFIX "lib"
    # The library base name must agree with the package manifest: the kernel's
    # native mod loader looks for lib<entry>.so next to mod.toml, and the
    # manifest's entry is "profiler_bench".
    OUTPUT_NAME "profiler_bench"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/mods/profiler_bench"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/mods/profiler_bench"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
# MinGW: the mod's one explicit exported symbol (the C entry table) would
# otherwise disable the linker's auto-export for the C++ surface the kernel
# calls through the ModInterface vtable.
if(MINGW)
    target_link_options(engine_mod_profiler_bench PRIVATE -Wl,--export-all-symbols)
endif()
