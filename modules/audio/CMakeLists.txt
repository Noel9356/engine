add_library(engine_module_audio SHARED
    module_plugin.cpp
    audio.cpp
)

# miniaudio: vendored, pinned single header (third_party/miniaudio). SYSTEM
# keeps its warnings out of the engine's warning-clean build; the C audio
# backend needs dl for its runtime linking of the platform device APIs.
target_link_libraries(engine_module_audio
    PUBLIC engine_kernel
    PUBLIC engine_capability_audio
    PUBLIC engine_capability_logging
    PRIVATE engine_miniaudio_bindings
)

set_target_properties(engine_module_audio PROPERTIES
    PREFIX "lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/modules"
    BUILD_RPATH "$ORIGIN;$ORIGIN/.."
    INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
if(MINGW)
    target_link_options(engine_module_audio PRIVATE -Wl,--export-all-symbols)
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(engine_module_audio PRIVATE dl)
endif()

add_library(engine_capability_audio INTERFACE)
target_include_directories(engine_capability_audio INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

engine_guard_capability_header("${CMAKE_CURRENT_SOURCE_DIR}/audio_capability.hpp")
