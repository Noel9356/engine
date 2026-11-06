add_library(
    engine_application
    STATIC
    source/application_host.cpp
    source/command_line.cpp
    source/frame_clock.cpp
    source/frame_pacer.cpp
)

add_library(
    engine::application
    ALIAS
    engine_application
)

target_include_directories(
    engine_application
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(
    engine_application
    PUBLIC engine_kernel
)

# FramePacer raises the Windows timer resolution while pacing is active.
if(WIN32)
    target_link_libraries(
        engine_application
        PUBLIC winmm
    )
endif()
target_compile_features(
    engine_application
    PUBLIC cxx_std_20
)

if(MSVC)
    target_compile_options(
        engine_application
        PRIVATE /W4
                /permissive-
    )
else()
    target_compile_options(
        engine_application
        PRIVATE -Wall
                -Wextra
                -Wpedantic
    )
endif()
