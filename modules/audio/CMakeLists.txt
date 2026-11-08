include(engine_fetch_dependency)

option(
    ENGINE_AUDIO_ENABLE_MINIAUDIO
    "Build the audio module with the miniaudio device/mixer backend"
    ON
)

set(
    ENGINE_AUDIO_MINIAUDIO_BACKENDS
    "DEFAULT"
    CACHE STRING
    "miniaudio backends to compile (DEFAULT or a semicolon-separated list)"
)
set_property(
    CACHE ENGINE_AUDIO_MINIAUDIO_BACKENDS
    PROPERTY STRINGS
             DEFAULT
             WASAPI
             DSOUND
             WINMM
             ALSA
             PULSEAUDIO
             JACK
             COREAUDIO
             SNDIO
             AUDIO4
             OSS
             AAudio
             OPENSL
             WEBAUDIO
             CUSTOM
             NULL
)
option(
    ENGINE_AUDIO_MINIAUDIO_DISABLE_RUNTIME_LINKING
    "Disable miniaudio runtime linking (useful for restricted platforms)"
    OFF
)

if(ENGINE_AUDIO_ENABLE_MINIAUDIO)
    engine_fetch_dependency(
        miniaudio
        OWNER
        audio
        GIT_REPOSITORY
        https://github.com/mackron/miniaudio.git
        GIT_TAG
        0.11.26
    )

    add_library(
        engine_audio_miniaudio
        STATIC
        "${miniaudio_SOURCE_DIR}/miniaudio.c"
    )
    target_include_directories(
        engine_audio_miniaudio
        SYSTEM PUBLIC "${miniaudio_SOURCE_DIR}"
    )
    target_compile_definitions(
        engine_audio_miniaudio
        PUBLIC MA_NO_ENCODING
               MA_NO_GENERATION
    )

    if(ENGINE_AUDIO_MINIAUDIO_BACKENDS STREQUAL "")
        message(FATAL_ERROR "ENGINE_AUDIO_MINIAUDIO_BACKENDS must be DEFAULT or a non-empty backend list")
    endif()

    if(ENGINE_AUDIO_MINIAUDIO_DISABLE_RUNTIME_LINKING AND ENGINE_AUDIO_MINIAUDIO_BACKENDS STREQUAL "DEFAULT")
        message(
            FATAL_ERROR
            "Set ENGINE_AUDIO_MINIAUDIO_BACKENDS explicitly when disabling runtime linking so required system libraries can be linked"
        )
    endif()

    if(NOT ENGINE_AUDIO_MINIAUDIO_BACKENDS STREQUAL "DEFAULT")
        set(_engine_audio_supported_backends
            WASAPI DSOUND WINMM ALSA PULSEAUDIO JACK COREAUDIO SNDIO AUDIO4 OSS AAUDIO OPENSL WEBAUDIO CUSTOM NULL
        )
        target_compile_definitions(
            engine_audio_miniaudio
            PUBLIC MA_ENABLE_ONLY_SPECIFIC_BACKENDS
        )
        set(_engine_audio_selected_backends)
        foreach(_engine_audio_backend IN LISTS ENGINE_AUDIO_MINIAUDIO_BACKENDS)
            string(TOUPPER "${_engine_audio_backend}" _engine_audio_backend_upper)
            if(NOT _engine_audio_backend_upper IN_LIST _engine_audio_supported_backends)
                message(
                    FATAL_ERROR
                    "Unsupported ENGINE_AUDIO_MINIAUDIO_BACKENDS entry: ${_engine_audio_backend}"
                )
            endif()
            list(APPEND _engine_audio_selected_backends "${_engine_audio_backend_upper}")
            target_compile_definitions(
                engine_audio_miniaudio
                PUBLIC "MA_ENABLE_${_engine_audio_backend_upper}"
            )
        endforeach()
    endif()

    if(ENGINE_AUDIO_MINIAUDIO_DISABLE_RUNTIME_LINKING)
        target_compile_definitions(
            engine_audio_miniaudio
            PUBLIC MA_NO_RUNTIME_LINKING
        )

        if(APPLE)
            target_link_libraries(
                engine_audio_miniaudio
                PUBLIC "-framework CoreFoundation"
                       "-framework CoreAudio"
                       "-framework AudioToolbox"
            )
        elseif(ANDROID)
            if("AAUDIO" IN_LIST _engine_audio_selected_backends)
                target_link_libraries(engine_audio_miniaudio PUBLIC aaudio)
            endif()
            if("OPENSL" IN_LIST _engine_audio_selected_backends)
                target_link_libraries(engine_audio_miniaudio PUBLIC OpenSLES)
            endif()
        elseif(UNIX AND NOT APPLE)
            set(_engine_audio_pkg_backends)
            foreach(_engine_audio_backend IN ITEMS ALSA PULSEAUDIO JACK SNDIO)
                if(_engine_audio_backend IN_LIST _engine_audio_selected_backends)
                    list(APPEND _engine_audio_pkg_backends "${_engine_audio_backend}")
                endif()
            endforeach()
            if(_engine_audio_pkg_backends)
                find_package(PkgConfig REQUIRED)
                foreach(_engine_audio_backend IN LISTS _engine_audio_pkg_backends)
                    if(_engine_audio_backend STREQUAL "ALSA")
                        pkg_check_modules(ENGINE_AUDIO_ALSA REQUIRED IMPORTED_TARGET alsa)
                        target_link_libraries(engine_audio_miniaudio PUBLIC PkgConfig::ENGINE_AUDIO_ALSA)
                    elseif(_engine_audio_backend STREQUAL "PULSEAUDIO")
                        pkg_check_modules(ENGINE_AUDIO_PULSEAUDIO REQUIRED IMPORTED_TARGET libpulse)
                        target_link_libraries(engine_audio_miniaudio PUBLIC PkgConfig::ENGINE_AUDIO_PULSEAUDIO)
                    elseif(_engine_audio_backend STREQUAL "JACK")
                        pkg_check_modules(ENGINE_AUDIO_JACK REQUIRED IMPORTED_TARGET jack)
                        target_link_libraries(engine_audio_miniaudio PUBLIC PkgConfig::ENGINE_AUDIO_JACK)
                    elseif(_engine_audio_backend STREQUAL "SNDIO")
                        pkg_check_modules(ENGINE_AUDIO_SNDIO REQUIRED IMPORTED_TARGET sndio)
                        target_link_libraries(engine_audio_miniaudio PUBLIC PkgConfig::ENGINE_AUDIO_SNDIO)
                    endif()
                endforeach()
            endif()
        elseif(WIN32 AND "DSOUND" IN_LIST _engine_audio_selected_backends)
            target_link_libraries(engine_audio_miniaudio PUBLIC dsound)
        endif()
    endif()

    target_link_libraries(
        engine_audio_miniaudio
        PUBLIC Threads::Threads
    )
    if(WIN32)
        target_link_libraries(
            engine_audio_miniaudio
            PUBLIC ole32
                   uuid
                   winmm
                   avrt
        )
    elseif(
        CMAKE_SYSTEM_NAME
        STREQUAL
        "Linux"
    )
        target_link_libraries(
            engine_audio_miniaudio
            PUBLIC dl
                   m
        )
    endif()
    set_target_properties(
        engine_audio_miniaudio
        PROPERTIES POSITION_INDEPENDENT_CODE
                   ON
    )
endif()

add_library(
    engine_audio_capabilities
    INTERFACE
)
target_include_directories(
    engine_audio_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_audio
    MODULE
    source/audio.cpp
    source/lua_bridge.cpp
)
target_include_directories(
    engine_module_audio
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../script_runtime/include"
)

target_compile_definitions(
    engine_module_audio
    PRIVATE ENGINE_AUDIO_MODULE_VERSION="0.3.0"
)

target_link_libraries(
    engine_module_audio
    PRIVATE engine_kernel
            engine_audio_capabilities
            engine_util_types
            Threads::Threads
)

if(ENGINE_AUDIO_ENABLE_MINIAUDIO)
    target_compile_definitions(
        engine_module_audio
        PRIVATE ENGINE_AUDIO_BACKEND_MINIAUDIO=1
    )
    target_link_libraries(
        engine_module_audio
        PRIVATE engine_audio_miniaudio
    )
endif()

set_target_properties(
    engine_module_audio
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/audio"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/audio"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/audio"
)

add_custom_command(
    TARGET engine_module_audio
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_audio>/module.toml"
    COMMENT "placing the audio module descriptor next to its library"
)
