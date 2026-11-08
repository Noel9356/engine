include(engine_fetch_dependency)

option(
    ENGINE_SCRIPT_RUNTIME_ENABLE_LUA
    "Build the script runtime module with Lua"
    ON
)

if(ENGINE_SCRIPT_RUNTIME_ENABLE_LUA)
    engine_fetch_dependency(
        lua
        OWNER
        script_runtime
        URL
        https://www.lua.org/ftp/lua-5.5.1.tar.gz
        URL_HASH
        SHA256=1c4b4068d67061f2a2231ad2b5422e77acea1487ea9890f6320af614f4373dce
    )

    set(lua_sources
        lapi.c
        lcode.c
        lctype.c
        ldebug.c
        ldo.c
        ldump.c
        lfunc.c
        lgc.c
        llex.c
        lmem.c
        lobject.c
        lopcodes.c
        lparser.c
        lstate.c
        lstring.c
        ltable.c
        ltm.c
        lundump.c
        lvm.c
        lzio.c
        lauxlib.c
        lbaselib.c
        lmathlib.c
        lstrlib.c
        ltablib.c
        lutf8lib.c
    )
    list(
        TRANSFORM
        lua_sources
        PREPEND "${lua_SOURCE_DIR}/src/"
    )
    add_library(
        engine_script_runtime_lua
        STATIC
        ${lua_sources}
    )
    target_include_directories(
        engine_script_runtime_lua
        PUBLIC "${lua_SOURCE_DIR}/src"
    )
    if(UNIX
       AND NOT
           APPLE
    )
        target_link_libraries(
            engine_script_runtime_lua
            PUBLIC m
        )
    endif()
    set_target_properties(
        engine_script_runtime_lua
        PROPERTIES POSITION_INDEPENDENT_CODE
                   ON
    )
endif()

add_library(
    engine_script_runtime_capabilities
    INTERFACE
)
target_include_directories(
    engine_script_runtime_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(
    engine_module_script_runtime
    MODULE
    source/script_runtime.cpp
)
target_compile_definitions(
    engine_module_script_runtime
    PRIVATE ENGINE_SCRIPT_RUNTIME_MODULE_VERSION="0.1.0"
)
target_link_libraries(
    engine_module_script_runtime
    PRIVATE engine_kernel
            engine_script_runtime_capabilities
            engine_util_types
            Threads::Threads
)
if(ENGINE_SCRIPT_RUNTIME_ENABLE_LUA)
    target_compile_definitions(
        engine_module_script_runtime
        PRIVATE ENGINE_SCRIPT_RUNTIME_BACKEND_LUA=1
    )
    target_link_libraries(
        engine_module_script_runtime
        PRIVATE engine_script_runtime_lua
    )
endif()

set_target_properties(
    engine_module_script_runtime
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/script_runtime"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/script_runtime"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/script_runtime"
)

add_custom_command(
    TARGET engine_module_script_runtime
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_script_runtime>/module.toml"
    COMMENT "placing the script_runtime module descriptor next to its library"
)
