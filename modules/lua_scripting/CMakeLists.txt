add_library(engine_module_lua_scripting STATIC
    lua_scripting.cpp
)

target_link_libraries(engine_module_lua_scripting
    PUBLIC engine_kernel
    PRIVATE engine_sol2_bindings
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_lua_scripting PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
