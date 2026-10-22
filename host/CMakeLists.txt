add_executable(engine
    main.cpp
    leak_check.cpp
)

target_link_libraries(engine
    PRIVATE engine_kernel
)

if(ENGINE_BUILD_MODULE_LOGGING)
    target_link_libraries(engine PRIVATE engine_module_logging)
endif()
if(ENGINE_BUILD_MODULE_WINDOW)
    target_link_libraries(engine PRIVATE engine_module_window)
endif()
if(ENGINE_BUILD_MODULE_INPUT)
    target_link_libraries(engine PRIVATE engine_module_input)
endif()
if(ENGINE_BUILD_MODULE_RENDERER)
    target_link_libraries(engine PRIVATE engine_module_renderer)
endif()
if(ENGINE_BUILD_MODULE_SCREENSHOT)
    target_link_libraries(engine PRIVATE engine_module_screenshot)
endif()
if(ENGINE_BUILD_MODULE_LUA_SCRIPTING)
    target_link_libraries(engine PRIVATE engine_module_lua_scripting)
endif()
if(ENGINE_BUILD_MODULE_PBR)
    target_link_libraries(engine PRIVATE engine_module_pbr)
endif()
if(ENGINE_BUILD_MODULE_RADIANCE_CASCADES)
    target_link_libraries(engine PRIVATE engine_module_radiance_cascades)
endif()
if(ENGINE_BUILD_MODULE_RAY_REFLECTIONS)
    target_link_libraries(engine PRIVATE engine_module_ray_reflections)
endif()
