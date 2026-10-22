add_library(engine_module_screenshot STATIC
    screenshot.cpp
)

target_compile_definitions(engine_module_screenshot PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)

target_link_libraries(engine_module_screenshot
    PUBLIC engine_kernel
    PRIVATE engine_stb_bindings
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_screenshot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
