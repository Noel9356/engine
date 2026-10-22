add_library(engine_module_input STATIC
    input.cpp
)

target_compile_definitions(engine_module_input PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(engine_module_input
    PUBLIC engine_kernel
    PRIVATE glfw
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_input PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
