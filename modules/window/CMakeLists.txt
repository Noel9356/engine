add_library(engine_module_window STATIC
    window.cpp
)

target_compile_definitions(engine_module_window PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(engine_module_window
    PUBLIC engine_kernel
    PRIVATE glfw
    # The window module creates Vulkan presentation surfaces (glfwCreateWindowSurface):
    # it needs the Vulkan C headers. Headers only, PRIVATE - the capability contract
    # stays implementation agnostic and carries no Vulkan types.
    PRIVATE Vulkan::Headers
)

# The host composition root includes the module header directly.
target_include_directories(engine_module_window PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
