add_library(engine_module_logging STATIC
    logging.cpp
)

target_link_libraries(engine_module_logging PUBLIC engine_kernel)

# The host composition root includes the module header directly.
target_include_directories(engine_module_logging PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
