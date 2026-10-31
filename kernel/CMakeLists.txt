add_library(engine_kernel SHARED
        source/component_context.cpp
        source/configuration.cpp
        source/dependency_resolver.cpp
        source/dynamic_library.cpp
        source/engine_kernel.cpp
        source/filesystem_utilities.cpp
        source/mod_discovery.cpp
        source/mod_manifest.cpp
        source/module_manifest.cpp
        source/module_plugin_cache.cpp
        source/sha256.cpp
)

set_target_properties(engine_kernel
        PROPERTIES
                PREFIX "lib"
                RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime"
                LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runtime"
                WINDOWS_EXPORT_ALL_SYMBOLS ON
)

target_include_directories(engine_kernel
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(engine_kernel
        PRIVATE engine_tomlplusplus_bindings
        PRIVATE Threads::Threads
)

if(MSVC)
        target_compile_options(engine_kernel PUBLIC /W4 /permissive-)
else()
        target_compile_options(engine_kernel PUBLIC -Wall -Wextra -Wpedantic)
endif()