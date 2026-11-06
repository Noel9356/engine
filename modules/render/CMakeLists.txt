include(engine_fetch_dependency)
include(engine_module_compile_shaders)

add_library(
    engine_render_capabilities
    INTERFACE
)
target_link_libraries(
    engine_render_capabilities
    INTERFACE engine_render_types
)
target_include_directories(
    engine_render_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_render_capabilities
    INTERFACE engine_window_capabilities
              engine_dep_glm
)

engine_fetch_dependency(
    glm
    OWNER
    render
    GIT_REPOSITORY
    https://github.com/g-truc/glm.git
    GIT_TAG
    1.0.1
    HEADER_ONLY
)

engine_fetch_dependency(
    stb
    OWNER
    render
    GIT_REPOSITORY
    https://github.com/nothings/stb.git
    GIT_TAG
    003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5
    HEADER_ONLY
)

engine_fetch_dependency(
    meshopt
    OWNER
    render
    GIT_REPOSITORY
    https://github.com/zeux/meshoptimizer.git
    GIT_TAG
    v1.3
    HEADER_ONLY
)

option(
    ENGINE_RENDER_ENABLE_VULKAN
    "Build the render module with the Vulkan backend"
    ON
)

if(ENGINE_RENDER_ENABLE_VULKAN)
    find_package(
        Vulkan
        QUIET
    )

    if(Vulkan_FOUND)
        if(NOT
           TARGET
           engine_vulkan
        )
            add_library(
                engine_vulkan
                INTERFACE
            )
            target_link_libraries(
                engine_vulkan
                INTERFACE Vulkan::Vulkan
            )
        endif()
    else()
        engine_fetch_dependency(
            vulkan_headers
            OWNER
            render
            GIT_REPOSITORY
            https://github.com/KhronosGroup/Vulkan-Headers.git
            GIT_TAG
            v1.4.309
            HEADER_ONLY
        )

        find_library(
            ENGINE_VULKAN_RUNTIME_LOADER
            NAMES vulkan-1
                  libvulkan.so.1
        )

        if(ENGINE_VULKAN_RUNTIME_LOADER)
            if(NOT
               TARGET
               engine_vulkan
            )
                add_library(
                    engine_vulkan
                    INTERFACE
                )
                target_include_directories(
                    engine_vulkan
                    INTERFACE "${vulkan_headers_SOURCE_DIR}/include"
                )
                target_link_libraries(
                    engine_vulkan
                    INTERFACE "${ENGINE_VULKAN_RUNTIME_LOADER}"
                )
            endif()
        else()
            message(STATUS "render module: no Vulkan loader runtime found; Vulkan backends are compiled out")
        endif()
    endif()
endif()

add_library(
    engine_module_render
    MODULE
    source/headless_backend.cpp
    source/render_data.cpp
    source/render.cpp
    source/vulkan_backend.cpp
    source/vulkan_device.cpp
    source/vulkan_frames.cpp
    source/vulkan_pipeline.cpp
    source/vulkan_resources.cpp
    source/vulkan_swapchain.cpp
)

target_compile_definitions(
    engine_module_render
    PRIVATE ENGINE_RENDER_MODULE_VERSION="0.1.0"
)

if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_definitions(
        engine_module_render
        PRIVATE GLM_FORCE_DEFAULT
                GLM_FORCE_INTRINSICS_V
    )
endif()

if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_options(
        engine_module_render
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno;-funroll-loops>
    )
endif()

if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    set_source_files_properties(
        source/headless_backend.cpp
        PROPERTIES COMPILE_OPTIONS
                   "-ffast-math;-fno-math-errno;-funroll-loops"
    )
endif()

engine_module_compile_shaders(
    TARGET
    engine_module_render
    NAMESPACE
    engine
    render
    SHADERS
    mesh.vert
    mesh.frag
)

target_link_libraries(
    engine_module_render
    PRIVATE engine_kernel
            engine_render_capabilities
            engine_window_capabilities
            engine_pbr_capabilities
            engine_direct_lighting_capabilities
            engine_dep_stb
            meshoptimizer
)

if(ENGINE_RENDER_ENABLE_VULKAN
   AND TARGET
       engine_vulkan
)
    engine_module_compile_shaders(
        TARGET
        engine_module_render
        NAMESPACE
        engine
        render
        SHADERS
        ui.vert
        ui.frag
    )

    engine_fetch_dependency(
        vma
        OWNER
        render
        GIT_REPOSITORY
        https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
        GIT_TAG
        v3.4.0
        HEADER_ONLY
        INCLUDE_SUBDIRS
        include
    )
    target_compile_definitions(
        engine_module_render
        PRIVATE ENGINE_RENDER_BACKEND_VULKAN=1
    )
    target_link_libraries(
        engine_module_render
        PRIVATE engine_vulkan
                engine_dep_vma
    )
elseif(ENGINE_RENDER_ENABLE_VULKAN)
    message(STATUS "render module: no Vulkan headers/loader found; building without the Vulkan backend")
endif()

set_target_properties(
    engine_module_render
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/render"
)

add_custom_command(
    TARGET engine_module_render
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_render>/module.toml"
    COMMENT "placing the render module descriptor next to its library"
)
