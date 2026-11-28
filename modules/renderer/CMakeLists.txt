include(engine_fetch_dependency)
include(engine_module_compile_shaders)

add_library(
    engine_renderer_capabilities
    INTERFACE
)
target_link_libraries(
    engine_renderer_capabilities
    INTERFACE engine_renderer_types
)
target_include_directories(
    engine_renderer_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_renderer_capabilities
    INTERFACE engine_window_capabilities
              engine_dep_glm
)

engine_fetch_dependency(
    glm
    OWNER
    renderer
    GIT_REPOSITORY
    https://github.com/g-truc/glm.git
    GIT_TAG
    1.0.1
    HEADER_ONLY
)

engine_fetch_dependency(
    stb
    OWNER
    renderer
    GIT_REPOSITORY
    https://github.com/nothings/stb.git
    GIT_TAG
    003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5
    HEADER_ONLY
)

engine_fetch_dependency(
    meshopt
    OWNER
    renderer
    GIT_REPOSITORY
    https://github.com/zeux/meshoptimizer.git
    GIT_TAG
    v1.3
    HEADER_ONLY
)

# libktx (Khronos KTX-Software): KTX 1/2 container parsing, zstd/BasisLZ
# supercompression inflation and Basis (ETC1S/UASTC) transcoding for
# createTextureFromFile's KTX path. Built static with the encoder and the
# command-line tools off: the engine only loads and transcodes.
set(KTX_FEATURE_TOOLS
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_TESTS
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_JNI
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_PY
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_DOC
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_TOOLS_CTS
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_LOADTEST_APPS
    OFF
    CACHE STRING
          ""
          FORCE
)
set(KTX_FEATURE_VK_UPLOAD
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_FEATURE_GL_UPLOAD
    OFF
    CACHE BOOL
          ""
          FORCE
)
set(KTX_GIT_VERSION_FULL
    v4.4.2
    CACHE STRING
          ""
          FORCE
)
set(BUILD_SHARED_LIBS
    OFF
    CACHE BOOL
          ""
          FORCE
)
engine_fetch_dependency(
    ktx
    OWNER
    renderer
    GIT_REPOSITORY
    https://github.com/KhronosGroup/KTX-Software.git
    GIT_TAG
    v4.4.2
    MAKE_AVAILABLE
)

# The fetched libktx inherits the engine's Release -march=native, and Basisu's
# SSE kernels refuse to compile under any AVX family (#error at the top of
# basisu_kernels_sse.cpp: the CPPSPMD design is written against the x86-64
# SSE2 baseline). Target-level options append AFTER the inherited architecture
# flag, so turning the AVX families off for the ktx target alone keeps the
# engine's own -march=native units (the AVX-512 rasterizer twins) untouched
# while the Basisu kernels build at the ISA they are written for.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ktx PRIVATE -mno-avx -mno-avx2 -mno-avx512f)
endif()

# bcdec: single-header BC1-BC7 software decoder (MIT/Unlicense) used by the
# KTX path's level-0 mean and the headless backend's block-texture decode.
engine_fetch_dependency(
    bcdec
    OWNER
    renderer
    GIT_REPOSITORY
    https://github.com/iOrange/bcdec.git
    GIT_TAG
    80859ed3b7afb1c527a2a99d70c61457bea72d0c
    HEADER_ONLY
)

option(
    ENGINE_RENDERER_ENABLE_VULKAN
    "Build the renderer module with the Vulkan backend"
    ON
)

if(ENGINE_RENDERER_ENABLE_VULKAN)
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
            renderer
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
            message(STATUS "renderer module: no Vulkan loader runtime found; Vulkan backends are compiled out")
        endif()
    endif()
endif()

add_library(
    engine_module_renderer
    MODULE
    source/backend/headless/headless_backend.cpp
    source/core/draw_scheduler.cpp
    source/core/frame_inputs.cpp
    source/core/ktx_texture_loader.cpp
    source/core/render_data.cpp
    source/core/renderer.cpp
    source/core/renderer_settings.cpp
    source/backend/vulkan/vulkan_backend.cpp
    source/backend/vulkan/vulkan_device.cpp
    source/backend/vulkan/vulkan_frames.cpp
    source/backend/vulkan/vulkan_gpu_probe.cpp
    source/backend/vulkan/vulkan_pipeline.cpp
    source/backend/vulkan/vulkan_resources.cpp
    source/backend/vulkan/vulkan_swapchain.cpp
)

target_include_directories(
    engine_module_renderer
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/source/core"
        "${CMAKE_CURRENT_SOURCE_DIR}/source/backend"
        "${CMAKE_CURRENT_SOURCE_DIR}/source/backend/headless"
        "${CMAKE_CURRENT_SOURCE_DIR}/source/backend/vulkan"
)

target_compile_definitions(
    engine_module_renderer
    PRIVATE ENGINE_RENDERER_MODULE_VERSION="0.1.0"
)

if(CMAKE_CXX_COMPILER_ID
   MATCHES
   "GNU|Clang|AppleClang"
)
    target_compile_definitions(
        engine_module_renderer
        PRIVATE GLM_FORCE_DEFAULT
                GLM_FORCE_INTRINSICS_V
    )
    target_compile_options(
        engine_module_renderer
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno;-funroll-loops>
    )
    # -ffast-math for the CPU rasterizer, minus its -ffinite-math-only
    # component: that flag makes GCC fold std::isfinite() to constant true,
    # silently deleting every NaN/Inf guard in the translation unit (mesh
    # bounds validation, clip-space rejection, degenerate-projection
    # handling), and it licenses comparison reorderings that are invalid for
    # the NaN-propagating std::min/std::max semantics the output encoders
    # (render_shared.hpp) and the near-plane clip walk are written against.
    # The rest of -ffast-math (reassociation, no signed zeros, no math errno)
    # stays; the finiteness guards in this file use the flag-proof
    # finite_math.hpp bit test regardless.
    set_source_files_properties(
        source/backend/headless/headless_backend.cpp
        PROPERTIES COMPILE_OPTIONS
                   "-ffast-math;-fno-finite-math-only;-fno-math-errno;-funroll-loops"
    )
endif()

engine_module_compile_shaders(
    TARGET
    engine_module_renderer
    NAMESPACE
    engine
    renderer
    SHADERS
    fur.frag
    fur.vert
    mesh.vert
    mesh.frag
)

target_link_libraries(
    engine_module_renderer
    PRIVATE engine_kernel
            engine_renderer_capabilities
            engine_util_types
            engine_window_capabilities
            engine_pbr_capabilities
            engine_direct_lighting_capabilities
            engine_radiance_cache_capabilities
            engine_reflection_field_capabilities
            engine_dep_stb
            engine_dep_bcdec
            ktx
            meshoptimizer
)

if(ENGINE_RENDERER_ENABLE_VULKAN
   AND TARGET
       engine_vulkan
)
    engine_module_compile_shaders(
        TARGET
        engine_module_renderer
        NAMESPACE
        engine
        renderer
        SHADERS
        ui.vert
        ui.frag
    )

    engine_fetch_dependency(
        vma
        OWNER
        renderer
        GIT_REPOSITORY
        https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
        GIT_TAG
        v3.4.0
        HEADER_ONLY
        INCLUDE_SUBDIRS
        include
    )
    target_compile_definitions(
        engine_module_renderer
        PRIVATE ENGINE_RENDERER_BACKEND_VULKAN=1
    )
    target_link_libraries(
        engine_module_renderer
        PRIVATE engine_vulkan
                engine_dep_vma
    )
elseif(ENGINE_RENDERER_ENABLE_VULKAN)
    message(STATUS "renderer module: no Vulkan headers/loader found; building without the Vulkan backend")
endif()

set_target_properties(
    engine_module_renderer
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/renderer"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/renderer"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/renderer"
)

add_custom_command(
    TARGET engine_module_renderer
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_renderer>/module.toml"
    COMMENT "placing the renderer module descriptor next to its library"
)
