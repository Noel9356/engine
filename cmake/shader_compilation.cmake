# Build time GLSL to SPIR-V compilation for module owned shaders.
#
# Shaders are module code, not content: they are the implementation of a
# rendering technique, compiled at build time and versioned with the module
# that owns the algorithm. Modules that ship GLSL sources own them inside
# their module directory (modules/<name>/shaders) and call
# engine_compile_module_shaders from their own CMakeLists, which keeps a
# module self-contained: copying the module directory brings its shaders
# along. A missing compiler is a precise configure time error, not a runtime
# surprise.

function(engine_compile_module_shaders)
    cmake_parse_arguments(ARG "" "TARGET" "SOURCES;INCLUDE_DIRS" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "engine_compile_module_shaders requires TARGET <module target>")
    endif()
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "engine_compile_module_shaders requires SOURCES <glsl files>")
    endif()

    find_program(ENGINE_GLSLANG_VALIDATOR
                 NAMES glslangValidator glslang
                 HINTS "$ENV{VULKAN_SDK}/bin" "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin64")
    if(NOT ENGINE_GLSLANG_VALIDATOR)
        message(FATAL_ERROR
                "shader compiler not found: install glslang-tools (Linux: apt install glslang-tools) "
                "or the Vulkan SDK (Windows), or pass -DENGINE_GLSLANG_VALIDATOR=<path>")
    endif()
    message(STATUS "shader compiler: ${ENGINE_GLSLANG_VALIDATOR}")

    # When the validator comes from a user prefix (no system install), its
    # shared libraries may live next to it; extend the loader path accordingly.
    get_filename_component(ENGINE_GLSLANG_BIN_DIR "${ENGINE_GLSLANG_VALIDATOR}" DIRECTORY)
    get_filename_component(ENGINE_GLSLANG_PREFIX "${ENGINE_GLSLANG_BIN_DIR}/.." ABSOLUTE)
    set(ENGINE_SHADER_LOADER_PATH
        "${ENGINE_GLSLANG_PREFIX}/lib:${ENGINE_GLSLANG_PREFIX}/lib/x86_64-linux-gnu:${ENGINE_GLSLANG_PREFIX}/lib64:$ENV{LD_LIBRARY_PATH}")

    # All modules write into one runtime shader directory; output file names
    # carry the module's shader names, so consumers load them through the
    # configured shader_directory without knowing which module owns them.
    # runtime/shaders (not runtime/assets/...): shaders are build output of
    # module code - the runtime 'assets' name is reserved for nothing, since
    # content assets live in mod packages.
    set(ENGINE_SHADER_OUTPUT_DIR "${CMAKE_BINARY_DIR}/runtime/shaders")
    file(MAKE_DIRECTORY "${ENGINE_SHADER_OUTPUT_DIR}")

    set(ENGINE_SHADER_OUTPUTS)
    foreach(shaderSource IN LISTS ARG_SOURCES)
        if(IS_ABSOLUTE "${shaderSource}")
            set(shaderSourcePath "${shaderSource}")
        else()
            set(shaderSourcePath "${CMAKE_CURRENT_SOURCE_DIR}/${shaderSource}")
        endif()
        get_filename_component(shaderName "${shaderSourcePath}" NAME)
        set(shaderOutput "${ENGINE_SHADER_OUTPUT_DIR}/${shaderName}.spv")
        set(includeArguments)
        set(shaderIncludeDepends)
        foreach(includeDirectory IN LISTS ARG_INCLUDE_DIRS)
            if(NOT IS_ABSOLUTE "${includeDirectory}")
                set(includeDirectory "${CMAKE_CURRENT_SOURCE_DIR}/${includeDirectory}")
            endif()
            list(APPEND includeArguments "-I${includeDirectory}")
            # Shared include headers (scene_common.glsl, lighting_common.glsl)
            # are build dependencies of every shader that includes them;
            # without this an include edit silently leaves stale SPIR-V in
            # the runtime shader directory (discovered the hard way while
            # fixing the gather visibility blend). New headers are picked up
            # on the next configure, the same tradeoff most generators make.
            file(GLOB _engineShaderIncludes "${includeDirectory}/*.glsl")
            list(APPEND shaderIncludeDepends ${_engineShaderIncludes})
        endforeach()
        add_custom_command(
            OUTPUT "${shaderOutput}"
            COMMAND ${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${ENGINE_SHADER_LOADER_PATH}"
                    "${ENGINE_GLSLANG_VALIDATOR}" -V ${includeArguments}
                    "${shaderSourcePath}"
                    -o "${shaderOutput}"
            DEPENDS "${shaderSourcePath}" ${shaderIncludeDepends}
            COMMENT "compiling shader ${shaderSource} for ${ARG_TARGET}"
            VERBATIM)
        list(APPEND ENGINE_SHADER_OUTPUTS "${shaderOutput}")
    endforeach()

    # The shader steps ride the owning module's binary, not the global ALL
    # target: a module excluded from the build (ENGINE_BUILD_MODULE_<NAME>=OFF)
    # compiles no shaders, and an in-tree consumer that pulls an excluded
    # module's implementation (the lighting techniques link the renderer)
    # brings the shaders along with it.
    add_custom_target(engine_shaders_${ARG_TARGET} DEPENDS ${ENGINE_SHADER_OUTPUTS})
    add_dependencies(${ARG_TARGET} engine_shaders_${ARG_TARGET})
endfunction()
