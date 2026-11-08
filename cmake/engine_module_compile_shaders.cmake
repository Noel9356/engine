# engine_module_compile_shaders
#
# Automatic shader compilation for one module. The module lists its GLSL
# sources (which live in the module's own shaders/ directory) and the build
# compiles them to SPIR-V at build time and embeds the words into generated
# headers - no committed shader artifacts, no manual regeneration step, and
# no shader code paths outside the module.
#
#     engine_module_compile_shaders(TARGET engine_module_renderer
#         NAMESPACE engine renderer
#         SHADERS mesh.vert mesh.frag
#     )
#
# For each 'shaders/<name>.<stage>' the helper generates, into the build
# tree, 'shaders/<name>_<stage>_gen.hpp' defining
#
#     inline constexpr std::uint32_t <stem><Stage>Spirv[] = { ... };
#
# (mesh.vert -> meshVertexSpirv; stage spellings Vert/Fragment/Compute/
# Geometry/Tessellation*), inside the given C++ namespace. The header is
# attached to the target and its directory joins the target's private
# include path, so sources write #include "mesh_vert_gen.hpp".
#
# Toolchain: glslangValidator is preferred, glslc is the fallback (both are
# probed in $VULKAN_SDK/bin as well as PATH). When neither exists, the
# helper falls back to a committed copy of the generated header kept next to
# the GLSL source (shaders/<name>.<stage>.gen.hpp); that copy is only read
# by toolchains without a shader compiler. No committed copy and no compiler
# is a configuration error - the backend would otherwise run with a stale or
# missing shader stage.

# The directory of this file, for the custom commands below.
set(ENGINE_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(engine_module_compile_shaders)
	cmake_parse_arguments(arg "" "TARGET" "NAMESPACE;SHADERS" ${ARGN})

	if(NOT arg_TARGET)
		message(FATAL_ERROR "engine_module_compile_shaders: TARGET is required")
	endif()
	if(NOT arg_SHADERS)
		message(FATAL_ERROR "engine_module_compile_shaders: SHADERS is required")
	endif()
	if(NOT arg_NAMESPACE)
		message(FATAL_ERROR "engine_module_compile_shaders: NAMESPACE is required")
	endif()

	set(shaderSourceDir "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
	set(shaderBinaryDir "${CMAKE_CURRENT_BINARY_DIR}/shaders")

	find_program(ENGINE_GLSLANG_VALIDATOR_PROGRAM
		glslangValidator
		HINTS
		"$ENV{VULKAN_SDK}/bin"
		"$ENV{VULKAN_SDK}/Bin"
	)
	find_program(ENGINE_GLSLC_PROGRAM
		glslc
		HINTS
		"$ENV{VULKAN_SDK}/bin"
		"$ENV{VULKAN_SDK}/Bin"
	)

	set(shaderCompiler "")
	set(shaderCompilerKind "")
	if(ENGINE_GLSLANG_VALIDATOR_PROGRAM)
		set(shaderCompiler "${ENGINE_GLSLANG_VALIDATOR_PROGRAM}")
		set(shaderCompilerKind "glslangValidator")
	elseif(ENGINE_GLSLC_PROGRAM)
		set(shaderCompiler "${ENGINE_GLSLC_PROGRAM}")
		set(shaderCompilerKind "glslc")
	endif()

	set(namespaceName)
	foreach(namespaceComponent IN LISTS arg_NAMESPACE)
		list(APPEND namespaceName "${namespaceComponent}")
	endforeach()

	foreach(shaderFile IN LISTS arg_SHADERS)
		set(shaderPath "${shaderSourceDir}/${shaderFile}")
		if(NOT EXISTS "${shaderPath}")
			message(FATAL_ERROR
				"engine_module_compile_shaders: shader '${shaderFile}' not found in ${shaderSourceDir}"
			)
		endif()

		# <stem>.<stage> -> <stem><Stage>Spirv (mesh.vert -> meshVertexSpirv).
		string(REGEX
			MATCH
			"^([A-Za-z0-9_]+)\\.([A-Za-z0-9_]+)$"
			shaderMatch
			"${shaderFile}"
		)
		if(NOT shaderMatch)
			message(FATAL_ERROR
				"engine_module_compile_shaders: shader file '${shaderFile}' must be named <stem>.<stage>"
			)
		endif()
		set(shaderStem "${CMAKE_MATCH_1}")
		set(shaderStage "${CMAKE_MATCH_2}")
		if(shaderStage STREQUAL "vert")
			set(stageWord "Vertex")
		elseif(shaderStage STREQUAL "frag")
			set(stageWord "Fragment")
		elseif(shaderStage STREQUAL "comp")
			set(stageWord "Compute")
		elseif(shaderStage STREQUAL "geom")
			set(stageWord "Geometry")
		elseif(shaderStage STREQUAL "tesc")
			set(stageWord "TessellationControl")
		elseif(shaderStage STREQUAL "tese")
			set(stageWord "TessellationEvaluation")
		else()
			message(FATAL_ERROR
				"engine_module_compile_shaders: unknown shader stage '${shaderStage}' in '${shaderFile}'"
			)
		endif()
		set(symbolName "${shaderStem}${stageWord}Spirv")

		set(generatedHeader "${shaderBinaryDir}/${shaderStem}_${shaderStage}_gen.hpp")

		if(shaderCompiler)
			set(compiledSpirv "${shaderBinaryDir}/${shaderFile}.spv")
			if(shaderCompilerKind STREQUAL "glslangValidator")
				set(compileCommand
					COMMAND
					"${shaderCompiler}"
					-V
					-o
					"${compiledSpirv}"
					"${shaderPath}"
				)
			else()
				set(compileCommand
					COMMAND
					"${shaderCompiler}"
					-o
					"${compiledSpirv}"
					"${shaderPath}"
				)
			endif()
			add_custom_command(
				OUTPUT
					"${compiledSpirv}"
					"${generatedHeader}"
				COMMAND "${CMAKE_COMMAND}" -E make_directory "${shaderBinaryDir}"
					${compileCommand}
				COMMAND "${CMAKE_COMMAND}" "-DSPIRV_FILE=${compiledSpirv}"
					"-DHEADER_FILE=${generatedHeader}" "-DSYMBOL_NAME=${symbolName}"
					"-DNAMESPACE=${namespaceName}" "-DSOURCE_SHADER=${shaderFile}" -P
					"${ENGINE_CMAKE_DIR}/engine_spirv_embed.cmake"
				DEPENDS
					"${shaderPath}"
					"${ENGINE_CMAKE_DIR}/engine_spirv_embed.cmake"
				VERBATIM
				COMMENT "compiling ${arg_TARGET} shader ${shaderFile} to SPIR-V (${shaderCompilerKind})"
			)
		else()
			set(committedHeader
				"${shaderSourceDir}/${shaderStem}_${shaderStage}_gen.hpp"
			)
			if(NOT EXISTS "${committedHeader}")
				message(FATAL_ERROR
					"engine_module_compile_shaders: no shader compiler (glslangValidator/glslc) on this "
					"machine and no committed SPIR-V header for '${shaderFile}' "
					"(expected ${committedHeader}). Install a Vulkan SDK, or commit the generated "
					"header so toolchains without a shader compiler can build the module."
				)
			endif()
			if("${shaderPath}" IS_NEWER_THAN "${committedHeader}")
				message(FATAL_ERROR
					"engine_module_compile_shaders: the committed SPIR-V header '${committedHeader}' "
					"is older than '${shaderPath}'. Install glslangValidator/glslc to regenerate it, "
					"then update the committed header before building without a shader compiler."
				)
			endif()
			message(STATUS
				"${arg_TARGET}: no shader compiler found; using the committed SPIR-V of ${shaderFile}"
			)
			add_custom_command(
				OUTPUT "${generatedHeader}"
				COMMAND "${CMAKE_COMMAND}" -E make_directory "${shaderBinaryDir}"
				COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${committedHeader}"
					"${generatedHeader}"
				DEPENDS "${committedHeader}"
				VERBATIM
				COMMENT "using the committed SPIR-V header of ${shaderFile} (no shader compiler on this machine)"
			)
		endif()

		target_sources("${arg_TARGET}" PRIVATE "${generatedHeader}")
		set_source_files_properties("${generatedHeader}" PROPERTIES GENERATED TRUE)
	endforeach()

	target_include_directories("${arg_TARGET}" PRIVATE "${shaderBinaryDir}")

	if(shaderCompiler)
		message(STATUS
			"${arg_TARGET}: shaders compile at build time with ${shaderCompilerKind}"
		)
	endif()
endfunction()
