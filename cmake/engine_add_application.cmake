include_guard(GLOBAL)

# Create an application executable using the engine runtime layout.
#
# engine_add_application(<target>
#     SOURCES <source> [...]
#     CAPABILITIES <module-name> [...]
#     [LIBRARIES <target-or-library> [...]]
#     [INCLUDE_DIRECTORIES <directory> [...]]
#     [COMPILE_OPTIONS <option> [...]]
#     [COMPILE_DEFINITIONS <definition> [...]]
#     [LINK_OPTIONS <option> [...]]
#     [CONFIG_DIRECTORY <directory>]
#     [ASSET_DIRECTORY <directory>]
#     [MOD_DIRECTORY <directory>]
#     [RUNTIME_DIRECTORY <directory>]
#     [OUTPUT_NAME <name>]
#     [WINDOWED]
# )
#
# CAPABILITIES links only the public capability headers. Runtime modules remain
# dynamically loaded by EngineKernel and are selected by the application's
# requirements at startup.
function(engine_add_application target)
	cmake_parse_arguments(PARSE_ARGV
		1
		APP
		"WINDOWED"
		"CONFIG_DIRECTORY;ASSET_DIRECTORY;MOD_DIRECTORY;RUNTIME_DIRECTORY;OUTPUT_NAME"
		"SOURCES;CAPABILITIES;LIBRARIES;INCLUDE_DIRECTORIES;COMPILE_OPTIONS;COMPILE_DEFINITIONS;LINK_OPTIONS"
	)

	if(APP_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR
			"engine_add_application(${target}): unparsed arguments: ${APP_UNPARSED_ARGUMENTS}"
		)
	endif()

	if(NOT APP_SOURCES)
		message(FATAL_ERROR "engine_add_application(${target}): SOURCES is required")
	endif()

	if(NOT TARGET engine_application)
		message(FATAL_ERROR
			"engine_add_application(${target}): add the engine project before this call"
		)
	endif()

	if(TARGET "${target}")
		message(FATAL_ERROR
			"engine_add_application(${target}): target already exists"
		)
	endif()

	if(APP_WINDOWED)
		add_executable(${target} WIN32 ${APP_SOURCES})
	else()
		add_executable(${target} ${APP_SOURCES})
	endif()

	target_compile_features(${target} PRIVATE cxx_std_20)
	target_link_libraries(${target} PRIVATE engine_application ${APP_LIBRARIES})

	if(APP_INCLUDE_DIRECTORIES)
		target_include_directories(${target} PRIVATE ${APP_INCLUDE_DIRECTORIES})
	endif()

	if(APP_COMPILE_OPTIONS)
		target_compile_options(${target} PRIVATE ${APP_COMPILE_OPTIONS})
	endif()

	if(APP_COMPILE_DEFINITIONS)
		target_compile_definitions(${target} PRIVATE ${APP_COMPILE_DEFINITIONS})
	endif()

	if(APP_LINK_OPTIONS)
		target_link_options(${target} PRIVATE ${APP_LINK_OPTIONS})
	endif()

	if(APP_OUTPUT_NAME)
		set_target_properties(${target} PROPERTIES OUTPUT_NAME "${APP_OUTPUT_NAME}")
	endif()

	foreach(capability IN LISTS APP_CAPABILITIES)
		if(NOT capability MATCHES "^[a-z][a-z0-9_]*$")
			message(FATAL_ERROR
				"engine_add_application(${target}): invalid capability module name '${capability}'"
			)
		endif()

		set(capability_target "engine_${capability}_capabilities")

		if(NOT TARGET "${capability_target}")
			message(FATAL_ERROR
				"engine_add_application(${target}): capability headers '${capability}' are unavailable; "
				"check that the '${capability}' module is enabled and built"
			)
		endif()

		target_link_libraries(${target} PRIVATE "${capability_target}")
	endforeach()

	# Give each application an isolated runtime root. This allows several
	# applications to share one engine build without sharing their config,
	# assets, or mod packages.
	if(NOT APP_RUNTIME_DIRECTORY)
		set(APP_RUNTIME_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/${target}")
	endif()

	get_filename_component(APP_RUNTIME_DIRECTORY
		"${APP_RUNTIME_DIRECTORY}"
		ABSOLUTE
		BASE_DIR
		"${CMAKE_BINARY_DIR}"
	)

	set_target_properties(${target}
		PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${APP_RUNTIME_DIRECTORY}"
	)

	if(APPLE)
		set_target_properties(${target} PROPERTIES BUILD_RPATH "@loader_path")
	elseif(UNIX)
		set_target_properties(${target} PROPERTIES BUILD_RPATH "$ORIGIN")
	endif()

	if(NOT APP_CONFIG_DIRECTORY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
		set(APP_CONFIG_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
	endif()

	if(NOT APP_ASSET_DIRECTORY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets")
		set(APP_ASSET_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets")
	endif()

	if(NOT APP_MOD_DIRECTORY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mods")
		set(APP_MOD_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/mods")
	endif()

	if(APP_CONFIG_DIRECTORY)
		get_filename_component(APP_CONFIG_DIRECTORY
			"${APP_CONFIG_DIRECTORY}"
			ABSOLUTE
			BASE_DIR
			"${CMAKE_CURRENT_SOURCE_DIR}"
		)
	endif()

	if(APP_ASSET_DIRECTORY)
		get_filename_component(APP_ASSET_DIRECTORY
			"${APP_ASSET_DIRECTORY}"
			ABSOLUTE
			BASE_DIR
			"${CMAKE_CURRENT_SOURCE_DIR}"
		)
	endif()

	if(APP_MOD_DIRECTORY)
		get_filename_component(APP_MOD_DIRECTORY
			"${APP_MOD_DIRECTORY}"
			ABSOLUTE
			BASE_DIR
			"${CMAKE_CURRENT_SOURCE_DIR}"
		)
	endif()

	set(runtime_content_commands
		COMMAND
		${CMAKE_COMMAND}
		-E
		make_directory
		"${APP_RUNTIME_DIRECTORY}"
		COMMAND
		${CMAKE_COMMAND}
		-E
		copy_directory
		"${CMAKE_BINARY_DIR}/runtime/modules"
		"${APP_RUNTIME_DIRECTORY}/modules"
		COMMAND
		${CMAKE_COMMAND}
		-E
		copy_directory
		"${CMAKE_BINARY_DIR}/runtime/config"
		"${APP_RUNTIME_DIRECTORY}/config"
		COMMAND
		${CMAKE_COMMAND}
		-E
		copy_directory
		"${CMAKE_BINARY_DIR}/runtime/mods"
		"${APP_RUNTIME_DIRECTORY}/mods"
		COMMAND
		${CMAKE_COMMAND}
		-E
		copy_if_different
		"$<TARGET_FILE:engine_kernel>"
		"${APP_RUNTIME_DIRECTORY}"
	)

	if(APP_CONFIG_DIRECTORY)
		if(NOT IS_DIRECTORY "${APP_CONFIG_DIRECTORY}")
			message(FATAL_ERROR
				"engine_add_application(${target}): CONFIG_DIRECTORY does not exist: ${APP_CONFIG_DIRECTORY}"
			)
		endif()
		list(APPEND runtime_content_commands
			COMMAND
			${CMAKE_COMMAND}
			-E
			make_directory
			"${APP_RUNTIME_DIRECTORY}/config"
			COMMAND
			${CMAKE_COMMAND}
			-E
			copy_directory
			"${APP_CONFIG_DIRECTORY}"
			"${APP_RUNTIME_DIRECTORY}/config"
		)
	endif()

	if(APP_ASSET_DIRECTORY)
		if(NOT IS_DIRECTORY "${APP_ASSET_DIRECTORY}")
			message(FATAL_ERROR
				"engine_add_application(${target}): ASSET_DIRECTORY does not exist: ${APP_ASSET_DIRECTORY}"
			)
		endif()
		list(APPEND runtime_content_commands
			COMMAND
			${CMAKE_COMMAND}
			-E
			make_directory
			"${APP_RUNTIME_DIRECTORY}/assets"
			COMMAND
			${CMAKE_COMMAND}
			-E
			copy_directory
			"${APP_ASSET_DIRECTORY}"
			"${APP_RUNTIME_DIRECTORY}/assets"
		)
	endif()

	if(APP_MOD_DIRECTORY)
		if(NOT IS_DIRECTORY "${APP_MOD_DIRECTORY}")
			message(FATAL_ERROR
				"engine_add_application(${target}): MOD_DIRECTORY does not exist: ${APP_MOD_DIRECTORY}"
			)
		endif()
		list(APPEND runtime_content_commands
			COMMAND
			${CMAKE_COMMAND}
			-E
			make_directory
			"${APP_RUNTIME_DIRECTORY}/mods"
			COMMAND
			${CMAKE_COMMAND}
			-E
			copy_directory
			"${APP_MOD_DIRECTORY}"
			"${APP_RUNTIME_DIRECTORY}/mods"
		)
	endif()

	add_custom_target(${target}_runtime_content ${runtime_content_commands})

	if(TARGET engine_runtime_layout)
		add_dependencies(${target}_runtime_content engine_runtime_layout)
	endif()

	add_dependencies(${target}_runtime_content engine_kernel)
	add_dependencies(${target} ${target}_runtime_content)
endfunction()
