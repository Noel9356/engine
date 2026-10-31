set(ENGINE_CAPABILITY_HEADER_GUARD_SCRIPT
	"${CMAKE_CURRENT_LIST_DIR}/capability_header_guard_script.cmake"
)

function(engine_capability_header_guard capabilityTarget)
	if(NOT TARGET "${capabilityTarget}")
		message(FATAL_ERROR
			"engine_capability_header_guard: '${capabilityTarget}' is not a target"
		)
	endif()

	set(guardSources)
	foreach(headerPath IN LISTS ARGN)
		list(APPEND guardSources "${headerPath}")
	endforeach()

	if(NOT guardSources)
		set(capabilityIncludeRoot
			"${CMAKE_CURRENT_SOURCE_DIR}/include/engine/capabilities"
		)
		if(IS_DIRECTORY "${capabilityIncludeRoot}")
			file(GLOB guardSources "${capabilityIncludeRoot}/*.hpp")
		endif()
	endif()

	if(NOT guardSources)
		message(FATAL_ERROR
			"engine_capability_header_guard: no capability headers found for '${capabilityTarget}' "
			"(expected headers under ${CMAKE_CURRENT_SOURCE_DIR}/include/engine/capabilities)"
		)
	endif()

	set(guardTarget "capability_header_guard_${capabilityTarget}")
	if(TARGET "${guardTarget}")
		message(FATAL_ERROR
			"engine_capability_header_guard: '${capabilityTarget}' already has a capability header guard"
		)
	endif()

	add_custom_target("${guardTarget}"
		COMMAND "${CMAKE_COMMAND}" "-DCAPABILITY_TARGET_NAME=${capabilityTarget}"
			"-DCAPABILITY_HEADERS=${guardSources}" -P
			"${ENGINE_CAPABILITY_HEADER_GUARD_SCRIPT}"
		VERBATIM
		COMMENT "verifying capability headers of ${capabilityTarget}"
	)
endfunction()
