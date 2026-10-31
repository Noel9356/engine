set(report "")

foreach(headerPath IN LISTS CAPABILITY_HEADERS)
	if(NOT EXISTS "${headerPath}")
		list(APPEND report
			"capability header of ${CAPABILITY_TARGET_NAME} is missing: ${headerPath}"
		)
		continue()
	endif()

	file(READ "${headerPath}" headerText)

	if(headerText MATCHES "#include[ \t]*\"")
		list(APPEND report
			"capability header of ${CAPABILITY_TARGET_NAME} uses a quoted include, which resolves "
			"relative to the header file: ${headerPath} (capability headers are reached through "
			"include directories, for example <engine/capabilities/foo.hpp>, never through paths "
			"relative to another component's directory)"
		)
	endif()

	if(NOT headerText MATCHES "#ifndef[ \t]+ENGINE_CAPABILITIES_[A-Z0-9_]+_HPP")
		list(APPEND report
			"capability header of ${CAPABILITY_TARGET_NAME} has no include guard of the form "
			"#ifndef ENGINE_CAPABILITIES_<NAME>_HPP: ${headerPath}"
		)
	elseif(NOT headerText MATCHES "#define[ \t]+ENGINE_CAPABILITIES_")
		list(APPEND report
			"capability header of ${CAPABILITY_TARGET_NAME} declares #ifndef "
			"ENGINE_CAPABILITIES_<NAME>_HPP but never #defines it: ${headerPath}"
		)
	elseif(NOT headerText MATCHES "#endif")
		list(APPEND report
			"capability header of ${CAPABILITY_TARGET_NAME} opens an include guard but never closes "
			"it with #endif: ${headerPath}"
		)
	endif()
endforeach()

if(report)
	foreach(line IN LISTS report)
		message("[capability header guard] ${line}")
	endforeach()
	message(FATAL_ERROR
		"capability header verification failed for ${CAPABILITY_TARGET_NAME}"
	)
endif()
