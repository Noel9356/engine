# engine_fetch_dependency
#
# The engine's third-party dependency helper. Each consumer (the kernel or a
# module) declares the dependencies it needs inside its own CMakeLists.txt -
# adding a dependency to a module never touches files outside that module.
#
#     engine_fetch_dependency(glm
#         OWNER render
#         GIT_REPOSITORY https://github.com/g-truc/glm.git
#         GIT_TAG 1.0.1
#         HEADER_ONLY
#     )
#
# After the call:
#  - ${glm_SOURCE_DIR} / ${glm_BINARY_DIR} name the populated sources
#    (FETCHCONTENT_SOURCE_DIR_<NAME> / FETCHCONTENT_SOURCE_DIR overrides in
#    the cache still win, as with plain FetchContent);
#  - with HEADER_ONLY, an INTERFACE target engine_dep_<name> is created (once)
#    carrying the include directories (${name}_SOURCE_DIR plus every
#    INCLUDE_SUBDIRS entry) as system includes, keeping third-party warnings
#    out of engine diagnostics, so consumers write
#    target_link_libraries(... PRIVATE engine_dep_glm).
#
# Coordination between modules is automatic:
#  - the first declaration wins and populates the sources;
#  - a second declaration of the same name must repeat the same repository
#    and revision, or configuration stops with an error naming both pins (two
#    modules silently linking different revisions of one library would be a
#    build-order lottery);
#  - either pin can be overridden from the cache without editing module
#    files: ENGINE_DEP_<NAME>_REPOSITORY / ENGINE_DEP_<NAME>_TAG.
#
# MAKE_AVAILABLE instead runs FetchContent_MakeAvailable for dependencies
# that ship their own CMake package (GLFW), exposing their real targets.

if(POLICY CMP0169)
	# FetchContent_Populate is deliberately used here (the engine populates
	# header-only sources without running their CMake); quiet the
	# deprecation warning until the helper moves to the FindPython-free
	# FetchContent API.
	cmake_policy(SET CMP0169 OLD)
endif()

include(FetchContent)

function(engine_fetch_dependency name)
	cmake_parse_arguments(arg
		"HEADER_ONLY;MAKE_AVAILABLE"
		"GIT_REPOSITORY;GIT_TAG;URL;URL_HASH;OWNER"
		"INCLUDE_SUBDIRS"
		${ARGN}
	)

	set(useGit FALSE)
	set(useUrl FALSE)

	if(arg_GIT_REPOSITORY OR arg_GIT_TAG)
		if(NOT arg_GIT_REPOSITORY OR NOT arg_GIT_TAG OR arg_URL OR arg_URL_HASH)
			message(FATAL_ERROR
				"engine_fetch_dependency(${name}): specify either GIT_REPOSITORY + GIT_TAG or URL + URL_HASH"
			)
		endif()

		set(useGit TRUE)
		set(repository "${arg_GIT_REPOSITORY}")
		set(tag "${arg_GIT_TAG}")

		# Cache overrides keep Git module declarations configurable.
		if(DEFINED
			CACHE{ENGINE_DEP_${name}_REPOSITORY}
			AND
			NOT
			ENGINE_DEP_${name}_REPOSITORY
			STREQUAL
			""
		)
			set(repository "${ENGINE_DEP_${name}_REPOSITORY}")
		endif()

		if(DEFINED
			CACHE{ENGINE_DEP_${name}_TAG}
			AND
			NOT
			ENGINE_DEP_${name}_TAG
			STREQUAL
			""
		)
			set(tag "${ENGINE_DEP_${name}_TAG}")
		endif()

	elseif(arg_URL OR arg_URL_HASH)
		if(NOT arg_URL OR NOT arg_URL_HASH)
			message(FATAL_ERROR
				"engine_fetch_dependency(${name}): URL and URL_HASH must be specified together"
			)
		endif()

		set(useUrl TRUE)
		set(repository "URL:${arg_URL}")
		set(tag "URL_HASH:${arg_URL_HASH}")
	else()
		message(FATAL_ERROR
			"engine_fetch_dependency(${name}): a Git repository/revision or a URL/SHA-256 pair is required"
		)
	endif()

	set(owner "${arg_OWNER}")

	if(owner STREQUAL "")
		set(owner "${PROJECT_NAME}")
	endif()

	# ---- dedupe across modules
	get_property(registeredPin GLOBAL PROPERTY "_ENGINE_DEPENDENCY_PIN_${name}")

	if(registeredPin)
		list(GET registeredPin 0 registeredRepository)
		list(GET registeredPin 1 registeredTag)
		get_property(registeredOwner GLOBAL
			PROPERTY "_ENGINE_DEPENDENCY_OWNER_${name}"
		)

		if(NOT)
			message(FATAL_ERROR
				"engine_fetch_dependency: dependency '${name}' is pinned twice with different revisions:\n"
				"  ${registeredOwner}: ${registeredRepository} (${registeredTag})\n"
				"  ${owner}: ${repository} (${tag})\n"
				"The modules must agree on one revision (or override both through the\n"
				"ENGINE_DEP_${name}_REPOSITORY / ENGINE_DEP_${name}_TAG cache entries)."
			)
		endif()

		message(STATUS
			"engine dependency '${name}': already populated (first declared by ${registeredOwner})"
		)

		# Re-publish the source/binary directory variables into the caller's
		# scope so the caller can reference them right after the call.
		get_property(publishedSourceDir GLOBAL
			PROPERTY "_ENGINE_DEPENDENCY_SOURCE_DIR_${name}"
		)

		if(publishedSourceDir)
			set("${name}_SOURCE_DIR" "${publishedSourceDir}" PARENT_SCOPE)
		endif()

		return()
	endif()

	if(useGit)
		# Git shallow clones only support branch and tag names reliably. An exact
		# commit hash may not be available as a shallow ref, so fetch its history
		# when a dependency is pinned to a full SHA-1 revision.
		string(LENGTH "${tag}" tagLength)
		set(gitOptions)

		if(NOT tag MATCHES "^[0-9a-fA-F]+$" OR NOT tagLength EQUAL 40)
			list(APPEND gitOptions GIT_SHALLOW TRUE)
		endif()

		FetchContent_Declare(
			"${name}"
			GIT_REPOSITORY
			"${repository}"
			GIT_TAG
			"${tag}"
			${gitOptions}
		)
	else()
		FetchContent_Declare(
			"${name}"
			URL
			"${arg_URL}"
			URL_HASH
			"${arg_URL_HASH}"
		)
	endif()

	if(arg_MAKE_AVAILABLE)
		FetchContent_MakeAvailable("${name}")
	else()
		FetchContent_GetProperties("${name}")
		if(NOT "${name}_POPULATED")
			FetchContent_Populate("${name}")
		endif()
	endif()

	set_property(
		GLOBAL
		PROPERTY
			"_ENGINE_DEPENDENCY_PIN_${name}"
			"${repository};${tag}"
	)
	set_property(GLOBAL PROPERTY "_ENGINE_DEPENDENCY_OWNER_${name}" "${owner}")
	if(DEFINED "${name}_SOURCE_DIR")
		set_property(
			GLOBAL
			PROPERTY
				"_ENGINE_DEPENDENCY_SOURCE_DIR_${name}"
				"${${name}_SOURCE_DIR}"
		)
	endif()

	if(arg_HEADER_ONLY)
		if(NOT TARGET "engine_dep_${name}")
			set(depIncludeDirs "${${name}_SOURCE_DIR}")
			foreach(includeSubdir IN LISTS arg_INCLUDE_SUBDIRS)

				list(APPEND depIncludeDirs "${${name}_SOURCE_DIR}/${includeSubdir}")
			endforeach()

			add_library("engine_dep_${name}" INTERFACE)
			target_include_directories("engine_dep_${name}" SYSTEM
				INTERFACE ${depIncludeDirs}
			)
		endif()
	endif()

	# Hand the populated directories back to the caller's scope.
	if(DEFINED "${name}_SOURCE_DIR")
		set("${name}_SOURCE_DIR" "${${name}_SOURCE_DIR}" PARENT_SCOPE)
	endif()

	if(DEFINED "${name}_BINARY_DIR")
		set("${name}_BINARY_DIR" "${${name}_BINARY_DIR}" PARENT_SCOPE)
	endif()

	message(STATUS
		"engine dependency '${name}': populated ${repository} at ${tag} (declared by ${owner})"
	)
endfunction()
