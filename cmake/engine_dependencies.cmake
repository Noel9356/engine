include(FetchContent)

if(POLICY CMP0169)
	cmake_policy(SET CMP0169 OLD)
endif()

set(
	ENGINE_TOMLPLUSPLUS_REPOSITORY
	"https://github.com/marzer/tomlplusplus.git"
	CACHE
		STRING
		"tomlplusplus repository used by the engine"
)
set(
	ENGINE_TOMLPLUSPLUS_TAG
	"v3.4.0"
	CACHE
		STRING
		"tomlplusplus revision used by the engine"
)

set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)

FetchContent_Declare(
	tomlplusplus
	GIT_REPOSITORY
	"${ENGINE_TOMLPLUSPLUS_REPOSITORY}"
	GIT_TAG
	"${ENGINE_TOMLPLUSPLUS_TAG}"
	GIT_SHALLOW
	TRUE
)

FetchContent_MakeAvailable(tomlplusplus)

if(NOT TARGET engine_tomlplusplus_bindings)
	add_library(engine_tomlplusplus_bindings INTERFACE)
	target_include_directories(engine_tomlplusplus_bindings
		INTERFACE
			"${tomlplusplus_SOURCE_DIR}/include"
			"${tomlplusplus_SOURCE_DIR}/include/toml++"
	)
endif()

set(
	ENGINE_LUA_REPOSITORY
	"https://github.com/lua/lua.git"
	CACHE
		STRING
		"Lua repository used by the engine"
)
set(
	ENGINE_LUA_TAG
	"v5.4.7"
	CACHE
		STRING
		"Lua revision used by the engine"
)

FetchContent_Declare(
	lua
	GIT_REPOSITORY
	"${ENGINE_LUA_REPOSITORY}"
	GIT_TAG
	"${ENGINE_LUA_TAG}"
	GIT_SHALLOW
	TRUE
)

FetchContent_GetProperties(lua)
if(NOT lua_POPULATED)
	FetchContent_Populate(lua)
endif()

if(NOT TARGET engine_lua_runtime)
	add_library(engine_lua_runtime STATIC "${lua_SOURCE_DIR}/onelua.c")
	target_include_directories(engine_lua_runtime PUBLIC "${lua_SOURCE_DIR}")
	target_compile_definitions(engine_lua_runtime PRIVATE MAKE_LIB)
endif()
