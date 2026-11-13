include(engine_fetch_dependency)

engine_fetch_dependency(
    glm
    OWNER
    character
    GIT_REPOSITORY
    https://github.com/g-truc/glm.git
    GIT_TAG
    1.0.1
    HEADER_ONLY
)

engine_fetch_dependency(
    tomlplusplus
    OWNER
    character
    GIT_REPOSITORY
    https://github.com/marzer/tomlplusplus.git
    GIT_TAG
    v3.4.0
    HEADER_ONLY
    INCLUDE_SUBDIRS
    include
)

add_library(
    engine_character_capabilities
    INTERFACE
)
target_include_directories(
    engine_character_capabilities
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(
    engine_character_capabilities
    INTERFACE engine_dep_glm
)

add_library(
    engine_module_character
    MODULE
    source/character.cpp
    source/configuration.cpp
    source/evaluation.cpp
    source/lua_bridge.cpp
    source/registry.cpp
    source/serialization.cpp
    source/validation.cpp
)

target_compile_definitions(
    engine_module_character
    PRIVATE ENGINE_CHARACTER_MODULE_VERSION="0.1.0"
)

target_include_directories(
    engine_module_character
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../script_runtime/include"
)

target_link_libraries(
    engine_module_character
    PRIVATE engine_kernel
            engine_character_capabilities
            engine_dep_glm
            engine_dep_tomlplusplus
)

set_target_properties(
    engine_module_character
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/character"
               RUNTIME_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/character"
               ARCHIVE_OUTPUT_DIRECTORY
               "${CMAKE_BINARY_DIR}/runtime/modules/character"
)

add_custom_command(
    TARGET engine_module_character
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/module.toml"
        "$<TARGET_FILE_DIR:engine_module_character>/module.toml"
    COMMENT "placing the character module descriptor next to its library"
)
