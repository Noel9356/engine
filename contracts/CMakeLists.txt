# The capability contracts: the shared vocabulary between modules and mods.
# This target is deliberately OUTSIDE the kernel tree: the kernel mediates
# capability demand and enforces permissions without knowing what any
# capability means, so the contracts cannot live under kernel/. Consumers
# (modules, native mods and the host) link this static library; each binary
# carries its own copy of the one compiled contract TU (the input key-name
# table), which keeps every plugin's symbols self-contained.
#
# The kernel does NOT link this target - by construction, engine code cannot
# reference a capability definition.

add_library(engine_contracts STATIC
    source/input_contract_names.cpp
)

target_include_directories(engine_contracts
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# ---- Contract purity guard -------------------------------------------------------
# Contracts are the mod-facing surface: implementation-, platform- and
# vendor-agnostic. This configure-time check keeps that promise mechanical: a
# contract that starts including a graphics, windowing or platform header fails
# the configure with a precise message instead of silently coupling mods to one
# renderer implementation.
file(GLOB ENGINE_CONTRACT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/engine/capabilities/*.hpp")
foreach(engineContractHeader ${ENGINE_CONTRACT_HEADERS})
    file(READ "${engineContractHeader}" engineContractContent)
    if(engineContractContent MATCHES "#[ \t]*include[ \t]*[<\"](vulkan/|GLFW/|GL/|X11/|windows\\.h)")
        message(FATAL_ERROR
                "capability contract '${engineContractHeader}' includes an implementation, platform or vendor "
                "specific header. Contracts must stay implementation-, platform- and vendor-agnostic; "
                "implementations live in modules behind opaque handles.")
    endif()
endforeach()
