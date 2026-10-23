# The portability guard for published capability headers.
#
# Every module publishes its capability vocabulary from its own directory
# (window_capability.hpp and friends). Those headers are the mod-facing
# surface: native mods include them, and script mods must stay able to run
# against ANY implementation of the same capability family. This configure
# time check keeps that promise mechanical: a published header that starts
# including a graphics, windowing, or platform header fails the configure
# with a precise message instead of silently coupling mods to one
# implementation.

function(engine_guard_capability_header capabilityHeader)
    if(NOT EXISTS "${capabilityHeader}")
        message(FATAL_ERROR
                "engine_guard_capability_header: '${capabilityHeader}' does not exist. The published "
                "capability header must live next to the module that provides the capability.")
    endif()
    file(READ "${capabilityHeader}" capabilityHeaderContent)
    if(capabilityHeaderContent MATCHES "#[ \t]*include[ \t]*[<\"](vulkan/|GLFW/|GL/|X11/|windows\\.h)")
        message(FATAL_ERROR
                "published capability header '${capabilityHeader}' includes an implementation, platform or "
                "vendor specific header. Published capability headers must stay implementation-, platform- "
                "and vendor-agnostic; implementations live in the module behind opaque handles.")
    endif()
endfunction()
