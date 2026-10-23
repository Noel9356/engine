add_executable(engine
    main.cpp
    leak_check.cpp
)

# The host links only the kernel and the capability contracts: every module
# arrives at runtime as a plugin loaded from <exe_dir>/modules (the kernel's
# loadModulePlugins phase), so the set of available modules is a property of
# the deployment, not of the link line. The ENGINE_BUILD_MODULE_* options keep
# controlling which plugin binaries the build produces; a demanded capability
# whose provider is missing from the modules directory fails startup with a
# precise diagnostic.
target_link_libraries(engine
    PRIVATE engine_kernel
    PRIVATE engine_contracts
)
