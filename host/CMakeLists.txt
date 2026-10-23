add_executable(engine
    main.cpp
    leak_check.cpp
)

# The host links only the kernel: every module arrives at runtime as a plugin
# loaded from <exe_dir>/modules (kernel/source/module_plugins.cpp), so the set
# of available modules is a property of the deployment, not of the link line.
# The ENGINE_BUILD_MODULE_* options keep controlling which plugin binaries the
# build produces; the kernel reports a module listed in the configuration but
# missing from the modules directory through its startup diagnostic.
target_link_libraries(engine
    PRIVATE engine_kernel
)
