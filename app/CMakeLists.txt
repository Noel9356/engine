add_executable(engine
    main.cpp
    app.cpp
)

# The app links only the kernel plus the published capability headers of the
# modules it acquires from: every module implementation arrives at runtime as
# a plugin loaded from <exe_dir>/modules (the kernel's loadModulePlugins
# phase), so the set of available modules is a property of the deployment, not
# of the link line. The capability targets below are INTERFACE (header path
# only) - the app compiles against the providers' published types and stays
# free of their binaries. The ENGINE_BUILD_MODULE_* options keep controlling
# which plugin binaries the build produces; a demanded capability whose
# provider is missing from the modules directory fails startup with a precise
# diagnostic.
target_link_libraries(engine
    PRIVATE engine_kernel
    PRIVATE engine_capability_window
    PRIVATE engine_capability_input
    PRIVATE engine_capability_renderer
    PRIVATE engine_capability_screenshot
)
