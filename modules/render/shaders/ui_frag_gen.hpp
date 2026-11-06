// Committed fallback SPIR-V for shaders/ui.frag, one word per line.
//
// Placeholder fallback for toolchains without a shader compiler: this copy
// is empty, so the overlay pipeline cannot be built from it and Vulkan
// shader-module creation fails unless a real shader compiler regenerates the
// header in the build tree. See engine_module_compile_shaders.cmake.
