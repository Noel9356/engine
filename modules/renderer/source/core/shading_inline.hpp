// Hot-path marker for the per-pixel shading entries: an outlined call per
// covered pixel measurably regresses the frame time (and blocks hoisting
// loop-invariant terms out of the rasterizer's scan loops), so the shading
// kernels force inlining on compilers that support it.

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#define ENGINE_SHADING_INLINE inline __attribute__((always_inline))
#else
#define ENGINE_SHADING_INLINE inline
#endif
