// Math shared by the radiance cascades march and gather stages.
//
// Two things live here because both stages previously carried byte-identical
// copies that had to be kept in sync by comment alone (a mismatch would
// desynchronize the line-of-sight check from the field the probe actually
// carries). One definition removes the failure mode instead of policing it:
//
// - The precomputed direction table (see the buffer below): the equirect
//   schedule used to be a function here that every consumer re-evaluated with
//   4-5 transcendentals per direction - the gather inside a per-pixel
//   per-probe quadrature loop (~1000+ evaluations per pixel), the march once
//   per invocation, the compose once per coarsest-cascade texel. The schedule
//   is a pure function of the angular schedule, constant across cascades AND
//   frames, so it is now computed once per rebuild on the CPU
//   (RadianceCascadesPass::rebuild builds the identical equirectangular
//   schedule) and uploaded as this buffer.

#ifndef ENGINE_CASCADE_COMMON_GLSL
#define ENGINE_CASCADE_COMMON_GLSL

// Precomputed direction table, one vec4 per (azimuth x elevation) direction in
// the march's image-layer order (azimuth-major: index = azIndex * elCount +
// elIndex): xyz the unit direction, w the direction's equirectangular solid
// angle (2*pi/azCount) * (pi/elCount) * cos(elevation) - the exact quadrature
// weight the gather's probeIrradiance previously recomputed per pixel.
layout(std430, set = 0, binding = 7) readonly buffer CascadeDirectionTable
{
    vec4 cascadeDirections[];
};

// Anchor depth of one probe inside its cascade slab [tStart, tEnd], given the
// cell's depth-buffer statistics:
//
// - Pure-cell anchor: the slab start - mid-air, so its FORWARD rays cover the
//   slab without re-hitting the cell's own surface. A surface NEARER than the
//   slab start pulls the anchor just in front of itself (continuous, from the
//   cell centre's view depth) so the march never starts behind geometry.
// - Mixed-cell anchor: the cell's far surface, just in front of itself and
//   clamped into the slab (anchoring on the framing occluder would read the
//   whole cell dark - the automated leak check fails exactly that way).
// - The pure/mixed classification is a CONTINUOUS blend: a hard branch put
//   neighbouring cells on opposite anchors, their fields disagreed, and the
//   compose bilinear interpolated stair steps between them. The blend ramps
//   over 25% of the slab (floored at 30 cm for the near slabs).
float cascadeAnchorDepth(float tStart, float tEnd, float centerDepth, float nearDepth, float farDepth)
{
    const float slabLength = max(tEnd - tStart, 0.001);
    const float pureAnchor = clamp(min(tStart, centerDepth - 0.05), 0.0, tStart);
    const float mixedAnchor = min(max(farDepth - 0.05, 0.0), tEnd - 0.05);
    const float spreadThreshold = max(0.3, 0.25 * slabLength);
    const float mixedness =
        smoothstep(0.0, 1.0, clamp((farDepth - nearDepth - spreadThreshold) / max(0.25 * slabLength, 0.001), 0.0, 1.0));
    return mix(pureAnchor, mixedAnchor, mixedness);
}

#endif // ENGINE_CASCADE_COMMON_GLSL
