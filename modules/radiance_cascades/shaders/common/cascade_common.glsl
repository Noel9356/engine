// Math shared by the radiance cascades march and gather stages.
//
// The anchor classification and the direction table live here because the two
// stages previously carried byte-identical copies with a "MUST keep the same
// formula" comment: a mismatch would desynchronize the line-of-sight check
// from the field the probe actually carries. One definition removes the
// failure mode instead of policing it.

#ifndef ENGINE_CASCADE_COMMON_GLSL
#define ENGINE_CASCADE_COMMON_GLSL

// Direction of one probe ray: equirectangular over (azCount x elCount), cell
// centres offset by half a step. CONSTANT angular schedule across cascades
// (every cascade carries the same direction count; the spatial grid refines
// instead).
vec3 cascadeDirection(uint azIndex, uint elIndex, uint azCount, uint elCount)
{
    const float azimuth = (float(azIndex) + 0.5) / float(azCount) * 6.283185307179586;
    const float elevation = (float(elIndex) + 0.5) / float(elCount) * 3.141592653589793 - 1.5707963267948966;
    return vec3(cos(elevation) * cos(azimuth), sin(elevation), cos(elevation) * sin(azimuth));
}

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
