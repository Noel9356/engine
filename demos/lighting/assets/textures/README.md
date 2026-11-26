Surface scans for the lighting demo (the demo loads them from this directory; a missing file degrades the ground to vertex color with one warning). The scans come from Poly Haven (https://polyhaven.com), published under the CC0 1.0 Universal license (https://polyhaven.com/license). The unmodified downloads ship beside the processed outputs, and scripts/process_lighting_textures.py regenerates the outputs from them deterministically.

ground_gravel_diff_1k.jpg
    Source asset "gravel floor" (https://polyhaven.com/a/gravel_floor), 1k diffuse map, processed for the renderer's linear-space multiply: desaturated toward its stored-space luminance, decoded to linear values, scaled so the mean luminance is exactly 1.0, and re-encoded as JPEG at quality 92 (the same pipeline the planet demo's ground set documents). The renderer's per-texture mean compensation (RenderCapability::textureMean) keeps the multiply brightness-neutral, so the ground's tone comes from the material base color and the scan contributes its luminance structure only.
ground_concrete_diff_1k.jpg
    Source asset "concrete floor 02" (https://polyhaven.com/a/concrete_floor_02), 1k diffuse map, processed with the same pipeline; the alternate ground variant (the tile grid alternates the two scans so neighboring tiles photograph different detail).
ground_gravel_nor_gl_1k.jpg
    Source asset "gravel floor" (https://polyhaven.com/a/gravel_floor), 1k OpenGL-convention normal map, passed through unmodified apart from the re-encode; the ground's per-sample relief under the sun and the point lights.
