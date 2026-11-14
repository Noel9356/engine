# fur module

Real-time fur content and policy for furry characters, built on the renderer
module's shell-fur draw path (`renderer::FurLayerParams`, `drawFur`,
`drawFurInstances`, ABI 13). This module owns the fur-domain work - procedural
strand textures, parameter presets, and the screen-size LOD evaluator - while
the renderer module owns the generic rendering machinery (fur pipeline,
shaders, headless mirror). The module owns no GPU state.

## Technique

Shell texturing (Lengyel et al. 2001, "Real-Time Fur over Arbitrary
Surfaces", the shells half): a furry surface draws its opaque base mesh once,
then the SAME mesh `layerCount` more times, each layer's vertices displaced
outward along their normals by `(layer / layerCount) * furHeight` and alpha
blended over what is already in the framebuffer. Each layer crops a
procedural STRAND TEXTURE against the layer ratio, so the stacked layers
build up strand volume; the strand texture's alpha channel is the per-texel
strand height field (1 at a strand root, 0 at its tip) and its rgb carries
the strand albedo variation.

The strand textures are generated here, deterministically, by
`generateFurTexture`: guard hairs (coarse columns, per-strand lengths), an
undercoat (finer half-offset columns that keep the root layers dense and
fill the silhouette gaps the original paper addressed with fins), wispiness
noise, and per-strand/texel albedo variation. Consumers upload the pixels
with `renderer.createTexture` into a material's baseColorTexture slot.

Lighting lives in the renderer's fur pipeline (half-Lambert diffuse +
Kajiya-Kay anisotropic specular against the strand tangent; see
`modules/renderer/shaders/fur.frag` and its CPU mirror `furShade` in
`modules/renderer/source/core/render_shared.hpp`). This module deliberately
does not duplicate that math.

## Performance model

Shell fur's cost is fill-rate: every shell layer blends over every covered
pixel, and the blend traffic - not the ALU - dominates (public measurements
put a 20-layer shell stack at roughly 1.1 ms per frame at Full HD on a
mid-range desktop GPU; integrated and mobile GPUs pay proportionally more).
Two conclusions drive the design:

1. The layer count is the lever. Everything else (one texture fetch, cheap
   lighting) is fixed per pixel; the layer count multiplies the whole cost.
2. Screen coverage scales the cost linearly. A fur surface that covers half
   the screen costs half the blend traffic.

The renderer module additionally mirrors the fur path in its headless
(software) rasterizer, so deterministic regressions can run without a GPU;
both sides share the same formulas through `furShade`/`furAlpha` and the
`furDisplaceVertex` CPU displacement.

## LOD

`evaluateFurLod` maps a surface's angular size (world bounding radius /
camera distance - the same metric the headless backend's mesh LOD uses) onto
a five-band shell budget, by default:

| band | angular size | layers | height scale |
| ---- | ------------ | ------ | ------------ |
| 0    | >= 0.32      | 14     | 1.0          |
| 1    | >= 0.16      | 10     | 1.0          |
| 2    | >= 0.08      | 6      | 0.9          |
| 3    | >= 0.03      | 3      | 0.8          |
| 4    | < 0.03       | 1      | 0.7          |

The OUTERMOST layer's alpha dissolves across the lower `lod_fade_fraction`
of each band, so the reduced layer count of the next band takes over without
a visible pop (the tips are already gone when the layer disappears). Once the
fade reaches zero the layer is folded away entirely and stops costing draw
calls. Strand height scales down with distance as well: long stray strands
rendered from a 3-layer stack read as sparse spikes.

All five bands' layer counts, thresholds, height scales, the fade fraction
and a hard `max_layers` ceiling are configuration keys (see
configuration.md); the renderer applies a second hard clamp
(`[renderer] fur_max_layers`) as a safety net behind the policy.

## Intended usage flow

1. Asset time: pick parameters (a preset via `presetParameters`, or custom),
   call `generateFurTexture`, upload through `renderer.createTexture`, and
   reference the texture in the fur material's baseColorTexture slot.
2. Frame time: for each furry surface, compute `angularSize =
   boundsRadius / distance`, call `evaluateFurLod`, and submit:
   - the opaque base mesh as a regular `drawMesh` (the depth anchor), then
   - one `drawFur(mesh, modelMatrix, material, FurLayerParams{...})` with the
     decision's `layerCount`, `furHeight = strandLength * furHeightScale`,
     `gravityStrength` from the parameters, `densityBoost`, and
     `outerLayerFade`.

Fur draws must sit AFTER the surface's base mesh (they blend over it) and
are never reordered by the renderer's draw sorting/batching: the layer order
inside one `drawFur` call is the compositing order.
