# Character customisation design (research-driven revision)

This document records the industry research behind the character
customisation module's revisions, the authoring contract that follows
from it, and how the demo content implements that contract. Read
`overview.md` first for the module's data model; this document explains the
deformation and junction decisions on top of it.

## 0. Fourth revision: what the density and options audit changed

A screenshot-driven audit of the third revision (front/side/rear captures of
every drive preset, junction zooms, and the demo's own self-test) found the
same classes of defect the user reported - missing-looking triangles,
disconnected junctions, a flat tail, overlapping double tails, narrow ranges,
too few parts - and drove these fixes:

- **The head floated off the neck from the side.** The demon head's skirt was
  7 mm deep and its mouth flared to 2.2x the neck radius, so the neck tube
  stood in a visible moat with the head's interior showing through. Every
  head now carries the canonical gripping collar: a flared tube welded below
  the cut ring that closes down to `neckRing.radius + 0.006` at 130 mm below
  the origin, so the procedural neck seals into every head from every view
  (the grip tracks `neckRadius` through the heads' baked `neck_fit` morphs).
- **A wing root showed an open hole into the torso.** The two wing source
  chains carried different root-ball geometry; one ball sat recessed behind
  its hole. The pipeline now re-centers both balls on the part origin,
  inflates them past the measured hole radius (0.078 vs 0.05), and recolors
  them toward the wing arm's palette so they read as anatomy, not spheres.
- **The tail was a flat sheet with zero-thickness slices, and its root ball
  poked out at the glutes in the wrong color.** The revision-3 T-Rex cut was
  replaced by three procedural lofted tail chains (reptile, spade, club):
  Catmull-Rom spine, 12-14 segment elliptical cross-sections with a real
  thickness profile, a fused root ball over the torso's tail hole, and a
  tip whose start radius overlaps the base's end radius (industry cuff
  overlap, no coplanar seam). Each base bakes a `tail_curl` morph whose
  interface ring delta is emitted into the generated metrics header and
  re-issued by the demo as the matching `JointTranslation` binding on
  `tail_end` - one formula, both sides, so the tip follows the curl.
- **Two tails hung on the same yaw and intersected.** The second tail's
  connection now spreads the pair symmetrically around the spine (+/-0.55
  rad, applied at connect time and held by the per-frame drivers with
  distinct sway phases), which is the shipped-creator pattern for duplicate
  appendages (explicit per-instance placement, never shared transforms).
- **Everything was low-poly.** Every checked-in part gets one more Loop
  subdivision level with the interface rings pinned (torso 2.0k -> 8.0k
  triangles, demon head 4.4k -> 17.6k, arms/legs/wings 3-4x), crease-aware
  per-corner normals recomputed, and the atlas-vote color blotches removed by
  neighbor-limited color smoothing (only near-equal colors blend, so hard
  boundaries like the shorts stay crisp). Morph deltas subdivide exactly with
  the mesh because Loop subdivision is linear in positions. Per-part density
  now sits at 1.6k-17.6k triangles (character total ~61k), still inside the
  software rasterizer's budget.
- **Too few options and too-restrictive ranges.** Seven head variants (the
  three revision-3 heads plus orc, yeti, alien, and blue demon cut from the
  2024 Ultimate Monsters re-export by the Head-joint partition rule), three
  tail chains, three new parameters (`neck.length`, `tail.curl`,
  `skin.tone`), a five-swatch skin-tone palette applied as a draw-time tint
  on the skin-tagged parts, every slider range widened (see the registry in
  `character_content.cpp`), and the compatibility rules re-bracketed for the
  new worst cases. The demo caps baked morphs at three per mesh no longer;
  weights are a plain vector, so future parts can bake deeper stacks.
- **A build defect the engine shipped with.** The committed fallback shader
  header `mesh_frag_gen.hpp` was missing its closing namespace braces, which
  broke every translation unit including it on toolchains without a shader
  compiler (toolchains with glslang shadowed the file via the build tree).
  The committed copy is balanced now.

## 0b. Third revision: what the visual audit changed (recap)

The revision-3 audit (capturing every drive preset, zooming into every
junction, and re-checking each extracted OBJ in a standalone z-buffer
renderer) fixed, and revision 4 builds on:

- **Faces disappeared from some views.** Two independent causes: the
  rasterizer picked its culling orientation from a single probe triangle,
  which a sharp-creased mesh could flip (culling nearly every front face);
  and the headless LOD path stride-decimates any mesh above 4096 triangles,
  which shreds an organic hero mesh into floating fragments. The probe now
  majority-votes over up to 64 spread triangles, and the character demo
  disables `renderer.lod_enabled` (its meshes are always near camera, so
  LOD switching had nothing to save).
- **Junctions read as disconnected.** The collar skirts on arms poked out
  of the parent's cut hole as floating plates. Arms, wings, and the tail
  root are now sealed with a fused ellipsoid ball joint that covers the
  parent's cut boundary from the outside at every parameter value; skirts
  remain only where the parent surface is closed (legs, head necks).
- **Wings lost their membranes from behind.** Membranes are open (single
  sided) surfaces, so correct backface culling removed them from the back.
  Open-surface triangles are now emitted double-sided.
- **Parts looked wrong or low-poly.** The pug head cut (a chibi box) became
  the Ultimate Monsters demon skull; the dragon head moved to the far more
  detailed Dragon Evolved skull; shading switched from flat-per-face
  to crease-aware smooth normals. Baked morphs are indexed after the vertex
  weld, so they actually reach the runtime now (earlier revisions silently
  dropped them on a count mismatch); the demon head's degenerate one-pixel
  UV strip gets an authored palette, and tiny color atlases are
  gutter-inpainted and voted per face.

## 1. What shipped games actually do

Findings from public postmortems, engine documentation, and tooling
discussions (all visited while designing this revision):

- **Body shape is one continuous surface, not snapped parts.** Body sliders in
  Cyberpunk 2077, Black Desert Online, and Skyrim (BodySlide/CBBE) drive
  morph targets or bone scaling on a single skinned base mesh. Black Desert
  uses bone scale/translation for body proportions and reserves blendshapes
  for the face; BodySlide bodies and outfits share identical topology so the
  same slider set deforms every piece identically and seams cannot open.
- **Modular swap-in parts are for gear, and their seams are designed away.**
  Unreal's modular character docs and forum practice: parts share one
  skeleton, junctions get an intentional overlap or a hidden cuff, and seam
  lines are moved under clothing. Equipment swapping - not body shaping.
- **Spore-style part snapping works because skin is generated over the
  parts.** Spore's creature editor drives an implicit metaball surface along
  a deformable spine; parts (rigblocks) attach to spine bones and the
  generated skin makes everything continuous by construction. A parts system
  without a continuity layer relies on matched junction geometry.
- **Customization meshes are rebuilt on change, never per frame.** Mutable
  (the character-customisation middleware behind several AAA titles)
  combines meshes and textures once per change, asynchronously; the runtime
  render path is an ordinary static mesh afterwards.
- **Extreme slider values cause the clipping players report.** Honey Select 2
  and Bodyslide guides both attribute clipping to extreme slider
  combinations; shipped games clamp ranges and some (Rockstar with
  fat/muscle sliders) removed features rather than ship the clipping.
- **Limb scaling needs counter-scaling at the ends.** VRChat limb-scaling
  prefabs scale arm/leg bones and inversely scale hands/feet so the ends
  keep their shape; junction rings are kept fixed to avoid gaps.

The consistent lesson: **a junction is safe only when both sides derive
their interface geometry from the same parameter formula, and geometry is
only ever re-derived when parameters change.**

Revision-4 additions (sources re-visited while planning this revision):

- **Variant buttons per slot and palettes beat raw pickers.** ESO's creator
  is organized as Race/Class/Body/Face menus with variant lists, Diablo 4's
  as Face & Body / Hair / Accessories; PSO2 and the GameDeveloper ethnicity
  articles favor dedicated skin-tone swatches over free color pickers. The
  demo panel therefore lists variants for every slot and ships a five-swatch
  skin-tone palette (a draw-time tint on the skin-tagged parts).
- **Duplicate appendages get explicit per-instance placement.** Spore's
  parts snap anywhere along a deformable spine; Unreal's Control-Rig spline
  chains drive tails/ears as chains; rigging guides treat tails as
  multi-joint secondary chains. Symmetric paired placement (Reallusion's
  force-symmetry/mirror-copy tooling) is the standard for duplicates - the
  second tail spreads around the spine instead of copying the first tail's
  transform.
- **Hero-mesh density without runtime tessellation = baked subdivision.**
  Overwatch-class hero characters sit at 15k-20k triangles, realistic AAA
  at 30k-60k+ (r/gamedev polycount threads); where the engine has no
  runtime tessellation, the standard workaround is exporting multiple
  subdivision levels (r/unrealengine). Loop subdivision is linear in vertex
  positions, so baked morph deltas survive re-subdivision exactly
  (`delta_new = L(delta_old)`) - the pipeline sub-divides the checked-in
  parts with interface rings pinned instead of re-cutting from sources.

## 2. The junction contract (authoring rules for this engine)

1. **Every junction interface is a function of shared parameters.** The
   torso neck stub radius, the neck piece, and the head collar radius are
   all computed from one `bodyMetrics()` evaluation; no asset may hard-code
   an interface radius that another asset also hard-codes.
2. **A ring that sits at its part origin never desyncs.** Parts whose only
   connector ring is at the local origin (limbs, tails, wings) may deform by
   whole-part scale: the ring stays on the anchor regardless of scale.
3. **A ring away from the origin must be driven by baked morph shapes, not
   part scale.** Scaling the whole head scales its collar ring and opens the
   junction (the "head neck wider than torso neck" defect). Heads therefore
   deform by morph deltas with the collar region masked out.
4. **Anchors that must move with parameters ride rig joints.** The neck
   rises with body height, shoulders move out with body width: those are
   rig joints translated by `JointTranslation` bindings, and the morph
   falloffs are authored so the interface rings move by exactly the same
   formula. One formula per interface, used by both the mesh morph and the
   binding.
5. **Parameter ranges are validated ranges.** Slider minimum/maximum values
   are chosen so that the extreme combinations stay junction-safe and
   free of deep self-intersection; the compatibility rules' relative-scale
   gate catches asset-level mismatches on top.

## 3. Deformation architecture in this build

The renderer in this engine has no skinning or GPU morph path, so the
industry morph-target pattern is implemented consumer-side, at change time:

- Part OBJs may carry `m <name>` morph blocks: per-vertex position deltas
  applied as `position += weight * delta`. Weights come from the assembly
  parameters (for example `head.size - 1.0`).
- The demo content layer owns the mesh upload. When a parameter changes it
  applies the affected morphs on the CPU (a few thousand vertices), reuploads
  the affected meshes, and lets the module's existing connector evaluation
  re-derive transforms. Nothing morphs per frame.
- Connector anchors that move with parameters are rig joints with
  `JointTranslation` bindings; the module already scales joint-anchored
  socket positions by part scale and keeps rotations rigid.

## 4. Performance model (software rasterizer)

The character screen must hold up on a 2-core CPU with the headless
rasterizer backend, which makes shaded pixels the budget:

- The sky dome is authored inward-facing so backface culling keeps only the
  near hemisphere, and it is drawn after the character so occluded pixels
  depth-reject cheaply.
- Module evaluation honours its dirty flags; validation runs when structure
  or parameters change, not every animation-driven re-transform.
- Character meshes stay in the low thousands of triangles per part.

## 5. Sources for the shipped assets

The checked-in OBJs are produced by two pipeline generations:
`scripts/build_character_parts.py` cut the revision-3 parts from CC0 packs
downloaded into `assets_dl/`, and `scripts/refine_character_parts.py`
(revision 4) upgraded those parts in place, built the procedural tails, and
cut four more heads from the 2024 re-export of the Ultimate Monsters pack
(mirrored at archive.org/item/ultimate-monsters; still CC0 by Quaternius).
`scripts/preview_part.py` is the standalone z-buffer previewer both
revisions used for asset audits. See `LICENSE.md` next to the OBJs:

- Humanoid torso, arms, legs: Quaternius *Universal Base Characters*
  (Superhero male), cut along the rig's joint partition (UE-style skeleton:
  `pelvis`/`spine_*`/`clavicle_*` for the torso, `upperarm_*`/`hand_*` per
  arm, `thigh_*`/`ball_*` per leg). Arms end in a fused deltoid ball; legs
  keep the buried interface skirt.
- Demon head: Quaternius *Ultimate Monsters* Demon, `Head`-joint surface of
  the head mesh only, anchored at the skull's measured neck boundary (the
  rig's Head joint sits mid-skull); authored red palette because the source
  UV set maps the head to an unpainted atlas strip. One Loop subdivision
  level.
- Dragon head: Quaternius *Ultimate Monsters* Dragon Evolved `Head`-joint
  surface; one Loop subdivision level.
- Serpent head: Quaternius *Ultimate Monsters* Dino `Head`-joint surface.
- Orc, yeti, alien, and blue demon heads: `Head`-joint partitions of the
  2024 re-export (the pack's glTF skeletons carry the same named joints;
  the partition rule is identical, the atlas palettes are sampled per
  corner from the 1024x1024 atlases, and the canonical gripping collar is
  welded below each cut ring). The Orc ships with its authored hood - the
  hood is what the pack weights to the Head joint.
- Wings: Quaternius *Ultimate Monsters* Dragon wing chains (one source side
  per engine side), rooted with a ball joint; open-boundary membrane
  triangles are emitted double-sided.
- Tail: three procedural lofted chains (reptile, spade, club) authored by
  `scripts/refine_character_parts.py` - Catmull-Rom spine, real thickness,
  fused root ball, baked `tail_curl` morph with its matching binding
  constant in the generated metrics header. (Revisions 2-3 cut a T-Rex
  tail, which turned out to be a near-flat sheet with zero-thickness
  slices.)

Joint partition cuts follow the animation boundary, so the extracted pieces
are exactly the regions the source skeleton deforms - the same seam the
source artists authored.
