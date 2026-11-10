# Character customisation design (research-driven revision)

This document records the industry research behind the character
customisation module's revisions, the authoring contract that follows
from it, and how the demo content implements that contract. Read
`overview.md` first for the module's data model; this document explains the
deformation and junction decisions on top of it.

## 0c. Seventh revision: downloaded models only, body plans, natural necks

The sixth revision's delivery was rejected: "The character customisation is
still far from good enough. DON'T generate body parts; download only."
The user's verdict on the revision 5/6 pipeline (procedural primitives
composed into heads/tails/wings, plus a procedural neck tube) was that the
parts read as generated - the 0.2 m collar-to-head tube read as a pipe, and
no amount of primitive shaping fixed that. Revision 7 replaces the content
generation pipeline wholesale:

- **Every body part is extracted from a downloaded, rigged model pack**
  (`scripts/extract_character_parts.py`). The script performs exactly four
  operations on the downloaded geometry - linear-blend skinning into a rest
  pose, dominant-joint-weight partition cuts (every triangle assigned to
  exactly one piece; no dropped scraps, no sliver multiplication - the
  revision-4 lesson), rigid transforms, and vertex-level morph/taper
  deformation - and never authors new geometry. Subdivision is not used
  anywhere.
- **Human parts come from the artist's own modular split.** Quaternius
  Ultimate Base Characters ships the male Ranger outfit as separate
  Head/Eyes/Eyebrows/Body/Arms/Legs/Feet meshes on one skeleton - the
  junctions are the artist's, so the demo needs no cut sealing at all on
  the humanoid plan. (Found by chasing the pack through a downstream Godot
  project that committed the glTF; the pack's own Drive mirror is quota-
  locked.)
- **The procedural neck tube is deleted.** The human head carries its own
  natural neck geometry; it sleeves over the ranger collar (the head's
  measured neck ring is wider than the collar hole, so the collar rim is
  inside the neck volume from every angle). This directly removes the "neck
  way too long / straight like a pipe" defect - the neck is now a real
  neck with its natural curve, of the artist's proportions.
- **Cross-species grafts seal with funnel morphs, not collars.** The dragon
  skull (Dragon Evolved, {Head,Neck} weight region cut on a plane along the
  neck axis) buries its base ring inside a collar whose baked `collar_open`
  morph flares the ring to the skull's measured base radius; the quadruped
  neck stub bakes `neck_open`/`neck_pinch` morphs that interpolate the stub
  between the dragon and human head ring radii. The runtime computes the
  weights from the attached head's measured ring - one measurement, both
  sides of every junction (the section-2 contract).
- **Two dragon sources supply the skull, the wings, and the tails.** The
  Dragon Evolved skull grafts as above; the basic Flying dragon's Wing1-4
  membrane chains attach in their bind pose (a swept bat wing - the
  perched pose does not need re-posing); the Evolved tail cone, cut below
  the hips, becomes the reptile tail (1.1 m, root pinched) and the naga
  lower body (3.2 m serpent body with a two-end taper). All animal-tail
  donors evaluated (horse/donkey/wolf/bull/deer) turned out to be flat
  hair sheets and were rejected - the same defect class as the rejected
  revision-3 T-Rex tail.
- **Body plans are assembly profiles.** Four registered profiles -
  humanoid, centaur (scaled quadruped hindquarters + human torso grafted at
  the withers), naga (human torso on the serpent body), quadruped (animal
  body with a grafted head) - switch through `body.plan`; the screen
  rebuilds the assembly because the topology changes. Four AnimaSim
  species (horse/wolf/deer/bull) swap on the live quadruped/centaur body
  instance via replacePart.
- **Duplicate appendages attach on real connectors.** Up to three heads
  share the collar connector (capacity 3) with lateral offsets and outward
  rolls; up to three wing pairs stack on dedicated wither connectors with
  phase-offset flap drivers; up to three arm pairs graft along the torso
  sides with outward yaw fans. The module scales joint-anchored connector
  positions by part scale automatically, so the parameter bindings are
  axis scales only - the revision 4-6 JointTranslation bindings were
  redundant against that behavior and are gone.
- **Verification.** `--character-test` rewrites the assembly 13 ways
  (both head variants on two plans, three heads, three wing pairs, three
  arm pairs, tail, all four plans, species swaps, parameter extremes,
  recipe round-trip) and passes 21/21 with zero warnings; 30 sample
  frames were captured and personally reviewed (contact sheets per plan,
  neck-junction close-ups, tail sweeps).

## 0d. Eighth revision: frame-audit defect fixes (geometry + rig)

The delivered revision-7 sample set was audited frame by frame; the defects
below were found, root-caused, and fixed. All geometry work remains
deformation of downloaded meshes - no authored primitives.

1. **Stale delivery.** The shipped frames predated the final build (fresh
   runs of the same binary rendered differently; the `broad`/`compact`
   frames were pixel-identical to the default). The whole set was re-shot
   from the current build by the persisted `scripts/shoot_samples8.sh`,
   with per-preset camera framing and junction close-ups
   (`--camera-target-y` aims the orbit; the screen no longer re-frames
   every update when explicit CLI framing is present).
2. **Species swap silently skipped.** `applyDrivePreset` assigned
   `quadSpecies_` directly, then `setQuadSpecies` early-returned because
   the field already equalled the request - every species preset kept the
   horse body. The pre-assignment is gone; wolf/deer/bull presets swap for
   real (frames 16/22/23 show three distinct bodies).
3. **Reptile tail inverted taper.** The old `build(root_factor=0.20)`
   crushed the root and kept the underbody belly: measured radius profile
   thin(0.066) -> fat(0.209) -> medium(0.163), i.e. a spindle. The tail is
   now reshaped about its measured spine into a monotonic taper
   (`reshape_taper`), slimmed at the mount ring, and bent on a droop arc
   (`progressive_bend`) so it leaves the tailbone ~25 deg below horizontal
   and tapers to a thin tip.
4. **Naga = torso on a spear.** The serpent body hung ~85 deg down from
   the pelvis and lifted the torso into the air. The naga body now follows
   an explicit direction curve (`bend_to_directions`: 87 deg native ->
   62 -> 45 -> 30 -> 16 -> 7 -> 5 deg along the arc) so it descends from
   the hips and lies along the ground, and `round_cross_sections`
   normalizes each local cross-section about the local spine tangent
   (stretching the hooked source blob had flattened sections into
   vertical-ellipse ribbons).
5. **Wing defects.** Span normalized 2.05 -> 1.55 m per wing (the old tips
   touched the floor), a -10 deg dihedral arch is baked about the root
   (the bind pose points the wing up-back, and the old flat fin drooped),
   the gray elbow-bone nub (17 all-gray verts poking out of the membrane
   silhouette) is dropped, and the open root ring is closed by a funnel
   cuff (`seal_ring_skirt`). Stacked wing slots roll outward per slot
   (0 / -12 / -22 deg) so three pairs stay readable.
6. **Centaur.** Hindquarters back to 1.0x (1.30x read as a jockey on a
   horse); the mount sank from y 0.185 to 0.075 so the torso's open hip
   ring is buried; the per-corner majority vote left ragged Neck1/Torso3
   boundary spikes at the withers - every triangle beyond the cut plane is
   now dropped and the chest ring is capped with a short stub seal.
7. **Extra arm pairs invisible.** The fan yawed about the vertical axis,
   which only spins a hanging arm in place. Slots now ROLL about the
   forward axis (-26 / -42 deg) with anchors moved to the torso surface
   (z -0.120 / -0.100).
8. **Two heads intersected** (lateral 0.085 vs dragon-head half-width
   0.19). Spread to +/-0.15 m with +/-14 deg roll.
9. **Dragon head.** The {Head,Neck} cut moved from -0.45 to -0.75 along
   the neck axis (a visible short neck instead of a skull glued to the
   collar), the base shell is sealed by a funnel skirt, and the graft
   still buries the base ring inside the flared collar (the `collar_open`
   target re-derives from the new cut ring).
10. **Quadruped graft** sinks 0.055 (was 0.105) so the horse neck stub rim
    stops bulging around the grafted head.
11. **Floating shoulder shard.** The ranger pauldron is weighted 100% to
    `upperarm_l`; keeping it in the bind-pose torso left a flat armor flap
    floating at the T-pose shoulder in every frame. It now rides the posed
    arm (and mirrors to the right side with it).
12. **Default humanoid** carries the reptile tail (the profile and the
    `human` preset no longer disagree).

Verification: zero-warning build; `--character-test` 21/21; 37 frames +
one matrix re-shot from this exact build and personally reviewed
(`work/sheets8/`).

## 0b. Sixth revision: anatomical wing/tail silhouettes, real faces

A frame-by-frame re-audit of the fifth revision's own 38-frame sample set
(each frame inspected, cross-checked across views, suspicious regions zoomed
4x) found six defect classes the earlier spot-check had missed:

- **Membrane wings read as paper fans.** The fingers fanned in the
  *sagittal* plane (backward-down), so the flat webbing showed its full face
  from the rear AND the side - the "pink blade" seen at the shoulder in side
  views. Rebuilt: humerus->elbow->wrist arm chain, four fingers spreading
  outward-backward with an upward arch, and webbing panels stretched
  between adjacent boundary curves with **concave scalloped trailing
  edges** (each interior chord dips toward the wrist). The fan plane now
  faces up-back: rear/top views see the wing face, side views see a thin
  arched profile.
- **Feather wings were two bare sticks with a few flat sheets**, far
  smaller than the membrane variant. Rebuilt at matching scale: 7 primaries
  fanning from the wrist outward-backward, 8 secondaries hanging from the
  forearm's trailing edge, 6 coverts over the root; every feather is
  cambered, individually rolled about its own spine (Rodrigues rotation) and
  length-jittered, so the wing reads as layered plumage.
- **Tails rested as stiff horizontal cones.** The old base curve drooped
  only 11.5 degrees over its length. The rest spine now leaves the tailbone
  angled back-down, steepens continuously, and ends ~54 degrees below
  horizontal; the rig's `tail_mid`/`tail_end` joints ride the new curve
  (mid at t=0.40, end 0.19 m of arc before the link ring, preserving the
  cuff overlap), and the root ball shrank 0.082->0.062 and sunk forward so
  it never bulges out of the lower back.
- **Spade/fan tail tips were paper sheets** (edge-on yaws rendered empty
  cells in the close-up sheet). Both fins are now lens volumes with
  thickness tapering to the edges.
- **The human head was effectively faceless at body scale**: beady dark
  eyes buried in the lid sphere, and the muzzle+jaw+chin stack read as a
  claw-like growth. Rebuilt as a wedge face: subtle nose ridge, eyeball with
  a visible sclera ring and a small iris cap (the old iris covered most of
  the visible hemisphere, so eyes read as holes), and a lip volume - a
  flattened capsule hugging the face with the dark lip line baked into its
  radial coloring. Anchor lesson recorded: the dome's cheek/jaw gaussians
  push the real surface 3-6 mm past the analytic ellipsoid, so facial
  geometry must sample the built mesh (`face_front_z`) instead of the
  analytic surface, or it ends up buried.
- **Head proportion / species-face cleanups.** Skulls scaled 1.10x relative
  to the collar and the collar shortened (0.130->0.104) - the old ratio read
  as a pin-head on a stilt neck for six of nine species. Alien eyes moved
  front-set with a smaller almond, its crater mouth replaced by a painted
  slit; demon/blue-demon muzzles widened and flattened with the mouth band
  narrowed to a lip line; yeti muzzle shortened with a painted lip; dragon
  and skull teeth shrunk and tucked into the mouth line; the hair cap now
  fades over a smoothstep band instead of a hard helmet rim.

Sample set re-rendered in full (`scripts/shoot_samples.sh`); frame 32
(extreme tall) re-shot with a farther orbit so the head is not clipped by
the viewport. Self-test after the rebuild: 13 passed / 0 failed / 1 skip;
fresh-archive build reproduces both with zero warnings.

## 0a. Fifth revision: complete procedural parts, per-face winding, honest silhouettes

The fourth revision's delivery failed re-verification: re-auditing it with a
fresh screenshot matrix and an OBJ audit showed the same user-visible defect
classes. The root cause was architectural - the head/tail/wing pieces were
*joint partitions of rigged pack meshes*, which are never complete body
parts: pieces weighted to neighbouring joints are dropped, leaving floating
scraps and faceless tubes, and subdivision then multiplies sliver triangles
(demon head: 17.5k triangles of which 9.4k were area < 1e-5 slivers). The
fifth revision replaces that pipeline wholesale:

- **Complete procedural heads (9).** `scripts/build_character_assets.py`
  authors every head from primitives: a shaped ellipsoid skull dome (brow /
  cheek / jaw gaussians), a lofted muzzle with a closed tip cap and softly
  blended lip strip, a jaw/chin ball, painted nostrils, species ears, horns
  or frill, eyes built as eyeball-in-lid (the lid sphere's front stays
  0.9·eye_size behind the eyeball front so the iris reads from every frontal
  angle), and the canonical gripping collar re-using the dome's exact cut
  ring in segment order. Species: human (skin-tagged, tinted by
  `skin.tone`), demon, dragon (long muzzle, teeth, serrated crest sheet),
  serpent (hood, slit eyes), orc (underbite, tusks), yeti (fur noise),
  alien (black almond eyes), blue demon, skull (dark sockets, tooth row).
  Each bakes the same three morphs (`head_size`, `muzzle_length`,
  `neck_fit`), so every head grafts onto the neck at every range end.
- **Complete procedural tails (5 base+tip chains).** reptile / spade /
  club / fang / fan: Catmull-Rom spine, 14-segment elliptical sections,
  ~0.85 m rest length, fused root ball, cuff-overlapped link ring, distinct
  tips (spade fin, club ellipsoid, barbed sting, vertical fin), baked
  `tail_curl` morph with the matching binding constant in the generated
  metrics header.
- **Complete procedural wings (2 variants, both sides).** Membrane: arm +
  four finger struts + filled webbing sheets spanning every gap between
  boundary curves (double-sided). Feather: central arm + nine flat vanes
  lying in the wing plane with quill/vane/tip color bands plus covert rows.
  The root cuff dives forward into the torso socket instead of a ball
  poking through.
- **Per-face outward winding at construction time.** The engine culls back
  faces (Vulkan pipeline; the headless backend probes winding against
  vertex normals), so mixed winding inside one primitive is fatal: culled
  outward faces expose buried geometry through the surface. Every primitive
  builder (`radial_dome`, `loft`, `uv_sphere`, `ellipsoid_at`, collar)
  now orients each face against its own analytic outward reference, and the
  component-level volume flip runs only on closed components (an open
  surface's signed volume is meaningless). The sphere builders also fixed
  a parametrisation bug that folded the lower hemisphere onto the upper
  one, and the dome pole now sits on the local +y axis instead of being
  dragged diagonally across the skull.
- **Honest silhouettes.** Muzzles extend until the tip sits in front of the
  dome surface at the tip's height (a buried muzzle is invisible under
  back-face culling), eyes protrude past their lids, and the muzzle tip is
  closed - the revision-4 renders showed funnel-shaped eyes and ring-shaped
  mouths caused by exactly these three geometric bugs.
- **Two tails straddle the spine.** The pair now spreads +/-0.62 rad AND the
  roots separate laterally (+/-0.04 m, tail1 right / tail2 left, reset on
  removal), so the root balls no longer coincide inside the pelvis.
- **Wing variants and scale.** `wings.span` now scales the wings at their
  authored size (0.95 x / 0.55 y gains instead of 0.60 / 0.30, which shrank
  the new wings into stubs), and the panel gains a Wing Style pair
  (membrane / feathered) swapped on live instances.
- **Options and ranges.** 9 heads, 5 tail chains, 2 wing variants; new
  drive presets (`human`, `skull`, `fangtail`, `fantail`, `featherwings`);
  the self-test swaps every head and every tail variant and re-checks the
  two-tail spread. Character total is ~14k triangles at 70-100 fps in the
  software rasterizer - density was cut where it only multiplied slivers,
  and spent where it buys shape (silhouette, faces, tips).

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

## Revision 9: downloaded-only parts, verbatim (current)

User mandate: "Ensure no character parts are generated by you. Character
parts must be downloaded only." Revisions 4-8 still baked vertex-level
deformations onto downloaded meshes (honest taper, bend arcs, cross-section
normalization, root seals, funnel morphs) - that is generating parts, and
the deformed pieces (flat ribbon tail, plank serpent, mascot-mask dragon
head) were exactly the worst-looking results in the sample frames.

### The verbatim rule

`scripts/extract_parts_verbatim.py` (replaces the rev-7 extractor) emits
every part as a downloaded model's own triangles, unchanged in shape:

- artist-split packs contribute whole mesh nodes (the Ranger male's
  `Male_Head`+`Eyes`+`Eyebrows`, `Male_Ranger_Body`+belts,
  `Male_Ranger_Arms`+`Arms_Bracer`+`Acc_Pauldron`,
  `Male_Ranger_Legs`+`Feet_Boots`; the hood node belongs to no part, which
  also removes the rev-8 "hood shard");
- single-mesh packs contribute the triangle subset dominated by the part's
  own rig joints (dragon `Neck`+`Head`, the flying dragon's `Wing1-4`
  chains, the Cethiel dragon's `tail1-3`);
- quadruped bodies are the WHOLE animal; the centaur variant is the whole
  animal minus its neck/head joints (subset selection, nothing added);
- poses run only through the asset's own joints (LBS with the asset's own
  weights: arms hang by rotating the UE `upperarm_*` joints);
- orientation/size changes are rigid transforms (translate/rotate/uniform
  scale), applied at extraction and listed in `extract_report.json`
  (dragon head 0.30, wing 0.85, tail 1.6/4.6, rest pitches -118/-102 deg);
- colors come from the assets' own textures (bilinear-sampled, sRGB ->
  linear) or material factors - no repainting;
- FORBIDDEN: caps, skirts, funnel seals, vertex reshaping, morph deltas,
  procedural recoloring.

### New sources

- Tails (reptile + serpent): OpenGameArt CC0 "Cethiel's Dragon 3D"
  (`work/assets_dl/oga2/dragon.dae` + `dragon.png`) - a skinned COLLADA
  with named `tail1-3` joints and an authored thorny curved tail. The old
  Dragon Evolved tail subset carried hip geometry and a curled bind shape;
  both dragon tails were unusable.
- The human skin texture `T_Regular_Male_Dark_BaseColor.png` was missing
  from the local mirror and is now fetched from the same ValTrainer
  repository - that was the rev-8 "pale clay head" root cause.

### Runtime changes

- The morph system is deleted end to end (`DeformationInput`,
  `applyDeformation`, `weightForMorph`, morph blocks in the OBJ loader);
  `head.size` is now a rigid uniform part scale on both heads.
- `character_metrics.gen.hpp` is regenerated from the verbatim parts
  (measured anchors: torso `headNeckY/Z`, shoulders, hips, per-species
  `NeckBase/Withers/Ground/Length`, tail and serpent extents, wing span).
- Body plans: quadruped = the WHOLE animal (own head, own tail; the
  dragon-head graft on quadrupeds is gone), centaur = whole horse minus
  neck/head + human torso over the stump opening, naga = torso + Cethiel
  serpent (rest pitch baked, grounded on the measured pitched extent),
  humanoid unchanged + baked-pitch reptile tail.
- The tail sway driver composes a yaw on top of the baked rest pitch; it
  must never replace the connection rotation (that bug made both tails
  spring upright in the first rev-9 build).
- The overview matrix renders every frame into an exact grid (no filler).

### Verification

- Build: 0 warnings (cmake Ninja Release, GLFW off).
- `--character-test`: 20 passed / 0 failed / 0 skipped.
- All 36 sample frames re-shot (`scripts/shoot_samples9.sh`) and personally
  reviewed via contact sheets: junctions close by overlap, no flat-ribbon
  tail, no plank serpent, no mascot mask, quadrupeds keep their own heads,
  framing covers every plan and seam.
