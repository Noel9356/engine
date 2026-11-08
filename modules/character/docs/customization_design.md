# Character customisation design (research-driven revision)

This document records the industry research behind the character
customisation module's revisions, the authoring contract that follows
from it, and how the demo content implements that contract. Read
`overview.md` first for the module's data model; this document explains the
deformation and junction decisions on top of it.

## 0. Third revision: what the visual audit changed

A screenshot-driven audit of the second revision (capturing every drive
preset, zooming into every junction, and re-checking each extracted OBJ in
a standalone z-buffer renderer) found four defect classes and drove these
fixes:

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
  detailed Dragon Evolved skull; the tail cut was re-measured on the T-Rex
  (the old box caught belly geometry); shading switched from flat-per-face
  to crease-aware smooth normals; the chunky cuts get one Loop subdivision
  level. Baked morphs are indexed after the vertex weld, so they actually
  reach the runtime now (earlier revisions silently dropped them on a
  count mismatch); the demon head's degenerate one-pixel UV strip gets an
  authored palette, and tiny color atlases are gutter-inpainted and voted
  per face.

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

All meshes in `demo_character_customisation/assets/character_parts/` are cut by
`scripts/build_character_parts.py` from CC0 packs downloaded into
`assets_dl/` (see the pack directories and `LICENSE.md` next to the OBJs):

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
- Wings: Quaternius *Ultimate Monsters* Dragon wing chains (one source side
  per engine side), rooted with a ball joint; open-boundary membrane
  triangles are emitted double-sided.
- Tail: two measured spatial cuts along the *Dinosaur Animated Pack*
  T-Rex's tail (the monster dragons are tailless), Loop-subdivided, rooted
  with a ball joint; the dark source palette is brightened.

Joint partition cuts follow the animation boundary, so the extracted pieces
are exactly the regions the source skeleton deforms - the same seam the
source artists authored.
