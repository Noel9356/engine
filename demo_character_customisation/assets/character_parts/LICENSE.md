# character_parts licensing

The meshes in this directory were extracted from freely licensed model packs
downloaded from the internet; every source is released under CC0 1.0
(Universal Public Domain Dedication) by Quaternius (https://quaternius.com):

- `torso_humanoid.obj` - torso/joint partition (root..neck + clavicles) of the
  Superhero male in the *Universal Base Characters* pack.
- `arm_left.obj`, `arm_right.obj` - the upperarm/hand chains of the same
  character (one source side per engine side, no mirroring), rooted with a
  fused ball joint.
- `leg_left.obj`, `leg_right.obj` - the thigh/foot chains of the same
  character (buried interface skirt at the hip).
- `head_demon.obj` - the Demon's `Head`-joint surface in *Ultimate Monsters*
  (head mesh only, anchored at the measured skull/neck boundary; authored
  red palette because the source UV strip is unpainted).
- `head_orc.obj`, `head_yeti.obj`, `head_alien.obj`, `head_bluedemon.obj` -
  the Orc, Yeti, Alien, and Blue Demon `Head`-joint surfaces of the 2024
  *Ultimate Monsters* re-export, cut by the same joint-partition rule,
  palettes sampled per corner from the 1024x1024 atlases, canonical
  gripping collar welded below each cut ring.
- `head_dragon.obj` - the Dragon Evolved `Head`-joint surface in
  *Ultimate Monsters*.
- `head_serpent.obj` - the Dino's `Head`-joint surface in *Ultimate
  Monsters*.
- `wing_left.obj`, `wing_right.obj` - the Dragon's wing chains in
  *Ultimate Monsters* (each engine side keeps its own source chain), rooted
  with a ball joint; open-boundary membrane triangles are double-sided.
- `tail_reptile.obj`, `tail_reptile_tip.obj` (and the `tail_spade*` /
  `tail_club*` siblings) - procedural lofted tail chains authored by
  `scripts/refine_character_parts.py`: Catmull-Rom spine, elliptical
  cross-sections with a real thickness profile, a fused root ball, and a
  baked `tail_curl` morph. (Revisions 2-3 cut a T-Rex tail from the
  *Dinosaur Animated Pack*, which turned out to be a near-flat sheet.)

The revision-3 extraction pipeline lives in `scripts/build_character_parts.py`
at the engine project root; the revision-4 upgrade pipeline
(`scripts/refine_character_parts.py`, plus the `preview_part.py` z-buffer
auditor) sub-divides these parts with pinned interface rings, rebuilds the
tails procedurally, and cuts four more heads (orc, yeti, alien, blue demon)
from the 2024 re-export of the same *Ultimate Monsters* pack (CC0 by
Quaternius, mirrored at archive.org/item/ultimate-monsters), which lives in
`assets_dl/um/`: skinned sources are cut along their rig partition (a part
is exactly what its joint moves, so the cut follows the animation boundary),
vertices are baked into the bind pose, texture/material colors are sampled
into per-vertex colors (tiny atlases are gutter-inpainted and sampled with a
per-face seven-point vote), chunky cuts receive one Loop subdivision level,
shading normals are crease-aware per corner, and every junction away from a
part's origin gets either a canonical interface ring with a color-matched
skirt (heads, legs) or a fused ball joint (arms, wings, tail). Parts whose
interface ring is not at their origin also ship baked morphs (`m`/`d` blocks,
indexed after the vertex weld) whose interface deltas match the runtime
anchor bindings; the shared constants are emitted as
`demo_character_customisation/source/demo/character_metrics.gen.hpp`. The OBJs here are the
checked-in outputs so the demo runs without network access; re-running the
script refreshes them from freshly downloaded packs. Only the neck
transition piece is procedural (see `demo_character_customisation/source/demo/character_content.cpp`).

CC0 1.0: https://creativecommons.org/publicdomain/zero/1.0/
