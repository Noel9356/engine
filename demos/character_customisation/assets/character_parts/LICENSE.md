# Part asset licenses

Every OBJ in this directory is a modified piece of a downloaded, licence-clean model pack; the `scripts/extract_species_parts.py` pipeline performs rigid transforms, hybrid triangle-subset selection (ear sheets on summed skin weights with the animal midline, tails on the union of weight-dominant and joint-proximity assignment, the remainder on joint-anchor proximity), boundary-ring caps (fans over the source boundary loops), boundary-ring straightening, pinned Loop subdivision on the low-density parts, and baked vertex-level morphs, and never authors new geometry. `scripts/refine_human_parts.py` welds the per-corner human-sourced exports and computes their smooth normals.

| Source | Files | Licence |
| --- | --- | --- |
| Quaternius Universal Base Characters (bare male body, via the ValTrainer Godot mirror `github.com/idoraz1/ValTrainer`) | torso_bare, arm_bare_left/right, leg_bare_left/right | CC0 |
| Quaternius Modular Character Outfits - Fantasy (ranger outfit, via the same mirror) | torso_ranger, arm_ranger_left/right, leg_ranger_left/right, clothes_belt, clothes_bracer_left/right, clothes_pauldron_left/right, clothes_hood | CC0 |
| Quaternius Ultimate Animated Animals (via the quaternius-showcase mirror `github.com/trebeljahr/quaternius-showcase`) | head_*, ear_*, tail_*, tail_*_taur, quad_taur_*, quad_body_* for fox, shibainu, husky, wolf, donkey, cow, deer, bull, alpaca, horse, stag | CC0 |
| Quaternius Ultimate Dinosaurs (via the same showcase mirror) | head_*, tail_*, tail_*_taur, quad_taur_*, quad_body_* for trex, raptor, triceratops, stegosaurus, apatosaurus, parasaurolophus | CC0 |
| Quaternius Easy Enemies (via the same showcase mirror) | head_rat, tail_rat, tail_rat_taur, quad_taur_rat, quad_body_rat | CC0 |
| Quaternius Animated Fish (via the same showcase mirror) | head_shark, tail_shark, tail_shark_taur, quad_taur_shark, quad_body_shark, head_dolphin, tail_dolphin, tail_dolphin_taur, quad_taur_dolphin, quad_body_dolphin | CC0 |
| Quaternius Ultimate Monsters (dragon head; dragon wings from the pack's flying dragon) | head_dragon, wing_dragon_left/right | CC0 |
| OpenGameArt "Cethiel's Dragon 3D" by Drummyfish (spiked tail chain) | tail_reptile | CC0 |
| Human head (the artist-split ranger head nodes, via the same ValTrainer mirror) | head_human | CC0 |

The downloaded GLBs are vendored in `assets_dl/` for reproducible extraction. The `extract_report.json` next to the OBJs records, for every part, the measured interface rings (neck, stump, tail, ear), the rigid transform applied, the boundary rings that were capped, and the baked morph blocks. The panel's preview thumbnails are rendered by the engine itself at startup from these same parts (never shipped as image files).
