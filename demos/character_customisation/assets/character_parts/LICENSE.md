# Part asset licenses

Every OBJ in this directory is a modified piece of a downloaded, licence-clean model pack; the `scripts/extract_pipeline.py` pipeline performs rigid transforms, triangle-subset selection by the source rig's own skin weights, boundary-ring caps (fans over the source boundary loops), and baked vertex-level morphs, and never authors new geometry.

| Source | Files | Licence |
| --- | --- | --- |
| Quaternius Universal Base Characters (bare male body, via the ValTrainer Godot mirror `github.com/idoraz1/ValTrainer`) | torso_bare, arm_bare_left/right, leg_bare_left/right | CC0 |
| Quaternius Modular Character Outfits - Fantasy (ranger outfit, via the same mirror) | torso_ranger, arm_ranger_left/right, leg_ranger_left/right, clothes_belt, clothes_bracer_left/right, clothes_pauldron_left/right, clothes_hood | CC0 |
| Quaternius Ultimate Animated Animals (via the animasim mirror `github.com/danwahl/animasim`) | head_*, ear_*, tail_*, quad_taur_*, quad_body_* for fox, shibainu, husky, wolf, donkey, cow, deer, bull, alpaca, horse, stag | CC0 |
| Quaternius Ultimate Monsters (dragon head; dragon wings from the pack's flying dragon) | head_dragon, wing_dragon_left/right | CC0 |
| OpenGameArt "Cethiel's Dragon 3D" by Drummyfish (spiked tail chain) | tail_reptile | CC0 |
| Human head (the artist-split ranger head nodes, via the same ValTrainer mirror) | head_human | CC0 |

The `extract_report.json` next to the OBJs records, for every part, the measured interface rings (neck, stump, tail, ear), the rigid transform applied, the boundary rings that were capped, and the baked morph blocks. `assets/previews/*.png` are rendered by the engine itself (`--part-previews`) from these same parts.
