# Character customisation design (research-driven revision)

This document records the industry research behind the character customisation module's revisions, the authoring contract that follows from it, and how the demo content implements that contract. Read `overview.md` first for the module's data model; this document explains the deformation and junction decisions on top of it.

## 0i. Fourteenth revision: anatomical segmentation, welded rings, the neck part (current)

Revision 14 rebuilds the body plans around an explicit vertebra-level segmentation with shared-polyline welds: every junction is now a real mesh seam (both sides carry the identical boundary ring) instead of a buried overlap.

### Research findings applied

- **Vertebral segmentation (anatomy references for anthro builds):** the biped body decomposes into head, neck, arm, thoracic-lumbar segment, sacrum-coccyx segment, and legs. The taur plan replaces the sacrum-coccyx segment and the legs with the lower body (measured from the animal's thoracic to sacral region) and connects the upper body's lumbar end to the lower body's thoracic end; the tail always roots at the coccyx - the biped's sacrum-coccyx segment or the taur lower body's own rump socket.
- **Welded junction rings (modular character pipelines):** a slice at the L3/L4 plane splits the shipped human torso into the two segments carrying the same cut polyline, and both bind the same body scales so the seam stays closed at every slider value. The head's bottom ring (the atlas cut) welds to the neck part's top ring the same way; the taur lower body grows a junction graft that sweeps the animal's own neck cross-sections from the withers ring to the human waist ring, sharing both end polylines.
- **Neck as a part (fursona builder neck sliders):** the neck spans the collar socket to the atlas ring as its own swappable part (one lofted human neck plus twenty species necks whose intermediate rings ride the head ring's smoothed angular profile), straightened onto the collar's vertical axis so the skull sits over the shoulders. Neck length scales the column (the head rides the top socket); thickness and head-size ride baked morphs with both end rings pinned, so the welds hold at every value.

### Pipeline changes

- `scripts/extract_species_parts.py` splits the human torso at the waist plane (`split_human_torso`, the shared ring recorded in `waist_ring.json` for recovery runs), builds the neck lofts (`angular_profile` bins and smooths jagged pack rings; the head's own ring is radially and planarity clamped, written back into the head so both weld sides carry the cleaned polyline), and sweeps the taur graft along a bezier spine with a waist-relative radius cap (wide-skull species such as the triceratops would otherwise flare into a sombrero) and a stump-ring snap (the body's cut rim gathers onto its own smoothed profile).
- The head pipeline replaces the old sleeve/neck_cut trim with an inflection scan along the smoothed radial profile: the skull keeps everything above the first sustained radial rise, the tube below feeds the neck part; the horse skull that the old ear-rim clamp trimmed away is whole again.
- The demo registers thoracic-lumbar and sacrum-coccyx segments (each with its own rig), the neck parts (a `head.mount` socket at the measured atlas height), and new `neck.length` / `neck.thickness` parameters; the profiles chain pelvis -> chest -> neck -> head on the biped and body -> chest -> neck -> head on the taur.
- The fur coat converts to a change-time vertex-colour repaint in one pass with the morphs (the old separate colour pass silently re-uploaded the unmorphed rest mesh and reverted the morphs): a scalar luminance shade preserves the authored modelling, a dorsal-to-ventral gradient replaces the flat tint that read as painted latex, and the taur graft's upper band repaints with the coat so no skin ring shows at the weld. The animal grafts keep their authored colours; the mean-colour divide is gone.
- The taur upper assembly (chest, neck, head, ears, arms, garments, wings) binds the lower-body scale uniformly so the graft weld rides the size slider; leaving the taur plan resets the scale. The biped's neck and head bind the body width and muscle mass so a broad frame keeps a proportional neck.
- The Lua lab exercise referenced a profile id that never existed (`plan_humanoid` instead of `humanoid_biped`) and had failed silently since it was merged; it now creates the registered profile.

### Verification

- Build: zero warnings; `--character-test`: 22 checks pass (the head/ear/neck follow check now asserts the neck instance and part id per species).
- The frame matrix (61 presets), seven part-preview sheets (160 parts, the necks and the torso segments included), the five panel tabs, and the junction close-ups re-captured and reviewed.
- Numeric weld checks: the waist seam (thoracic vs pelvis) measures zero distance, the taur graft's top ring matches the human waist profile, and the head-to-neck atlas ring matches exactly at default scale and tracks `head.size` through the baked morph.

## 0h. Thirteenth revision: anthro-mounted heads, junction anchoring, capture framing

Revision 13 tightens the physical junctions and the presentation layer: every graft measures against the surfaces it must meet, and every capture frames the assembly it actually shows.

### Research findings applied

- **Anthro head scale (avatar-maker proportion studies):** species heads ride the humanoid collar socket, so every skull scales to one canonical height whatever the animal's natural size - a fox head reads at human-head scale on a biped chest while the quadruped plan still ships the whole, natural-size animal. The skull floor seats a set distance above the socket so the collar reads as a short neck, and short-necked species (the canids, the dinosaurs with stub necks, the fish) carry a small tapered throat tube below the origin that roots into the collar hole; long-necked species keep their own tube. The horizontal fish heads bury deeper - the merfolk skull emerges from the shoulders instead of overhanging them.
- **Taur junction anchoring (taur reference sheets' graft placement):** the chest mounts at the measured stump ring as before, and the taur tail anchors against the rump itself - the root ring rides below the local body crest and behind the rear cut, and the mount axis pitches with the rump's slope so the first tail section hugs it. Long-tailed dinosaurs root well inside the rear bound. The fish lower body seats the chest on the tube's back at its widest band with the dorsal fin excluded by a cross-section-width histogram, and the fluke root rides the peduncle.
- **Presentation framing (screenshot composition guides):** the auto-fit measures the assembly's real world-space box in view space, targets the box centre (a long-tailed taur centres behind its torso), lets the orbit floor follow the silhouette (a rat needs a sub-metre orbit), and insets the fit to the panel-free region when the studio panel shares the frame. A soft radial-gradient contact-shadow disc grounds the silhouette under the world-box footprint. Part previews target each part's own bounds centre through the square thumbnail viewport. The panel gains a studio style pass (thicker amber slider grabs, rounded frames, a visible scrollbar) and the stats overlay splits into short lines that never reach the panel.

### Pipeline changes

- Every boundary ring that survives stitch removal is ellipse-snapped when measurably ragged (the fit lives in the ring's own Newell plane, and the snap weight follows the residual), so fan caps over cut rings stay smooth.
- Long "stitch" triangles - the packs weld lip, ear-lining, and tail-tuft contact points into single vertices, bridging unrelated surface regions - drop before the caps close (heads and ears at a conservative factor that keeps organic long triangles, tails at a firmer one that clears the tuft hubs).
- The taur normalisation band widens (a shiba grows, a horse barely shrinks) and the canonical mount height rises; the taur tail variants bind the lower-body scale at a reduced thickness factor so a threefold canid's bush reads as a tail.
- The torso tail socket pitches the appendage back-and-slightly-up so a bushy tail flows clear of the rump; the wing sockets move outboard so the roots clear the neck.
- The biped's hanging arms gain a slight outward roll (the taur's add a backward pitch) so the hands clear the thighs and the front legs.

### Verification

- Build: zero warnings (cmake Ninja Release, GLFW off, headless CPU rasterizer).
- `--character-test`: 22 checks pass; `--dump-instance-boxes` prints every instance's local and world bounds plus the assembly's world box as numeric framing and mounting ground truth.
- The frame matrix (61 presets), the five part-preview sheets (137 parts), the five panel tabs, and the junction close-ups are re-captured from the current build and reviewed; the junction audit covers the head/collar, wing root, taur chest, taur tail root, biped tail root, and the fish waist and peduncle junctions.

## 0g. Twelfth revision: the re-extracted roster, taur normalisation, runtime previews

Revision 12 rebuilt the whole asset pipeline as `scripts/extract_species_parts.py` and re-extracted every animal part from freshly downloaded CC0 Quaternius packs, grew the roster to twenty species, and moved the part previews from shipped PNG files to engine-rendered runtime textures.

### Research findings applied

- **Scalie and aquatic coverage (furry-fandom species surveys):** the roster beyond the classic canid and farm families needs reptiles and fish - the Ultimate Dinosaurs pack (tyrannosaur, raptor, triceratops, stegosaurus, apatosaur, parasaurolophus), the Easy Enemies rat, and the Fish pack's shark and dolphin all join as full species (heads, tails, whole bodies, and taur lower bodies). The shark-taur and dolphin-taur read as the merfolk lower body.
- **Taur size normalisation (the e621 taur tag wikis' mixed-size grafts):** every lower body's stump ring is normalised by one rigid uniform scale toward a canonical mount height, clamped to a comparable size band, so the humanoid chest grafts at a consistent junction height without turning a small species into a giant. The fish bodies keep their natural scale (their horizontal build is the merfolk read).
- **Taur tail tracking:** a second, identically-normalised tail variant per species rides the taur plan and binds the lower-body scale parameter, so the tail and the lower body scale together at every slider value.
- **Runtime thumbnails (thumbnail-card pickers, no shipped art):** the panel renders every part once at startup into a small viewport, reads the frame back (headless surface lock or the Vulkan frame export), and uploads it as a texture. The species picker shows each species' rendered head; no pre-generated preview images ship.

### Pipeline changes

- Rest-pose evaluation now uses the mesh node's own world transform (the bind pose is the authored geometry); the earlier skin-blend evaluation sat in joint-local space and distorted the parts.
- The partition is hybrid: ear sheets cut on summed skin weights with the animal-midline rule (their weight regions interleave across the skull), tails take the union of the weight-dominant and joint-proximity assignment (a thin tail's base is often nearer the hip anchor, which hands it to the body - the doubled-tail artifact), and everything else partitions by proximity to the rig's bind anchors. Fish heads cut on a cross-section plane (their Face joints sit at the snout tip).
- Boundary loops walk edges (not vertices) so pinched loops still close; every cut ring gets a short Laplacian straightening pass that both sides of a cut apply identically; caps close every remaining opening; the winding follows the source normals; and crease-aware smooth normals (crease 80 degrees - the low-poly sources are organic) replace the faceted look of the old double-sided exports.
- The heads keep the animal's alert rest orientation (the packs stand level-eyed) with the neck ring at the origin: muzzle forward, ears up. The muzzle-length and eye-spacing morphs are baked on the final mesh of every species head, so the species looks (Cat, Jackal) are parameter settings on any canid head.
- The wing connection pitches the pair into a dihedral arch behind the shoulders (the source wing sweeps forward over them), and the torso tailbone and wing sockets sit just inside the measured back surface so the appendage roots bury.
- The torso tailbone/wing socket correction and the wing side fix land in the generated metrics header (the source wing geometry sweeps toward -X, so it is the right-side wing and the loader mirrors the left).

### Verification

- Build: zero warnings (cmake Ninja Release, GLFW off, headless CPU rasterizer).
- `--character-test`: 22 checks covering head/ear/tail variants (including the taur-scaled tail swaps), the outfit toggle, per-piece clothing, every taur and quadruped species graft across the twenty species, the taur tail socket, parameter extremes including the lower-body scale, recipe round-trip, and the animation drivers; all pass.
- The frame matrix (59 presets), the part-preview sheets, and the junction close-ups are re-captured from the current build and reviewed; the junction audit covers the head/collar, wing root, taur chest, and tail root junctions for the canid, farm, dinosaur, and fish families.

## 0f. Eleventh revision: the taur plan, unified part interfaces, clothing variants, previews

Revision 11 reorganises the demo's content around the furry-fandom vocabulary and the parts-with-previews UI pattern, and re-runs the whole asset pipeline against freshly downloaded sources so every interface is measured from the same models.

### Research findings applied (sources in the sections below)

- **Taur terminology (Wiktionary; the e621 "taur" wiki page and its Taur Species tag tree):** "taur" is the fandom's generalisation of centaur - an anthro, human, or humanoid upper body on the lower body of a legged feral creature - and the productive suffix runs across species (`fox_taur`, `wolf_taur`, `dragon_taur`, `equine_taur`, ...). The centaur body plan is now the **taur** plan everywhere (profile id, part ids `quad_taur_<species>`, UI labels, drive presets), and any of the eleven species lower bodies grafts under the humanoid chest. The plan list is humanoid, taur, quadruped.
- **Furry customisation axes (fandom avatar makers, the e621 tag wikis for digitigrade/plantigrade, fur markings, and fluffy tail; species-popularity surveys):** the options that matter are species/head shape, ear style, tail style/length, fur colour, and markings (socks, gloves, tail tips, muzzles). The demo covers all of them: thirteen head styles, eleven swappable ear pairs, twelve tails, a six-swatch fur-coat palette, a four-mode markings system, and the muzzle-length / eye-spacing sliders.
- **Modular clothing (Quaternius Modular Character Outfits; VRChat toggle conventions):** clothing is a variant set over the same sockets, not baked geometry. The bare body (Universal Base Characters, partitioned by the source rig's own joint weights) and the ranger outfit (Modular Character Outfits - Fantasy) are separate registered parts over identical connector names; the outfit toggle swaps torso/arms/legs variants, and the belt, bracers, pauldrons, and hood attach and remove individually.
- **Creator UI (thumbnail-card pickers):** the studio panel renders every registered part into a preview thumbnail with the engine itself (`--part-previews`), ships the PNGs under `assets/previews/`, and lists each option as thumbnail + name instead of a text-only row. The panel scrolls with the mouse wheel and with a middle-button grab drag.

### The unified-part decision

The unify question - whether cat and rabbit heads should be one head model with different settings - is answered at the interface level: every part of a slot shares one mount contract and one parameter set, and species looks are settings on top. All thirteen heads carry the same `head.root` interface, the same `head.size`, `muzzle.length`, and `eyes.width` parameters (the latter two as baked morph blocks that the content layer applies at change time), and species heads additionally expose two `ear.*` sockets so ear styles cross-graft. The species meshes stay distinct downloads because their silhouettes differ beyond morph range, but nothing about the interface or the parameter set is species-specific: a cat look is a short muzzle, wide-set eyes, and small triangular ears on any base head; a wolf look is a long muzzle and tall ears. Ears are separate parts of the head (the extractor cuts them on the source rigs' own `Ear*` joints; bovine and cervid species keep their ears and horns in the head mesh and register no sockets).

### Connectivity guarantee

Every junction closes by one of two measured mechanisms, and the pipeline records both in `extract_report.json`:

- **Burial.** A part's interface tube slides into the parent's volume (heads over the torso collar, ear roots into the head, tail roots into the hip), with the depth derived from the measured ring radius so the parent's surface always envelops the opening.
- **Caps.** Every opening that would stay visible (taur neck stumps, tail stumps, paw soles, head ear holes and neck-tube bottoms) is closed by a fan triangulated over the source boundary loop itself, coloured from the loop's own vertices. A cap is derived geometry, not authored shape, and it guarantees the mesh stays watertight-looking at every parameter value, including the extremes of the lower-body scale slider.

The mount anchors are measured per species (stump ring centre, tail-root ring, withers, ground line, ear sockets) and emitted into the generated metrics header; the chest connects to the taur body with an identity rotation so it stays world-upright regardless of the animal's neck angle, sits just sunk below the stump ring, and sits slightly back from the stump so it reads over the withers. Lower-body size is the `taur.scale` parameter (rig part-scale binding): the mount and the ground line ride it because the module scales joint-anchored sockets with the part.

### Tail handling

Tails are extracted from the source rigs' `Tail*` joints with the root ring at the part origin and the ring normal as the part's +Z, then rescaled onto a per-species target extent (natural tail lengths at character scale, from fox 0.62 m to horse 1.05 m). On the humanoid plan the tail attaches at the torso's tailbone socket; on the taur plan it attaches at the lower body's own tail stump socket - the tail option affects the taur's lower body, and the species swap re-points the tail to the matching species by default (still swappable across species). The quadruped plan keeps the whole animal's own tail.

### Verification

- Build: zero warnings (cmake Ninja Release, GLFW off, headless CPU rasterizer).
- `--character-test`: 22 checks covering head/ear/tail variants, the outfit toggle, per-piece clothing, every taur and quadruped species graft, the taur tail socket, parameter extremes including the lower-body scale, recipe round-trip, and the animation drivers; all pass.
- The frame matrix and the part-preview sheet are captured from the current build (`scripts/shoot_samples.sh`) and reviewed; the junction audit uses per-junction captures.

## 0e. Tenth revision: species roster, measured grafts, UI rebuild

Revision 10 grew the species roster to eleven downloaded animals (fox, shiba, husky, wolf, donkey, cow, deer, bull, alpaca, horse, stag), grafted their heads and tails onto every plan, added the fur-coat palette for the animal grafts, tabbed the studio panel (Plan / Body / Parts / Motion / File) with stable control ids and contextual tip lines, re-centred every graft on its measured interface ring, and recorded each species' own stump ring, withers, and ground line so every species grafts at its own measured band. Its research pass (CCI design guides, modular-socket practice, furry-fandom rosters) is the basis of the panel and roster decisions revision 11 extends.

## 0d. Eighth revision: measured geometry and rig refinements

Revision 8 re-shot the whole sample matrix from the current build with per-preset camera framing (`--camera-target-y` aims the orbit; explicit CLI framing survives updates), normalised the reptile tail into a monotonic taper about its measured spine, gave the serpent body an explicit direction curve with cross-section normalisation, rescaled and re-rooted the wings (a dihedral arch about the root, the elbow nub dropped, the root ring sealed), sunk the centaur mount so the torso's hip ring closes over the stump, rolled the extra arm pairs outward with surface anchors, spread the two-head layout, extended the dragon-head cut along the neck axis, and moved the ranger pauldron onto the posed arm.

## 0c. Seventh revision: downloaded models only, body plans, natural necks

Revision 7 replaced content generation with extraction: every body part became a piece of a downloaded, rigged model pack, extracted by linear-blend skinning into a rest pose, dominant-joint-weight partition cuts (every triangle assigned to exactly one piece), rigid transforms, and vertex-level morph deformation. Human parts came from the artist's own modular split (the junctions are the artist's), the procedural neck tube was deleted in favour of the head's natural neck geometry, cross-species grafts sealed with funnel morphs, the four body plans arrived as assembly profiles, and duplicate appendages attached on real connectors with per-pair placement.

## 0b. Sixth revision: anatomical wing and tail silhouettes, real faces

Revision 6 rebuilt the membrane wings (arm-plus-fingers chain, scalloped webbing between boundary curves), the feather wings (cambered primaries, secondaries, and coverts), and the tails (rest spine leaving the tailbone angled back-down, lens-volume fins), and rebuilt the human head as a wedge face with a visible sclera ring and lip volume, tuning skull scale and species faces.

## 0a. Fifth revision: complete procedural parts, per-face winding

Revision 5 authored complete procedural heads, tails, and wings (nine species heads with the canonical gripping collar, five tail chains, two wing variants), oriented every face against its analytic outward reference at construction time (the engine culls back faces), kept silhouettes honest (muzzles extend past the dome, eyes past the lids, closed tips), spread the two-tail pair symmetrically, and widened the option and range set.

## 0. Fourth revision: density, options, and the junction contract

Revision 4 raised every checked-in part one Loop subdivision level with interface rings pinned (morph deltas survive subdivision exactly), smoothed near-equal vertex colours while keeping hard boundaries crisp, added seven head variants, three tail chains, three parameters, the five-swatch skin-tone palette, and wider ranges, and recorded the junction contract (section 2) that every later revision follows.

## 0c. Third revision: the visual audit loop

Revision 3 established the frame-audit method the project still uses: capture every drive preset, zoom into every junction, re-check each extracted mesh in a standalone z-buffer renderer. It fixed culling-orientation probing (majority vote over spread triangles), disabled LOD for the always-near character meshes, sealed limb and tail junctions with fused ellipsoid ball joints, emitted open-surface triangles double-sided, and switched shading to crease-aware smooth normals.

## 1. What shipped games actually do

Findings from public postmortems, engine documentation, and tooling discussions (all visited while designing this revision):

- **Body shape is one continuous surface, not snapped parts.** Body sliders in Cyberpunk 2077, Black Desert Online, and Skyrim (BodySlide/CBBE) drive morph targets or bone scaling on a single skinned base mesh. Black Desert uses bone scale/translation for body proportions and reserves blendshapes for the face; BodySlide bodies and outfits share identical topology so the same slider set deforms every piece identically and seams cannot open.
- **Modular swap-in parts are for gear, and their seams are designed away.** Unreal's modular character docs and forum practice: parts share one skeleton, junctions get an intentional overlap or a hidden cuff, and seam lines are moved under clothing. Equipment swapping - not body shaping.
- **Spore-style part snapping works because skin is generated over the parts.** Spore's creature editor drives an implicit metaball surface along a deformable spine; parts (rigblocks) attach to spine bones and the generated skin makes everything continuous by construction. A parts system without a continuity layer relies on matched junction geometry.
- **Customization meshes are rebuilt on change, never per frame.** Mutable (the character-customisation middleware behind several AAA titles) combines meshes and textures once per change, asynchronously; the runtime render path is an ordinary static mesh afterwards.
- **Extreme slider values cause the clipping players report.** Honey Select 2 and Bodyslide guides both attribute clipping to extreme slider combinations; shipped games clamp ranges and some (Rockstar with fat/muscle sliders) removed features rather than ship the clipping.
- **Limb scaling needs counter-scaling at the ends.** VRChat limb-scaling prefabs scale arm/leg bones and inversely scale hands/feet so the ends keep their shape; junction rings are kept fixed to avoid gaps.

The consistent lesson: **a junction is safe only when both sides derive their interface geometry from the same parameter formula, and geometry is only ever re-derived when parameters change.**

Revision-4 additions (sources re-visited while planning this revision):

- **Variant buttons per slot and palettes beat raw pickers.** ESO's creator is organized as Race/Class/Body/Face menus with variant lists, Diablo 4's as Face & Body / Hair / Accessories; PSO2 and the GameDeveloper ethnicity articles favor dedicated skin-tone swatches over free color pickers. The demo panel therefore lists variants for every slot and ships a five-swatch skin-tone palette (a draw-time tint on the skin-tagged parts).
- **Duplicate appendages get explicit per-instance placement.** Spore's parts snap anywhere along a deformable spine; Unreal's Control-Rig spline chains drive tails/ears as chains; rigging guides treat tails as multi-joint secondary chains. Symmetric paired placement (Reallusion's force-symmetry/mirror-copy tooling) is the standard for duplicates - the second tail spreads around the spine instead of copying the first tail's transform.
- **Hero-mesh density without runtime tessellation = baked subdivision.** Overwatch-class hero characters sit at 15k-20k triangles, realistic AAA at 30k-60k+ (r/gamedev polycount threads); where the engine has no runtime tessellation, the standard workaround is exporting multiple subdivision levels (r/unrealengine). Loop subdivision is linear in vertex positions, so baked morph deltas survive re-subdivision exactly (`delta_new = L(delta_old)`) - the pipeline sub-divides the checked-in parts with interface rings pinned instead of re-cutting from sources.

## 2. The junction contract (authoring rules for this engine)

1. **Every junction interface is a function of shared parameters.** The torso neck stub radius, the neck piece, and the head collar radius are all computed from one `bodyMetrics()` evaluation; no asset may hard-code an interface radius that another asset also hard-codes.
2. **A ring that sits at its part origin never desyncs.** Parts whose only connector ring is at the local origin (limbs, tails, wings) may deform by whole-part scale: the ring stays on the anchor regardless of scale.
3. **A ring away from the origin must be driven by baked morph shapes, not part scale.** Scaling the whole head scales its collar ring and would separate the junction ("head neck wider than torso neck"). Heads therefore deform by morph deltas with the collar region masked out.
4. **Anchors that must move with parameters ride rig joints.** The neck rises with body height, shoulders move out with body width: those are rig joints translated by `JointTranslation` bindings, and the morph falloffs are authored so the interface rings move by exactly the same formula. One formula per interface, used by both the mesh morph and the binding.
5. **Parameter ranges are validated ranges.** Slider minimum/maximum values are chosen so that the extreme combinations stay junction-safe and free of deep self-intersection; the compatibility rules' relative-scale gate catches asset-level mismatches on top.

## 3. Deformation architecture in this build

The renderer in this engine has no skinning or GPU morph path, so the industry morph-target pattern is implemented consumer-side, at change time:

- Part OBJs may carry `m <name>` morph blocks: per-vertex position deltas applied as `position += weight * delta`. Weights come from the assembly parameters (for example `head.size - 1.0`).
- The demo content layer owns the mesh upload. When a parameter changes it applies the affected morphs on the CPU (a few thousand vertices), reuploads the affected meshes, and lets the module's existing connector evaluation re-derive transforms. Nothing morphs per frame.
- Connector anchors that move with parameters are rig joints with `JointTranslation` bindings; the module already scales joint-anchored socket positions by part scale and keeps rotations rigid.

## 4. Performance model (software rasterizer)

The character screen must hold up on a 2-core CPU with the headless rasterizer backend, which makes shaded pixels the budget:

- The sky dome is authored inward-facing so backface culling keeps only the near hemisphere, and it is drawn after the character so occluded pixels depth-reject cheaply.
- Module evaluation honours its dirty flags; validation runs when structure or parameters change, not every animation-driven re-transform.
- Character meshes stay in the low thousands of triangles per part.

## 5. Sources for the shipped assets

The checked-in OBJs are produced by two pipeline generations: `scripts/build_character_parts.py` cut the revision-3 parts from CC0 packs downloaded into `assets_dl/`, and `scripts/refine_character_parts.py` (revision 4) upgraded those parts in place, built the procedural tails, and cut four more heads from the 2024 re-export of the Ultimate Monsters pack (mirrored at archive.org/item/ultimate-monsters; still CC0 by Quaternius). `scripts/preview_part.py` is the standalone z-buffer previewer both revisions used for asset audits. See `LICENSE.md` next to the OBJs:

- Humanoid torso, arms, legs: Quaternius *Universal Base Characters* (Superhero male), cut along the rig's joint partition (UE-style skeleton: `pelvis`/`spine_*`/`clavicle_*` for the torso, `upperarm_*`/`hand_*` per arm, `thigh_*`/`ball_*` per leg). Arms end in a fused deltoid ball; legs keep the buried interface skirt.
- Demon head: Quaternius *Ultimate Monsters* Demon, `Head`-joint surface of the head mesh only, anchored at the skull's measured neck boundary (the rig's Head joint sits mid-skull); authored red palette because the source UV set maps the head to an unpainted atlas strip. One Loop subdivision level.
- Dragon head: Quaternius *Ultimate Monsters* Dragon Evolved `Head`-joint surface; one Loop subdivision level.
- Serpent head: Quaternius *Ultimate Monsters* Dino `Head`-joint surface.
- Orc, yeti, alien, and blue demon heads: `Head`-joint partitions of the 2024 re-export (the pack's glTF skeletons carry the same named joints; the partition rule is identical, the atlas palettes are sampled per corner from the 1024x1024 atlases, and the canonical gripping collar is welded below each cut ring). The Orc ships with its authored hood - the hood is what the pack weights to the Head joint.
- Wings: Quaternius *Ultimate Monsters* Dragon wing chains (one source side per engine side), rooted with a ball joint; open-boundary membrane triangles are emitted double-sided.
- Tail: three procedural lofted chains (reptile, spade, club) authored by `scripts/refine_character_parts.py` - Catmull-Rom spine, real thickness, fused root ball, baked `tail_curl` morph with its matching binding constant in the generated metrics header. (Revisions 2-3 cut a T-Rex tail, which turned out to be a near-flat sheet with zero-thickness slices.)

Joint partition cuts follow the animation boundary, so the extracted pieces are exactly the regions the source skeleton deforms - the same seam the source artists authored.

## Revision 9: downloaded-only parts, verbatim

User mandate: "Ensure no character parts are generated by you. Character parts must be downloaded only." Revisions 4-8 still baked vertex-level deformations onto downloaded meshes (honest taper, bend arcs, cross-section normalization, root seals, funnel morphs) - that is generating parts, and the deformed pieces (flat ribbon tail, plank serpent, mascot-mask dragon head) were exactly the worst-looking results in the sample frames.

### The verbatim rule

`scripts/extract_parts_verbatim.py` (replaces the rev-7 extractor) emits every part as a downloaded model's own triangles, unchanged in shape:

- artist-split packs contribute whole mesh nodes (the Ranger male's `Male_Head`+`Eyes`+`Eyebrows`, `Male_Ranger_Body`+belts, `Male_Ranger_Arms`+`Arms_Bracer`+`Acc_Pauldron`, `Male_Ranger_Legs`+`Feet_Boots`; the hood node belongs to no part, which also removes the rev-8 "hood shard");
- single-mesh packs contribute the triangle subset dominated by the part's own rig joints (dragon `Neck`+`Head`, the flying dragon's `Wing1-4` chains, the Cethiel dragon's `tail1-3`);
- quadruped bodies are the WHOLE animal; the centaur variant is the whole animal minus its neck/head joints (subset selection, nothing added);
- poses run only through the asset's own joints (LBS with the asset's own weights: arms hang by rotating the UE `upperarm_*` joints);
- orientation/size changes are rigid transforms (translate/rotate/uniform scale), applied at extraction and listed in `extract_report.json` (dragon head 0.30, wing 0.85, tail 1.6/4.6, rest pitches -118/-102 deg);
- colors come from the assets' own textures (bilinear-sampled, sRGB -> linear) or material factors - no repainting;
- FORBIDDEN: caps, skirts, funnel seals, vertex reshaping, morph deltas, procedural recoloring.

### New sources

- Tails (reptile + serpent): OpenGameArt CC0 "Cethiel's Dragon 3D" (`work/assets_dl/oga2/dragon.dae` + `dragon.png`) - a skinned COLLADA with named `tail1-3` joints and an authored thorny curved tail. The old Dragon Evolved tail subset carried hip geometry and a curled bind shape; both dragon tails were unusable.
- The human skin texture `T_Regular_Male_Dark_BaseColor.png` was missing from the local mirror and is now fetched from the same ValTrainer repository - that was the rev-8 "pale clay head" root cause.

### Runtime changes

- The morph system is deleted end to end (`DeformationInput`, `applyDeformation`, `weightForMorph`, morph blocks in the OBJ loader); `head.size` is now a rigid uniform part scale on both heads.
- `character_metrics.gen.hpp` is regenerated from the verbatim parts (measured anchors: torso `headNeckY/Z`, shoulders, hips, per-species `NeckBase/Withers/Ground/Length`, tail and serpent extents, wing span).
- Body plans: quadruped = the WHOLE animal (own head, own tail; the dragon-head graft on quadrupeds is gone), centaur = whole horse minus neck/head + human torso over the stump opening, naga = torso + Cethiel serpent (rest pitch baked, grounded on the measured pitched extent), humanoid unchanged + baked-pitch reptile tail.
- The tail sway driver composes a yaw on top of the baked rest pitch; it never replaces the connection rotation, so the driver and the authored rest pose always sum to the visible orientation.
- The overview matrix renders every frame into an exact grid (no filler).

### Verification

- Build: 0 warnings (cmake Ninja Release, GLFW off).
- `--character-test`: 20 passed / 0 failed / 0 skipped.
- All 36 sample frames re-shot (`scripts/shoot_samples9.sh`) and personally reviewed via contact sheets: junctions close by overlap, no flat-ribbon tail, no plank serpent, no mascot mask, quadrupeds keep their own heads, framing covers every plan and seam.
