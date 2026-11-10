# planet module configuration

All keys live in the `[planet]` section of the module's `config.toml` (copied into the runtime layout at `config/modules/planet/config.toml`). Every key is optional; missing keys keep the defaults mirrored in `PlanetGenerationParams`. `validate()` clamps every value into a safe range (lengths are capped relative to `radius_meters`, angles normalized, NaN replaced by defaults), so hostile config cannot produce degenerate geometry. Per-planet overrides arrive through the capability's `PlanetDescription` (`seed`, `radius_meters`, `sea_level_offset_meters`, `center_meters`).

The shipped defaults describe the showcase planet: radius 60 km, Earth-like relief fractions (~1% of the radius), large-scale relief layers scaled to the radius, close-range layers metric. Globe views read as a planet, ground views read as a landscape.

## Identity and scale

| Key | Default | Effect |
| --- | --- | --- |
| `seed` | `1337` | Root of every noise stream (SplitMix64 schedule). Changing it reshapes the entire planet. |
| `radius_meters` | `60000.0` | Mean planet radius (the datum sphere). Range [1, 1e9]. Scale the large-scale wavelengths with it (see usage notes). |
| `sea_level_offset_meters` | `0.0` | Raises (+) or lowers (-) the ocean surface relative to the datum. |
| `axis` | `[0, 1, 0]` | Spin axis; latitude for climate is measured against it. Normalized in `validate()`. |

## Octree / LOD

| Key | Default | Effect |
| --- | --- | --- |
| `leaf_grid_n` | `16` | Marching-cubes cells per leaf chunk axis (grid is `(n+1)^3` samples plus a 1-voxel apron). Must be EVEN so chunk faces sit on coarse planes. Range [4, 64]. |
| `target_voxel_meters` | `2.5` | Desired finest voxel size; drives the derived octree depth (`derivedMaxDepth`). The lower clamp is `radius / 131072` (depth-capped by the 20-bit chunk-key packing). |
| `max_depth_limit` | `18` | Hard cap for the octree depth (also capped at 20 by the key packing). |
| `lod_angular_voxel_radians` | `0.02` | Screen-space LOD: refine a node while its voxel subtends more than this many radians at the eye. Detail follows apparent size, independent of planet radius, viewport and zoom. Smaller = finer views everywhere, visible chunk count grows quadratically. |
| `lod_band_limit_factor` | `2.0` | Per-depth low-pass: density content below `voxel * factor` fades out before coarse chunks sample it. Marching cubes needs ~4 samples per wavelength; 2.0 renders content at full strength from 4 voxels up and fades it to zero at 2 voxels. Without it, sub-voxel octaves alias into random spikes on every coarse LOD surface (the "spiky globe"). Raise it to trade close-range roughness for cheaper, smoother chunks. |
| `horizon_culling` | `true` | Skip nodes fully inside the shadow cone of the guaranteed-solid inner shell ball. |
| `bounding_factor` | `1.4` | Root cube half-size = `radius * factor`. Range [1.05, 4]. |
| `max_concurrent_builds` | `16` | Chunk mesh builds in flight at once (the pool runs wide open; ground views select thousands of chunks). |
| `max_new_builds_per_update` | `48` | Build requests issued per `updatePlanet` call. |
| `chunk_eviction_updates` | `240` | Updates a hidden mesh stays cached before eviction. |
| `chunk_cache_capacity` | `16384` | Hard cap on cached chunk meshes (oldest evicted first). Must exceed the whole visible set on the deepest ground views (~9000 chunks at radius 60000) or ready meshes evict and rebuild in a churn loop. |

## Relief shaping (meters unless noted)

Large-scale layers (continents ... regional) carry wavelengths scaled to the 60 km radius so coarse globe chunks can resolve them; close-range layers (hills and finer) stay metric.

| Key | Default | Effect |
| --- | --- | --- |
| `continent_amplitude_meters` | `560.0` | Height of the continent base field. |
| `continent_wavelength_meters` | `34000.0` | Feature scale of the continent field. |
| `continent_octaves` | `4` | fBm octaves of the continent field. Fewer octaves = fewer medium patches: landmass cores stay contiguous; coastline complexity comes from the warp plus near-shore hills. |
| `domain_warp_amplitude` | `0.18` | ANGULAR coastline bending (bends features by roughly amplitude * R meters): 0.18 shears coastlines ~10 km on the 60 km planet without shredding landmasses into filaments (0.5 gives archipelagos). |
| `domain_warp_wavelength_meters` | `14000.0` | Scale of the warp field. |
| `mountain_amplitude_meters` | `1100.0` | Ridge height where the mountain mask saturates. |
| `mountain_wavelength_meters` | `9000.0` | Crest scale of the ridged field. |
| `mountain_mask_low` / `mountain_mask_high` | `0.08` / `0.45` | Continent-field values where mountains start / reach full height. |
| `mountain_sharpness` | `1.25` | Exponent applied to the ridged field. |
| `mountain_octaves` | `5` | fBm octaves of the ridged field. |
| `mountain_mass_fraction` | `0.50` | Amplitude share of the broad smooth mass field (the rest goes to the ridged crest field; a pure ridge sum reads as uniform needle carpets). |
| `range_cluster_strength` | `0.55` | How strongly uplift groups into distinct ranges with foothill gaps. |
| `range_cluster_wavelength_meters` | `20000.0` | Scale of the range-cluster field. |
| `regional_amplitude_meters` | `80.0` | Broad plateaus/basins/valley systems between the continent and hill bands - the landform-structure layer at globe scale. |
| `regional_wavelength_meters` | `6500.0` | Scale of the regional field. |
| `regional_octaves` | `3` | fBm octaves of the regional field. |
| `hill_amplitude_meters` | `110.0` | Medium-frequency rolling terrain (metric; radius-independent). |
| `hill_wavelength_meters` | `1000.0` | Hill scale. |
| `hill_octaves` | `5` | fBm octaves of the hills. |
| `detail_amplitude_meters` | `14.0` | Fine detail layer: surface roughness between the hill wavelength and the voxel size (rock ribs, soil ripples, scree). Weighted like the overhang term: mountains and cliffs carry full strength, plains keep `detail_plains_fraction`. |
| `detail_wavelength_meters` | `120.0` | Feature size of the detail layer's base octave. Keep the finest octave (base / 2^(octaves-1)) at 2.5+ voxel sizes or coarse LOD chunks alias it into lumps. |
| `detail_plains_fraction` | `0.22` | Detail strength kept on flat ground (above ~0.3 boils plains into noise lumps). |
| `detail_octaves` | `4` | fBm octaves of the detail layer. |
| `terrace_strength` | `0.60` | 0 = smooth terrain, 1 = full plateaus (mesa country). Terracing is additionally gated to genuinely terraced country (cliff mask) inside the field, so open slopes never corduroy. |
| `terrace_steps` | `9.0` | Approximate plateau count across the relief range. |
| `cliff_wavelength_meters` | `1400.0` | Scale of the terrace mask field. |

## True-3D terms (what a heightmap cannot do)

| Key | Default | Effect |
| --- | --- | --- |
| `overhang_amplitude_meters` | `38.0` | 3D noise added to the signed density; displaces the surface sideways (ledges, overhangs). Fades out underwater and near the shoreline. |
| `overhang_wavelength_meters` | `300.0` | Feature size of the overhang noise. Large amplitudes at short wavelengths boil the surface into lumps. |
| `overhang_plains_fraction` | `0.06` | Fraction of the overhang strength kept on flat ground (0 = only mountains/cliffs displace in 3D). |
| `strata_amplitude_meters` | `3.0` | Banded rock strata on cliff faces (gated to high cliff-mask country inside the field). |
| `strata_band_meters` | `34.0` | Strata band spacing. |
| `strata_warp` | `1.1` | Noise bending of the bands. Values >= 2 fold the sinusoid into nested ripple rings (an artefact, not geology). |
| `coastal_flatten_meters` | `80.0` | Half-width of the flattening band around sea level: detail fields (hills, terraces, overhangs, strata) are damped there so shorelines slope into the sea instead of zig-zagging across it (a zig-zag reads as a web of sub-sea ponds with beach rims). |
| `coastal_flatten_floor` | `0.10` | Residual detail fraction right at the shoreline. |
| `caves_enabled` | `false` | Cave worm carving (two extra noise fields per density sample). |
| `cave_threshold` | `0.16` | Worm field magnitude below which material is carved. |
| `cave_worm_wavelength_meters` | `240.0` | Tunnel feature scale. |
| `cave_carve_meters` | `60.0` | Maximum carve depth. |

## Oceans

| Key | Default | Effect |
| --- | --- | --- |
| `shallow_water_meters` | `18.0` | Shoreline band classified as shallow ocean. |
| `deep_water_meters` | `140.0` | Depth where the ocean counts as deep. |
| `water_depth_tint_meters` | `70.0` | Depth over which the water surface tint saturates. |
| `water_enabled` | `true` | Generate water surface patches (marching cubes on the sea sphere). |
| `water_color_shallow` | `[0.060, 0.205, 0.275]` | Linear RGB at zero depth. |
| `water_color_deep` | `[0.012, 0.060, 0.130]` | Linear RGB at full tint depth. |
| `water_roughness` | `0.42` | Gloss hint handed to the renderer's water material. Low values give mirror-sharp sun glints, which makes the coarse far-water facets band at grazing angles; higher values spread the highlight. |

## Climate and biomes

| Key | Default | Effect |
| --- | --- | --- |
| `equator_temperature` | `1.0` | temperature01 at the equator, sea level. |
| `pole_temperature` | `0.02` | temperature01 at the poles, sea level. |
| `temperature_perturbation` | `0.09` | Amplitude of the wandering temperature noise. |
| `temperature_wavelength_meters` | `20000.0` | Scale of the temperature noise. |
| `temperature_lapse_meters` | `1100.0` | Meters of altitude per 1.0 temperature01. |
| `moisture_wavelength_meters` | `14000.0` | Scale of the humidity noise. |
| `moisture_perturbation` | `0.24` | Amplitude of the humidity noise. |
| `moisture_warp_amplitude` | `0.22` | Domain-warp of the humidity field: bends biome patches into elongated, organic shapes (unwarped, patches read as round camouflage blobs). |
| `coastal_moisture_boost` | `0.18` | Onshore humidity: low land near the sea stays green (without it the shoreline belt paints a beige desert fringe around every landmass). |
| `coastal_moisture_band_meters` | `400.0` | Elevation over which the coastal moisture boost fades out. |
| `moisture_latitude_bands` | `true` | ITCZ wet belt + subtropical dry belts + temperate wet belt. |
| `color_variation` | `0.14` | Per-point albedo jitter (0 = flat palette). |
| `snow_line_meters` | `1600.0` | Sea-level snow line; drops as the local climate cools. Only true peaks are glaciated; polar caps come from latitude alone. |
| `beach_width_meters` | `2.0` | Shoreline band above sea level. Kept narrow: on flattened coastal plains a wide band paints every drainage dip as sand. |
| `rock_slope_threshold` | `0.62` | Slope (1 - dot(normal, radial)) where bare rock takes over (about 32 degrees). Low enough for genuine cliff faces to show rock, high enough that aliased ridge crests stay snow-covered instead of speckling with rock dots. |
| `rock_strata_contrast` | `0.10` | Luminance banding of rock faces at the strata band spacing (sediment stripes). 0 disables; keyed to `strata_band_meters`. |
| `biome_blend_width` | `0.06` | Half-width of the biome color blend on the temperature/moisture axes. Biome colors interpolate between palette rows instead of hard-switching per vertex (hard switches smear into pale stains across triangles). |
| `slope_blend_width` | `0.08` | Blend half-width around the rock slope threshold. |
| `elevation_blend_fraction` | `0.12` | Snow-line and water-band color blends as a fraction of the band width. |
| `biome_palette` | 16 rows | Linear RGB rows in `Biome` enum order (deep ocean .. rock); the ocean-floor rows sit deliberately darker than the water surface colors. Entries may be omitted from the end. |

## Usage notes

- **Scaling a planet.** Raise `radius_meters` and scale the wavelengths you want to keep proportional (continents, mountains, regional, climate fields, snow line); close-range amplitudes stay metric (a 650 m mountain is a 650 m mountain on any planet). The domain warp amplitude is ANGULAR - its metric effect grows with R (0.18 bends ~10 km coastlines at 60 km radius). The derived octree depth grows automatically until voxels reach `target_voxel_meters` (or the depth cap); `validate()`'s lower voxel clamp is `radius / 131072`.
- **Performance knobs.** Visible chunk count scales roughly with `1 / target_voxel_meters^2` near the camera and with `lod_angular_voxel_radians` overall. `leaf_grid_n` trades chunk granularity against per-chunk cost. `max_concurrent_builds` / `max_new_builds_per_update` pace streaming; caves cost two extra noise fields per sample.
- **Chunk boundaries and seams.** Same-depth chunks weld bit-identically (one global lattice); across depth changes the octree keeps a 2:1 balance and the fine side CONFORMS its boundary-plane samples to the coarse lattice's interpolant on segments whose far cell is a rendered coarse leaf, then EXTENDS its surface one cell layer past those faces - the fine and coarse surfaces interpenetrate along the whole transition, so no viewing ray can slip between them. Conform masks are recomputed whenever the visible set changes; chunks whose stored mesh was built under a different mask rebuild automatically (the stale mesh keeps rendering until the rebuild lands).
- **Surface normals.** Vertex normals average the gradient of the (up to) four cells adjacent to the edge, in canonical min-corner form - a pure function of the edge's global position, so shared boundary vertices stay bit-identical across chunks. The average removes the per-cell normal steps that used to speckle steep creases with dark facets.
- **Water patches.** Water depth tint and foam sample `densityAt` directly (one pure field), so the tint is continuous across chunk and LOD boundaries; cells fully buried under terrain are skipped, which removes grazing-angle slivers where the sea sphere grazes the surface.
- **Surface artefacts checklist.** Pale stains on green terrain: raise `biome_blend_width`/`slope_blend_width`. Dotted dark lines along ridges: raise `rock_slope_threshold`. Shredded cliff faces: lower `strata_warp`, raise `strata_band_meters`, lower `overhang_amplitude_meters`. Sub-sea pond webs near coasts: raise `coastal_flatten_meters`. Spiky silhouettes at any scale: raise `lod_band_limit_factor` (and keep the finest detail octave at 2.5+ voxels).
