# planet module configuration

All keys live in the `[planet]` section of the module's `config.toml` (copied into the runtime layout at `config/modules/planet/config.toml`). Every key is optional; missing keys keep the defaults mirrored in `PlanetGenerationParams`. `validate()` clamps every value into a safe range (lengths are capped relative to `radius_meters`, angles normalized, NaN replaced by defaults), so hostile config cannot produce degenerate geometry. Per-planet overrides arrive through the capability's `PlanetDescription` (`seed`, `radius_meters`, `sea_level_offset_meters`, `center_meters`).

The shipped defaults describe the showcase planet: Earth's volumetric mean radius (6,371 km), real-Earth relief fractions (the mountain term's 3 km crests are ~0.05% of the radius, where Everest sits at 0.14%), large-scale relief layers scaled to the radius, close-range layers metric. Globe views read as a real planet (a smooth disc whose mountains photograph as shading), ground views read as a landscape, and a standing eye meets its horizon at the ~4.7 km a real 1.7 m eye sees.

## Identity and scale

| Key | Default | Effect |
| --- | --- | --- |
| `seed` | `1337` | Root of every noise stream (SplitMix64 schedule). Changing it reshapes the entire planet. |
| `radius_meters` | `6371000.0` | Mean planet radius (the datum sphere; the default is Earth's volumetric mean radius). Range [1, 1e9]. Scale the large-scale wavelengths with it (see usage notes). |
| `sea_level_offset_meters` | `0.0` | Raises (+) or lowers (-) the ocean surface relative to the datum. Clamped into the terrain's relief band: the sea sphere must stay inside the shell the octree prunes against, so offsets beyond the relief amplitudes are pulled back in. |
| `axis` | `[0, 1, 0]` | Spin axis; latitude for climate is measured against it. Normalized in `validate()`. |

## Octree / LOD

| Key | Default | Effect |
| --- | --- | --- |
| `leaf_grid_n` | `16` | Marching-cubes cells per leaf chunk axis (grid is `(n+1)^3` samples plus a 1-voxel apron). Must be EVEN so chunk faces sit on coarse planes. Range [4, 64]. |
| `target_voxel_meters` | `2.0` | Desired finest voxel size; drives the derived octree depth (`derivedMaxDepth`). The lower clamp is `radius / 4194304` (depth-capped by the chunk key's 19-level limit). |
| `max_depth_limit` | `19` | Hard cap for the octree depth. The chunk key stores the depth in 5 bits and its lattice coordinates in 19 bits, so 19 is also the structural maximum. |
| `selection_update_distance_meters` | `0` | Eye movement below which `updatePlanet` keeps the previous visible selection (the streaming passes still run). `0` derives the finest voxel: a smaller step cannot change any LOD decision, so a walking camera re-selects every few dozen frames instead of every frame. | 
| `node_record_lifetime_updates` | `96` | Selections an octree node record (its solid/air probe verdict) survives before the prune pass drops it. Longer lifetimes keep revisited views free of the first-visit field probes; the resident cost is a few dozen bytes per record. |
| `node_record_prune_interval_updates` | `16` | Cadence of the record prune pass (clamped to the lifetime so visible records never age out between scans). |
| `lod_angular_voxel_radians` | `0.02` | Screen-space LOD: refine a node while its voxel subtends more than this many radians at the eye. Detail follows apparent size, independent of planet radius, viewport and zoom. Smaller = finer views everywhere, visible chunk count grows quadratically. |
| `lod_band_limit_factor` | `3.0` | Per-depth low-pass: density content below `voxel * factor` fades out before coarse chunks sample it. Marching cubes needs ~4 samples per wavelength; 3.0 renders content at full strength from 6 voxels up and fades it to zero at 3 voxels. Without it, sub-voxel octaves alias into random spikes on every coarse LOD surface (the "spiky globe"), and marginal 3-4-voxel belts alias into shimmering shard facets - the factor keeps both out of the mesh entirely. Raise it to trade close-range roughness for cheaper, smoother chunks. |
| `horizon_culling` | `true` | Skip nodes fully occluded behind the planet: everything below the guaranteed-solid inner shell ball, plus - for an eye above the water - everything behind the sea sphere (the opaque water sheet occludes the underwater half of the shell at the waterline instead of at the deep floor). |
| `lod_missing_fallback` | `true` | Streaming coverage for visible chunks whose mesh has not finished building: such a chunk draws its nearest finished ancestor whose chunk width fits the hole's distance (the stand-in's width stays within twice that distance, so its on-screen extent stays bounded) instead of contributing no geometry, and the streamer additionally requests the chunk's coarsest pure ancestor (the coarsest one passing the screen-space rule and the width bound under which no finished visible chunk lies, so its mesh covers the hole without overlapping finished geometry). While holes still lack stand-ins, the scaffold builds go ahead of the fine ones (an umbrella chunk covers every hole in its region; a fine chunk covers one), and once coverage is complete the apparent-screen-size order resumes. The planet therefore appears coarse-first - the widest gaps stream the widest chunks - and sharpens in place as the fine meshes land, so a fresh view never waits on the build queue before showing terrain. |
| `lod_scaffold_builds_per_update` | `8` | Scaffold request budget per `updatePlanet` call (see `lod_missing_fallback`): at most this many pure-ancestor builds are requested per update; while uncovered holes remain they are issued ahead of the fine requests so the coverage umbrella forms first, and afterwards they compete on apparent screen size in the same order as the fine chunks. `0` disables scaffold builds; the draw-side fallback then uses only ancestors that exist anyway. |
| `bounding_factor` | `1.4` | Root cube half-size = `radius * factor`. Range [1.05, 4]. |
| `max_concurrent_builds` | `16` | Chunk mesh builds in flight at once (the pool runs wide open; ground views select thousands of chunks). |
| `starved_inline_build_interval_updates` | `30` | Consecutive updates with queued work but no delivered mesh after which one chunk builds inline on the calling thread (the streaming fallback for machines too small to give the pool CPU time). `0` keeps the pool-only behavior. |
| `max_new_builds_per_update` | `48` | Build requests issued per `updatePlanet` call. Requests are ordered by apparent size (chunk width over distance, the same quantity the LOD rule is built on), so the widest-on-screen unresolved chunks build first. |
| `max_live_planets` | `16` | How many planets one module instance may keep alive at once (each owns a worker pool). `createPlanet` reports `CapacityReached` beyond the cap. |
| `chunk_eviction_updates` | `240` | Updates a hidden mesh stays cached before eviction. The stamp-refresh and eviction scan follows this value whenever it is shorter than the default eight-update cadence, so thresholds below eight never age a visible chunk out between scans. |
| `chunk_cache_capacity` | `16384` | Hard cap on cached chunk meshes (oldest evicted first). Must exceed the whole visible set on the deepest ground views (the screen-space LOD rule bounds the visible set, which stays radius-independent in angular terms) or ready meshes evict and rebuild in a churn loop. |

## Relief shaping (meters unless noted)

Large-scale layers (continents ... regional) carry wavelengths scaled to the Earth-scale radius (real continents span 3,000-8,000 km, orographic belts keep ~1,000 km spacing, regional plateaus/basins ride 300-1,000 km); close-range layers (hills and finer) stay metric.

| Key | Default | Effect |
| --- | --- | --- |
| `continent_amplitude_meters` | `700.0` | Height of the continent base field (real land averages ~840 m above sea level). |
| `continent_wavelength_meters` | `3610000.0` | Feature scale of the continent field (real continents span 3,000-8,000 km). |
| `continent_octaves` | `4` | fBm octaves of the continent field. Fewer octaves = fewer medium patches: landmass cores stay contiguous; coastline complexity comes from the warp plus near-shore hills. |
| `domain_warp_amplitude` | `0.18` | ANGULAR coastline bending (bends features by roughly amplitude * R meters): 0.18 shears coastlines ~1,150 km at Earth's radius - the scale real shoreline convolution reaches - without shredding landmasses into filaments (0.5 gives archipelagos). |
| `domain_warp_wavelength_meters` | `1487000.0` | Scale of the warp field. |
| `mountain_amplitude_meters` | `3000.0` | Ridge height where the mountain mask saturates (real range crests run 2-4.5 km; the 2.4 km snow line caps them). |
| `mountain_wavelength_meters` | `956000.0` | Belt-placement scale of the ridged field (~1,000 km, the spacing real orogenic systems keep). |
| `mountain_mask_low` / `mountain_mask_high` | `0.08` / `0.45` | Continent-field values where mountains start / reach full height. |
| `mountain_sharpness` | `1.25` | Exponent applied to the ridged field. |
| `mountain_octaves` | `5` | fBm octaves of the ridged field. |
| `mountain_mass_wavelength_multiple` | `1.8` | Wavelength of the broad mass field as a multiple of the crest wavelength. |
| `mountain_crest_mass_coupling` | `0.35` | Crest weight at zero mass: higher keeps ribbed crest lines alive across low-mass country, lower concentrates the ridges on the big massifs. |
| `mountain_mass_fraction` | `0.50` | Amplitude share of the broad smooth mass field (the rest goes to the ridged crest field; a pure ridge sum reads as uniform needle carpets). |
| `range_cluster_strength` | `0.55` | How strongly uplift groups into distinct ranges with foothill gaps. |
| `range_cluster_gate_low` / `range_cluster_gate_high` | `-0.35` / `0.45` | Cluster-field window where the uplift gate opens / saturates (continent-field units). |
| `range_cluster_wavelength_meters` | `2124000.0` | Scale of the range-cluster field (real orogenic belts run ~2,000 km). |
| `mountain_erosion_weight` | `1.0` | Blend of the analytic erosion filter's mountain profile against the classic mass-and-crest profile above (`0` = classic, `1` = fully eroded mountains; the belts keep the same continent mask and range clustering at every weight). |
| `mountain_erosion_face_blend_width` | `0.02` | Width of the cubemap face-blend window (fraction of the dominant axis): near face seams the two overlapping faces blend across this band so gully stripes continue around the sphere. |
| `mountain_erosion_unit_meters` | `9000.0` | Arc length ONE filter unit spans - the METRIC ridge/valley/gully scale of the eroded belts (real ridge spacing runs 1-10 km), independent of the belt-placement `mountain_wavelength_meters`. `0` derives the unit from the mountain wavelength (small-planet convenience). |
| `mountain_erosion_terrain_frequency` | `1.0` | Frequency of the filter's base terrain: massifs sit at `mountain_erosion_unit_meters / frequency` (one filter unit spans that many meters of arc at the cubemap face centers). |
| `mountain_erosion_terrain_amplitude` | `0.25` | Amplitude of the filter's base terrain in its `[0, 1]` height band - the massif relief share of the eroded profile. |
| `mountain_erosion_terrain_octaves` | `3` | Octaves of the filter's base terrain. |
| `mountain_erosion_terrain_gain` | `0.10` | Gain of the filter's base terrain octaves. |
| `mountain_erosion_terrain_lacunarity` | `2.0` | Lacunarity of the filter's base terrain octaves. |
| `mountain_erosion_height_offset_low` | `-0.35` | Constant height offset compensating the gully stack's ridge-up bias, scaled by the accumulated erosion magnitude (partially compensating keeps the eroded belt's mean height on the input terrain's average). |
| `mountain_erosion_height_offset_blend` | `0.0` | Blend of the offset toward the negated fade target (`0` = constant offset, `1` = fully altitude-keyed). |
| `mountain_erosion_scale` | `0.28` | Coarsest gully stripe wavelength in units of `mountain_erosion_unit_meters` times `mountain_erosion_cell_scale` (default stripes span ~1.8 km). |
| `mountain_erosion_strength` | `0.30` | Erosion amplitude of the gully stack. |
| `mountain_erosion_gully_weight` | `0.55` | The stripes' share of each octave against the fade target; lower values keep peaks pointy, higher values carve deeper gullies. |
| `mountain_erosion_detail` | `1.2` | How strictly finer gully octaves are restricted to steeper ground (lower = fine gullies across gentler slopes, higher = only on the steepest ground). |
| `mountain_erosion_cell_scale` | `0.70` | Cell size of the stripe interpolation grid relative to the stripe wavelength. |
| `mountain_erosion_normalization` | `0.50` | Partial stripe re-normalization keeping gully magnitudes consistent (`0` = full normalization, `1` = none). |
| `mountain_erosion_octaves` | `5` | Gully octaves over the stripe scale (each at half the wavelength of the previous). |
| `mountain_erosion_gain` | `0.80` | Gain of the gully octaves: above `0.5` the fine gullies carry more slope than the coarse ones, which is what makes the erosion texture visible at walking scale. |
| `mountain_erosion_lacunarity` | `2.0` | Lacunarity of the gully octaves. |
| `mountain_erosion_rounding_ridge` | `0.10` | Rounding applied to ridge crests (real peaks are never razor-sharp). |
| `mountain_erosion_rounding_crease` | `0.00` | Rounding applied to gully creases (gully floors collect sediment). |
| `mountain_erosion_rounding_global` | `0.10` | Global multiplier of both roundings. |
| `mountain_erosion_rounding_gain` | `2.0` | Per-octave compounding of the global rounding multiplier. |
| `mountain_erosion_onset_input` | `1.25` | Slope the input terrain needs before gullies appear on it. |
| `mountain_erosion_onset_octave` | `1.25` | Slope each gully octave needs before it contributes. |
| `mountain_erosion_onset_ridge_map` | `2.80` | Slope threshold of the ridge map's own fade chain. |
| `mountain_erosion_onset_ridge_mask` | `1.50` | Slope threshold of the ridge map's mask chain. |
| `mountain_erosion_assumed_slope` | `0.70` | Fixed slope magnitude the gully directions use, blended over the terrain's own slope so gullies stay aligned through rounded peaks and valley floors. |
| `mountain_erosion_assumed_slope_blend` | `1.0` | Blend toward the assumed slope (`1` = fully assumed; the filter's own derivatives are approximations). |
| `mountain_erosion_profile_low` / `mountain_erosion_profile_high` | `0.30` / `0.52` | The filter's height band remapped linearly onto the mountain profile `[0, 1]`, so valley floors sit at the belt base and crests reach the full `mountain_amplitude_meters`. Widen for gentler relief, narrow for wilder. |
| `regional_amplitude_meters` | `140.0` | Broad plateaus/basins/valley systems between the continent and hill bands - the landform-structure layer at globe scale. |
| `regional_wavelength_meters` | `690000.0` | Scale of the regional field (real plateaus/basins ride 300-1,000 km). |
| `regional_octaves` | `3` | fBm octaves of the regional field. |
| `hill_amplitude_meters` | `110.0` | Medium-frequency rolling terrain (metric; radius-independent). |
| `hill_wavelength_meters` | `1000.0` | Hill scale. |
| `hill_octaves` | `5` | fBm octaves of the hills. |
| `detail_amplitude_meters` | `14.0` | Fine detail layer: surface roughness between the hill wavelength and the voxel size (rock ribs, soil ripples, scree). Weighted like the overhang term: mountains and cliffs carry full strength, plains keep `detail_plains_fraction`. |
| `detail_wavelength_meters` | `120.0` | Feature size of the detail layer's base octave. Keep the finest octave (base / 2^(octaves-1)) at 2.5+ voxel sizes or coarse LOD chunks alias it into lumps. |
| `detail_plains_fraction` | `0.22` | Detail strength kept on flat ground (above ~0.3 boils plains into noise lumps). |
| `detail_octaves` | `4` | fBm octaves of the detail layer. |
| `terrace_strength` | `0.95` | 0 = smooth terrain, 1 = full plateaus (mesa country). Terracing runs through a sharper, lower gate than the cliff mask itself (full strength a quarter of the way into the steepness band), so genuinely steep flanks quantize into crisp benches and risers while open ground below the band stays smooth; the cliff mask shape still grades the strata, overhang and detail weighting. |
| `terrace_steps` | `9.0` | Approximate plateau count across the relief range (step height = total relief / steps; 9 steps over the showcase relief makes ~200 m benches). |
| `terrace_riser_fraction` | `0.15` | Fraction of each terrace step the RISER spans. The riser's grade is the flank slope divided by this fraction, so small values build genuinely cliff-like faces: 0.15 turns a 0.35-grade flank into a ~60-degree rock wall (with strata and overhang ledges riding the same mask); 0.4 smears the step into a long shoulder. Below ~0.1 the riser thins under one voxel. |
| `cliff_wavelength_meters` | `1400.0` | Scale of the cliff-mask variety modulation (spacing between mask-band variations). |
| `cliff_steepness_low` | `0.42` | Cliff-mask gate over the terrain's steepness energy: where the mask opens (terracing/strata/overhangs begin). Lower widens terraced country, higher restricts it to the steepest flanks. |
| `cliff_steepness_high` | `0.70` | Steepness energy where the mask saturates (full-strength terraces, strata and overhangs). Keep at least ~0.2 above `cliff_steepness_low`. |
| `strata_gate_low` / `strata_gate_high` | `0.34` / `0.85` | Cliff-mask window where the strata banding gate opens / saturates (keeps banding off open slopes while letting the flanks the camera actually sees carry their beds). |
| `cliff_sharpness` | `2.6` | Exponent concentrating the hill-flank term of the cliff mask. Higher narrows terraced hill country into tighter bands; the mountain-crest term is unaffected. |

## Geologic provinces

A very-low-frequency field classifies every landmass region into layered sedimentary country (warm bedded tones, stronger strata and terraces), shield/volcanic country (cool gray tones, weaker strata) and pale carbonate patches (limestone). The geometry biases and the rock albedo read the same field, so cliff faces look and structure themselves as one geology. The carbonate patches follow the climate the way real limestone country does - they only appear in warm lowlands (tropical shelves, exposed reef tracts and karst plains), gated by the sea-level temperature and an elevation band scaled from the deep-water and snow-line keys.

| Key | Default | Effect |
| --- | --- | --- |
| `provinces_enabled` | `true` | Master switch of the province field. |
| `province_wavelength_meters` | `0` | Feature scale of the province field; `0` derives 1.6x the continent wavelength (provinces span whole landmass regions). |
| `province_rock_variation` | `0.55` | Strength of the bare-rock palette tint (sedimentary tan/brown, volcanic gray, carbonate pale). |
| `province_strata_bias` | `0.35` | How much stronger the strata banding runs in sedimentary than in shield/volcanic country. |
| `province_terrace_bias` | `0.30` | How much stronger terracing runs in sedimentary than in shield/volcanic country. |

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
| `coastal_flatten_full_fraction` / `coastal_flatten_reach_fraction` | `0.45` / `1.6` | Fractions of the flatten band where the taper reaches full damping (below) and full detail strength (above). The steepened curve keeps shoals from speckling. |
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
| `abyssal_mask_low` / `abyssal_mask_high` | `-0.60` / `-0.02` | Continent-field window the abyssal mask rides (negative = basins): the sea floor sinks where the continent field sits inside this band. |
| `submarine_fade_low_fraction` / `submarine_fade_high_fraction` | `0.02` / `0.06` | Height fractions of the continent amplitude where the submarine fade closes (below: ocean-floor detail fully suppressed) and opens (above: land detail untouched). |
| `abyssal_depth_meters` | `3500.0` | Extra depth the sea floor sinks toward as the continent field dips into its basins: the coast keeps a shallow, sunlit continental shelf that slopes into dark abyssal plains, instead of one uniformly shallow flooded plain (the base relief amplitudes alone bottom out around the continent amplitude). The shelf is as wide and smooth as the continent geography itself, and every detail layer stays faded out over the deepened slope. Clamped to 8% of `radius_meters` so the floor stays a surface feature on small bodies. `0` disables. |
| `water_depth_tint_meters` | `70.0` | Depth over which the water surface tint saturates in the BLUE channel; red and green saturate at fixed fractions of it (0.28x and 0.6x - the physical absorption ordering, red first), so the ramp runs shallow turquoise -> green-teal mid shelf -> navy deep. |
| `water_enabled` | `true` | Generate water surface patches (marching cubes on the sea sphere). |
| `water_color_shallow` | `[0.070, 0.255, 0.275]` | Linear RGB at zero depth (the turquoise of sunlit shallow water over sand). |
| `water_color_deep` | `[0.022, 0.095, 0.210]` | Linear RGB at full tint depth - the saturated royal blue orbital photography reads on the lit open ocean. |
| `water_roughness` | `0.22` | Gloss hint handed to the renderer's water material at close range. Low values give the sea a crisp sun-glint path that the wave normals break into glitter. |
| `water_roughness_orbital` | `0.46` | High-altitude blend target for the same hint: the material widens the water lobe toward this value as the eye climbs out of the atmosphere, trading close-range glitter for the broad, smoothly attenuating sun patch orbital views of the ocean show. The value sits past the Cox-Munk sea-state slope spread so the coarse far-water facet grid never photographs through the lobe as a faceted plate pattern. |
| `sea_ice_temperature` | `0.10` | Sea-level temperature below which the water surface freezes. The field is the same climate field that drives the land's freeze line, so polar oceans ice over where the land turns tundra, with an organic, noise-shaped edge. `0` disables sea ice. |
| `sea_ice_transition` | `0.05` | Temperature range around `sea_ice_temperature` over which the ice color blends in. |
| `water_color_ice` | `[0.78, 0.85, 0.94]` | Linear RGB of the frozen water surface; slightly blue against the warm-white cloud tops so polar caps read as ice, not weather. |
| `water_wave_amplitude` | `0.28` | Radian-scale tilt applied to the water surface normals by a deterministic two-scale noise field (the configured swell plus a half-wavelength chop at a third of the tilt), giving the sea a sun-glint texture with sparkle at close range. The tilt fades out under the LOD band limit like every other noise term, so coarse chunks keep a smooth sea. `0` gives a perfectly smooth surface. |
| `water_wave_wavelength_meters` | `8.0` | Feature size of the wave-swell field (the chop rides at half of it). |
| `water_wave_chop_fraction` | `0.33` | Amplitude of the half-wavelength chop as a fraction of the swell tilt (the chop breaks the sun glint into sparkle). `0` gives a single-scale swell. |
| `water_foam_width_meters` | `14.0` | Depth band under the surface that reads as surf - the breaker rim hugging the waterline. The band's falloff exponent (see `water_foam_falloff_exponent`) keeps the bright rim at the shore even on ultra-gentle shelves (a plain smoothstep would wash whole shallow lagoons white). |
| `water_foam_falloff_exponent` | `3.0` | Falloff exponent of the breaker band across its depth range: 2 keeps a broad wash of surf over the shelf, 4 squeezes it into a tight rim at the waterline. |
| `water_foam_strength` | `0.60` | 0 = no surf, 1 = a solid white breaker band. The crest modulation keys to the offshore direction (read from the terrain gradient at the waterline), so the band breaks into elongated breaker ranks running parallel to the coast instead of isotropic patches; between ranks a base wash keeps the rim continuous. |
| `water_shore_sand_brightness` | `1.35` | Brightness lift of the sand seen through the shallow shelf (sunlit flats under a thin sheet read brighter than the raw beach row). |
| `water_shore_mix_meters` | `12.0` | Depth over which the water surface blends toward the beach-sand tone transmitted by the water's own hue, so very shallow flats read as sandy turquoise (the bottom seen through the sheet) instead of one pale sheet. `0` disables the blend. |

## Climate and biomes

| Key | Default | Effect |
| --- | --- | --- |
| `equator_temperature` | `1.0` | temperature01 at the equator, sea level. |
| `pole_temperature` | `0.02` | temperature01 at the poles, sea level. |
| `temperature_perturbation` | `0.09` | Amplitude of the wandering temperature noise. |
| `temperature_wavelength_meters` | `2124000.0` | Scale of the temperature noise (scales with the radius like the other climate fields). |
| `temperature_lapse_meters` | `1100.0` | Meters of altitude per 1.0 temperature01. |
| `moisture_wavelength_meters` | `1487000.0` | Scale of the humidity noise (scales with the radius like the other climate fields). |
| `moisture_perturbation` | `0.24` | Amplitude of the humidity noise. |
| `moisture_warp_amplitude` | `0.22` | Domain-warp of the humidity field: bends biome patches into elongated, organic shapes (unwarped, patches read as round camouflage blobs). |
| `coastal_moisture_boost` | `0.18` | Onshore humidity: low land near the sea stays green (without it the shoreline belt paints a beige desert fringe around every landmass). |
| `coastal_moisture_band_meters` | `400.0` | Elevation over which the coastal moisture boost fades out. |
| `climate_continentality_strength` | `0.30` | How strongly continental interiors dry out away from the ocean: humidity drops as the continent field climbs into its interior cores, so landmass cores trend arid while the coasts keep their green fringe. `0` disables. |
| `climate_rain_shadow_strength` | `0.35` | Humidity dropped downwind of mountain country: the mountain mass envelope is sampled upwind along the prevailing wind, so the lee side of every range carries a dry belt the way orographic precipitation leaves one. `0` disables. |
| `climate_wind_zones_enabled` | `true` | Prevailing winds for the rain shadow follow the latitude zones every rotating planet shows (easterly trades in the tropics, westerlies over the temperate band, easterly polar winds); `false` blows uniformly west-to-east. | Elevation over which the coastal moisture boost fades out. |
| `moisture_latitude_bands` | `true` | Circulation-cell humidity belts (see the four amplitude keys below). |
| `moisture_belt_equator_amplitude` | `0.20` | Humidity added by the ITCZ wet belt on the equator. |
| `moisture_belt_subtropical_amplitude` | `-0.26` | Humidity shift of the subtropical dry belts (~26 degrees latitude). |
| `moisture_belt_temperate_amplitude` | `0.10` | Humidity added by the wet belt over the temperate westerlies (~54 degrees). |
| `moisture_belt_polar_amplitude` | `-0.06` | Humidity shift of the polar highs (~88 degrees). Belt centers/widths follow the classic circulation cells; `0` removes a belt. |
| `color_variation` | `0.34` | Per-point albedo variation (0 = flat palette). The base and mid-range scales apply as a multiplicative hue tint (each channel weighted differently, so the mottle reads as warm/cool patches); the ground scale is ADDITIVE in absolute color units and rotates the hue - positive noise pushes a point toward its dry, warm direction (red and green lift, blue sinks) and negative noise toward the lush, cool one. Absolute offsets are what a close-range eye sees: dark albedos would swallow a proportional wiggle of the same nominal strength. Octave scales below the chunk's VERTEX spacing fade out (band-limited like the geometric noise), so coarse LOD chunks do not alias the mottle into speckle. |
| `color_variation_wavelength_meters` | `260.0` | Feature size of the albedo jitter field. The field carries three scales: the configured wavelength mottles biome sheets at globe scale, an octave at an eighth of it covers the mid-range, and an octave at `color_variation_ground_wavelength_meters` keeps ground-level views from reading as one unbroken color. |
| `color_variation_ground_wavelength_meters` | `30.0` | Near-field mottle octave of the albedo jitter field; the surface texture a walking camera sees. Keep it well above four times the finest voxel size: a vertex field sampled at four vertices per wavelength photographs as stretched linear gradients across the rendered triangles, so the octave stays broad (smooth per-vertex hue patches) and the 1-4 m band belongs to the renderer's per-pixel procedural mottle, which no triangle grid samples. Clamped to at most `color_variation_wavelength_meters`. |
| `soil_patch_strength` | `0.30` | Erosion patches: a sparse ground-scale field pulls vegetated ground toward the bare-soil tone (the rock palette row), so biome sheets break into soil scars, gravel fans and worn earth instead of one unbroken color. The classification window is deliberately narrow, so the patches stay sparse and distinct rather than washing the sheet in broad stains. Skipped where rock, snow or beach already own the surface; band-limited at the chunk's vertex spacing like every other noise term. `0` disables. |
| `soil_patch_wavelength_meters` | `60.0` | Feature size of the erosion patches. Keep it above roughly four times the finest voxel so the field survives the LOD band limit. |
| `dry_patch_strength` | `0.45` | Dryness mosaic: a broad ground-scale field senesces vegetated ground - the current biome tone lerps toward its dry, yellow-tan counterpart (the color scaled toward the red end), so grasslands, meadows and forest floors break into the lush/dry mosaics real vegetation shows at walking and mid distance. The dry tone derives from the blended biome color itself, so every biome keeps its own dry direction (grass goes khaki, forest olive, tundra straw) without a second palette. Skipped where rock, snow or beach own the surface; band-limited at the chunk's vertex spacing. `0` disables. |
| `dry_patch_wavelength_meters` | `30.0` | Feature size of the dryness mosaic. Keep it above roughly four times the finest voxel so the field survives the LOD band limit. |
| `snow_line_meters` | `2400.0` | Sea-level snow line; drops as the local climate cools. Only true peaks are glaciated; polar caps come from latitude alone. |
| `beach_width_meters` | `5.0` | Shoreline band above sea level - wide enough to read as sand from a standing eye. |
| `beach_shoreline_wobble` | `0.45` | Fraction the shoreline band's width undulates by along the shore (a low-frequency noise stream, ~55 m features), so the sand/grass boundary meanders like a real dune line instead of tracing the ruler-straight elevation contour the coastal flattening leaves. Stretches the wobble pinches toward zero width have no beach; 0 keeps the constant width everywhere. |
| `beach_wet_width_meters` | `6.0` | Swash band above the waterline that reads as wet sand (real beaches darken sharply where the last wave reached). 0 disables. |
| `beach_wet_darkening` | `0.45` | How dark the wet band gets (0 = dry sand everywhere, 1 = near-black rim). |
| `rock_slope_threshold` | `0.46` | Slope (1 - dot(normal, radial)) where bare rock takes over. Steep faces scrape soil and grass into bare rock; rolling hills stay green. |
| `rock_strata_contrast` | `0.75` | Luminance banding of rock faces at the strata band spacing (sediment stripes). The band thickness follows a low-frequency warp (beds vary, and the gate's wide spread keeps most beds faint, so banded stretches read as geology rather than a periodic pattern) and the stripe fades out below two samples per bed at the chunk's vertex spacing, so coarse chunks never alias it into per-vertex speckle. 0 disables; keyed to `strata_band_meters`. |
| `biome_blend_width` | `0.06` | Half-width of the biome color blend on the temperature/moisture axes. Biome colors interpolate between palette rows instead of hard-switching per vertex (hard switches smear into pale stains across triangles). |
| `slope_blend_width` | `0.08` | Blend half-width around the rock slope threshold. |
| `elevation_blend_fraction` | `0.12` | Snow-line and water-band color blends as a fraction of the band width. |
| `biome_palette` | 16 rows | Linear RGB rows in `Biome` enum order (deep ocean .. rock). The rows carry the albedos the surface photographs through a scene's exposure: vegetation rows sit near real canopy/grass reflectances (dark and saturated), sand and tundra stay bright, snow keeps its ~0.9, and the ocean-floor rows sit deliberately darker than the water surface colors. Entries may be omitted from the end. |

## Surface texturing

The mesher partitions terrain into two surface groups - ground (soil, grass, snow fields) and rock (steep faces past `rock_slope_threshold`, gated by the same cliff/steepness masks that shape the strata and terraces) - and tiles texture coordinates over each with a continuous projective frame, divided by the group's tile size in meters, so the tiling is world-space continuous across chunk boundaries (no per-chunk UV islands) and a consumer's detail textures meet the generator here. Each group keys that frame to what its surfaces face: the ground group (and the water surface, in its own group) projects against the RADIAL direction - rolling-ground normals wiggle with the noise field, and a normal-keyed frame would kink the tile lattice triangle-to-triangle (stripes shear and flip direction at every triangle edge, photographing as a herringbone) - while the rock group keeps the cubemap-style blend of the three axis-plane projections weighted by |normal_axis|^4 (steep faces genuinely face an axis plane, and the normal map's per-triangle tangent frame needs the UVs to follow the face). Chunks with both groups emit one mesh per group (plus water), so each draws with its own material; a group with no geometry emits nothing.

| Key | Default | Effect |
| --- | --- | --- |
| `terrain_texture_tile_meters_ground` | `6.0` | World meters one ground-group texture tile spans (soil/grass detail scale). |
| `terrain_texture_tile_meters_rock` | `14.0` | World meters one rock-group texture tile spans; rock faces read coarser beds than soil reads grain. |
| `water_texture_tile_meters` | `8.0` | World meters one water-surface texture tile spans (wave/glint detail scale). |
| `texture_uv_warp_strength` | `0.35` | Amplitude (in texture tiles) of the low-frequency simplex field that bends each group's tile lattice, so the world never photographs every tile at the same phase and the visible repeat of a tiled scan breaks up into organic variation. A pure world-space function of position, band-limited like every other noise term (it fades out below the chunk's own LOD cutoff), and clamped well under one tile so the frame never folds onto itself. `0` restores the unbent world-space tiling. |
| `texture_uv_warp_wavelength_meters` | `64.0` | Feature size of that warp field (a few tiles at the shipped tile sizes). Clamped to at least 4 m and to a quarter of the planet radius. |
| `texture_chunk_rotation` | `true` | Turns each chunk's terrain tiling by a deterministic multiple of 90 degrees, hashed from the planet seed and the chunk address per surface group, so neighboring chunks never photograph the same scan in the same orientation. Quarter turns map the tile lattice onto itself, so the frames stay seamless inside every chunk and tileable textures keep tiling; `false` keeps every chunk's frame unrotated. |

## Surface walking

A kinematic, point-feet character controller lives in the module (`SurfaceWalker`, see `walker.hpp`): gravity pulls toward the planet center, movement lives in the tangent plane, and terrain collision queries the same pure density field the mesher samples (a march-and-bisect along the local radial direction), so overhang ledges and cave floors are walkable without any collision proxy geometry. A full physics module is not needed for this: the field answers the ground queries directly, and the controller stays deterministic (fixed inputs walk one exact path, which the demo's self-test asserts).

| Key | Default | Effect |
| --- | --- | --- |
| `surface_gravity_mps2` | `9.81` | Constant pull toward the planet center, meters/second². |
| `walk_speed_mps` | `4.5` | Ground movement speed. |
| `walk_sprint_multiplier` | `2.6` | Sprint speed = walk speed × this. |
| `jump_speed_mps` | `5.0` | Upward impulse when jumping; 0 disables jumping. |
| `eye_height_meters` | `1.7` | Camera height above the feet (the walker reports an eye position). |
| `walk_ground_snap_meters` | `0.9` | Deepest drop the feet follow without becoming a fall; walking off a bigger drop goes airborne. |
| `walk_max_step_up_meters` | `0.55` | Highest ledge the character walks straight onto; taller walls block and the movement slides along them. |
| `walk_max_climb_grade` | `3.5` | Rise/run above which a slope is unclimbable (a tangent value; 3.5 ≈ 74 degrees). |
| `walk_terminal_velocity_mps` | `55.0` | Speed cap of the falling integration (bounds the landing probe a long fall needs). | Rise/run above which a slope is unclimbable (a tangent value; 3.5 ≈ 74 degrees). |
| `swim_enabled` | `true` | Float at the sea surface over water deeper than the wade band. |
| `swim_speed_mps` | `2.6` | Movement speed while swimming. |
| `swim_wade_depth_meters` | `1.2` | Water shallower than this stays walkable (the terrain floor carries the feet). |

## Sky backdrop

The module exposes a pure sky model (`atmosphere.hpp`): a zenith-to-horizon gradient keyed to the view direction's elevation above the local horizon, an altitude weight that fades the whole sky to space over `atmosphere_height_meters`, a sun-elevation tint (neutral through the day, warm gold through the low-sun band, deep blue at night), a direct-light transmittance tint (the Rayleigh-style white -> warm -> deep red curve the sun's own light follows toward the horizon, amplitude `sun_direct_tint_strength`), and an aerial-perspective sample for one sight line (the sun-tinted haze the air adds between the eye and a distant point - its weight grows exponentially with the sight-line length on the `aerial_perspective_scale_meters` scale, approaches `aerial_perspective_asymptote` at infinite distance, and fades out as the eye climbs into space, and its color is the horizon band under the same sun tint, so distant terrain melts into the sky behind it). The two tints deliberately disagree toward the horizon - a red sun under a gold sky is the real sunset read - so consumers light terrain and draw the disc with `sunDirectTint` while painting the sky and haze with the scattered tint. All functions are pure and thread-safe, and the colors are linear RGB like the water colors, so a consumer can paint a sky dome, a frame clear color, or apply distance haze to their scene - the demo draws a gradient sphere around the walking eye, hands the horizon color to the renderer as the clear color, fades its chunk draws toward the haze with distance (space views keep the atmosphere limb shell as their depth cue), and pushes the transmittance tint into its `[pbr]` sun color each frame so terrain, water and specular glints share one spectral story with the disc (the eye's atmosphere weight fades the curve back to white for space views).

| Key | Default | Effect |
| --- | --- | --- |
| `atmosphere_enabled` | `true` | Master switch; `false` leaves the sky entirely to the consumer. |
| `atmosphere_height_meters` | `100000.0` | Eye altitude over which the sky fades to black (space); Earth's visible air shell (the Kármán line sits at 100 km, 1.6% of the radius - the thin blue limb orbital photographs show). The sky is at full strength below 15% of this height. |
| `atmosphere_zenith_color` | `[0.060, 0.170, 0.580]` | Linear RGB at the zenith. The defaults are chosen for the renderer's shared tone mapping (which brightens midtones), so the pre-tone-map values sit deeper and more saturated than the sky they finally display. |
| `atmosphere_horizon_color` | `[0.470, 0.620, 0.860]` | Linear RGB at the horizon; below it the haze darkens gently so ground-level gaps read as atmosphere, not void. |
| `aerial_perspective_scale_meters` | `8000.0` | Scale length of the aerial-perspective haze's exponential, in meters (the distance at which the haze has covered ~63% of its asymptote). Real clear-air haze runs at this scale (Koschmieder visibilities of 20-80 km imply ~5-20 km extinction scale lengths): larger values keep the midground clear and let the far ridge dissolve gradually; smaller values haze the whole distance range over at once. |
| `aerial_perspective_asymptote` | `0.65` | Haze weight the profile approaches at infinite distance. Keeps the farthest terrain faintly readable as terrain instead of fading fully into sky; `0` disables the aerial perspective entirely. |
| `sun_direct_tint_strength` | `1.0` | How strongly the DIRECT sun light (terrain/water/specular lighting, the sun disc) adopts the atmosphere's transmittance tint - the white -> warm -> deep red shift the sun's own light gains as its path through the air lengthens toward the horizon (Rayleigh-style curve, see `sunDirectTint` in `atmosphere.hpp`). `1` gives the full physical curve, `0` keeps the consumer's configured sun color unchanged at every elevation. The scattered light (sky gradient, haze) keeps its own sunset tint either way, so `0` restores the look of a scene lit by one fixed sun color. |
| `sky_sun_halo_strength` | `0.32` | Whitening strength of the forward-scatter structure in the sun-aware sky gradient (see the `skyGradientColor` twin in `atmosphere.hpp`): the Mie glare wedge around the sun plus the horizon band's brightening on the sun's side of the dome, while the anti-sun sky keeps the deep saturated blue. `0` restores the sun-independent zenith-to-horizon ramp exactly. |
| `sky_warm_horizon_strength` | `0.50` | Strength of the gold the sun side of the horizon carries through the late-afternoon band (the hue swing toward amber on the sun's azimuth, grading into the cool anti-sun haze around the dome). Rides the sun elevation like the halo term; `0` keeps the plain horizon color at every azimuth. |

## Clouds

The sky model also carries a `CloudField` (`atmosphere.hpp`): a band-limited coverage field sampled on a spherical shell at `cloud_altitude_meters` above the datum, thresholded so `cloud_coverage` is the approximate sky fraction under cloud, with a fine breakup octave (amplitude `cloud_detail_strength`) for ragged cumulus edges and an inverted-cellular billow octave (amplitude `cloud_billow_strength`) that packs the rounded cumulus lobes into the bank bodies, a seed-derived wind frame that stretches the banks along a deterministic flow direction (real weather systems organize into elongated streaks, not round blobs) with a low-frequency swirl bending their shapes, a very-low-frequency weather field that shifts the threshold regionally so banks organize into large cloudy and clear systems, and a latitude-band term keyed to the spin axis that adds the circulation-cell pattern every rotating atmosphere shows (a wet equatorial belt, dry subtropics, stormy mid-latitudes). Sampling is a pure function with a per-octave seed schedule derived from the planet seed, and the field is band-limited by the optional low-pass cutoff like every other noise term, so consumers with a coarse sample spacing fade the sub-spacing octaves out instead of aliasing them into speckle. The demo meshes the shell for its globe and orbit views (quads whose corners read clear are skipped, so the layer is geometry with organic cloud-shaped edges); the layer renders only for cameras outside the shell, since a camera between the surface and the shell would see the far side of the sphere through the sky above the horizon.

The same coverage field also drives the cloud SHADOWS: chunk meshes darken their terrain and water albedo under the coverage sampled along the sun ray through each surface point (see `cloud_shadow_strength`), so a bank's shadow falls away from the bank the way real cloud shadows do, and the shadow agrees with the cloud layer a consumer paints from the same field. The shadow reads the base field without the billow lobe octave: the shadow's penumbra widening (see `cloud_shadow_softness`) cannot resolve lobe-scale detail, and keeping the octave out of the per-chunk evaluation holds the streaming build cost at its pre-billow level.

Terrain vertices also carry SELF-SHADING (`terrainSelfShadow` in `density.hpp`): the mesher darkens each surface vertex by the relief the sun ray out of it actually meets - a fixed-resolution march of the density field up-sun over `terrain_self_shadow_reach_meters`, band-limited at a fixed fraction of the reach so every chunk depth evaluating a shared boundary vertex agrees. Escarpments, ridgelines and ravine walls then cast shade into their lee the way real relief photographs, beside the cloud shade and under the renderer's own slope lighting (which stays the authority on face direction). The march reads the same configured sun as the cloud shadows (`cloud_shadow_sun_direction`), so the two shades never disagree about where the light stands; night-side points (that sun at or below the local horizon) stay unshaded, and the whole evaluation runs at mesh-build time on the worker threads, costing the rendered frame nothing.

| Key | Default | Effect |
| --- | --- | --- |
| `clouds_enabled` | `true` | Master switch of the coverage field; `false` reports clear sky everywhere. |
| `cloud_coverage` | `0.40` | Approximate fraction of the sky under cloud. `0` disables. The latitude bands and weather organization stack their coverage on top of the base threshold, so the visible fraction reads higher than this value; keeping the base under half leaves the clear lanes that carry the land/ocean contrast. |
| `cloud_wavelength_meters` | `520000.0` | Feature scale of the bank field - raise it with `radius_meters` like the other large-scale layers. The individual cloud masses an orbital photograph resolves run ~200-800 km across; the weather field organizes the banks into the larger (1,000-2,500 km) systems. |
| `cloud_altitude_meters` | `2000.0` | Height of the cloud shell above the datum (real cumulus bases: ~300-2,400 m over land). |
| `cloud_detail_strength` | `0.35` | Amplitude of the fine breakup octave relative to the base field; higher values shred the bank edges into ragged cumulus. |
| `cloud_billow_strength` | `0.32` | Amplitude of the inverted-cellular billow octave relative to the base field: one-at-lobe-centers cellular distance packs the rounded cumulus lobes into the bank bodies (the Perlin-Worley pairing volumetric-cloud pipelines use - smooth fBm alone reads as soft cotton). `0` keeps purely smooth banks. The RENDERED banks sample the octave; cloud shadows read the base field without it - a penumbra-widened shadow cannot resolve lobe-scale detail, and the shadow field runs per streaming chunk, where the octave's 27-cell walk would tax every build. |
| `cloud_edge_softness` | `0.22` | Half-width of the coverage threshold band: narrow values scatter crisp clumps, wide values grow soft decks with veiled edges whose thinning fringes a consumer can paint (the demo's cloud layers fade their contour geometry through the fringe instead of cutting off at a polygon boundary). |
| `cloud_weather_strength` | `0.45` | Amplitude of the very-low-frequency regional weather shift (positive weather = cloudier); stronger values organize the banks into larger cloudy and clear systems. |
| `cloud_weather_wavelength_multiple` | `4.0` | Wavelength of the regional weather field as a multiple of the cloud field's own wavelength: larger values organize the banks into fewer, broader weather systems. |
| `cloud_latitude_band_strength` | `0.16` | Amplitude of the latitudinal coverage bands (a wet equatorial belt, dry subtropics, stormy mid-latitudes), keyed to the spin axis. `0` gives a latitude-blind field. |
| `cloud_wisp_strength` | `0.18` | Amplitude share of the wisp octave pair the texture-consumer path resolves (see `CloudField::coverageTexel` in `engine/planet/atmosphere.hpp`): the torn fringes and ragged holes whose feature size sits under every mesh grid's vertex spacing, so only a baked cloud texture shows them. Mesh shells band-limit the pair out at their own grid spacing, so the two reads stay consistent at the scale each resolves. `0` leaves the texture at the plain field's structure.
| `cloud_convective_strength` | `0.50` | Amplitude of the cellular convective octave pair the texture-consumer path resolves: the rounded clumping a ground-level eye parses as cumulus structure (the sky above ~25 degrees of elevation samples only a few kilometers of the cloud shell, so the clumps ride exactly the scale that patch photographs at; from orbit the pair reads as the cotton-wool grain of real decks). Mesh shells and cloud shadows band-limit the pair out. `0` leaves the baked texture at the bank field's own shapes.
| `cloud_cyclone_strength` | `0.45` | Strength multiplier of the seed-derived cyclone vortices: each storm shears the coverage field's nearby banks into trailing spiral arms and grows its own cloud shield - a two-armed spiral of near-solid bands around a subsidence-cleared eye, brightening into a central dense overcast and isolated from the province by a thinned moat - composited over the thresholded field so the system photographs at full contrast whatever region it sits in. Storms alternate hemispheres and stride same-hemisphere longitudes apart, keeping each system's shield readable on its own. Evaluated on the texture path; mesh consumers band-limit the whole storm system out. `0` leaves the deck unstirred, `2` grows every storm's shield and wrap. |
| `cloud_convective_wavelength_meters` | `8000.0` | METRIC wavelength of the coarse convective octave (independent of the radius-scaled `cloud_wavelength_meters`): the open-cell clumping scale - real cumulus fields pack their clumps into ~8 km mesoscale cells. `0` derives it from the bank wavelength (small-planet convenience). |
| `cloud_convective_fine_wavelength_meters` | `2000.0` | METRIC wavelength of the fine convective octave: the individual cumulus clump scale (a typical cumulus spans ~2 km). `0` derives it from the bank wavelength. |
| `cloud_wisp_wavelength_a_meters` | `540.0` | METRIC wavelength of the coarse torn-fringe octave (independent of the radius-scaled bank wavelength). `0` derives it from the bank wavelength (small-planet convenience). |
| `cloud_wisp_wavelength_b_meters` | `325.0` | METRIC wavelength of the fine torn-fringe octave. `0` derives it from the bank wavelength. |
| `cloud_band_frequency` | `3.4` | Frequency of those bands over latitude (radians of phase per radian of latitude): ~3.4 places a wet equatorial belt, dry subtropics and wet mid-latitudes the way Earth's circulation cells do. |
| `cloud_color` | `[0.985, 0.975, 0.955]` | Linear RGB of the cloud tops; warm white, so the layer reads against the bluer sea ice at globe scale. |
| `cloud_shadow_strength` | `0.12` | How strongly the terrain and water albedo darkens under the coverage field (sampled along the sun ray through each point). `0` disables cloud shadows. |
| `cloud_shadow_softness` | `0.72` | Penumbra softening of the cast shadows: the shadow field widens the coverage edge band by this fraction, so shadows read as soft-edged darkening under their banks instead of a hard copy of the cloud outlines. `0` keeps the shadow exactly as sharp as the banks. |
| `cloud_shadow_sun_direction` | `[0.50, 0.45, 0.74]` | Unit vector of the sunlight that casts the cloud shadows and the terrain self-shading - pair it with the direction your scene's lighting uses (the demo ships it next to its `[pbr] sun_direction` and warns when the two disagree). Points on the night side of this sun get no shadow. |
| `terrain_self_shadow_strength` | `0.55` | How strongly the terrain albedo darkens under relief the sun ray out of each vertex meets (the fixed-resolution march described above). `0` disables terrain self-shading. |
| `terrain_self_shadow_reach_meters` | `160.0` | Sun-ray travel the march resolves: an occluder farther up-sun than this stops shading the vertex, and the march's own density band rides at a sixth of it (finer relief is the renderer's slope lighting's own scale). Kept LOD-independent so chunks at different depths agree on shared vertices. Clamped to `radius_meters/65536 .. radius_meters/4`. |

## High cirrus deck

Above the cumulus shell the module models a thin ice-crystal deck (`cirrusStreakWeight` in `atmosphere.hpp`): a direction-space field of filamentary streaks stretched along the latitude circles (the zonal wind's prevailing direction), torn into bundles by a fine filament octave and thresholded by the `cirrus_coverage_amount` quantile - a translucent veil rather than banks. Sampling is a pure function of direction with the planet seed's per-stream schedule, band-limited by the consumer's own sample spacing the way every field is. The demo meshes the deck as a translucent shell for its globe and orbit views (the streak intensity rides an alpha texture, with a day-side gate that lets the veil fall to nothing through the terminator), so orbital views read the filaments over the sunlit planet; consumers that ignore the deck simply keep a clear high sky.

| Key | Default | Effect |
| --- | --- | --- |
| `cirrus_enabled` | `true` | Master switch of the streak field; `false` reports a clear high sky everywhere. |
| `cirrus_strength` | `1.0` | Brightness of the deck a consumer paints from the field; `0` disables (the field reports clear). |
| `cirrus_altitude_meters` | `3400.0` | Height of the veil's shell above the datum; `validate()` keeps it above the cumulus shell. |
| `cirrus_coverage_amount` | `0.30` | Approximate sky fraction under streak (the threshold's quantile); `0` reads as a clear high sky. Clamped to `0..0.6`. |

## Stars

The module exposes a seed-derived star field for the space backdrop (`atmosphere.hpp`): a direction-space lattice whose cell size follows `stars_density`, each cell hashing (seed + cell) into at most one star with a jittered position, a power-law brightness (most stars faint, a few bright) and a subtle color temperature (warm through white through blue). When `galaxy_band_strength` is positive the distribution additionally concentrates along a seed-derived great circle (the galactic plane): cells near the plane light up several times more often and run brighter, streaked by a seed-derived dark rift of dust lanes, so the backdrop reads as a Milky Way band over the uniform scatter. Two entry points serve two consumer styles: `starFieldColor` samples the field as a color (with the same band-limiting every other field applies, so a painted backdrop keeps the bright stars instead of aliasing the faint ones into speckle), and `forEachStar` enumerates the stars themselves for consumers that build one small disc per star. The demo meshes the per-star discs for its globe and orbit views and hides the layer inside the atmosphere.

| Key | Default | Effect |
| --- | --- | --- |
| `stars_enabled` | `true` | Master switch; `false` reports empty space everywhere. |
| `stars_density` | `0.5` | Star spacing: `0` sparse skies, `1` dense skies (the lattice tightens quadratically). |
| `stars_brightness` | `1.0` | Luminance multiplier of the whole field; the brightest stars saturate to white through the tone mapping. |
| `star_disc_radius_min_radians` | `0.0011` | Sky angle the faintest star's backdrop disc subtends; the radius a consumer's mesh draws grows with the square root of the star's brightness. Point-like defaults keep the field crisp at capture resolutions. |
| `star_disc_radius_max_radians` | `0.0024` | Sky angle the brightest star's disc grows to. Larger maxima read as soft blobs once a supersampling resolve averages them; `validate()` orders a reversed range and clamps both inside `0.0002..0.012`. |
| `galaxy_band_strength` | `1.0` | How strongly the star field concentrates along the seed-derived galactic plane: band cells hold stars several times more often, run brighter and lean warm, and a dark dust rift snakes through the plane's midline. `0` keeps the uniform field exactly. |
| `galaxy_band_width_radians` | `0.35` | Angular half-width of the band on the sky; the presence and brightness weights fall off exponentially in sine-galactic-latitude over it. Clamped to `0.05..1.2`. |

## Moon companion

The module places a companion body by a pure orbital model (`moonDirection` in `atmosphere.hpp`): a circular orbit whose plane tilts off the equator by `moon_orbit_inclination_radians` around a seed-derived node longitude, swept at `2*pi / moon_orbital_period_seconds`, with the phase at time zero anchored near quadrature plus a seed-derived offset (the `cloud_shadow_sun_direction` convention supplies the sun reference; the fixed lead keeps the visible face roughly half lit, so the companion reads as the daytime moon every photograph shows instead of a near-new disc whose unlit face would shade black against the daylight sky). The function returns a unit direction from the planet's center; consumers place their own geometry at whatever distance their far plane allows. The demo shades the companion as a LIT sphere (the same sun that lights the terrain lights it, so the phase, terminator and crescent are the shading's own output at every eye height), scales it to `moon_angular_radius_radians`, orients one face planetward (tidally locked), and tints it with the atmosphere's direct transmittance keyed on the companion's own elevation - a horizon moon reddens the way the low sun does.

| Key | Default | Effect |
| --- | --- | --- |
| `moon_enabled` | `true` | Master switch; `false` makes `moonDirection` return a zero vector (no companion anywhere). |
| `moon_angular_radius_radians` | `0.0080` | Sky angle the companion's disc subtends (about 0.46 degrees - a clearly readable disc at capture resolutions). Clamped to `0.0005..0.02`. |
| `moon_orbital_period_seconds` | `1200.0` | Seconds per revolution; the phase advances `2*pi` per period. Clamped to `60..1e6`. |
| `moon_orbit_inclination_radians` | `0.35` | Tilt of the orbit plane off the equator (the node longitude is seed-derived). Clamped to `0..1.55`. |
| `moon_albedo` | `1.0` | Brightness multiplier of the regolith (the demo's baked maria/highland albedos scale with it). Clamped to `0..8`. |

## Aurora

The module owns the polar oval's shape constants (`auroraOvalLatitudeLowRadians` .. `High`, the curtain's base and top altitudes - see `atmosphere.hpp`); the glow itself is the consumer's bake. The demo meshes a translucent emissive ring above each spin pole: the vertex colors carry the curtain's chroma only (an oxygen-green foot fading into a violet crown with altitude), while the whole intensity field - night gate, vertical ray striations, altitude fade - rides the alpha channel of a small texture the transparent route multiplies into the blend factor. That split keeps the bright day side clean without alpha-darkening it: a day-side column's texels carry alpha near zero and composite to nothing, instead of the drawn-but-black columns the earlier color-only bake smeared across the bright limb. The night gate is baked against the run's sun direction; columns whose foot falls below the gate floor are skipped outright. The ring draws in the renderer's transparent pass, depth-tested so the planet hides each ring's far side.

| Key | Default | Effect |
| --- | --- | --- |
| `aurora_enabled` | `true` | Master switch for the demo's curtain mesh. |
| `aurora_strength` | `0.35` | Scales the curtain's emissive brightness; `0` disables the glow. Clamped to `0..1`. |

## Surface scatter

The ground-level real-world elements the terrain's color field cannot express: vegetation, scattered surface rocks, bird flocks, breaking-wave foam, valley mist and the contact shadows that tie the scattered geometry to the ground. The module owns the density rules (which biome carries which element and how much - forests carry trees, grasslands carry grass, shrubland carries shrubs, open ground carries scattered rocks, ice and dunes stay bare); consumers scatter their own geometry against them. The demo does this deterministically in `planet_scatter.cpp`: candidate positions key to a planet-centered lattice of world-space cells, so a cell always evaluates to the same plant, and only the annulus gate around the eye changes between rebuilds. The foliage renders as alpha-cutout billboards (the renderer's `alphaCutoff` material masking, glTF alphaMode MASK semantics), the sea foam and mist as unlit translucent sprites, and the bird flocks as a per-frame vertex-updated mesh. Space views skip the whole system.

| Key | Default | Effect |
| --- | --- | --- |
| `vegetation_enabled` | `true` | Master switch for the demo's vegetation scatter. |
| `vegetation_tree_density` | `1.0` | Multiplier of the climate-derived tree presence (forest/taiga/savanna biomes). `0` removes trees. Clamped to `0..4`. |
| `vegetation_bush_density` | `1.0` | Multiplier for shrubs and bushes (shrubland, grassland, forest understory). Clamped to `0..4`. |
| `vegetation_grass_density` | `1.0` | Multiplier for grass tufts on grassy ground (the near-camera layer). Clamped to `0..4`. |
| `vegetation_tree_radius_meters` | `1400.0` | Scatter radius around the eye for trees and bushes. Clamped to `[100, 20000]`. |
| `vegetation_grass_radius_meters` | `46.0` | Radius of the near-camera grass ring (grass is the most expensive element per pixel, so the ring stays bounded and the sward thins across the ring's outer half so the boundary does not pop). Clamped to `[5, 200]`. |
| `surface_rock_density` | `1.0` | Multiplier for scattered boulders on open ground (rock, snow, desert and tundra carry more than farmland). Clamped to `0..4`. |
| `surface_rock_radius_meters` | `260.0` | Scatter radius for the boulders. Clamped to `[20, 4000]`. |
| `surface_shadow_strength` | `1.0` | Sun-aligned contact shadows under every scattered tree, bush and rock: soft dark quads laid on the ground, offset and stretched along the away-from-sun direction by the site's sun elevation (a low sun drags the shadow long and pale, a high sun bunches it under the element). `0` disables them. Clamped to `0..2`. |
| `whitecap_enabled` | `true` | Master switch for the breaking-crest foam on open water. |
| `whitecap_strength` | `0.65` | Scales the crest coverage; `0` is a glassy calm. Clamped to `0..1`. |
| `whitecap_wind_speed_mps` | `9.0` | The 10 m wind speed the sea state follows: whitecaps start forming near 3 m/s and the coverage climbs roughly with the cube of the wind (the parameterization follows the observational whitecap laws). Clamped to `[0, 40]`. |
| `surf_foam_enabled` | `true` | Master switch for the shore-parallel foam bands (a breaker band seaward of the waterline plus a swash band at it). |
| `surf_foam_strength` | `0.70` | Scales the bands' opacity. Clamped to `0..1`. |
| `birds_enabled` | `false` | Master switch for the bird flocks. Off by default: at capture resolutions a distant flock renders as a few-pixel sprite stack that the eye parses as a floating artifact rather than as birds. |
| `bird_flock_count` | `2.0` | Flocks within the scatter radius (they ride closed elliptical paths over ground or water near the eye and roost when the sun is below the horizon at their site). Clamped to `0..16`. |
| `bird_flock_size` | `9.0` | Birds per flock (they fly in a loose echelon with per-bird flap phases). Clamped to `0..64`. |
| `valley_mist_enabled` | `true` | Master switch for the ground-fog banks. |
| `valley_mist_strength` | `0.30` | Scales the mist opacity; banks pool in terrain depressions and over open water near the shore, denser toward sunrise (real radiation fog peaks at dawn). `0` clears the valleys. Clamped to `0..1`. |
| `scatter_rebuild_distance_meters` | `18.0` | Camera movement that triggers a re-scatter (between rebuilds the world stands still; the sea-state churn and the birds keep animating). Clamped to `[4, 500]`. |

## Debug

| Key | Default | Effect |
| --- | --- | --- |
| `debug_chunk_borders` | `false` | Highlights chunk borders: terrain and water vertices within `debug_chunk_border_width_meters` of their own chunk's boundary planes carry a fixed accent color blended over their shaded color, so every rendered chunk reads as a lit grid cell on the surface. The overlay exposes selection density, LOD transitions and seam behavior on demand; it rides the vertex colors only, so geometry, normals and the determinism guarantees are untouched. |
| `debug_chunk_border_width_meters` | `2.0` | Half-width of the tinted band along each boundary plane (the band fades from the full accent at the plane to nothing at this distance). Range [0.05, 1% of the radius]. |

## Usage notes

- **Scaling a planet.** Raise `radius_meters` and scale the wavelengths you want to keep proportional (continents, mountains, regional, climate fields, snow line); close-range amplitudes stay metric (a 650 m mountain is a 650 m mountain on any planet), and the erosion filter's ridge/valley unit (`mountain_erosion_unit_meters`) plus the cloud texture octaves (`cloud_convective_*_wavelength_meters`, `cloud_wisp_wavelength_*_meters`) are metric by default, so gully, cumulus and fringe scale stay real at any radius. The domain warp amplitude is ANGULAR - its metric effect grows with R (0.18 bends ~1,150 km coastlines at Earth's radius). The derived octree depth grows automatically until voxels reach `target_voxel_meters` (or the depth cap); `validate()`'s lower voxel clamp is `radius / 4194304`.
- **Performance knobs.** Visible chunk count scales roughly with `1 / target_voxel_meters^2` near the camera and with `lod_angular_voxel_radians` overall. `leaf_grid_n` trades chunk granularity against per-chunk cost. `max_concurrent_builds` / `max_new_builds_per_update` pace streaming (builds run on the module's dedicated worker pool, one thread per permitted concurrent build); caves cost two extra noise fields per sample. Terrain layers whose mask is zero for a sample (mountains over the ocean, detail fields under water) are skipped, and noise terms whose band-limit fade is zero at the chunk's cutoff are never evaluated at all, so coastal and ocean geometry is cheaper than mountainous terrain at the same depth. Cloud shadows cost one coverage evaluation per surface vertex (about two percent of a chunk build).
- **Chunk boundaries and seams.** Same-depth chunks weld bit-identically (one global lattice); across depth changes the octree keeps a 2:1 balance and the fine side CONFORMS its boundary-plane samples to the coarse lattice's interpolant on segments whose far cell is a rendered coarse leaf, then EXTENDS its surface one cell layer past those faces - the fine and coarse surfaces interpenetrate along the whole transition, so no viewing ray can slip between them. Conform masks are recomputed whenever the visible set changes; chunks whose stored mesh was built under a different mask rebuild automatically (the stale mesh keeps rendering until the rebuild lands, and each chunk view carries a content hash so consumers refresh their uploaded copies when the rebuild arrives).
- **Surface normals.** Vertex normals average the gradient of the (up to) four cells adjacent to the edge, in canonical min-corner form - a pure function of the edge's global position, so shared boundary vertices stay bit-identical across chunks, and steep creases shade smoothly across their faces.
- **Water patches.** The water depth tint uses PER-CHANNEL saturation depths (water absorbs red fastest, green next, blue last, so the ramp runs shallow turquoise -> green-teal mid shelf -> navy deep) sampled from `densityAt` directly (one pure field), so the tint is continuous across chunk and LOD boundaries; cells fully buried under terrain are skipped, which removes grazing-angle slivers where the sea sphere grazes the surface. A surf band (`water_foam_width_meters` / `water_foam_strength`) marks the breaker rim along the waterline - the crest modulation follows the offshore direction derived from the terrain gradient, so the foam reads as breaker ranks parallel to the coast, very shallow flats blend toward the sand-through-water tone (`water_shore_mix_meters`), and the wave-slope field carries two scales (the configured swell plus a half-wavelength chop) for close-range glint texture. The surface freezes where `sea_ice_temperature` is reached (see the oceans table); all follow the same pure-field rules, so ice edges and glint texture stay continuous across chunks.
- **Surface artefacts checklist.** Pale stains on green terrain: raise `biome_blend_width`/`slope_blend_width`. Dotted dark lines along ridges: raise `rock_slope_threshold`. Shredded cliff faces: lower `strata_warp`, raise `strata_band_meters`, lower `overhang_amplitude_meters`. Sub-sea pond webs near coasts: raise `coastal_flatten_meters`. Spiky silhouettes at any scale: raise `lod_band_limit_factor` (and keep the finest detail octave at 2.5+ voxels).
