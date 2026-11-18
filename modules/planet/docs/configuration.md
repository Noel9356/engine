# planet module configuration

All keys live in the `[planet]` section of the module's `config.toml` (copied into the runtime layout at `config/modules/planet/config.toml`). Every key is optional; missing keys keep the defaults mirrored in `PlanetGenerationParams`. `validate()` clamps every value into a safe range (lengths are capped relative to `radius_meters`, angles normalized, NaN replaced by defaults), so hostile config cannot produce degenerate geometry. Per-planet overrides arrive through the capability's `PlanetDescription` (`seed`, `radius_meters`, `sea_level_offset_meters`, `center_meters`).

The shipped defaults describe the showcase planet: radius 60 km, Earth-like relief fractions (~1% of the radius), large-scale relief layers scaled to the radius, close-range layers metric. Globe views read as a planet, ground views read as a landscape.

## Identity and scale

| Key | Default | Effect |
| --- | --- | --- |
| `seed` | `1337` | Root of every noise stream (SplitMix64 schedule). Changing it reshapes the entire planet. |
| `radius_meters` | `60000.0` | Mean planet radius (the datum sphere). Range [1, 1e9]. Scale the large-scale wavelengths with it (see usage notes). |
| `sea_level_offset_meters` | `0.0` | Raises (+) or lowers (-) the ocean surface relative to the datum. Clamped into the terrain's relief band: the sea sphere must stay inside the shell the octree prunes against, so offsets beyond the relief amplitudes are pulled back in. |
| `axis` | `[0, 1, 0]` | Spin axis; latitude for climate is measured against it. Normalized in `validate()`. |

## Octree / LOD

| Key | Default | Effect |
| --- | --- | --- |
| `leaf_grid_n` | `16` | Marching-cubes cells per leaf chunk axis (grid is `(n+1)^3` samples plus a 1-voxel apron). Must be EVEN so chunk faces sit on coarse planes. Range [4, 64]. |
| `target_voxel_meters` | `2.5` | Desired finest voxel size; drives the derived octree depth (`derivedMaxDepth`). The lower clamp is `radius / 131072` (depth-capped by the chunk key's 15-level limit). |
| `max_depth_limit` | `15` | Hard cap for the octree depth. The chunk key stores the depth in 4 bits, so 15 is also the structural maximum. |
| `lod_angular_voxel_radians` | `0.02` | Screen-space LOD: refine a node while its voxel subtends more than this many radians at the eye. Detail follows apparent size, independent of planet radius, viewport and zoom. Smaller = finer views everywhere, visible chunk count grows quadratically. |
| `lod_band_limit_factor` | `2.0` | Per-depth low-pass: density content below `voxel * factor` fades out before coarse chunks sample it. Marching cubes needs ~4 samples per wavelength; 2.0 renders content at full strength from 4 voxels up and fades it to zero at 2 voxels. Without it, sub-voxel octaves alias into random spikes on every coarse LOD surface (the "spiky globe"). Raise it to trade close-range roughness for cheaper, smoother chunks. |
| `horizon_culling` | `true` | Skip nodes fully inside the shadow cone of the guaranteed-solid inner shell ball. |
| `bounding_factor` | `1.4` | Root cube half-size = `radius * factor`. Range [1.05, 4]. |
| `max_concurrent_builds` | `16` | Chunk mesh builds in flight at once (the pool runs wide open; ground views select thousands of chunks). |
| `max_new_builds_per_update` | `48` | Build requests issued per `updatePlanet` call. Requests are ordered by apparent size (chunk width over distance, the same quantity the LOD rule is built on), so the widest-on-screen unresolved chunks build first. |
| `max_live_planets` | `16` | How many planets one module instance may keep alive at once (each owns a worker pool). `createPlanet` reports `CapacityReached` beyond the cap. |
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
| `terrace_strength` | `0.95` | 0 = smooth terrain, 1 = full plateaus (mesa country). Terracing is gated to genuinely steep country (cliff mask) inside the field, so open slopes never corduroy. |
| `terrace_steps` | `9.0` | Approximate plateau count across the relief range (step height = total relief / steps; 9 steps over the showcase relief makes ~200 m benches). |
| `terrace_riser_fraction` | `0.15` | Fraction of each terrace step the RISER spans. The riser's grade is the flank slope divided by this fraction, so small values build genuinely cliff-like faces: 0.15 turns a 0.35-grade flank into a ~60-degree rock wall (with strata and overhang ledges riding the same mask); 0.4 smears the step into a long shoulder. Below ~0.1 the riser thins under one voxel. |
| `cliff_wavelength_meters` | `1400.0` | Scale of the cliff-mask variety modulation (spacing between mask-band variations). |
| `cliff_steepness_low` | `0.42` | Cliff-mask gate over the terrain's steepness energy: where the mask opens (terracing/strata/overhangs begin). Lower widens terraced country, higher restricts it to the steepest flanks. |
| `cliff_steepness_high` | `0.70` | Steepness energy where the mask saturates (full-strength terraces, strata and overhangs). Keep at least ~0.2 above `cliff_steepness_low`. |
| `cliff_sharpness` | `2.6` | Exponent concentrating the hill-flank term of the cliff mask. Higher narrows terraced hill country into tighter bands; the mountain-crest term is unaffected. |

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
| `abyssal_depth_meters` | `3500.0` | Extra depth the sea floor sinks toward as the continent field dips into its basins: the coast keeps a shallow, sunlit continental shelf that slopes into dark abyssal plains, instead of one uniformly shallow flooded plain (the base relief amplitudes alone bottom out around the continent amplitude). The shelf is as wide and smooth as the continent geography itself, and every detail layer stays faded out over the deepened slope. Clamped to 8% of `radius_meters` so the floor stays a surface feature on small bodies. `0` disables. |
| `water_depth_tint_meters` | `70.0` | Depth over which the water surface tint saturates in the BLUE channel; red and green saturate at fixed fractions of it (0.28x and 0.6x - the physical absorption ordering, red first), so the ramp runs shallow turquoise -> green-teal mid shelf -> navy deep. |
| `water_enabled` | `true` | Generate water surface patches (marching cubes on the sea sphere). |
| `water_color_shallow` | `[0.070, 0.255, 0.275]` | Linear RGB at zero depth (the turquoise of sunlit shallow water over sand). |
| `water_color_deep` | `[0.012, 0.060, 0.130]` | Linear RGB at full tint depth. |
| `water_roughness` | `0.55` | Gloss hint handed to the renderer's water material. Low values give mirror-sharp sun glints, which makes the coarse far-water facets band at grazing angles; higher values spread the highlight. |
| `sea_ice_temperature` | `0.10` | Sea-level temperature below which the water surface freezes. The field is the same climate field that drives the land's freeze line, so polar oceans ice over where the land turns tundra, with an organic, noise-shaped edge. `0` disables sea ice. |
| `sea_ice_transition` | `0.05` | Temperature range around `sea_ice_temperature` over which the ice color blends in. |
| `water_color_ice` | `[0.78, 0.85, 0.94]` | Linear RGB of the frozen water surface; slightly blue against the warm-white cloud tops so polar caps read as ice, not weather. |
| `water_wave_amplitude` | `0.16` | Radian-scale tilt applied to the water surface normals by a deterministic two-scale noise field (the configured swell plus a half-wavelength chop at a third of the tilt), giving the sea a sun-glint texture with sparkle at close range. The tilt fades out under the LOD band limit like every other noise term, so coarse chunks keep a smooth sea. `0` gives a perfectly smooth surface. |
| `water_wave_wavelength_meters` | `8.0` | Feature size of the wave-swell field (the chop rides at half of it). |
| `water_foam_width_meters` | `2.5` | Depth band under the surface that reads as surf - the breaker rim hugging the waterline. The band's falloff is cubed, so the bright rim stays at the shore even on ultra-gentle shelves (a plain smoothstep would wash whole shallow lagoons white). |
| `water_foam_strength` | `0.60` | 0 = no surf, 1 = a solid white breaker band. The crest modulation keys to the offshore direction (read from the terrain gradient at the waterline), so the band breaks into elongated breaker ranks running parallel to the coast instead of isotropic patches; between ranks a base wash keeps the rim continuous. |
| `water_shore_mix_meters` | `3.0` | Depth over which the water surface blends toward the beach-sand tone transmitted by the water's own hue, so very shallow flats read as sandy turquoise (the bottom seen through the sheet) instead of one pale sheet. `0` disables the blend. |

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
| `moisture_latitude_bands` | `true` | Circulation-cell humidity belts (see the four amplitude keys below). |
| `moisture_belt_equator_amplitude` | `0.20` | Humidity added by the ITCZ wet belt on the equator. |
| `moisture_belt_subtropical_amplitude` | `-0.26` | Humidity shift of the subtropical dry belts (~26 degrees latitude). |
| `moisture_belt_temperate_amplitude` | `0.10` | Humidity added by the wet belt over the temperate westerlies (~54 degrees). |
| `moisture_belt_polar_amplitude` | `-0.06` | Humidity shift of the polar highs (~88 degrees). Belt centers/widths follow the classic circulation cells; `0` removes a belt. |
| `color_variation` | `0.34` | Per-point albedo variation (0 = flat palette). The base and mid-range scales apply as a multiplicative hue tint (each channel weighted differently, so the mottle reads as warm/cool patches); the ground scale is ADDITIVE in absolute color units and rotates the hue - positive noise pushes a point toward its dry, warm direction (red and green lift, blue sinks) and negative noise toward the lush, cool one. Absolute offsets are what a close-range eye sees: dark albedos would swallow a proportional wiggle of the same nominal strength. Octave scales below the chunk's VERTEX spacing fade out (band-limited like the geometric noise), so coarse LOD chunks do not alias the mottle into speckle. |
| `color_variation_wavelength_meters` | `260.0` | Feature size of the albedo jitter field. The field carries three scales: the configured wavelength mottles biome sheets at globe scale, an octave at an eighth of it covers the mid-range, and an octave at `color_variation_ground_wavelength_meters` keeps ground-level views from reading as one unbroken color. |
| `color_variation_ground_wavelength_meters` | `11.0` | Near-field mottle octave of the albedo jitter field; the surface texture a walking camera sees. Keep it above roughly twice the finest voxel size - the LOD band limit fades shorter octaves out entirely (the showcase's 2.6 m voxels carry an 11 m mottle at full strength). Clamped to at most `color_variation_wavelength_meters`. |
| `soil_patch_strength` | `0.30` | Erosion patches: a sparse ground-scale field pulls vegetated ground toward the bare-soil tone (the rock palette row), so biome sheets break into soil scars, gravel fans and worn earth instead of one unbroken color. The classification window is deliberately narrow, so the patches stay sparse and distinct rather than washing the sheet in broad stains. Skipped where rock, snow or beach already own the surface; band-limited at the chunk's vertex spacing like every other noise term. `0` disables. |
| `soil_patch_wavelength_meters` | `60.0` | Feature size of the erosion patches. Keep it above roughly four times the finest voxel so the field survives the LOD band limit. |
| `dry_patch_strength` | `0.45` | Dryness mosaic: a broad ground-scale field senesces vegetated ground - the current biome tone lerps toward its dry, yellow-tan counterpart (the color scaled toward the red end), so grasslands, meadows and forest floors break into the lush/dry mosaics real vegetation shows at walking and mid distance. The dry tone derives from the blended biome color itself, so every biome keeps its own dry direction (grass goes khaki, forest olive, tundra straw) without a second palette. Skipped where rock, snow or beach own the surface; band-limited at the chunk's vertex spacing. `0` disables. |
| `dry_patch_wavelength_meters` | `30.0` | Feature size of the dryness mosaic. Keep it above roughly four times the finest voxel so the field survives the LOD band limit. |
| `snow_line_meters` | `2400.0` | Sea-level snow line; drops as the local climate cools. Only true peaks are glaciated; polar caps come from latitude alone. |
| `beach_width_meters` | `5.0` | Shoreline band above sea level - wide enough to read as sand from a standing eye. |
| `beach_wet_width_meters` | `6.0` | Swash band above the waterline that reads as wet sand (real beaches darken sharply where the last wave reached). 0 disables. |
| `beach_wet_darkening` | `0.45` | How dark the wet band gets (0 = dry sand everywhere, 1 = near-black rim). |
| `rock_slope_threshold` | `0.46` | Slope (1 - dot(normal, radial)) where bare rock takes over. Steep faces scrape soil and grass into bare rock; rolling hills stay green. |
| `rock_strata_contrast` | `0.24` | Luminance banding of rock faces at the strata band spacing (sediment stripes). The band thickness follows a low-frequency warp (beds vary, and the gate's wide spread keeps most beds faint, so banded stretches read as geology rather than a periodic pattern) and the stripe fades out below two samples per bed at the chunk's vertex spacing, so coarse chunks never alias it into per-vertex speckle. 0 disables; keyed to `strata_band_meters`. |
| `biome_blend_width` | `0.06` | Half-width of the biome color blend on the temperature/moisture axes. Biome colors interpolate between palette rows instead of hard-switching per vertex (hard switches smear into pale stains across triangles). |
| `slope_blend_width` | `0.08` | Blend half-width around the rock slope threshold. |
| `elevation_blend_fraction` | `0.12` | Snow-line and water-band color blends as a fraction of the band width. |
| `biome_palette` | 16 rows | Linear RGB rows in `Biome` enum order (deep ocean .. rock). The rows carry the albedos the surface photographs through a scene's exposure: vegetation rows sit near real canopy/grass reflectances (dark and saturated), sand and tundra stay bright, snow keeps its ~0.9, and the ocean-floor rows sit deliberately darker than the water surface colors. Entries may be omitted from the end. |

## Surface walking

A kinematic, point-feet character controller lives in the module (`SurfaceWalker`, see `walker.hpp`): gravity pulls toward the planet center, movement lives in the tangent plane, and terrain collision queries the same pure density field the mesher samples (a march-and-bisect along the local radial direction), so overhang ledges and cave floors are walkable without any collision proxy geometry. A full physics module is not needed for this: the field answers the ground queries directly, and the controller stays deterministic (fixed inputs walk one exact path, which the demo's `--self-test` asserts).

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
| `swim_enabled` | `true` | Float at the sea surface over water deeper than the wade band. |
| `swim_speed_mps` | `2.6` | Movement speed while swimming. |
| `swim_wade_depth_meters` | `1.2` | Water shallower than this stays walkable (the terrain floor carries the feet). |

## Sky backdrop

The module exposes a pure sky model (`atmosphere.hpp`): a zenith-to-horizon gradient keyed to the view direction's elevation above the local horizon, an altitude weight that fades the whole sky to space over `atmosphere_height_meters`, a sun-elevation tint (neutral through the day, warm gold through the low-sun band, deep blue at night), and an aerial-perspective sample for one sight line (the sun-tinted haze the air adds between the eye and a distant point - its weight grows exponentially with the sight-line length on a scale riding `atmosphere_height_meters` and fades out as the eye climbs into space, and its color is the horizon band under the same sun tint, so distant terrain melts into the sky behind it). All functions are pure and thread-safe, and the colors are linear RGB like the water colors, so a consumer can paint a sky dome, a frame clear color, or apply distance haze to their scene - the demo draws a gradient sphere around the walking eye, hands the horizon color to the renderer as the clear color, and fades its chunk draws toward the haze with distance (space views keep the atmosphere limb shell as their depth cue).

| Key | Default | Effect |
| --- | --- | --- |
| `atmosphere_enabled` | `true` | Master switch; `false` leaves the sky entirely to the consumer. |
| `atmosphere_height_meters` | `20000.0` | Eye altitude over which the sky fades to black (space). The sky is at full strength below 15% of this height. |
| `atmosphere_zenith_color` | `[0.040, 0.130, 0.460]` | Linear RGB at the zenith. The defaults are chosen for the renderer's shared tone mapping (which brightens midtones), so the pre-tone-map values sit deeper and more saturated than the sky they finally display. |
| `atmosphere_horizon_color` | `[0.220, 0.400, 0.780]` | Linear RGB at the horizon; below it the haze darkens gently so ground-level gaps read as atmosphere, not void. |

## Clouds

The sky model also carries a `CloudField` (`atmosphere.hpp`): a band-limited coverage field sampled on a spherical shell at `cloud_altitude_meters` above the datum, thresholded so `cloud_coverage` is the approximate sky fraction under cloud, with a fine breakup octave (amplitude `cloud_detail_strength`) for ragged cumulus edges, a seed-derived wind frame that stretches the banks along a deterministic flow direction (real weather systems organize into elongated streaks, not round blobs) with a low-frequency swirl bending their shapes, a very-low-frequency weather field that shifts the threshold regionally so banks organize into large cloudy and clear systems, and a latitude-band term keyed to the spin axis that adds the circulation-cell pattern every rotating atmosphere shows (a wet equatorial belt, dry subtropics, stormy mid-latitudes). Sampling is a pure function with a per-octave seed schedule derived from the planet seed, and the field is band-limited by the optional low-pass cutoff like every other noise term, so consumers with a coarse sample spacing fade the sub-spacing octaves out instead of aliasing them into speckle. The demo meshes the shell for its globe and orbit views (quads whose corners read clear are skipped, so the layer is geometry with organic cloud-shaped edges); the layer renders only for cameras outside the shell, since a camera between the surface and the shell would see the far side of the sphere through the sky above the horizon.

The same coverage field also drives the cloud SHADOWS: chunk meshes darken their terrain and water albedo under the coverage sampled along the sun ray through each surface point (see `cloud_shadow_strength`), so a bank's shadow falls away from the bank the way real cloud shadows do, and the shadow agrees with the cloud layer a consumer paints from the same field.

| Key | Default | Effect |
| --- | --- | --- |
| `clouds_enabled` | `true` | Master switch of the coverage field; `false` reports clear sky everywhere. |
| `cloud_coverage` | `0.42` | Approximate fraction of the sky under cloud. `0` disables. |
| `cloud_wavelength_meters` | `13000.0` | Feature scale of the bank field - raise it with `radius_meters` like the other large-scale layers. Around 0.2 of the radius gives Earth-like weather systems that span 3-6 features around the globe. |
| `cloud_altitude_meters` | `1200.0` | Height of the cloud shell above the datum. |
| `cloud_detail_strength` | `0.35` | Amplitude of the fine breakup octave relative to the base field; higher values shred the bank edges into ragged cumulus. |
| `cloud_edge_softness` | `0.22` | Half-width of the coverage threshold band: narrow values scatter crisp clumps, wide values grow soft decks with veiled edges whose thinning fringes a consumer can paint (the demo's cloud layers fade their contour geometry through the fringe instead of cutting off at a polygon boundary). |
| `cloud_weather_strength` | `0.45` | Amplitude of the very-low-frequency regional weather shift (positive weather = cloudier); stronger values organize the banks into larger cloudy and clear systems. |
| `cloud_weather_wavelength_multiple` | `4.0` | Wavelength of the regional weather field as a multiple of the cloud field's own wavelength: larger values organize the banks into fewer, broader weather systems. |
| `cloud_latitude_band_strength` | `0.16` | Amplitude of the latitudinal coverage bands (a wet equatorial belt, dry subtropics, stormy mid-latitudes), keyed to the spin axis. `0` gives a latitude-blind field. |
| `cloud_band_frequency` | `3.4` | Frequency of those bands over latitude (radians of phase per radian of latitude): ~3.4 places a wet equatorial belt, dry subtropics and wet mid-latitudes the way Earth's circulation cells do. |
| `cloud_color` | `[0.985, 0.975, 0.955]` | Linear RGB of the cloud tops; warm white, so the layer reads against the bluer sea ice at globe scale. |
| `cloud_shadow_strength` | `0.45` | How strongly the terrain and water albedo darkens under the coverage field (sampled along the sun ray through each point). `0` disables cloud shadows. |
| `cloud_shadow_sun_direction` | `[0.50, 0.45, 0.74]` | Unit vector of the sunlight that casts the cloud shadows - pair it with the direction your scene's lighting uses (the demo ships it next to its `[pbr] sun_direction` and warns when the two disagree). Points on the night side of this sun get no shadow. |

## Stars

The module exposes a seed-derived star field for the space backdrop (`atmosphere.hpp`): a direction-space lattice whose cell size follows `stars_density`, each cell hashing (seed + cell) into at most one star with a jittered position, a power-law brightness (most stars faint, a few bright) and a subtle color temperature (warm through white through blue). Two entry points serve two consumer styles: `starFieldColor` samples the field as a color (with the same band-limiting every other field applies, so a painted backdrop keeps the bright stars instead of aliasing the faint ones into speckle), and `forEachStar` enumerates the stars themselves for consumers that build one small disc per star. The demo meshes the per-star discs for its globe and orbit views and hides the layer inside the atmosphere.

| Key | Default | Effect |
| --- | --- | --- |
| `stars_enabled` | `true` | Master switch; `false` reports empty space everywhere. |
| `stars_density` | `0.5` | Star spacing: `0` sparse skies, `1` dense skies (the lattice tightens quadratically). |
| `stars_brightness` | `1.0` | Luminance multiplier of the whole field; the brightest stars saturate to white through the tone mapping. |

## Usage notes

- **Scaling a planet.** Raise `radius_meters` and scale the wavelengths you want to keep proportional (continents, mountains, regional, climate fields, snow line); close-range amplitudes stay metric (a 650 m mountain is a 650 m mountain on any planet). The domain warp amplitude is ANGULAR - its metric effect grows with R (0.18 bends ~10 km coastlines at 60 km radius). The derived octree depth grows automatically until voxels reach `target_voxel_meters` (or the depth cap); `validate()`'s lower voxel clamp is `radius / 131072`.
- **Performance knobs.** Visible chunk count scales roughly with `1 / target_voxel_meters^2` near the camera and with `lod_angular_voxel_radians` overall. `leaf_grid_n` trades chunk granularity against per-chunk cost. `max_concurrent_builds` / `max_new_builds_per_update` pace streaming (builds run on the module's dedicated worker pool, one thread per permitted concurrent build); caves cost two extra noise fields per sample. Terrain layers whose mask is zero for a sample (mountains over the ocean, detail fields under water) are skipped, and noise terms whose band-limit fade is zero at the chunk's cutoff are never evaluated at all, so coastal and ocean geometry is cheaper than mountainous terrain at the same depth. Cloud shadows cost one coverage evaluation per surface vertex (about two percent of a chunk build).
- **Chunk boundaries and seams.** Same-depth chunks weld bit-identically (one global lattice); across depth changes the octree keeps a 2:1 balance and the fine side CONFORMS its boundary-plane samples to the coarse lattice's interpolant on segments whose far cell is a rendered coarse leaf, then EXTENDS its surface one cell layer past those faces - the fine and coarse surfaces interpenetrate along the whole transition, so no viewing ray can slip between them. Conform masks are recomputed whenever the visible set changes; chunks whose stored mesh was built under a different mask rebuild automatically (the stale mesh keeps rendering until the rebuild lands, and each chunk view carries a content hash so consumers refresh their uploaded copies when the rebuild arrives).
- **Surface normals.** Vertex normals average the gradient of the (up to) four cells adjacent to the edge, in canonical min-corner form - a pure function of the edge's global position, so shared boundary vertices stay bit-identical across chunks, and steep creases shade smoothly across their faces.
- **Water patches.** The water depth tint uses PER-CHANNEL saturation depths (water absorbs red fastest, green next, blue last, so the ramp runs shallow turquoise -> green-teal mid shelf -> navy deep) sampled from `densityAt` directly (one pure field), so the tint is continuous across chunk and LOD boundaries; cells fully buried under terrain are skipped, which removes grazing-angle slivers where the sea sphere grazes the surface. A surf band (`water_foam_width_meters` / `water_foam_strength`) marks the breaker rim along the waterline - the crest modulation follows the offshore direction derived from the terrain gradient, so the foam reads as breaker ranks parallel to the coast, very shallow flats blend toward the sand-through-water tone (`water_shore_mix_meters`), and the wave-slope field carries two scales (the configured swell plus a half-wavelength chop) for close-range glint texture. The surface freezes where `sea_ice_temperature` is reached (see the oceans table); all follow the same pure-field rules, so ice edges and glint texture stay continuous across chunks.
- **Surface artefacts checklist.** Pale stains on green terrain: raise `biome_blend_width`/`slope_blend_width`. Dotted dark lines along ridges: raise `rock_slope_threshold`. Shredded cliff faces: lower `strata_warp`, raise `strata_band_meters`, lower `overhang_amplitude_meters`. Sub-sea pond webs near coasts: raise `coastal_flatten_meters`. Spiky silhouettes at any scale: raise `lod_band_limit_factor` (and keep the finest detail octave at 2.5+ voxels).
