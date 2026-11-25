// PlanetGenerationParams: every knob of the planet generator.
//
// No generation constant is hardcoded at call sites: defaults live here
// (mirrored in the module's config.toml), the module reads [planet]
// configuration keys into this struct at construct(), and per-planet
// overrides arrive through the capability's PlanetDescription. validate()
// clamps every field into a safe range so hostile config cannot produce
// degenerate geometry, and derives the aggregate bounds (maximum relief,
// octree depth, shell band) the octree culling relies on.

#pragma once

#include <cstdint>
#include <engine/planet/types.hpp>

#include <cmath>
#include <string>

namespace engine
{
namespace planet
{

// Fractal Brownian motion helper settings shared by several fields.
struct FbmParams
{
    std::uint32_t octaves    = 5;    // noise evaluations per fbm call
    double        lacunarity = 2.0;  // frequency multiplier per octave
    double        gain       = 0.5;  // amplitude multiplier per octave
};

// Hard cap on fBm octave counts everywhere (validate() clamps configured
// octave counts to this; the per-octave seed tables are sized accordingly).
inline constexpr std::uint32_t maxFbmOctaves = 12;

struct PlanetGenerationParams
{
    // ---- identity and scale --------------------------------------------------
    std::uint64_t seed           = 1;      // root of every noise stream
    double        radiusMeters   = 60000.0; // mean planet radius (datum). The showcase
    // default is a 60 km planet: with Earth-scale relief AMPLITUDES this keeps the
    // relief fraction near real planets (~1%), so globe views read as a planet rather
    // than a spiked marble. Large-scale relief wavelengths scale with the radius; the
    // close-range layers (hills and finer) stay metric - see config.toml.
    double        centerMeters[3] = {0.0, 0.0, 0.0};
    double        seaLevelOffsetMeters = 0.0; // + raises the oceans above the datum
    double        axis[3]        = {0.0, 1.0, 0.0}; // spin axis (latitude pole), normalized in validate()

    // ---- chunk cache -------------------------------------------------------------
    std::uint32_t chunkEvictionUpdates = 240; // updates a hidden mesh stays cached before eviction
    std::uint32_t chunkCacheCapacity  = 16384; // hard cap on cached chunk meshes (oldest evicted)

    // ---- octree / level of detail ---------------------------------------------
    std::uint32_t leafGridN          = 16;   // marching-cubes cells per leaf chunk axis
    double        targetVoxelMeters  = 2.5;  // desired finest voxel size (drives max depth)
    // Eye movement (meters) below which update() keeps the previous visible
    // selection instead of re-running the descent. The LOD decision for a
    // point cannot flip until the eye moves a good part of the distance to
    // the ring boundary, so a sub-voxel movement pins the whole selection:
    // descent, balancing, conform masks and stamping are all skipped. 0 (the
    // default) picks the derived finest voxel; a walking camera at human
    // speeds then re-selects every few dozen frames instead of every frame.
    // Large values trade nearby-LOD freshness for update headroom.
    double        selectionUpdateDistanceMeters = 0.0;
    // Octree node-record bookkeeping (selection caches): records remember
    // their solid/air probe verdict and cached geometry for this many
    // consecutive selections before the prune pass drops them, and the prune
    // runs every this many selections. Longer lifetimes keep returning views
    // (a walking camera revisiting ground, orbiting cameras) free of the
    // first-visit field probes at the cost of resident memory (a few dozen
    // bytes per record; ground views touch tens of thousands).
    std::uint32_t nodeRecordLifetimeUpdates = 96;
    std::uint32_t nodeRecordPruneIntervalUpdates = 16;
    std::uint32_t maxDepthLimit      = 15;   // hard cap for the octree depth (kept within the
                                             // 4-bit depth field of the chunk key, see types.hpp)
    // Per-depth angular (screen-space) refinement: a chunk refines while its
    // VOXEL subtends more than lodAngularVoxelRadians at the eye. Detail then
    // follows apparent size instead of raw eye distance: the whole visible
    // disk of a globe view refines to whatever depth makes voxels a few
    // pixels (the old distance rule kept entire planets at depth 0-1 - the
    // "low detailed globe"), surface views keep the finest depth within the
    // near field, and the rule is independent of planet radius, viewport and
    // zoom. The distance uses the node bounding-sphere point nearest the eye,
    // so a node stays coarse only when its whole volume is far enough away.
    double        lodAngularVoxelRadians = 0.02;
    // Per-depth low-pass on the density field: a marching-cubes lattice needs
    // ~4 samples per wavelength to reconstruct a surface without visible
    // jaggedness (2 samples per wavelength is the hard Nyquist floor, but at
    // that density every crest turns into a sawtooth). Content below
    // voxel * lodBandLimitFactor fades to zero, reaching full strength at twice
    // that wavelength - with the default 2.0, content at 4+ voxels renders at
    // full strength and nothing below 2 voxels survives at all. This is what
    // keeps coarse LOD chunks from reconstructing sub-voxel noise as random
    // spikes - the "spiky globe" artefact - and the finest leaf depth from
    // sawtoothing its own last octave. The coarse side of a depth transition
    // evaluates its lattice with its own (stronger) cutoff and the conforming
    // boundary machinery hands exactly those values to the fine side, so the
    // shared plane stays consistent with the coarse neighbor's mesh.
    double        lodBandLimitFactor = 2.0;
    bool          horizonCulling     = true; // skip nodes behind the planet horizon
    // Streaming coverage for visible chunks whose mesh has not finished
    // building: such a chunk draws its nearest finished ANCESTOR whose LOD
    // suits its distance (the same screen-space rule the selection uses,
    // with a 2x tolerance) instead of contributing nothing, and the
    // streamer additionally requests the chunk's coarsest pure ancestor -
    // the coarsest ancestor under which no finished visible chunk lies, so
    // its mesh covers the hole without overlapping finished geometry. The
    // planet therefore appears coarse-first (the widest gaps stream the
    // widest chunks) and sharpens in place as the fine meshes land, instead
    // of showing empty sky until the build queue catches up.
    bool          lodMissingFallback = true;
    // Scaffold request budget per update (see lodMissingFallback): at most
    // this many pure-ancestor builds are requested per update, competing on
    // apparent screen size so the widest coverage builds first. 0 disables
    // scaffold builds; the draw-side fallback then uses only ancestors that
    // exist anyway.
    std::uint32_t lodScaffoldBuildsPerUpdate = 8;
    double        boundingFactor     = 1.4;  // root cube half-size = radius * this
    std::uint32_t maxConcurrentBuilds = 16;  // in-flight async chunk mesh builds
    std::uint32_t maxNewBuildsPerUpdate = 48;  // build requests issued per update() call
    // Consecutive updates with queued work but no delivered mesh after which
    // update() builds one chunk inline on the calling thread (the streaming
    // fallback for machines too small to give the pool any CPU). Lower values
    // borrow more frame time for streaming; 0 keeps the pool-only behavior.
    std::uint32_t starvedInlineBuildIntervalUpdates = 30;
    // Upper bound on simultaneously live planets for one module instance
    // (each planet owns a worker pool; the cap keeps a runaway consumer from
    // stacking unbounded thread counts). createPlanet reports
    // CapacityReached beyond it.
    std::uint32_t maxLivePlanets = 16;

    // ---- relief shaping (meters unless noted) ---------------------------------
    double continentAmplitudeMeters = 560.0;  // ± height of the continent base field
    double continentWavelengthMeters = 34000.0; // feature scale of the continent field
    FbmParams continentsFbm{4, 2.0, 0.5};    // octaves etc. of the continent field. Fewer
    // octaves = fewer medium patches: the landmass cores stay contiguous and
    // coastline complexity comes from the warp and the hills near the shore.

    // domainWarpAmplitude is an ANGULAR displacement: at radius R it shears
    // features by roughly amplitude * R meters, so it must SHRINK as planets
    // grow (0.5 shreds a 60 km planet into archipelago filaments; 0.18 bends
    // coastlines by ~10 km there while keeping landmasses contiguous).
    double domainWarpAmplitude = 0.18;  // continent-coast warping strength (field units)
    double domainWarpWavelengthMeters = 14000.0;

    double mountainAmplitudeMeters  = 1100.0; // ridge height at mask = 1
    double mountainWavelengthMeters = 9000.0;
    double mountainMaskLow          = 0.08;  // continent field where mountains begin
    double mountainMaskHigh         = 0.45;  // continent field where mountains reach full height
    double mountainSharpness        = 1.25;   // exponent applied to the ridged crest field;
                                              // > 2 concentrates all height into narrow
                                              // needles (the "spiky planet" look)
    FbmParams mountainsFbm{5, 2.0, 0.5};

    // Mountain profile: a pure ridge sum puts every meter of height into
    // narrow crest lines - from orbit the ranges read as fields of needles
    // and close up the slopes are all rib. mountainMassFraction splits the
    // mountain amplitude between a broad, smooth "mass" field (wide uplifted
    // massifs, rounded flanks, real silhouette) and the crest field (ridge
    // lines riding on top of the mass). Range clustering gates the whole
    // mountain term with an even broader field so uplift groups into distinct
    // ranges separated by lower foothill country instead of carpeting every
    // landmass uniformly.
    double mountainMassFraction         = 0.50;  // [0,1] amplitude share of the broad mass
    // Wavelength multiple of the broad mountain mass field relative to the
    // crest field (the mass rides a multiple of mountainWavelengthMeters).
    double mountainMassWavelengthMultiple = 1.8;
    // Crest-line weight of the mountain profile at zero mass: the crest term
    // is crest * (coupling + (1 - coupling) * mass), so higher values keep
    // ribbed crest lines alive across low-mass country and lower values
    // concentrate the ridges on the big massifs.
    double mountainCrestMassCoupling    = 0.35;  // [0,1]
    double rangeClusterStrength         = 0.55;  // [0,1] mountain clustering into ranges
    double rangeClusterWavelengthMeters = 20000.0;
    // Continent-field window where the range-cluster gate opens / saturates
    // (continent-field units): uplift clusters where the cluster field sits
    // inside this band.
    double rangeClusterGateLow  = -0.35;
    double rangeClusterGateHigh = 0.45;

    // ---- eroded mountains (the Advanced Terrain Erosion filter) ---------------
    // The mountain belt relief can come from an analytic erosion filter (see
    // erosion.hpp for the technique and its attribution): directional gully
    // stripes aligned with the terrain slope carve branching valleys and
    // leave sharp ridges between them, evaluated per point in isolation so
    // streaming chunks sample the same eroded field at any size. The belts
    // keep the same continent mask and range clustering as the fBm profile
    // above; mountainErosionWeight blends the two profiles (1 = fully
    // eroded mountains, 0 = the classic mass-plus-crest profile). All filter
    // knobs below mirror the published parameters.
    double mountainErosionWeight = 1.0; // [0,1] blend of eroded vs fBm mountain profile
    // Width of the cubemap face-blend window (fraction of the dominant-axis
    // coordinate): samples near face seams blend the two overlapping faces
    // across this band so stripes continue around the sphere.
    double mountainErosionFaceBlendWidth = 0.02;
    // Base terrain the filter erodes. One unit of the filter's 2D domain
    // spans mountainWavelengthMeters of arc at the cubemap face centers, so
    // the massif scale is the wavelength divided by the frequency and the
    // amplitude is in the filter's [0, 1] height band.
    double        mountainErosionTerrainFrequency  = 1.0;
    double        mountainErosionTerrainAmplitude  = 0.25;
    std::uint32_t mountainErosionTerrainOctaves    = 3;
    double        mountainErosionTerrainGain       = 0.10;
    double        mountainErosionTerrainLacunarity = 2.0;
    // Height offset the filter applies (a constant blended with the negated
    // fade target), scaled by the accumulated erosion magnitude: the
    // recommended -0.65 compensates the ridge-up bias of the gully stack and
    // keeps the eroded belt's mean height on the input terrain's average.
    double mountainErosionHeightOffsetLow  = -0.35; // constant part of the offset
    double mountainErosionHeightOffsetBlend = 0.0;  // [0,1] blend toward -fadeTarget
    // Gully stack: scale sets the coarsest stripe wavelength (in units of
    // mountainWavelengthMeters times cellScale), strength the erosion
    // amplitude, gullyWeight the stripes' share of each octave against the
    // fade target (lower values keep peaks pointy), detail how strictly
    // finer octaves are restricted to steeper ground, cellScale the stripe
    // cell size, normalization the partial stripe re-normalization (0 =
    // full, 1 = none), and octaves / gain / lacunarity the fractal stack
    // over the stripe scale.
    double        mountainErosionScale         = 0.28;
    double        mountainErosionStrength      = 0.30;
    double        mountainErosionGullyWeight   = 0.55;
    double        mountainErosionDetail        = 1.2;
    double        mountainErosionCellScale     = 0.70;
    double        mountainErosionNormalization = 0.50;
    std::uint32_t mountainErosionOctaves       = 5;
    double        mountainErosionGain          = 0.80;
    double        mountainErosionLacunarity    = 2.0;
    // Edge rounding (ridge, crease): softens the sharp ridge crests and
    // gully creases the stack produces (real peaks are never razor-sharp and
    // gully floors collect sediment). The global multiplier scales both and
    // its gain compounds it per octave.
    double mountainErosionRoundingRidge  = 0.10;
    double mountainErosionRoundingCrease = 0.00;
    double mountainErosionRoundingGlobal = 0.10;
    double mountainErosionRoundingGain   = 2.0;
    // Slope onset thresholds (input slope, per-octave slope, ridge-map
    // slope, ridge-map mask): how much slope the masks demand before
    // gullies appear on the input terrain, on each octave, and on the ridge
    // map's own fade chain. Higher values push gullies onto ever steeper
    // ground only.
    double mountainErosionOnsetInput     = 1.25;
    double mountainErosionOnsetOctave    = 1.25;
    double mountainErosionOnsetRidgeMap  = 2.80;
    double mountainErosionOnsetRidgeMask = 1.50;
    // Assumed slope (magnitude, blend): the gully directions use a fixed
    // assumed slope magnitude blended over the input terrain's own slope, so
    // gullies stay aligned through rounded peaks and valley floors where
    // the real slope vanishes. Blend 1 = fully assumed (the recommended
    // setting; the filter's own derivatives are approximations).
    double mountainErosionAssumedSlope      = 0.70;
    double mountainErosionAssumedSlopeBlend = 1.0;
    // Output remap: the filter's height (a band around 0.5) is remapped
    // linearly onto the mountain profile [0, 1] between these bounds, so
    // valley floors sit at the belt base and crests reach the full mountain
    // amplitude. The defaults bracket the recommended shaping's output
    // range; widen them for gentler relief, narrow them for wilder.
    double mountainErosionProfileLow  = 0.30;
    double mountainErosionProfileHigh = 0.52;

    // Regional relief between the continent and hill bands: broad plateaus,
    // basins and valley systems. This is the layer that gives landmasses
    // structure at globe scale (where anything finer than a rendered voxel is
    // invisible) and gives mid-altitude views large, readable landforms.
    double regionalAmplitudeMeters  = 80.0;
    double regionalWavelengthMeters = 6500.0;
    FbmParams regionalFbm{3, 2.0, 0.5};

    double hillAmplitudeMeters     = 110.0;  // medium-frequency rolling terrain
    double hillWavelengthMeters    = 1000.0;
    FbmParams hillsFbm{5, 2.0, 0.5};

    // Fine detail layer: the surface roughness close-range cameras actually
    // see (rock ribs, soil ripples, scree). Without it the terrain between
    // the hill wavelength and the voxel size is featureless and reads as
    // plastic. Weighted like the overhang term: mountains and cliffs carry
    // full strength, plains keep detailPlainsFraction.
    double detailAmplitudeMeters  = 14.0;    // roughness height at the base wavelength
    double detailWavelengthMeters = 120.0;  // feature size of the finest terrain layer.
    // Keep the base wavelength >= ~26x the target voxel: the 4-octave stack then
    // bottoms out near the band limit (4 voxels) and no octave is wasted below it.
    double detailPlainsFraction   = 0.22;   // detail strength kept on flat ground;
                                            // higher values boil plains into noise lumps
    FbmParams detailFbm{4, 2.0, 0.5};

    double terraceStrength  = 0.95;  // 0 = smooth terrain, 1 = full plateaus
    double terraceSteps     = 9.0;   // approximate plateau count across the field
    // Fraction of each terrace step the RISER spans. The riser's grade is
    // the flank slope divided by this fraction, so small values build
    // genuinely cliff-like faces: 0.15 turns a 0.35-grade flank into a
    // ~60-degree rock riser (with the strata and overhang terms riding the
    // same cliff mask, the risers read as layered cliff walls, while the
    // treads stay flat benches). 0.4 smears the step into a long shoulder.
    double terraceRiserFraction = 0.15;
    double cliffWavelengthMeters = 1400.0; // wavelength of the terrace mask field
    // Cliff-mask gate over the terrain's own steepness energy (the slope the
    // composed hill/crest fields carry at the cliff wavelength). Terracing,
    // strata banding and overhang weighting all ride this mask, so the gate
    // decides where mesa country forms: at the defaults it opens on genuine
    // hill flanks and massif slopes, keeping plains smooth.
    double cliffSteepnessLow  = 0.42; // steepness energy where the cliff mask opens
    double cliffSteepnessHigh  = 0.70; // steepness energy where the mask saturates
    double cliffSharpness      = 2.6;  // exponent concentrating the crest bands; higher
                                       // narrows terraced country into tighter bands

    // ---- 3D overhangs (what a heightfield cannot do) ---------------------------
    double overhangAmplitudeMeters   = 38.0;  // 3D noise added to the signed density
    double overhangWavelengthMeters  = 300.0; // feature size of the overhang noise
    double overhangPlainsFraction    = 0.06;  // overhang strength kept on flat ground
                                              // (0 = only mountains/cliffs displace in 3D)
    double strataAmplitudeMeters     = 3.0;   // banded rock strata on cliffs. NOTE: on a slope
                                              // of grade s, this term adds +- 2*pi*A/band to the
                                              // local slope - large amplitudes turn whole hillsides
                                              // into corduroy (every band a vertical riser). The term
                                              // is additionally gated to genuinely terraced country
                                              // (cliff mask >= ~0.65) so open slopes stay clean
    double strataBandMeters          = 34.0;  // spacing of the strata bands
    double strataWarp                = 1.1;   // noise bending of the bands; >= 2 folds them
                                              // into nested ripple rings (visual artefact)
    // Cliff-mask window where the strata banding gate opens / saturates (the
    // gate keeps banding off open slopes - see strataAmplitudeMeters).
    double strataGateLow  = 0.34;
    double strataGateHigh = 0.85;

    // ---- geologic provinces -----------------------------------------------------
    // A very-low-frequency field classifies every landmass region into one of
    // three rock characters - layered sedimentary country (warm bedded
    // tones, strong strata and terraces), shield/volcanic country (cool
    // gray tones, weaker strata) and pale carbonate patches (limestone) -
    // and biases the terrain's strata and terrace behavior to match. The
    // rock characters ride the SAME seed streams in density and climate, so
    // the geometry and the albedo agree about what the ground is made of.
    bool   provincesEnabled       = true;
    double provinceWavelengthMeters = 0.0; // 0 = derived (1.6 x continent wavelength)
    double provinceRockVariation  = 0.55;  // [0,1] strength of the rock palette tint
    double provinceStrataBias     = 0.35;  // [0,1] strata amplitude bias toward sedimentary country
    double provinceTerraceBias    = 0.30;  // [0,1] terrace strength bias toward sedimentary country

    // ---- coastal shaping --------------------------------------------------------
    // Real shorelines are planed off by waves and sediment: the medium-frequency
    // detail (hills, terraces, overhangs, strata) fades toward the datum so
    // coastlines slope into the sea instead of zig-zagging across the waterline
    // (which reads as a web of sub-sea ponds with beach rims).
    double coastalFlattenMeters = 80.0;   // half-width of the flattening band around sea level
    double coastalFlattenFloor  = 0.10;   // residual detail fraction right at the shoreline
    // Fractions of coastalFlattenMeters where the taper reaches full damping
    // (below fullFraction of the band) and full detail strength (above
    // reachFraction). The steepened curve keeps shoals from speckling.
    double coastalFlattenFullFraction  = 0.45;
    double coastalFlattenReachFraction = 1.6;

    // ---- caves (optional; cost is two extra noise fields per sample) -----------
    bool   cavesEnabled            = false;
    double caveThreshold           = 0.16;  // |worm field| below this carves
    double caveWormWavelengthMeters = 240.0;
    double caveCarveMeters         = 60.0;  // max depth of carved density

    // ---- ocean bands (depth thresholds for water color and ocean biomes) -------
    double shallowWaterMeters = 18.0;   // shoreline band above the sea floor
    double deepWaterMeters    = 140.0;  // depth where the ocean counts as "deep"
    // Abyssal deepening: the sea floor sinks from the shoreline toward this
    // extra depth as the continent field dips into its basins, carving real
    // ocean profiles - a shallow, sunlit continental shelf that slopes into
    // dark abyssal plains - instead of one uniformly shallow shelf (the
    // base relief amplitudes alone bottom out around the continent
    // amplitude, which reads as a flooded plain from above). 0 disables.
    double abyssalDepthMeters = 3500.0;
    // Continent-field window the abyssal mask rides (continent-field units,
    // negative = basins): the sea floor sinks where the continent field sits
    // inside this band. Keep low < high; the window's lower edge shapes how
    // far from the coast the deepening starts.
    double abyssalMaskLow  = -0.60;
    double abyssalMaskHigh = -0.02;
    // Height fractions of the continent amplitude where the submarine fade
    // closes (below lowFraction of the amplitude, ocean-floor detail is
    // fully suppressed) and opens (above highFraction, land detail is
    // untouched).
    double submarineFadeLowFraction  = 0.02;
    double submarineFadeHighFraction = 0.06;

    // ---- climate ---------------------------------------------------------------
    double equatorTemperature = 1.0;   // temperature01 at the equator, sea level
    double poleTemperature    = 0.02;  // temperature01 at the poles, sea level
    double temperaturePerturbation = 0.09; // amplitude of the temperature noise field
    double temperatureWavelengthMeters = 20000.0;
    double temperatureLapseMeters = 1100.0; // meters of altitude per 1.0 temperature01
    double moistureWavelengthMeters = 14000.0;
    double moisturePerturbation = 0.24;
    double moistureWarpAmplitude = 0.22;    // bends the moisture field so biome patches
                                            // follow organic, elongated shapes instead of
                                            // round camouflage blobs
    // Coastal humidity: onshore winds keep low land near the sea humid (real
    // coastal plains are greener than dry interiors - without this the flat
    // tapered shoreline band paints a beige desert fringe around every
    // landmass and the globe reads as speckled).
    double coastalMoistureBoost = 0.18;     // moisture added right at the shoreline
    double coastalMoistureBandMeters = 400.0; // elevation over which the boost fades out
    // Continental interiors dry out away from the ocean: humidity drops as
    // the continent field climbs deep into its positive range (the same
    // field that shapes the landmasses), so landmass cores trend arid and
    // the coasts keep their green fringe. 0 disables.
    double climateContinentalityStrength = 0.30;
    // Rain shadow: humidity drops DOWNWIND of mountain country. The mass of
    // the mountain field is sampled upwind along the prevailing wind (see
    // climateWindZonesEnabled) at the mountain wavelength's distance, so the
    // lee side of every range carries a dry belt the way orographic
    // precipitation leaves one. 0 disables.
    double climateRainShadowStrength = 0.35;
    // Prevailing winds for the rain shadow: true follows the latitude zones
    // every rotating planet shows (easterly trades inside ~30 degrees,
    // westerlies over the temperate band, easterly polar winds), false blows
    // uniformly from the configured eastern hemisphere direction.
    bool   climateWindZonesEnabled = true;
    bool   moistureLatitudeBands = true;    // ITCZ wet belt + subtropical dry belts
    // Amplitudes of the four circulation-cell humidity belts (see climate.hpp
    // for the fixed belt geometry: an ITCZ wet belt on the equator, subtropical
    // dry belts, a mild wet belt over the temperate westerlies and weak polar
    // highs). Positive wets the belt's latitude band, negative dries it; 0
    // removes the belt. Keys: moisture_belt_*_amplitude.
    double moistureBeltEquatorAmplitude    = 0.20;
    double moistureBeltSubtropicalAmplitude = -0.26;
    double moistureBeltTemperateAmplitude  = 0.10;
    double moistureBeltPolarAmplitude      = -0.06;
    double colorVariation = 0.34;           // per-point albedo jitter (0 = flat palette)
    double colorVariationWavelengthMeters = 260.0; // feature size of the albedo jitter field
    double colorVariationGroundWavelengthMeters = 11.0; // near-field mottle for ground-level views;
                                                        // keep above ~2x the finest voxel or the LOD
                                                        // band limit fades it out entirely
    // Erosion patches: a sparse ground-scale field that pulls the albedo of
    // vegetated ground toward the bare-soil tone (the rock palette row), so
    // biome sheets break up into soil scars, gravel fans and worn earth the
    // way real ground does. Strength 0 disables; the wavelength is the patch
    // feature size (keep it above ~4x the finest voxel so the field survives
    // the LOD band limit). Keys: soil_patch_strength, soil_patch_wavelength_meters.
    double soilPatchStrength          = 0.30;
    double soilPatchWavelengthMeters  = 60.0;
    // Dryness mosaic: a broad ground-scale field that senesces vegetated
    // ground - the current biome tone shifts toward its dry, yellow-tan
    // counterpart over roughly a third of the surface, breaking grasslands,
    // meadows and forests into the lush/dry mosaics real vegetation shows at
    // walking and mid distance. Strength 0 disables; the wavelength is the
    // mosaic feature size (keep it above ~4x the finest voxel). Keys:
    // dry_patch_strength, dry_patch_wavelength_meters.
    double dryPatchStrength           = 0.45;
    double dryPatchWavelengthMeters   = 30.0;
    double snowLineMeters = 2400.0;          // sea-level snow line (drops as climate cools);
                                            // only the highest massifs are glaciated, polar
                                            // caps come from latitude alone. Earth's
                                            // permanent snow line sits near 4500 m at the
                                            // equator and ~3000 m in the Alps; against this
                                            // planet's relief amplitudes the scaled value
                                            // keeps mid-latitude hills green and snow
                                            // pinned to high country and polar air.
    double beachWidthMeters = 5.0;          // shoreline band width above sea level (wide
                                            // enough to read as sand from a standing eye)
    // Shoreline wobble: the band's effective width undulates along the shore
    // with a low-frequency simplex stream (a fixed wavelength of a few dozen
    // meters, band-limited like every other colour term), so the sand/grass
    // boundary follows an irregular, meandering line instead of the
    // ruler-straight elevation contour the coastal flattening leaves behind -
    // on a flattened coast the constant-width band photographs as a cut-out
    // slab. Stretches where the wobble pinches the width toward zero simply
    // have no beach, the way real coasts alternate dune, marsh and rock.
    // 0 keeps the constant configured width everywhere.
    double beachShorelineWobble = 0.45;     // +- fraction of the beach width the shoreline meanders
    // Wet-sand band above the waterline: real beaches darken sharply in the
    // swash zone. Two keys keep the band a configuration decision - width and
    // how dark the wet sand reads.
    double beachWetWidthMeters = 6.0;       // elevation band above sea level that reads wet
    double beachWetDarkening   = 0.45;     // 0 = dry sand everywhere, 1 = near-black wet band
    double rockSlopeThreshold = 0.46;       // slope (1 - up-dot-normal) where rock takes over;
                                            // steep faces scrape soil and grass into bare
                                            // rock while rolling hills stay green
    double rockStrataContrast = 0.75;       // luminance banding on rock faces, keyed to the
                                            // strata band spacing (sediment layer stripes);
                                            // strong values read as dirt smears at grazing light

    // ---- surface texturing -----------------------------------------------------
    // The mesher partitions every chunk's terrain into a GROUND and a ROCK
    // surface group (see meshing.hpp) and tiles texture coordinates over each
    // with a cubemap-style projection of the dominant normal axis. These keys
    // set how many METERS one texture tile spans per group: the generator's
    // world-space tiling and the consumer's textures meet here (a 3 m tile
    // with a 1024-texel texture photographs at ~3 mm per texel). Keep every
    // tile at or above the finest voxel's band limit - a tile much smaller
    // than a coarse chunk's voxel fades out with the LOD anyway.
    double terrainTextureTileMetersGround = 8.0;  // meters per texture tile on ground surfaces
    double terrainTextureTileMetersRock   = 14.0;  // meters per texture tile on rock surfaces
    // The water sheet tiles the same projection keyed to the radial
    // direction (stable against the wave-slope normal wiggles).
    double waterTextureTileMeters = 8.0;          // meters per texture tile on the sea sheet
    // Anti-repetition of the tiled terrain frames. The UV warp bends the
    // tile lattice with a low-frequency noise field (a pure world-space
    // function of position, so chunk borders and LOD transitions keep their
    // shared-vertex semantics), which breaks the global phase alignment
    // that otherwise photographs every tile at the same world offset; the
    // warp amplitude is expressed in texture tiles, so it scales with the
    // tile sizes automatically. Values above ~0.4 begin to fight the rock
    // normal map's per-triangle tangent reconstruction, whose UV gradient
    // must keep dominating the warp's. The warp band-limits like every
    // other noise term (it fades out below the chunk's own LOD cutoff).
    double textureUvWarpStrength          = 0.35;
    double textureUvWarpWavelengthMeters  = 64.0;
    // The per-chunk frame rotation turns each chunk's terrain tiling by a
    // deterministic multiple of 90 degrees (hashed from the planet seed and
    // the chunk address, one draw per surface group), so neighboring chunks
    // never photograph the same scan in the same orientation. 90-degree
    // turns map the tile lattice onto itself, so the frames stay seamless
    // inside every chunk and tileable textures keep tiling.
    bool textureChunkRotation = true;

    // Biome colors are BLENDED, not hard-switched: per-vertex hard switches smear
    // into pale stains across triangles (the "blotch" artefact). These widths
    // control how softly each transition interpolates between palette rows.
    double biomeBlendWidth       = 0.06;  // half-width on the temperature/moisture axes
    double slopeBlendWidth       = 0.08;  // half-width around the rock slope threshold
    double elevationBlendFraction = 0.12; // snow/water band blend as a fraction of band width

    // Biome palette, linear RGB rows in Biome enum order (biomeCount rows of 3).
    // The rows carry the albedos the surface photographs through the demo's
    // exposure (albedo x sun x NdotL lands land mid-tone and only snow near
    // white): vegetation rows sit near real canopy/grass reflectances (dark
    // and saturated), sand and tundra stay bright, snow keeps its ~0.9.
    float biomePalette[biomeCount * 3] = {
        // deep ocean      ocean            shallow ocean
        0.006f, 0.024f, 0.055f,   0.013f, 0.048f, 0.100f,   0.034f, 0.115f, 0.165f,
        // beach           desert           savanna
        0.550f, 0.460f, 0.280f,   0.480f, 0.360f, 0.180f,   0.340f, 0.300f, 0.110f,
        // grassland       shrubland        temperate forest
        0.130f, 0.220f, 0.050f,   0.150f, 0.190f, 0.070f,   0.060f, 0.130f, 0.035f,
        // seasonal forest tropical forest   temperate rainforest
        0.050f, 0.130f, 0.030f,   0.035f, 0.100f, 0.025f,   0.030f, 0.090f, 0.030f,
        // taiga           tundra           snow
        0.050f, 0.085f, 0.048f,   0.260f, 0.240f, 0.170f,   0.870f, 0.900f, 0.930f,
        // rock (cliff faces)
        0.280f, 0.250f, 0.210f,
    };

    // ---- surface walking (kinematic controller; see walker.hpp) ----------------
    // Everything a point-feet character needs to cross the terrain: planet-
    // centric gravity, walk/sprint/swim speeds, the jump impulse, the camera
    // eye offset, and the collision windows (how far the feet stick to the
    // ground going downhill and how high a ledge they can step onto).
    double surfaceGravityMetersPerSecond2 = 9.81; // constant pull toward the center
    double walkSpeedMetersPerSecond       = 4.5;  // ground movement speed
    double walkSprintMultiplier           = 2.6;  // sprint speed = walk speed * this
    double jumpSpeedMetersPerSecond       = 5.0;  // upward impulse; 0 disables jumping
    double eyeHeightMeters                = 1.7;  // camera height above the feet
    double walkGroundSnapMeters           = 0.9;  // drop the feet follow without falling
    double walkMaxStepUpMeters            = 0.55; // highest ledge walked straight onto
    double walkMaxClimbGrade              = 3.5;  // rise/run above this blocks (tan; ~74 deg)
    // Speed cap of the falling integration. Terminal velocity bounds the
    // landing probe the fall needs (the probe scales with speed * delta), so
    // a long fall resolves the ground with a bounded number of march steps
    // instead of accelerating without limit toward a tunnel-through.
    double walkTerminalVelocityMetersPerSecond = 55.0;
    bool   swimEnabled                    = true; // float at the sea surface over deep water
    double swimSpeedMetersPerSecond       = 2.6;  // movement speed while swimming
    double swimWadeDepthMeters            = 1.2;  // water shallower than this stays walkable

    // ---- atmosphere (sky backdrop; see atmosphere.hpp) --------------------------
    // The sky a camera inside the air sees: a zenith-to-horizon gradient
    // (linear RGB colors, like the water colors) that fades out toward space
    // over atmosphere_height_meters. Consumers multiply the sun tint on top.
    // The defaults are chosen for the renderer's shared tone mapping: the
    // pipeline brightens midtones, so the pre-tone-map values sit deeper and
    // more saturated than the sky they finally display.
    bool  atmosphereEnabled       = true;
    double atmosphereHeightMeters = 20000.0; // sky fades to space over this altitude
    float atmosphereZenithColor[3]  = {0.060f, 0.170f, 0.580f};
    float atmosphereHorizonColor[3] = {0.470f, 0.620f, 0.860f};
    // Aerial perspective (see atmosphere.hpp): the haze weight along a sight
    // line saturates exponentially with distance. The scale is the
    // exponential's scale length in meters - larger values keep the
    // midground clear and let the far ridge dissolve gradually instead of
    // the whole distance range hazing over at once. The asymptote caps the
    // haze weight even at infinite distance, keeping the farthest terrain
    // faintly readable as terrain instead of fading fully into sky.
    double aerialPerspectiveScaleMeters   = 3600.0; // sight-line distance at which the haze has covered
                                                    // ~63% of its asymptote (the exponential's scale
                                                    // length, in meters - short enough that a ground
                                                    // view's midground starts dissolving and the far
                                                    // ridge melts into the horizon band)
    double aerialPerspectiveAsymptote     = 0.72; // haze weight the profile approaches at infinite distance
    // How strongly the DIRECT sun light (terrain/water/specular lighting, the
    // sun disc) adopts the atmosphere's transmittance tint (see
    // sunDirectTint in atmosphere.hpp): 1 gives the physical curve, 0 keeps
    // the consumer's configured sun color unchanged at every elevation.
    // Scattered light (the sky gradient, the haze) keeps its own tint either
    // way, so 0 restores the look of a scene lit by one fixed sun color.
    double sunDirectTintStrength = 1.0;

    // ---- water -----------------------------------------------------------------
    bool  waterEnabled        = true;
    float waterColorShallow[3] = {0.070f, 0.255f, 0.275f};
    float waterColorDeep[3]    = {0.012f, 0.060f, 0.130f};
    float waterRoughness      = 0.22f; // consumer hint (exposed via chunk material); low values give the
                                       // close-range sea a crisp sun-glint path (the per-vertex wave
                                       // normals break it into glitter)
    float waterRoughnessOrbital = 0.36f; // the hint's high-altitude blend target: orbital eyes see flat
                                         // far-water facets, so the lobe widens from the close-range
                                         // value toward this - kept well under 1 so the sun glint
                                         // survives as the broad patch orbital photographs show
    // Depth tint per channel: water absorbs red fastest, green next, blue
    // last, so the surface tint runs shallow turquoise -> green-teal mid shelf
    // -> navy deep instead of one gray-blue mix. The configured
    // water_depth_tint_meters is the BLUE channel's saturation depth; red and
    // green saturate at fixed fractions of it.
    double waterDepthTintMeters = 70.0; // depth where the water surface tint saturates (blue channel)
    // Shallow shelf: the surface blends toward the beach-sand tone
    // transmitted by the water's own hue over this depth band, so flats read
    // sandy turquoise (the bottom seen through the sheet) instead of one
    // pale sheet. 0 disables.
    double waterShoreMixMeters  = 3.0;
    // Brightness lift of that transmitted sand: sunlit flats under a thin
    // sheet read brighter than the raw beach palette row. 1.0 = no lift.
    double waterShoreSandBrightness = 1.35;
    // Surf: a white breaker band where the water shallows toward the shore,
    // plus a noise modulation so the breaker reads as patchy wave crests
    // instead of a painted stripe.
    double waterFoamWidthMeters = 2.5;  // depth band under the surface that reads as surf; wide
                                        // enough to read as a breaker line from a standing eye
    double waterFoamStrength    = 0.60; // 0 = no foam, 1 = solid white surf band
    // Falloff exponent of the breaker band across its depth range: 2 keeps a
    // broad wash of surf over the whole shallow shelf, 4 squeezes the
    // breakers into a tight rim at the waterline (real surf hugs the shore).
    double waterFoamFalloffExponent = 3.0;
    // Sea ice: the water surface freezes where the SEA-LEVEL temperature drops
    // below seaIceTemperature (the same climate field that drives the land's
    // freeze line, so ice caps and snowy poles line up with an organic, noisy
    // edge). 0 disables. The transition blends ice color in over
    // seaIceTransition of temperature around the threshold.
    double seaIceTemperature  = 0.10;
    double seaIceTransition   = 0.05;
    // Sea ice reads slightly blue against the warm-white cloud tops, so the
    // polar caps stay distinguishable from the weather layer at globe scale.
    float waterColorIce[3]    = {0.78f, 0.85f, 0.94f};
    // Wave slope: deterministic noise tilts the water surface normals around
    // the radial direction, giving the sea sun-glint texture at close range.
    // The amplitude is a radian-scale tilt; it fades out under the LOD band
    // limit like every other noise term, so coarse chunks keep a smooth sea.
    double     waterWaveAmplitude          = 0.16;
    double     waterWaveWavelengthMeters   = 8.0;
    // Amplitude of the half-wavelength chop riding the swell, as a fraction
    // of the swell tilt: the chop breaks the sun glint into sparkle. 0 gives
    // a single-scale swell.
    double     waterWaveChopFraction       = 0.33;

    // ---- clouds (coverage field; see atmosphere.hpp) ----------------------------
    // A pure noise field on a spherical shell above the datum: consumers paint
    // a cloud layer from it (the demo meshes the shell for globe/orbit views).
    // Coverage maps the field through a smooth threshold, so
    // cloud_coverage_amount is the approximate fraction of the sky under
    // cloud and the edges stay organic. The field is band-limited like every
    // other noise term, so coverage sampled at a coarse spacing stays free of
    // sub-spacing speckle.
    bool  cloudsEnabled       = true;
    double cloudCoverageAmount    = 0.42;   // approximate sky fraction under cloud (0 = clear)
    double cloudWavelengthMeters  = 13000.0; // feature scale of the cloud field (scale with the radius):
                                             // Earth-like weather systems span ~3-6 features around
                                             // the globe, so keep the wavelength at ~0.2 of the radius
    double cloudAltitudeMeters    = 1200.0; // height of the cloud shell above the datum
    double cloudDetailStrength    = 0.35;   // amplitude of the fine breakup octave relative to the base field
    // Amplitude of the inverted-cellular billow octave relative to the base
    // field: one-at-lobe-centers cellular distance packs the rounded
    // cumulus lobes into the bank bodies (the Perlin-Worley pairing
    // volumetric-cloud pipelines use - smooth fBm alone reads as soft
    // cotton). 0 keeps purely smooth banks.
    double cloudBillowStrength    = 0.32;
    // Softness of the cloud edge: the coverage field crosses into cloud over
    // a band of this half-width. Narrow values scatter crisp cloud clumps;
    // wide values grow soft decks with veiled edges whose thinning fringes
    // consumers can paint (the deck contour then dissolves into the sky
    // instead of cutting off at a polygon boundary).
    double cloudEdgeSoftness = 0.22;
    // Amplitude of the very-low-frequency weather field that shifts the
    // coverage threshold regionally (positive weather = cloudier). Stronger
    // values organize the banks into larger cloudy and clear systems.
    double cloudWeatherStrength = 0.45;
    // Wavelength of that weather field as a multiple of the cloud field's own
    // wavelength: larger values organize the banks into fewer, broader
    // weather systems.
    double cloudWeatherWavelengthMultiple = 4.0;
    // Amplitude of the latitudinal coverage bands (a wet equatorial belt,
    // dry subtropics, stormy mid-latitudes), keyed to the spin axis. 0 gives
    // a latitude-blind field.
    double cloudLatitudeBandStrength = 0.16;
    // Texture-consumer detail (see CloudField::coverageTexel): the wisp
    // octave pair's amplitude share (the torn fringes and ragged holes only
    // a texture's texel lattice resolves) and the cyclone vortex family's
    // strength multiplier (the spiral shear that wraps the banks near each
    // seed-derived storm into comma heads and arms).
    double cloudWispStrength      = 0.40;
    double cloudCycloneStrength   = 0.70;
    // Frequency of those bands over latitude (radians of phase per radian of
    // latitude): ~3.4 places a wet equatorial belt, dry subtropics and wet
    // mid-latitudes the way Earth's circulation cells do.
    double cloudBandFrequency = 3.4;
    // Warm white, so the cloud layer reads against the bluer sea ice at
    // globe scale (both are bright; the hue carries the distinction).
    float cloudColor[3]          = {0.985f, 0.975f, 0.955f}; // linear RGB of the cloud tops
    // Cloud shadows on the ground: the terrain and water albedo darkens
    // under the same coverage field the cloud layer renders from, sampled
    // along the SUN RAY through the surface point (the cloud bank that
    // actually blocks this point's sunlight sits up-sun at the shell
    // altitude), so the shadows fall AWAY from their banks the way real
    // cloud shadows do and stay consistent with the cloud layer from orbit
    // and from the ground. 0 disables.
    double cloudShadowStrength   = 0.45;
    // Penumbra softening of the cast shadows: the shadow field widens the
    // coverage's own edge band by this fraction, so shadows read as soft-
    // edged darkening under their banks (real cloud shadows blur with the
    // sun's angular size) instead of a hard copy of the cloud outlines.
    // 0 keeps the shadow exactly as sharp as the banks above it.
    double cloudShadowSoftness   = 0.50;
    // Direction (unit, world space) of the sunlight that casts the cloud
    // shadows - the one lighting the scene the consumer renders (the demo
    // pairs it with its [pbr] sun_direction). The default matches the
    // demo's shipped sun; a sun below the local horizon turns the shadows
    // off for that point (night side). The terrain self-shading below
    // reads the SAME direction, so the relief's own shade and the cloud
    // shade always agree about where the sun stands.
    double cloudShadowSunDirection[3] = {0.50, 0.45, 0.74};

    // Terrain self-shading: the mesher darkens each surface vertex by the
    // relief the sun ray out of it actually meets (a fixed-resolution march
    // of the density field up-sun), so escarpments, ridgelines and ravine
    // walls cast shade into the ground behind them the way real relief
    // photographs. The shade rides the vertex albedo the same way the cloud
    // shadow does - the renderer's own lighting stays the authority on
    // slope shading; this term only adds what slopes alone cannot express,
    // terrain occluding terrain. 0 disables.
    double terrainSelfShadowStrength = 0.55;
    // Sun-ray travel the per-vertex march resolves (a tall escarpment's
    // shadow reach): occluders farther up-sun than this stop mattering, and
    // the march's density cutoff rides at a fraction of it (relief finer
    // than that fraction is the renderer's slope shading's own scale, not
    // shadow casting). Kept LOD-independent - the same march from the same
    // point whatever chunk depth samples it - so neighboring chunks at
    // different depths agree on every shared boundary vertex.
    double terrainSelfShadowReachMeters = 160.0;

    // ---- high cirrus deck (streak field; see atmosphere.hpp) ---------------------
    // A thin ice-crystal deck above the cumulus shell: filamentary streaks
    // stretched along the zonal wind, sampled as a pure direction-space
    // field the same way clouds and stars are. The streaks run at a fixed
    // angular scale (the deck's own feature size in radians), so the field
    // reads consistently from orbit and from closer space views. The demo
    // meshes the deck as a translucent shell for its globe and orbit
    // views; consumers that ignore it lose the wisps and nothing else.
    bool   cirrusEnabled       = true;
    double cirrusStrength      = 1.0;   // brightness of the deck (0 reads as a clear sky)
    double cirrusAltitudeMeters = 3400.0; // deck height above the datum (above the cumulus shell)
    double cirrusCoverageAmount = 0.30;  // approximate sky fraction under streak (0 = clear)

    // ---- stars (space backdrop; see atmosphere.hpp) ------------------------------
    // A seed-derived star field for the space behind the planet: consumers
    // sample the pure direction-space function when they paint their space
    // backdrop (the demo meshes a star dome for its globe and orbit views
    // and fades it out with the atmosphere weight). Like every other field
    // of the module it takes the consumer's sample spacing and band-limits
    // stars below it, so coarse backdrops keep the bright stars instead of
    // aliasing the faint ones into speckle.
    bool   starsEnabled    = true;
    double starsDensity    = 0.5;  // star spacing: 0 sparse, 1 dense skies
    double starsBrightness = 1.0;  // luminance multiplier of the whole field
    // Disc size range the consumer's backdrop meshes draw per star (the
    // demo's star dome): the radius a faintest star's disc covers and the
    // radius the brightest one grows to (radius scales with sqrt
    // brightness between them). Point-like defaults keep the field crisp at
    // capture resolutions - larger maxima read as soft blobs once the
    // 2x2 supersampling resolve averages them.
    double starDiscRadiusMinRadians = 0.0011; // faintest star: a tight point
    double starDiscRadiusMaxRadians = 0.0024; // brightest star: a small disc
    // Galactic band (the Milky Way read): a seed-derived great circle the
    // star field concentrates along - stars appear there several times more
    // often, run brighter, and sit in a seed-derived dark rift of dust lanes
    // the way the real galaxy's unresolved-star glow organizes against its
    // dust. 0 keeps the uniform field (every cell an equal chance).
    double galaxyBandStrength = 1.0;     // 0 = latitude-blind star scatter
    double galaxyBandWidthRadians = 0.35; // angular half-width of the band (radians of sky)

    // ---- moon (backdrop companion; see atmosphere.hpp moonDirection) -----------
    // A second celestial body: the module owns the pure orbital model (a
    // direction on the sky), consumers mesh it. The demo shades it as a lit
    // sphere - the same sun that lights the terrain lights the moon, so the
    // phase, the terminator and the crescent read for free and stay consistent
    // with the scene from every view height. Angular size is configured
    // directly (the demo places the mesh at its backdrop distance and scales
    // to this radius), so the read does not depend on the orbit radius.
    bool   moonEnabled = true;
    double moonAngularRadiusRadians = 0.0080; // ~0.46 deg - the showcase's disc radius on the sky
    double moonOrbitalPeriodSeconds = 1200.0; // one revolution per this many seconds
    double moonOrbitInclinationRadians = 0.35; // tilt of the orbit plane off the equator
    double moonAlbedo = 1.0;                    // brightness multiplier of the regolith

    // ---- aurora (polar night glow; the demo meshes the ring) ------------------
    // An emissive curtain ring above the magnetic (here: spin) poles that
    // only glows on the night side - baked against the run's sun direction -
    // so a space view reads a green arc along the dark limb while day-side
    // and twilight ground views stay clean. 0 strength disables the glow.
    bool   auroraEnabled = true;
    double auroraStrength = 0.55; // 0 = no glow, 1 = the full curtain brightness

    // ---- surface scatter (ground-level life; consumers mesh it) ---------------
    // The ground-level real-world elements the terrain's color field cannot
    // express: vegetation, surface rocks, bird flocks, breaking-wave foam and
    // valley mist. The module owns the DENSITY RULES (which climate state
    // carries which element, how much of it, where it stops) and these knobs;
    // consumers scatter their own geometry against them (the demo does this
    // deterministically from the planet seed, camera cell and biome gates).
    // Densities multiply each element's climate-derived presence (0 disables
    // that element outright); radii bound the scatter ring around the eye.
    bool   vegetationEnabled    = true;
    double vegetationTreeDensity  = 1.0;  // forest/taiga/savanna trees
    double vegetationBushDensity  = 1.0;  // shrubs and bushes
    double vegetationGrassDensity = 1.0;  // grass tufts on grassy ground
    double vegetationTreeRadiusMeters  = 1400.0; // trees scatter out to here
    double vegetationGrassRadiusMeters = 18.0;   // grass lives close to the eye; the ring
                                               // fade thins the sward at its edge
    double surfaceRockDensity        = 1.0; // scattered boulders on open ground
    double surfaceRockRadiusMeters   = 260.0;
    double surfaceShadowStrength     = 1.0; // sun-aligned contact shadows under the scattered elements
    bool   whitecapsEnabled       = true;
    double whitecapStrength       = 0.65; // 0 = glassy calm, 1 = heavy sea speckle
    double whitecapWindSpeedMps   = 9.0;  // the 10 m wind speed the sea state follows
    bool   surfFoamEnabled        = true; // shore-parallel breaking-wave foam bands
    double surfFoamStrength       = 0.70;
    bool   birdsEnabled           = true;
    double birdFlockCount         = 2.0;  // flocks within the scatter radius
    double birdFlockSize          = 9.0;  // birds per flock
    bool   valleyMistEnabled      = true;
    double valleyMistStrength     = 0.50; // 0 = clear valleys, 1 = heavy ground fog
    double scatterRebuildDistanceMeters = 18.0; // camera movement that triggers a re-scatter

    // ---- debug ------------------------------------------------------------------
    // Chunk-border overlay: tints terrain and water vertices that sit within
    // debug_chunk_border_width_meters of their own chunk's boundary planes,
    // so every rendered chunk lights up as a grid cell on the surface. The
    // overlay rides the vertex colors only - geometry, normals, and the
    // determinism guarantees are untouched, and a chunk built with the
    // overlay on differs from one built without it only in its colors.
    bool   debugChunkBorders          = false;
    double debugChunkBorderWidthMeters = 2.0; // half-width of the tinted band along each boundary plane

    // ---- derived values (filled by validate(), read-only afterwards) -----------
    double derivedRootHalfMeters   = 0.0; // root cube half-size
    double derivedRootSizeMeters   = 0.0; // root cube edge length (2 * half)
    std::uint32_t derivedMaxDepth  = 0;   // deepest octree level actually used
    double derivedVoxelMeters      = 0.0; // leaf voxel size (leaf cube / leafGridN)
    double derivedMinShellRadius   = 0.0; // inner radius of the terrain band
    double derivedMaxShellRadius   = 0.0; // outer radius of the terrain band
    double derivedMaxReliefMeters  = 0.0; // total possible relief above/below the datum
    double derivedSeaRadiusMeters  = 0.0; // radius of the ocean surface
    double derivedMountainMassWavelengthMeters = 0.0; // wavelength of the broad mountain mass field
    double derivedProvinceWavelengthMeters = 0.0;     // wavelength of the geologic-province field
    double derivedMacroClimateStrideMeters = 0.0;     // sampling stride of the mesher's macro-climate grid

    // Clamp every field into its documented range and recompute the derived
    // values. Returns false (with an error string) when a value cannot be
    // salvaged (NaN, non-positive radius, ...).
    bool validate(std::string &error);
};

// Interleaved planet vertex, matching the renderer's canonical 11-float
// layout (position, normal, color, uv - 44-byte stride) so consumers can
// upload chunk meshes as MeshDescription views without repacking.
struct PlanetVertex
{
    float position[3];
    float normal[3];
    float color[3];
    float uv[2];
};

static_assert(sizeof(PlanetVertex) == 44, "planet vertices must match the renderer's canonical layout");

} // namespace planet
} // namespace engine

