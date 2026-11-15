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
    double        boundingFactor     = 1.4;  // root cube half-size = radius * this
    std::uint32_t maxConcurrentBuilds = 16;  // in-flight async chunk mesh builds
    std::uint32_t maxNewBuildsPerUpdate = 48;  // build requests issued per update() call
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
    double rangeClusterStrength         = 0.55;  // [0,1] mountain clustering into ranges
    double rangeClusterWavelengthMeters = 20000.0;

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

    // ---- coastal shaping --------------------------------------------------------
    // Real shorelines are planed off by waves and sediment: the medium-frequency
    // detail (hills, terraces, overhangs, strata) fades toward the datum so
    // coastlines slope into the sea instead of zig-zagging across the waterline
    // (which reads as a web of sub-sea ponds with beach rims).
    double coastalFlattenMeters = 80.0;   // half-width of the flattening band around sea level
    double coastalFlattenFloor  = 0.10;   // residual detail fraction right at the shoreline

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
    bool   moistureLatitudeBands = true;    // ITCZ wet belt + subtropical dry belts
    double colorVariation = 0.24;           // per-point albedo jitter (0 = flat palette)
    double colorVariationWavelengthMeters = 260.0; // feature size of the albedo jitter field
    double colorVariationGroundWavelengthMeters = 3.5; // metre-scale mottle for ground-level views
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
    // Wet-sand band above the waterline: real beaches darken sharply in the
    // swash zone. Two keys keep the band a configuration decision - width and
    // how dark the wet sand reads.
    double beachWetWidthMeters = 6.0;       // elevation band above sea level that reads wet
    double beachWetDarkening   = 0.45;     // 0 = dry sand everywhere, 1 = near-black wet band
    double rockSlopeThreshold = 0.46;       // slope (1 - up-dot-normal) where rock takes over;
                                            // steep faces scrape soil and grass into bare
                                            // rock while rolling hills stay green
    double rockStrataContrast = 0.30;       // luminance banding on rock faces, keyed to the
                                            // strata band spacing (sediment layer stripes);
                                            // strong values read as dirt smears at grazing light

    // Biome colors are BLENDED, not hard-switched: per-vertex hard switches smear
    // into pale stains across triangles (the "blotch" artefact). These widths
    // control how softly each transition interpolates between palette rows.
    double biomeBlendWidth       = 0.06;  // half-width on the temperature/moisture axes
    double slopeBlendWidth       = 0.08;  // half-width around the rock slope threshold
    double elevationBlendFraction = 0.12; // snow/water band blend as a fraction of band width

    // Biome palette, linear RGB rows in Biome enum order (biomeCount rows of 3).
    float biomePalette[biomeCount * 3] = {
        // deep ocean      ocean            shallow ocean
        0.006f, 0.024f, 0.055f,   0.013f, 0.048f, 0.100f,   0.034f, 0.115f, 0.165f,
        // beach           desert           savanna
        0.540f, 0.490f, 0.350f,   0.560f, 0.500f, 0.330f,   0.520f, 0.470f, 0.245f,
        // grassland       shrubland        temperate forest
        0.360f, 0.500f, 0.220f,   0.310f, 0.355f, 0.165f,   0.180f, 0.330f, 0.130f,
        // seasonal forest tropical forest   temperate rainforest
        0.130f, 0.360f, 0.120f,   0.100f, 0.310f, 0.090f,   0.090f, 0.280f, 0.110f,
        // taiga           tundra           snow
        0.150f, 0.260f, 0.180f,   0.480f, 0.500f, 0.450f,   0.870f, 0.900f, 0.930f,
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
    float atmosphereZenithColor[3]  = {0.040f, 0.130f, 0.460f};
    float atmosphereHorizonColor[3] = {0.220f, 0.400f, 0.780f};

    // ---- water -----------------------------------------------------------------
    bool  waterEnabled        = true;
    float waterColorShallow[3] = {0.085f, 0.320f, 0.330f};
    float waterColorDeep[3]    = {0.012f, 0.060f, 0.130f};
    float waterRoughness      = 0.42f; // consumer hint (exposed via chunk material); low values make the
                                       // coarse far-water facets band with sharp mirror glints
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
    // Surf: a white breaker band where the water shallows toward the shore,
    // plus a noise modulation so the breaker reads as patchy wave crests
    // instead of a painted stripe.
    double waterFoamWidthMeters = 2.5;  // depth band under the surface that reads as surf; wide
                                        // enough to read as a breaker line from a standing eye
    double waterFoamStrength    = 0.55; // 0 = no foam, 1 = solid white surf band
    // Sea ice: the water surface freezes where the SEA-LEVEL temperature drops
    // below seaIceTemperature (the same climate field that drives the land's
    // freeze line, so ice caps and snowy poles line up with an organic, noisy
    // edge). 0 disables. The transition blends ice color in over
    // seaIceTransition of temperature around the threshold.
    double seaIceTemperature  = 0.10;
    double seaIceTransition   = 0.05;
    float waterColorIce[3]    = {0.80f, 0.86f, 0.90f};
    // Wave slope: deterministic noise tilts the water surface normals around
    // the radial direction, giving the sea sun-glint texture at close range.
    // The amplitude is a radian-scale tilt; it fades out under the LOD band
    // limit like every other noise term, so coarse chunks keep a smooth sea.
    double     waterWaveAmplitude          = 0.12;
    double     waterWaveWavelengthMeters   = 8.0;

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
    double cloudWavelengthMeters  = 9000.0; // feature scale of the cloud field (scale with the radius)
    double cloudAltitudeMeters    = 1200.0; // height of the cloud shell above the datum
    double cloudDetailStrength    = 0.45;   // amplitude of the fine breakup octave relative to the base field
    // Softness of the cloud edge: the coverage field crosses into cloud over
    // a band of this half-width. Narrow values scatter crisp cloud clumps;
    // wide values grow soft decks with veiled edges.
    double cloudEdgeSoftness = 0.10;
    // Amplitude of the very-low-frequency weather field that shifts the
    // coverage threshold regionally (positive weather = cloudier). Stronger
    // values organize the banks into larger cloudy and clear systems.
    double cloudWeatherStrength = 0.45;
    // Amplitude of the latitudinal coverage bands (a wet equatorial belt,
    // dry subtropics, stormy mid-latitudes), keyed to the spin axis. 0 gives
    // a latitude-blind field.
    double cloudLatitudeBandStrength = 0.16;
    float cloudColor[3]          = {0.96f, 0.97f, 1.0f}; // linear RGB of the cloud tops

    // ---- derived values (filled by validate(), read-only afterwards) -----------
    double derivedRootHalfMeters   = 0.0; // root cube half-size
    double derivedRootSizeMeters   = 0.0; // root cube edge length (2 * half)
    std::uint32_t derivedMaxDepth  = 0;   // deepest octree level actually used
    double derivedLeafSizeMeters   = 0.0; // leaf node cube size at derivedMaxDepth
    double derivedVoxelMeters      = 0.0; // leaf voxel size
    double derivedMinShellRadius   = 0.0; // inner radius of the terrain band
    double derivedMaxShellRadius   = 0.0; // outer radius of the terrain band
    double derivedMaxReliefMeters  = 0.0; // total possible relief above/below the datum
    double derivedSeaRadiusMeters  = 0.0; // radius of the ocean surface
    double derivedMountainMassWavelengthMeters = 0.0; // wavelength of the broad mountain mass field

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

