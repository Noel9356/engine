// PlanetGenerationParams: every knob of the planet generator.
//
// No generation constant is hardcoded at call sites: defaults live here
// (mirrored in the module's config.toml), the module reads [planet]
// configuration keys into this struct at construct(), and per-planet
// overrides arrive through the capability's PlanetDescription. validate()
// clamps every field into a safe range so hostile config cannot produce
// degenerate geometry, and derives the aggregate bounds (maximum relief,
// octree depth, shell band) the octree culling relies on.

#ifndef ENGINE_PLANET_PARAMS_HPP
#define ENGINE_PLANET_PARAMS_HPP

#include <engine/planet/types.hpp>

#include <algorithm>
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
    std::uint32_t chunkCacheCapacity  = 8192; // hard cap on cached chunk meshes (oldest evicted)

    // ---- octree / level of detail ---------------------------------------------
    std::uint32_t leafGridN          = 16;   // marching-cubes cells per leaf chunk axis
    double        targetVoxelMeters  = 2.5;  // desired finest voxel size (drives max depth)
    std::uint32_t maxDepthLimit      = 18;   // hard cap for the octree depth
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

    double terraceStrength  = 0.60;  // 0 = smooth terrain, 1 = full plateaus
    double terraceSteps     = 9.0;   // approximate plateau count across the field
    double cliffWavelengthMeters = 1400.0; // wavelength of the terrace mask field

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
    double waterDepthTintMeters = 70.0; // depth over which the water surface tint saturates

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
    double colorVariation = 0.14;           // per-point albedo jitter (0 = flat palette)
    double snowLineMeters = 1600.0;          // sea-level snow line (drops as climate cools);
                                            // only the highest massifs are glaciated, polar
                                            // caps come from latitude alone
    double beachWidthMeters = 2.0;          // shoreline band width above sea level (narrow: sand hugs the waterline)
    double rockSlopeThreshold = 0.62;       // slope (1 - up-dot-normal) where rock takes over;
                                            // high enough that aliased ridge crests stay snowed
                                            // and terraced risers stay grassed
    double rockStrataContrast = 0.10;       // luminance banding on rock faces, keyed to the
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

    // ---- water -----------------------------------------------------------------
    bool  waterEnabled        = true;
    float waterColorShallow[3] = {0.060f, 0.205f, 0.275f};
    float waterColorDeep[3]    = {0.012f, 0.060f, 0.130f};
    float waterRoughness      = 0.42f; // consumer hint (exposed via chunk material); low values make the
                                       // coarse far-water facets band with sharp mirror glints

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

#endif // ENGINE_PLANET_PARAMS_HPP
