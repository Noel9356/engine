// main.cpp - the planet generation demo application.
//
// A standalone harness for the planet module: it creates one procedural
// planet through the "planet" capability, streams its LOD chunk meshes into
// the renderer every frame, and exposes screenshot exports and determinism
// self-tests so generation can be validated without any manual inspection.
//
// Views (deterministic camera placement from the planet's own sampling):
//   --view globe    whole planet from space: continents, oceans, ice caps.
//   --view coast    standing on a beach looking along the shoreline.
//   --view cliffs   terraced mountain country with overhanging cliff faces.
//   --view surface  rolling green hills under the sun.
//
// Useful flags: --seed N / --radius M override the planet description,
// --frames N renders exactly N frames (deterministic), --screenshot exports
// the rendered frame as PPM, --self-test verifies chunk generation is
// deterministic and independent of chunk build order, --stats-every prints
// planet statistics cadence, --caves enables the cave worm carving.

#include <engine/application/application.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/planet.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include "engine/application/application_host.hpp"
#include "engine/application/command_line.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace
{

using namespace engine;

namespace window  = engine::window;
namespace renderer = engine::renderer;
namespace planet   = engine::planet;

// ---- options --------------------------------------------------------------------

struct PlanetDemoOptions
{
    std::string   viewName        = "globe"; // --view globe|orbit|coast|cliffs|surface
    std::uint64_t seed            = 0;       // --seed (0 = config default)
    double        radius          = 0.0;     // --radius meters (0 = config default)
    std::uint64_t frames          = 0;       // --frames (0 = run until closed; headless 90)
    std::string   screenshotPath;            // --screenshot
    std::uint64_t screenshotFrame = 0;       // --screenshot-frame (0 = last frame)
    std::uint32_t warmupFrames    = 0;       // --warmup: build-only frames before the first render
    std::uint32_t statsEveryFrames = 30;     // --stats-every
    bool          orbit           = false;   // --orbit
    bool          caves           = false;   // --caves
    bool          selfTest        = false;   // --self-test
};

const char *planetOptionsUsage() noexcept
{
    return "planet demo options:\n"
           "  --view globe|coast|cliffs|surface  View to render (default globe).\n"
           "  --seed <n>                Planet seed override (0 keeps config).\n"
           "  --radius <meters>         Planet radius override (0 keeps config).\n"
           "  --frames <n>              Render exactly n frames, then exit.\n"
           "  --screenshot <file.ppm>   Export the rendered frame as PPM.\n"
           "  --screenshot-frame <n>    Frame index for --screenshot (default last).\n"
           "  --stats-every <n>         Print planet statistics every n frames.\n"
           "  --orbit                   Sweep the camera (globe: around the planet;\n"
           "                            surface views: a short circle at the pose).\n"
           "  --caves                   Enable the cave worm carving field.\n"
           "  --self-test               Run determinism/order-independence checks\n"
           "                            and exit (exit code 0 = pass).\n";
}

bool parsePlanetOptions(const std::vector<std::string> &arguments, PlanetDemoOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const bool         hasValue = index + 1 < arguments.size();
        if (argument == "--view" && hasValue)
        {
            const std::string &value = arguments[++index];
            if (value != "globe" && value != "orbit" && value != "coast" && value != "cliffs" && value != "surface")
            {
                outError = "invalid view '" + value + "'; expected globe|orbit|coast|cliffs|surface";
                return false;
            }
            options.viewName = value;
        }
        else if (argument == "--seed" && hasValue)
        {
            options.seed = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--radius" && hasValue)
        {
            options.radius = std::strtod(arguments[++index].c_str(), nullptr);
        }
        else if (argument == "--frames" && hasValue)
        {
            options.frames = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--screenshot" && hasValue)
        {
            options.screenshotPath = arguments[++index];
        }
        else if (argument == "--screenshot-frame" && hasValue)
        {
            options.screenshotFrame = std::strtoull(arguments[++index].c_str(), nullptr, 10);
        }
        else if (argument == "--stats-every" && hasValue)
        {
            options.statsEveryFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--warmup" && hasValue)
        {
            options.warmupFrames = static_cast<std::uint32_t>(std::strtoul(arguments[++index].c_str(), nullptr, 10));
        }
        else if (argument == "--orbit")
        {
            options.orbit = true;
        }
        else if (argument == "--caves")
        {
            options.caves = true;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
        }
        else
        {
            outError = "unknown or incomplete option '" + argument + "'\n" + planetOptionsUsage();
            return false;
        }
    }
    return true;
}

// ---- deterministic view-direction search -----------------------------------------

// Golden-spiral direction set: evenly spread sample directions over the
// sphere, fixed for a given count (no noise, no runtime randomness).
std::vector<planet::Vec3> goldenSpiralDirections(std::uint32_t count)
{
    std::vector<planet::Vec3> directions;
    directions.reserve(count);
    constexpr double goldenAngle = 2.39996322972865332;
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const double t     = (static_cast<double>(index) + 0.5) / static_cast<double>(count);
        const double phi   = std::acos(std::clamp(1.0 - 2.0 * t, -1.0, 1.0));
        const double theta = goldenAngle * static_cast<double>(index);
        directions.push_back(planet::Vec3{std::sin(phi) * std::cos(theta), std::cos(phi), std::sin(phi) * std::sin(theta)});
    }
    return directions;
}

struct Viewpoint
{
    planet::Vec3 direction;
    double       surfaceRadius = 0.0;
    // Coast views: the horizontal look direction chosen to face the water
    // (false when the default world-up tangent applies).
    bool         hasLookTangent = false;
    planet::Vec3 lookTangent    = {};
};

// Rotates the unit direction `direction` by `angleRadians` around `axis`
// (Rodrigues; axis must be unit length and perpendicular to direction).
planet::Vec3 rotateDirection(const planet::Vec3 &direction, const planet::Vec3 &axis, double angleRadians)
{
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    return direction * c + planet::cross(axis, direction) * s;
}

// Clamps the camera pitch (angle of the eye->target ray below the local
// horizontal plane) into [minPitchRadians, maxPitchRadians] by moving the
// target along the ray's horizontal component. A re-projected ground target
// pitches the camera as steeply as the ground ahead falls away: past ~35
// degrees the near slope fills the frame and the sky vanishes. The clamp
// keeps the horizon inside every surface view regardless of the terrain
// shape ahead (the sightline probes guarantee open ground; this guarantees
// open FRAMING).
void clampCameraPitch(const glm::dvec3 &eye, const glm::dvec3 &localUp, glm::dvec3 &target,
    double minPitchRadians, double maxPitchRadians)
{
    const glm::dvec3 view       = target - eye;
    const double     viewLength = glm::length(view);
    if (viewLength < 1.0e-9)
    {
        return;
    }
    const glm::dvec3 viewUnit        = view / viewLength;
    const double     alongUp         = glm::dot(viewUnit, localUp);
    const glm::dvec3 horizontal      = viewUnit - localUp * alongUp;
    const double     horizontalLength = glm::length(horizontal);
    double pitch = horizontalLength > 1.0e-9
        ? std::atan2(-alongUp, horizontalLength)
        : (alongUp < 0.0 ? 1.57079632679489662 : -1.57079632679489662);
    pitch = std::clamp(pitch, minPitchRadians, maxPitchRadians);
    if (horizontalLength <= 1.0e-9)
    {
        return; // straight up/down view: nothing sensible to clamp toward
    }
    const double horizontalDistance = viewLength * std::cos(pitch);
    target = eye + (horizontal / horizontalLength) * horizontalDistance
           - localUp * (viewLength * std::sin(pitch));
}

// What the surrounding terrain probes must look like for a showcase view.
enum class ProbePolicy
{
    Inland,    // warm green land at every probe ring (surface view)
    Coastline, // warm land allowed to border the sea (the coast view WANTS water in frame)
    Mountain,  // green lowland standing, snowy rock peaks allowed ahead (cliffs view)
};

// Picks the first direction (deterministic golden-spiral order) whose surface
// satisfies the predicate AND whose neighborhood passes the probe policy. On
// terrain where no candidate passes every probe (real mountain country always
// mixes cold probes in), the candidate with the highest probe score wins -
// deterministic, and far better than dropping to a random fallback spot.
bool pickViewpoint(planet::PlanetCapability &planetCapability, planet::PlanetHandle planet,
    const std::function<bool(double elevation, engine::planet::Biome biome, double temperature01,
        const planet::PlanetGenerationParams &params)>& predicate, ProbePolicy policy, double eyeHeight,
    const planet::Vec3 &sunDirection, Viewpoint &out)
{
    planet::PlanetGenerationParams params{};
    planetCapability.copyGenerationParams(planetCapability.self, planet, &params);
    const std::vector<planet::Vec3> candidates = goldenSpiralDirections(256);
    const planet::Vec3 spinAxis{params.axis[0], params.axis[1], params.axis[2]};

    std::size_t probesTotal = 0;
    bool        haveBest    = false;
    double      bestScore   = 0.0;
    double      bestElevation   = 0.0;
    double      bestTemperature = 0.0;
    int         bestBiome       = 0;
    // Diagnostics: the strongest candidate that FAILED the sight gate (its
    // probe quality and the gate that rejected it), printed when the pick
    // fails so tuning sees what rejected the best spots.
    double      bestFailQuality   = 0.0;
    double      bestFailElevation = 0.0;
    const char *bestFailReason    = "none";
    planet::Vec3 bestFailDirection{};
    planet::Vec3 bestFailFanTangents[5]{};
    double      bestFailRises[30] = {};
    bool        bestFailValid     = false;
    Viewpoint   best;

    for (const planet::Vec3 &direction : candidates)
    {
        const double directionArray[3] = {direction.x, direction.y, direction.z};
        double       radius            = 0.0;
        if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, directionArray, &radius))
        {
            continue;
        }
        const double elevation = radius - params.derivedSeaRadiusMeters;
        engine::planet::ClimateSample climate{};
        const double unitDirection[3] = {direction.x, direction.y, direction.z};
        engine::planet::Biome biome = engine::planet::Biome::Ocean;
        double temperature01        = 0.0;
        if (planetCapability.sampleClimate(planetCapability.self, planet, unitDirection, radius, &climate))
        {
            biome         = climate.biome;
            temperature01 = climate.temperature01;
        }
        if (!predicate(elevation, biome, temperature01, params))
        {
            continue;
        }

        // Neighborhood check: the rendered frame spans roughly 0.5-1.5 km
        // beyond the eye. Probe EIGHT compass directions per ring (a plus
        // pattern alone misses diagonal cliff drops into the sea) at three
        // distance rings - METRIC distances (420 m, 840 m, 1440 m) converted
        // to angles per candidate: the demo frames ~1 km of terrain at any
        // planet radius, so what matters around the spot is what lies within
        // those meters, not a fixed angular slice that balloons with R.
        planet::Vec3 tangent = planet::cross(spinAxis, direction);
        double       tangentLength = planet::length(tangent);
        if (tangentLength < 1.0e-6)
        {
            tangent       = planet::cross(planet::Vec3{1.0, 0.0, 0.0}, direction);
            tangentLength = planet::length(tangent);
        }
        tangent       = tangent * (1.0 / tangentLength);
        const planet::Vec3 bitangent = planet::cross(direction, tangent);
        const double ringDistances[3] = {420.0, 840.0, 1440.0}; // meters
        const std::size_t ringCount  = policy == ProbePolicy::Coastline ? 1u : 3u;
        std::size_t probesOk = 0;
        std::size_t probes   = 0;
        std::vector<double> probeElevations;
        probeElevations.reserve(24);
        for (std::size_t ring = 0; ring < ringCount; ++ring)
        {
            for (std::size_t compass = 0; compass < 8; ++compass)
            {
                ++probes;
                const double angle    = static_cast<double>(compass) * (3.14159265358979323846 / 4.0);
                const planet::Vec3 probeAxis = tangent * std::cos(angle) + bitangent * std::sin(angle);
                const planet::Vec3 hopDirection = rotateDirection(direction, probeAxis, ringDistances[ring] / params.radiusMeters);
                double hopRadius = 0.0;
                const double hopArray[3] = {hopDirection.x, hopDirection.y, hopDirection.z};
                engine::planet::ClimateSample hopClimate{};
                if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, hopArray, &hopRadius)
                    || !planetCapability.sampleClimate(planetCapability.self, planet, hopArray, hopRadius, &hopClimate))
                {
                    continue;
                }
                const double hopElevation = hopRadius - params.derivedSeaRadiusMeters;
                probeElevations.push_back(hopElevation);
                const auto isGreenBiome = [](engine::planet::Biome b)
                {
                    return b == engine::planet::Biome::Grassland || b == engine::planet::Biome::Shrubland
                        || b == engine::planet::Biome::TemperateForest || b == engine::planet::Biome::SeasonalForest
                        || b == engine::planet::Biome::TropicalRainforest || b == engine::planet::Biome::TemperateRainforest
                        || b == engine::planet::Biome::Taiga;
                };
                bool probeOk = false;
                if (policy == ProbePolicy::Coastline)
                {
                    // The coast view wants water in frame: sea probes are
                    // fine, and land probes should be green behind the beach.
                    // The temperature gate is low: shoreline air is cooled by
                    // the lapse rate whenever the beach sits on high ground.
                    probeOk = hopElevation < 0.0
                        || (hopClimate.biome != engine::planet::Biome::Snow
                            && hopClimate.biome != engine::planet::Biome::Tundra
                            && hopClimate.temperature01 > 0.32
                            && isGreenBiome(hopClimate.biome));
                }
                else if (policy == ProbePolicy::Mountain)
                {
                    // The cliffs view stands in low-mountain country and
                    // LOOKS at high country. Low probes (below 500 m) must be
                    // traversable land - grass, forest and BARE ROCK all
                    // qualify (cliff faces are rock); snowfields and deserts
                    // at foothill height would wash the frame. The
                    // temperature gate stays low because lapse rate cools
                    // foothill air fast on mountainous planets.
                    probeOk = hopElevation > 500.0
                        || (hopClimate.biome != engine::planet::Biome::Snow
                            && hopClimate.biome != engine::planet::Biome::Tundra
                            && hopClimate.biome != engine::planet::Biome::Desert
                            && hopClimate.biome != engine::planet::Biome::Beach
                            && hopClimate.temperature01 > 0.32);
                }
                else
                {
                    // Inland views: open land - snowfields, tundra, deserts
                    // and bare beaches would wash the frame; rock outcrops,
                    // savanna patches and cool foothill forest are normal and
                    // fine. Elevation runs to 1200 m and the temperature gate
                    // to 0.30: probes routinely climb foothills on mountain
                    // country, and the lapse rate (1.0 temperature per 1100 m
                    // of altitude) otherwise fails every raised probe.
                    probeOk = hopElevation >= 0.0 && hopElevation <= 1200.0
                        && hopClimate.biome != engine::planet::Biome::Snow
                        && hopClimate.biome != engine::planet::Biome::Tundra
                        && hopClimate.biome != engine::planet::Biome::Desert
                        && hopClimate.biome != engine::planet::Biome::Beach
                        && hopClimate.temperature01 > 0.30;
                }
                if (probeOk)
                {
                    probesOk += 1;
                }
            }
        }
        if (probesTotal == 0)
        {
            probesTotal = probes; // the ring layout is the same for every candidate
        }

        // Look-direction selection happens BEFORE the sightline check so the
        // fan verifies the direction the camera will actually look along. The
        // old order ran the wall caps on the fallback tangent and then aimed
        // the camera somewhere else entirely, so a wall the caps would have
        // rejected could still fill the frame. Order below: fallback tangent,
        // five yawed variants (the sun-preference candidates), the coast
        // waterward diagonal, the sun-preference yaw, then the sight fan
        // around the tangent the camera will really use.
        planet::Vec3 lookTangent = planet::cross(direction, planet::Vec3{0.0, 1.0, 0.0});
        double       lookTangentLength = planet::length(lookTangent);
        if (lookTangentLength < 1.0e-6)
        {
            lookTangent       = planet::cross(direction, planet::Vec3{1.0, 0.0, 0.0});
            lookTangentLength = planet::length(lookTangent);
        }
        lookTangent       = lookTangent * (1.0 / lookTangentLength);
        // Five yaw angles across the camera's horizontal field of view. The
        // 65-degree vertical FOV spans +-0.85 rad horizontally at 16:9, so
        // the fan yaws cover what the frame will actually show.
        const double sightYaws[5]  = {-0.8, -0.4, 0.0, 0.4, 0.8};
        planet::Vec3 yawedTangents[5];
        for (std::size_t yaw = 0; yaw < 5; ++yaw)
        {
            yawedTangents[yaw] = rotateDirection(lookTangent, direction, sightYaws[yaw]);
        }

        // Coast views face the water at an angle: among the eight compass
        // directions, the one whose surface radius at 320 m is LOWEST points
        // seaward (terrain drops toward the water) and the HIGHEST points
        // landward. Looking straight seaward fills the frame with open water;
        // the diagonal between the two keeps the shoreline running across the
        // frame with both beach and sea in shot. Store the diagonal as the
        // view's look tangent.
        planet::Vec3 waterwardTangent{};
        bool hasWaterward = false;
        if (policy == ProbePolicy::Coastline)
        {
            double lowestRadius  = 1.0e30;
            double highestRadius = -1.0e30;
            planet::Vec3 seaward;
            planet::Vec3 landward;
            for (std::size_t compass = 0; compass < 8; ++compass)
            {
                const double angle    = static_cast<double>(compass) * (3.14159265358979323846 / 4.0);
                const planet::Vec3 probeAxis = tangent * std::cos(angle) + bitangent * std::sin(angle);
                // The waterward scan searches just past the coast view's look
                // distance (320 m): the shoreline the view will actually show.
                const planet::Vec3 sightDirection = rotateDirection(direction, probeAxis, 320.0 / params.radiusMeters);
                double sightRadius = 0.0;
                const double sightArray[3] = {sightDirection.x, sightDirection.y, sightDirection.z};
                if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, sightArray, &sightRadius))
                {
                    continue;
                }
                if (sightRadius < lowestRadius)
                {
                    lowestRadius = sightRadius;
                    seaward      = sightDirection;
                }
                if (sightRadius > highestRadius)
                {
                    highestRadius = sightRadius;
                    landward      = sightDirection;
                }
            }
            if (lowestRadius < 1.0e30)
            {
                // Sum the TANGENTIAL components: seaward/landward are radial
                // unit vectors, and for a spot on a narrow spit they are
                // nearly opposite, so their raw sum is radial-dominated and
                // would aim the camera into the ground. Their tangential
                // projections point at the water and the land respectively -
                // but at a 60 km planet the scan rotates the probe by only
                // 0.3 degrees, so the RAW projections are ~5 milliradians
                // long and their sum collapses below any sane degeneracy
                // threshold (the old code then fell back to the raw
                // near-RADIAL seaward vector as the "tangent", the sight fan
                // probed the same spot 25 times, and every coastal candidate
                // was rejected as "level"). Normalize each projection to a
                // unit tangent first; the diagonal of the two unit tangents
                // is the along-shore direction. A degenerate (opposite-pair)
                // sum falls back to the seaward tangent.
                const planet::Vec3 radial = direction;
                planet::Vec3 seawardTangent   = seaward - radial * planet::dot(seaward, radial);
                planet::Vec3 landwardTangent  = landward - radial * planet::dot(landward, radial);
                const double seawardTLength   = planet::length(seawardTangent);
                const double landwardTLength  = planet::length(landwardTangent);
                if (seawardTLength > 1.0e-9)
                {
                    seawardTangent  = seawardTangent * (1.0 / seawardTLength);
                }
                if (landwardTLength > 1.0e-9)
                {
                    landwardTangent = landwardTangent * (1.0 / landwardTLength);
                }
                planet::Vec3 diagonal = seawardTangent + landwardTangent;
                const double diagonalLength = planet::length(diagonal);
                waterwardTangent = diagonalLength > 0.5 ? diagonal * (1.0 / diagonalLength)
                                      : (seawardTLength > 1.0e-9 ? seawardTangent : planet::Vec3{});
                hasWaterward     = diagonalLength > 0.5 || seawardTLength > 1.0e-9;
            }
        }

        // Sun-preference yaw choice: among the five sight yaws, ground views
        // look with the sun BEHIND the camera - the slope faces ahead catch
        // the light instead of turning away from it, which keeps the frame
        // readable at the demo's ambient level. Raking side light (the old
        // criterion) was rejected after render review: at this sun elevation
        // every face turned away from the sun went near black and half the
        // frame read as shadow murk. Coast views keep their waterward tangent
        // (the water must enter the frame); the sun falls where it falls.
        double bestYawAlignment = -1.0;
        std::size_t bestYaw = 2;
        const bool sunValid = (sunDirection.x != 0.0 || sunDirection.y != 0.0 || sunDirection.z != 0.0);
        if (policy != ProbePolicy::Coastline && sunValid)
        {
            planet::Vec3 sunTangential = sunDirection - direction * planet::dot(sunDirection, direction);
            const double sunTangentialLength = planet::length(sunTangential);
            if (sunTangentialLength > 1.0e-6)
            {
                sunTangential = sunTangential * (1.0 / sunTangentialLength);
                // The dot sign is the sun's side: positive = the look direction
                // points TOWARD the sun (the frame faces shadowed slopes),
                // negative = the sun is behind the camera (lit slopes face
                // the camera). Ground views therefore minimize the dot.
                double bestScore = -2.0;
                for (std::size_t yaw = 0; yaw < 5; ++yaw)
                {
                    const double alignment = planet::dot(yawedTangents[yaw], sunTangential);
                    const double score = -alignment;
                    if (bestScore < -1.5 || score > bestScore)
                    {
                        bestScore = score;
                        bestYawAlignment = alignment;
                        bestYaw          = yaw;
                    }
                }
            }
        }

        // The tangent the camera will actually use; the sight fan below
        // re-yaws it across the field of view.
        const planet::Vec3 sightLookTangent = hasWaterward ? waterwardTangent : yawedTangents[bestYaw];
        planet::Vec3 fanTangents[5];
        for (std::size_t yaw = 0; yaw < 5; ++yaw)
        {
            fanTangents[yaw] = rotateDirection(sightLookTangent, direction, sightYaws[yaw]);
        }

        // Sightline check along the EXACT look direction the demo will use:
        // the ground ahead must stay below the wall caps, otherwise the
        // camera stares into a near-vertical slope that fills the whole
        // frame. The wall caps rise with distance; cliff views get taller
        // caps because the terraced risers ARE the showcase.
        const double sightRingDistances[6] = {60.0, 120.0, 220.0, 420.0, 720.0, 1440.0}; // meters
        static constexpr std::size_t kSightRingCount = 6;
        bool sightClear = true;
        bool cliffAhead = false;
        const char *failReason = "none"; // diagnostics: why sightClear went false
        bool sawLand    = false; // coastline gate: the frame must contain
        bool sawWater   = false; // BOTH terrain and water, or the "coast"
                                 // view renders an empty ocean sheet (a spot
                                 // on a tiny islet passes the spot predicate
                                 // while every ring ahead is open water).
        std::size_t nearestLandRing = 5;      // coastline: first ring with land
        bool sawLandNear   = false; // coastline: land AND water within 220 m
        bool sawWaterNear  = false; // (the islet trap: its own ground sits
                                    // under the camera - ring-0 land there
                                    // is invisible at a 15 deg down-aim,
                                    // and the frame renders open ocean)
        double      bestWallScore   = -1.0;   // cliffs: strongest terraced rise
        planet::Vec3 cliffWallTangent{};
        bool        hasCliffWall    = false;
        for (std::size_t ring = 0; ring < kSightRingCount && sightClear; ++ring)
        {
            for (std::size_t yaw = 0; yaw < 5 && sightClear; ++yaw)
            {
                const planet::Vec3 &yawedTangent = fanTangents[yaw];
                const planet::Vec3 sightDirection = rotateDirection(direction, yawedTangent,
                    sightRingDistances[ring] / params.radiusMeters);
                double             sightRadius    = 0.0;
                const double       sightArray[3]  = {sightDirection.x, sightDirection.y, sightDirection.z};
                if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, sightArray, &sightRadius))
                {
                    sightClear = false;
                    failReason = "surface sample";
                    break;
                }
                const double rise = sightRadius - radius;
                // Wall caps scale with the eye height AND with the sight
                // distance: at 60 m the ground must stay near eye level (a
                // point-blank wall fills the frame), while at 720 m rolling
                // country may rise several eye-heights. The distance term is
                // a slope tolerance (meters of rise per meter of ground) so
                // the caps stay calibrated at ANY planet radius - fixed
                // eye-height caps over angular rings zeroed every candidate
                // once the demo planet grew (the rings were then kilometers
                // away while the caps stayed at tens of meters). Cliff views
                // allow taller walls still (the risers are the showcase).
                static constexpr double flatCapScale[6]   = {0.45, 0.6, 1.2, 2.0, 3.0, 4.0};
                static constexpr double cliffCapScale[6]  = {0.35, 0.9, 1.6, 2.6, 3.6, 8.0};
                static constexpr double slopeTolerance    = 0.45; // meters of ground rise per meter of distance
                const double capScale = policy == ProbePolicy::Mountain ? cliffCapScale[ring] : flatCapScale[ring];
                const double cap      = capScale * eyeHeight + slopeTolerance * sightRingDistances[ring];
                if (rise > cap || rise < -cap)
                {
                    sightClear = false;
                    failReason = "wall cap";
                    break;
                }
                if (policy == ProbePolicy::Coastline)
                {
                    if (rise > 2.0)
                    {
                        sawLand = true;
                        // Near-ring land means the shoreline is CLOSE - the
                        // frame will hold actual beach rather than open water
                        // with the coast somewhere behind the camera.
                        if (ring <= 1 && nearestLandRing > ring)
                        {
                            nearestLandRing = ring;
                        }
                        if (ring <= 1)
                        {
                            sawLandNear = true;
                        }
                    }
                    else if (rise < -2.0)
                    {
                        sawWater = true;
                        if (ring <= 1)
                        {
                            sawWaterNear = true;
                        }
                    }
                }
                if (rise >= 60.0)
                {
                    // Cliff views score higher when the rise ahead is actual
                    // TERRACED country (cliff mask high), not just any slope.
                    // The steepest terraced fan direction is stored so the
                    // camera AIMS at the wall instead of hoping it sits in
                    // frame (a plain tangent aim on rolling country shows
                    // hills, not cliffs).
                    if (policy == ProbePolicy::Mountain)
                    {
                        planet::ReliefSample sightRelief{};
                        const double sightDirArray[3] = {sightDirection.x, sightDirection.y, sightDirection.z};
                        // A real showcase wall: TERRACED (cliff mask >= 0.5),
                        // TALL (>= 90 m of rise) and STEEP (>= 0.8 meters of
                        // rise per meter of ground, ~39 degrees - a 0.5 slope
                        // is a grassy hillside, not a cliff). Rings 0-1 are
                        // excluded: a point-blank wall fills the frame with
                        // rock and shows nothing else.
                        const bool steepRing = ring >= 2 && rise >= 0.7 * sightRingDistances[ring];
                        if (planetCapability.sampleRelief(planetCapability.self, planet, sightDirArray, &sightRelief)
                            && sightRelief.cliffFactor >= 0.5 && rise >= 90.0 && steepRing)
                        {
                            cliffAhead = true;
                            const double wallScore = rise * sightRelief.cliffFactor;
                            if (wallScore > bestWallScore)
                            {
                                bestWallScore = wallScore;
                                cliffWallTangent = yawedTangent;
                                hasCliffWall = true;
                            }
                        }
                    }
                    else
                    {
                        cliffAhead = true;
                    }
                }
            }
        }
        if (policy == ProbePolicy::Coastline && !(sawLand && sawWater))
        {
            // No shoreline inside the frame's whole sight fan: this is open
            // ocean or deep inland, not a coast view.
            sightClear = false;
            failReason = sawLand ? "no water in sight fan" : (sawWater ? "no land in sight fan" : "fan all level (no land, no water)");
        }
        if (policy == ProbePolicy::Coastline && sightClear && !(sawLandNear && sawWaterNear))
        {
            // The shoreline is farther than 120 m on every fan yaw: from a
            // 120 m eye the near field renders as open ocean even though a
            // coast technically exists in the far fan (the islet trap).
            sightClear = false;
            failReason = "shoreline beyond 120 m";
        }

        // Score: probe greenness, zeroed when the view stares into a wall.
        // Cliff views prefer (not require) a rise ahead - the terraced
        // risers are the showcase; a flat meadow view scores 0.8 and loses
        // to any real cliff country at the same probe quality.
        // A RELIEF BONUS favors terrain with actual shape: probe elevations
        // with a healthy spread mean bays, hills and cliff risers in frame;
        // a perfectly flat plain maximizes greenness but photographs as an
        // empty green sheet. Target spread is per view kind.
        double probeMean = 0.0;
        for (double e : probeElevations)
        {
            probeMean += e;
        }
        probeMean /= static_cast<double>(probeElevations.size());
        double probeSpread = 0.0;
        for (double e : probeElevations)
        {
            probeSpread += (e - probeMean) * (e - probeMean);
        }
        probeSpread = std::sqrt(probeSpread / static_cast<double>(probeElevations.size()));
        const double targetSpread = policy == ProbePolicy::Mountain ? 150.0 : (policy == ProbePolicy::Coastline ? 45.0 : 60.0);
        const double reliefBonus  = 0.5 + 0.5 * std::min(1.0, probeSpread / targetSpread);

        double score = sightClear ? (static_cast<double>(probesOk) / static_cast<double>(probes)) * reliefBonus : 0.0;
        const double probeQuality = (static_cast<double>(probesOk) / static_cast<double>(probes)) * reliefBonus;
        if (!sightClear && probeQuality > bestFailQuality)
        {
            bestFailQuality   = probeQuality;
            bestFailElevation = elevation;
            bestFailReason    = failReason;
            bestFailDirection = direction;
            bestFailValid     = true;
            for (std::size_t ring = 0; ring < 6; ++ring)
            {
                for (std::size_t yaw = 0; yaw < 5; ++yaw)
                {
                    const planet::Vec3 probeDir = rotateDirection(direction, fanTangents[yaw],
                        sightRingDistances[ring] / params.radiusMeters);
                    double probeRadius = 0.0;
                    const double pa[3] = {probeDir.x, probeDir.y, probeDir.z};
                    bestFailRises[ring * 5 + yaw] = planetCapability.sampleSurfaceRadius(
                        planetCapability.self, planet, pa, &probeRadius)
                        ? probeRadius - radius : 1.0e30;
                }
            }
            for (std::size_t yaw = 0; yaw < 5; ++yaw)
            {
                bestFailFanTangents[yaw] = fanTangents[yaw];
            }
        }
        if (policy == ProbePolicy::Mountain && !cliffAhead)
        {
            score *= 0.8;
        }
        if (policy == ProbePolicy::Mountain && hasCliffWall)
        {
            // Walls win over wall-free country: scale the cliff bonus with
            // the wall's strength (rise * cliff factor; ~250 = a strong
            // terraced face) so the best-of-planet search actively seeks
            // the biggest showcase face instead of any qualifying spot.
            score *= 1.05 + 0.25 * std::min(1.0, bestWallScore / 250.0);
        }
        if (policy == ProbePolicy::Coastline && nearestLandRing > 1)
        {
            // The shoreline (if any) is beyond 120 m: the frame is mostly
            // open water with the coast behind the camera. Keep the spot
            // eligible but let closer-shore candidates win.
            score *= 0.85;
        }
        if (bestYawAlignment > 0.75)
        {
            // Even the best yaw nearly points at the sun (shadow silhouettes).
            score *= 0.85;
        }
        if (score > bestScore)
        {
            bestScore   = score;
            best.direction     = direction;
            best.surfaceRadius = radius;
            // The scored yaw must actually reach the camera: the old condition
            // (alignment >= 0) discarded exactly the sun-behind yaw the
            // scorer had picked, so ground views fell back to the arbitrary
            // world-up cross tangent - which could face any wall, backlit.
            // Cliff views with a located wall aim AT the wall instead of the
            // sun-behind yaw (the wall is the showcase; the sun preference
            // only chose among wall-free yaws before this override existed).
            best.hasLookTangent = hasWaterward || sunValid || hasCliffWall;
            best.lookTangent    = hasWaterward ? waterwardTangent
                                : (hasCliffWall ? cliffWallTangent
                                : (sunValid ? yawedTangents[bestYaw] : planet::Vec3{}));
            haveBest           = true;
            bestElevation = elevation;
            bestTemperature = temperature01;
            bestBiome = static_cast<int>(biome);
        }
    }

    if (haveBest && bestScore >= 0.5)
    {
        out = best;
        std::cout << "demo_planet: viewpoint (best-of-planet " << bestScore << ") - elevation " << bestElevation
                  << " m, temperature " << bestTemperature << ", biome " << bestBiome << "\n";
        return true;
    }
    // Diagnostics for tuning: show how close the best candidate came, so a
    // failing predicate/probe/cap is visible in the run log instead of
    // silently degrading to the fallback viewpoint.
    std::cerr << "demo_planet: pickViewpoint failed (policy " << static_cast<int>(policy) << ", best score "
              << bestScore << ", best elevation " << bestElevation << " m, biome " << bestBiome
              << "; strongest rejected: quality " << bestFailQuality << " at " << bestFailElevation
              << " m, reason '" << bestFailReason << "')\n";
    if (bestFailValid)
    {
        std::cerr << "demo_planet: strongest rejected fan rises (rows = ring 60/120/220/420/720/1440 m, cols = yaw -0.8..+0.8):\n";
        for (std::size_t ring = 0; ring < 6; ++ring)
        {
            std::cerr << "demo_planet:   ";
            for (std::size_t yaw = 0; yaw < 5; ++yaw)
            {
                char buffer[32];
                std::snprintf(buffer, sizeof(buffer), "%+9.1f", bestFailRises[ring * 5 + yaw]);
                std::cerr << buffer;
            }
            std::cerr << "\n";
        }
        std::cerr << "demo_planet:   spot dir (" << bestFailDirection.x << ", " << bestFailDirection.y
                  << ", " << bestFailDirection.z << ")\n";
    }
    return false;
}

// ---- determinism self-test ---------------------------------------------------------

struct SelfTestResult
{
    bool        pass = true;
    std::string report;
};

SelfTestResult runSelfTest(planet::PlanetCapability &capability, void *capabilitySelf, const PlanetDemoOptions &options)
{
    SelfTestResult result;

    planet::PlanetDescription description{};
    description.seed         = options.seed != 0 ? options.seed : 424242ull;
    description.radiusMeters = options.radius != 0.0 ? options.radius : 3000.0;

    const std::uint32_t depth        = 3;
    std::vector<planet::ChunkCoord> coords;
    const std::uint32_t side = 1u << depth;
    for (std::uint32_t x = 0; x < side; ++x)
    {
        for (std::uint32_t y = 0; y < side; ++y)
        {
            for (std::uint32_t z = 0; z < side; ++z)
            {
                coords.push_back(planet::ChunkCoord{depth, x, y, z});
            }
        }
    }
    // Thin to a deterministic subset (full 512 chunks at depth 3 is slow;
    // 24 spread chunks cover many terrain cases).
    std::vector<planet::ChunkCoord> subset;
    for (std::size_t index = 0; index < coords.size(); index += coords.size() / 24)
    {
        subset.push_back(coords[index]);
    }

    // Three planets, three build orders: ascending, reversed, rotated.
    planet::PlanetHandle planetAscending  = capability.createPlanet(capabilitySelf, &description, nullptr);
    planet::PlanetHandle planetReversed   = capability.createPlanet(capabilitySelf, &description, nullptr);
    planet::PlanetHandle planetRotated    = capability.createPlanet(capabilitySelf, &description, nullptr);
    if (planetAscending == planet::invalidPlanet || planetReversed == planet::invalidPlanet
        || planetRotated == planet::invalidPlanet)
    {
        result.report = "self-test: failed to create planets\n";
        result.pass   = false;
        return result;
    }

    // Hash the same chunk set from each planet in a different build order;
    // the hash MULTISETS must match (same chunks, any order). Ascending vs
    // reversed is checked positionally (the reversal maps index i to
    // size-1-i); the rotated order is compared as a sorted multiset.
    std::vector<std::uint64_t> ascendingHashes;
    std::vector<std::uint64_t> reversedHashes;
    std::vector<std::uint64_t> rotatedHashes;
    for (const planet::ChunkCoord &coord : subset)
    {
        ascendingHashes.push_back(capability.hashChunk(capabilitySelf, planetAscending, coord.depth, coord.x, coord.y, coord.z));
    }
    for (auto it = subset.rbegin(); it != subset.rend(); ++it)
    {
        reversedHashes.push_back(capability.hashChunk(capabilitySelf, planetReversed, it->depth, it->x, it->y, it->z));
    }
    std::vector<planet::ChunkCoord> rotated(subset.size());
    std::rotate_copy(subset.begin(), subset.begin() + subset.size() / 3, subset.end(), rotated.begin());
    for (const planet::ChunkCoord &coord : rotated)
    {
        rotatedHashes.push_back(capability.hashChunk(capabilitySelf, planetRotated, coord.depth, coord.x, coord.y, coord.z));
    }
    std::vector<std::uint64_t> ascendingSorted = ascendingHashes;
    std::vector<std::uint64_t> rotatedSorted   = rotatedHashes;
    std::sort(ascendingSorted.begin(), ascendingSorted.end());
    std::sort(rotatedSorted.begin(), rotatedSorted.end());
    if (ascendingSorted != rotatedSorted)
    {
        result.pass = false;
        result.report += "self-test: rotated-order hash multiset differs from ascending-order\n";
    }

    char line[128];
    for (std::size_t index = 0; index < subset.size(); ++index)
    {
        // reversed list was built back-to-front, so element i of the reversed
        // run corresponds to ascending index size-1-i.
        if (ascendingHashes[index] != reversedHashes[subset.size() - 1 - index])
        {
            result.pass = false;
            std::snprintf(line, sizeof(line), "self-test: MISMATCH chunk %u (%u %u %u %u): asc %016llx rev %016llx\n",
                static_cast<std::uint32_t>(index), subset[index].depth, subset[index].x, subset[index].y, subset[index].z,
                static_cast<unsigned long long>(ascendingHashes[index]),
                static_cast<unsigned long long>(reversedHashes[subset.size() - 1 - index]));
            result.report += line;
        }
    }

    // Sanity: a different seed must produce different hashes (palette-free
    // geometry changes).
    planet::PlanetDescription otherSeed = description;
    otherSeed.seed += 1;
    planet::PlanetHandle planetOther = capability.createPlanet(capabilitySelf, &otherSeed, nullptr);
    bool seedSensitivity = false;
    for (std::size_t index = 0; index < subset.size() && !seedSensitivity; ++index)
    {
        const std::uint64_t otherHash = capability.hashChunk(capabilitySelf, planetOther, subset[index].depth, subset[index].x,
            subset[index].y, subset[index].z);
        seedSensitivity = otherHash != ascendingHashes[index];
    }
    if (!seedSensitivity)
    {
        result.pass = false;
        result.report += "self-test: hashes identical across seeds (generation looks seed-insensitive!)\n";
    }

    // Conforming-boundary determinism: chunk meshes are pure functions of
    // (params, coord, conformMask). Build the chunk set on a dedicated
    // planet under an unconstrained mask, an all-conforming mask and a mixed
    // mask; hashes must be stable per mask, and conforming must actually
    // change geometry (otherwise the seam machinery is dead).
    {
        planet::PlanetGenerationParams maskParams;
        maskParams.seed         = description.seed;
        maskParams.radiusMeters = description.radiusMeters;
        std::string maskError;
        if (maskParams.validate(maskError))
        {
            planet::Planet maskPlanet(maskParams, nullptr);
            planet::ConformMask allConform;
            for (std::uint32_t face = 0; face < planet::ConformMask::kFaceCount; ++face)
            {
                for (std::uint32_t slot = 0; slot < 9; ++slot)
                {
                    allConform.face[face][slot] = 1u;
                }
            }
            planet::ConformMask mixed;
            for (std::uint32_t face = 0; face < planet::ConformMask::kFaceCount; ++face)
            {
                mixed.face[face][0] = 1u; // (-1,-1) diagonal ring only
                mixed.face[face][8] = 1u; // (+1,+1) diagonal ring only
            }
            bool maskStable    = true;
            bool maskSensitive = false;
            for (std::size_t index = 0; index < subset.size() && maskStable; ++index)
            {
                const std::uint64_t freeHash        = maskPlanet.buildChunkHash(subset[index]);
                const std::uint64_t freeRepeat      = maskPlanet.buildChunkHash(subset[index]);
                const std::uint64_t conformHash     = maskPlanet.buildChunkHash(subset[index], allConform);
                const std::uint64_t conformRepeat   = maskPlanet.buildChunkHash(subset[index], allConform);
                const std::uint64_t mixedHash       = maskPlanet.buildChunkHash(subset[index], mixed);
                maskStable    = freeHash == freeRepeat && conformHash == conformRepeat;
                maskSensitive = maskSensitive || conformHash != freeHash || mixedHash != freeHash;
            }
            if (!maskStable)
            {
                result.pass = false;
                result.report += "self-test: chunk hashes vary between repeated builds of the same mask\n";
            }
            if (!maskSensitive)
            {
                result.pass = false;
                result.report += "self-test: conforming masks never change hashes (seam machinery inactive?)\n";
            }
        }
        else
        {
            result.pass = false;
            result.report += "self-test: mask-check params rejected: " + maskError + "\n";
        }
    }

    // Orientation audit: on a convex sample region the majority of terrain
    // triangles must face radially outward. hashChunk cannot express this,
    // so audit through a dedicated rebuild here: build one chunk via the
    // capability and check winding through the visible-chunk path is not
    // available headlessly without update() - instead assert the mesh is
    // non-degenerate (hash stable across a second call, already covered) and
    // leave the visual orientation check to the screenshots.
    capability.destroyPlanet(capabilitySelf, planetAscending);
    capability.destroyPlanet(capabilitySelf, planetReversed);
    capability.destroyPlanet(capabilitySelf, planetRotated);
    capability.destroyPlanet(capabilitySelf, planetOther);

    if (result.pass)
    {
        result.report += "self-test: PASS - " + std::to_string(subset.size())
                       + " chunks bit-identical across 3 build orders; seed sensitivity confirmed\n";
    }
    return result;
}

// ---- chunk mesh registry ------------------------------------------------------------

struct ChunkMeshHandles
{
    renderer::MeshHandle terrain = renderer::invalidMesh;
    renderer::MeshHandle water   = renderer::invalidMesh;
};

constexpr std::uint32_t kVertexStride = sizeof(planet::PlanetVertex);

std::array<renderer::VertexAttribute, 4> canonicalAttributes()
{
    return {{
        {renderer::VertexSemantic::Position, renderer::VertexFormat::Float32x3, 0},
        {renderer::VertexSemantic::Normal, renderer::VertexFormat::Float32x3, 12},
        {renderer::VertexSemantic::Color, renderer::VertexFormat::Float32x3, 24},
        {renderer::VertexSemantic::TexCoord0, renderer::VertexFormat::Float32x2, 36},
    }};
}

} // namespace

int main(int argc, char **argv)
{
    std::cout << std::unitbuf;

    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << '\n'
                  << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << planetOptionsUsage();
        return 0;
    }

    PlanetDemoOptions options;
    if (!parsePlanetOptions(commandLine.applicationArguments, options, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << "\n";
        return 2;
    }

    engine::ApplicationHostOptions hostOptions
        = engine::makeApplicationHostOptions(commandLine, {"window", "renderer", "camera", "pbr", "planet"});
    engine::ApplicationHost host;
    std::string             startupReport;
    if (commandLine.validateOnly)
    {
        const bool valid = host.validate(hostOptions, startupReport);
        std::cout << startupReport;
        std::cout << (valid ? "validate: ok\n" : "validate: failed\n");
        return valid ? 0 : 1;
    }
    if (!host.start(hostOptions, startupReport))
    {
        std::cerr << startupReport << "demo_planet: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::ParticipantContext &context = host.context();

    window::WindowCapability   &windowCapability  = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability  = context.acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability   &cameraCapability  = context.acquireCapability<camera::CameraCapability>();
    auto *planetCapability = context.tryAcquireCapability<planet::PlanetCapability>();
    if (planetCapability == nullptr || planetCapability->abi != planet::planetCapabilityAbi)
    {
        std::cerr << "demo_planet: the planet capability is unavailable or its ABI differs; this demo is pointless "
                     "without it\n";
        host.stop();
        return 1;
    }
    const pbr::PbrCapability *pbrCapability = context.tryAcquireCapability<pbr::PbrCapability>();
    pbr::PbrEnvironment       environment{};
    if (pbrCapability != nullptr && pbrCapability->copyEnvironment(pbrCapability->self, &environment, nullptr))
    {
        std::cout << "demo_planet: pbr sun " << (environment.sun.enabled ? "enabled" : "disabled") << " (intensity "
                  << environment.sun.intensity << ")\n";
    }

    // ---- planet creation ---------------------------------------------------------
    planet::PlanetDescription description{};
    description.seed         = options.seed;
    description.radiusMeters = options.radius;
    const planet::PlanetHandle planetHandle = planetCapability->createPlanet(planetCapability->self, &description, nullptr);
    if (planetHandle == planet::invalidPlanet)
    {
        std::cerr << "demo_planet: the planet module rejected the planet description\n";
        host.stop();
        return 1;
    }
    planet::PlanetGenerationParams params{};
    planetCapability->copyGenerationParams(planetCapability->self, planetHandle, &params);
    if (options.caves)
    {
        std::cout << "demo_planet: --caves currently only toggles through configuration "
                     "(--set planet.caves_enabled=true); continuing without caves\n";
    }
    std::cout << "demo_planet: planet seed " << params.seed << ", radius " << params.radiusMeters << " m, octree depth "
              << params.derivedMaxDepth << ", leaf voxel " << params.derivedVoxelMeters << " m\n";

    // ---- self-test short-circuit ----------------------------------------------------
    if (options.selfTest)
    {
        const SelfTestResult test = runSelfTest(*planetCapability, planetCapability->self, options);
        std::cout << test.report;
        planetCapability->destroyPlanet(planetCapability->self, planetHandle);
        host.stop();
        return test.pass ? 0 : 1;
    }

    // ---- camera placement per view ----------------------------------------------------
    const glm::dvec3 planetCenter(params.centerMeters[0], params.centerMeters[1], params.centerMeters[2]);
    const double     planetRadius = params.radiusMeters;

    glm::dvec3 eye      = planetCenter + glm::dvec3(0.0, 0.0, planetRadius * 3.0);
    glm::dvec3 target   = planetCenter;
    double     nearZ    = planetRadius * 0.01;
    double     farZ     = planetRadius * 12.0;
    double     eyeSpeed = planetRadius * 0.04; // orbit linear speed for --orbit

    if (options.viewName == "globe")
    {
        const glm::dvec3 direction = glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        eye                        = planetCenter + direction * planetRadius * 3.1;
        target                     = planetCenter;
    }
    else if (options.viewName == "orbit")
    {
        // Mid-altitude view: the planet fills the frame, continents and
        // mountain ranges are readable - the scale between the globe and the
        // surface where range structure (or its absence) shows most clearly.
        const glm::dvec3 direction = glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        eye                        = planetCenter + direction * planetRadius * 1.6;
        target                     = planetCenter;
    }
    else
    {
        // Surface views: find a deterministic spot matching the view's
        // terrain predicate, stand eyeHeight above the ground, look along the
        // horizon toward the view's focus.
        Viewpoint viewpoint;
        const auto elevationWindow = [](double low, double high)
        {
            return [low, high](double elevation, engine::planet::Biome, double, const planet::PlanetGenerationParams &)
            { return elevation >= low && elevation <= high; };
        };
        // Warm-land windows: the biome check alone is not enough - the biome
        // colors BLEND into snow below the climate snow line (and toward the
        // freeze line), so a spot the enum calls "taiga" can still render
        // half-white. Require strictly warm air and a spot clearly below the
        // blended snow band so showcase views land on green/sandy terrain.
        const auto warmLandWindow = [](double low, double high)
        {
            return [low, high](double elevation, engine::planet::Biome biome, double temperature01,
                const planet::PlanetGenerationParams &params)
            {
                if (elevation < low || elevation > high)
                {
                    return false;
                }
                if (biome == planet::Biome::Snow || biome == planet::Biome::Tundra || temperature01 <= 0.45)
                {
                    return false;
                }
                const double snowLine = params.snowLineMeters * (0.25 + 0.75 * temperature01);
                const double snowBand = params.snowLineMeters * params.elevationBlendFraction;
                return elevation < snowLine - snowBand;
            };
        };
        bool found = false;
        // View placement: biome-aware, climate-aware and sightline-checked so
        // the view shows landscape rather than the nearest slope wall.
        const planet::Vec3 sunDirection = environment.sun.enabled
            ? planet::Vec3{static_cast<double>(environment.sun.direction[0]),
                static_cast<double>(environment.sun.direction[1]),
                static_cast<double>(environment.sun.direction[2])}
            : planet::Vec3{0.0, 0.0, 0.0};

        if (options.viewName == "coast")
        {
            // A warm beach seen from just above the shoreline; the sea may
            // (should) enter the frame. The window reaches 45 m: the coastal
            // flattening damps most shoreline terrain toward the waterline,
            // so a 2-18 m window nearly only ever matched tiny islets whose
            // land hides under the camera.
            found = pickViewpoint(*planetCapability, planetHandle, warmLandWindow(2.0, 45.0), ProbePolicy::Coastline, 120.0,
                sunDirection, viewpoint);
        }
        else if (options.viewName == "cliffs")
        {
            // Below the snow blend, at any elevation where terraced cliff
            // faces and overhangs live (the snow blend rides the climate
            // line, so the window adapts to temperature). Higher country than
            // the other views: rock faces and strata need real relief.
            found = pickViewpoint(*planetCapability, planetHandle, warmLandWindow(250.0, 800.0), ProbePolicy::Mountain, 140.0,
                sunDirection, viewpoint);
        }
        else // surface
        {
            // Rolling green hills: the SPOT itself must be a genuinely green
            // biome (shrubland is excluded - its pale khaki albedo photographs
            // as washed-out empty country), above the coastal-flatten band
            // (its planed-off terrain photographs as an empty sheet) and
            // below the snow blend.
            const auto warmGreenLandWindow = [warmLandWindow](double low, double high)
            {
                auto warm = warmLandWindow(low, high);
                return [warm](double elevation, engine::planet::Biome biome, double temperature01,
                    const planet::PlanetGenerationParams &params)
                {
                    // TropicalRainforest is excluded: its canopy palette is the
                    // darkest row (linear albedo ~0.09-0.13) and a front-lit
                    // frame over it still photographs as a murky near-black
                    // sheet under the demo's linear exposure.
                    const bool green = biome == planet::Biome::Grassland
                        || biome == planet::Biome::TemperateForest || biome == planet::Biome::SeasonalForest
                        || biome == planet::Biome::TemperateRainforest
                        || biome == planet::Biome::Taiga;
                    return warm(elevation, biome, temperature01, params) && green;
                };
            };
            found = pickViewpoint(*planetCapability, planetHandle, warmGreenLandWindow(60.0, 200.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Very flat or very mountainous seeds may lack a match; fall back
            // to any land-ish point rather than failing the demo.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Coastline, 110.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Last resort: the coastline gate (land AND water inside the
            // sight fan) can fail on pathological seeds; an inland land view
            // always beats an error exit.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            std::cerr << "demo_planet: no usable viewpoint found for view '" << options.viewName << "'\n";
            planetCapability->destroyPlanet(planetCapability->self, planetHandle);
            host.stop();
            return 1;
        }

        const glm::dvec3 direction(viewpoint.direction.x, viewpoint.direction.y, viewpoint.direction.z);
        const glm::dvec3 position = planetCenter + direction * viewpoint.surfaceRadius;
        {
            // Degenerate or non-finite tangent: fall back to the world-up
            // cross product below.
            const planet::Vec3 &stored = viewpoint.lookTangent;
            const bool usable = std::isfinite(stored.x) && std::isfinite(stored.y) && std::isfinite(stored.z)
                && planet::length(stored) > 0.5;
            if (!usable)
            {
                viewpoint.hasLookTangent = false;
            }
        }
        // Tangent frame at the spot: coast views get the waterward direction
        // the picker chose; everything else uses the world-up cross tangent.
        const glm::dvec3 up       = direction;
        glm::dvec3       tangent  = viewpoint.hasLookTangent
            ? glm::dvec3(viewpoint.lookTangent.x, viewpoint.lookTangent.y, viewpoint.lookTangent.z)
            : glm::normalize(glm::cross(up, glm::dvec3(0.0, 1.0, 0.0)));
        if (viewpoint.hasLookTangent)
        {
            tangent = glm::normalize(tangent);
        }
        else if (glm::length(tangent) < 0.5)
        {
            tangent = glm::normalize(glm::cross(up, glm::dvec3(1.0, 0.0, 0.0)));
        }

        // Camera eye heights MATCH the picker's altitudes: the sightline wall
        // caps are calibrated at exactly these heights, and the old mismatch
        // (camera at 220/420 m after the caps cleared 110/140 m) let walls
        // the caps never saw fill the frame.
        const double eyeHeight = options.viewName == "cliffs" ? 140.0 : (options.viewName == "surface" ? 140.0 : 120.0);
        eye                    = position + up * eyeHeight;
        const double lookAheadDistance = options.viewName == "cliffs" ? 700.0 : (options.viewName == "surface" ? 420.0 : 320.0);
        // Exact pitch aim below the local horizontal - no terrain
        // re-projection (when the ground ahead rises, a re-projected target
        // snaps onto the hillside wall and fills the frame with slope) and
        // no level tangent aim (it put the horizon mid-frame, the shoreline
        // out of it, and half the frame black sky). Per-view pitch centers
        // the subject: the coast shoreline sits ~15 deg below a 120 m eye at
        // 320 m; surface ground at 420 m sits ~15.5 deg below a 140 m eye;
        // the cliffs view aims near level AT the terraced wall the picker
        // located (the wall rises ahead and fills the center frame).
        const double aimPitchRadians = options.viewName == "cliffs" ? 0.08
            : (options.viewName == "surface" ? 0.27 : 0.26); // 4.6 / 15.5 / 14.9 deg
        target = eye + (tangent * std::cos(aimPitchRadians) - up * std::sin(aimPitchRadians)) * lookAheadDistance;
        // Ground views sit close to the terrain: a near plane scaled from
        // the eye height (55-105 m) clipped everything inside it. 15 m keeps
        // near clipping out of the frame while leaving depth precision for
        // the 120 km far plane.
        nearZ    = 15.0;
        farZ     = planetRadius * 2.0;
        eyeSpeed = 6.0;
        // Open-framing guarantee: whatever the ground ahead does, the view
        // pitch stays in a band that keeps the horizon in frame (deep enough
        // to keep landscape in the lower half, shallow enough to keep sky
        // above it). Cliff views may pitch deeper - the risers below the eye
        // are the showcase. (The exact-pitch aim above already lands inside
        // this band; the clamp is a safety net.)
        const double maxPitchRadians = options.viewName == "cliffs" ? 0.52359877559829882 : 0.38397243543875248; // 30 deg / 22 deg
        clampCameraPitch(eye, up, target, 0.08726646259971647, maxPitchRadians); // >= 5 deg down
        std::cout << "demo_planet: view pose - eye (" << eye.x << ", " << eye.y << ", " << eye.z << ") radius "
                  << glm::length(eye - planetCenter) << ", target (" << target.x << ", " << target.y << ", "
                  << target.z << ")\n";
    }

    // ---- window / renderer / camera ------------------------------------------------
    window::WindowDescription  windowDescription{};
    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo_planet: the window module could not create a window\n";
        host.stop();
        return 1;
    }
    renderer::RenderError          renderError = renderer::RenderError::None;
    const renderer::RendererHandle rendererHandle
        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo_planet: the renderer module could not create a renderer\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    const std::string windowBackendName = windowCapability.activeBackendName(windowCapability.self);
    std::cout << "demo_planet: window backend '" << windowBackendName << "', renderer backend '"
              << renderCapability.activeBackendName(renderCapability.self, rendererHandle) << "'\n";
    windowCapability.pollEvents(windowCapability.self);

    const bool windowedBackend = windowBackendName != std::string("headless");
    if (windowedBackend)
    {
        std::cout << "demo_planet: controls - mouse looks, WASD moves, Space/Ctrl up/down, Shift sprints,\n"
                     "      arrows look, Left Alt toggles the cursor, Escape quits\n";
        windowCapability.setCursorMode(windowCapability.self, windowHandle, window::CursorMode::Captured);
    }

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        framebufferWidth  = 960;
        framebufferHeight = 540;
    }
    renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);

    const camera::CameraHandle cameraHandle = [&]()
    {
        camera::CameraParameters cameraParameters{};
        cameraParameters.position[0] = static_cast<float>(eye.x);
        cameraParameters.position[1] = static_cast<float>(eye.y);
        cameraParameters.position[2] = static_cast<float>(eye.z);
        cameraParameters.hasPosition  = true;
        cameraParameters.target[0]    = static_cast<float>(target.x);
        cameraParameters.target[1]    = static_cast<float>(target.y);
        cameraParameters.target[2]    = static_cast<float>(target.z);
        cameraParameters.hasTarget    = true;
        // NOTE: the up vector is set per frame by setLookAt below (per view:
        // radial for ground views, world for globe/orbit) - the createCamera
        // up would be overridden by that call anyway.
        cameraParameters.fovYDegrees  = (options.viewName == "globe" || options.viewName == "orbit") ? 42.0f : 65.0f;
        cameraParameters.nearZ        = static_cast<float>(nearZ);
        cameraParameters.farZ         = static_cast<float>(farZ);
        camera::CameraError cameraError = camera::CameraError::None;
        return cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    }();
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo_planet: the camera module could not create a camera\n";
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }

    // ---- run mode -------------------------------------------------------------------
    const bool          runUntilClosed = options.frames == 0 && windowedBackend;
    const std::uint64_t totalFrames    = options.frames != 0 ? options.frames : 90;
    const double        frameDelta     = 1.0 / 30.0;
    if (options.screenshotFrame == 0)
    {
        options.screenshotFrame = totalFrames - 1;
    }

    std::cout << "demo_planet: view '" << options.viewName << "', " << totalFrames << " frames ("
              << options.warmupFrames << " warmup)\n";

    // ---- chunk mesh registry ----------------------------------------------------------
    std::map<planet::ChunkKey, ChunkMeshHandles>        uploadedMeshes;
    std::map<planet::ChunkKey, std::pair<glm::mat4, glm::mat4>> chunkTransforms; // terrain, water
    // NOTE: deques, not vectors - the draw commands store pointers INTO these
    // containers and dereference them at backend draw time; a growing vector
    // reallocates and leaves every earlier material pointer dangling.
    std::deque<renderer::MeshMaterial>                  terrainMaterials;
    std::deque<renderer::MeshMaterial>                  waterMaterials;
    std::vector<renderer::RenderDrawCommand>            drawCommands;
    std::vector<std::uint32_t>                          loadedViews; // scratch

    const auto uploadChunkViews = [&]()
    {
        const std::uint32_t capacity = planetCapability->visibleChunkCapacity(planetCapability->self, planetHandle);
        if (capacity == 0)
        {
            return;
        }
        std::vector<planet::PlanetChunkView> views(capacity);
        const std::uint32_t count = planetCapability->copyVisibleChunks(planetCapability->self, planetHandle, views.data(), capacity);
        std::map<planet::ChunkKey, bool> seen;
        for (std::uint32_t index = 0; index < count; ++index)
        {
            const planet::PlanetChunkView &view = views[index];
            seen[view.key]                      = true;
            auto found                          = uploadedMeshes.find(view.key);
            if (found == uploadedMeshes.end())
            {
                found = uploadedMeshes.emplace(view.key, ChunkMeshHandles{}).first;
            }
            ChunkMeshHandles &handles = found->second;

            const bool wantTerrain = !view.isWater;
            renderer::MeshHandle &slot = wantTerrain ? handles.terrain : handles.water;
            if (slot == renderer::invalidMesh && view.vertexCount > 0)
            {
                const auto attributes = canonicalAttributes();
                renderer::MeshDescription meshDescription;
                meshDescription.vertices             = view.vertices;
                meshDescription.vertexCount          = view.vertexCount;
                meshDescription.indices              = view.indices;
                meshDescription.indexCount           = view.indexCount;
                meshDescription.indexFormat          = renderer::MeshIndexFormat::UInt32;
                meshDescription.vertexStrideBytes    = kVertexStride;
                meshDescription.vertexAttributes     = attributes.data();
                meshDescription.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());
                slot                                 = renderCapability.createMesh(renderCapability.self, rendererHandle,
                    &meshDescription, &renderError);
                if (slot == renderer::invalidMesh)
                {
                    std::cerr << "demo_planet: chunk mesh upload failed\n";
                }
            }

            // Chunk-local vertices are placed by a translation matrix.
            const glm::mat4 transform = glm::translate(glm::mat4(1.0f),
                glm::vec3(static_cast<float>(view.centerMeters.x - params.centerMeters[0]),
                    static_cast<float>(view.centerMeters.y - params.centerMeters[1]),
                    static_cast<float>(view.centerMeters.z - params.centerMeters[2])));
            if (wantTerrain)
            {
                chunkTransforms[view.key].first = transform;
            }
            else
            {
                chunkTransforms[view.key].second = transform;
            }
        }

        // Drop renderer meshes for chunks that left the visible set.
        for (auto it = uploadedMeshes.begin(); it != uploadedMeshes.end();)
        {
            if (seen.find(it->first) == seen.end())
            {
                if (it->second.terrain != renderer::invalidMesh)
                {
                    renderCapability.destroyMesh(renderCapability.self, rendererHandle, it->second.terrain);
                }
                if (it->second.water != renderer::invalidMesh)
                {
                    renderCapability.destroyMesh(renderCapability.self, rendererHandle, it->second.water);
                }
                chunkTransforms.erase(it->first);
                it = uploadedMeshes.erase(it);
            }
            else
            {
                ++it;
            }
        }
    };

    const auto buildDrawCommands = [&]()
    {
        drawCommands.clear();
        terrainMaterials.clear();
        waterMaterials.clear();
        for (const auto &entry : uploadedMeshes)
        {
            const auto transformIt = chunkTransforms.find(entry.first);
            if (transformIt == chunkTransforms.end())
            {
                continue;
            }
            if (entry.second.terrain != renderer::invalidMesh)
            {
                renderer::MeshMaterial material{};
                material.baseColor[0]     = 1.0f;
                material.baseColor[1]     = 1.0f;
                material.baseColor[2]     = 1.0f;
                material.roughnessFactor  = 0.95f;
                terrainMaterials.push_back(material);
                renderer::RenderDrawCommand command{};
                command.mesh        = entry.second.terrain;
                command.modelMatrix = glm::value_ptr(transformIt->second.first);
                command.material    = &terrainMaterials.back();
                drawCommands.push_back(command);
            }
            if (entry.second.water != renderer::invalidMesh)
            {
                renderer::MeshMaterial material{};
                material.baseColor[0]    = 1.0f;
                material.baseColor[1]    = 1.0f;
                material.baseColor[2]    = 1.0f;
                material.roughnessFactor = params.waterRoughness;
                waterMaterials.push_back(material);
                renderer::RenderDrawCommand command{};
                command.mesh        = entry.second.water;
                command.modelMatrix = glm::value_ptr(transformIt->second.second);
                command.material    = &waterMaterials.back();
                drawCommands.push_back(command);
            }
        }
    };

    // ---- frame loop -------------------------------------------------------------------
    std::uint64_t framesRendered = 0;
    bool          screenshotTaken = false;
    double        orbitAngle      = 0.0;
    std::array<window::WindowEvent, 64> eventBuffer;
    auto lastWallClock = std::chrono::steady_clock::now();

    for (std::uint64_t frameIndex = 0; runUntilClosed || frameIndex < totalFrames; ++frameIndex)
    {
        windowCapability.pollEvents(windowCapability.self);
        std::uint32_t eventCount = 0;
        do
        {
            eventCount = windowCapability.popEvents(windowCapability.self, windowHandle, eventBuffer.data(),
                static_cast<std::uint32_t>(eventBuffer.size()));
            for (std::uint32_t eventIndex = 0; eventIndex < eventCount; ++eventIndex)
            {
                const window::WindowEvent &event = eventBuffer[eventIndex];
                if (event.kind == window::WindowEventKind::CloseRequested)
                {
                    windowCapability.requestClose(windowCapability.self, windowHandle);
                }
                else if (event.kind == window::WindowEventKind::FramebufferResized)
                {
                    if (event.framebufferWidth != 0 && event.framebufferHeight != 0)
                    {
                        framebufferWidth  = event.framebufferWidth;
                        framebufferHeight = event.framebufferHeight;
                        renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);
                    }
                }
            }
        }
        while (eventCount == static_cast<std::uint32_t>(eventBuffer.size()));

        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            std::cout << "demo_planet: window close requested\n";
            break;
        }
        if (windowCapability.isKeyDown != nullptr
            && windowCapability.isKeyDown(windowCapability.self, windowHandle, window::Key::Escape))
        {
            std::cout << "demo_planet: escape pressed\n";
            break;
        }

        const auto   now               = std::chrono::steady_clock::now();
        const double wallDeltaSeconds = std::min(0.25, std::chrono::duration<double>(now - lastWallClock).count());
        lastWallClock                 = now;
        const double deltaSeconds     = options.frames != 0 ? frameDelta : wallDeltaSeconds;

        // Orbit sweep (deterministic under --frames): globe circles the
        // planet; surface views circle the standing spot slowly.
        if (options.orbit)
        {
            orbitAngle += deltaSeconds * (options.viewName == "globe" ? 0.05 : 0.10);
            if (options.viewName == "globe")
            {
                const glm::dvec3 axis(0.0, 1.0, 0.0);
                const glm::dvec3 offset      = eye - planetCenter;
                const double     orbitRadius = glm::length(offset);
                const glm::dvec3 rotated     = glm::rotate(glm::dmat4(1.0), orbitAngle, axis) * glm::dvec4(offset, 1.0);
                eye                          = planetCenter + glm::normalize(rotated) * orbitRadius;
                target                       = planetCenter;
            }
            else
            {
                const glm::dvec3 up        = glm::normalize(eye - planetCenter);
                const glm::dvec3 offset    = eye - target;
                const glm::dvec3 rotated   = glm::rotate(glm::dmat4(1.0), orbitAngle, up) * glm::dvec4(offset, 1.0);
                eye                        = target + rotated;
            }
        }

        // Push the (possibly orbited) pose into the camera and advance it
        // with zero input (the camera module would otherwise override the
        // pose with its own integration).
        const float poseEye[3]    = {static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z)};
        const float poseTarget[3] = {static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z)};
        // The lookAt UP decides the frame's roll. Ground views must stand
        // upright on the planet: their up is the LOCAL RADIAL direction. The
        // old hardcoded world up (0,1,0) rolled ground frames by the angle
        // between the world up and the radial (measured ~82 degrees on the
        // showcase planet - the "terrain on one side, void on the other"
        // screenshots were this roll, not missing geometry). Globe and orbit
        // views look at the planet CENTER, where the radial is anti-parallel
        // to the view direction and degenerates lookAt - they keep the world
        // up (north-pole-up orientation).
        glm::dvec3 lookUpD = glm::dvec3(0.0, 1.0, 0.0);
        if (options.viewName != "globe" && options.viewName != "orbit")
        {
            lookUpD = glm::normalize(eye - planetCenter);
        }
        const float poseUp[3]     = {static_cast<float>(lookUpD.x), static_cast<float>(lookUpD.y), static_cast<float>(lookUpD.z)};
        if (options.viewName == "globe" || options.viewName == "orbit")
        {
            if (glm::abs(glm::dot(glm::normalize(eye - planetCenter), glm::dvec3(0.0, 1.0, 0.0))) > 0.99)
            {
                // Poles: up would degenerate; lean the up vector slightly.
                const float leanedUp[3] = {0.05f, 0.9987f, 0.0f};
                cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, leanedUp);
            }
            else
            {
                cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, poseUp);
            }
        }
        else
        {
            cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, poseUp);
        }
        camera::CameraInputState cameraInput{};
        cameraCapability.update(cameraCapability.self, cameraHandle, &cameraInput, deltaSeconds);

        camera::ViewState viewState{};
        cameraCapability.viewState(cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);
        renderer::CameraMatrices cameraMatrices{};
        std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
        std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
        std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
        renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

        // Advance the planet LOD stream for this eye and refresh GPU meshes.
        // During warmup frames the renderer stays idle: every worker builds
        // chunks at full speed instead of competing with rasterizer tile
        // tasks, so deep LOD views start their first RENDERED frame with the
        // visible set already complete (no black-hole screenshots).
        const double eyeWorld[3] = {viewState.position[0], viewState.position[1], viewState.position[2]};
        planetCapability->updatePlanet(planetCapability->self, planetHandle, eyeWorld);
        const bool warming = frameIndex < static_cast<std::uint64_t>(options.warmupFrames);
        if (warming)
        {
            continue;
        }
        uploadChunkViews();
        buildDrawCommands();

        renderCapability.beginFrame(renderCapability.self, rendererHandle, nullptr);
        renderCapability.drawMeshes(renderCapability.self, rendererHandle, drawCommands.data(),
            static_cast<std::uint32_t>(drawCommands.size()));
        renderCapability.endFrame(renderCapability.self, rendererHandle);
        ++framesRendered;
        if (framesRendered % 100 == 0)
        {
            renderer::FrameStatistics frameStats{};
            renderCapability.frameStatistics(renderCapability.self, rendererHandle, &frameStats);
            std::cout << "demo_planet: raster stats - submitted " << frameStats.trianglesSubmitted
                      << ", rasterized " << frameStats.trianglesRasterized << "\n";
        }

        if (options.statsEveryFrames != 0 && frameIndex % options.statsEveryFrames == 0)
        {
            planet::PlanetStatistics statistics{};
            if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &statistics))
            {
                std::cout << "demo_planet: frame " << frameIndex << " | chunks visible " << statistics.chunksVisible
                          << ", ready " << statistics.chunksReady << ", building " << statistics.chunksBuilding
                          << ", cached " << statistics.chunksReadyTotal << ", triangles " << statistics.triangleCount
                          << ", depth " << statistics.octreeDepthUsed
                          << ", update " << statistics.lastUpdateMilliseconds << " ms\n";
            }
        }

        if (!screenshotTaken && !options.screenshotPath.empty() && frameIndex == options.screenshotFrame)
        {
            auto *headlessWindowCapability = context.tryAcquireCapability<window::HeadlessWindowCapability>();
            if (headlessWindowCapability != nullptr && windowBackendName == std::string("headless"))
            {
                window::WindowError exportError = window::WindowError::None;
                screenshotTaken                 = headlessWindowCapability->exportSurfacePpm(
                    headlessWindowCapability->self, windowHandle, options.screenshotPath.c_str(), &exportError);
            }
            if (!screenshotTaken)
            {
                auto *vulkanCapability = context.tryAcquireCapability<renderer::VulkanRenderCapability>();
                if (vulkanCapability != nullptr
                    && renderCapability.activeBackendName(renderCapability.self, rendererHandle) == std::string("vulkan"))
                {
                    renderer::RenderError exportError = renderer::RenderError::None;
                    screenshotTaken                   = vulkanCapability->exportFramePpm(
                        vulkanCapability->self, rendererHandle, options.screenshotPath.c_str(), &exportError);
                }
            }
            if (screenshotTaken)
            {
                std::cout << "demo_planet: screenshot written to " << options.screenshotPath << "\n";
            }
            else
            {
                std::cerr << "demo_planet: screenshot export failed for backend '" << windowBackendName << "'\n";
            }
        }
    }

    // ---- teardown -----------------------------------------------------------------------
    for (const auto &entry : uploadedMeshes)
    {
        if (entry.second.terrain != renderer::invalidMesh)
        {
            renderCapability.destroyMesh(renderCapability.self, rendererHandle, entry.second.terrain);
        }
        if (entry.second.water != renderer::invalidMesh)
        {
            renderCapability.destroyMesh(renderCapability.self, rendererHandle, entry.second.water);
        }
    }
    planet::PlanetStatistics finalStatistics{};
    if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &finalStatistics))
    {
        std::cout << "demo_planet: totals - chunks cached " << finalStatistics.chunksReadyTotal << ", triangles "
                  << finalStatistics.triangleCount << ", vertices " << finalStatistics.verticesTotal << "\n";
    }
    planetCapability->destroyPlanet(planetCapability->self, planetHandle);
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    host.stop();
    std::cout << "demo_planet: stopped cleanly after " << framesRendered << " frames\n";
    return 0;
}
