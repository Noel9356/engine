#include <engine/planet/capabilities.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include "planet_demo_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iostream>
#include <vector>

namespace demo::planet_demo
{
namespace planet = engine::planet;

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

// The biome name table: the classifier's 16 climate zones with their CLI
// spellings (shared by DEMO_BIOME validation and the biome-pinned view).
const std::vector<std::string> &biomeNames()
{
    static const std::vector<std::string> names = {
        "deep_ocean", "ocean", "shallow_ocean", "beach", "desert", "savanna", "grassland",
        "shrubland", "temperate_forest", "seasonal_forest", "tropical_rainforest",
        "temperate_rainforest", "taiga", "tundra", "snow", "rock",
    };
    return names;
}

bool biomeFromName(const std::string &name, planet::Biome &outBiome)
{
    const std::vector<std::string> &names = biomeNames();
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        if (name == names[index])
        {
            outBiome = static_cast<planet::Biome>(index);
            return true;
        }
    }
    return false;
}

const char *biomeName(planet::Biome biome)
{
    const std::uint32_t id = static_cast<std::uint32_t>(biome);
    return id < biomeNames().size() ? biomeNames()[id].c_str() : "unknown";
}

// The three open-water climate zones (depth-banded by the classifier).
bool isWaterBiome(planet::Biome biome)
{
    return biome == planet::Biome::DeepOcean || biome == planet::Biome::Ocean
        || biome == planet::Biome::ShallowOcean;
}

std::size_t runBiomeCensus(planet::PlanetCapability &planetCapability, planet::PlanetHandle planet,
    const planet::PlanetGenerationParams &params, std::uint32_t samples)
{
    const std::vector<std::string>                 &names = biomeNames();
    std::vector<std::size_t>                        counts(names.size(), 0);
    const std::vector<planet::Vec3>                 directions = goldenSpiralDirections(samples);
    std::size_t                                     sampled   = 0;
    for (const planet::Vec3 &direction : directions)
    {
        double radius = 0.0;
        const double dirArray[3] = {direction.x, direction.y, direction.z};
        if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, dirArray, &radius))
        {
            continue;
        }
        engine::planet::ClimateSample climate{};
        if (!planetCapability.sampleClimate(planetCapability.self, planet, dirArray, radius, &climate))
        {
            continue;
        }
        engine::planet::Biome biome = climate.biome;
        // Slope-derived reclassification, same as the pin machinery: climate
        // samples always carry a flat-ground slope, so bare rock never shows
        // up in climate.biome; the relief field's cliff factor (the mesher's
        // rock-row gate) is where rock actually lives.
        planet::ReliefSample relief{};
        if (planetCapability.sampleRelief(planetCapability.self, planet, dirArray, &relief)
            && relief.cliffFactor >= params.cliffSteepnessLow)
        {
            biome = planet::Biome::Rock;
        }
        const std::size_t id = static_cast<std::size_t>(biome);
        if (id < counts.size())
        {
            counts[id] += 1;
        }
        sampled += 1;
    }
    std::size_t present = 0;
    std::string absent;
    for (std::size_t id = 0; id < names.size(); ++id)
    {
        if (counts[id] == 0)
        {
            absent += (absent.empty() ? "" : ", ") + names[id];
            continue;
        }
        present += 1;
        std::printf("demo_planet:   %-22s %8zu  %6.2f%%\n", names[id].c_str(), counts[id],
            100.0 * static_cast<double>(counts[id]) / static_cast<double>(sampled));
    }
    std::printf("demo_planet: biome census over %zu of %u golden-spiral samples: %zu of %zu climate zones present\n",
        sampled, samples, present, names.size());
    if (!absent.empty())
    {
        std::printf("demo_planet:   absent on this seed: %s\n", absent.c_str());
    }
    return present;
}

glm::dvec3 pickMacroViewDirection(planet::PlanetCapability &planetCapability, planet::PlanetHandle planet,
    const glm::dvec3 &sunDirection, double sunWeight, double sideWeight, double lift, double eyeRadii)
{
    // Space-view composition (globe and orbit): the camera keeps the
    // sun-anchored bearing (the off-sun angle that models relief and crosses
    // the terminator through the disc, the equator lift that keeps the polar
    // cap in frame) but the bearing's LONGITUDE is chosen by census: a fixed
    // longitude points at whatever hemisphere that slot carries, and a seed
    // whose water sits there photographs the planet as an ocean world with
    // no readable geography. Each candidate's visible cap (the cone the eye
    // actually sees from eyeRadii radii out) is sampled over a fixed spiral
    // and scored on the mix real planet photography reads - a landmass
    // balance around two-fifths land, biome variety, and a touch of polar
    // ice. Pure function of the climate field: no extra state, no streaming.
    const glm::dvec3 sun = glm::normalize(sunDirection);
    const glm::dvec3 up(0.0, 1.0, 0.0);
    const double capCos = std::clamp(1.0 / std::max(1.05, eyeRadii), 0.2, 0.95);

    // Cap-local sample spiral: unit directions within the cap cone around
    // +Z, rotated into each candidate's equatorial basis. 96 points hold the
    // census error under a few percent while the whole scan stays a few
    // thousand field evaluations.
    std::vector<planet::Vec3> capSamples;
    {
        const std::vector<planet::Vec3> spiral = goldenSpiralDirections(384);
        for (const planet::Vec3 &direction : spiral)
        {
            if (direction.z >= capCos)
            {
                capSamples.push_back(direction);
            }
        }
    }

    double         bestScore = -1.0;
    glm::dvec3     bestDirection{};
    std::size_t    bestLand = 0;
    std::size_t    bestBiomes = 0;
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    for (int step = 0; step < 24; ++step)
    {
        const double   longitude = kTwoPi * static_cast<double>(step) / 24.0;
        const planet::Vec3 sunLocal{sun.x, sun.y, sun.z};
        const planet::Vec3 sunRotated = rotateDirection(sunLocal,
            planet::Vec3{up.x, up.y, up.z}, -longitude);
        const glm::dvec3 sunRot = glm::normalize(glm::dvec3(sunRotated.x, sunRotated.y, sunRotated.z));
        glm::dvec3     side = glm::cross(sunRot, up);
        if (glm::length(side) < 1.0e-6)
        {
            side = glm::cross(sunRot, glm::dvec3(1.0, 0.0, 0.0));
        }
        side = glm::normalize(side);
        const glm::dvec3 direction = glm::normalize(sunRot * sunWeight + side * sideWeight + up * lift);

        // Equatorial basis of the candidate; the cap spiral's +Z rides it.
        glm::dvec3     east = glm::cross(up, direction);
        if (glm::length(east) < 1.0e-6)
        {
            east = glm::cross(glm::dvec3(0.0, 0.0, 1.0), direction);
        }
        east = glm::normalize(east);
        const glm::dvec3 north = glm::normalize(glm::cross(direction, east));

        std::size_t water = 0;
        std::size_t ice = 0;
        std::vector<bool> biomeSeen(static_cast<std::size_t>(planet::Biome::Count), false);
        std::size_t biomes = 0;
        for (const planet::Vec3 &sample : capSamples)
        {
            const glm::dvec3 world = east * static_cast<double>(sample.x)
                + north * static_cast<double>(sample.y)
                + direction * static_cast<double>(sample.z);
            const double dirArray[3] = {world.x, world.y, world.z};
            double radius = 0.0;
            if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, dirArray, &radius))
            {
                continue;
            }
            engine::planet::ClimateSample climate{};
            if (!planetCapability.sampleClimate(planetCapability.self, planet, dirArray, radius, &climate))
            {
                continue;
            }
            const planet::Biome biome = climate.biome;
            if (isWaterBiome(biome))
            {
                water += 1;
            }
            if (biome == planet::Biome::Snow || biome == planet::Biome::Tundra)
            {
                ice += 1;
            }
            const std::size_t id = static_cast<std::size_t>(biome);
            if (id < biomeSeen.size() && !biomeSeen[id])
            {
                biomeSeen[id] = true;
                biomes += 1;
            }
        }
        // Ocean sun glint: the specular mirror point of a spherical sea
        // (the surface normal that bisects the eye and the sun) lands near
        // normalize(candidate + sun). Real Earth photography almost always
        // carries the sun's mirror on open water - the bright elongated
        // glint patch is one of the strongest realism cues an orbital
        // photograph holds, and a mirror that lands on a continent shows
        // none at all. The mirror sample rides the same climate census.
        const glm::dvec3 mirrorDirection = glm::normalize(direction + sunRot);
        const double mirrorArray[3] = {mirrorDirection.x, mirrorDirection.y, mirrorDirection.z};
        double mirrorRadius = 0.0;
        bool   mirrorIsWater = false;
        if (planetCapability.sampleSurfaceRadius(planetCapability.self, planet, mirrorArray, &mirrorRadius))
        {
            engine::planet::ClimateSample mirrorClimate{};
            if (planetCapability.sampleClimate(planetCapability.self, planet, mirrorArray, mirrorRadius,
                    &mirrorClimate))
            {
                mirrorIsWater = isWaterBiome(mirrorClimate.biome);
            }
        }

        const std::size_t sampled = capSamples.size();
        if (sampled == 0)
        {
            continue;
        }
        const double landFraction = 1.0 - static_cast<double>(water) / static_cast<double>(sampled);
        const double iceFraction = static_cast<double>(ice) / static_cast<double>(sampled);
        // Land balance: peak at 0.42 land, falling to zero at 0.12/0.72 - an
        // ocean hemisphere and an unbroken supercontinent both read as
        // featureless discs.
        const double landScore = std::clamp(1.0 - std::abs(landFraction - 0.42) / 0.30, 0.0, 1.0);
        const double varietyScore = std::clamp(static_cast<double>(biomes) / 8.0, 0.0, 1.0);
        const double iceScore = std::clamp(iceFraction / 0.06, 0.0, 1.0);
        const double glintScore = mirrorIsWater ? 1.0 : 0.0;
        const double score = 0.45 * landScore + 0.25 * varietyScore + 0.10 * iceScore + 0.20 * glintScore;
        if (score > bestScore)
        {
            bestScore = score;
            bestDirection = direction;
            bestLand = sampled - water;
            bestBiomes = biomes;
        }
    }
    if (bestScore < 0.0)
    {
        // No candidate scored (the capability refused every sample): fall
        // back to the historical fixed bearing.
        bestDirection = glm::normalize(sun * sunWeight
            + glm::normalize(glm::cross(sun, up)) * sideWeight + up * lift);
    }
    std::printf("demo_planet: macro view bearing - land %.0f%% of cap, %zu climate zones (score %.2f)\n",
        100.0 * static_cast<double>(bestLand) / static_cast<double>(capSamples.size()),
        bestBiomes, bestScore);
    return bestDirection;
}

bool pickViewpoint(planet::PlanetCapability &planetCapability, planet::PlanetHandle planet,
    const std::function<bool(double elevation, engine::planet::Biome biome, double temperature01,
        const planet::PlanetGenerationParams &params)>& predicate, ProbePolicy policy, double eyeHeight,
    const planet::Vec3 &sunDirection, Viewpoint &out, const planet::Biome *neighborhoodBiome,
    std::size_t choiceIndex)
{
    planet::PlanetGenerationParams params{};
    planetCapability.copyGenerationParams(planetCapability.self, planet, &params);
    const planet::Vec3 spinAxis{params.axis[0], params.axis[1], params.axis[2]};

    // Mountain policy searches a much finer candidate set: terraced massif
    // flanks cover only a few percent of the globe, and the plain 256-point
    // spiral can miss them entirely. The coarse relief sample
    // (sampleRelief - one field evaluation, no surface march) pre-filters
    // the fine spiral down to candidates that already stand on terraced
    // mountain country; the full probe/fan machinery then runs on those.
    std::vector<planet::Vec3> candidates;
    // Pinned views (and pinned beach views riding the coastline machinery)
    // need the fine spiral: a single-biome predicate is far sparser than any
    // windowed one (deserts and bare-rock faces each cover a few percent of
    // the globe), and the coarse 256-point spiral can miss a zone entirely -
    // the pin then silently degraded to the generic surface spot and
    // photographed the wrong biome. The pre-filter matches the PINNED BIOME's
    // own surface sample (not the mountain pre-filter - that gate would
    // exclude exactly the flat desert basins and low rock terraces a pin
    // wants). The candidates list itself is only a pre-filter: the predicate
    // still runs on the full climate/elevation state, and the neighborhood
    // probes below still dominate the score.
    const bool prefilterByBiome = neighborhoodBiome != nullptr
        && (policy == ProbePolicy::Pinned || policy == ProbePolicy::Coastline);
    // The unpinned coast view is just as sparse as a biome pin: its
    // 2-45 m shoreband predicate only ever matches a sliver of the globe
    // (the beach ring itself), and no practical spiral lands inside it -
    // the 2048-point spiral's own spacing (~1.6 km on the showcase globe)
    // already exceeds the 220 m ring the near-shore fan gate reads, so the
    // coarse search silently degraded to the generic fallback spot and
    // photographed inland hills instead of a shoreline. The coast search
    // instead keeps the spiral's near-shore WATER points and marches each
    // to its waterline (see the pre-filter below); the coastline fan gates
    // then still verify real land AND water in frame.
    const bool prefilterByShore = policy == ProbePolicy::Coastline && neighborhoodBiome == nullptr;
    // The shore march's own constants: the standing elevation the marched
    // candidate climbs to (inside the coast predicate's 2-45 m window and
    // above the flattened shoreline band's near-waterline floor), how far
    // inland the climb may search, how far the compass probes search for
    // land around a near-shore water point, how deep a water point may sit
    // before marching from it stops being a shoreline search, and how many
    // marched candidates the search wants before it stops escalating the
    // spiral (a coastline's spiral footprint is thin; one pass finds a
    // handful).
    constexpr double kShoreStandMeters     = 3.0;
    constexpr double kShoreClimbMeters     = 2000.0;
    constexpr double kShoreProbeMeters     = 480.0;
    constexpr double kShoreProbeMaxMeters  = 7680.0;
    constexpr double kShoreMaxWaterDepthMeters = 400.0;
    constexpr std::size_t kShoreCandidateFloor = 24;
    if (prefilterByBiome || prefilterByShore)
    {
        std::uint32_t spiralCount = 2048;
        candidates.reserve(96);
        for (int attempt = 0; attempt < 2
            && (prefilterByShore ? candidates.size() < kShoreCandidateFloor : candidates.empty());
            ++attempt)
        {
            const std::vector<planet::Vec3> fine = goldenSpiralDirections(spiralCount);
            for (const planet::Vec3 &direction : fine)
            {
                if (candidates.size() >= 128)
                {
                    break;
                }
                double radius = 0.0;
                const double dirArray[3] = {direction.x, direction.y, direction.z};
                if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, dirArray, &radius))
                {
                    continue;
                }
                if (prefilterByShore)
                {
                    // The shoreline march: a point ON the beach is what the
                    // coastline gates need (land under foot, open water
                    // inside the near fan rings), and no practical spiral
                    // lands there - the 2048-point spiral's own spacing
                    // (~1.6 km on the showcase globe) already exceeds the
                    // 220 m ring the near-shore gate reads. So the search
                    // runs the other way around: keep the spiral's
                    // near-shore WATER points and march each one to its
                    // waterline along the first compass direction that
                    // finds land, stepping a fixed distance inland. The
                    // marched candidate then carries real water within the
                    // probe reach by construction, and the fan gates below
                    // still verify the full frame composition.
                    const double waterElevation = radius - params.derivedSeaRadiusMeters;
                    if (waterElevation < -kShoreMaxWaterDepthMeters || waterElevation > -1.0)
                    {
                        continue; // open ocean (too far to march) or already land
                    }
                    planet::Vec3 shoreTangent = planet::cross(spinAxis, direction);
                    double       shoreTangentLength = planet::length(shoreTangent);
                    if (shoreTangentLength < 1.0e-6)
                    {
                        shoreTangent       = planet::cross(direction, planet::Vec3{1.0, 0.0, 0.0});
                        shoreTangentLength = planet::length(shoreTangent);
                    }
                    if (shoreTangentLength < 1.0e-6)
                    {
                        continue;
                    }
                    shoreTangent = shoreTangent * (1.0 / shoreTangentLength);
                    const planet::Vec3 shoreBitangent = planet::normalized(planet::cross(direction, shoreTangent));
                    // The land bearing search: the coastal flattening leaves
                    // near-shore shelves nearly level (a few meters of fall
                    // over a kilometer), so a fixed probe reach misses the
                    // waterline on every gentle shore. Each compass bearing
                    // expands its probe outward (kShoreProbeMeters, doubling)
                    // until it stands on dry land or leaves the search
                    // reach; the steepest landward bearing wins the march.
                    std::size_t         landBearing        = 8;
                    planet::Vec3        landProbe{};
                    double              landProbeElevation = -1.0e30;
                    for (std::size_t compass = 0; compass < 8; ++compass)
                    {
                        const double angle = static_cast<double>(compass) * (3.14159265358979323846 / 4.0);
                        const planet::Vec3 probeAxis
                            = shoreTangent * std::cos(angle) + shoreBitangent * std::sin(angle);
                        for (double probeDistance = kShoreProbeMeters; probeDistance <= kShoreProbeMaxMeters;
                             probeDistance *= 2.0)
                        {
                            const planet::Vec3 probe
                                = rotateDirection(direction, probeAxis, probeDistance / params.radiusMeters);
                            double probeRadius = 0.0;
                            const double probeArray[3] = {probe.x, probe.y, probe.z};
                            if (!planetCapability.sampleSurfaceRadius(
                                    planetCapability.self, planet, probeArray, &probeRadius))
                            {
                                break;
                            }
                            const double probeElevation = probeRadius - params.derivedSeaRadiusMeters;
                            if (probeElevation <= 0.0)
                            {
                                continue; // still water: push the probe outward
                            }
                            if (probeElevation > landProbeElevation)
                            {
                                landBearing        = compass;
                                landProbe          = probe;
                                landProbeElevation = probeElevation;
                            }
                            break; // dry land on this bearing: stop expanding
                        }
                    }
                    bool marched = false;
                    if (landBearing < 8)
                    {
                        // The winning bearing's tangent axis (the march
                        // rotates around it, toward the land it found).
                        const double landAngle = static_cast<double>(landBearing) * (3.14159265358979323846 / 4.0);
                        const planet::Vec3 probeAxis
                            = shoreTangent * std::cos(landAngle) + shoreBitangent * std::sin(landAngle);
                        // Bisect the waterline between the water point and
                        // the land probe (the span can run kilometers on a
                        // gentle shelf; the halvings resolve it to ~30 m),
                        // then climb inland from the crossing.
                        planet::Vec3 waterSide = direction;
                        planet::Vec3 landSide  = landProbe;
                        for (std::size_t step = 0; step < 9; ++step)
                        {
                            const planet::Vec3 middle = planet::normalized(planet::Vec3{
                                0.5 * (waterSide.x + landSide.x), 0.5 * (waterSide.y + landSide.y),
                                0.5 * (waterSide.z + landSide.z)});
                            double middleRadius = 0.0;
                            const double middleArray[3] = {middle.x, middle.y, middle.z};
                            if (!planetCapability.sampleSurfaceRadius(
                                    planetCapability.self, planet, middleArray, &middleRadius))
                            {
                                break;
                            }
                            if (middleRadius > params.derivedSeaRadiusMeters)
                            {
                                landSide = middle;
                            }
                            else
                            {
                                waterSide = middle;
                            }
                        }
                        // From the waterline, climb inland to the shore
                        // march's target standing elevation: the coastal
                        // flattening damps the shoreline band's detail to a
                        // few percent, so a FIXED step lands within a meter
                        // of the waterline and the predicate's own window
                        // rejects it. The climb bisects between the
                        // waterline and a probe kShoreClimbMeters inland;
                        // a shore too flat to reach the target inside that
                        // reach is left to the fan gates of some steeper
                        // candidate (a beach below the window renders as a
                        // sandbar, not a shore).
                        planet::Vec3 inlandBase = rotateDirection(
                            waterSide, probeAxis, kShoreClimbMeters / params.radiusMeters);
                        double inlandBaseRadius = 0.0;
                        const double inlandBaseArray[3] = {inlandBase.x, inlandBase.y, inlandBase.z};
                        if (!planetCapability.sampleSurfaceRadius(
                                planetCapability.self, planet, inlandBaseArray, &inlandBaseRadius)
                            || inlandBaseRadius - params.derivedSeaRadiusMeters < kShoreStandMeters)
                        {
                            continue; // too flat to stand on inside the climb reach
                        }
                        planet::Vec3 lowSide  = waterSide;   // at the waterline: elevation ~0
                        planet::Vec3 highSide = inlandBase;  // at/above the target elevation
                        for (std::size_t step = 0; step < 6; ++step)
                        {
                            const planet::Vec3 middle = planet::normalized(planet::Vec3{
                                0.5 * (lowSide.x + highSide.x), 0.5 * (lowSide.y + highSide.y),
                                0.5 * (lowSide.z + highSide.z)});
                            double middleRadius = 0.0;
                            const double middleArray[3] = {middle.x, middle.y, middle.z};
                            if (!planetCapability.sampleSurfaceRadius(
                                    planetCapability.self, planet, middleArray, &middleRadius))
                            {
                                break;
                            }
                            if (middleRadius - params.derivedSeaRadiusMeters >= kShoreStandMeters)
                            {
                                highSide = middle;
                            }
                            else
                            {
                                lowSide = middle;
                            }
                        }
                        const planet::Vec3 inland = highSide;
                        // Two water points can march to the same beach: keep
                        // the first, drop duplicates within a beach-length of
                        // an existing candidate.
                        bool duplicate = false;
                        for (const planet::Vec3 &existing : candidates)
                        {
                            if (planet::dot(existing, inland) > 1.0 - 1.0e-10)
                            {
                                duplicate = true;
                                break;
                            }
                        }
                        if (!duplicate)
                        {
                            candidates.push_back(inland);
                            marched = true;
                        }
                    }
                    continue;
                }
                engine::planet::ClimateSample climate{};
                if (!planetCapability.sampleClimate(planetCapability.self, planet, dirArray, radius, &climate))
                {
                    continue;
                }
                bool keep = climate.biome == *neighborhoodBiome;
                if (!keep && *neighborhoodBiome == planet::Biome::Rock)
                {
                    // Rock is slope-only through the capability (climate samples
                    // always carry a flat-ground slope), so the pre-filter probes
                    // the relief field's cliff factor - the same steepness gate
                    // the mesher's rock row keys on.
                    planet::ReliefSample relief{};
                    if (planetCapability.sampleRelief(planetCapability.self, planet, dirArray, &relief))
                    {
                        // cliffFactor is the terraced-country mask (saturating at
                        // cliffSteepnessHigh, opening at cliffSteepnessLow) - NOT
                        // the slope threshold the classifier uses; a face the
                        // mask calls terraced IS the bare-rock face the mesher's
                        // rock row paints on its steep risers.
                        keep = relief.cliffFactor >= params.cliffSteepnessLow;
                    }
                }
                if (keep)
                {
                    candidates.push_back(direction);
                }
            }
            // A zone can exist only in pockets smaller than the spiral's
            // footprint: escalate once to an 8x denser spiral before calling
            // the biome absent (the failure log then distinguishes "rare"
            // from "absent").
            const bool escalate = prefilterByShore
                ? candidates.size() < kShoreCandidateFloor
                : candidates.empty();
            if (escalate && attempt == 0)
            {
                if (prefilterByShore)
                {
                    std::cout << "demo_planet: the shoreline march produced " << candidates.size()
                              << " candidates on the " << spiralCount
                              << "-point spiral; escalating to " << (spiralCount * 8) << " points\n";
                }
                else
                {
                    std::cout << "demo_planet: biome '" << biomeName(*neighborhoodBiome)
                              << "' produced no candidates on the " << spiralCount
                              << "-point spiral; escalating to " << (spiralCount * 8) << " points\n";
                }
                spiralCount *= 8;
            }
        }
    }
    else if (policy == ProbePolicy::Mountain)
    {
        const std::vector<planet::Vec3> fine = goldenSpiralDirections(2048);
        candidates.reserve(96);
        for (const planet::Vec3 &direction : fine)
        {
            if (candidates.size() >= 96)
            {
                break;
            }
            planet::ReliefSample relief{};
            const double dirArray[3] = {direction.x, direction.y, direction.z};
            if (!planetCapability.sampleRelief(planetCapability.self, planet, dirArray, &relief))
            {
                continue;
            }
            if (relief.cliffFactor < 0.45 || relief.mountainFactor < 0.40)
            {
                continue;
            }
            const double elevation = params.radiusMeters + relief.heightMeters - params.derivedSeaRadiusMeters;
            if (elevation < 150.0 || elevation > 1200.0)
            {
                continue;
            }
            candidates.push_back(direction);
        }
    }
    else
    {
        candidates = goldenSpiralDirections(256);
    }

    std::size_t probesTotal = 0;
    bool        haveBest    = false;
    double      bestScore   = 0.0;
    double      bestElevation   = 0.0;
    double      bestTemperature = 0.0;
    int         bestBiome       = 0;
    // Runner-up (choiceIndex > 0): the second-strongest candidate at least
    // kRunnerSeparationDot apart on the globe. Substitution pins reuse it so
    // a substituted sample photographs a DIFFERENT pocket of the same zone
    // instead of duplicating that zone's own pin pixel-for-pixel.
    Viewpoint   runner{};
    double      runnerScore = 0.0;
    double      runnerElevation = 0.0;
    double      runnerTemperature = 0.0;
    int         runnerBiome = 0;
    bool        haveRunner = false;
    static constexpr double kRunnerSeparationDot = 0.995; // ~0.1 rad apart - the arc distance
    // scales with the planet radius (~7.6 km on a 60 km globe, ~640 km on Earth's)
    // Diagnostics: the strongest candidate that FAILED the sight gate (its
    // probe quality and the gate that rejected it), printed when the pick
    // fails so tuning sees what rejected the best spots.
    double      bestFailQuality   = 0.0;
    double      bestFailElevation = 0.0;
    const char *bestFailReason    = "none";
    planet::Vec3 bestFailDirection{};
    planet::Vec3 bestFailFanTangents[5]{};
    double      bestFailRises[35] = {};
    bool        bestFailValid     = false;
    Viewpoint   best;

    for (const planet::Vec3 &direction : candidates)
    {
        // Day-side gate: the module's atmosphere tints the sky by the sun's
        // elevation above the spot's local horizon, so a viewpoint on the
        // planet's night side renders a night sky over terrain the fixed
        // global sun still lights (the renderer carries no terminator).
        // Keep candidates where the sun stands at least ~15 deg over their
        // local horizon; the remaining hemispheres hold plenty of spots.
        if (sunDirection.x != 0.0 || sunDirection.y != 0.0 || sunDirection.z != 0.0)
        {
            const planet::Vec3 sunUnit = planet::normalized(sunDirection);
            if (planet::dot(sunUnit, direction) < 0.25)
            {
                continue;
            }
        }
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
        // Slope-derived biomes (bare rock) need the terrain's own steepness:
        // the capability samples climate with a flat-ground slope, so a spot
        // on a cliff face reports the biome of the flats at its feet. The
        // relief sample's cliff factor IS the terrain's steepness gate (the
        // same field the mesher's rock row keys on), so re-classify from it
        // and re-run the predicate so the pin gates see the honest biome.
        {
            planet::ReliefSample spotRelief{};
            const double spotDirArray[3] = {direction.x, direction.y, direction.z};
            if (planetCapability.sampleRelief(planetCapability.self, planet, spotDirArray, &spotRelief)
                && spotRelief.cliffFactor >= params.cliffSteepnessLow)
            {
                biome = planet::Biome::Rock;
                if (!predicate(elevation, biome, temperature01, params))
                {
                    continue;
                }
            }
        }
        if (!predicate(elevation, biome, temperature01, params))
        {
            continue;
        }

        // Mountain policy: the SPOT itself must stand on terraced MOUNTAIN
        // country - the cliff mask rides the terrain's own steepness, so a
        // high mask puts quantized terrace steps, strata bands and overhang
        // ledges under and around the camera, and the mountain gate keeps
        // the relief amplitude that crosses several terrace steps (hill
        // country spans about one step, which quantization can only flatten
        // into benches). That gate - not a distant bulk wall - is what makes
        // the view show cliff country: planet-scale mountain flanks average
        // ~0.3 m/m of slope, so 35-degree walls at the probe scale are
        // vanishingly rare and wall-gated picks fell back to meadows.
        if (policy == ProbePolicy::Mountain)
        {
            planet::ReliefSample spotRelief{};
            const double spotDirArray[3] = {direction.x, direction.y, direction.z};
            if (!planetCapability.sampleRelief(planetCapability.self, planet, spotDirArray, &spotRelief)
                || spotRelief.cliffFactor < 0.45 || spotRelief.mountainFactor < 0.40)
            {
                continue;
            }
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

        // Mountain policy rings pull in to a quarter-kilometer first hop:
        // the cliff view's showcase is the terrace RISER and the strata beds
        // riding it, and the angular LOD rule only resolves a step's wall
        // and its banding well inside a few hundred meters - a wall the
        // 420 m rings locate meshes coarse enough that its steps smooth
        // into ramps and its beds average out. The nearer rings also gate
        // the near-ground biome on what actually sits beside the camera.
        const double flatRingDistances[3] = {420.0, 840.0, 1440.0}; // meters
        const double mountainRingDistances[3] = {240.0, 420.0, 840.0}; // meters
        const double *ringDistances = policy == ProbePolicy::Mountain ? mountainRingDistances : flatRingDistances;
        const std::size_t ringCount  = policy == ProbePolicy::Coastline ? 1u : 3u;
        std::size_t probesOk = 0;
        std::size_t probes   = 0;
        // Rock pins count steep hops (cliff-masked or high mountain
        // country): the score below prefers canyons and terrace systems over
        // gentle semi-arid hills with a passing cliff factor, while keeping
        // the weak spots eligible (a hard hop gate would strand seeds whose
        // warm rock country is all low-relief).
        std::size_t steepHops = 0;
        std::size_t aridHops   = 0;
        std::vector<double> probeElevations;
        probeElevations.reserve(24);
        // Cliff-wall bookkeeping (Mountain policy): the strongest terraced
        // rise found in ANY compass direction around the spot. The wall is
        // the showcase of the cliffs view, and searching the full compass
        // rose (not just the five camera yaws) finds walls the fan would
        // have missed; the camera then aims at the stored axis. The rings
        // walk OUTWARD, and a wall found on a NEARER ring wins outright: the
        // angular LOD rule only resolves terrace risers and strata bands at
        // a few hundred meters, so a bigger but farther face would render as
        // a smooth giant massif while a nearer qualifying wall keeps its
        // benches and beds.
        double      bestWallScore = -1.0;
        std::size_t bestWallRing  = 3;
        planet::Vec3 cliffWallTangent{};
        bool        hasCliffWall  = false;
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
                if (policy == ProbePolicy::Pinned)
                {
                    // Biome-pinned sample views: the spot's biome identity is
                    // the predicate's own gate, and the ring probes score how
                    // deeply the spot sits INSIDE the pinned zone (a fringe
                    // spot's ring is mostly the neighboring biome - it
                    // photographs as that neighbor with a few stray pins).
                    // Water views pass no neighborhood biome: every hop is
                    // open sea and all count. Bare-rock faces are the one
                    // zone that can never own a whole ring: the biome exists
                    // only on steep faces, whose 420+ m ring hops land on the
                    // terraces and slopes around them, so rock accepts the
                    // cold high-country neighbors a cliff stands in.
                    if (neighborhoodBiome == nullptr)
                    {
                        probeOk = true;
                    }
                    else if (*neighborhoodBiome == planet::Biome::Rock)
                    {
                        // A cliff face's ring hops land on the terraces, scree
                        // and slopes of the massif it belongs to - the rock
                        // country the sample photographs. Bare-rock faces
                        // cover only a few percent of any globe, so no ring
                        // is ever uniformly rock; accept every hop that
                        // sampled at all (the spot predicate and the
                        // cliff-mask pre-filter already guarantee the spot
                        // itself is terraced mountain country).
                        probeOk = true;
                        // Steep-hop bookkeeping: hops that land on
                        // cliff-masked faces or real mountain country mark
                        // the spot as sitting inside a canyon/terrace system
                        // rather than beside one lone bluff.
                        engine::planet::ReliefSample hopRelief{};
                        if (planetCapability.sampleRelief(planetCapability.self, planet, hopArray, &hopRelief)
                            && (hopRelief.cliffFactor >= params.cliffSteepnessLow
                                || hopRelief.mountainFactor >= 0.40))
                        {
                            ++steepHops;
                        }
                        // Arid-hop bookkeeping: rock reads strongest as
                        // canyon/strata country in dry belts (desert and
                        // savanna surroundings); green farmland around a
                        // cliff reads as an outcrop in a meadow. The score
                        // prefers arid settings when the seed offers them.
                        if (hopClimate.biome == planet::Biome::Desert
                            || hopClimate.biome == planet::Biome::Savanna)
                        {
                            ++aridHops;
                        }
                    }
                    else
                    {
                        probeOk = hopClimate.biome == *neighborhoodBiome;
                    }
                }
                else if (policy == ProbePolicy::Coastline)
                {
                    if (neighborhoodBiome != nullptr && isWaterBiome(*neighborhoodBiome))
                    {
                        // Pinned water sample (deep_ocean / ocean /
                        // shallow_ocean): the pin's own depth window chose
                        // the zone, every hop is open sea, and the odd islet
                        // in a ring is a bonus, not a disqualifier.
                        probeOk = true;
                    }
                    else if (neighborhoodBiome != nullptr)
                    {
                        // Pinned coastline zone (beach): sea probes pass, the
                        // pinned zone's own hops pass (beach-to-beach), and
                        // the green hinterland behind the shoreline passes.
                        probeOk = hopElevation < 0.0
                            || hopClimate.biome == *neighborhoodBiome
                            || (hopClimate.temperature01 > 0.32 && isGreenBiome(hopClimate.biome));
                    }
                    else
                    {
                        // The coast view wants water in frame: sea probes
                        // pass, and land probes pass when the shore country
                        // is warm - the flattened shoreline band the march
                        // lands in spans beach, savanna and desert shores
                        // on this generator, and any of them photographs as
                        // a coast (only snowfields and tundra wash the
                        // frame out). The temperature gate is low: shoreline
                        // air is cooled by the lapse rate whenever the beach
                        // sits on high ground.
                        probeOk = hopElevation < 0.0
                            || (hopClimate.biome != engine::planet::Biome::Snow
                                && hopClimate.biome != engine::planet::Biome::Tundra
                                && hopClimate.temperature01 > 0.32);
                    }
                }
                else if (policy == ProbePolicy::Mountain)
                {
                    // The cliffs view stands IN terraced mountain country,
                    // which the lapse rate keeps cold: taiga and tundra air
                    // is the honest climate there, and the picker's
                    // snow-blend window (not this gate) keeps white slopes
                    // out of the frame. Low probes (below 500 m) must be
                    // traversable land - grass, taiga, tundra and BARE ROCK
                    // all qualify; only actual snowfields, deserts and
                    // beaches would wash the frame.
                    probeOk = hopElevation > 500.0
                        || (hopClimate.biome != engine::planet::Biome::Snow
                            && hopClimate.biome != engine::planet::Biome::Desert
                            && hopClimate.biome != engine::planet::Biome::Beach);
                    // Wall detection rides the same probes: a TERRACED rise
                    // (the cliff mask at the probe is high, so the flank
                    // carries terracing/strata) that is TALL (>= 90 m) and
                    // steep at bulk scale (>= 0.45 m/m - the quantized
                    // terrace risers supply the near-vertical faces).
                    const double hopRise  = hopRadius - radius;
                    const double hopSlope  = hopRise / ringDistances[ring];
                    if (hopRise >= 90.0 && hopSlope >= 0.45)
                    {
                        planet::ReliefSample hopRelief{};
                        const double hopDirArray[3] = {hopDirection.x, hopDirection.y, hopDirection.z};
                        if (planetCapability.sampleRelief(planetCapability.self, planet, hopDirArray, &hopRelief)
                            && hopRelief.cliffFactor >= 0.5)
                        {
                            const double wallScore = hopRise * hopRelief.cliffFactor;
                            const bool nearerRing = !hasCliffWall || ring < bestWallRing;
                            if (nearerRing || (ring == bestWallRing && wallScore > bestWallScore))
                            {
                                bestWallScore   = wallScore;
                                bestWallRing    = ring;
                                cliffWallTangent = probeAxis;
                                hasCliffWall    = true;
                            }
                        }
                    }
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
        const bool sunValid = (sunDirection.x != 0.0 || sunDirection.y != 0.0 || sunDirection.z != 0.0);
        planet::Vec3 waterwardTangent{};
        bool hasWaterward = false;
        if (policy == ProbePolicy::Coastline)
        {
            // The waterward scan starts just past the coast view's look
            // distance (320 m: the shoreline the near-shore view shows) and
            // doubles outward on flat shelves - this generator's coastal
            // flattening routinely leaves the waterline a kilometer or more
            // from dry ground, and a fixed 320 m probe then mistook the
            // gently falling shelf for open water. A probe that lands below
            // the sea radius IS the water; among water bearings the nearest
            // one wins, and only when no bearing finds water does the
            // lowest-radius (steepest fall) bearing stand in.
            double lowestRadius  = 1.0e30;
            double highestRadius = -1.0e30;
            double nearestWaterMeters = 1.0e30;
            planet::Vec3 seaward;
            planet::Vec3 landward;
            for (std::size_t compass = 0; compass < 8; ++compass)
            {
                const double angle    = static_cast<double>(compass) * (3.14159265358979323846 / 4.0);
                const planet::Vec3 probeAxis = tangent * std::cos(angle) + bitangent * std::sin(angle);
                for (double probeDistance = 320.0; probeDistance <= 5120.0; probeDistance *= 2.0)
                {
                    const planet::Vec3 sightDirection
                        = rotateDirection(direction, probeAxis, probeDistance / params.radiusMeters);
                    double sightRadius = 0.0;
                    const double sightArray[3] = {sightDirection.x, sightDirection.y, sightDirection.z};
                    if (!planetCapability.sampleSurfaceRadius(planetCapability.self, planet, sightArray, &sightRadius))
                    {
                        break;
                    }
                    if (sightRadius < params.derivedSeaRadiusMeters)
                    {
                        // Open water on this bearing at this distance: the
                        // nearest water bearing wins the scan.
                        if (probeDistance < nearestWaterMeters)
                        {
                            nearestWaterMeters = probeDistance;
                            seaward            = sightDirection;
                        }
                        break;
                    }
                    if (sightRadius < lowestRadius)
                    {
                        lowestRadius = sightRadius;
                        if (nearestWaterMeters >= 1.0e30)
                        {
                            seaward = sightDirection; // steepest seaward fall so far
                        }
                    }
                    if (sightRadius > highestRadius)
                    {
                        highestRadius = sightRadius;
                        landward      = sightDirection;
                    }
                }
            }
            if (lowestRadius < 1.0e30)
            {
                // Normalize each projection to a unit tangent first; the
                // diagonal of the two unit tangents is the along-shore
                // direction. When the pair is near-opposite (a straight
                // coastline: the seaward and landward probes point against
                // each other), the along-shore direction is the tangent
                // PERPENDICULAR to the seaward axis - falling back to the
                // raw seaward tangent aimed the camera straight out to open
                // ocean (an all-water "coast" frame with land behind the
                // camera).
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
                planet::Vec3 alongShore = diagonalLength > 0.5 ? diagonal * (1.0 / diagonalLength)
                    : (seawardTLength > 1.0e-9 ? planet::normalized(planet::cross(radial, seawardTangent))
                                                : planet::Vec3{});
                // Aim bias: the along-shore diagonal aims the CENTER of the
                // frame at the waterline. On bent shores the land then ends
                // up past the frame edge (the render reads as open ocean),
                // so a NEAR shoreline rotates a quarter radian toward the
                // land - the water keeps its frame share while real coast
                // stays in view. A FAR shoreline (the flat shelves again:
                // water past the sight fan's near rings) rotates the other
                // way, toward the water, so the waterline crosses the frame
                // at all instead of sitting outside the view's edge.
                if (planet::length(alongShore) > 1.0e-9)
                {
                    const bool nearShore = nearestWaterMeters <= 640.0;
                    const double biasRadians = nearShore ? 0.25 : -0.35;
                    const planet::Vec3 &biasTangent = nearShore ? landwardTangent : seawardTangent;
                    if (landwardTLength > 1.0e-9 || !nearShore)
                    {
                        alongShore = planet::normalized(
                            alongShore * std::cos(biasRadians) + biasTangent * std::sin(biasRadians));
                    }
                }
                waterwardTangent = alongShore;
                hasWaterward     = planet::length(alongShore) > 1.0e-9;
                // Glint bias: rotate the along-shore tangent a quarter of
                // the way toward the sun's horizon azimuth. The sun's
                // specular wedge on the water centers on the sun's azimuth
                // (the slopes of the wave field mirror the sun to an eye
                // looking roughly sunward over the surface), and a pure
                // along-shore look leaves that wedge at or beyond the
                // frame's edge - the sea reads as a flat sheet. The rotation
                // keeps the shoreline crossing the frame while the glint
                // wedge enters over the open water.
                if (hasWaterward && sunValid)
                {
                    planet::Vec3 sunTangent = sunDirection
                        - direction * planet::dot(sunDirection, direction);
                    const double sunTangentLength = planet::length(sunTangent);
                    if (sunTangentLength > 1.0e-6)
                    {
                        sunTangent = sunTangent * (1.0 / sunTangentLength);
                        const planet::Vec3 glintward = planet::normalized(
                            alongShore + sunTangent * 0.9);
                        if (planet::length(glintward) > 1.0e-9)
                        {
                            waterwardTangent = glintward;
                        }
                    }
                }
            }
        }

        // Sun-preference yaw choice: among the five sight yaws, ground views
        // look CROSS-LIT - the sun ~70 degrees off the view axis, at or just
        // outside the frame's edge. The old criterion (sun fully behind the
        // camera) front-lit the whole frame: every slope faced the camera
        // together, NdotL read nearly constant across the view, and the
        // render flattened into mush with no readable light direction. Cross
        // light turns the near slopes toward the camera and the far ones away
        // (the directional shading the eye reads first), and the sun's
        // azimuth still sits close enough to the frame edge that the sky's
        // glare wedge and the warm horizon read on the sun's side. Coast
        // views keep their waterward tangent (the water must enter the
        // frame); the sun falls where the glint rotation below puts it.
        double bestYawError = 1.0e30;
        double bestYawAlignment = 0.0;
        std::size_t bestYaw = 2;
        if (policy != ProbePolicy::Coastline && sunValid)
        {
            planet::Vec3 sunTangential = sunDirection - direction * planet::dot(sunDirection, direction);
            const double sunTangentialLength = planet::length(sunTangential);
            if (sunTangentialLength > 1.0e-6)
            {
                sunTangential = sunTangential * (1.0 / sunTangentialLength);
                // Target alignment: cos(72 deg) = 0.30 keeps the sun
                // cross-lit at (or a hair beyond) the frame's edge while the
                // slopes that face it read strongly lit against the ones
                // that face away.
                constexpr double kSunCrossLightAlignment = 0.30;
                for (std::size_t yaw = 0; yaw < 5; ++yaw)
                {
                    const double alignment = planet::dot(yawedTangents[yaw], sunTangential);
                    const double error = std::abs(alignment - kSunCrossLightAlignment);
                    if (error < bestYawError)
                    {
                        bestYawError = error;
                        bestYawAlignment = alignment;
                        bestYaw = yaw;
                    }
                }
            }
        }

        // The tangent the camera will actually use; the sight fan below
        // re-yaws it across the field of view. Cliff views with a located
        // wall aim AT the wall (the override at the best-tracking below),
        // so the fan must verify the WALL tangent, not the sun-preference
        // yaw: verifying a tangent the camera then discards let cliff views
        // aim across unverified ground (a below-datum basin with a distant
        // far shore reads as a coast view, not a cliff view).
        const planet::Vec3 sightLookTangent = hasCliffWall ? cliffWallTangent
            : (hasWaterward ? waterwardTangent : yawedTangents[bestYaw]);
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
        // Coastline fans carry one ring beyond the other views: the shore
        // march may place the camera up to its climb reach from the
        // waterline on the flattest shelves, and the water gates need a
        // probe that reaches past it (a coast whose water never enters the
        // fan fails "no water in sight fan" and the whole view falls back
        // to an inland spot).
        const double sightRingDistances[7] = {60.0, 120.0, 220.0, 420.0, 720.0, 1440.0, 2880.0}; // meters
        const std::size_t kSightRingCount = policy == ProbePolicy::Coastline ? 7u : 6u;
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
        // Mountain policy: the compass-rose wall search above already
        // located the strongest terraced rise; that wall IS the
        // cliff-ahead signal (the camera aims at it below).
        cliffAhead = hasCliffWall;
        // Pinned water sample (deep_ocean / ocean / shallow_ocean): open
        // water IS the subject. The coast gates below demand land AND water
        // in the fan - right for a coast view, wrong for a deep-ocean pin
        // (every candidate fails "no land in sight fan" and the pin silently
        // degrades to the generic fallback spot). Water pins keep the wall
        // caps (they still frame a readable horizon) but skip the
        // composition gates.
        const bool waterPin = policy == ProbePolicy::Coastline && neighborhoodBiome != nullptr
            && isWaterBiome(*neighborhoodBiome);
        // Water-pin land fraction: an ocean-band sample photographs open
        // water, not a coastline; a shallow-band sample is nearshore by
        // nature. Scoring the fraction of fan probes that hit land lets the
        // best-of search put the ocean-band pin offshore while the shallow
        // band keeps its shelf framing (adjacent depth bands would otherwise
        // pick near-identical shoreline spots and duplicate the sample).
        std::size_t landFanProbes  = 0;
        std::size_t totalFanProbes = 0;
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
                static constexpr double flatCapScale[7]   = {0.45, 0.6, 1.2, 2.0, 3.0, 4.0, 6.0};
                // Cliff-view caps tolerate whole terrace steps in the near
                // fan: the quantized risers ARE the showcase, and a step is
                // ~200 m tall on the showcase planet. The UP side is
                // generous; the DOWN side keeps the flat caps: a terrace
                // bench may drop one step below the eye, but an arbitrarily
                // deep descent (a valley basin, a below-datum sea) would aim
                // the cliffs view across open water at a distant far shore,
                // which is a coast view, not a cliff view.
                static constexpr double cliffCapScale[7]  = {1.5, 2.2, 3.2, 5.0, 7.0, 10.0, 14.0};
                static constexpr double slopeTolerance    = 0.45; // meters of ground rise per meter of distance
                const bool cliffView = policy == ProbePolicy::Mountain;
                const double upCapScale = cliffView ? cliffCapScale[ring] : flatCapScale[ring];
                const double downCapScale = flatCapScale[ring];
                const double upCap = upCapScale * eyeHeight + slopeTolerance * sightRingDistances[ring];
                const double downCap = downCapScale * eyeHeight + slopeTolerance * sightRingDistances[ring];
                if (rise > upCap || rise < -downCap)
                {
                    sightClear = false;
                    failReason = "wall cap";
                    break;
                }
                const double sightElevation = sightRadius - params.derivedSeaRadiusMeters;
                if (waterPin)
                {
                    ++totalFanProbes;
                    if (sightElevation > 2.0)
                    {
                        ++landFanProbes;
                    }
                }
                if (policy == ProbePolicy::Coastline)
                {
                    // Land/water flags key on the probe's ABSOLUTE elevation
                    // against the sea radius (a rise measured from the spot
                    // would call every depression beside a mountain spot
                    // "water" even when it is dry inland land far above the
                    // sea, and the fallback picks would render hillsides).
                    if (sightElevation > 2.0)
                    {
                        sawLand = true;
                        // Near-ring land means the shoreline is CLOSE - the
                        // frame will hold actual beach rather than open water
                        // with the coast somewhere behind the camera.
                        if (ring <= 2 && nearestLandRing > ring)
                        {
                            nearestLandRing = ring;
                        }
                        if (ring <= 2)
                        {
                            sawLandNear = true;
                        }
                    }
                    else if (sightElevation < -0.25)
                    {
                        // Water is water at any depth: the water sheet
                        // renders wherever terrain dips below the sea
                        // radius, and this generator's flattened shelves
                        // slope so gently that their first kilometer of
                        // open water sits within a couple of meters of the
                        // datum - a wide deadband here classifies real sea
                        // as "neither" and the coastline gates never see
                        // it. The land side keeps its 2 m floor (the beach
                        // band is land even at a meter up).
                        sawWater = true;
                        if (ring <= 2)
                        {
                            sawWaterNear = true;
                        }
                    }
                }
                if (rise >= 60.0 && policy != ProbePolicy::Mountain)
                {
                    // Ground views (not the cliffs view) just prefer some
                    // relief in the frame; the cliffs wall comes from the
                    // compass-rose search above.
                    cliffAhead = true;
                }
            }
        }
        if (policy == ProbePolicy::Coastline && !waterPin && !(sawLand && sawWater))
        {
            // No shoreline inside the frame's whole sight fan: this is open
            // ocean or deep inland, not a coast view.
            sightClear = false;
            failReason = sawLand ? "no water in sight fan" : (sawWater ? "no land in sight fan" : "fan all level (no land, no water)");
        }
        // A shoreline beyond the near rings still reads as a coast from a
        // 120 m eye on the flattened shelves this generator produces (the
        // water enters the frame across the midground instead of the near
        // field), so a far shoreline stays eligible and the score below
        // ranks it behind any closer-water candidate. The islet trap is the
        // opposite failure (no LAND near) and keeps its hard gate.
        const bool farShoreline = policy == ProbePolicy::Coastline && !waterPin && sightClear && !sawWaterNear;
        if (policy == ProbePolicy::Coastline && !waterPin && sightClear && !sawLandNear)
        {
            // No land within 220 m on any fan yaw: the spot is open water
            // or a skerry smaller than the near rings (the islet trap).
            sightClear = false;
            failReason = "no land near (islet)";
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
        // Pinned samples score on the neighborhood fraction ALONE: the relief
        // bonus exists for showcase landscape views (a perfectly flat plain
        // photographs as an empty sheet), but a climate-zone sample's subject
        // is the zone itself - flat taiga bogs and low forest shelves are the
        // HONEST sample for those climates, and the relief factor was pushing
        // exactly those pins under the pass threshold.
        const double probeFraction = static_cast<double>(probesOk) / static_cast<double>(probes);

        double score = sightClear ? probeFraction * (policy == ProbePolicy::Pinned ? 1.0 : reliefBonus) : 0.0;
        if (waterPin && totalFanProbes > 0 && sightClear)
        {
            // Water-pin land avoidance: the ocean band should photograph open
            // water, and adjacent depth bands (ocean vs shallow_ocean) hug the
            // same shorelines - without this term both bands' best-of picks
            // land on the same coast and the two samples duplicate each other.
            // A 60% penalty at all-land keeps any nearshore spot eligible
            // (pathological all-coast seeds) while making the offshore
            // candidate win whenever one exists.
            score *= 1.0 - 0.6 * (static_cast<double>(landFanProbes) / static_cast<double>(totalFanProbes));
        }
        if (neighborhoodBiome != nullptr && *neighborhoodBiome == planet::Biome::Rock && probes > 0 && sightClear)
        {
            // Rock-pin terracing preference: a warm cliff spot whose ring is
            // full of cliff-masked or mountainous hops sits inside a real
            // canyon/terrace system (0.75x .. 2.0x), while a lone bluff on
            // gentle hills keeps a passing score. The best-of search then
            // lands the rock sample on the strongest strata country the warm
            // belt offers instead of the first qualifying slope.
            score *= 0.75 + 1.25 * (static_cast<double>(steepHops) / static_cast<double>(probes));
            // Rock-pin arid preference: dry surroundings (desert/savanna
            // country) let the strata walls read as canyon country; the
            // term stays a preference so seeds with only green rock country
            // still place a pin.
            score *= 0.85 + 0.90 * (static_cast<double>(aridHops) / static_cast<double>(probes));
        }
        const double probeQuality = probeFraction * reliefBonus;
        if (!sightClear && probeQuality > bestFailQuality)
        {
            bestFailQuality   = probeQuality;
            bestFailElevation = elevation;
            bestFailReason    = failReason;
            bestFailDirection = direction;
            bestFailValid     = true;
            for (std::size_t ring = 0; ring < kSightRingCount; ++ring)
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
            // No wall in the compass rose: the spot still sits on terraced
            // country (the gate above), so keep it eligible with a mild
            // penalty - a wall ahead is a bonus, not the only way to show
            // cliffs.
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
        if (policy == ProbePolicy::Coastline && nearestLandRing > 2)
        {
            // The shoreline (if any) is beyond 220 m: the frame is mostly
            // open water with the coast behind the camera. Keep the spot
            // eligible but let closer-shore candidates win.
            score *= 0.85;
        }
        if (farShoreline)
        {
            // Water beyond the near rings (see farShoreline above): a
            // midground shore loses to a near-shore candidate at equal
            // quality but stays in the running on gentle-shelf seeds.
            score *= 0.85;
        }
        if (bestYawAlignment > 0.75)
        {
            // Even the best yaw nearly points at the sun (shadow silhouettes).
            score *= 0.85;
        }
        // Low-sun band preference: the sun is one fixed world direction, so
        // a spot's OWN sun elevation is whatever latitude it landed on - a
        // midday sun flattens relief into mush (NdotL near-constant across
        // every slope), hides the sun disc above the frame, and switches
        // the sky's warm horizon band off. Candidates standing in the
        // sun's low-elevation band (13 to 30 degrees up: the late-afternoon
        // latitudes) double their score, and deep-midday spots (the sun
        // over ~45 degrees up) halve it - the search lands the showcase in
        // the golden band whenever the biome windows have any presence
        // there, and still places a view (unflattered, day side) when they
        // do not. Macro views never score here: the globe's terminator is
        // the whole point from orbit.
        {
            const double sunUp = sunValid
                ? planet::dot(planet::Vec3{sunDirection.x, sunDirection.y, sunDirection.z}, direction)
                : 1.0;
            if (sunUp >= 0.16 && sunUp <= 0.40)
            {
                score *= 2.0;
            }
            else if (sunUp > 0.70)
            {
                score *= 0.5;
            }
            else if (sunUp > 0.40)
            {
                // Above the golden band but under midday: mild penalty, so
                // any qualifying candidate INSIDE the band wins first.
                score *= 0.8;
            }
            else if (sunUp < 0.05)
            {
                // Day side only: the terminator's own band and the night
                // side read as dusk and dark, not as landscape.
                score *= 0.4;
            }
        }
        if (score > bestScore)
        {
            // Demote the previous best to the runner-up slot (it is the
            // strongest known alternative distinct from the new best).
            if (haveBest && planet::dot(best.direction, direction) < kRunnerSeparationDot)
            {
                runner          = best;
                runnerScore     = bestScore;
                runnerElevation = bestElevation;
                runnerTemperature = bestTemperature;
                runnerBiome     = bestBiome;
                haveRunner      = true;
            }
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
        else if (score > runnerScore && haveBest
            && planet::dot(best.direction, direction) < kRunnerSeparationDot)
        {
            runnerScore     = score;
            runner.direction = direction;
            runner.surfaceRadius = radius;
            runner.hasLookTangent = hasWaterward || sunValid || hasCliffWall;
            runner.lookTangent    = hasWaterward ? waterwardTangent
                                : (hasCliffWall ? cliffWallTangent
                                : (sunValid ? yawedTangents[bestYaw] : planet::Vec3{}));
            haveRunner      = true;
            runnerElevation = elevation;
            runnerTemperature = temperature01;
            runnerBiome     = static_cast<int>(biome);
        }
    }

    // Pass threshold: 0.5 for showcase views. Pinned samples take a quarter
    // of the ring: a climate zone covering a fraction of a percent of the
    // globe (the census's rarest rows) sits as mosaic patches inside other
    // biomes' country, and the best achievable neighborhood fraction is
    // small - the predicate already guarantees the SPOT itself is the
    // pinned zone, so the threshold only guards the degenerate all-fringe
    // case, not showcase composition.
    const double passThreshold = policy == ProbePolicy::Pinned ? 0.25 : 0.5;
    if (haveBest && bestScore >= passThreshold)
    {
        const bool useRunner = choiceIndex > 0 && haveRunner;
        out = useRunner ? runner : best;
        std::cout << "demo_planet: viewpoint (" << (useRunner ? "runner-up" : "best-of-planet") << " "
                  << (useRunner ? runnerScore : bestScore) << ") - elevation "
                  << (useRunner ? runnerElevation : bestElevation)
                  << " m, temperature " << (useRunner ? runnerTemperature : bestTemperature) << ", biome "
                  << (useRunner ? runnerBiome : bestBiome)
                  << ", sun elevation " << (std::asin(std::clamp(planet::dot(
                        (useRunner ? runner : best).direction,
                        planet::Vec3{sunDirection.x, sunDirection.y, sunDirection.z}), -1.0, 1.0)) * 57.29577951308232)
                  << " deg\n";
        if (useRunner && choiceIndex > 1)
        {
            std::cout << "demo_planet: viewpoint choice index " << choiceIndex
                      << " beyond the tracked runner-up; using the runner-up\n";
        }
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
        const std::size_t printRingCount = policy == ProbePolicy::Coastline ? 7u : 6u;
        std::cerr << "demo_planet: strongest rejected fan rises (rows = ring 60/120/220/420/720/1440"
                     << (printRingCount > 6 ? "/2880" : "") << " m, cols = yaw -0.8..+0.8):\n";
        for (std::size_t ring = 0; ring < printRingCount; ++ring)
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

} // namespace demo::planet_demo
