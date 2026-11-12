#include "planet_demo_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
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

} // namespace demo::planet_demo
