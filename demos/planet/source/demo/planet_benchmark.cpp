// Performance benchmark for the planet generation core.
//
// Deterministic micro- and macro-benchmarks over the module's pure fields
// and chunk build path, so tuning and optimization work can be compared
// across runs and machines: field sampling costs, per-depth chunk mesh
// builds over a fixed surface-hugging chunk set, and the streaming
// pipeline's delivery rate and update overhead. Everything the benchmark
// measures derives from the module's own parameters (seed, radius), so
// `--bench` next to `--self-test` gives a repeatable profile without any
// manual inspection.

#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/planet.hpp>
#include <engine/planet/types.hpp>
#include "planet_demo_support.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace demo::planet_demo
{
namespace planet = engine::planet;

namespace
{

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct TimingStats
{
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;
};

TimingStats summarize(std::vector<double> samples)
{
    TimingStats stats;
    if (samples.empty())
    {
        return stats;
    }
    std::sort(samples.begin(), samples.end());
    double sum = 0.0;
    for (double sample : samples)
    {
        sum += sample;
    }
    stats.mean = sum / static_cast<double>(samples.size());
    stats.min = samples.front();
    stats.max = samples.back();
    return stats;
}

void printStatsLine(const char *label, const TimingStats &stats, const char *unit)
{
    std::printf("bench: %-26s mean %10.4f  min %10.4f  max %10.4f  %s\n", label, stats.mean, stats.min, stats.max,
        unit);
}

// Times one evaluation per sample; the volatile sink keeps the calls from
// being optimized away.
template <typename Sample, typename Evaluate>
std::vector<double> timeSamples(const std::vector<Sample> &samples, const Evaluate &evaluate)
{
    std::vector<double> times;
    times.reserve(samples.size());
    volatile double sink = 0.0;
    for (const Sample &sample : samples)
    {
        const auto start = Clock::now();
        sink += evaluate(sample);
        times.push_back(millisecondsSince(start));
    }
    (void)sink;
    return times;
}

} // namespace

int runBenchmark(const planet::PlanetGenerationParams &params)
{
    std::printf("bench: planet seed %llu, radius %.0f m, depth %u, voxel %.3f m, workers %u\n",
        static_cast<unsigned long long>(params.seed), params.radiusMeters, params.derivedMaxDepth,
        params.derivedVoxelMeters, params.maxConcurrentBuilds);

    planet::DensityField field(params);

    // ---- field sampling ------------------------------------------------------
    {
        const std::vector<planet::Vec3> directions = goldenSpiralDirections(512);
        const double sampleRadius = params.radiusMeters + 64.0;
        std::vector<planet::Vec3> points;
        points.reserve(directions.size());
        for (const planet::Vec3 &direction : directions)
        {
            points.push_back(direction * sampleRadius);
        }
        const double fineCutoff = params.derivedVoxelMeters * params.lodBandLimitFactor;

        printStatsLine("densityAt (raw)",
            summarize(timeSamples(points, [&field](const planet::Vec3 &p) { return field.densityAt(p); })), "ms");
        printStatsLine(
            "densityAt (finest cutoff)",
            summarize(timeSamples(points, [&field, fineCutoff](const planet::Vec3 &p) { return field.densityAt(p, fineCutoff); })),
            "ms");
        printStatsLine("reliefAt",
            summarize(timeSamples(directions, [&field](const planet::Vec3 &d) { return field.reliefAt(d).heightMeters; })), "ms");
        printStatsLine("climate sample",
            summarize(timeSamples(directions,
                [&field, &params](const planet::Vec3 &d)
                { return field.sampleClimate(d, params.radiusMeters + 10.0, 0.2).color[0]; })),
            "ms");
        printStatsLine("seaLevelTemperature",
            summarize(timeSamples(directions, [&field](const planet::Vec3 &d) { return field.climate().seaLevelTemperature(d); })),
            "ms");
        printStatsLine("cloudShadow",
            summarize(timeSamples(directions,
                [&field, &params](const planet::Vec3 &d) { return field.cloudShadow(d, params.derivedVoxelMeters); })),
            "ms");
        printStatsLine("surfaceRadius",
            summarize(timeSamples(directions, [&field](const planet::Vec3 &d) { return field.surfaceRadius(d); })), "ms");
    }

    // ---- chunk builds per depth ----------------------------------------------
    {
        std::uint32_t depths[6] = {2u, 5u, 8u, 11u, 12u, params.derivedMaxDepth};
        std::vector<std::uint32_t> uniqueDepths;
        for (std::uint32_t depth : depths)
        {
            if (depth <= params.derivedMaxDepth
                && std::find(uniqueDepths.begin(), uniqueDepths.end(), depth) == uniqueDepths.end())
            {
                uniqueDepths.push_back(depth);
            }
        }
        for (std::uint32_t depth : uniqueDepths)
        {
            // Surface-hugging chunk set: the cells containing the surface
            // points under a golden spiral of directions (spread over the
            // whole globe - land, sea floor, coasts and mountains), deduped
            // so wide cells at coarse depths do not dominate the timing.
            const std::vector<planet::Vec3> directions = goldenSpiralDirections(80);
            std::vector<planet::ChunkCoord> chunks;
            const double nodeSize = planet::chunkSizeMeters(params, depth);
            const double half = params.derivedRootHalfMeters;
            const std::int64_t cellCount = static_cast<std::int64_t>(1) << depth;
            for (const planet::Vec3 &direction : directions)
            {
                const double surfaceRadius = field.surfaceRadius(direction);
                if (surfaceRadius <= 0.0)
                {
                    continue;
                }
                const planet::Vec3 point = direction * surfaceRadius;
                const auto cellOf = [&](double value)
                {
                    return std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor((value + half) / nodeSize)), 0,
                        cellCount - 1);
                };
                planet::ChunkCoord coord{depth, static_cast<std::uint32_t>(cellOf(point.x)),
                    static_cast<std::uint32_t>(cellOf(point.y)), static_cast<std::uint32_t>(cellOf(point.z))};
                bool duplicate = false;
                for (const planet::ChunkCoord &existing : chunks)
                {
                    if (existing == coord)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate)
                {
                    chunks.push_back(coord);
                }
                if (chunks.size() >= 6u)
                {
                    break;
                }
            }
            if (chunks.empty())
            {
                continue;
            }
            std::vector<double> times;
            std::uint64_t triangles = 0;
            std::uint64_t vertices = 0;
            for (const planet::ChunkCoord &coord : chunks)
            {
                const auto start = Clock::now();
                const planet::ChunkMesh mesh = planet::buildChunkMesh(params, field, coord);
                times.push_back(millisecondsSince(start));
                triangles += mesh.triangleCount;
                vertices += mesh.terrainVertices.size() + mesh.waterVertices.size();
            }
            const TimingStats stats = summarize(times);
            std::printf("bench: chunk depth %2u (%6.0f m) x%u  mean %10.3f  min %10.3f  max %10.3f  ms  (%llu tri, "
                        "%llu vtx)\n",
                depth, planet::chunkSizeMeters(params, depth), static_cast<std::uint32_t>(chunks.size()), stats.mean,
                stats.min, stats.max, static_cast<unsigned long long>(triangles),
                static_cast<unsigned long long>(vertices));
        }
    }

    // ---- shore profile diagnostic ---------------------------------------------
    // Walk one beach cross-section: elevation of the terrain across the
    // waterline plus the water-surface quantities the mesher derives from it
    // (depth, foam band, depth-tint fraction). Shows how wide the surf and
    // shelf-tint bands actually are at the configured parameters.
    {
        const std::vector<planet::Vec3> directions = goldenSpiralDirections(256);
        planet::Vec3 waterline{};
        planet::Vec3 towardSea{};
        bool haveCrossing = false;
        for (const planet::Vec3 &probe : directions)
        {
            const double elevation = field.surfaceRadius(probe) - params.derivedSeaRadiusMeters;
            if (elevation < 0.5 || elevation > 12.0)
            {
                continue;
            }
            // Local search: 16 tangent directions, 10 m steps, until the
            // elevation crosses zero (the waterline in that direction).
            for (std::uint32_t bearing = 0; bearing < 16 && !haveCrossing; ++bearing)
            {
                const double azimuth = 6.283185307179586 * static_cast<double>(bearing) / 16.0;
                planet::Vec3 side = planet::cross(probe, planet::Vec3{0.0, 1.0, 0.0});
                if (planet::length(side) < 1.0e-3)
                {
                    side = planet::cross(probe, planet::Vec3{1.0, 0.0, 0.0});
                }
                side = side * (1.0 / planet::length(side));
                planet::Vec3 step = side * std::cos(azimuth) + planet::cross(side, probe) * std::sin(azimuth);
                step = step * (1.0 / planet::length(step));
                double previousArc = 0.0;
                double previousElevation = elevation;
                for (double arc = 10.0; arc <= 500.0; arc += 10.0)
                {
                    const double angle = arc / params.radiusMeters;
                    const planet::Vec3 walked = planet::normalized(probe * std::cos(angle) + step * std::sin(angle));
                    const double walkedElevation = field.surfaceRadius(walked) - params.derivedSeaRadiusMeters;
                    if (previousElevation > 0.0 && walkedElevation <= 0.0)
                    {
                        // Waterline between previousArc and arc: refine by bisection.
                        double lowArc = previousArc;
                        double highArc = arc;
                        for (std::uint32_t iteration = 0; iteration < 20; ++iteration)
                        {
                            const double midArc = 0.5 * (lowArc + highArc);
                            const double midAngle = midArc / params.radiusMeters;
                            const planet::Vec3 mid = planet::normalized(
                                probe * std::cos(midAngle) + step * std::sin(midAngle));
                            if (field.surfaceRadius(mid) - params.derivedSeaRadiusMeters > 0.0)
                            {
                                lowArc = midArc;
                            }
                            else
                            {
                                highArc = midArc;
                            }
                        }
                        const double crossAngle = 0.5 * (lowArc + highArc) / params.radiusMeters;
                        waterline = planet::normalized(probe * std::cos(crossAngle) + step * std::sin(crossAngle));
                        towardSea = step;
                        haveCrossing = true;
                        break;
                    }
                    previousArc = arc;
                    previousElevation = walkedElevation;
                }
            }
            if (haveCrossing)
            {
                break;
            }
        }
        if (haveCrossing)
        {
            const double seaRadius = params.derivedSeaRadiusMeters;
            const double radiusMeters = params.radiusMeters;
            const double foamWidth = params.waterFoamWidthMeters;
            const double tintDepth = params.waterDepthTintMeters;
            std::printf("bench: shore profile (arc meters, terrain elevation, water depth, foam weight, tint):\n");
            for (int step = -8; step <= 60; ++step)
            {
                const double arc = 6.0 * static_cast<double>(step);
                // Rotate the waterline direction toward the sea by arc/R.
                const double angle = arc / radiusMeters;
                const planet::Vec3 direction = planet::normalized(
                    waterline * std::cos(angle) + towardSea * std::sin(angle));
                const double elevation = field.surfaceRadius(direction) - seaRadius;
                const planet::Vec3 surfacePoint = direction * seaRadius;
                const double depth = std::max(0.0, -field.densityAt(surfacePoint));
                const double depthBand = 1.0 - planet::smoothstep(0.0, foamWidth, depth);
                const double foamBand = depthBand * depthBand * depthBand * depthBand;
                std::printf("bench:   %+7.1f m  elev %+8.2f  depth %7.2f  foam %5.3f  tint %5.3f\n", arc, elevation,
                    depth, foamBand, planet::clamp01(depth / tintDepth));
            }
        }
        else
        {
            std::printf("bench: shore profile - no waterline crossing found in the probe set\n");
        }
    }

    // ---- streaming pipeline --------------------------------------------------
    {
        planet::Planet planet(params);
        const std::vector<planet::Vec3> directions = goldenSpiralDirections(64);
        const planet::Vec3 spawn = directions[7];
        const double surfaceRadius = field.surfaceRadius(spawn);
        const planet::Vec3 eye = spawn * (surfaceRadius + 1.7);

        // Delivery rate: run updates until the visible set is fully built or
        // the wall-clock budget runs out; the sleep mirrors the demo's warmup
        // loop (the workers need wall-clock time).
        constexpr double kBudgetSeconds = 20.0;
        const auto streamStart = Clock::now();
        const auto deadline = streamStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(kBudgetSeconds));
        std::uint32_t updates = 0;
        std::vector<double> updateCosts;
        bool complete = false;
        while (Clock::now() < deadline)
        {
            const auto start = Clock::now();
            planet.update(eye);
            updateCosts.push_back(millisecondsSince(start));
            ++updates;
            const planet::PlanetStatistics &statistics = planet.statistics();
            if (statistics.chunksVisible > 0 && statistics.chunksBuilding == 0
                && statistics.chunksReady >= statistics.chunksVisible)
            {
                complete = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const double streamSeconds = millisecondsSince(streamStart) / 1000.0;
        const planet::PlanetStatistics &statistics = planet.statistics();
        std::printf("bench: streaming - %u updates, %u/%u visible ready (%s), %u cached, %.1f chunks/s average\n",
            updates, statistics.chunksReady, statistics.chunksVisible, complete ? "complete" : "budget cut",
            statistics.chunksReadyTotal, static_cast<double>(statistics.chunksReadyTotal) / std::max(0.001, streamSeconds));
        printStatsLine("update (streaming)", summarize(updateCosts), "ms");

        // Depth histogram of the visible set: which LOD levels the selection
        // keeps for a standing eye (where the request budget and the conform
        // mask work scale with the visible count).
        {
            std::vector<planet::PlanetChunkView> views(statistics.chunksVisible);
            const std::uint32_t copied = planet.copyVisibleChunks(views.data(), static_cast<std::uint32_t>(views.size()));
            std::uint32_t perDepth[16] = {};
            for (std::uint32_t index = 0; index < copied; ++index)
            {
                std::uint32_t depth = 0;
                std::uint32_t x = 0;
                std::uint32_t y = 0;
                std::uint32_t z = 0;
                planet::unpackChunkKey(views[index].key, depth, x, y, z);
                if (depth < 16)
                {
                    ++perDepth[depth];
                }
            }
            std::printf("bench: visible depth histogram:");
            for (std::uint32_t depth = 0; depth < 16; ++depth)
            {
                if (perDepth[depth] != 0)
                {
                    std::printf(" d%u:%u", depth, perDepth[depth]);
                }
            }
            std::printf("\n");
        }

        // Idle updates: a static eye with nothing left to deliver must stay
        // cheap (the fast path the demo's walk mode leans on between moves).
        {
            std::vector<double> idle;
            for (std::uint32_t index = 0; index < 240; ++index)
            {
                const auto start = Clock::now();
                planet.update(eye);
                idle.push_back(millisecondsSince(start));
            }
            printStatsLine("update (idle eye)", summarize(idle), "ms");
        }
        // Selection churn: a moving eye re-selects every update.
        {
            std::vector<double> moving;
            planet::Vec3 walkingEye = eye;
            for (std::uint32_t index = 0; index < 60; ++index)
            {
                const planet::Vec3 up = planet::normalized(walkingEye);
                planet::Vec3 heading = planet::cross(up, planet::Vec3{0.0, 1.0, 0.0});
                if (planet::length(heading) < 0.1)
                {
                    heading = planet::cross(up, planet::Vec3{1.0, 0.0, 0.0});
                }
                heading = heading * (1.0 / planet::length(heading));
                walkingEye = walkingEye + heading * 4.5;
                const auto start = Clock::now();
                planet.update(walkingEye);
                moving.push_back(millisecondsSince(start));
            }
            printStatsLine("update (walking eye)", summarize(moving), "ms");
        }
    }
    return 0;
}

} // namespace demo::planet_demo
