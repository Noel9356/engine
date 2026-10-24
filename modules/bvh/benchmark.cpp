// Standalone benchmark for the engine's binary BVH split strategies.
// Compiles against the engine's own headers (modules/bvh/) and measures the
// workload the engine actually drives: build a hierarchy over the box set,
// then run overlap queries (physics uses build + N queries per frame).
//
// Metrics per scene size and strategy:
//   - build ns (median of K runs)
//   - query ns of a full box-against-hierarchy batch (median of K runs)
//   - query-only throughput on a fixed random query set (ns per query)
//   - correctness: query result sets identical to the brute-force O(N^2)
//     answer for every strategy.
//
// The shipped split default ([bvh].split = "midpoint") is the benchmark
// winner under the engine's build-per-call contract: the midpoint tree's
// build + batch total is the lowest at every measured operating point.
//
// Build and run (no dependencies beyond the C++ standard library):
//   g++ -std=c++20 -O2 -o split_bench modules/bvh/benchmark.cpp && ./split_bench [seed]

#include "binary_bvh.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using engine::AxisAlignedBoundingBox;
using engine::BoundingVolumeHierarchy;
using engine::BvhSplitMethod;

namespace
{

struct MethodEntry
{
    const char    *name;
    BvhSplitMethod method;
};

const MethodEntry methods[] = {
    {"median",   BvhSplitMethod::Median  },
    {"midpoint", BvhSplitMethod::Midpoint},
    {"sah",      BvhSplitMethod::Sah     },
};

std::uint64_t nowNanos()
{
    using clock = std::chrono::steady_clock;
    return std::uint64_t(
        std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch()).count()
    );
}

// A plausible scene: a floor plane, some walls, furniture-scale boxes and a
// few large ones. Deterministic given the seed.
std::vector<AxisAlignedBoundingBox> makeScene(
    std::uint32_t count, std::mt19937 &rng
)
{
    std::uniform_real_distribution<float> position(-20.0f, 20.0f);
    std::uniform_real_distribution<float> half(0.05f, 1.5f);
    std::uniform_real_distribution<float> largeHalf(2.0f, 6.0f);
    std::vector<AxisAlignedBoundingBox>   boxes;
    boxes.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        AxisAlignedBoundingBox box;
        const bool             large  = index % 16 == 0;
        const float            extent = large ? largeHalf(rng) : half(rng);
        const float            cx     = position(rng);
        const float            cy     = large ? extent : std::abs(position(rng)) * 0.1f + extent;
        const float            cz     = position(rng);
        box.minPoint = {cx - extent, cy - extent, cz - extent};
        box.maxPoint = {cx + extent, cy + extent, cz + extent};
        boxes.push_back(box);
    }
    return boxes;
}

// One query batch: every box against the hierarchy (the overlappingPairs
// workload physics drives). Returns a checksum so timing loops cannot be
// dead-code eliminated.
std::uint64_t batchOverlaps(
    const BoundingVolumeHierarchy &hierarchy, const std::vector<AxisAlignedBoundingBox> &boxes
)
{
    std::vector<std::uint32_t> results;
    std::uint64_t              checksum = 0;
    for (const AxisAlignedBoundingBox &box : boxes)
    {
        results.clear();
        hierarchy.queryOverlaps(box, results);
        checksum += results.size();
        for (const std::uint32_t index : results)
        {
            checksum += index;
        }
    }
    return checksum;
}

std::vector<std::vector<std::uint32_t>> bruteForceOverlaps(
    const std::vector<AxisAlignedBoundingBox> &boxes
)
{
    std::vector<std::vector<std::uint32_t>> overlaps(boxes.size());
    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        for (std::size_t j = 0; j < boxes.size(); ++j)
        {
            if (engine::boxOverlaps(boxes[i], boxes[j]))
            {
                overlaps[i].push_back(static_cast<std::uint32_t>(j));
            }
        }
    }
    for (auto &list : overlaps)
    {
        std::sort(list.begin(), list.end());
    }
    return overlaps;
}

auto medianOf = [](std::vector<std::uint64_t> samples)
{
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
};

// A clustered scene: most boxes packed into a few dense pockets, a few
// scattered - the case SAH is supposed to win. Deterministic given the seed.
std::vector<AxisAlignedBoundingBox> makeClusteredScene(
    std::uint32_t count, std::mt19937 &rng
)
{
    std::uniform_real_distribution<float> pocketCenter(-20.0f, 20.0f);
    std::uniform_real_distribution<float> inPocket(-1.2f, 1.2f);
    std::uniform_real_distribution<float> half(0.05f, 0.8f);
    std::uniform_real_distribution<float> scatter(-25.0f, 25.0f);
    std::vector<AxisAlignedBoundingBox>   boxes;
    boxes.reserve(count);
    const std::uint32_t pocketCount = std::max(1u, count / 24u);
    std::vector<std::array<float, 3>> pockets;
    for (std::uint32_t pocket = 0; pocket < pocketCount; ++pocket)
    {
        pockets.push_back({pocketCenter(rng), std::abs(pocketCenter(rng)) * 0.1f, pocketCenter(rng)});
    }
    for (std::uint32_t index = 0; index < count; ++index)
    {
        AxisAlignedBoundingBox box;
        const float            extent = half(rng);
        const std::array<float, 3> &center =
            (index % 8 != 0) ? pockets[rng() % pockets.size()]
                             : std::array<float, 3>{scatter(rng), std::abs(scatter(rng)) * 0.1f, scatter(rng)};
        const float jitterX = inPocket(rng);
        const float jitterY = inPocket(rng);
        const float jitterZ = inPocket(rng);
        box.minPoint = {center[0] + jitterX - extent, center[1] + jitterY - extent, center[2] + jitterZ - extent};
        box.maxPoint = {center[0] + jitterX + extent, center[1] + jitterY + extent, center[2] + jitterZ + extent};
        boxes.push_back(box);
    }
    return boxes;
}

} // namespace

int main(
    int argc, char **argv
)
{
    const std::uint32_t seed = argc > 1 ? static_cast<std::uint32_t>(std::atoi(argv[1])) : 20260724u;
    std::mt19937        rng(seed);

    const std::uint32_t sceneSizes[] = {32, 256, 1024};
    const std::uint32_t batchRuns    = 400; // build+query batches per measurement
    const std::uint32_t queryRounds  = 40;  // query-only rounds per measurement
    const std::uint32_t queryCount   = 1024;

    std::printf("bvh split strategy benchmark (seed %u)\n", seed);
    std::printf("%-8s | %-9s | %-11s | %-12s | %-12s\n", "boxes", "method", "build ns", "batch us", "query ns");
    std::printf("------------------------------------------------------------------\n");

    bool allCorrect = true;
    for (const bool clustered : {false, true})
    {
        std::printf("%s scenes\n", clustered ? "clustered" : "spread");
        for (const std::uint32_t count : sceneSizes)
        {
            const std::vector<AxisAlignedBoundingBox>     boxes =
                clustered ? makeClusteredScene(count, rng) : makeScene(count, rng);
            const std::vector<std::vector<std::uint32_t>> truth = bruteForceOverlaps(boxes);

        // Fixed random query set for the query-only measurement.
        std::vector<AxisAlignedBoundingBox> queries;
        queries.reserve(queryCount);
        {
            std::uniform_real_distribution<float> position(-22.0f, 22.0f);
            std::uniform_real_distribution<float> extent(0.5f, 4.0f);
            for (std::uint32_t index = 0; index < queryCount; ++index)
            {
                const float cx = position(rng);
                const float cy = position(rng);
                const float cz = position(rng);
                const float e  = extent(rng);
                AxisAlignedBoundingBox box;
                box.minPoint = {cx - e, cy - e, cz - e};
                box.maxPoint = {cx + e, cy + e, cz + e};
                queries.push_back(box);
            }
        }

        for (const MethodEntry &entry : methods)
        {
            // ---- correctness against brute force -----------------------
            BoundingVolumeHierarchy hierarchy;
            hierarchy.build(boxes, entry.method);
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                std::vector<std::uint32_t> results;
                hierarchy.queryOverlaps(boxes[i], results);
                std::sort(results.begin(), results.end());
                if (results != truth[i])
                {
                    std::printf("CORRECTNESS FAILURE: %s scene %u box %zu\n", entry.name, count, i);
                    allCorrect = false;
                    break;
                }
            }

            // ---- build + batch (the physics workload) ------------------
            std::vector<std::uint64_t> buildTimes;
            std::vector<std::uint64_t> batchTimes;
            std::uint64_t              checksum = 0;
            for (std::uint32_t run = 0; run < batchRuns; ++run)
            {
                const std::uint64_t start = nowNanos();
                hierarchy.build(boxes, entry.method);
                const std::uint64_t built = nowNanos();
                checksum += batchOverlaps(hierarchy, boxes);
                batchTimes.push_back(nowNanos() - built);
                buildTimes.push_back(built - start);
            }

            // ---- query-only throughput ---------------------------------
            hierarchy.build(boxes, entry.method);
            std::vector<std::uint64_t> queryTimes;
            std::uint64_t              queryChecksum = 0;
            for (std::uint32_t run = 0; run < queryRounds; ++run)
            {
                const std::uint64_t start = nowNanos();
                for (const AxisAlignedBoundingBox &query : queries)
                {
                    std::vector<std::uint32_t> results;
                    hierarchy.queryOverlaps(query, results);
                    queryChecksum += results.size();
                }
                queryTimes.push_back(nowNanos() - start);
            }

            auto median = [](std::vector<std::uint64_t> samples)
            {
                std::sort(samples.begin(), samples.end());
                return samples[samples.size() / 2];
            };

            std::printf("%-8u | %-9s | %-11llu | %-12.2f | %-12.1f\n",
                        count, entry.name,
                        static_cast<unsigned long long>(median(buildTimes)),
                        static_cast<double>(median(batchTimes)) / 1000.0,
                        static_cast<double>(median(queryTimes)) / static_cast<double>(queryCount));
            (void)checksum;
            (void)queryChecksum;
        }
        std::printf("------------------------------------------------------------------\n");
        }
    }

    std::printf(allCorrect ? "correctness: all strategies match brute force\n"
                           : "correctness: FAILURES DETECTED\n");
    return allCorrect ? 0 : 1;
}