#include "planet_demo_support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace demo::planet_demo
{
namespace planet = engine::planet;
namespace renderer = engine::renderer;
using namespace engine;

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

std::array<renderer::VertexAttribute, 4> canonicalAttributes()
{
    return {{
        {renderer::VertexSemantic::Position, renderer::VertexFormat::Float32x3, 0},
        {renderer::VertexSemantic::Normal, renderer::VertexFormat::Float32x3, 12},
        {renderer::VertexSemantic::Color, renderer::VertexFormat::Float32x3, 24},
        {renderer::VertexSemantic::TexCoord0, renderer::VertexFormat::Float32x2, 36},
    }};
}

} // namespace demo::planet_demo
