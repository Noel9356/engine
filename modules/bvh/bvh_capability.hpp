#ifndef ENGINE_MODULES_BVH_BVH_CAPABILITY_HPP
#define ENGINE_MODULES_BVH_BVH_CAPABILITY_HPP

// The published capability surface of the bvh module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic: boxes are plain
// values, the contract is numbers in and numbers out.
//
// Provider model: this header is the family contract ("bvh"). The provider
// in this module is the implementation-bound member "bvh.binary" - a binary
// bounding volume hierarchy over caller-supplied boxes, its split strategy
// configurable ([bvh].split, default "midpoint": the benchmark winner, see
// modules/bvh/binary_bvh.hpp). A different spatial index (a grid, a dynamic
// BVH) declares its own member of
// the family and wins through the [providers] configuration section; consumers
// keep demanding "bvh" and notice nothing but the answers.

#include <array>
#include <cstdint>
#include <vector>

namespace engine
{

// Axis-aligned box, plain arrays: [0]=min, [1]=max. The vocabulary type of
// the family: every consumer builds these from its own world representation.
struct AxisAlignedBoundingBox
{
    std::array<float, 3> minPoint = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> maxPoint = {0.0f, 0.0f, 0.0f};

    std::array<float, 3> center() const
    {
        return {(minPoint[0] + maxPoint[0]) * 0.5f,
                (minPoint[1] + maxPoint[1]) * 0.5f,
                (minPoint[2] + maxPoint[2]) * 0.5f};
    }
};

inline bool boxOverlaps(
    const AxisAlignedBoundingBox &a, const AxisAlignedBoundingBox &b
)
{
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (a.maxPoint[axis] < b.minPoint[axis] || b.maxPoint[axis] < a.minPoint[axis])
        {
            return false;
        }
    }
    return true;
}

// Capability "bvh": CPU-side "what is near what" answers over axis-aligned
// boxes, without touching the GPU.
//
// Why a module: several consumers need the same spatial index - the physics
// module culls collision pairs through it, and a future CPU SDF bake (the
// renderer's cost model is linear in the object count; a baked distance field
// needs fast neighborhood queries) would rest on the same answers. One
// provider, demand-driven like every module: a run whose mods do not pull a
// consumer of the family never pays for it.
//
// Stateless service: every call takes the full box set and answers from a
// hierarchy built over it. At the engine's registry caps a rebuild costs
// microseconds, and a stateless contract needs no invalidation protocol -
// the caller's box list is always the newest truth there is.
class BvhCapability
{
public:
    static constexpr const char *capabilityName = "bvh";

    virtual ~BvhCapability() = default;

    // One-shot broadphase: builds a hierarchy over `boxes` and appends, for
    // each index i, every box index returned by an overlap query with
    // boxes[i] (exact test, no false positives; a box overlaps itself, so i
    // can appear in its own list). Results per index are unordered - callers
    // that iterate symmetric pairs must not act twice on one pair.
    // outOverlaps is resized to boxes.size() and cleared per index.
    virtual void overlappingPairs(
        const std::vector<AxisAlignedBoundingBox> &boxes,
        std::vector<std::vector<std::uint32_t>>   &outOverlaps
    ) = 0;

    // One query against the given box set: appends every box overlapping the
    // query box to outIndices (unordered, no false positives). Convenience
    // for single-query callers; batch callers use overlappingPairs.
    virtual void queryOverlaps(
        const std::vector<AxisAlignedBoundingBox> &boxes,
        const AxisAlignedBoundingBox              &query,
        std::vector<std::uint32_t>                &outIndices
    ) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_BVH_BVH_CAPABILITY_HPP
