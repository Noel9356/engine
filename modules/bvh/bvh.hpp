#ifndef ENGINE_MODULES_BVH_BVH_HPP
#define ENGINE_MODULES_BVH_BVH_HPP

#include "bvh_capability.hpp"
#include "binary_bvh.hpp"
#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <string>
#include <vector>

namespace engine
{

// Module "bvh": the binary bounding volume hierarchy provider (family member
// "bvh.binary").
//
// The module IS the capability: BvhCapability's contract is stateless spatial
// queries, and the implementation answers them from a binary BVH built per
// call (see binary_bvh.hpp). There is no runtime state to keep coherent - the
// caller's box list is the truth, the hierarchy is a per-call snapshot of it.
//
// The split strategy is configuration ([bvh].split: median | midpoint | sah,
// default midpoint - the benchmark winner, see binary_bvh.hpp) read once at
// construct and applied to every build the capability serves.
class BvhModule final : public ModuleInterface, public BvhCapability
{
public:
    BvhModule() = default;
    ~BvhModule() override = default;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // BvhCapability.
    void overlappingPairs(
        const std::vector<AxisAlignedBoundingBox> &boxes,
        std::vector<std::vector<std::uint32_t>>   &outOverlaps
    ) override;
    void queryOverlaps(
        const std::vector<AxisAlignedBoundingBox> &boxes,
        const AxisAlignedBoundingBox              &query,
        std::vector<std::uint32_t>                &outIndices
    ) override;

private:
    ComponentManifest componentManifest;
    LoggerCapability *logger = nullptr;
    // The split strategy [bvh].split selected at construct; every hierarchy
    // the capability builds uses it.
    BvhSplitMethod    splitMethod = BvhSplitMethod::Midpoint;
};

} // namespace engine

#endif // ENGINE_MODULES_BVH_BVH_HPP
