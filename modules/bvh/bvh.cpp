#include "bvh.hpp"

#include <algorithm>

namespace engine
{

void BvhModule::construct(
    ComponentContext &context
)
{
    componentManifest.name                  = "bvh";
    componentManifest.kind                  = ComponentKind::Module;
    // The implementation-bound member name of the "bvh" family: a different
    // spatial index declares its own member and wins through the [providers]
    // configuration; consumers keep demanding the family.
    componentManifest.providedCapabilities  = {"bvh.binary"};
    componentManifest.requiredCapabilities  = {"logging"};
    componentManifest.requiredPermissions   = {"capability.logging"};
    componentManifest.knownConfigKeys       = {};

    logger = &context.acquireCapability<LoggerCapability>();
}

void BvhModule::start()
{
    logger->logMessage(LogLevel::Information, "bvh module started (binary median-split hierarchy)");
}

void BvhModule::stop()
{
    logger = nullptr;
}

void *BvhModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "bvh.binary")
    {
        return static_cast<BvhCapability *>(this);
    }
    return nullptr;
}

void BvhModule::overlappingPairs(
    const std::vector<AxisAlignedBoundingBox> &boxes,
    std::vector<std::vector<std::uint32_t>>   &outOverlaps
)
{
    // One hierarchy per call, one walk per box. At the registry cap the
    // whole batch is microseconds; a persistent hierarchy would need an
    // invalidation protocol for no measurable gain.
    BoundingVolumeHierarchy hierarchy;
    hierarchy.build(boxes);

    outOverlaps.clear();
    outOverlaps.resize(boxes.size());
    for (std::size_t index = 0; index < boxes.size(); ++index)
    {
        hierarchy.queryOverlaps(boxes[index], outOverlaps[index]);
    }
}

void BvhModule::queryOverlaps(
    const std::vector<AxisAlignedBoundingBox> &boxes,
    const AxisAlignedBoundingBox              &query,
    std::vector<std::uint32_t>                &outIndices
)
{
    BoundingVolumeHierarchy hierarchy;
    hierarchy.build(boxes);
    hierarchy.queryOverlaps(query, outIndices);
}

} // namespace engine
