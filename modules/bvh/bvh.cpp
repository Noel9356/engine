#include "bvh.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace engine
{

namespace
{

// The [bvh].split choices, in the spelling the configuration accepts. The
// default is the benchmark winner (modules/bvh/benchmark.cpp): under the
// engine's build-per-call contract the midpoint tree has the lowest
// build + query total at every measured box count.
constexpr const char *defaultSplitName = "midpoint";

BvhSplitMethod splitMethodFromName(
    const std::string &name
)
{
    if (name == "median")
    {
        return BvhSplitMethod::Median;
    }
    if (name == "midpoint")
    {
        return BvhSplitMethod::Midpoint;
    }
    if (name == "sah")
    {
        return BvhSplitMethod::Sah;
    }
    return BvhSplitMethod::Midpoint; // unreachable; parseSplitName rejects first
}

// Returns an empty string when the name parses; otherwise the diagnostic.
std::string validateSplitName(
    const std::string &name
)
{
    if (name == "median" || name == "midpoint" || name == "sah")
    {
        return {};
    }
    return "bvh module: [bvh] split '" + name
         + "' is not a known strategy (median, midpoint, sah)";
}

const char *nameOfSplitMethod(
    BvhSplitMethod method
)
{
    switch (method)
    {
    case BvhSplitMethod::Median: return "median";
    case BvhSplitMethod::Midpoint: return "midpoint";
    case BvhSplitMethod::Sah: return "sah";
    }
    return "midpoint";
}

} // namespace

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
    componentManifest.knownConfigKeys       = {"bvh.split"};

    logger = &context.acquireCapability<LoggerCapability>();

    // Split strategy: [bvh].split, one of median | midpoint | sah. The
    // shipped module default (config.toml) and the code default agree on the
    // benchmark winner; an assembly overrides from engine.toml, which wins
    // over every defaults layer.
    const std::string splitName = context.configurationString("split").value_or(defaultSplitName);
    if (const std::string splitError = validateSplitName(splitName); !splitError.empty())
    {
        throw std::runtime_error(splitError);
    }
    splitMethod = splitMethodFromName(splitName);
}

void BvhModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        std::string("bvh module started (binary ") + nameOfSplitMethod(splitMethod) + "-split hierarchy)"
    );
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
    // One hierarchy per call, one walk per box, split strategy from [bvh].
    // At the registry cap the whole batch is microseconds; a persistent
    // hierarchy would need an invalidation protocol for no measurable gain.
    BoundingVolumeHierarchy hierarchy;
    hierarchy.build(boxes, splitMethod);

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
    hierarchy.build(boxes, splitMethod);
    hierarchy.queryOverlaps(query, outIndices);
}

} // namespace engine
