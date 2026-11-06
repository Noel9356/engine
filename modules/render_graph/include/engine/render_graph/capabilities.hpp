// Public capability surface of the render_graph module.
//
// Declares the "render_graph" capability: validation, topological pass
// ordering, budgeted transient-resource aliasing, and scheduled execution
// for render graphs described as plain structs. Consumers bind it via
// ParticipantContext::acquireCapability<RenderGraphCapability>().

#ifndef ENGINE_RENDER_GRAPH_CAPABILITIES_HPP
#define ENGINE_RENDER_GRAPH_CAPABILITIES_HPP

#include <engine/render/capabilities.hpp>

#include <cstdint>

namespace engine
{
namespace render_graph
{

// Sentinel resource index/slot meaning "none"; also pre-fills the compile
// slot output for resources that never alias.
inline constexpr std::uint32_t invalidResourceSlot = 0xffffffffu;

// How one pass touches one resource.
enum class ResourceAccess : std::uint32_t
{
    Read = 0,
    Write,
    ReadWrite
};
// Error codes reported through the optional outError parameters: malformed
// input (bad indices, bad alignment, undefined read, duplicate use in one
// pass), exceeded limits or budget, dependency cycle, scheduler failure.
enum class RenderGraphError : std::uint32_t
{
    None = 0,
    InvalidInput,
    CapacityExceeded,
    DependencyCycle,
    BudgetExceeded,
    ExecutionFailed
};

// Compile/execute limits and aliasing switch. The module accepts
// maxPasses/maxResources >= 1 and a transient budget up to 16 GiB.
struct RenderGraphSettings
{
    render::QualityPreset quality                    = render::QualityPreset::Auto;
    std::uint32_t         maxPasses                  = 256;
    std::uint32_t         maxResources               = 512;
    std::uint64_t         transientMemoryBudgetBytes = 512ull * 1024ull * 1024ull;
    bool                  transientAliasing          = true;
};

// Settings scaled for one device by resolveProfile: weaker GPUs get fewer
// passes/resources and a smaller transient budget.
struct RenderGraphProfile
{
    std::uint32_t maxPasses                  = 0;
    std::uint32_t maxResources               = 0;
    std::uint64_t transientMemoryBudgetBytes = 0;
    bool          transientAliasing          = false;
    bool          asyncCompute               = false;
};

// One graph resource. alignmentBytes must be a power of two and sizeBytes
// non-zero (validated); compatibilityClass groups resources whose physical
// memory may be shared; transient resources are graph-allocated and must
// not be initiallyDefined (their content starts undefined).
struct RenderGraphResourceDescription
{
    std::uint64_t sizeBytes          = 0;
    std::uint64_t alignmentBytes     = 256;
    std::uint32_t compatibilityClass = 0;
    bool          transient          = true;
    bool          initiallyDefined   = false; // content valid before the first write (imported inputs)
};

struct RenderGraphResourceUse
{
    std::uint32_t  resourceIndex = 0;
    ResourceAccess access        = ResourceAccess::Read;
};

// One pass: the resources it uses and the pass indices that must run first.
// uses/dependencies are borrowed and only need to stay alive for the
// duration of the compile/execute call.
struct RenderGraphPassDescription
{
    const RenderGraphResourceUse *uses            = nullptr;
    std::uint32_t                 useCount        = 0;
    const std::uint32_t          *dependencies    = nullptr;
    std::uint32_t                 dependencyCount = 0;
};

// Per-pass execution hook. passIndex is the caller's original pass index,
// not the scheduled position.
using RenderGraphExecuteCallback = void (*)(void *userData, std::uint32_t passIndex);

// Compile statistics: transientSlotCount is the number of physical
// allocations left after aliasing and transientBytes their combined size.
struct RenderGraphCompileResult
{
    std::uint32_t passCount          = 0;
    std::uint32_t resourceCount      = 0;
    std::uint32_t transientSlotCount = 0;
    std::uint64_t transientBytes     = 0;
};

// ABI version of RenderGraphCapability. Consumers compare it against the
// abi field after acquisition and must not call entries on a mismatch.
inline constexpr std::uint32_t renderGraphCapabilityAbi = 2;

// The "render_graph" capability: function pointers bound to the module
// singleton (self is the first argument of every entry).
struct RenderGraphCapability
{
    static constexpr const char *capabilityName = "render_graph";
    std::uint32_t                abi            = renderGraphCapabilityAbi;
    void                        *self           = nullptr;
    // Current settings; setSettings validates against the same limits and
    // clears the plan cache on success.
    const RenderGraphSettings *(*settings)(void *self);
    bool (*setSettings)(void *self, const RenderGraphSettings *settings);
    // Scales the configured limits for the given device into outProfile.
    bool (*resolveProfile)(void *self, const render::DeviceProfile *device, RenderGraphProfile *outProfile);
    // Orders passes topologically and assigns transient resources to
    // physical slots. outPassOrder/outResourceSlots need room for
    // passCount/resourceCount entries; on failure outError says why and the
    // outputs are not meaningful.
    bool (*compile)(void *self, const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, std::uint32_t *outPassOrder,
        std::uint32_t passCapacity, std::uint32_t *outResourceSlots, std::uint32_t resourceCapacity,
        RenderGraphCompileResult *outResult, RenderGraphError *outError);
    // Runs every pass on the kernel task scheduler in dependency order,
    // blocking the caller until all pass callbacks finished.
    bool (*execute)(void *self, const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        RenderGraphExecuteCallback callback, void *userData, RenderGraphError *outError);
};

} // namespace render_graph
} // namespace engine

#endif
