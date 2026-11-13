// render_graph module implementation.
//
// Binds the "render_graph" capability: compiles graphs (validation,
// topological order, budgeted transient aliasing), executes them on the
// kernel task scheduler, and caches compiled plans keyed by graph shape.
//
// The heavy lifting lives in dedicated translation units:
//   - plan_cache.cpp    : LRU cache of compiled plans keyed by graph shape.
//   - aliasing.cpp      : transient resource aliasing allocator.
//   - execution.cpp     : cached TaskGraph build and execution.
//   - render_graph_compiler.cpp : validation, hazard edges, topological order.
//
// This file wires those pieces to the kernel module interface and the
// capability function-pointer table.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>

#include "internal/aliasing.hpp"
#include "internal/execution.hpp"
#include "internal/plan_cache.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace engine
{
namespace render_graph
{
namespace
{

// Ranges accepted through setSettings; construct() clamps config keys to
// the same limits.
bool validSettings(const RenderGraphSettings &value)
{
    return value.quality <= renderer::QualityPreset::Ultra && value.maxPasses >= 1 && value.maxPasses <= 4096
        && value.maxResources >= 1 && value.maxResources <= 8192
        && value.transientMemoryBudgetBytes <= (16ull * 1024ull * 1024ull * 1024ull);
}

// Maps the configured quality string to the preset enum.
renderer::QualityPreset qualityFromString(const std::string &value)
{
    if (value == "low")
        return renderer::QualityPreset::Low;
    if (value == "medium")
        return renderer::QualityPreset::Medium;
    if (value == "high")
        return renderer::QualityPreset::High;
    if (value == "ultra")
        return renderer::QualityPreset::Ultra;
    return renderer::QualityPreset::Auto;
}

// Maps the configured aliasing-strategy string to the enum.
detail::AliasingStrategy strategyFromString(const std::string &value)
{
    if (value == "first_fit")
        return detail::AliasingStrategy::FirstFit;
    if (value == "worst_fit")
        return detail::AliasingStrategy::WorstFit;
    return detail::AliasingStrategy::BestFit;
}

// Module singleton backing RenderGraphCapability. Holds the settings, the
// plan cache, the execution cache, and borrowed scheduler/profiler pointers;
// delegates compilation and execution to the detail helpers.
class RenderGraphModule final : public ModuleInterface
{
public:
    RenderGraphModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads [render_graph] with per-key clamping to the valid ranges; absent
    // or invalid keys keep the defaults. Also configures the plan cache and
    // execution cache.
    void construct(ParticipantContext &context) override
    {
        scheduler_ = &context.taskScheduler();
        settings_  = RenderGraphSettings{};
        if (const auto value = context.configurationStringInSection("render_graph", "quality"); value.has_value())
            settings_.quality = qualityFromString(*value);
        if (const auto value = context.configurationIntegerInSection("render_graph", "max_passes");
            value.has_value() && *value > 0)
            settings_.maxPasses = static_cast<std::uint32_t>(std::min<std::int64_t>(*value, 4096));
        if (const auto value = context.configurationIntegerInSection("render_graph", "max_resources");
            value.has_value() && *value > 0)
            settings_.maxResources = static_cast<std::uint32_t>(std::min<std::int64_t>(*value, 8192));
        if (const auto value = context.configurationIntegerInSection("render_graph", "transient_budget_mb");
            value.has_value() && *value >= 0)
            settings_.transientMemoryBudgetBytes
                = static_cast<std::uint64_t>(std::min<std::int64_t>(*value, 16384)) * 1024ull * 1024ull;
        settings_.transientAliasing
            = context.configurationBooleanInSection("render_graph", "transient_aliasing").value_or(true);

        // Aliasing strategy is an internal config knob (not exposed through
        // RenderGraphSettings to keep the ABI stable).
        aliasingStrategy_ = detail::AliasingStrategy::BestFit;
        if (const auto value = context.configurationStringInSection("render_graph", "aliasing_strategy");
            value.has_value())
            aliasingStrategy_ = strategyFromString(*value);

        // Compiling a plan is pure CPU work with the same result for a stable
        // graph shape; caching it removes that per-frame cost. Default 8
        // entries (set plan_cache_entries = 0 to disable).
        std::size_t planCacheEntries = 8;
        if (const auto value = context.configurationIntegerInSection("render_graph", "plan_cache_entries");
            value && *value >= 0 && *value <= 16)
            planCacheEntries = static_cast<std::size_t>(*value);
        std::size_t planCacheMaxElements = 16384;
        if (const auto value = context.configurationIntegerInSection("render_graph", "plan_cache_max_elements");
            value && *value >= 0 && *value <= 1'000'000)
            planCacheMaxElements = static_cast<std::size_t>(*value);
        planCache_.configure(planCacheEntries, planCacheMaxElements);

        // Execution cache reuses a prebuilt TaskGraph across frames when the
        // plan came from the cache; disabled only for diagnostics.
        const bool executionCacheEnabled
            = context.configurationBooleanInSection("render_graph", "execution_cache").value_or(true);
        executionCache_.setEnabled(executionCacheEnabled);

        profiler_ = &context.profiler();
    }

    void start() override
    {}
    void stop() override
    {}

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == RenderGraphCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "render_graph",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"render_graph"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self     = this;
        capability_.settings = [](void *self) -> const RenderGraphSettings *
        { return &static_cast<RenderGraphModule *>(self)->settings_; };
        capability_.setSettings = [](void *self, const RenderGraphSettings *value)
        { return static_cast<RenderGraphModule *>(self)->setSettingsImpl(value); };
        capability_.resolveProfile = [](void *self, const renderer::DeviceProfile *device, RenderGraphProfile *out)
        { return static_cast<RenderGraphModule *>(self)->resolveProfileImpl(device, out); };
        capability_.compile = [](void                                  *self,
                                  const RenderGraphPassDescription     *passes,
                                  std::uint32_t                         passCount,
                                  const RenderGraphResourceDescription *resources,
                                  std::uint32_t                         resourceCount,
                                  std::uint32_t                        *order,
                                  std::uint32_t                         orderCapacity,
                                  std::uint32_t                        *slots,
                                  std::uint32_t                         slotCapacity,
                                  RenderGraphCompileResult             *result,
                                  RenderGraphError                     *error)
        {
            return static_cast<RenderGraphModule *>(self)->compileImpl(
                passes, passCount, resources, resourceCount, order, orderCapacity, slots, slotCapacity, result, error);
        };
        capability_.execute = [](void                                  *self,
                                  const RenderGraphPassDescription     *passes,
                                  std::uint32_t                         passCount,
                                  const RenderGraphResourceDescription *resources,
                                  std::uint32_t                         resourceCount,
                                  RenderGraphExecuteCallback            callback,
                                  void                                 *userData,
                                  RenderGraphError                     *error)
        {
            return static_cast<RenderGraphModule *>(self)->executeImpl(
                passes, passCount, resources, resourceCount, callback, userData, error);
        };
    }

    bool setSettingsImpl(const RenderGraphSettings *value)
    {
        if (value == nullptr || !validSettings(*value))
            return false;
        settings_ = *value;
        planCache_.clear();
        return true;
    }

    // Auto quality derives a memory scale from the device class (non-Vulkan
    // gets 0.125); explicit presets scale by 0.25x/0.5x/1x/2x. Scaled limits
    // are clamped back to the configured settings.
    bool resolveProfileImpl(const renderer::DeviceProfile *device, RenderGraphProfile *out) const
    {
        if (device == nullptr || out == nullptr)
            return false;
        float memoryScale = 0.125f;
        if (device->isVulkan)
        {
            if (device->deviceClass == renderer::DeviceClass::DiscreteGpu)
                memoryScale = 1.0f;
            else if (device->deviceClass == renderer::DeviceClass::IntegratedGpu)
                memoryScale = 0.5f;
            else
                memoryScale = 0.25f;
            if (device->deviceLocalMemoryBytes != 0)
                memoryScale = std::min(memoryScale,
                    static_cast<float>(device->deviceLocalMemoryBytes) / (4.0f * 1024.0f * 1024.0f * 1024.0f));
        }
        float qualityScale = 1.0f;
        switch (settings_.quality)
        {
        case renderer::QualityPreset::Low:
            qualityScale = 0.25f;
            break;
        case renderer::QualityPreset::Medium:
            qualityScale = 0.5f;
            break;
        case renderer::QualityPreset::High:
            qualityScale = 1.0f;
            break;
        case renderer::QualityPreset::Ultra:
            qualityScale = 2.0f;
            break;
        case renderer::QualityPreset::Auto:
            break;
        }
        const float scale = settings_.quality == renderer::QualityPreset::Auto ? memoryScale : qualityScale;
        out->maxPasses = std::min(settings_.maxPasses,
            std::max(1u, static_cast<std::uint32_t>(static_cast<double>(settings_.maxPasses) * scale)));
        out->maxResources = std::min(settings_.maxResources,
            std::max(1u, static_cast<std::uint32_t>(static_cast<double>(settings_.maxResources) * scale)));
        const std::uint64_t budget = static_cast<std::uint64_t>(
            static_cast<double>(settings_.transientMemoryBudgetBytes) * scale);
        out->transientMemoryBudgetBytes = std::min(budget, settings_.transientMemoryBudgetBytes);
        out->transientAliasing          = settings_.transientAliasing;
        // asyncCompute is reserved for a future async-queue path; the field
        // stays false so consumers see a stable contract.
        out->asyncCompute = false;
        return true;
    }

    // Cache-aware plan lookup: a hit returns the cached plan (kept alive by
    // cachedPlan), a miss builds into localPlan and stores it. outPlan
    // points at whichever is used.
    bool preparePlan(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, bool includeResourceLifetimes,
        RenderGraphPlan &localPlan, std::shared_ptr<const RenderGraphPlan> &cachedPlan, const RenderGraphPlan *&outPlan,
        RenderGraphError &outError) const
    {
        if (!planCache_.enabled())
        {
            if (!buildRenderGraphPlan(
                    passes, passCount, resources, resourceCount, localPlan, outError, includeResourceLifetimes))
                return false;
            outPlan = &localPlan;
            return true;
        }

        cachedPlan = planCache_.find(passes, passCount, resources, resourceCount, includeResourceLifetimes);
        if (cachedPlan)
        {
            outPlan  = cachedPlan.get();
            outError = RenderGraphError::None;
            return true;
        }
        if (!buildRenderGraphPlan(
                passes, passCount, resources, resourceCount, localPlan, outError, includeResourceLifetimes))
            return false;

        // graphElementCount bounds the cache entry's memory; the plan already
        // knows its pass/resource/edge counts so the sum stays exact without
        // re-walking the caller's arrays.
        const std::size_t graphElementCount = static_cast<std::size_t>(passCount)
            + static_cast<std::size_t>(resourceCount) + localPlan.edges.size();
        cachedPlan = planCache_.store(
            passes, passCount, resources, resourceCount, includeResourceLifetimes, graphElementCount, localPlan);
        outPlan = cachedPlan ? cachedPlan.get() : &localPlan;
        return true;
    }

    // Validates capacities and pointers, obtains a plan with lifetimes, then
    // delegates slot assignment to the aliasing allocator.
    bool compileImpl(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, std::uint32_t *outOrder,
        std::uint32_t orderCapacity, std::uint32_t *outSlots, std::uint32_t slotCapacity,
        RenderGraphCompileResult *outResult, RenderGraphError *outError) const
    {
        ScopedProfileTimer profileTimer(*profiler_, PerformanceMetric::RenderGraphCompile);
        const auto         fail = [outError](RenderGraphError error)
        {
            if (outError != nullptr)
                *outError = error;
            return false;
        };
        if (passCount > settings_.maxPasses || resourceCount > settings_.maxResources || orderCapacity < passCount
            || slotCapacity < resourceCount)
            return fail(RenderGraphError::CapacityExceeded);
        if ((passCount != 0 && (passes == nullptr || outOrder == nullptr))
            || (resourceCount != 0 && (resources == nullptr || outSlots == nullptr)) || outResult == nullptr)
            return fail(RenderGraphError::InvalidInput);

        RenderGraphPlan                        localPlan;
        std::shared_ptr<const RenderGraphPlan> cachedPlan;
        const RenderGraphPlan                 *analysis      = nullptr;
        RenderGraphError                       analysisError = RenderGraphError::None;
        if (!preparePlan(
                passes, passCount, resources, resourceCount, true, localPlan, cachedPlan, analysis, analysisError))
            return fail(analysisError);

        detail::AliasingConfig aliasingConfig;
        aliasingConfig.aliasingEnabled           = settings_.transientAliasing;
        aliasingConfig.strategy                  = aliasingStrategy_;
        aliasingConfig.transientMemoryBudgetBytes = settings_.transientMemoryBudgetBytes;

        detail::AliasingResult aliasingResult{};
        RenderGraphError       aliasingError = RenderGraphError::None;
        if (!detail::allocateTransientSlots(
                *analysis, resources, resourceCount, outSlots, aliasingConfig, aliasingResult, aliasingError))
            return fail(aliasingError);

        std::copy(analysis->order.begin(), analysis->order.end(), outOrder);
        *outResult = RenderGraphCompileResult{
            passCount, resourceCount, aliasingResult.slotCount, aliasingResult.allocatedBytes};
        if (outError != nullptr)
            *outError = RenderGraphError::None;
        return true;
    }

    // Obtains a plan (without lifetimes) and delegates to the execution cache.
    bool executeImpl(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        RenderGraphExecuteCallback callback, void *userData, RenderGraphError *outError) const
    {
        ScopedProfileTimer profileTimer(*profiler_, PerformanceMetric::RenderGraphExecute);
        const auto         fail = [outError](RenderGraphError error)
        {
            if (outError != nullptr)
                *outError = error;
            return false;
        };
        if (scheduler_ == nullptr || callback == nullptr)
            return fail(RenderGraphError::InvalidInput);
        if (passCount > settings_.maxPasses || resourceCount > settings_.maxResources)
            return fail(RenderGraphError::CapacityExceeded);

        RenderGraphPlan                        localPlan;
        std::shared_ptr<const RenderGraphPlan> cachedPlan;
        const RenderGraphPlan                 *analysis      = nullptr;
        RenderGraphError                       analysisError = RenderGraphError::None;
        if (!preparePlan(
                passes, passCount, resources, resourceCount, false, localPlan, cachedPlan, analysis, analysisError))
            return fail(analysisError);

        RenderGraphError executeError = RenderGraphError::None;
        if (!executionCache_.execute(
                cachedPlan, *analysis, passCount, callback, userData, *scheduler_, executeError))
            return fail(executeError);
        if (outError != nullptr)
            *outError = RenderGraphError::None;
        return true;
    }

    // scheduler_/profiler_ are borrowed from the ParticipantContext in
    // construct(); the profiler feeds the RenderGraphCompile/Execute metrics.
    ParticipantManifest         manifest_;
    RenderGraphCapability       capability_{};
    RenderGraphSettings         settings_{};
    TaskScheduler              *scheduler_ = nullptr;
    PerformanceProfiler        *profiler_  = nullptr;
    detail::RenderGraphPlanCache planCache_;
    detail::ExecutionCache      executionCache_;
    detail::AliasingStrategy    aliasingStrategy_ = detail::AliasingStrategy::BestFit;
};

} // namespace
} // namespace render_graph
} // namespace engine

// Module entry table the kernel loads from the .so: factory plus the
// provided/required capability lists (kept in sync with module.toml).
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"render_graph"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "render_graph",
            .moduleVersion = "0.1.0",
            .createModule = []() -> engine::ModuleInterface * { return new engine::render_graph::RenderGraphModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
