// Compiled-plan LRU cache implementation.

#include "internal/plan_cache.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace engine::render_graph::detail
{

void RenderGraphPlanCache::configure(std::size_t entryCapacity, std::size_t maximumGraphElements) noexcept
{
    std::lock_guard lock(mutex_);
    entries_.clear();
    capacity_             = std::min<std::size_t>(entryCapacity, 16u);
    maximumGraphElements_ = maximumGraphElements;
    try
    {
        entries_.reserve(capacity_);
    }
    catch (...)
    {
        capacity_ = 0;
    }
}

void RenderGraphPlanCache::clear() noexcept
{
    std::lock_guard lock(mutex_);
    entries_.clear();
}

bool RenderGraphPlanCache::enabled() const noexcept
{
    return capacity_ != 0 && maximumGraphElements_ != 0;
}

std::shared_ptr<const RenderGraphPlan> RenderGraphPlanCache::find(const RenderGraphPassDescription *passes,
    std::uint32_t passCount, const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
    bool includeResourceLifetimes) const noexcept
{
    if (capacity_ == 0 || (passCount != 0 && passes == nullptr) || (resourceCount != 0 && resources == nullptr))
        return {};
    std::lock_guard lock(mutex_);
    for (Entry &entry : entries_)
    {
        if (!matches(entry, passes, passCount, resources, resourceCount, includeResourceLifetimes))
            continue;
        entry.lastUse = ++clock_;
        return entry.plan;
    }
    return {};
}

std::shared_ptr<const RenderGraphPlan> RenderGraphPlanCache::store(const RenderGraphPassDescription *passes,
    std::uint32_t passCount, const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
    bool includeResourceLifetimes, std::size_t graphElementCount, RenderGraphPlan &plan) const noexcept
{
    if (capacity_ == 0 || graphElementCount > maximumGraphElements_ || (passCount != 0 && passes == nullptr)
        || (resourceCount != 0 && resources == nullptr))
        return {};

    Entry entry;
    try
    {
        entry.includeResourceLifetimes = includeResourceLifetimes;
        entry.passShapes.reserve(passCount);
        if (resourceCount != 0)
            entry.resources.assign(resources, resources + resourceCount);
        for (std::uint32_t pass = 0; pass < passCount; ++pass)
        {
            const RenderGraphPassDescription &description = passes[pass];
            if ((description.useCount != 0 && description.uses == nullptr)
                || (description.dependencyCount != 0 && description.dependencies == nullptr))
                return {};
            PassShape shape;
            shape.useOffset        = entry.uses.size();
            shape.useCount         = description.useCount;
            shape.dependencyOffset = entry.dependencies.size();
            shape.dependencyCount  = description.dependencyCount;
            entry.passShapes.push_back(shape);
            if (description.useCount != 0)
                entry.uses.insert(entry.uses.end(), description.uses, description.uses + description.useCount);
            if (description.dependencyCount != 0)
                entry.dependencies.insert(entry.dependencies.end(),
                    description.dependencies,
                    description.dependencies + description.dependencyCount);
        }
        entry.plan = std::make_shared<RenderGraphPlan>(std::move(plan));
    }
    catch (...)
    {
        return {};
    }

    std::lock_guard lock(mutex_);
    for (Entry &existing : entries_)
    {
        if (matches(existing, passes, passCount, resources, resourceCount, includeResourceLifetimes))
        {
            existing.lastUse = ++clock_;
            return existing.plan;
        }
    }
    entry.lastUse = ++clock_;
    if (entries_.size() >= capacity_)
    {
        const auto oldest = std::min_element(entries_.begin(),
            entries_.end(),
            [](const Entry &left, const Entry &right) { return left.lastUse < right.lastUse; });
        if (oldest != entries_.end())
            entries_.erase(oldest);
    }
    const std::shared_ptr<const RenderGraphPlan> result = entry.plan;
    try
    {
        entries_.push_back(std::move(entry));
    }
    catch (...)
    {
        return result;
    }
    return result;
}

bool RenderGraphPlanCache::sameResource(
    const RenderGraphResourceDescription &left, const RenderGraphResourceDescription &right) noexcept
{
    return left.sizeBytes == right.sizeBytes && left.alignmentBytes == right.alignmentBytes
        && left.compatibilityClass == right.compatibilityClass && left.transient == right.transient
        && left.initiallyDefined == right.initiallyDefined;
}

bool RenderGraphPlanCache::sameUse(const RenderGraphResourceUse &left, const RenderGraphResourceUse &right) noexcept
{
    return left.resourceIndex == right.resourceIndex && left.access == right.access;
}

bool RenderGraphPlanCache::matches(const Entry &entry, const RenderGraphPassDescription *passes,
    std::uint32_t passCount, const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
    bool includeResourceLifetimes) noexcept
{
    if (entry.includeResourceLifetimes != includeResourceLifetimes || entry.passShapes.size() != passCount
        || entry.resources.size() != resourceCount || (passCount != 0 && passes == nullptr)
        || (resourceCount != 0 && resources == nullptr))
        return false;
    for (std::uint32_t index = 0; index < resourceCount; ++index)
        if (!sameResource(entry.resources[index], resources[index]))
            return false;
    for (std::uint32_t index = 0; index < passCount; ++index)
    {
        const RenderGraphPassDescription &description = passes[index];
        const PassShape                  &shape       = entry.passShapes[index];
        if (description.useCount != shape.useCount || description.dependencyCount != shape.dependencyCount
            || (description.useCount != 0 && description.uses == nullptr)
            || (description.dependencyCount != 0 && description.dependencies == nullptr))
            return false;
        for (std::uint32_t use = 0; use < shape.useCount; ++use)
            if (!sameUse(entry.uses[shape.useOffset + use], description.uses[use]))
                return false;
        for (std::uint32_t dependency = 0; dependency < shape.dependencyCount; ++dependency)
            if (entry.dependencies[shape.dependencyOffset + dependency] != description.dependencies[dependency])
                return false;
    }
    return true;
}

} // namespace engine::render_graph::detail
