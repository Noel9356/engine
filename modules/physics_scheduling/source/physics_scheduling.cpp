// physics_scheduling module implementation.
//
// The module owns handle-keyed schedules of consumer-registered items and
// exposes the SchedulingCapability function table. Every schedule advances
// one fixed step per updateSchedule call: the due set comes from a staggered
// modulo test (item slot index spreads the items of one interval evenly
// across steps), and the fidelity level transitions follow the importance
// score through the configured threshold ladder with demotion hysteresis.
// There is no stepping order dependence beyond the item vector's
// registration order, no map iteration, and no allocation after the item
// budget fills, which keeps identical input sequences bit-identical.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_scheduling/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_scheduling
{
namespace
{

struct ItemRecord
{
    std::uint64_t id = 0;

    // Registration ordinal; the stagger phase derives from it and stays
    // stable across removals of other items.
    std::uint64_t slotIndex = 0;

    std::uint32_t updateInterval = 1;
    bool          due            = false;

    FidelityLevel level      = FidelityLevel::Full;
    double        importance = 0.0;
};

struct ScheduleRecord
{
    ScheduleHandle handle = invalidSchedule;

    ScheduleDescription description{};

    std::vector<ItemRecord>                      items;
    std::unordered_map<std::uint64_t, std::size_t> itemIndexById;
    std::vector<std::uint64_t>                   dueIds;

    std::uint64_t nextSlotIndex = 0;
    std::uint64_t stepIndex     = 0;
    std::uint32_t dueCountLastUpdate = 0;
};

struct SchedulingSettings
{
    std::uint32_t maximumSchedules       = 16;
    std::uint32_t maximumItemsPerSchedule = 65536;
};

class SchedulingModule final : public ModuleInterface
{
public:
    SchedulingModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        if (const auto value = context.configurationIntegerInSection("physics_scheduling", "maximum_schedules");
            value && *value > 0 && *value <= 4096)
            settings_.maximumSchedules = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_scheduling", "maximum_items_per_schedule");
            value && *value > 0 && *value <= 4'000'000)
            settings_.maximumItemsPerSchedule = static_cast<std::uint32_t>(*value);
        schedules_.reserve(settings_.maximumSchedules);
    }

    void start() override
    {}

    void stop() override
    {
        schedules_.clear();
        scheduleIndexByHandle_.clear();
        nextScheduleHandle_ = 1;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == SchedulingCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_scheduling",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_scheduling"}};
    }

    static void setError(SchedulingError *output, SchedulingError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
    }

    ScheduleRecord *findSchedule(ScheduleHandle handle)
    {
        const auto iterator = scheduleIndexByHandle_.find(handle);
        return iterator != scheduleIndexByHandle_.end() ? &schedules_[iterator->second] : nullptr;
    }

    static bool validateScheduleDescription(const ScheduleDescription &description)
    {
        return description.defaultUpdateInterval > 0u && description.defaultUpdateInterval <= 65536u
            && std::isfinite(description.fullThreshold) && std::isfinite(description.simplifiedThreshold)
            && std::isfinite(description.proceduralThreshold) && std::isfinite(description.animationThreshold)
            && description.fullThreshold > description.simplifiedThreshold
            && description.simplifiedThreshold > description.proceduralThreshold
            && description.proceduralThreshold > description.animationThreshold
            && description.animationThreshold > 0.0 && description.animationThreshold <= 1.0
            && description.fullThreshold <= 1.0 && std::isfinite(description.demotionMargin)
            && description.demotionMargin >= 0.0 && description.demotionMargin < 1.0;
    }

    // The entry threshold that granted `level`: scores below the threshold
    // times the demotion margin release the item back down the ladder.
    static double levelEntryThreshold(const ScheduleDescription &description, FidelityLevel level)
    {
        switch (level)
        {
        case FidelityLevel::Full:
            return description.fullThreshold;
        case FidelityLevel::Simplified:
            return description.simplifiedThreshold;
        case FidelityLevel::Procedural:
            return description.proceduralThreshold;
        case FidelityLevel::Animation:
            return description.animationThreshold;
        case FidelityLevel::Cached:
        default:
            return 0.0;
        }
    }

    static FidelityLevel levelForScore(const ScheduleDescription &description, double score)
    {
        if (score >= description.fullThreshold)
        {
            return FidelityLevel::Full;
        }
        if (score >= description.simplifiedThreshold)
        {
            return FidelityLevel::Simplified;
        }
        if (score >= description.proceduralThreshold)
        {
            return FidelityLevel::Procedural;
        }
        if (score >= description.animationThreshold)
        {
            return FidelityLevel::Animation;
        }
        return FidelityLevel::Cached;
    }

    // ---- capability implementations ---------------------------------------

    ScheduleHandle createScheduleImpl(const ScheduleDescription *description, SchedulingError *error)
    {
        setError(error, SchedulingError::None);
        if (description == nullptr || !validateScheduleDescription(*description))
        {
            setError(error, SchedulingError::InvalidDescription);
            return invalidSchedule;
        }
        if (schedules_.size() >= settings_.maximumSchedules)
        {
            setError(error, SchedulingError::CapacityExceeded);
            return invalidSchedule;
        }
        if (nextScheduleHandle_ == invalidSchedule)
        {
            setError(error, SchedulingError::CapacityExceeded);
            return invalidSchedule;
        }
        ScheduleRecord schedule;
        schedule.handle      = nextScheduleHandle_;
        nextScheduleHandle_  = nextScheduleHandle_ == std::numeric_limits<ScheduleHandle>::max()
            ? invalidSchedule
            : nextScheduleHandle_ + 1;
        schedule.description = *description;
        schedules_.push_back(std::move(schedule));
        scheduleIndexByHandle_.emplace(schedules_.back().handle, schedules_.size() - 1);
        return schedules_.back().handle;
    }

    void destroyScheduleImpl(ScheduleHandle handle)
    {
        const auto iterator = scheduleIndexByHandle_.find(handle);
        if (iterator == scheduleIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        scheduleIndexByHandle_.erase(iterator);
        if (index + 1 != schedules_.size())
        {
            std::swap(schedules_[index], schedules_.back());
            scheduleIndexByHandle_[schedules_[index].handle] = index;
        }
        schedules_.pop_back();
    }

    bool addItemImpl(ScheduleHandle handle, const ItemDescription *description, SchedulingError *error)
    {
        setError(error, SchedulingError::None);
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr)
        {
            setError(error, SchedulingError::InvalidHandle);
            return false;
        }
        if (description == nullptr || !std::isfinite(description->importance) || description->importance < 0.0)
        {
            setError(error, SchedulingError::InvalidDescription);
            return false;
        }
        if (description->updateInterval > 65536u)
        {
            setError(error, SchedulingError::InvalidDescription);
            return false;
        }
        if (schedule->items.size() >= settings_.maximumItemsPerSchedule)
        {
            setError(error, SchedulingError::CapacityExceeded);
            return false;
        }
        if (schedule->itemIndexById.find(description->id) != schedule->itemIndexById.end())
        {
            setError(error, SchedulingError::InvalidDescription);
            return false;
        }
        ItemRecord item;
        item.id             = description->id;
        item.slotIndex      = schedule->nextSlotIndex++;
        item.updateInterval = description->updateInterval != 0u ? description->updateInterval
                                                                : schedule->description.defaultUpdateInterval;
        item.importance     = std::clamp(description->importance, 0.0, 1.0);
        item.level          = levelForScore(schedule->description, item.importance);
        item.due            = (schedule->stepIndex + item.slotIndex) % item.updateInterval == 0u;
        schedule->items.push_back(std::move(item));
        schedule->itemIndexById.emplace(schedule->items.back().id, schedule->items.size() - 1);
        return true;
    }

    bool removeItemImpl(ScheduleHandle handle, std::uint64_t id)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr)
        {
            return false;
        }
        const auto iterator = schedule->itemIndexById.find(id);
        if (iterator == schedule->itemIndexById.end())
        {
            return false;
        }
        const std::size_t index = iterator->second;
        schedule->itemIndexById.erase(iterator);
        if (index + 1 != schedule->items.size())
        {
            schedule->items[index] = schedule->items.back();
            schedule->itemIndexById[schedule->items[index].id] = index;
        }
        schedule->items.pop_back();
        return true;
    }

    ItemRecord *findItem(ScheduleRecord *schedule, std::uint64_t id)
    {
        const auto iterator = schedule->itemIndexById.find(id);
        return iterator != schedule->itemIndexById.end() ? &schedule->items[iterator->second] : nullptr;
    }

    bool setItemUpdateIntervalImpl(ScheduleHandle handle, std::uint64_t id, std::uint32_t interval)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || interval == 0u || interval > 65536u)
        {
            return false;
        }
        ItemRecord *item = findItem(schedule, id);
        if (item == nullptr)
        {
            return false;
        }
        item->updateInterval = interval;
        item->due            = (schedule->stepIndex + item->slotIndex) % interval == 0u;
        return true;
    }

    bool setItemImportanceImpl(ScheduleHandle handle, std::uint64_t id, double importance)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        ItemRecord *item = findItem(schedule, id);
        if (item == nullptr)
        {
            return false;
        }
        item->importance = std::clamp(importance, 0.0, 1.0);
        return true;
    }

    std::uint32_t updateScheduleImpl(ScheduleHandle handle)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr)
        {
            return 0u;
        }
        ++schedule->stepIndex;
        schedule->dueIds.clear();
        for (ItemRecord &item : schedule->items)
        {
            item.due = (schedule->stepIndex + item.slotIndex) % item.updateInterval == 0u;

            // Fidelity ladder with demotion hysteresis: promotions apply
            // immediately, demotions wait until the score falls below the
            // current level's entry threshold times the margin.
            const FidelityLevel target = levelForScore(schedule->description, item.importance);
            if (target > item.level)
            {
                item.level = target;
            }
            else if (target < item.level)
            {
                const double releaseAt
                    = levelEntryThreshold(schedule->description, item.level) * schedule->description.demotionMargin;
                if (item.importance < releaseAt)
                {
                    item.level = target;
                }
            }

            if (item.due)
            {
                schedule->dueIds.push_back(item.id);
            }
        }
        schedule->dueCountLastUpdate = static_cast<std::uint32_t>(schedule->dueIds.size());
        return schedule->dueCountLastUpdate;
    }

    bool isItemDueImpl(ScheduleHandle handle, std::uint64_t id)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr)
        {
            return false;
        }
        const ItemRecord *item = findItem(schedule, id);
        return item != nullptr ? item->due : false;
    }

    bool itemLevelImpl(ScheduleHandle handle, std::uint64_t id, FidelityLevel *outLevel)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || outLevel == nullptr)
        {
            return false;
        }
        const ItemRecord *item = findItem(schedule, id);
        if (item == nullptr)
        {
            return false;
        }
        *outLevel = item->level;
        return true;
    }

    bool itemCountImpl(ScheduleHandle handle, std::uint32_t *outCount)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || outCount == nullptr)
        {
            return false;
        }
        *outCount = static_cast<std::uint32_t>(schedule->items.size());
        return true;
    }

    bool copyItemStatesImpl(
        ScheduleHandle handle, ItemState *outStates, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || outRequired == nullptr)
        {
            return false;
        }
        *outRequired = static_cast<std::uint32_t>(schedule->items.size());
        if (outStates == nullptr || capacity < schedule->items.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < schedule->items.size(); ++index)
        {
            const ItemRecord &item = schedule->items[index];
            ItemState         state;
            state.id             = item.id;
            state.due            = item.due;
            state.updateInterval = item.updateInterval;
            state.level          = item.level;
            state.importance     = item.importance;
            outStates[index]     = state;
        }
        return true;
    }

    bool copyDueItemsImpl(
        ScheduleHandle handle, std::uint64_t *outIds, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || outRequired == nullptr)
        {
            return false;
        }
        *outRequired = schedule->dueCountLastUpdate;
        if (outIds == nullptr || capacity < schedule->dueCountLastUpdate)
        {
            return false;
        }
        for (std::uint32_t index = 0; index < schedule->dueCountLastUpdate; ++index)
        {
            outIds[index] = schedule->dueIds[index];
        }
        return true;
    }

    bool scheduleStatisticsImpl(ScheduleHandle handle, ScheduleStatistics *outStatistics)
    {
        ScheduleRecord *schedule = findSchedule(handle);
        if (schedule == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        ScheduleStatistics statistics;
        statistics.itemCount      = static_cast<std::uint32_t>(schedule->items.size());
        statistics.dueItemCount   = schedule->dueCountLastUpdate;
        for (const ItemRecord &item : schedule->items)
        {
            ++statistics.levelCounts[static_cast<std::uint32_t>(item.level)];
        }
        *outStatistics = statistics;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsSchedulingCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_SCHEDULING_BIND(field, method)                                                                  \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<SchedulingModule *>(self)->method(args...); }
        ENGINE_PHYSICS_SCHEDULING_BIND(createSchedule, createScheduleImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(destroySchedule, destroyScheduleImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(addItem, addItemImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(removeItem, removeItemImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(setItemUpdateInterval, setItemUpdateIntervalImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(setItemImportance, setItemImportanceImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(updateSchedule, updateScheduleImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(isItemDue, isItemDueImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(itemLevel, itemLevelImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(itemCount, itemCountImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(copyItemStates, copyItemStatesImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(copyDueItems, copyDueItemsImpl);
        ENGINE_PHYSICS_SCHEDULING_BIND(scheduleStatistics, scheduleStatisticsImpl);
#undef ENGINE_PHYSICS_SCHEDULING_BIND
    }

    ParticipantManifest    manifest_;
    SchedulingCapability   capability_{};

    SchedulingSettings settings_;

    std::vector<ScheduleRecord>                            schedules_;
    std::unordered_map<ScheduleHandle, std::size_t>        scheduleIndexByHandle_;
    ScheduleHandle                                         nextScheduleHandle_ = 1;
};

} // namespace
} // namespace engine::physics_scheduling

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_scheduling capability; no required capabilities, no
// permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_scheduling"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion    = engine::engineModuleAbiVersion,
        .moduleName    = "physics_scheduling",
        .moduleVersion = "0.1.0",
        .createModule  = []() -> engine::ModuleInterface *{ return new engine::physics_scheduling::SchedulingModule(); },
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
