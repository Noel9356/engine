#ifndef ENGINE_KERNEL_CONFIGURATION_WATCH_HPP
#define ENGINE_KERNEL_CONFIGURATION_WATCH_HPP

#include <engine/kernel/component_interface.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

namespace engine
{

class ConfigurationWatchHub;

class ConfigurationWatch final
{
public:
    ConfigurationWatch() = default;

    ConfigurationWatch(
        ConfigurationWatchHub *ownerHub,
        std::uint64_t          watchId
    )
        : owner(ownerHub),
          watchId(watchId)
    {}

    ~ConfigurationWatch()
    {
        release();
    }

    ConfigurationWatch(const ConfigurationWatch &)            = delete;
    ConfigurationWatch &operator=(const ConfigurationWatch &) = delete;

    ConfigurationWatch(ConfigurationWatch &&other) noexcept
        : owner(other.owner),
          watchId(other.watchId)
    {
        other.owner   = nullptr;
        other.watchId = invalidWatchId;
    }

    ConfigurationWatch &operator=(ConfigurationWatch &&other) noexcept
    {
        if (this != &other)
        {
            release();
            owner         = other.owner;
            watchId       = other.watchId;
            other.owner   = nullptr;
            other.watchId = invalidWatchId;
        }
        return *this;
    }

    void release();

    bool valid() const
    {
        return owner != nullptr && watchId != invalidWatchId;
    }

private:
    static constexpr std::uint64_t invalidWatchId = 0;

    ConfigurationWatchHub *owner   = nullptr;
    std::uint64_t          watchId = invalidWatchId;
};

class ConfigurationWatchHub final
{
public:
    ConfigurationWatchHub() = default;

    explicit ConfigurationWatchHub(EventBus &eventBus)
    {
        bind(eventBus);
    }

    ~ConfigurationWatchHub();

    ConfigurationWatchHub(const ConfigurationWatchHub &)            = delete;
    ConfigurationWatchHub &operator=(const ConfigurationWatchHub &) = delete;

    ConfigurationWatchHub(ConfigurationWatchHub &&other) noexcept;

    ConfigurationWatchHub &operator=(ConfigurationWatchHub &&other) noexcept;

    void bind(EventBus &eventBus);

    [[nodiscard]] ConfigurationWatch watch(
        const std::string                       &dottedKeyPath,
        std::function<void(const std::string &)> callback
    );

private:
    friend class ConfigurationWatch;

    struct WatchEntry
    {
        std::uint64_t                            id = 0;
        std::string                              key;
        std::function<void(const std::string &)> callback;
    };

    void releaseWatch(std::uint64_t watchId);

    void handleReloaded(const ConfigurationReloadedEvent &event);

    EventBus               *bus = nullptr;
    EventSubscription       reloadSubscription;
    std::vector<WatchEntry> watches;
    std::uint64_t           nextWatchId = 1;
};

inline ConfigurationWatchHub::~ConfigurationWatchHub()
{
    watches.clear();
}

inline ConfigurationWatchHub::ConfigurationWatchHub(ConfigurationWatchHub &&other) noexcept
    : bus(other.bus),
      reloadSubscription(std::move(other.reloadSubscription)),
      watches(std::move(other.watches)),
      nextWatchId(other.nextWatchId)
{
    other.bus         = nullptr;
    other.nextWatchId = 1;
}

inline ConfigurationWatchHub &ConfigurationWatchHub::operator=(ConfigurationWatchHub &&other) noexcept
{
    if (this != &other)
    {
        watches.clear();
        reloadSubscription.release();
        bus                = other.bus;
        reloadSubscription = std::move(other.reloadSubscription);
        watches            = std::move(other.watches);
        nextWatchId        = other.nextWatchId;
        other.bus          = nullptr;
        other.nextWatchId  = 1;
    }
    return *this;
}

inline void ConfigurationWatchHub::bind(EventBus &eventBus)
{
    reloadSubscription.release();
    bus                = &eventBus;
    reloadSubscription = EventSubscription(
        &eventBus,
        typeid(ConfigurationReloadedEvent),
        eventBus.channel<ConfigurationReloadedEvent>()
            .subscribe<ConfigurationWatchHub, &ConfigurationWatchHub::handleReloaded>(this)
    );
}

inline ConfigurationWatch ConfigurationWatchHub::watch(
    const std::string                       &dottedKeyPath,
    std::function<void(const std::string &)> callback
)
{
    const std::uint64_t watchIdentifier = nextWatchId++;
    WatchEntry          entry;
    entry.id       = watchIdentifier;
    entry.key      = dottedKeyPath;
    entry.callback = std::move(callback);
    watches.push_back(std::move(entry));
    return ConfigurationWatch(this, watchIdentifier);
}

inline void ConfigurationWatchHub::releaseWatch(std::uint64_t watchId)
{
    for (std::size_t index = 0; index < watches.size(); ++index)
    {
        if (watches[index].id == watchId)
        {
            watches.erase(watches.begin() + static_cast<std::ptrdiff_t>(index));
            return;
        }
    }
}

inline void ConfigurationWatchHub::handleReloaded(const ConfigurationReloadedEvent &event)
{
    std::vector<std::uint64_t> ids;
    ids.reserve(watches.size());
    for (const WatchEntry &entry : watches)
    {
        ids.push_back(entry.id);
    }
    for (const std::uint64_t id : ids)
    {
        const auto stillHeld
            = std::find_if(watches.begin(), watches.end(), [id](const WatchEntry &entry) { return entry.id == id; });
        if (stillHeld == watches.end())
        {
            continue;
        }
        if (std::find(event.changedKeys.begin(), event.changedKeys.end(), stillHeld->key) != event.changedKeys.end())
        {
            stillHeld->callback(stillHeld->key);
        }
    }
}

inline void ConfigurationWatch::release()
{
    if (owner != nullptr && watchId != invalidWatchId)
    {
        owner->releaseWatch(watchId);
    }
    owner   = nullptr;
    watchId = invalidWatchId;
}

} // namespace engine

#endif
