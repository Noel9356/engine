#ifndef ENGINE_KERNEL_COMPONENT_INTERFACE_HPP
#define ENGINE_KERNEL_COMPONENT_INTERFACE_HPP

#include <engine/kernel/component_manifest.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/main_thread_scheduler.hpp>
#include <engine/kernel/permission_system.hpp>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace engine
{

struct CapabilityNameHash
{
    using is_transparent = void;

    std::size_t operator()(std::string_view text) const noexcept
    {
        return std::hash<std::string_view>{}(text);
    }
};

class ConfigurationStore;

struct ScriptModDescription
{
    std::string           modId;
    std::filesystem::path scriptPath;

    std::string hostCapability;

    // What the mod declared in mod.toml. The host module owns the script
    // sandbox: these lists are the per-mod grant surface the host is expected
    // to enforce (a capability is callable only when listed here; a
    // permission-gated bridge function checks the permission list).
    std::vector<std::string> declaredCapabilities;

    std::vector<std::string> declaredPermissions;

    std::string version;
};

class ScriptModHost
{
public:
    virtual ~ScriptModHost() = default;

    virtual void hostScriptMod(const ScriptModDescription &description) = 0;
};

class ComponentAccessError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct ConfigurationReloadedEvent
{
    std::vector<std::string> changedKeys;
};

struct EngineFrameEvent
{
    std::uint64_t frameIndex     = 0;
    double        deltaSeconds   = 0.0;
    double        elapsedSeconds = 0.0;
};

struct EngineFixedUpdateEvent
{
    double deltaSeconds = 0.0;
};

class CapabilityResolver
{
public:
    virtual ~CapabilityResolver() = default;

    virtual void *resolveCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) = 0;

    virtual std::vector<void *> resolveEachCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) = 0;

    virtual bool hasSingleCapabilityProvider(
        const std::string &capabilityName,
        ComponentKind      requesterKind
    ) = 0;

    virtual void *tryResolveCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) = 0;
};

class ComponentContext final
{
public:
    ComponentContext(
        EventBus                 &eventBus,
        CapabilityResolver       &capabilityResolver,
        PermissionSystem         &permissionSystem,
        const ConfigurationStore &configuration,
        std::string               componentName,
        ComponentKind             componentKind,
        const std::unordered_set<
            std::string,
            CapabilityNameHash,
            std::equal_to<>> *demandedCapabilities,
        MainThreadScheduler  &mainThreadScheduler
    );

    EventBus &eventBus()
    {
        return eventBus_;
    }

    void *acquireCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionScratch_))
        {
            const std::string permissionName = permissionScratch_;
            throw ComponentAccessError(
                "component '" + componentName_ + "' was blocked by the permission system while acquiring '"
                + capabilityName + "': the permission '" + permissionName
                + "' was not granted. Declare the requirement in the manifest."
            );
        }
        return capabilityResolver_.resolveCapabilityPointer(componentName_, componentKind_, capabilityName);
    }

    template <typename CapabilityType>
    CapabilityType &acquireCapability() const
    {
        void *capabilityPointer = acquireCapability(std::string(CapabilityType::capabilityName));
        return *static_cast<CapabilityType *>(capabilityPointer);
    }

    std::vector<void *> acquireEachCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionScratch_))
        {
            const std::string permissionName = permissionScratch_;
            throw ComponentAccessError(
                "component '" + componentName_ + "' was blocked by the permission system while acquiring '"
                + capabilityName + "': the permission '" + permissionName
                + "' was not granted. Declare the requirement in the manifest."
            );
        }
        return capabilityResolver_.resolveEachCapabilityPointer(componentName_, componentKind_, capabilityName);
    }

    template <typename CapabilityType>
    std::vector<CapabilityType *> acquireEachCapability() const
    {
        std::vector<void *> capabilityPointers = acquireEachCapability(std::string(CapabilityType::capabilityName));
        std::vector<CapabilityType *> typed;
        typed.reserve(capabilityPointers.size());
        for (void *capabilityPointer : capabilityPointers)
        {
            typed.push_back(static_cast<CapabilityType *>(capabilityPointer));
        }
        return typed;
    }

    void *tryAcquireCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionScratch_))
        {
            return nullptr;
        }
        return capabilityResolver_.tryResolveCapabilityPointer(componentName_, componentKind_, capabilityName);
    }

    template <typename CapabilityType>
    CapabilityType *tryAcquireCapability() const
    {
        void *capabilityPointer = tryAcquireCapability(std::string(CapabilityType::capabilityName));
        return capabilityPointer != nullptr ? static_cast<CapabilityType *>(capabilityPointer) : nullptr;
    }

    MainThreadScheduler &scheduler()
    {
        return mainThreadScheduler_;
    }

    bool isCapabilityDemanded(std::string_view capabilityName) const
    {
        return demandedCapabilities_ != nullptr
            && demandedCapabilities_->find(capabilityName) != demandedCapabilities_->end();
    }

    bool isCapabilityAvailable(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(componentName_, permissionScratch_))
        {
            return false;
        }
        return capabilityResolver_.hasSingleCapabilityProvider(capabilityName, componentKind_);
    }

    std::optional<std::string>  configurationString(const std::string &key) const;
    std::optional<std::int64_t> configurationInteger(const std::string &key) const;
    std::optional<bool>         configurationBoolean(const std::string &key) const;
    std::optional<double>       configurationFloating(const std::string &key) const;

    std::optional<std::string> configurationStringInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::vector<std::string> configurationStringArrayInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::optional<std::int64_t> configurationIntegerInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::optional<bool> configurationBooleanInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::optional<double> configurationFloatingInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::vector<double> configurationFloatingArrayInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::size_t configurationArraySizeInSection(
        const std::string &section,
        const std::string &path
    ) const;

    std::vector<std::string> configurationTableKeysInSection(
        const std::string &section,
        const std::string &path
    ) const;

private:
    EventBus                 &eventBus_;
    CapabilityResolver       &capabilityResolver_;
    PermissionSystem         &permissionSystem_;
    const ConfigurationStore &configuration_;
    std::string               componentName_;

    ComponentKind componentKind_ = ComponentKind::Application;
    const std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>> *demandedCapabilities_ = nullptr;
    MainThreadScheduler                                                        &mainThreadScheduler_;

    mutable std::string permissionScratch_;
};

class ModuleInterface
{
public:
    virtual ~ModuleInterface() = default;

    virtual const ComponentManifest &manifest() const = 0;

    virtual void construct(ComponentContext &context) = 0;
    virtual void start()                              = 0;
    virtual void stop()                               = 0;

    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;

    virtual ScriptModHost *scriptModHost()
    {
        return nullptr;
    }
};

class ModInterface
{
public:
    virtual ~ModInterface() = default;

    virtual const ComponentManifest &manifest() const = 0;

    virtual void construct(ComponentContext &context) = 0;
    virtual void start()                              = 0;
    virtual void stop()                               = 0;

    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;
};

} // namespace engine

#endif
