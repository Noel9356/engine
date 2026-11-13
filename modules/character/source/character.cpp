// character module implementation (module class and capability surface).
//
// Implements the "character" capability behind the module contract: a content
// registry (parameters, compatibility rules, rig fragments, body parts,
// assembly profiles), live character assemblies with a validated attachment
// tree, and versioned TOML recipes (serialization.cpp). Domain work lives in
// focused translation units - registry.cpp (definition copying and indexed
// lookups), validation.cpp (the mandatory check suite), evaluation.cpp
// (layered transform evaluation), configuration.cpp ([character] budgets) -
// so this file only coordinates state, validation policy, error reporting,
// change events, and the capability bindings. An optional Lua bridge
// (lua_bridge.cpp) exposes the assembly operations to script runtimes.
//
// Transform model (column vectors, column-major matrices, right-handed world
// space in meters):
//
//   T_child_world = T_parent_world * T_parentSocket_local * T_offset
//                   * inverse(T_childSocket_local)
//   M_instance_world = T_instance_rigid * S_parameters
//
// Attachment hierarchy is deliberately separate from skeleton hierarchy:
// each part keeps an independent rig fragment, and joint-space connector
// anchors resolve against the owning part's fragment only. Skeletons are
// never merged.

#include "configuration.hpp"
#include "evaluation.hpp"
#include "lua_bridge.hpp"
#include "math_utils.hpp"
#include "records.hpp"
#include "registry.hpp"
#include "validation.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <unordered_set>
#include <utility>

namespace engine::character
{
namespace
{

using internal::BindingRecord;
using internal::Budgets;
using internal::CharacterRecord;
using internal::CompatibilityRuleRecord;
using internal::ConnectionRecord;
using internal::ConnectorRecord;
using internal::MaterialSlotRecord;
using internal::ParameterRecord;
using internal::PartInstance;
using internal::PartRecord;
using internal::ProfileRecord;
using internal::Registry;
using internal::RigJointRecord;
using internal::RigRecord;

} // namespace

// The module: content registry + live assemblies behind the capability.
// Mutation and query entry points serialize on one mutex; registration,
// validation, and evaluation stages live in their own translation units.
class CharacterModule final : public ModuleInterface
{
public:
    CharacterModule()
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
        // Reads [character] budgets and subscribes for configuration hot
        // reload; acquires script_runtime for the optional Lua bridge.
        context_ = &context;
        {
            std::lock_guard stateLock(stateMutex_);
            internal::readBudgets(context, budgets_);
        }

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.character     = &capability_;
            luaBridgeContext_.mutex         = &stateMutex_;
            runtimeInitializer_             = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeCharacterLuaRuntime, &luaBridgeContext_);
        }

        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<CharacterModule, &CharacterModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
        {
            std::lock_guard stateLock(stateMutex_);
            characters_.clear();
            registry_                   = Registry{};
            luaBridgeContext_.character = nullptr;
        }
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == CharacterCapability::capabilityName)
        {
            return &capability_;
        }
        return nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        ParticipantManifest manifest;
        manifest.name                 = "character";
        manifest.kind                 = ParticipantKind::EngineModule;
        manifest.version              = "0.1.0";
        manifest.providedCapabilities = {"character"};
        manifest.requiredCapabilities = {};
        manifest.optionalCapabilities = {"script_runtime"};
        return manifest;
    }

    // Re-reads the budgets when any changed key starts with "character.";
    // content that already exists keeps its data, only new registrations and
    // assemblies respect the refreshed limits.
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("character.", 0) == 0; });
        if (!changed)
        {
            return;
        }
        std::lock_guard stateLock(stateMutex_);
        if (context_ != nullptr)
        {
            internal::readBudgets(*context_, budgets_);
        }
    }

    // ---- content registration --------------------------------------------

    bool registerParameterImpl(const ParameterDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !internal::stringNonEmpty(definition->id))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.parameters.size() >= budgets_.maximumParameters)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.parameterIndices.find(std::string_view(definition->id)) != registry_.parameterIndices.end())
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (!(definition->minimum <= definition->defaultV && definition->defaultV <= definition->maximum)
            || !internal::finiteFloat(definition->minimum) || !internal::finiteFloat(definition->maximum)
            || !internal::finiteFloat(definition->defaultV))
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        ParameterRecord record;
        record.id          = definition->id;
        record.displayName = internal::copyString(definition->displayName);
        if (record.displayName.empty())
        {
            record.displayName = record.id;
        }
        record.minimum  = definition->minimum;
        record.maximum  = definition->maximum;
        record.defaultV = definition->defaultV;
        registry_.parameterIndices.emplace(record.id, registry_.parameters.size());
        registry_.parameters.push_back(std::move(record));

        // New parameters become available to existing assemblies at their
        // default value (parameter values are registry-index aligned).
        for (auto &entry : characters_)
        {
            entry.second->parameterValues.push_back(registry_.parameters.back().defaultV);
        }
        return succeed(error);
    }

    bool registerCompatibilityRuleImpl(const CompatibilityRuleDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !internal::stringNonEmpty(definition->id)
            || !internal::stringNonEmpty(definition->parentRole) || !internal::stringNonEmpty(definition->childRole))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.rules.size() >= budgets_.maximumRules)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.ruleIdIndices.find(std::string_view(definition->id)) != registry_.ruleIdIndices.end())
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (!internal::finiteFloat(definition->minimumRelativeScale)
            || !internal::finiteFloat(definition->maximumRelativeScale)
            || definition->minimumRelativeScale <= 0.0f
            || definition->minimumRelativeScale > definition->maximumRelativeScale)
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        CompatibilityRuleRecord record;
        record.id                   = definition->id;
        record.parentRole           = definition->parentRole;
        record.childRole            = definition->childRole;
        record.allowed              = definition->allowed;
        record.minimumRelativeScale = definition->minimumRelativeScale;
        record.maximumRelativeScale = definition->maximumRelativeScale;
        record.requiredParentTag    = internal::copyString(definition->requiredParentTag);
        record.requiredChildTag     = internal::copyString(definition->requiredChildTag);
        registry_.rules.push_back(std::move(record));
        registry_.indexRule(static_cast<std::uint32_t>(registry_.rules.size() - 1));
        return succeed(error);
    }

    bool registerRigImpl(const RigDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !internal::stringNonEmpty(definition->id)
            || (definition->joints == nullptr && definition->jointCount != 0))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.rigs.size() >= budgets_.maximumRigs || definition->jointCount > budgets_.maximumJointsPerRig)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.rigIndices.find(std::string_view(definition->id)) != registry_.rigIndices.end())
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        RigRecord record;
        record.id = definition->id;
        std::unordered_set<std::string> jointSemantics;
        jointSemantics.reserve(definition->jointCount);
        for (std::uint32_t jointIndex = 0; jointIndex < definition->jointCount; ++jointIndex)
        {
            const RigJointDefinition &joint = definition->joints[jointIndex];
            if (!internal::stringNonEmpty(joint.semantic) || jointSemantics.count(joint.semantic) != 0)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            // Enforce parent-before-child ordering: a fragment is only valid
            // when every parent index references an earlier joint or -1.
            if (joint.parentIndex < -1 || joint.parentIndex >= static_cast<std::int32_t>(jointIndex))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (!internal::finiteVector3(joint.translation) || !internal::finiteQuaternion(joint.rotation)
                || !internal::nonZeroQuaternion(joint.rotation) || !internal::finiteVector3(joint.scale))
            {
                return fail(error, CharacterError::InvalidArgument);
            }

            RigJointRecord jointRecord;
            jointRecord.semantic    = joint.semantic;
            jointRecord.parentIndex = joint.parentIndex;
            jointRecord.translation = internal::copyVector3(joint.translation);
            jointRecord.rotation    = internal::copyQuaternion(joint.rotation);
            jointRecord.scale       = internal::copyVector3(joint.scale);
            jointSemantics.insert(joint.semantic);
            record.joints.push_back(std::move(jointRecord));
        }

        registry_.rigIndices.emplace(record.id, registry_.rigs.size());
        registry_.rigs.push_back(std::move(record));
        return succeed(error);
    }

    bool registerPartImpl(const PartDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !internal::stringNonEmpty(definition->id))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.parts.size() >= budgets_.maximumRegisteredParts
            || definition->connectorCount > budgets_.maximumConnectorsPerPart
            || definition->parameterBindingCount > budgets_.maximumBindingsPerPart
            || definition->materialSlotCount > budgets_.maximumMaterialSlotsPerPart
            || definition->tagCount > budgets_.maximumTagsPerPart)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.partIndices.find(std::string_view(definition->id)) != registry_.partIndices.end())
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (!internal::finiteVector3(definition->boundsCenter) || !internal::finiteFloat(definition->boundsRadius)
            || definition->boundsRadius <= 0.0f)
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if ((definition->connectors == nullptr && definition->connectorCount != 0)
            || (definition->parameterBindings == nullptr && definition->parameterBindingCount != 0)
            || (definition->materialSlots == nullptr && definition->materialSlotCount != 0)
            || (definition->tags == nullptr && definition->tagCount != 0))
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        // Import-time validation: the referenced rig must be registered
        // before the part that uses it.
        PartRecord record;
        record.id          = definition->id;
        record.displayName = internal::copyString(definition->displayName);
        if (record.displayName.empty())
        {
            record.displayName = record.id;
        }
        record.geometryId    = internal::copyString(definition->geometryId);
        record.rigId         = internal::copyString(definition->rigId);
        record.boundsCenter  = internal::copyVector3(definition->boundsCenter);
        record.boundsRadius  = definition->boundsRadius;
        record.triangleCount = definition->triangleCount;
        if (!record.rigId.empty())
        {
            const auto rig = registry_.rigIndices.find(std::string_view(record.rigId));
            if (rig == registry_.rigIndices.end())
            {
                return fail(error, CharacterError::UnknownAsset);
            }
            record.rigIndex = rig->second;
        }

        std::unordered_set<std::string> connectorIds;
        connectorIds.reserve(definition->connectorCount);
        for (std::uint32_t index = 0; index < definition->connectorCount; ++index)
        {
            ConnectorRecord connector;
            if (!internal::copyConnectorDefinition(definition->connectors[index], connector, error))
            {
                return false;
            }
            if (!connectorIds.insert(connector.id).second)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            record.connectors.push_back(std::move(connector));
        }

        for (std::uint32_t index = 0; index < definition->parameterBindingCount; ++index)
        {
            const ParameterBinding &binding = definition->parameterBindings[index];
            if (!internal::stringNonEmpty(binding.parameter) || !internal::finiteFloat(binding.scale)
                || !internal::finiteVector3(binding.offset))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            BindingRecord bindingRecord;
            bindingRecord.parameter = binding.parameter;
            bindingRecord.target    = binding.target;
            bindingRecord.targetId  = internal::copyString(binding.targetId);
            bindingRecord.scale     = binding.scale;
            bindingRecord.offset    = internal::copyVector3(binding.offset);
            record.bindings.push_back(std::move(bindingRecord));
        }

        std::unordered_set<std::string> slotIds;
        for (std::uint32_t index = 0; index < definition->materialSlotCount; ++index)
        {
            const MaterialSlotDefinition &slot = definition->materialSlots[index];
            if (!internal::stringNonEmpty(slot.id) || !internal::finiteVector3(slot.baseColor)
                || !internal::finiteFloat(slot.roughness) || !internal::finiteFloat(slot.metallic))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            MaterialSlotRecord slotRecord;
            slotRecord.id        = slot.id;
            slotRecord.baseColor = internal::copyVector3(slot.baseColor);
            slotRecord.roughness = slot.roughness;
            slotRecord.metallic  = slot.metallic;
            if (!slotIds.insert(slotRecord.id).second)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            record.materialSlots.push_back(std::move(slotRecord));
        }

        for (std::uint32_t index = 0; index < definition->tagCount; ++index)
        {
            const char *tag = definition->tags[index];
            if (!internal::stringNonEmpty(tag))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            record.tags.push_back(std::string(tag));
        }

        // Resolve anchors and binding targets against the registry (joint
        // and connector indices for evaluation, existence checks for early
        // registration failures).
        if (!internal::resolvePartReferences(registry_, record, error))
        {
            return false;
        }

        registry_.partIndices.emplace(record.id, registry_.parts.size());
        registry_.parts.push_back(std::move(record));
        return succeed(error);
    }

    bool unregisterPartImpl(const char *partId)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (!internal::stringNonEmpty(partId))
        {
            return false;
        }
        const auto found = registry_.partIndices.find(std::string_view(partId));
        if (found == registry_.partIndices.end())
        {
            return false;
        }
        const std::uint32_t index = found->second;
        registry_.parts.erase(registry_.parts.begin() + index);
        registry_.partIndices.erase(found);
        for (auto &entry : registry_.partIndices)
        {
            if (entry.second > index)
            {
                --entry.second;
            }
        }
        return true;
    }

    std::uint32_t parameterCountImpl()
    {
        std::lock_guard stateLock(stateMutex_);
        return static_cast<std::uint32_t>(registry_.parameters.size());
    }

    bool parameterInfoImpl(std::uint32_t index, ParameterDefinition *outDefinition)
    {
        std::lock_guard stateLock(stateMutex_);
        if (outDefinition == nullptr || index >= registry_.parameters.size())
        {
            return false;
        }
        const ParameterRecord &record = registry_.parameters[index];
        *outDefinition                = ParameterDefinition{};
        outDefinition->id             = record.id.c_str();
        outDefinition->displayName    = record.displayName.c_str();
        outDefinition->minimum        = record.minimum;
        outDefinition->maximum        = record.maximum;
        outDefinition->defaultV       = record.defaultV;
        return true;
    }

    std::uint32_t partCountImpl()
    {
        std::lock_guard stateLock(stateMutex_);
        return static_cast<std::uint32_t>(registry_.parts.size());
    }

    bool partIdImpl(std::uint32_t index, const char **outPartId)
    {
        std::lock_guard stateLock(stateMutex_);
        if (outPartId == nullptr || index >= registry_.parts.size())
        {
            return false;
        }
        *outPartId = registry_.parts[index].id.c_str();
        return true;
    }

    bool registerProfileImpl(const AssemblyProfileDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !internal::stringNonEmpty(definition->id))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.profiles.size() >= budgets_.maximumProfiles)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.profileIndices.find(std::string_view(definition->id)) != registry_.profileIndices.end())
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        ProfileRecord record;
        record.id          = definition->id;
        record.displayName = internal::copyString(definition->displayName);
        if (record.displayName.empty())
        {
            record.displayName = record.id;
        }

        // The profile is a topology contract: parts must be registered, the
        // referenced connectors must exist, and the attachment relationships
        // must already form a single-parented acyclic tree.
        std::unordered_set<std::string>                      instanceNames;
        std::unordered_map<std::string, const PartRecord *>  profileParts;
        std::unordered_map<std::string, std::string>         parentOf;
        for (std::uint32_t index = 0; index < definition->partCount; ++index)
        {
            const ProfilePart &part = definition->parts[index];
            if (!internal::stringNonEmpty(part.name) || !internal::stringNonEmpty(part.partId))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            const PartRecord *partRecord = internal::findPartRecord(registry_, part.partId);
            if (partRecord == nullptr)
            {
                return fail(error, CharacterError::UnknownAsset);
            }
            if (!instanceNames.insert(part.name).second)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            profileParts.emplace(part.name, partRecord);
            record.parts.push_back({part.name, part.partId});
        }

        for (std::uint32_t index = 0; index < definition->connectionCount; ++index)
        {
            const ProfileConnection &connection = definition->connections[index];
            if (!internal::stringNonEmpty(connection.parentName)
                || !internal::stringNonEmpty(connection.parentConnector)
                || !internal::stringNonEmpty(connection.childName)
                || !internal::stringNonEmpty(connection.childConnector))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (instanceNames.count(connection.parentName) == 0 || instanceNames.count(connection.childName) == 0
                || connection.parentName == connection.childName)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (internal::findConnector(*profileParts[connection.parentName], connection.parentConnector) == nullptr
                || internal::findConnector(*profileParts[connection.childName], connection.childConnector) == nullptr)
            {
                return fail(error, CharacterError::UnknownConnector);
            }
            if (!parentOf.emplace(connection.childName, connection.parentName).second)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (!internal::finiteVector3(connection.offsetTranslation)
                || !internal::finiteQuaternion(connection.offsetRotation)
                || !internal::nonZeroQuaternion(connection.offsetRotation))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            ProfileRecord::Connection connectionRecord;
            connectionRecord.parentName        = connection.parentName;
            connectionRecord.parentConnector   = connection.parentConnector;
            connectionRecord.childName         = connection.childName;
            connectionRecord.childConnector    = connection.childConnector;
            connectionRecord.offsetTranslation = internal::copyVector3(connection.offsetTranslation);
            connectionRecord.offsetRotation    = internal::copyQuaternion(connection.offsetRotation);
            record.connections.push_back(std::move(connectionRecord));
        }

        // Reject cycles: every child chain must terminate at a parentless
        // instance.
        for (const auto &entry : parentOf)
        {
            std::string current = entry.first;
            for (std::size_t steps = 0; steps <= record.parts.size(); ++steps)
            {
                const auto parent = parentOf.find(current);
                if (parent == parentOf.end())
                {
                    break;
                }
                current = parent->second;
                if (current == entry.first)
                {
                    return fail(error, CharacterError::InvalidArgument);
                }
            }
        }

        registry_.profileIndices.emplace(record.id, registry_.profiles.size());
        registry_.profiles.push_back(std::move(record));
        return succeed(error);
    }

    // ---- assembly instances ------------------------------------------------

    CharacterHandle createAssemblyImpl(const char *profileId, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (characters_.size() >= budgets_.maximumCharacters || nextCharacterId_ == invalidCharacter)
        {
            fail(error, CharacterError::CapacityExceeded);
            return invalidCharacter;
        }

        auto record = std::make_unique<CharacterRecord>();
        record->id  = nextCharacterId_++;
        if (internal::stringNonEmpty(profileId))
        {
            const auto profile = registry_.profileIndices.find(std::string_view(profileId));
            if (profile == registry_.profileIndices.end())
            {
                fail(error, CharacterError::UnknownAsset);
                return invalidCharacter;
            }
            const ProfileRecord &profileRecord = registry_.profiles[profile->second];
            record->profileId                  = profileRecord.id;
            std::unordered_map<std::string, InstanceId> profileInstances;
            for (const ProfileRecord::Part &part : profileRecord.parts)
            {
                const PartRecord *partRecord = internal::findPartRecord(registry_, part.partId);
                if (partRecord == nullptr)
                {
                    fail(error, CharacterError::UnknownAsset);
                    return invalidCharacter;
                }
                const InstanceId instanceId = appendInstance(*record, *partRecord, part.name);
                if (instanceId == invalidInstance)
                {
                    fail(error, CharacterError::CapacityExceeded);
                    return invalidCharacter;
                }
                profileInstances.emplace(part.name, instanceId);
            }
            for (const ProfileRecord::Connection &connection : profileRecord.connections)
            {
                ConnectionRecord connectionRecord;
                connectionRecord.parent            = profileInstances[connection.parentName];
                connectionRecord.parentConnector   = connection.parentConnector;
                connectionRecord.child             = profileInstances[connection.childName];
                connectionRecord.childConnector    = connection.childConnector;
                connectionRecord.offsetTranslation = connection.offsetTranslation;
                connectionRecord.offsetRotation    = connection.offsetRotation;
                record->connections.push_back(std::move(connectionRecord));
            }
        }

        // Every registered parameter starts at its default value.
        record->parameterValues.reserve(registry_.parameters.size());
        for (const ParameterRecord &parameter : registry_.parameters)
        {
            record->parameterValues.push_back(parameter.defaultV);
        }

        const CharacterHandle handle = record->id;
        characters_.emplace(handle, std::move(record));
        publishChange(handle, CharacterChangeKind::StructureChanged);
        if (error != nullptr)
        {
            *error = CharacterError::None;
        }
        return handle;
    }

    void destroyAssemblyImpl(CharacterHandle character)
    {
        std::lock_guard stateLock(stateMutex_);
        if (characters_.erase(character) != 0)
        {
            publishChange(character, CharacterChangeKind::AssemblyDestroyed);
        }
    }

    PartInstance *instanceFor(CharacterRecord &record, InstanceId instance) const
    {
        const auto found = record.instanceIndicesById.find(instance);
        return found != record.instanceIndicesById.end() ? &record.parts[found->second] : nullptr;
    }

    const PartInstance *instanceFor(const CharacterRecord &record, InstanceId instance) const
    {
        const auto found = record.instanceIndicesById.find(instance);
        return found != record.instanceIndicesById.end() ? &record.parts[found->second] : nullptr;
    }

    // Appends a part snapshot as a new instance and maintains the name and
    // id index maps. Returns invalidInstance when the per-assembly part
    // budget is exhausted.
    InstanceId appendInstance(CharacterRecord &record, const PartRecord &part, const std::string &instanceName)
    {
        if (record.parts.size() >= budgets_.maximumPartsPerCharacter)
        {
            return invalidInstance;
        }
        PartInstance instance;
        instance.id     = record.nextInstanceId++;
        instance.name   = instanceName;
        instance.partId = part.id;
        instance.part   = part;
        record.instancesByName.emplace(instance.name, instance.id);
        record.instanceIndicesById.emplace(instance.id, static_cast<std::uint32_t>(record.parts.size()));
        record.parts.push_back(std::move(instance));
        markStructureDirty(record);
        return record.parts.back().id;
    }

    InstanceId addPartImpl(
        CharacterHandle character, const char *partId, const char *instanceName, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            fail(error, CharacterError::InvalidHandle);
            return invalidInstance;
        }
        if (!internal::stringNonEmpty(partId) || !internal::stringNonEmpty(instanceName))
        {
            fail(error, CharacterError::InvalidArgument);
            return invalidInstance;
        }
        if (record->instancesByName.find(std::string_view(instanceName)) != record->instancesByName.end())
        {
            fail(error, CharacterError::ValidationFailed);
            return invalidInstance;
        }
        const PartRecord *part = internal::findPartRecord(registry_, partId);
        if (part == nullptr)
        {
            fail(error, CharacterError::UnknownAsset);
            return invalidInstance;
        }
        const InstanceId instance = appendInstance(*record, *part, instanceName);
        if (instance == invalidInstance)
        {
            fail(error, CharacterError::CapacityExceeded);
            return invalidInstance;
        }
        publishChange(character, CharacterChangeKind::StructureChanged);
        if (error != nullptr)
        {
            *error = CharacterError::None;
        }
        return instance;
    }

    bool removePartImpl(CharacterHandle character, InstanceId instance)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || instanceFor(*record, instance) == nullptr)
        {
            return false;
        }
        record->connections.erase(std::remove_if(record->connections.begin(),
                                      record->connections.end(),
                                      [instance](const ConnectionRecord &connection)
                                      { return connection.parent == instance || connection.child == instance; }),
            record->connections.end());
        for (std::size_t index = 0; index < record->parts.size();)
        {
            if (record->parts[index].id == instance)
            {
                record->instancesByName.erase(record->parts[index].name);
                record->parts.erase(record->parts.begin() + static_cast<std::ptrdiff_t>(index));
            }
            else
            {
                ++index;
            }
        }
        rebuildInstanceIndices(*record);
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return true;
    }

    bool replacePartImpl(CharacterHandle character, InstanceId instance, const char *newPartId, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || newPartId == nullptr)
        {
            return fail(error, record == nullptr ? CharacterError::InvalidHandle : CharacterError::InvalidArgument);
        }
        PartInstance *part = instanceFor(*record, instance);
        if (part == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }
        const PartRecord *replacement = internal::findPartRecord(registry_, newPartId);
        if (replacement == nullptr)
        {
            return fail(error, CharacterError::UnknownAsset);
        }
        part->part   = *replacement;
        part->partId = replacement->id;
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return succeed(error);
    }

    ConnectionRecord *connectionForChild(CharacterRecord &record, InstanceId child)
    {
        for (ConnectionRecord &connection : record.connections)
        {
            if (connection.child == child)
            {
                return &connection;
            }
        }
        return nullptr;
    }

    bool connectImpl(CharacterHandle character, InstanceId parent, const char *parentConnector, InstanceId child,
        const char *childConnector, const float *offsetTranslation, const float *offsetRotation, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }
        if (parentConnector == nullptr || childConnector == nullptr || parent == invalidInstance
            || child == invalidInstance)
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        PartInstance *parentPart = instanceFor(*record, parent);
        PartInstance *childPart  = instanceFor(*record, child);
        if (parentPart == nullptr || childPart == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }
        if (parent == child)
        {
            return fail(error, CharacterError::ValidationFailed);
        }
        const ConnectorRecord *parentConnectorRecord = internal::findConnector(parentPart->part, parentConnector);
        const ConnectorRecord *childConnectorRecord  = internal::findConnector(childPart->part, childConnector);
        if (parentConnectorRecord == nullptr || childConnectorRecord == nullptr)
        {
            return fail(error, CharacterError::UnknownConnector);
        }
        if (record->connections.size() >= budgets_.maximumConnectionsPerCharacter)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        // Structural attachment relationships must stay acyclic: reject a
        // connection that would make the child its own ancestor.
        if (wouldCreateCycle(*record, parent, child))
        {
            return fail(error, CharacterError::ValidationFailed);
        }
        if (offsetTranslation != nullptr && !internal::finiteVector3(offsetTranslation))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (offsetRotation != nullptr
            && (!internal::finiteQuaternion(offsetRotation) || !internal::nonZeroQuaternion(offsetRotation)))
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        // Eager data-defined compatibility: the role pair must be allowed by
        // an explicit or (parentRole, childRole) rule, and connector capacity
        // must hold. Scale-range and tag conditions are checked by
        // validation, which also covers later part replacements.
        const CompatibilityRuleRecord *rule
            = internal::resolveCompatibilityRule(registry_, *parentConnectorRecord, *childConnectorRecord);
        if (rule == nullptr || !rule->allowed)
        {
            lastErrorMessage_ = "no compatibility rule allows role pair ('" + parentConnectorRecord->role + "', '"
                              + childConnectorRecord->role + "')";
            return fail(error, CharacterError::ValidationFailed);
        }
        const glm::vec3 newOffsetTranslation
            = offsetTranslation != nullptr ? internal::copyVector3(offsetTranslation) : glm::vec3(0.0f);
        const glm::quat newOffsetRotation = offsetRotation != nullptr
            ? internal::copyQuaternion(offsetRotation)
            : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

        // Re-parenting: a child has exactly one structural parent, so an
        // existing incoming connection is replaced. An identical connection
        // (same sockets and offsets) is a no-op so repeated calls from
        // careless callers do not invalidate evaluation caches.
        const auto capacityReached = [&](bool excludeChild) -> bool
        {
            std::uint32_t used = 0;
            for (const ConnectionRecord &connection : record->connections)
            {
                if (connection.parent == parent && connection.parentConnector == parentConnector
                    && (!excludeChild || connection.child != child))
                {
                    ++used;
                }
            }
            if (used >= parentConnectorRecord->maxConnections)
            {
                lastErrorMessage_ = "connector '" + std::string(parentConnector) + "' capacity "
                                  + std::to_string(parentConnectorRecord->maxConnections) + " exceeded";
                return true;
            }
            return false;
        };
        if (ConnectionRecord *existing = connectionForChild(*record, child); existing != nullptr)
        {
            const bool sameTarget = existing->parent == parent && existing->parentConnector == parentConnector
                && existing->childConnector == childConnector && existing->offsetTranslation == newOffsetTranslation
                && existing->offsetRotation == newOffsetRotation;
            if (sameTarget)
            {
                return succeed(error);
            }
            if ((existing->parent != parent || existing->parentConnector != parentConnector)
                && capacityReached(true))
            {
                return fail(error, CharacterError::ValidationFailed);
            }
            existing->parent            = parent;
            existing->parentConnector   = parentConnector;
            existing->childConnector    = childConnector;
            existing->offsetTranslation = newOffsetTranslation;
            existing->offsetRotation    = newOffsetRotation;
        }
        else
        {
            if (capacityReached(false))
            {
                return fail(error, CharacterError::ValidationFailed);
            }
            ConnectionRecord connectionRecord;
            connectionRecord.parent            = parent;
            connectionRecord.parentConnector   = parentConnector;
            connectionRecord.child             = child;
            connectionRecord.childConnector    = childConnector;
            connectionRecord.offsetTranslation = newOffsetTranslation;
            connectionRecord.offsetRotation    = newOffsetRotation;
            record->connections.push_back(std::move(connectionRecord));
        }
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return succeed(error);
    }

    static bool wouldCreateCycle(const CharacterRecord &record, InstanceId parent, InstanceId child)
    {
        // Walk the cached structural parents; when the topology is stale the
        // connection list itself is scanned as a fallback so a cycle can
        // never slip through a mutation.
        InstanceId current = parent;
        while (current != invalidInstance)
        {
            if (current == child)
            {
                return true;
            }
            const auto found = record.instanceIndicesById.find(current);
            if (found != record.instanceIndicesById.end() && record.topologyValid)
            {
                current = record.parts[found->second].parent;
                continue;
            }
            const ConnectionRecord *incoming = nullptr;
            for (const ConnectionRecord &connection : record.connections)
            {
                if (connection.child == current)
                {
                    incoming = &connection;
                    break;
                }
            }
            current = incoming != nullptr ? incoming->parent : invalidInstance;
        }
        return false;
    }

    bool disconnectImpl(CharacterHandle character, InstanceId child)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return false;
        }
        const std::size_t before = record->connections.size();
        record->connections.erase(
            std::remove_if(record->connections.begin(),
                record->connections.end(),
                [child](const ConnectionRecord &connection) { return connection.child == child; }),
            record->connections.end());
        if (record->connections.size() == before)
        {
            return false;
        }
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return true;
    }

    bool setAssemblyTransformImpl(CharacterHandle character, const float *translation, const float *rotation)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return false;
        }
        if ((translation != nullptr && !internal::finiteVector3(translation))
            || (rotation != nullptr
                && (!internal::finiteQuaternion(rotation) || !internal::nonZeroQuaternion(rotation))))
        {
            return false;
        }
        // Identical values are a no-op: per-frame callers that re-send the
        // current placement neither invalidate the caches nor republish the
        // change event.
        const bool translationChanged = translation != nullptr
            && (record->rootTranslation.x != translation[0] || record->rootTranslation.y != translation[1]
                || record->rootTranslation.z != translation[2]);
        const bool rotationChanged = rotation != nullptr && record->rootRotation != internal::copyQuaternion(rotation);
        if (!translationChanged && !rotationChanged)
        {
            return true;
        }
        if (translationChanged)
        {
            record->rootTranslation = internal::copyVector3(translation);
        }
        if (rotationChanged)
        {
            record->rootRotation = internal::copyQuaternion(rotation);
        }
        markTransformsDirty(*record);
        publishChange(character, CharacterChangeKind::OffsetsChanged);
        return true;
    }

    bool setConnectionOffsetImpl(
        CharacterHandle character, InstanceId child, const float *offsetTranslation, const float *offsetRotation)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return false;
        }
        ConnectionRecord *connection = connectionForChild(*record, child);
        if (connection == nullptr)
        {
            return false;
        }
        if ((offsetTranslation != nullptr && !internal::finiteVector3(offsetTranslation))
            || (offsetRotation != nullptr
                && (!internal::finiteQuaternion(offsetRotation) || !internal::nonZeroQuaternion(offsetRotation))))
        {
            return false;
        }
        const bool translationChanged = offsetTranslation != nullptr
            && (connection->offsetTranslation.x != offsetTranslation[0]
                || connection->offsetTranslation.y != offsetTranslation[1]
                || connection->offsetTranslation.z != offsetTranslation[2]);
        const bool rotationChanged
            = offsetRotation != nullptr && connection->offsetRotation != internal::copyQuaternion(offsetRotation);
        if (!translationChanged && !rotationChanged)
        {
            return true;
        }
        if (translationChanged)
        {
            connection->offsetTranslation = internal::copyVector3(offsetTranslation);
        }
        if (rotationChanged)
        {
            connection->offsetRotation = internal::copyQuaternion(offsetRotation);
        }
        markTransformsDirty(*record);
        publishChange(character, CharacterChangeKind::OffsetsChanged);
        return true;
    }

    // ---- parameters ---------------------------------------------------------

    bool setParameterImpl(CharacterHandle character, const char *parameter, float value, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || !internal::stringNonEmpty(parameter))
        {
            return fail(error, record == nullptr ? CharacterError::InvalidHandle : CharacterError::InvalidArgument);
        }
        if (!internal::finiteFloat(value))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        const auto found = registry_.parameterIndices.find(std::string_view(parameter));
        if (found == registry_.parameterIndices.end())
        {
            return fail(error, CharacterError::UnknownAsset);
        }
        const ParameterRecord &definition = registry_.parameters[found->second];
        if (found->second >= record->parameterValues.size())
        {
            return fail(error, CharacterError::UnknownAsset);
        }
        const float clamped = std::clamp(value, definition.minimum, definition.maximum);
        if (record->parameterValues[found->second] == clamped)
        {
            // Same value: keep evaluation caches and the event bus calm.
            return succeed(error);
        }
        record->parameterValues[found->second] = clamped;
        markParametersDirty(*record);
        publishChange(character, CharacterChangeKind::ParametersChanged);
        return succeed(error);
    }

    bool getParameterImpl(CharacterHandle character, const char *parameter, float *outValue)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || !internal::stringNonEmpty(parameter) || outValue == nullptr)
        {
            return false;
        }
        const auto found = registry_.parameterIndices.find(std::string_view(parameter));
        if (found == registry_.parameterIndices.end() || found->second >= record->parameterValues.size())
        {
            return false;
        }
        *outValue = record->parameterValues[found->second];
        return true;
    }

    // ---- validation and evaluation -----------------------------------------

    void collectIssues(const CharacterRecord &record, std::vector<ValidationIssue> &issues) const
    {
        internal::collectIssues(registry_, record, issues);
    }

    std::uint32_t validateImpl(CharacterHandle character, ValidationIssue *outIssues, std::uint32_t capacity)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return 0;
        }
        collectIssues(*record, record->lastIssues);
        if (outIssues != nullptr)
        {
            const std::uint32_t count
                = std::min<std::uint32_t>(capacity, static_cast<std::uint32_t>(record->lastIssues.size()));
            std::copy_n(record->lastIssues.begin(), count, outIssues);
        }
        return static_cast<std::uint32_t>(record->lastIssues.size());
    }

    bool evaluateImpl(CharacterHandle character, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }

        const bool structureChanged  = record->structureDirty;
        const bool parametersChanged = record->parametersDirty;

        // Validation re-runs only when the validated surface changed
        // (structure or parameters). While issues remain, evaluation fails
        // fast regardless of further transform updates, so an invalid
        // assembly never becomes active; fixing the cause revalidates on the
        // next call.
        if (record->validationPending)
        {
            record->validationPending = false;
            if (budgets_.validateOnEvaluate)
            {
                collectIssues(*record, record->lastIssues);
            }
            else
            {
                record->lastIssues.clear();
            }
        }
        if (budgets_.validateOnEvaluate && !record->lastIssues.empty())
        {
            record->statistics.issueCount = static_cast<std::uint32_t>(record->lastIssues.size());
            return fail(error, CharacterError::ValidationFailed);
        }
        if (!structureChanged && !parametersChanged && !record->transformsDirty && record->boundsValid
            && record->topologyValid)
        {
            return succeed(error);
        }

        if (structureChanged || !record->topologyValid)
        {
            internal::prepareTopology(registry_, *record);
        }
        if (structureChanged || parametersChanged)
        {
            internal::applyParameters(registry_, *record);
        }
        if (!internal::evaluateTransforms(registry_, *record))
        {
            return fail(error, CharacterError::ValidationFailed);
        }
        internal::updateBoundsAndStatistics(registry_, *record);
        record->statistics.issueCount = 0;

        record->structureDirty  = false;
        record->parametersDirty = false;
        record->transformsDirty = false;
        // Publish only real changes: the per-frame animation path already
        // published OffsetsChanged from setConnectionOffset, and an
        // Evaluated event on every transform refresh would spam the bus.
        if (structureChanged || parametersChanged)
        {
            publishChange(character, CharacterChangeKind::Evaluated);
        }
        return succeed(error);
    }

    // ---- queries -------------------------------------------------------------

    std::uint32_t instanceCountImpl(CharacterHandle character)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        return record != nullptr ? static_cast<std::uint32_t>(record->parts.size()) : 0;
    }

    InstanceId instanceIdImpl(CharacterHandle character, std::uint32_t index)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || index >= record->parts.size())
        {
            return invalidInstance;
        }
        return record->parts[index].id;
    }

    bool instanceInfoImpl(CharacterHandle character, InstanceId instance, const char **outName, const char **outPartId,
        InstanceId *outParent)
    {
        std::lock_guard     stateLock(stateMutex_);
        CharacterRecord    *record = recordFor(character);
        const PartInstance *part   = record != nullptr ? instanceFor(*record, instance) : nullptr;
        if (part == nullptr)
        {
            return false;
        }
        if (outName != nullptr)
        {
            *outName = part->name.c_str();
        }
        if (outPartId != nullptr)
        {
            *outPartId = part->partId.c_str();
        }
        if (outParent != nullptr)
        {
            if (record->topologyValid)
            {
                // The cached parent matches the connection list whenever the
                // topology is current (mutations invalidate it).
                *outParent = part->parent;
            }
            else
            {
                *outParent = invalidInstance;
                for (const ConnectionRecord &connection : record->connections)
                {
                    if (connection.child == instance)
                    {
                        *outParent = connection.parent;
                        break;
                    }
                }
            }
        }
        return true;
    }

    bool instanceTransformImpl(CharacterHandle character, InstanceId instance, float *outMatrix16)
    {
        std::lock_guard     stateLock(stateMutex_);
        CharacterRecord    *record = recordFor(character);
        const PartInstance *part   = record != nullptr ? instanceFor(*record, instance) : nullptr;
        if (part == nullptr || outMatrix16 == nullptr || !part->transformValid)
        {
            return false;
        }
        std::memcpy(outMatrix16, glm::value_ptr(part->world), sizeof(float) * 16);
        return true;
    }

    bool boundsImpl(CharacterHandle character, float *outCenter3, float *outRadius)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || !record->boundsValid)
        {
            return false;
        }
        if (outCenter3 != nullptr)
        {
            outCenter3[0] = record->boundsCenter.x;
            outCenter3[1] = record->boundsCenter.y;
            outCenter3[2] = record->boundsCenter.z;
        }
        if (outRadius != nullptr)
        {
            *outRadius = record->boundsRadius;
        }
        return true;
    }

    bool statisticsImpl(CharacterHandle character, CharacterStatistics *outStatistics)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        *outStatistics            = record->statistics;
        outStatistics->issueCount = static_cast<std::uint32_t>(record->lastIssues.size());
        return true;
    }

    // ---- serialization --------------------------------------------------------

    bool saveRecipeImpl(CharacterHandle character, const char *filePath, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }
        if (filePath == nullptr || filePath[0] == '\0')
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        std::string errorMessage;
        if (!internal::saveRecipeToFile(registry_, *record, filePath, errorMessage))
        {
            lastErrorMessage_ = errorMessage;
            return fail(error, CharacterError::IoFailed);
        }
        return succeed(error);
    }

    bool loadRecipeImpl(CharacterHandle character, const char *filePath, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr)
        {
            return fail(error, CharacterError::InvalidHandle);
        }
        if (filePath == nullptr || filePath[0] == '\0')
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        std::string errorMessage;
        if (!internal::loadRecipeFromFile(registry_, budgets_, *record, filePath, errorMessage))
        {
            lastErrorMessage_ = errorMessage;
            markStructureDirty(*record);
            return fail(error, CharacterError::IoFailed);
        }
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return succeed(error);
    }

    const char *lastErrorMessageImpl()
    {
        std::lock_guard stateLock(stateMutex_);
        return lastErrorMessage_.c_str();
    }

    // ---- helpers ------------------------------------------------------------

    CharacterRecord *recordFor(CharacterHandle character)
    {
        const auto found = characters_.find(character);
        return found != characters_.end() ? found->second.get() : nullptr;
    }

    // Reindexes parts after a removal shifted their positions.
    static void rebuildInstanceIndices(CharacterRecord &record)
    {
        record.instanceIndicesById.clear();
        for (std::size_t index = 0; index < record.parts.size(); ++index)
        {
            record.instanceIndicesById.emplace(record.parts[index].id, static_cast<std::uint32_t>(index));
        }
    }

    static void markStructureDirty(CharacterRecord &record)
    {
        record.structureDirty    = true;
        record.parametersDirty   = true;
        record.transformsDirty   = true;
        record.validationPending = true;
        record.topologyValid     = false;
        for (PartInstance &part : record.parts)
        {
            part.transformValid = false;
        }
    }

    static void markParametersDirty(CharacterRecord &record)
    {
        record.parametersDirty   = true;
        record.transformsDirty   = true;
        record.validationPending = true;
    }

    static void markTransformsDirty(CharacterRecord &record)
    {
        record.transformsDirty = true;
    }

    static bool fail(CharacterError *error, CharacterError code)
    {
        if (error != nullptr)
        {
            *error = code;
        }
        return false;
    }

    static bool succeed(CharacterError *error)
    {
        if (error != nullptr)
        {
            *error = CharacterError::None;
        }
        return true;
    }

    void publishChange(CharacterHandle character, CharacterChangeKind kind)
    {
        if (context_ != nullptr)
        {
            // Deferred delivery: character operations are reachable from
            // script callbacks that themselves run inside event dispatch
            // (on_start/on_frame); a synchronous publish would re-enter the
            // bus and deadlock. Deferred events are flushed by the app every
            // frame, preserving the invalidation semantics.
            context_->eventBus().publishDeferred(CharacterAssemblyChangedEvent{.character = character, .kind = kind});
        }
    }

    void assignCapabilityPointers();

    ParticipantContext                                                   *context_ = nullptr;
    Registry                                                              registry_;
    Budgets                                                               budgets_;
    std::unordered_map<CharacterHandle, std::unique_ptr<CharacterRecord>> characters_;
    CharacterHandle                                                       nextCharacterId_ = 1;
    std::string                                                           lastErrorMessage_;
    mutable std::recursive_mutex                                          stateMutex_;
    SubscriptionToken                                                     reloadSubscription_ = 0;

    script_runtime::ScriptRuntimeCapability       *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_
        = script_runtime::invalidScriptRuntimeInitializer;
    CharacterLuaBridgeContext luaBridgeContext_{};

    ParticipantManifest manifest_;
    CharacterCapability capability_{};
};

// Capability bindings: every entry forwards to the matching member above.
void CharacterModule::assignCapabilityPointers()
{
    capability_.abi  = characterCapabilityAbi;
    capability_.self = this;
    capability_.registerParameter
        = [](void *self, const ParameterDefinition *definition, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->registerParameterImpl(definition, error); };
    capability_.registerCompatibilityRule
        = [](void *self, const CompatibilityRuleDefinition *definition, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->registerCompatibilityRuleImpl(definition, error); };
    capability_.registerRig = [](void *self, const RigDefinition *definition, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->registerRigImpl(definition, error); };
    capability_.registerPart = [](void *self, const PartDefinition *definition, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->registerPartImpl(definition, error); };
    capability_.registerProfile = [](void *self, const AssemblyProfileDefinition *definition, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->registerProfileImpl(definition, error); };
    capability_.unregisterPart = [](void *self, const char *partId)
    { return static_cast<CharacterModule *>(self)->unregisterPartImpl(partId); };
    capability_.parameterCount
        = [](void *self) { return static_cast<CharacterModule *>(self)->parameterCountImpl(); };
    capability_.parameterInfo = [](void *self, std::uint32_t index, ParameterDefinition *outDefinition)
    { return static_cast<CharacterModule *>(self)->parameterInfoImpl(index, outDefinition); };
    capability_.partCount = [](void *self) { return static_cast<CharacterModule *>(self)->partCountImpl(); };
    capability_.partId    = [](void *self, std::uint32_t index, const char **outPartId)
    { return static_cast<CharacterModule *>(self)->partIdImpl(index, outPartId); };

    capability_.createAssembly = [](void *self, const char *profileId, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->createAssemblyImpl(profileId, error); };
    capability_.destroyAssembly = [](void *self, CharacterHandle character)
    { static_cast<CharacterModule *>(self)->destroyAssemblyImpl(character); };
    capability_.addPart = [](void            *self,
                             CharacterHandle character,
                             const char     *partId,
                             const char     *instanceName,
                             CharacterError *error)
    { return static_cast<CharacterModule *>(self)->addPartImpl(character, partId, instanceName, error); };
    capability_.removePart = [](void *self, CharacterHandle character, InstanceId instance)
    { return static_cast<CharacterModule *>(self)->removePartImpl(character, instance); };
    capability_.replacePart =
        [](void *self, CharacterHandle character, InstanceId instance, const char *newPartId, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->replacePartImpl(character, instance, newPartId, error); };
    capability_.connect = [](void            *self,
                             CharacterHandle character,
                             InstanceId      parent,
                             const char     *parentConnector,
                             InstanceId      child,
                             const char     *childConnector,
                             const float    *offsetTranslation,
                             const float    *offsetRotation,
                             CharacterError *error)
    {
        return static_cast<CharacterModule *>(self)->connectImpl(
            character, parent, parentConnector, child, childConnector, offsetTranslation, offsetRotation, error);
    };
    capability_.disconnect = [](void *self, CharacterHandle character, InstanceId child)
    { return static_cast<CharacterModule *>(self)->disconnectImpl(character, child); };
    capability_.setAssemblyTransform
        = [](void *self, CharacterHandle character, const float *translation, const float *rotation)
    { return static_cast<CharacterModule *>(self)->setAssemblyTransformImpl(character, translation, rotation); };
    capability_.setConnectionOffset = [](void            *self,
                                         CharacterHandle character,
                                         InstanceId      child,
                                         const float    *offsetTranslation,
                                         const float    *offsetRotation)
    {
        return static_cast<CharacterModule *>(self)->setConnectionOffsetImpl(
            character, child, offsetTranslation, offsetRotation);
    };

    capability_.setParameter
        = [](void *self, CharacterHandle character, const char *parameter, float value, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->setParameterImpl(character, parameter, value, error); };
    capability_.getParameter = [](void *self, CharacterHandle character, const char *parameter, float *outValue)
    { return static_cast<CharacterModule *>(self)->getParameterImpl(character, parameter, outValue); };

    capability_.evaluate = [](void *self, CharacterHandle character, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->evaluateImpl(character, error); };
    capability_.validate
        = [](void *self, CharacterHandle character, ValidationIssue *outIssues, std::uint32_t capacity)
    { return static_cast<CharacterModule *>(self)->validateImpl(character, outIssues, capacity); };
    capability_.instanceCount = [](void *self, CharacterHandle character)
    { return static_cast<CharacterModule *>(self)->instanceCountImpl(character); };
    capability_.instanceId = [](void *self, CharacterHandle character, std::uint32_t index)
    { return static_cast<CharacterModule *>(self)->instanceIdImpl(character, index); };
    capability_.instanceInfo = [](void            *self,
                                  CharacterHandle character,
                                  InstanceId      instance,
                                  const char    **outName,
                                  const char    **outPartId,
                                  InstanceId     *outParent)
    {
        return static_cast<CharacterModule *>(self)->instanceInfoImpl(
            character, instance, outName, outPartId, outParent);
    };
    capability_.instanceTransform
        = [](void *self, CharacterHandle character, InstanceId instance, float *outMatrix16)
    { return static_cast<CharacterModule *>(self)->instanceTransformImpl(character, instance, outMatrix16); };
    capability_.bounds = [](void *self, CharacterHandle character, float *outCenter3, float *outRadius)
    { return static_cast<CharacterModule *>(self)->boundsImpl(character, outCenter3, outRadius); };
    capability_.statistics = [](void *self, CharacterHandle character, CharacterStatistics *outStatistics)
    { return static_cast<CharacterModule *>(self)->statisticsImpl(character, outStatistics); };

    capability_.saveRecipe = [](void *self, CharacterHandle character, const char *filePath, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->saveRecipeImpl(character, filePath, error); };
    capability_.loadRecipe = [](void *self, CharacterHandle character, const char *filePath, CharacterError *error)
    { return static_cast<CharacterModule *>(self)->loadRecipeImpl(character, filePath, error); };
    capability_.lastErrorMessage
        = [](void *self) { return static_cast<CharacterModule *>(self)->lastErrorMessageImpl(); };
}

} // namespace engine::character

// Module entry point queried by the kernel after dlopen.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"character"};
    static const char *const optionalCapabilities[] = {"script_runtime"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "character",
        .moduleVersion           = "0.1.0",
        .createModule = []() -> engine::ModuleInterface * { return new engine::character::CharacterModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = optionalCapabilities,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
