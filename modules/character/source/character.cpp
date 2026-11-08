// character module implementation.
//
// Implements the "character" capability behind the module contract: a content
// registry (parameters, compatibility rules, rig fragments, body parts,
// assembly profiles), live character assemblies with a validated attachment
// tree, connector-aligned transform evaluation with per-part parameter
// bindings, and versioned TOML recipes (serialization.cpp). An optional Lua
// bridge (lua_bridge.cpp) exposes the assembly operations to script runtimes.
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

#include "character_module.hpp"
#include "lua_bridge.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <new>
#include <optional>
#include <string>
#include <unordered_map>
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

constexpr std::size_t kMaximumIssues = 256;

bool finiteFloat(float value)
{
    return std::isfinite(value);
}

bool finiteVector3(const float *values)
{
    return values != nullptr && finiteFloat(values[0]) && finiteFloat(values[1]) && finiteFloat(values[2]);
}

bool finiteQuaternion(const float *values)
{
    return values != nullptr && finiteFloat(values[0]) && finiteFloat(values[1]) && finiteFloat(values[2])
        && finiteFloat(values[3]);
}

bool nonZeroQuaternion(const float *values)
{
    const float squaredLength
        = values[0] * values[0] + values[1] * values[1] + values[2] * values[2] + values[3] * values[3];
    return squaredLength > 1.0e-8f;
}

glm::vec3 copyVector3(const float *values)
{
    return glm::vec3(values[0], values[1], values[2]);
}

glm::quat copyQuaternion(const float *values)
{
    glm::quat   rotation(values[3], values[0], values[1], values[2]);
    const float length = glm::length(rotation);
    if (length > 1.0e-8f)
    {
        rotation = rotation / length;
    }
    else
    {
        rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    return rotation;
}

std::string copyString(const char *text)
{
    return text != nullptr ? std::string(text) : std::string();
}

bool stringNonEmpty(const char *text)
{
    return text != nullptr && text[0] != '\0';
}

void appendIssue(std::vector<ValidationIssue> &issues, ValidationCode code, InstanceId instance,
    const std::string &subject, const std::string &message)
{
    if (issues.size() >= kMaximumIssues)
    {
        return;
    }
    ValidationIssue issue{};
    issue.code     = code;
    issue.instance = instance;
    std::snprintf(issue.subject, sizeof(issue.subject), "%s", subject.c_str());
    std::snprintf(issue.message, sizeof(issue.message), "%s", message.c_str());
    issues.push_back(issue);
}

// Lookups -------------------------------------------------------------

const RigRecord *findRig(const Registry &registry, const std::string &id)
{
    const auto found = registry.rigIndices.find(id);
    return found != registry.rigIndices.end() ? &registry.rigs[found->second] : nullptr;
}

const ParameterRecord *findParameter(const Registry &registry, const std::string &id)
{
    const auto found = registry.parameterIndices.find(id);
    return found != registry.parameterIndices.end() ? &registry.parameters[found->second] : nullptr;
}

const PartRecord *findPart(const Registry &registry, const std::string &id)
{
    const auto found = registry.partIndices.find(id);
    return found != registry.partIndices.end() ? &registry.parts[found->second] : nullptr;
}

const ProfileRecord *findProfile(const Registry &registry, const std::string &id)
{
    const auto found = registry.profileIndices.find(id);
    return found != registry.profileIndices.end() ? &registry.profiles[found->second] : nullptr;
}

const CompatibilityRuleRecord *findCompatibilityRuleForRoles(
    const std::vector<CompatibilityRuleRecord> &rules, const std::string &parentRole, const std::string &childRole)
{
    const CompatibilityRuleRecord *wildcard = nullptr;
    for (const CompatibilityRuleRecord &rule : rules)
    {
        const bool parentMatches = rule.parentRole == parentRole || rule.parentRole == "*";
        const bool childMatches  = rule.childRole == childRole || rule.childRole == "*";
        if (parentMatches && childMatches)
        {
            if (rule.parentRole == parentRole && rule.childRole == childRole)
            {
                return &rule; // exact pair wins
            }
            if (wildcard == nullptr)
            {
                wildcard = &rule;
            }
        }
    }
    return wildcard;
}

// Copies one connector definition into a stored record. Returns false with
// the error out-parameter set when the definition is malformed.
bool copyConnectorDefinition(const ConnectorDefinition &definition, ConnectorRecord &record, CharacterError *error)
{
    const bool malformed
        = !stringNonEmpty(definition.id) || !stringNonEmpty(definition.role)
       || (definition.anchor.space == AnchorSpace::Joint && !stringNonEmpty(definition.anchor.jointSemantic))
       || !finiteVector3(definition.anchor.translation) || !finiteQuaternion(definition.anchor.rotation)
       || !nonZeroQuaternion(definition.anchor.rotation) || definition.maxConnections == 0
       || definition.maxConnections > 64;
    if (malformed)
    {
        if (error != nullptr)
        {
            *error = CharacterError::InvalidArgument;
        }
        return false;
    }

    record.id                  = definition.id;
    record.role                = definition.role;
    record.anchorSpace         = definition.anchor.space;
    record.anchorJointSemantic = copyString(definition.anchor.jointSemantic);
    record.anchorTranslation   = copyVector3(definition.anchor.translation);
    record.anchorRotation      = copyQuaternion(definition.anchor.rotation);
    record.compatibilityRule   = copyString(definition.compatibilityRule);
    record.maxConnections      = definition.maxConnections;
    return true;
}

} // namespace

namespace internal
{

glm::mat4 makeTrsMatrix(const glm::vec3 &translation, const glm::quat &rotation)
{
    return glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);
}

const ConnectorRecord *findConnector(const PartRecord &part, const std::string &id)
{
    for (const ConnectorRecord &connector : part.connectors)
    {
        if (connector.id == id)
        {
            return &connector;
        }
    }
    return nullptr;
}

} // namespace internal

using internal::findConnector;
using internal::makeTrsMatrix;

// The module: content registry + live assemblies behind the capability.
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
        context_ = &context;
        readConfiguration();

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (scriptRuntime_ != nullptr && scriptRuntime_->abi == script_runtime::scriptRuntimeCapabilityAbi
            && scriptRuntime_->registerRuntimeInitializer != nullptr)
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.character     = &capability_;
            luaBridgeContext_.mutex         = &stateMutex_;
            runtimeInitializer_             = scriptRuntime_->registerRuntimeInitializer(
                scriptRuntime_->self, initializeCharacterLuaRuntime, &luaBridgeContext_);
        }
    }

    void start() override
    {}

    void stop() override
    {
        if (scriptRuntime_ != nullptr && runtimeInitializer_ != script_runtime::invalidScriptRuntimeInitializer
            && scriptRuntime_->unregisterRuntimeInitializer != nullptr)
        {
            scriptRuntime_->unregisterRuntimeInitializer(scriptRuntime_->self, runtimeInitializer_);
            runtimeInitializer_ = script_runtime::invalidScriptRuntimeInitializer;
        }
        {
            std::lock_guard stateLock(stateMutex_);
            characters_.clear();
            registry_ = Registry{};
        }
        luaBridgeContext_.character = nullptr;
        context_                    = nullptr;
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

    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;
        Budgets            &budgets = budgets_;

        budgets              = Budgets{};
        const auto readCount = [&](const char *key, std::size_t &target, std::size_t maximum)
        {
            if (const auto value = context.configurationIntegerInSection("character", key);
                value && *value > 0 && static_cast<std::uint64_t>(*value) <= maximum)
            {
                target = static_cast<std::size_t>(*value);
            }
        };
        readCount("maximum_characters", budgets.maximumCharacters, 65536);
        readCount("maximum_parts", budgets.maximumRegisteredParts, 65536);
        readCount("maximum_parts_per_character", budgets.maximumPartsPerCharacter, 65536);
        readCount("maximum_connections_per_character", budgets.maximumConnectionsPerCharacter, 262144);
        readCount("maximum_connectors_per_part", budgets.maximumConnectorsPerPart, 1024);
        readCount("maximum_bindings_per_part", budgets.maximumBindingsPerPart, 1024);
        readCount("maximum_material_slots_per_part", budgets.maximumMaterialSlotsPerPart, 256);
        readCount("maximum_tags_per_part", budgets.maximumTagsPerPart, 256);
        readCount("maximum_parameters", budgets.maximumParameters, 65536);
        readCount("maximum_rules", budgets.maximumRules, 65536);
        readCount("maximum_rigs", budgets.maximumRigs, 65536);
        readCount("maximum_joints_per_rig", budgets.maximumJointsPerRig, 65536);
        readCount("maximum_profiles", budgets.maximumProfiles, 65536);
        if (const auto validate = context.configurationBooleanInSection("character", "validate_on_evaluate"))
        {
            budgets.validateOnEvaluate = *validate;
        }
    }

    // ---- capability implementation ---------------------------------------

    bool registerParameterImpl(const ParameterDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !stringNonEmpty(definition->id))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.parameters.size() >= budgets_.maximumParameters)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.parameterIndices.count(definition->id) != 0)
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (!(definition->minimum <= definition->defaultV && definition->defaultV <= definition->maximum)
            || !finiteFloat(definition->minimum) || !finiteFloat(definition->maximum)
            || !finiteFloat(definition->defaultV))
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        ParameterRecord record;
        record.id          = definition->id;
        record.displayName = copyString(definition->displayName);
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
        // default value.
        for (auto &entry : characters_)
        {
            entry.second->parameterValues.emplace(record.id, record.defaultV);
        }
        return succeed(error);
    }

    bool registerCompatibilityRuleImpl(const CompatibilityRuleDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !stringNonEmpty(definition->id) || !stringNonEmpty(definition->parentRole)
            || !stringNonEmpty(definition->childRole))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.rules.size() >= budgets_.maximumRules)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        for (const CompatibilityRuleRecord &rule : registry_.rules)
        {
            if (rule.id == definition->id)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
        }
        if (!finiteFloat(definition->minimumRelativeScale) || !finiteFloat(definition->maximumRelativeScale)
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
        record.requiredParentTag    = copyString(definition->requiredParentTag);
        record.requiredChildTag     = copyString(definition->requiredChildTag);
        registry_.rules.push_back(std::move(record));
        return succeed(error);
    }

    bool registerRigImpl(const RigDefinition *definition, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (definition == nullptr || !stringNonEmpty(definition->id)
            || (definition->joints == nullptr && definition->jointCount != 0))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.rigs.size() >= budgets_.maximumRigs || definition->jointCount > budgets_.maximumJointsPerRig)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.rigIndices.count(definition->id) != 0)
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
            if (!stringNonEmpty(joint.semantic) || jointSemantics.count(joint.semantic) != 0)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            // Enforce parent-before-child ordering: a fragment is only valid
            // when every parent index references an earlier joint or -1.
            if (joint.parentIndex < -1 || joint.parentIndex >= static_cast<std::int32_t>(jointIndex))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (!finiteVector3(joint.translation) || !finiteQuaternion(joint.rotation)
                || !nonZeroQuaternion(joint.rotation) || !finiteVector3(joint.scale))
            {
                return fail(error, CharacterError::InvalidArgument);
            }

            RigJointRecord jointRecord;
            jointRecord.semantic    = joint.semantic;
            jointRecord.parentIndex = joint.parentIndex;
            jointRecord.translation = copyVector3(joint.translation);
            jointRecord.rotation    = copyQuaternion(joint.rotation);
            jointRecord.scale       = copyVector3(joint.scale);
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
        if (definition == nullptr || !stringNonEmpty(definition->id))
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
        if (registry_.partIndices.count(definition->id) != 0)
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (!finiteVector3(definition->boundsCenter) || !finiteFloat(definition->boundsRadius)
            || definition->boundsRadius <= 0.0f)
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        // Import-time validation: every referenced rig and parameter must be
        // registered before the part that uses it.
        const std::string rigId = copyString(definition->rigId);
        if (!rigId.empty() && findRig(registry_, rigId) == nullptr)
        {
            return fail(error, CharacterError::UnknownAsset);
        }

        PartRecord record;
        record.id          = definition->id;
        record.displayName = copyString(definition->displayName);
        if (record.displayName.empty())
        {
            record.displayName = record.id;
        }
        record.geometryId    = copyString(definition->geometryId);
        record.rigId         = rigId;
        record.boundsCenter  = copyVector3(definition->boundsCenter);
        record.boundsRadius  = definition->boundsRadius;
        record.triangleCount = definition->triangleCount;

        std::unordered_set<std::string> connectorIds;
        connectorIds.reserve(definition->connectorCount);
        for (std::uint32_t index = 0; index < definition->connectorCount; ++index)
        {
            if (definition->connectors == nullptr)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            ConnectorRecord connector;
            if (!copyConnectorDefinition(definition->connectors[index], connector, error))
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
            if (!stringNonEmpty(binding.parameter))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (findParameter(registry_, binding.parameter) == nullptr)
            {
                return fail(error, CharacterError::UnknownAsset);
            }
            if (!finiteFloat(binding.scale) || !finiteVector3(binding.offset))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            BindingRecord bindingRecord;
            bindingRecord.parameter = binding.parameter;
            bindingRecord.target    = binding.target;
            bindingRecord.targetId  = copyString(binding.targetId);
            bindingRecord.scale     = binding.scale;
            bindingRecord.offset    = copyVector3(binding.offset);
            // Import-time validation of the binding target reference.
            switch (bindingRecord.target)
            {
            case ParameterBindingTarget::JointTranslation:
                if (!rigHasJoint(record.rigId, bindingRecord.targetId))
                {
                    return fail(error, CharacterError::InvalidArgument);
                }
                break;
            case ParameterBindingTarget::ConnectorOffsetTranslation:
                if (findConnector(record, bindingRecord.targetId) == nullptr)
                {
                    return fail(error, CharacterError::UnknownConnector);
                }
                break;
            case ParameterBindingTarget::PartScaleX:
            case ParameterBindingTarget::PartScaleY:
            case ParameterBindingTarget::PartScaleZ:
                break;
            }
            record.bindings.push_back(std::move(bindingRecord));
        }

        std::unordered_set<std::string> slotIds;
        for (std::uint32_t index = 0; index < definition->materialSlotCount; ++index)
        {
            const MaterialSlotDefinition &slot = definition->materialSlots[index];
            if (!stringNonEmpty(slot.id) || !finiteVector3(slot.baseColor) || !finiteFloat(slot.roughness)
                || !finiteFloat(slot.metallic))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            MaterialSlotRecord slotRecord;
            slotRecord.id        = slot.id;
            slotRecord.baseColor = copyVector3(slot.baseColor);
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
            if (!stringNonEmpty(tag))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            record.tags.push_back(std::string(tag));
        }

        registry_.partIndices.emplace(record.id, registry_.parts.size());
        registry_.parts.push_back(std::move(record));
        return succeed(error);
    }

    bool unregisterPartImpl(const char *partId)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (!stringNonEmpty(partId))
        {
            return false;
        }
        const auto found = registry_.partIndices.find(partId);
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
        if (definition == nullptr || !stringNonEmpty(definition->id))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (registry_.profiles.size() >= budgets_.maximumProfiles)
        {
            return fail(error, CharacterError::CapacityExceeded);
        }
        if (registry_.profileIndices.count(definition->id) != 0)
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        ProfileRecord record;
        record.id          = definition->id;
        record.displayName = copyString(definition->displayName);
        if (record.displayName.empty())
        {
            record.displayName = record.id;
        }

        std::unordered_set<std::string> instanceNames;
        for (std::uint32_t index = 0; index < definition->partCount; ++index)
        {
            const ProfilePart &part = definition->parts[index];
            if (!stringNonEmpty(part.name) || !stringNonEmpty(part.partId))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (findPart(registry_, part.partId) == nullptr)
            {
                return fail(error, CharacterError::UnknownAsset);
            }
            if (!instanceNames.insert(part.name).second)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            record.parts.push_back({part.name, part.partId});
        }

        for (std::uint32_t index = 0; index < definition->connectionCount; ++index)
        {
            const ProfileConnection &connection = definition->connections[index];
            if (!stringNonEmpty(connection.parentName) || !stringNonEmpty(connection.parentConnector)
                || !stringNonEmpty(connection.childName) || !stringNonEmpty(connection.childConnector))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (instanceNames.count(connection.parentName) == 0 || instanceNames.count(connection.childName) == 0
                || connection.parentName == connection.childName)
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            if (!finiteVector3(connection.offsetTranslation) || !finiteQuaternion(connection.offsetRotation)
                || !nonZeroQuaternion(connection.offsetRotation))
            {
                return fail(error, CharacterError::InvalidArgument);
            }
            ProfileRecord::Connection connectionRecord;
            connectionRecord.parentName        = connection.parentName;
            connectionRecord.parentConnector   = connection.parentConnector;
            connectionRecord.childName         = connection.childName;
            connectionRecord.childConnector    = connection.childConnector;
            connectionRecord.offsetTranslation = copyVector3(connection.offsetTranslation);
            connectionRecord.offsetRotation    = copyQuaternion(connection.offsetRotation);
            record.connections.push_back(std::move(connectionRecord));
        }

        registry_.profileIndices.emplace(record.id, registry_.profiles.size());
        registry_.profiles.push_back(std::move(record));
        return succeed(error);
    }

    CharacterHandle createAssemblyImpl(const char *profileId, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        if (characters_.size() >= budgets_.maximumCharacters)
        {
            fail(error, CharacterError::CapacityExceeded);
            return invalidCharacter;
        }

        auto record = std::make_unique<CharacterRecord>();
        record->id  = nextCharacterId_++;
        if (stringNonEmpty(profileId))
        {
            const ProfileRecord *profile = findProfile(registry_, profileId);
            if (profile == nullptr)
            {
                fail(error, CharacterError::UnknownAsset);
                return invalidCharacter;
            }
            record->profileId = profile->id;
            std::unordered_map<std::string, InstanceId> profileInstances;
            for (const ProfileRecord::Part &part : profile->parts)
            {
                const PartRecord *partRecord = findPart(registry_, part.partId);
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
            for (const ProfileRecord::Connection &connection : profile->connections)
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
        for (const ParameterRecord &parameter : registry_.parameters)
        {
            record->parameterValues.emplace(parameter.id, parameter.defaultV);
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
        characters_.erase(character);
    }

    PartInstance *instanceFor(CharacterRecord &record, InstanceId instance) const
    {
        for (PartInstance &part : record.parts)
        {
            if (part.id == instance)
            {
                return &part;
            }
        }
        return nullptr;
    }

    const PartInstance *instanceFor(const CharacterRecord &record, InstanceId instance) const
    {
        for (const PartInstance &part : record.parts)
        {
            if (part.id == instance)
            {
                return &part;
            }
        }
        return nullptr;
    }

    InstanceId appendInstance(CharacterRecord &record, const PartRecord &part, const std::string &instanceName)
    {
        if (record.parts.size() >= budgets_.maximumPartsPerCharacter)
        {
            return invalidInstance;
        }
        PartInstance instance;
        instance.id     = nextInstanceId(record);
        instance.name   = instanceName;
        instance.partId = part.id;
        instance.part   = part;
        record.parts.push_back(std::move(instance));
        markStructureDirty(record);
        return record.parts.back().id;
    }

    // Instance ids are per-assembly, monotonically increasing and never
    // reused, so external references stay stable across removals.
    static InstanceId nextInstanceId(const CharacterRecord &record)
    {
        InstanceId maximum = invalidInstance;
        for (const PartInstance &part : record.parts)
        {
            maximum = std::max(maximum, part.id);
        }
        return maximum + 1;
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
        if (!stringNonEmpty(partId) || !stringNonEmpty(instanceName))
        {
            fail(error, CharacterError::InvalidArgument);
            return invalidInstance;
        }
        if (record->instancesByName.count(instanceName) != 0)
        {
            fail(error, CharacterError::ValidationFailed);
            return invalidInstance;
        }
        const PartRecord *part = findPart(registry_, partId);
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
        record->instancesByName.emplace(instanceName, instance);
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
        if (record == nullptr)
        {
            return false;
        }
        const PartInstance *part = instanceFor(*record, instance);
        if (part == nullptr)
        {
            return false;
        }
        record->connections.erase(std::remove_if(record->connections.begin(),
                                      record->connections.end(),
                                      [instance](const ConnectionRecord &connection)
                                      { return connection.parent == instance || connection.child == instance; }),
            record->connections.end());
        record->instancesByName.erase(part->name);
        record->parts.erase(std::remove_if(record->parts.begin(),
                                record->parts.end(),
                                [instance](const PartInstance &candidate) { return candidate.id == instance; }),
            record->parts.end());
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
        const PartRecord *replacement = findPart(registry_, newPartId);
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
        if (findConnector(parentPart->part, parentConnector) == nullptr
            || findConnector(childPart->part, childConnector) == nullptr)
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
        if (offsetTranslation != nullptr && !finiteVector3(offsetTranslation))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        if (offsetRotation != nullptr && (!finiteQuaternion(offsetRotation) || !nonZeroQuaternion(offsetRotation)))
        {
            return fail(error, CharacterError::InvalidArgument);
        }

        // Eager data-defined compatibility: the role pair must be allowed by
        // an explicit or (parentRole, childRole) rule, and connector capacity
        // must hold. Scale-range and tag conditions are checked by
        // validation, which also covers later part replacements.
        const ConnectorRecord         *parentConnectorRecord = findConnector(parentPart->part, parentConnector);
        const ConnectorRecord         *childConnectorRecord  = findConnector(childPart->part, childConnector);
        const CompatibilityRuleRecord *rule                  = nullptr;
        if (!parentConnectorRecord->compatibilityRule.empty())
        {
            for (const CompatibilityRuleRecord &candidate : registry_.rules)
            {
                if (candidate.id == parentConnectorRecord->compatibilityRule)
                {
                    rule = &candidate;
                    break;
                }
            }
        }
        if (rule == nullptr)
        {
            rule = findCompatibilityRuleForRoles(
                registry_.rules, parentConnectorRecord->role, childConnectorRecord->role);
        }
        if (rule == nullptr || !rule->allowed)
        {
            lastErrorMessage_ = "no compatibility rule allows role pair ('" + parentConnectorRecord->role + "', '"
                              + childConnectorRecord->role + "')";
            return fail(error, CharacterError::ValidationFailed);
        }
        {
            const ConnectionRecord *existing = connectionForChild(*record, child);
            const bool              sameTarget
                = existing != nullptr && existing->parent == parent && existing->parentConnector == parentConnector;
            if (existing == nullptr || !sameTarget)
            {
                std::uint32_t used = 0;
                for (const ConnectionRecord &connection : record->connections)
                {
                    if (connection.parent == parent && connection.parentConnector == parentConnector
                        && connection.child != child)
                    {
                        ++used;
                    }
                }
                if (used >= parentConnectorRecord->maxConnections)
                {
                    lastErrorMessage_ = "connector '" + std::string(parentConnector) + "' capacity "
                                      + std::to_string(parentConnectorRecord->maxConnections) + " exceeded";
                    return fail(error, CharacterError::ValidationFailed);
                }
            }
        }

        ConnectionRecord connectionRecord;
        connectionRecord.parent          = parent;
        connectionRecord.parentConnector = parentConnector;
        connectionRecord.child           = child;
        connectionRecord.childConnector  = childConnector;
        connectionRecord.offsetTranslation
            = offsetTranslation != nullptr ? copyVector3(offsetTranslation) : glm::vec3(0.0f);
        connectionRecord.offsetRotation
            = offsetRotation != nullptr ? copyQuaternion(offsetRotation) : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

        // Re-parenting: a child has exactly one structural parent, so an
        // existing incoming connection is replaced.
        if (ConnectionRecord *existing = connectionForChild(*record, child); existing != nullptr)
        {
            *existing = std::move(connectionRecord);
        }
        else
        {
            record->connections.push_back(std::move(connectionRecord));
        }
        markStructureDirty(*record);
        publishChange(character, CharacterChangeKind::StructureChanged);
        return succeed(error);
    }

    static bool wouldCreateCycle(const CharacterRecord &record, InstanceId parent, InstanceId child)
    {
        InstanceId current = parent;
        while (current != invalidInstance)
        {
            if (current == child)
            {
                return true;
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
        if ((translation != nullptr && !finiteVector3(translation))
            || (rotation != nullptr && (!finiteQuaternion(rotation) || !nonZeroQuaternion(rotation))))
        {
            return false;
        }
        if (translation != nullptr)
        {
            record->rootTranslation = copyVector3(translation);
        }
        if (rotation != nullptr)
        {
            record->rootRotation = copyQuaternion(rotation);
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
        if ((offsetTranslation != nullptr && !finiteVector3(offsetTranslation))
            || (offsetRotation != nullptr && (!finiteQuaternion(offsetRotation) || !nonZeroQuaternion(offsetRotation))))
        {
            return false;
        }
        if (offsetTranslation != nullptr)
        {
            connection->offsetTranslation = copyVector3(offsetTranslation);
        }
        if (offsetRotation != nullptr)
        {
            connection->offsetRotation = copyQuaternion(offsetRotation);
        }
        markTransformsDirty(*record);
        publishChange(character, CharacterChangeKind::OffsetsChanged);
        return true;
    }

    bool setParameterImpl(CharacterHandle character, const char *parameter, float value, CharacterError *error)
    {
        std::lock_guard stateLock(stateMutex_);
        lastErrorMessage_.clear();
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || !stringNonEmpty(parameter))
        {
            return fail(error, record == nullptr ? CharacterError::InvalidHandle : CharacterError::InvalidArgument);
        }
        const ParameterRecord *definition = findParameter(registry_, parameter);
        if (definition == nullptr)
        {
            return fail(error, CharacterError::UnknownAsset);
        }
        if (!finiteFloat(value))
        {
            return fail(error, CharacterError::InvalidArgument);
        }
        const float clamped                = std::clamp(value, definition->minimum, definition->maximum);
        record->parameterValues[parameter] = clamped;
        markParametersDirty(*record);
        publishChange(character, CharacterChangeKind::ParametersChanged);
        return succeed(error);
    }

    bool getParameterImpl(CharacterHandle character, const char *parameter, float *outValue)
    {
        std::lock_guard  stateLock(stateMutex_);
        CharacterRecord *record = recordFor(character);
        if (record == nullptr || !stringNonEmpty(parameter) || outValue == nullptr)
        {
            return false;
        }
        const auto found = record->parameterValues.find(parameter);
        if (found == record->parameterValues.end())
        {
            return false;
        }
        *outValue = found->second;
        return true;
    }

    // ---- validation -------------------------------------------------------

    void collectIssues(const CharacterRecord &record, std::vector<ValidationIssue> &issues) const
    {
        issues.clear();

        // Unique instance names.
        std::unordered_map<std::string, std::uint32_t> nameCounts;
        for (const PartInstance &part : record.parts)
        {
            ++nameCounts[part.name];
        }
        for (const PartInstance &part : record.parts)
        {
            if (nameCounts[part.name] > 1)
            {
                appendIssue(issues,
                    ValidationCode::DuplicateInstanceName,
                    part.id,
                    part.name,
                    "instance name is used by " + std::to_string(nameCounts[part.name]) + " instances");
            }
        }

        // Connections reference live instances and existing connectors.
        std::unordered_map<InstanceId, std::uint32_t> parentedCount;
        for (const ConnectionRecord &connection : record.connections)
        {
            const PartInstance *parent = instanceFor(record, connection.parent);
            const PartInstance *child  = instanceFor(record, connection.child);
            if (parent == nullptr || child == nullptr)
            {
                appendIssue(issues,
                    ValidationCode::UnknownAsset,
                    connection.child,
                    connection.childConnector,
                    "connection references a removed instance");
                continue;
            }
            if (findConnector(parent->part, connection.parentConnector) == nullptr)
            {
                appendIssue(issues,
                    ValidationCode::UnknownConnector,
                    connection.child,
                    connection.parentConnector,
                    "parent part '" + parent->partId + "' has no connector '" + connection.parentConnector + "'");
            }
            if (findConnector(child->part, connection.childConnector) == nullptr)
            {
                appendIssue(issues,
                    ValidationCode::UnknownConnector,
                    connection.child,
                    connection.childConnector,
                    "child part '" + child->partId + "' has no connector '" + connection.childConnector + "'");
            }
            ++parentedCount[connection.child];
            if (parentedCount[connection.child] > 1)
            {
                appendIssue(issues,
                    ValidationCode::MultipleParents,
                    connection.child,
                    child->name,
                    "instance has more than one structural parent");
            }
        }

        // Exactly one root.
        std::vector<InstanceId> roots;
        for (const PartInstance &part : record.parts)
        {
            if (parentedCount.count(part.id) == 0)
            {
                roots.push_back(part.id);
            }
        }
        if (record.parts.empty())
        {
            appendIssue(
                issues, ValidationCode::NoRoot, invalidInstance, record.profileId, "assembly contains no parts");
        }
        else if (roots.empty())
        {
            appendIssue(issues,
                ValidationCode::NoRoot,
                invalidInstance,
                record.profileId,
                "assembly has no root: every instance has a parent");
        }
        else if (roots.size() > 1)
        {
            for (const InstanceId root : roots)
            {
                const PartInstance *part = instanceFor(record, root);
                appendIssue(issues,
                    ValidationCode::MultipleRoots,
                    root,
                    part != nullptr ? part->name : "",
                    "assembly has more than one root instance");
            }
        }

        // Acyclic attachment tree: everything must be reachable from roots.
        {
            std::unordered_set<InstanceId> visited;
            std::vector<InstanceId>        pending(roots.begin(), roots.end());
            while (!pending.empty())
            {
                const InstanceId current = pending.back();
                pending.pop_back();
                if (!visited.insert(current).second)
                {
                    continue;
                }
                for (const ConnectionRecord &connection : record.connections)
                {
                    if (connection.parent == current && visited.count(connection.child) == 0)
                    {
                        pending.push_back(connection.child);
                    }
                }
            }
            for (const PartInstance &part : record.parts)
            {
                if (visited.count(part.id) == 0 && !record.parts.empty())
                {
                    appendIssue(issues,
                        ValidationCode::Cycle,
                        part.id,
                        part.name,
                        "instance is not reachable from a root (cyclic attachment)");
                }
            }
        }

        // Connector capacity.
        for (const PartInstance &part : record.parts)
        {
            for (const ConnectorRecord &connector : part.part.connectors)
            {
                std::uint32_t used = 0;
                for (const ConnectionRecord &connection : record.connections)
                {
                    if (connection.parent == part.id && connection.parentConnector == connector.id)
                    {
                        ++used;
                    }
                }
                if (used > connector.maxConnections)
                {
                    appendIssue(issues,
                        ValidationCode::ConnectorCapacity,
                        part.id,
                        part.name + "." + connector.id,
                        "connector allows " + std::to_string(connector.maxConnections) + " connection(s) but has "
                            + std::to_string(used));
                }
            }
        }

        // Compatibility rules, tag conditions, and relative scale limits.
        for (const ConnectionRecord &connection : record.connections)
        {
            const PartInstance *parent = instanceFor(record, connection.parent);
            const PartInstance *child  = instanceFor(record, connection.child);
            if (parent == nullptr || child == nullptr)
            {
                continue;
            }
            const ConnectorRecord *parentConnector = findConnector(parent->part, connection.parentConnector);
            const ConnectorRecord *childConnector  = findConnector(child->part, connection.childConnector);
            if (parentConnector == nullptr || childConnector == nullptr)
            {
                continue; // already reported above
            }

            const CompatibilityRuleRecord *rule = nullptr;
            if (!parentConnector->compatibilityRule.empty())
            {
                for (const CompatibilityRuleRecord &candidate : registry_.rules)
                {
                    if (candidate.id == parentConnector->compatibilityRule)
                    {
                        rule = &candidate;
                        break;
                    }
                }
            }
            if (rule == nullptr)
            {
                rule = findCompatibilityRuleForRoles(registry_.rules, parentConnector->role, childConnector->role);
            }
            const std::string subject = child->name + " -> " + parent->name + "." + parentConnector->id;
            if (rule == nullptr)
            {
                appendIssue(issues,
                    ValidationCode::IncompatibleConnection,
                    connection.child,
                    subject,
                    "no compatibility rule for role pair ('" + parentConnector->role + "', '" + childConnector->role
                        + "')");
            }
            else if (!rule->allowed)
            {
                appendIssue(issues,
                    ValidationCode::IncompatibleConnection,
                    connection.child,
                    subject,
                    "rule '" + rule->id + "' forbids role pair ('" + parentConnector->role + "', '"
                        + childConnector->role + "')");
            }
            else
            {
                if (!rule->requiredParentTag.empty() && !partHasTag(parent->part, rule->requiredParentTag))
                {
                    appendIssue(issues,
                        ValidationCode::RequiredTagMissing,
                        connection.parent,
                        subject,
                        "parent part is missing required tag '" + rule->requiredParentTag + "'");
                }
                if (!rule->requiredChildTag.empty() && !partHasTag(child->part, rule->requiredChildTag))
                {
                    appendIssue(issues,
                        ValidationCode::RequiredTagMissing,
                        connection.child,
                        subject,
                        "child part is missing required tag '" + rule->requiredChildTag + "'");
                }
                const glm::vec3 parentScale = parent->scale;
                const glm::vec3 childScale  = child->scale;
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    const float parentAxis = std::max(parentScale[static_cast<glm::vec3::length_type>(axis)], 1.0e-4f);
                    const float ratio      = childScale[static_cast<glm::vec3::length_type>(axis)] / parentAxis;
                    if (ratio < rule->minimumRelativeScale || ratio > rule->maximumRelativeScale)
                    {
                        appendIssue(issues,
                            ValidationCode::ScaleOutOfRange,
                            connection.child,
                            subject,
                            "relative scale " + std::to_string(ratio) + " outside ["
                                + std::to_string(rule->minimumRelativeScale) + ", "
                                + std::to_string(rule->maximumRelativeScale) + "]");
                        break;
                    }
                }
            }
        }

        // Joint references from anchors and bindings.
        for (const PartInstance &part : record.parts)
        {
            if (part.part.rigId.empty())
            {
                continue;
            }
            const RigRecord *rig = findRig(registry_, part.part.rigId);
            if (rig == nullptr)
            {
                appendIssue(issues,
                    ValidationCode::UnknownAsset,
                    part.id,
                    part.name,
                    "part references unregistered rig '" + part.part.rigId + "'");
                continue;
            }
            for (const ConnectorRecord &connector : part.part.connectors)
            {
                if (connector.anchorSpace == AnchorSpace::Joint
                    && rigJointIndex(*rig, connector.anchorJointSemantic) < 0)
                {
                    appendIssue(issues,
                        ValidationCode::InvalidJointReference,
                        part.id,
                        part.name + "." + connector.id,
                        "anchor references missing joint '" + connector.anchorJointSemantic + "'");
                }
            }
            for (const BindingRecord &binding : part.part.bindings)
            {
                if (binding.target == ParameterBindingTarget::JointTranslation
                    && rigJointIndex(*rig, binding.targetId) < 0)
                {
                    appendIssue(issues,
                        ValidationCode::InvalidJointReference,
                        part.id,
                        part.name,
                        "binding references missing joint '" + binding.targetId + "'");
                }
            }
        }

        // Parameter ranges.
        for (const auto &entry : record.parameterValues)
        {
            const ParameterRecord *definition = findParameter(registry_, entry.first);
            if (definition != nullptr && (entry.second < definition->minimum || entry.second > definition->maximum))
            {
                appendIssue(issues,
                    ValidationCode::ParameterOutOfRange,
                    invalidInstance,
                    entry.first,
                    "value " + std::to_string(entry.second) + " outside [" + std::to_string(definition->minimum) + ", "
                        + std::to_string(definition->maximum) + "]");
            }
        }
    }

    static bool partHasTag(const PartRecord &part, const std::string &tag)
    {
        for (const std::string &candidate : part.tags)
        {
            if (candidate == tag)
            {
                return true;
            }
        }
        return false;
    }

    static std::int32_t rigJointIndex(const RigRecord &rig, const std::string &semantic)
    {
        for (std::uint32_t index = 0; index < rig.joints.size(); ++index)
        {
            if (rig.joints[index].semantic == semantic)
            {
                return static_cast<std::int32_t>(index);
            }
        }
        return -1;
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

    // ---- evaluation -------------------------------------------------------

    // Computes the per-instance parameter scale factors (mesh scale) from the
    // part bindings and the assembly's parameter values.
    void applyParameterScales(const CharacterRecord &record, PartInstance &part) const
    {
        glm::vec3 scale(1.0f);
        for (const BindingRecord &binding : part.part.bindings)
        {
            if (binding.target != ParameterBindingTarget::PartScaleX
                && binding.target != ParameterBindingTarget::PartScaleY
                && binding.target != ParameterBindingTarget::PartScaleZ)
            {
                continue;
            }
            const auto found = record.parameterValues.find(binding.parameter);
            if (found == record.parameterValues.end())
            {
                continue;
            }
            const ParameterRecord *definition = findParameter(registry_, binding.parameter);
            if (definition == nullptr)
            {
                continue;
            }
            const float delta = found->second - definition->defaultV;
            scale[static_cast<glm::vec3::length_type>(binding.target)] *= 1.0f + delta * binding.scale;
        }
        part.scale = glm::clamp(scale, glm::vec3(0.01f), glm::vec3(100.0f));
    }

    // Builds the rig's world-within-part matrices for the current (parameter
    // adjusted) local pose. Returns false when the rig cannot be resolved.
    bool evaluateRig(const CharacterRecord &record, PartInstance &part, std::vector<glm::mat4> &jointMatrices) const
    {
        jointMatrices.clear();
        if (part.part.rigId.empty())
        {
            return true;
        }
        const RigRecord *rig = findRig(registry_, part.part.rigId);
        if (rig == nullptr)
        {
            return false;
        }
        jointMatrices.resize(rig->joints.size());
        for (std::uint32_t index = 0; index < rig->joints.size(); ++index)
        {
            const RigJointRecord &joint       = rig->joints[index];
            glm::vec3             translation = joint.translation;
            // JointTranslation bindings shift rest positions so anchors stay
            // synchronized with parameter-driven deformation.
            for (const BindingRecord &binding : part.part.bindings)
            {
                if (binding.target != ParameterBindingTarget::JointTranslation || binding.targetId != joint.semantic)
                {
                    continue;
                }
                const auto             found      = record.parameterValues.find(binding.parameter);
                const ParameterRecord *definition = findParameter(registry_, binding.parameter);
                if (found != record.parameterValues.end() && definition != nullptr)
                {
                    translation += (found->second - definition->defaultV) * binding.offset;
                }
            }
            const glm::mat4 local
                = makeTrsMatrix(translation, joint.rotation) * glm::scale(glm::mat4(1.0f), joint.scale);
            jointMatrices[index]
                = joint.parentIndex >= 0 ? jointMatrices[static_cast<std::uint32_t>(joint.parentIndex)] * local : local;
        }
        return true;
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
        // (structure or parameters). A pure transform refresh - the per-frame
        // animation drivers - keeps the last issue list and skips the walk
        // entirely when nothing moved.
        if (structureChanged || parametersChanged)
        {
            if (budgets_.validateOnEvaluate)
            {
                collectIssues(*record, record->lastIssues);
            }
            else
            {
                record->lastIssues.clear();
            }
            if (!record->lastIssues.empty())
            {
                record->statistics.issueCount = static_cast<std::uint32_t>(record->lastIssues.size());
                record->structureDirty        = false;
                record->parametersDirty       = false;
                return fail(error, CharacterError::ValidationFailed);
            }
        }
        if (!structureChanged && !parametersChanged && !record->transformsDirty && !record->boundsDirty
            && record->boundsValid)
        {
            return succeed(error);
        }

        // Child adjacency, rebuilt only after structure changes; the walk
        // below is O(parts + connections) instead of scanning every
        // connection for every part.
        if (!record->adjacencyValid)
        {
            record->childConnections.assign(record->parts.size(), {});
            for (std::size_t index = 0; index < record->connections.size(); ++index)
            {
                const auto parent = std::find_if(record->parts.begin(), record->parts.end(),
                    [&connection = record->connections[index]](const PartInstance &part)
                    { return part.id == connection.parent; });
                if (parent != record->parts.end())
                {
                    const std::size_t parentIndex = static_cast<std::size_t>(parent - record->parts.begin());
                    record->childConnections[parentIndex].push_back(index);
                }
            }
            record->adjacencyValid = true;
        }

        // Resolved rigs per instance (kept alive for the tree walk below).
        std::vector<std::vector<glm::mat4>> rigPoses(record->parts.size());
        for (std::size_t index = 0; index < record->parts.size(); ++index)
        {
            PartInstance &part = record->parts[index];
            applyParameterScales(*record, part);
            if (!evaluateRig(*record, part, rigPoses[index]))
            {
                return fail(error, CharacterError::UnknownAsset);
            }
        }

        // Root placement.
        const glm::mat4 rootMatrix = makeTrsMatrix(record->rootTranslation, record->rootRotation);

        // Map instance id -> position in the parts vector for child lookups.
        std::unordered_map<InstanceId, std::size_t> instancePositions;
        instancePositions.reserve(record->parts.size() * 2);
        for (std::size_t index = 0; index < record->parts.size(); ++index)
        {
            instancePositions.emplace(record->parts[index].id, index);
        }

        // Connector lookup. A connector anchor resolves to a position and
        // rotation in the owning part's local frame; the walk then lifts the
        // parent socket into world space with the part's parameter scale
        // applied to the position (deformed geometry lives in the scaled
        // space, so anchors must follow it) and the rotation kept rigid
        // (parameter scale never shears attachment frames).
        struct ResolvedSocket
        {
            const ConnectorRecord *connector = nullptr;
            glm::vec3              position{0.0f};
            glm::quat              rotation{1.0f, 0.0f, 0.0f, 0.0f};
        };
        auto resolveSocket = [&](const PartInstance           &part,
                                 const std::string            &connectorId,
                                 const std::vector<glm::mat4> &joints) -> ResolvedSocket
        {
            ResolvedSocket socket;
            socket.connector = findConnector(part.part, connectorId);
            if (socket.connector == nullptr)
            {
                return socket;
            }
            const glm::mat4 *jointMatrix = nullptr;
            if (socket.connector->anchorSpace == AnchorSpace::Joint && !part.part.rigId.empty())
            {
                const RigRecord   *rig = findRig(registry_, part.part.rigId);
                const std::int32_t jointIndex
                    = rig != nullptr ? rigJointIndex(*rig, socket.connector->anchorJointSemantic) : -1;
                if (jointIndex >= 0 && static_cast<std::uint32_t>(jointIndex) < joints.size())
                {
                    jointMatrix = &joints[static_cast<std::uint32_t>(jointIndex)];
                }
            }
            const glm::mat4 anchorMatrix
                = makeTrsMatrix(socket.connector->anchorTranslation, socket.connector->anchorRotation);
            const glm::mat4 socketLocal = jointMatrix != nullptr ? (*jointMatrix) * anchorMatrix : anchorMatrix;
            socket.position             = glm::vec3(socketLocal[3]);
            socket.rotation             = glm::quat_cast(glm::mat3(socketLocal));
            return socket;
        };

        // Connection lookup by child for the tree walk.
        std::unordered_map<InstanceId, const ConnectionRecord *> incoming;
        for (const ConnectionRecord &connection : record->connections)
        {
            incoming.emplace(connection.child, &connection);
        }

        // Walk the attachment tree from the root, composing connector-aligned
        // rigid transforms; parameter scale rides on socket positions and on
        // the instance render matrix, never on the inherited frames. The
        // explicit stack avoids a heap-allocated recursive std::function.
        bool                                              walkFailed = false;
        std::vector<std::pair<std::size_t, glm::mat4>>    walkStack;
        const auto                                        pushChildren = [&](std::size_t partIndex,
            const glm::mat4 &parentRigidWorld)
        {
            PartInstance &part                  = record->parts[partIndex];
            part.rigidWorld                     = parentRigidWorld;
            part.world                          = parentRigidWorld * glm::scale(glm::mat4(1.0f), part.scale);
            part.transformValid                 = true;
            const glm::quat parentWorldRotation = glm::quat_cast(glm::mat3(parentRigidWorld));
            for (const std::size_t connectionIndex : record->childConnections[partIndex])
            {
                const ConnectionRecord &connection = record->connections[connectionIndex];
                const auto              childPosition = instancePositions.find(connection.child);
                if (childPosition == instancePositions.end())
                {
                    walkFailed = true;
                    return;
                }
                const PartInstance  &childPart = record->parts[childPosition->second];
                const ResolvedSocket parentSocket
                    = resolveSocket(part, connection.parentConnector, rigPoses[partIndex]);
                const ResolvedSocket childSocket
                    = resolveSocket(childPart, connection.childConnector, rigPoses[childPosition->second]);
                if (parentSocket.connector == nullptr || childSocket.connector == nullptr)
                {
                    walkFailed = true;
                    return;
                }
                // T_child = T_socket(parent, scale-applied) * T_offset
                //           * inverse(T_socket(child, scale-applied)), with
                // every scale folded into positions so T_child stays rigid.
                const glm::vec3 parentSocketWorld(
                    parentRigidWorld * glm::vec4(part.scale * parentSocket.position, 1.0f));
                const glm::quat parentSocketRotation = parentWorldRotation * parentSocket.rotation;
                const glm::quat childRotation
                    = parentSocketRotation * connection.offsetRotation * glm::inverse(childSocket.rotation);
                const glm::vec3 childTranslation = parentSocketWorld
                                                 + parentSocketRotation * connection.offsetTranslation
                                                 - childRotation * (childPart.scale * childSocket.position);
                walkStack.emplace_back(childPosition->second, makeTrsMatrix(childTranslation, childRotation));
            }
        };

        for (std::size_t index = 0; index < record->parts.size() && !walkFailed; ++index)
        {
            if (incoming.count(record->parts[index].id) == 0)
            {
                walkStack.emplace_back(index, rootMatrix);
                while (!walkStack.empty() && !walkFailed)
                {
                    const auto [partIndex, parentWorld] = walkStack.back();
                    walkStack.pop_back();
                    pushChildren(partIndex, parentWorld);
                }
            }
        }
        if (walkFailed)
        {
            return fail(error, CharacterError::ValidationFailed);
        }

        // Bounds: union of the transformed part bound spheres.
        glm::vec3 boundsCenter(0.0f);
        float     boundsRadius = 0.0f;
        bool      haveBounds   = false;
        for (const PartInstance &part : record->parts)
        {
            if (!part.transformValid)
            {
                continue;
            }
            const glm::vec3 center(part.world * glm::vec4(part.part.boundsCenter, 1.0f));
            const float radius = part.part.boundsRadius * std::max(part.scale.x, std::max(part.scale.y, part.scale.z));
            if (!haveBounds)
            {
                boundsCenter = center;
                boundsRadius = radius;
                haveBounds   = true;
            }
            else
            {
                const float distance = glm::length(center - boundsCenter);
                if (boundsRadius >= distance + radius)
                {
                    // Existing bounds already contain the part.
                }
                else if (radius >= distance + boundsRadius)
                {
                    boundsCenter = center;
                    boundsRadius = radius;
                }
                else
                {
                    boundsRadius = (boundsRadius + distance + radius) * 0.5f;
                }
            }
        }
        record->boundsCenter = boundsCenter;
        record->boundsRadius = boundsRadius;
        record->boundsValid  = haveBounds;

        // Statistics.
        std::uint32_t jointCount = 0;
        for (const PartInstance &part : record->parts)
        {
            if (!part.part.rigId.empty())
            {
                if (const RigRecord *rig = findRig(registry_, part.part.rigId); rig != nullptr)
                {
                    jointCount += static_cast<std::uint32_t>(rig->joints.size());
                }
            }
        }
        record->statistics.partCount       = static_cast<std::uint32_t>(record->parts.size());
        record->statistics.connectionCount = static_cast<std::uint32_t>(record->connections.size());
        record->statistics.jointCount      = jointCount;
        record->statistics.triangleCount   = 0;
        for (const PartInstance &part : record->parts)
        {
            record->statistics.triangleCount += part.part.triangleCount;
        }
        record->statistics.issueCount = 0;

        record->structureDirty  = false;
        record->parametersDirty = false;
        record->transformsDirty = false;
        record->boundsDirty     = false;
        // Publish only real changes: the per-frame animation path already
        // published OffsetsChanged from setConnectionOffset, and an
        // Evaluated event on every transform refresh would spam the bus.
        if (structureChanged || parametersChanged)
        {
            publishChange(character, CharacterChangeKind::Evaluated);
        }
        return succeed(error);
    }

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
        if (!internal::loadRecipeFromFile(registry_, *record, filePath, errorMessage))
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

    // ---- helpers ----------------------------------------------------------

    CharacterRecord *recordFor(CharacterHandle character)
    {
        const auto found = characters_.find(character);
        return found != characters_.end() ? found->second.get() : nullptr;
    }

    static void markStructureDirty(CharacterRecord &record)
    {
        record.structureDirty  = true;
        record.parametersDirty = true;
        record.transformsDirty = true;
        record.boundsDirty     = true;
        record.adjacencyValid  = false;
    }

    static void markParametersDirty(CharacterRecord &record)
    {
        record.parametersDirty = true;
        record.transformsDirty = true;
        record.boundsDirty     = true;
    }

    static void markTransformsDirty(CharacterRecord &record)
    {
        record.transformsDirty = true;
        record.boundsDirty     = true;
    }

    bool rigHasJoint(const std::string &rigId, const std::string &semantic) const
    {
        const RigRecord *rig = findRig(registry_, rigId);
        return rig != nullptr && rigJointIndex(*rig, semantic) >= 0;
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

    void assignCapabilityPointers()
    {
        capability_.abi               = characterCapabilityAbi;
        capability_.self              = this;
        capability_.registerParameter = [](void *self, const ParameterDefinition *definition, CharacterError *error)
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

    ParticipantContext                                                   *context_ = nullptr;
    Registry                                                              registry_;
    Budgets                                                               budgets_;
    std::unordered_map<CharacterHandle, std::unique_ptr<CharacterRecord>> characters_;
    CharacterHandle                                                       nextCharacterId_ = 1;
    std::string                                                           lastErrorMessage_;
    mutable std::recursive_mutex                                          stateMutex_;

    script_runtime::ScriptRuntimeCapability       *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_
        = script_runtime::invalidScriptRuntimeInitializer;
    CharacterLuaBridgeContext luaBridgeContext_{};

    ParticipantManifest manifest_;
    CharacterCapability capability_{};
};

} // namespace engine::character

// Module entry point queried by the kernel after dlopen.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"character"};
    static const char *const optionalCapabilities[] = {"script_runtime"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "character",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::character::CharacterModule(); },
        .providedCapabilities = providedCapabilities,
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
