// Registry construction and lookup helpers - see registry.hpp.

#include "registry.hpp"

#include "math_utils.hpp"

#include <optional>

namespace engine::character
{
namespace internal
{

void Registry::indexRule(std::uint32_t position)
{
    const CompatibilityRuleRecord &rule = rules[position];
    ruleIdIndices.emplace(rule.id, position);
    if (rule.parentRole == "*" || rule.childRole == "*")
    {
        wildcardRules.push_back(position);
    }
}

std::uint32_t findParameterIndex(const Registry &registry, std::string_view id)
{
    const auto found = registry.parameterIndices.find(id);
    return found != registry.parameterIndices.end() ? found->second : unresolvedIndex;
}

const PartRecord *findPartRecord(const Registry &registry, std::string_view id)
{
    const auto found = registry.partIndices.find(id);
    return found != registry.partIndices.end() ? &registry.parts[found->second] : nullptr;
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

std::int32_t rigJointIndex(const RigRecord &rig, const std::string &semantic)
{
    for (std::size_t index = 0; index < rig.joints.size(); ++index)
    {
        if (rig.joints[index].semantic == semantic)
        {
            return static_cast<std::int32_t>(index);
        }
    }
    return -1;
}

bool partHasTag(const PartRecord &part, const std::string &tag)
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

const CompatibilityRuleRecord *resolveCompatibilityRule(
    const Registry &registry, const ConnectorRecord &parentConnector, const ConnectorRecord &childConnector)
{
    if (!parentConnector.compatibilityRule.empty())
    {
        const auto found = registry.ruleIdIndices.find(parentConnector.compatibilityRule);
        if (found != registry.ruleIdIndices.end())
        {
            return &registry.rules[found->second];
        }
    }

    // Exact role pair wins; otherwise the first wildcard rule in
    // registration order.
    for (const CompatibilityRuleRecord &rule : registry.rules)
    {
        if (rule.parentRole == parentConnector.role && rule.childRole == childConnector.role)
        {
            return &rule;
        }
    }
    for (const std::uint32_t position : registry.wildcardRules)
    {
        const CompatibilityRuleRecord &rule = registry.rules[position];
        const bool parentMatches = rule.parentRole == "*" || rule.parentRole == parentConnector.role;
        const bool childMatches  = rule.childRole == "*" || rule.childRole == childConnector.role;
        if (parentMatches && childMatches)
        {
            return &rule;
        }
    }
    return nullptr;
}

bool copyConnectorDefinition(const ConnectorDefinition &definition, ConnectorRecord &record, CharacterError *error)
{
    const bool malformed = !stringNonEmpty(definition.id) || !stringNonEmpty(definition.role)
        || (definition.anchor.space == AnchorSpace::Joint && !stringNonEmpty(definition.anchor.jointSemantic))
        || !finiteVector3(definition.anchor.translation) || !finiteQuaternion(definition.anchor.rotation)
        || !nonZeroQuaternion(definition.anchor.rotation) || definition.maxConnections == 0
        || definition.maxConnections > kMaximumConnectionsPerConnector;
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

bool resolvePartReferences(const Registry &registry, PartRecord &part, CharacterError *error)
{
    const RigRecord *rig = part.rigIndex != noRigIndex ? &registry.rigs[part.rigIndex] : nullptr;

    // Joint-space anchors must name a joint of the part's rig.
    for (ConnectorRecord &connector : part.connectors)
    {
        if (connector.anchorSpace != AnchorSpace::Joint)
        {
            continue;
        }
        if (rig == nullptr)
        {
            if (error != nullptr)
            {
                *error = CharacterError::InvalidArgument;
            }
            return false;
        }
        connector.anchorJointIndex = rigJointIndex(*rig, connector.anchorJointSemantic);
        if (connector.anchorJointIndex < 0)
        {
            if (error != nullptr)
            {
                *error = CharacterError::InvalidArgument;
            }
            return false;
        }
    }

    // Bindings must name registered parameters and resolvable targets.
    for (BindingRecord &binding : part.bindings)
    {
        const std::uint32_t parameterIndex = findParameterIndex(registry, binding.parameter);
        if (parameterIndex == unresolvedIndex)
        {
            if (error != nullptr)
            {
                *error = CharacterError::UnknownAsset;
            }
            return false;
        }
        binding.parameterIndex = parameterIndex;

        switch (binding.target)
        {
        case ParameterBindingTarget::JointTranslation:
            if (rig == nullptr)
            {
                if (error != nullptr)
                {
                    *error = CharacterError::InvalidArgument;
                }
                return false;
            }
            binding.targetIndex = rigJointIndex(*rig, binding.targetId);
            if (binding.targetIndex < 0)
            {
                if (error != nullptr)
                {
                    *error = CharacterError::InvalidArgument;
                }
                return false;
            }
            break;
        case ParameterBindingTarget::ConnectorOffsetTranslation: {
            for (std::uint32_t slot = 0; slot < part.connectors.size(); ++slot)
            {
                if (part.connectors[slot].id == binding.targetId)
                {
                    binding.targetIndex = static_cast<std::int32_t>(slot);
                    break;
                }
            }
            if (binding.targetIndex < 0)
            {
                if (error != nullptr)
                {
                    *error = CharacterError::UnknownConnector;
                }
                return false;
            }
            break;
        }
        case ParameterBindingTarget::PartScaleX:
        case ParameterBindingTarget::PartScaleY:
        case ParameterBindingTarget::PartScaleZ:
            break;
        }
    }
    return true;
}

} // namespace internal
} // namespace engine::character
