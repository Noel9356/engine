// Validation pass - see validation.hpp.

#include "validation.hpp"

#include "math_utils.hpp"
#include "registry.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine::character
{
namespace internal
{
namespace
{

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

const PartInstance *instanceFor(const CharacterRecord &record, InstanceId instance)
{
    const auto found = record.instanceIndicesById.find(instance);
    return found != record.instanceIndicesById.end() ? &record.parts[found->second] : nullptr;
}

} // namespace

void collectIssues(const Registry &registry, const CharacterRecord &record, std::vector<ValidationIssue> &issues)
{
    issues.clear();

    // Unique instance names.
    std::unordered_map<std::string_view, std::uint32_t> nameCounts;
    nameCounts.reserve(record.parts.size());
    for (const PartInstance &part : record.parts)
    {
        ++nameCounts[part.name];
    }
    for (const PartInstance &part : record.parts)
    {
        const std::uint32_t uses = nameCounts[std::string_view(part.name)];
        if (uses > 1)
        {
            appendIssue(issues,
                ValidationCode::DuplicateInstanceName,
                part.id,
                part.name,
                "instance name is used by " + std::to_string(uses) + " instances");
        }
    }

    // Connections reference live instances and existing connectors.
    std::unordered_map<InstanceId, std::uint32_t> parentedCount;
    parentedCount.reserve(record.connections.size());
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
    roots.reserve(record.parts.size());
    for (const PartInstance &part : record.parts)
    {
        if (parentedCount.count(part.id) == 0)
        {
            roots.push_back(part.id);
        }
    }
    if (record.parts.empty())
    {
        appendIssue(issues, ValidationCode::NoRoot, invalidInstance, record.profileId, "assembly contains no parts");
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
            if (visited.count(part.id) == 0)
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

        const CompatibilityRuleRecord *rule = resolveCompatibilityRule(registry, *parentConnector, *childConnector);
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

    // Joint references from anchors and bindings. Registration resolves and
    // verifies every reference; the cached indices re-check them against the
    // current registry state as defense in depth.
    for (const PartInstance &part : record.parts)
    {
        if (part.part.rigId.empty())
        {
            continue;
        }
        const RigRecord *rig = part.part.rigIndex != noRigIndex ? &registry.rigs[part.part.rigIndex] : nullptr;
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
            if (connector.anchorSpace == AnchorSpace::Joint && connector.anchorJointIndex < 0)
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
            if (binding.target == ParameterBindingTarget::JointTranslation && binding.targetIndex < 0)
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
    const std::size_t parameterCount = std::min(registry.parameters.size(), record.parameterValues.size());
    for (std::size_t index = 0; index < parameterCount; ++index)
    {
        const ParameterRecord &definition = registry.parameters[index];
        const float            value      = record.parameterValues[index];
        if (value < definition.minimum || value > definition.maximum)
        {
            appendIssue(issues,
                ValidationCode::ParameterOutOfRange,
                invalidInstance,
                definition.id,
                "value " + std::to_string(value) + " outside [" + std::to_string(definition.minimum) + ", "
                    + std::to_string(definition.maximum) + "]");
        }
    }
}

} // namespace internal
} // namespace engine::character
