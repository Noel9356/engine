// Evaluation stages - see evaluation.hpp.

#include "evaluation.hpp"

#include "math_utils.hpp"
#include "registry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace engine::character
{
namespace internal
{

void prepareTopology(const Registry &registry, CharacterRecord &record)
{
    (void)registry;

    // Resolve every connection to part/connector indices once per structure
    // change; the walk then runs without string compares or hash lookups.
    // Connections that reference removed instances or missing connectors keep
    // sentinel indices so the walk (or the validation pass) reports them.
    record.resolvedConnections.assign(record.connections.size(), ResolvedConnection{});
    std::vector<bool> hasParent(record.parts.size(), false);
    for (std::size_t index = 0; index < record.connections.size(); ++index)
    {
        const ConnectionRecord &connection = record.connections[index];
        ResolvedConnection     &resolved   = record.resolvedConnections[index];
        const auto              parent     = record.instanceIndicesById.find(connection.parent);
        const auto              child      = record.instanceIndicesById.find(connection.child);
        if (parent == record.instanceIndicesById.end() || child == record.instanceIndicesById.end())
        {
            continue;
        }
        resolved.parentIndex = parent->second;
        resolved.childIndex  = child->second;

        const PartInstance &parentPart = record.parts[resolved.parentIndex];
        const PartInstance &childPart  = record.parts[resolved.childIndex];
        for (std::uint32_t slot = 0; slot < parentPart.part.connectors.size(); ++slot)
        {
            if (parentPart.part.connectors[slot].id == connection.parentConnector)
            {
                resolved.parentConnectorSlot = slot;
                break;
            }
        }
        for (std::uint32_t slot = 0; slot < childPart.part.connectors.size(); ++slot)
        {
            if (childPart.part.connectors[slot].id == connection.childConnector)
            {
                resolved.childConnectorSlot = slot;
                break;
            }
        }
        // A child has exactly one structural parent; when crafted state
        // carries more than one incoming connection, the first in connection
        // order wins the transform walk, matching the previous map-based
        // behaviour (validation reports the extra parents).
        if (!hasParent[resolved.childIndex])
        {
            hasParent[resolved.childIndex]   = true;
            record.parts[resolved.childIndex].parent = connection.parent;
        }
    }

    record.childConnections.assign(record.parts.size(), {});
    for (std::size_t index = 0; index < record.resolvedConnections.size(); ++index)
    {
        const ResolvedConnection &resolved = record.resolvedConnections[index];
        if (resolved.parentIndex == unresolvedIndex || resolved.childIndex == unresolvedIndex)
        {
            continue;
        }
        record.childConnections[resolved.parentIndex].push_back(index);
    }

    record.rootIndices.clear();
    for (std::size_t index = 0; index < record.parts.size(); ++index)
    {
        if (!hasParent[index])
        {
            record.rootIndices.push_back(static_cast<std::uint32_t>(index));
            record.parts[index].parent = invalidInstance;
        }
    }

    record.topologyValid = true;
}

void applyParameters(const Registry &registry, CharacterRecord &record)
{
    const std::size_t parameterCount = std::min(registry.parameters.size(), record.parameterValues.size());

    // Per-part parameter scale and rig poses.
    std::vector<glm::vec3> jointDeltas;
    for (PartInstance &part : record.parts)
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
            if (binding.parameterIndex >= parameterCount)
            {
                continue;
            }
            const ParameterRecord &definition = registry.parameters[binding.parameterIndex];
            const float            delta      = record.parameterValues[binding.parameterIndex] - definition.defaultV;
            scale[static_cast<glm::vec3::length_type>(binding.target)] *= 1.0f + delta * binding.scale;
        }
        part.scale = glm::clamp(scale, glm::vec3(kMinimumPartScale), glm::vec3(kMaximumPartScale));

        part.rigPoses.clear();
        const RigRecord *rig = part.part.rigIndex != noRigIndex ? &registry.rigs[part.part.rigIndex] : nullptr;
        if (rig == nullptr)
        {
            part.sockets.clear();
            continue;
        }

        // JointTranslation bindings shift rest positions so anchors stay
        // synchronized with parameter-driven deformation; accumulate the
        // per-joint deltas in one pass over the bindings instead of scanning
        // the bindings for every joint.
        jointDeltas.assign(rig->joints.size(), glm::vec3(0.0f));
        for (const BindingRecord &binding : part.part.bindings)
        {
            if (binding.target != ParameterBindingTarget::JointTranslation || binding.targetIndex < 0
                || static_cast<std::size_t>(binding.targetIndex) >= jointDeltas.size()
                || binding.parameterIndex >= parameterCount)
            {
                continue;
            }
            const ParameterRecord &definition = registry.parameters[binding.parameterIndex];
            jointDeltas[static_cast<std::size_t>(binding.targetIndex)]
                += (record.parameterValues[binding.parameterIndex] - definition.defaultV) * binding.offset;
        }

        part.rigPoses.resize(rig->joints.size());
        for (std::size_t index = 0; index < rig->joints.size(); ++index)
        {
            const RigJointRecord &joint = rig->joints[index];
            const glm::vec3       translation = joint.translation + jointDeltas[index];
            const glm::mat4       local
                = makeTrsMatrix(translation, joint.rotation) * glm::scale(glm::mat4(1.0f), joint.scale);
            part.rigPoses[index]
                = joint.parentIndex >= 0 ? part.rigPoses[static_cast<std::size_t>(joint.parentIndex)] * local : local;
        }
    }

    // Resolved socket frames: a connector anchor becomes a position and
    // rotation in the owning part's local frame. Positions ride the rig pose
    // (parameter-adjusted); rotations stay rigid. Refreshed together with the
    // rig poses so the transform walk reuses them as-is.
    for (PartInstance &part : record.parts)
    {
        part.sockets.assign(part.part.connectors.size(), PartInstance::Socket{});
        for (std::size_t slot = 0; slot < part.part.connectors.size(); ++slot)
        {
            const ConnectorRecord &connector = part.part.connectors[slot];
            const glm::mat4       *jointMatrix = nullptr;
            if (connector.anchorSpace == AnchorSpace::Joint && connector.anchorJointIndex >= 0
                && static_cast<std::size_t>(connector.anchorJointIndex) < part.rigPoses.size())
            {
                jointMatrix = &part.rigPoses[static_cast<std::size_t>(connector.anchorJointIndex)];
            }
            const glm::mat4 anchorMatrix = makeTrsMatrix(connector.anchorTranslation, connector.anchorRotation);
            const glm::mat4 socketLocal  = jointMatrix != nullptr ? (*jointMatrix) * anchorMatrix : anchorMatrix;
            PartInstance::Socket &socket = part.sockets[slot];
            socket.position              = glm::vec3(socketLocal[3]);
            socket.rotation              = glm::quat_cast(glm::mat3(socketLocal));
        }
    }
}

bool evaluateTransforms(const Registry &registry, CharacterRecord &record)
{
    (void)registry;

    // Root placement.
    const glm::mat4 rootMatrix = makeTrsMatrix(record.rootTranslation, record.rootRotation);

    // Explicit stack avoids a heap-allocated recursive std::function.
    bool                                           walkFailed = false;
    std::vector<std::pair<std::size_t, glm::mat4>> walkStack;
    const auto pushChildren = [&](std::size_t partIndex, const glm::mat4 &parentRigidWorld)
    {
        PartInstance &part  = record.parts[partIndex];
        part.rigidWorld     = parentRigidWorld;
        part.world          = parentRigidWorld * glm::scale(glm::mat4(1.0f), part.scale);
        part.transformValid = true;
        const glm::quat parentWorldRotation = glm::quat_cast(glm::mat3(parentRigidWorld));
        for (const std::size_t topologyIndex : record.childConnections[partIndex])
        {
            const ResolvedConnection &resolved = record.resolvedConnections[topologyIndex];
            if (resolved.parentConnectorSlot == unresolvedIndex || resolved.childConnectorSlot == unresolvedIndex
                || resolved.parentConnectorSlot >= part.sockets.size()
                || resolved.childConnectorSlot >= record.parts[resolved.childIndex].sockets.size())
            {
                walkFailed = true;
                return;
            }
            const PartInstance  &childPart = record.parts[resolved.childIndex];
            const PartInstance::Socket parentSocket = part.sockets[resolved.parentConnectorSlot];
            const PartInstance::Socket childSocket
                = record.parts[resolved.childIndex].sockets[resolved.childConnectorSlot];

            // T_child = T_socket(parent, scale-applied) * T_offset
            //           * inverse(T_socket(child, scale-applied)), with
            // every scale folded into positions so T_child stays rigid.
            const glm::vec3 parentSocketWorld(parentRigidWorld * glm::vec4(part.scale * parentSocket.position, 1.0f));
            const glm::quat parentSocketRotation = parentWorldRotation * parentSocket.rotation;
            const glm::quat childRotation
                = parentSocketRotation * record.connections[topologyIndex].offsetRotation
                  * glm::inverse(childSocket.rotation);
            const glm::vec3 childTranslation = parentSocketWorld
                + parentSocketRotation * record.connections[topologyIndex].offsetTranslation
                - childRotation * (childPart.scale * childSocket.position);
            walkStack.emplace_back(resolved.childIndex, makeTrsMatrix(childTranslation, childRotation));
        }
    };

    for (const std::uint32_t rootIndex : record.rootIndices)
    {
        walkStack.emplace_back(rootIndex, rootMatrix);
        while (!walkStack.empty() && !walkFailed)
        {
            const auto [partIndex, parentWorld] = walkStack.back();
            walkStack.pop_back();
            pushChildren(partIndex, parentWorld);
        }
        if (walkFailed)
        {
            break;
        }
    }
    return !walkFailed;
}

void updateBoundsAndStatistics(const Registry &registry, CharacterRecord &record)
{
    (void)registry;

    // Bounds: union of the transformed part bound spheres.
    glm::vec3 boundsCenter(0.0f);
    float     boundsRadius = 0.0f;
    bool      haveBounds   = false;
    for (const PartInstance &part : record.parts)
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
    record.boundsCenter = boundsCenter;
    record.boundsRadius = boundsRadius;
    record.boundsValid  = haveBounds;

    // Statistics.
    std::uint32_t jointCount = 0;
    std::uint32_t triangles  = 0;
    for (const PartInstance &part : record.parts)
    {
        if (part.part.rigIndex != noRigIndex && part.part.rigIndex < registry.rigs.size())
        {
            jointCount += static_cast<std::uint32_t>(registry.rigs[part.part.rigIndex].joints.size());
        }
        triangles += part.part.triangleCount;
    }
    record.statistics.partCount       = static_cast<std::uint32_t>(record.parts.size());
    record.statistics.connectionCount = static_cast<std::uint32_t>(record.connections.size());
    record.statistics.jointCount      = jointCount;
    record.statistics.triangleCount   = triangles;
}

} // namespace internal
} // namespace engine::character
