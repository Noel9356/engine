#include "gltf_import.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace engine
{

namespace
{

std::array<float, 3> vec3FromArray(
    const JsonValue *value, const std::array<float, 3> &fallback
)
{
    if (value == nullptr || !value->isArray() || value->array.size() < 3)
    {
        return fallback;
    }
    return {
        static_cast<float>(value->array[0].numberOr(0.0)),
        static_cast<float>(value->array[1].numberOr(0.0)),
        static_cast<float>(value->array[2].numberOr(0.0)),
    };
}

std::array<float, 4> vec4FromArray(
    const JsonValue *value, const std::array<float, 4> &fallback
)
{
    if (value == nullptr || !value->isArray() || value->array.size() < 4)
    {
        return fallback;
    }
    return {
        static_cast<float>(value->array[0].numberOr(0.0)),
        static_cast<float>(value->array[1].numberOr(0.0)),
        static_cast<float>(value->array[2].numberOr(0.0)),
        static_cast<float>(value->array[3].numberOr(0.0)),
    };
}

// One node's resolved world transform plus its glTF index.
struct NodeRecord
{
    GltfMat4            worldTransform{};
    bool                hasMesh    = false;
    std::size_t         meshIndex  = 0;
    std::string         name;
    std::vector<std::size_t> children;
};

// Reads one node's LOCAL transform (matrix or TRS, exactly as the glTF 2.0
// spec prescribes: when matrix is present, TRS MUST be ignored).
GltfMat4 localTransformOf(
    const JsonValue &node
)
{
    const JsonValue *matrix = node.find("matrix");
    if (matrix != nullptr && matrix->isArray() && matrix->array.size() == 16)
    {
        GltfMat4 result{};
        for (std::size_t index = 0; index < 16; ++index)
        {
            result[index] = static_cast<float>(matrix->array[index].numberOr(0.0));
        }
        return result;
    }
    const std::array<float, 3> translation = vec3FromArray(node.find("translation"), {0.0f, 0.0f, 0.0f});
    // glTF rotation is (x, y, z, w) with w last.
    const std::array<float, 4> rotation =
        vec4FromArray(node.find("rotation"), {0.0f, 0.0f, 0.0f, 1.0f});
    const std::array<float, 3> scale = vec3FromArray(node.find("scale"), {1.0f, 1.0f, 1.0f});
    return gltfComposeTrs(translation, rotation, scale);
}

// Builds the node table with composed world transforms: one depth-first walk,
// each world transform computed exactly once from its parent's.
void collectNodes(
    const JsonValue               &nodesArray,
    std::size_t                    nodeIndex,
    const GltfMat4               &parentTransform,
    std::vector<NodeRecord>       &outNodes,
    std::vector<bool>             &visited,
    std::string                   &errorReport
)
{
    if (nodeIndex >= nodesArray.array.size())
    {
        errorReport = "node index " + std::to_string(nodeIndex) + " is out of range";
        return;
    }
    if (visited[nodeIndex])
    {
        // A cycle is a malformed file (the spec forbids it); cutting the walk
        // here keeps the import finite without a crash.
        return;
    }
    visited[nodeIndex] = true;

    const JsonValue &node = nodesArray.array[nodeIndex];
    NodeRecord       record;
    record.name       = node.find("name") != nullptr ? node.find("name")->stringOr("") : "";
    record.worldTransform = gltfMultiplyMatrix(parentTransform, localTransformOf(node));

    const JsonValue *mesh = node.find("mesh");
    if (mesh != nullptr && mesh->isNumber())
    {
        record.hasMesh   = true;
        record.meshIndex = static_cast<std::size_t>(mesh->number);
    }

    const JsonValue *children = node.find("children");
    if (children != nullptr && children->isArray())
    {
        for (const JsonValue &child : children->array)
        {
            if (child.isNumber())
            {
                record.children.push_back(static_cast<std::size_t>(child.number));
            }
        }
    }

    // Copy the child list before recursing: the record lives in a vector that
    // recursion may reallocate.
    const std::vector<std::size_t> childIndices = record.children;
    const bool                     hasMesh      = record.hasMesh;
    const std::size_t              meshIndex    = record.meshIndex;
    const std::string              name         = record.name;
    const GltfMat4                 world        = record.worldTransform;

    outNodes[nodeIndex]                = NodeRecord{};
    outNodes[nodeIndex].worldTransform = world;
    outNodes[nodeIndex].hasMesh        = hasMesh;
    outNodes[nodeIndex].meshIndex      = meshIndex;
    outNodes[nodeIndex].name           = name;
    outNodes[nodeIndex].children       = childIndices;

    for (std::size_t child : childIndices)
    {
        collectNodes(nodesArray, child, world, outNodes, visited, errorReport);
        if (!errorReport.empty())
        {
            return;
        }
    }
}

// Material factors of one glTF material index (or the default material).
struct ImportedMaterial
{
    std::array<float, 3> albedo    = {0.8f, 0.8f, 0.8f};
    float                roughness = 1.0f;
    float                metallic  = 0.0f;
    float                emissive  = 0.0f;
};

ImportedMaterial materialOf(
    const JsonValue *materialsArray, std::size_t materialIndex
)
{
    ImportedMaterial imported;
    if (materialsArray == nullptr || !materialsArray->isArray() || materialIndex >= materialsArray->array.size())
    {
        return imported;
    }
    const JsonValue &material = materialsArray->array[materialIndex];
    const JsonValue *pbr      = material.find("pbrMetallicRoughness");
    if (pbr != nullptr && pbr->isObject())
    {
        // baseColorFactor is RGBA; the alpha channel has no counterpart in
        // the engine's material model and is ignored.
        const std::array<float, 3> baseColor =
            vec3FromArray(pbr->find("baseColorFactor"), {1.0f, 1.0f, 1.0f});
        imported.albedo    = baseColor;
        imported.metallic  = static_cast<float>(pbr->find("metallicFactor")->numberOr(1.0));
        imported.roughness = static_cast<float>(pbr->find("roughnessFactor")->numberOr(1.0));
    }
    const std::array<float, 3> emissiveFactor =
        vec3FromArray(material.find("emissiveFactor"), {0.0f, 0.0f, 0.0f});
    // The engine's emissive is scalar radiance (albedo * strength); the glTF
    // factor is RGB. The strongest channel drives the strength - an honest
    // approximation for the emissive surfaces content authors model.
    imported.emissive =
        std::max(emissiveFactor[0], std::max(emissiveFactor[1], emissiveFactor[2]));
    return imported;
}

// Grows the local-space AABB of one primitive from its POSITION accessor's
// min/max (both are JSON fields - the importer never decodes buffer views).
bool accessorBounds(
    const JsonValue &accessor, std::array<float, 3> &outMin, std::array<float, 3> &outMax
)
{
    const JsonValue *min = accessor.find("min");
    const JsonValue *max = accessor.find("max");
    if (min == nullptr || max == nullptr || !min->isArray() || !max->isArray() || min->array.size() < 3
        || max->array.size() < 3)
    {
        return false;
    }
    outMin = {
        static_cast<float>(min->array[0].numberOr(0.0)),
        static_cast<float>(min->array[1].numberOr(0.0)),
        static_cast<float>(min->array[2].numberOr(0.0)),
    };
    outMax = {
        static_cast<float>(max->array[0].numberOr(0.0)),
        static_cast<float>(max->array[1].numberOr(0.0)),
        static_cast<float>(max->array[2].numberOr(0.0)),
    };
    return true;
}

} // namespace

GltfMat4 gltfMultiplyMatrix(
    const GltfMat4 &a, const GltfMat4 &b
)
{
    // Column-major: result[column * 4 + row].
    GltfMat4 result{};
    for (std::size_t column = 0; column < 4; ++column)
    {
        for (std::size_t row = 0; row < 4; ++row)
        {
            float sum = 0.0f;
            for (std::size_t k = 0; k < 4; ++k)
            {
                sum += a[k * 4 + row] * b[column * 4 + k];
            }
            result[column * 4 + row] = sum;
        }
    }
    return result;
}

GltfMat4 gltfComposeTrs(
    const std::array<float, 3> &translation,
    const std::array<float, 4> &rotationQuaternion,
    const std::array<float, 3> &scale
)
{
    // Unit quaternion to 3x3 rotation (row-major within the 3x3 block).
    const float x = rotationQuaternion[0];
    const float y = rotationQuaternion[1];
    const float z = rotationQuaternion[2];
    const float w = rotationQuaternion[3];
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z;
    const float wx = w * x, wy = w * y, wz = w * z;

    GltfMat4 result{};
    result[0]  = (1.0f - 2.0f * (yy + zz)) * scale[0]; // column 0, row 0
    result[1]  = (2.0f * (xy + wz)) * scale[0];        // column 0, row 1
    result[2]  = (2.0f * (xz - wy)) * scale[0];        // column 0, row 2
    result[4]  = (2.0f * (xy - wz)) * scale[1];        // column 1, row 0
    result[5]  = (1.0f - 2.0f * (xx + zz)) * scale[1];
    result[6]  = (2.0f * (yz + wx)) * scale[1];
    result[8]  = (2.0f * (xz + wy)) * scale[2];
    result[9]  = (2.0f * (yz - wx)) * scale[2];
    result[10] = (1.0f - 2.0f * (xx + yy)) * scale[2];
    result[12] = translation[0];
    result[13] = translation[1];
    result[14] = translation[2];
    result[15] = 1.0f;
    return result;
}

std::array<float, 3> gltfTransformPoint(
    const GltfMat4 &matrix, const std::array<float, 3> &point
)
{
    return {
        matrix[0] * point[0] + matrix[4] * point[1] + matrix[8] * point[2] + matrix[12],
        matrix[1] * point[0] + matrix[5] * point[1] + matrix[9] * point[2] + matrix[13],
        matrix[2] * point[0] + matrix[6] * point[1] + matrix[10] * point[2] + matrix[14],
    };
}

std::uint32_t gltfImportIntoScene(
    const JsonValue &document,
    SceneCapability &scene,
    const std::string &sourceName,
    std::string &errorReport
)
{
    // The three top-level arrays the importer consumes.
    const JsonValue *nodesArray     = document.find("nodes");
    const JsonValue *meshesArray    = document.find("meshes");
    const JsonValue *materialsArray = document.find("materials");
    if (nodesArray == nullptr || !nodesArray->isArray())
    {
        errorReport = sourceName + ": the document has no nodes array - nothing to import";
        return 0;
    }
    if (meshesArray == nullptr || !meshesArray->isArray())
    {
        errorReport = sourceName + ": the document has no meshes array - nothing to import";
        return 0;
    }

    // The root nodes: the default scene, or scene 0, or every parentless node
    // (the spec's fallback for files without a scenes array).
    std::vector<std::size_t> rootIndices;
    const JsonValue         *scenesArray = document.find("scenes");
    if (scenesArray != nullptr && scenesArray->isArray() && !scenesArray->array.empty())
    {
        const double sceneNumber = document.find("scene") != nullptr && document.find("scene")->isNumber()
                                       ? document.find("scene")->number
                                       : 0.0;
        const std::size_t sceneIndex =
            sceneNumber >= 0.0 && sceneNumber < static_cast<double>(scenesArray->array.size())
                ? static_cast<std::size_t>(sceneNumber)
                : 0;
        const JsonValue *scene = &scenesArray->array[sceneIndex];
        const JsonValue *roots = scene->find("nodes");
        if (roots != nullptr && roots->isArray())
        {
            for (const JsonValue &root : roots->array)
            {
                if (root.isNumber())
                {
                    rootIndices.push_back(static_cast<std::size_t>(root.number));
                }
            }
        }
    }
    if (rootIndices.empty())
    {
        // Fallback: every node no other node lists as a child.
        std::vector<bool> isChild(nodesArray->array.size(), false);
        for (const JsonValue &node : nodesArray->array)
        {
            const JsonValue *children = node.find("children");
            if (children == nullptr || !children->isArray())
            {
                continue;
            }
            for (const JsonValue &child : children->array)
            {
                if (child.isNumber() && child.number >= 0.0
                    && child.number < static_cast<double>(isChild.size()))
                {
                    isChild[static_cast<std::size_t>(child.number)] = true;
                }
            }
        }
        for (std::size_t index = 0; index < isChild.size(); ++index)
        {
            if (!isChild[index])
            {
                rootIndices.push_back(index);
            }
        }
    }
    if (rootIndices.empty())
    {
        errorReport = sourceName + ": the document defines no root nodes";
        return 0;
    }

    // One world-transform walk over the whole hierarchy.
    std::vector<NodeRecord> nodes(nodesArray->array.size());
    std::vector<bool>       visited(nodesArray->array.size(), false);
    GltfMat4                identity{};
    identity[0] = identity[5] = identity[10] = identity[15] = 1.0f;
    for (std::size_t root : rootIndices)
    {
        collectNodes(*nodesArray, root, identity, nodes, visited, errorReport);
        if (!errorReport.empty())
        {
            errorReport = sourceName + ": " + errorReport;
            return 0;
        }
    }

    // Spawn one bounding-box object per mesh primitive.
    std::uint32_t spawned = 0;
    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex)
    {
        const NodeRecord &node = nodes[nodeIndex];
        if (!node.hasMesh || node.meshIndex >= meshesArray->array.size())
        {
            continue;
        }
        const JsonValue &mesh = meshesArray->array[node.meshIndex];
        const JsonValue *primitives = mesh.find("primitives");
        if (primitives == nullptr || !primitives->isArray())
        {
            continue;
        }
        for (const JsonValue &primitive : primitives->array)
        {
            const JsonValue *attributes = primitive.find("attributes");
            if (attributes == nullptr || !attributes->isObject())
            {
                continue;
            }
            const JsonValue *positionAccessor = attributes->find("POSITION");
            if (positionAccessor == nullptr || !positionAccessor->isNumber())
            {
                continue; // a primitive without positions draws nothing anywhere
            }
            const JsonValue *accessorsArray = document.find("accessors");
            if (accessorsArray == nullptr || !accessorsArray->isArray()
                || positionAccessor->number < 0.0
                || positionAccessor->number >= static_cast<double>(accessorsArray->array.size()))
            {
                continue;
            }
            std::array<float, 3> localMin{};
            std::array<float, 3> localMax{};
            if (!accessorBounds(accessorsArray->array[static_cast<std::size_t>(positionAccessor->number)], localMin, localMax))
            {
                continue; // no min/max: the accessor was written without bounds
            }

            // All 8 corners through the world transform, then the world AABB
            // (a rotated box's bounds are the corners' bounds).
            std::array<float, 3> worldMin{std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::max()};
            std::array<float, 3> worldMax{-std::numeric_limits<float>::max(),
                                          -std::numeric_limits<float>::max(),
                                          -std::numeric_limits<float>::max()};
            for (int corner = 0; corner < 8; ++corner)
            {
                const std::array<float, 3> local{corner & 1 ? localMax[0] : localMin[0],
                                                 corner & 2 ? localMax[1] : localMin[1],
                                                 corner & 4 ? localMax[2] : localMin[2]};
                const std::array<float, 3> world = gltfTransformPoint(node.worldTransform, local);
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    worldMin[axis] = std::min(worldMin[axis], world[axis]);
                    worldMax[axis] = std::max(worldMax[axis], world[axis]);
                }
            }

            SceneObjectDescriptor object;
            object.type    = scene_object_type::box;
            object.size    = {std::max((worldMax[0] - worldMin[0]) * 0.5f, 0.001f),
                              std::max((worldMax[1] - worldMin[1]) * 0.5f, 0.001f),
                              std::max((worldMax[2] - worldMin[2]) * 0.5f, 0.001f)};
            object.position = {(worldMin[0] + worldMax[0]) * 0.5f,
                               (worldMin[1] + worldMax[1]) * 0.5f,
                               (worldMin[2] + worldMax[2]) * 0.5f};

            const JsonValue *materialIndex = primitive.find("material");
            if (materialIndex != nullptr && materialIndex->isNumber())
            {
                const ImportedMaterial material = materialOf(materialsArray, static_cast<std::size_t>(materialIndex->number));
                object.albedo           = material.albedo;
                object.roughness        = material.roughness;
                object.metallic         = material.metallic;
                object.emissiveStrength = material.emissive;
            }

            if (scene.addObject(object) == SceneCapability::invalidObject)
            {
                // Registry full: report what arrived. The caller decides
                // whether that is acceptable for this asset.
                return spawned;
            }
            ++spawned;
        }
    }
    return spawned;
}

} // namespace engine
