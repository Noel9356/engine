#ifndef ENGINE_MODULES_ASSET_GLTF_IMPORT_HPP
#define ENGINE_MODULES_ASSET_GLTF_IMPORT_HPP

// The glTF 2.0 importer: pure logic over a parsed JSON document and a scene
// capability. No Vulkan, no window, no filesystem access of its own (the
// caller reads the file and hands over the text) - which is what makes the
// importer unit-testable with a synthetic document and the real scene module.

#include "asset_capability.hpp"
#include "gltf_json.hpp"
#include "scene_capability.hpp"

#include <array>
#include <string>

namespace engine
{

// The matrix stack: glTF node transforms arrive as either a 4x4 column-major
// matrix or a translation/rotation(quaternion)/scale triple. One 4x4 type and
// the two compose operations - kept here because the renderer's scene_math
// serves the rasterizer's needs, not the importer's hierarchy walk, and the
// importer must not depend on renderer internals.
using GltfMat4 = std::array<float, 16>;

// Multiplies two column-major 4x4 matrices (result = a * b).
GltfMat4 gltfMultiplyMatrix(
    const GltfMat4 &a, const GltfMat4 &b
);

// Composes translation * rotation * scale from a TRS triple.
GltfMat4 gltfComposeTrs(
    const std::array<float, 3>    &translation,
    const std::array<float, 4>    &rotationQuaternion,
    const std::array<float, 3>    &scale
);

// Transforms one point by a column-major 4x4 (the w row is assumed 0 0 0 1).
std::array<float, 3> gltfTransformPoint(
    const GltfMat4 &matrix, const std::array<float, 3> &point
);

// Imports one parsed glTF document into the scene registry. Returns the
// number of spawned objects; fills errorReport on structural failure (the
// count is then 0). World transforms are composed down the node hierarchy
// exactly once per node, so deep hierarchies cost one matrix multiply per
// level, not per primitive.
std::uint32_t gltfImportIntoScene(
    const JsonValue     &document,
    SceneCapability     &scene,
    const std::string   &sourceName,
    std::string         &errorReport
);

} // namespace engine

#endif // ENGINE_MODULES_ASSET_GLTF_IMPORT_HPP
