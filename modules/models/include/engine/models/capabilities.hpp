// Public capability surface of the models module.
//
// Declares the ABI revision, handle/error/option types, and the function-pointer
// table the kernel hands to consumers that require the "models" capability.
// Function pointers take the capability's own `self` as their first argument.

#ifndef ENGINE_MODELS_CAPABILITIES_HPP
#define ENGINE_MODELS_CAPABILITIES_HPP

#include <engine/models/material.hpp>
#include <engine/renderer/mesh.hpp>

#include <cstdint>

namespace engine
{
namespace models
{

// ABI revision of ModelsCapability; consumers check this before calling entries.
inline constexpr std::uint32_t modelsCapabilityAbi = 7u;

// Opaque loaded-model id. 0 means "none" and is never handed out; handles stay
// valid until destroyPluginel() or module stop.
using ModelHandle                         = std::uint32_t;
inline constexpr ModelHandle invalidModel = 0;
// Per-triangle material index meaning "no material" (OBJ faces without usemtl,
// formats without materials).
inline constexpr std::uint32_t invalidMaterial = 0xffffffffu;

// Why a load failed. Loaders write this through the optional out-parameter and
// also return invalidModel; None is written on success.
enum class ModelLoadError : std::uint32_t
{
    None = 0,
    FileNotFound,   // path did not resolve through any configured search path
    ParseFailed,    // unreadable, malformed, unsupported feature/extension
    NoGeometry,     // parsed but produced no renderable triangles
    InvalidHandle,  // (reserved for capability callers; internal lookups return "empty" answers instead)
    InvalidOptions, // ModelLoadOptions failed validation (NaNs, out-of-range enums, ...)
    FileTooLarge,   // exceeds models.max_file_size_mb (or index-count overflow)
    TooManyModels   // models.max_loaded_models reached or handle space exhausted
};

// How vertex normals are produced. "Preserve" means normals from the source
// file (transformed by the load options); "generate" computes them geometrically.
enum class ModelNormalMode : std::uint32_t
{
    PreserveOrGenerateSmooth = 0, // keep file normals, generate smooth ones only where missing
    PreserveOnly,                 // never generate; missing normals stay a placeholder
    GenerateSmooth,               // ignore file normals, area-weighted smooth normals everywhere
    GenerateFlat                  // per-face normals; each triangle gets unshared vertices
};

// Importer selection. Auto picks one from the file extension; the other values
// force an importer regardless of extension.
enum class ModelFormat : std::uint32_t
{
    Auto = 0,
    Obj,
    Stl,
    Ply,
    Off,
    Gltf // both .gltf (JSON) and .glb (binary container)
};

// Triangle winding (front-face order) handling after import.
enum class ModelWindingMode : std::uint32_t
{
    Automatic = 0, // reverse when the scale makes handedness flip (odd number of negative axes)
    Preserve,      // never touch index order
    Reverse        // always swap the last two corners of every triangle
};

// Counters describing one loaded model, filled after optimization. Vertex and
// index counts are totals over all parts, in the post-transform geometry.
struct ModelStatistics
{
    std::uint64_t sourceFileSizeBytes = 0;
    std::uint64_t vertexCount         = 0;
    std::uint64_t indexCount          = 0;
    std::uint32_t partCount           = 0;
    std::uint32_t materialCount       = 0;
};

// Per-load controls. Defaults match config.toml; pass nullptr to loadModelWithOptions
// to use the configured module defaults. The implementation validates (and
// normalizes the rotation quaternion) before importing and reports
// ModelLoadError::InvalidOptions for unusable values.
struct ModelLoadOptions
{
    float scale[3]       = {1.0f, 1.0f, 1.0f};
    float translation[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4]    = {0.0f, 0.0f, 0.0f, 1.0f}; // unit quaternion, x y z w; renormalized before use
    float pivot[3]       = {0.0f, 0.0f, 0.0f};       // point positions are rotated/scaled around
    float uvScale[2]     = {1.0f, 1.0f};
    float uvOffset[2]    = {0.0f, 0.0f};

    ModelNormalMode  normalMode  = ModelNormalMode::PreserveOrGenerateSmooth;
    ModelFormat      format      = ModelFormat::Auto;
    ModelWindingMode windingMode = ModelWindingMode::Automatic;

    bool flipTextureV       = true;
    bool importVertexColors = false;
    bool weldVertices       = true;
    bool importMaterials    = true;

    const char *materialSearchPath = nullptr; // borrowed for the duration of the load (OBJ/MTL lookup)
    bool        optimizeMesh       = true;    // run the module's meshoptimizer passes after import
};

// The capability struct itself: consumed via ParticipantContext::acquireCapability<ModelsCapability>()
// and called as, e.g., capability.loadModel(capability.self, path, &error). Every
// entry is synchronous and must be called from the thread that owns the module.
struct ModelsCapability
{
    static constexpr const char *capabilityName = "models";

    std::uint32_t abi  = modelsCapabilityAbi; // consumer compares against modelsCapabilityAbi
    void         *self = nullptr;             // module instance, pass back as the first argument

    ModelHandle (*loadModel)(void *self, const char *path, ModelLoadError *error); // uses configured defaults
    void (*destroyPluginel)(void *self, ModelHandle model);
    std::uint32_t (*partCount)(void *self, ModelHandle model);
    // Fills a render-ready description of one part. The vertex/index pointers alias
    // module-owned buffers that stay valid until destroyPluginel(model) or module stop.
    bool (*part)(void *self, ModelHandle model, std::uint32_t partIndex, renderer::MeshDescription *outDescription);
    const char *(*partName)(void *self, ModelHandle model, std::uint32_t partIndex);     // nullptr when out of range
    bool (*bounds)(void *self, ModelHandle model, float *outMinimum, float *outMaximum); // either out may be null
    ModelHandle (*loadModelWithOptions)(
        void *self, const char *path, const ModelLoadOptions *options, ModelLoadError *error);

    // Materials are indexed 0..materialCount-1 in source order. Name and texture
    // path strings are owned by the module and live as long as the model handle.
    std::uint32_t (*materialCount)(void *self, ModelHandle model);
    bool (*material)(void *self, ModelHandle model, std::uint32_t materialIndex, ModelMaterial *outMaterial);
    const char *(*materialName)(void *self, ModelHandle model, std::uint32_t materialIndex);
    const char *(*materialTexturePath)(
        void *self, ModelHandle model, std::uint32_t materialIndex, ModelMaterialTextureSlot slot); // null when unset

    bool (*materialTextureOptions)(void *self, ModelHandle model, std::uint32_t materialIndex,
        ModelMaterialTextureSlot slot, ModelMaterialTextureOptions *outOptions);
    // Per-triangle material indices (invalidMaterial where unassigned). The array
    // aliases module storage and is ordered like the part's triangle list.
    bool (*partTriangleMaterials)(void *self, ModelHandle model, std::uint32_t partIndex,
        const std::uint32_t **outMaterialIndices, std::uint32_t *outTriangleCount);
    bool (*statistics)(void *self, ModelHandle model, ModelStatistics *outStatistics);
};

} // namespace models
} // namespace engine

#endif
