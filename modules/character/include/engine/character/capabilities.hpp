// Public capability surface of the character module.
//
// Data-driven character assembly for modular body parts: a content registry
// (body part assets, rig fragments, parameters, compatibility rules, assembly
// profiles), a validated attachment-tree assembly runtime with connector-
// aligned transforms, per-asset parameter bindings for deformation, and
// versioned TOML assembly recipes.
//
// The module owns no rendering, physics, or animation state. Consumers draw
// assembled parts through the renderer capability using the world matrices
// produced by evaluate(), and drive appendage animation by updating
// connection offsets. Skinning/retargeting integration with the
// skeletal_animation module and GPU morph targets are documented extension
// points: the renderer in this engine build has no skinning or morph path
// (the glTF importer rejects skinned meshes), so deformation is implemented
// as transform-level parameter bindings, exactly the subset this engine can
// render.

#ifndef ENGINE_CHARACTER_CAPABILITIES_HPP
#define ENGINE_CHARACTER_CAPABILITIES_HPP

#include <cstdint>

namespace engine::character
{

// Capability ABI. Bump when any struct or entry in CharacterCapability
// changes layout so consumers can reject mismatched providers.
inline constexpr std::uint32_t characterCapabilityAbi = 1u;

// Assembly (character instance) handle; 0 (invalidCharacter) means "none".
// Handles are never reused.
using CharacterHandle = std::uint32_t;
inline constexpr CharacterHandle invalidCharacter = 0;

// One body part instance inside an assembly; 0 (invalidInstance) means
// "none". Instance ids are per-assembly and stay stable across parameter and
// transform changes.
using InstanceId = std::uint32_t;
inline constexpr InstanceId invalidInstance = 0;

// Recipe format version written by saveRecipe; loadRecipe accepts this
// version and rejects anything newer.
inline constexpr std::uint32_t recipeFormatVersion = 1u;

// Error codes written through optional CharacterError out-parameters.
enum class CharacterError : std::uint32_t
{
    None = 0,
    CreationFailed,
    InvalidHandle,
    InvalidArgument,
    UnknownAsset,     // part, rig, profile, or parameter id is not registered
    UnknownConnector, // connector id missing on the referenced part
    ValidationFailed, // the requested operation violates an assembly rule
    CapacityExceeded, // module budget (characters, parts, joints...) exceeded
    IoFailed          // recipe could not be written or parsed
};

// Space a connector anchor is defined in. PartLocal anchors are fixed to the
// part's reference frame; Joint anchors follow a rig joint of the owning
// part, evaluated with the current (parameter-adjusted) joint pose.
enum class AnchorSpace : std::uint32_t
{
    PartLocal = 0,
    Joint
};

// Where a parameter binding applies. Transform-level targets only: this
// engine build has no mesh-space morph or skinning path, so deformations are
// expressed as mesh scale, joint rest-pose translation, and connector anchor
// adjustments.
enum class ParameterBindingTarget : std::uint32_t
{
    PartScaleX = 0,
    PartScaleY,
    PartScaleZ,
    JointTranslation,           // targetId = joint semantic id
    ConnectorOffsetTranslation  // targetId = connector id
};

// One anchor of a connector: position + orientation in the chosen space.
// translation is in world units, rotation is a unit quaternion (x, y, z, w).
// Strings are borrowed during registration; the module copies them.
struct ConnectorAnchorDefinition
{
    AnchorSpace space = AnchorSpace::PartLocal;

    // Joint-space anchors: semantic id of a joint in the owning part's rig.
    const char *jointSemantic = nullptr;

    float translation[3] = {0.0f, 0.0f, 0.0f};

    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};
};

// One connector of a body part: a named, role-typed attach point with a
// capacity and an optional explicit compatibility rule. Connection
// compatibility falls back to the (parentRole, childRole) rule pair when
// compatibilityRule is null or empty.
struct ConnectorDefinition
{
    const char *id   = nullptr;
    const char *role = nullptr;

    ConnectorAnchorDefinition anchor{};

    const char *compatibilityRule = nullptr;

    std::uint32_t maxConnections = 1;
};

// One material slot of a body part. Colors are linear RGB 0..1. The module
// carries slot definitions as asset metadata (validation + recipes); the
// consumer maps them to renderer materials.
struct MaterialSlotDefinition
{
    const char *id = nullptr;

    float baseColor[3] = {1.0f, 1.0f, 1.0f};

    float roughness = 0.8f;
    float metallic  = 0.0f;
};

// Maps one global parameter onto one deformation target of the owning part.
// Scale targets multiply: effect = 1 + (value - parameterDefault) * scale.
// Translation targets add:    effect = (value - parameterDefault) * offset.
struct ParameterBinding
{
    const char *parameter = nullptr;

    ParameterBindingTarget target = ParameterBindingTarget::PartScaleX;

    // Joint semantic or connector id for the joint/connector targets; ignored
    // (may be null) for part scale targets.
    const char *targetId = nullptr;

    float scale = 0.0f;

    float offset[3] = {0.0f, 0.0f, 0.0f};
};

// One registered body part asset. geometryId is an opaque consumer-side key
// (the module never loads geometry); bounds describe the part in its
// reference local space; triangleCount feeds statistics. Tags are free-form
// labels (for example "slot:head", "plan:biped") used by compatibility rules
// and by tools to group replaceable variants. All strings/arrays are
// borrowed during registration and copied by the module.
struct PartDefinition
{
    const char *id          = nullptr;
    const char *displayName = nullptr;

    const char *geometryId = nullptr;

    // "" (or null) for static parts without a rig fragment.
    const char *rigId = nullptr;

    const ConnectorDefinition   *connectors     = nullptr;
    std::uint32_t                connectorCount = 0;

    const ParameterBinding      *parameterBindings     = nullptr;
    std::uint32_t                parameterBindingCount = 0;

    const MaterialSlotDefinition *materialSlots     = nullptr;
    std::uint32_t                 materialSlotCount = 0;

    const char *const *tags     = nullptr;
    std::uint32_t       tagCount = 0;

    float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
    float boundsRadius    = 1.0f;

    std::uint32_t triangleCount = 0;
};

// One joint of a rig fragment. parentIndex < 0 marks the fragment root;
// otherwise it must reference an earlier joint (parent-before-child order).
// rotation is a unit quaternion (x, y, z, w); translation/scale are in the
// parent joint's frame.
struct RigJointDefinition
{
    const char *semantic = nullptr;

    std::int32_t parentIndex = -1;

    float translation[3] = {0.0f, 0.0f, 0.0f};

    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};

    float scale[3] = {1.0f, 1.0f, 1.0f};
};

// One rig fragment: an independent joint hierarchy with semantic joint ids.
// Fragments attach to characters through connector anchors (joint-space
// anchors reference these semantics) - skeletons are deliberately NOT merged
// between parts.
struct RigDefinition
{
    const char *id = nullptr;

    const RigJointDefinition *joints     = nullptr;
    std::uint32_t             jointCount = 0;
};

// One data-defined connector compatibility rule: whether a parent connector
// of parentRole accepts a child connector of childRole, subject to a
// relative-scale range and optional required tags on both parts. Either role
// may be "*" as a wildcard. Rules are looked up by exact role pair first,
// then by wildcard pair; unmatched pairs are incompatible.
struct CompatibilityRuleDefinition
{
    const char *id = nullptr;

    const char *parentRole = nullptr;
    const char *childRole  = nullptr;

    bool allowed = true;

    // Allowed ratio between the child part's world scale and the parent
    // part's world scale (per axis, max/min), guarding gross size mismatches.
    float minimumRelativeScale = 0.25f;
    float maximumRelativeScale = 4.0f;

    const char *requiredParentTag = nullptr;
    const char *requiredChildTag  = nullptr;
};

// One registered global parameter with its clamping range.
struct ParameterDefinition
{
    const char *id          = nullptr;
    const char *displayName = nullptr;

    float minimum  = 0.0f;
    float maximum  = 1.0f;
    float defaultV = 0.0f;
};

// One named part instance of a profile's default recipe.
struct ProfilePart
{
    const char *name   = nullptr; // instance name, unique within the profile
    const char *partId = nullptr; // registered body part asset id
};

// One connection of a profile's default recipe.
struct ProfileConnection
{
    const char *parentName      = nullptr;
    const char *parentConnector = nullptr;

    const char *childName      = nullptr;
    const char *childConnector = nullptr;

    // Authored correction applied between the two connector frames; the
    // attached part's transform is derived from the connectors, this offset
    // only refines the alignment.
    float offsetTranslation[3] = {0.0f, 0.0f, 0.0f};
    float offsetRotation[4]    = {0.0f, 0.0f, 0.0f, 1.0f};
};

// One assembly profile: a validated topology contract (a compatible set of
// anatomical components) that createAssembly instantiates as the default
// recipe. Parts may be added/removed/replaced afterwards.
struct AssemblyProfileDefinition
{
    const char *id          = nullptr;
    const char *displayName = nullptr;

    const ProfilePart      *parts     = nullptr;
    std::uint32_t           partCount = 0;

    const ProfileConnection *connections     = nullptr;
    std::uint32_t            connectionCount = 0;
};

// Validation codes, one per mandatory check. validate() reports every failed
// condition with the offending instance and a human-readable message.
enum class ValidationCode : std::uint32_t
{
    None = 0,
    UnknownAsset,           // part references an unregistered rig
    UnknownConnector,       // a connection names a missing connector
    DuplicateInstanceName,  // two instances share one name
    MultipleParents,        // non-root instance with two structural parents
    NoRoot,                 // assembly has no parentless instance
    MultipleRoots,          // assembly has more than one parentless instance
    Cycle,                  // attachment relationships form a cycle
    ConnectorCapacity,      // a connector exceeds maxConnections
    IncompatibleConnection, // no compatibility rule allows the role pair
    ScaleOutOfRange,        // relative part scales violate the matched rule
    RequiredTagMissing,     // a rule's anatomical tag condition failed
    InvalidJointReference,  // anchor or binding references a missing joint
    ParameterOutOfRange,    // stored parameter value outside its range
    InvalidTransform        // non-finite transform data (recipe or offset)
};

// One validation diagnostic. subject names the offending asset/instance
// (null-terminated); message is a human-readable explanation.
struct ValidationIssue
{
    ValidationCode code = ValidationCode::None;

    InstanceId instance = invalidInstance;

    char subject[96] = {};

    char message[192] = {};
};

// Aggregated character statistics, refreshed by evaluate().
struct CharacterStatistics
{
    std::uint32_t partCount       = 0;
    std::uint32_t connectionCount = 0;
    std::uint32_t jointCount      = 0;
    std::uint32_t triangleCount   = 0;
    std::uint32_t issueCount      = 0;
};

// What changed about an assembly, published as CharacterAssemblyChangedEvent
// so other systems can invalidate only what is affected.
enum class CharacterChangeKind : std::uint32_t
{
    StructureChanged = 0, // parts/connections added, removed, or replaced
    ParametersChanged,    // parameter values changed (deformation + anchors)
    OffsetsChanged,       // connection offsets changed (animation drivers)
    Evaluated             // transforms/bounds were recomputed
};

// Published on the engine event bus after any assembly mutation or
// evaluation. character is the affected assembly; kind classifies the change.
struct CharacterAssemblyChangedEvent
{
    CharacterHandle     character = invalidCharacter;
    CharacterChangeKind kind      = CharacterChangeKind::Evaluated;
};

// The "character" capability. Calls are expected on one thread (typically the
// main/simulation thread); a mutex additionally serializes capability and Lua
// bridge entry points.
struct CharacterCapability
{
    static constexpr const char *capabilityName = "character";

    std::uint32_t abi = characterCapabilityAbi;

    void *self = nullptr;

    // ---- content registry (Asset Management) -------------------------------
    bool (*registerParameter)(void *self, const ParameterDefinition *definition, CharacterError *error);
    bool (*registerCompatibilityRule)(void *self, const CompatibilityRuleDefinition *definition,
        CharacterError *error);
    bool (*registerRig)(void *self, const RigDefinition *definition, CharacterError *error);
    bool (*registerPart)(void *self, const PartDefinition *definition, CharacterError *error);
    bool (*registerProfile)(void *self, const AssemblyProfileDefinition *definition, CharacterError *error);

    // Removes a previously registered part; assemblies holding instances of
    // it keep their data (the part snapshot lives in the assembly).
    bool (*unregisterPart)(void *self, const char *partId);

    std::uint32_t (*parameterCount)(void *self);
    bool (*parameterInfo)(void *self, std::uint32_t index, ParameterDefinition *outDefinition);

    std::uint32_t (*partCount)(void *self);
    bool (*partId)(void *self, std::uint32_t index, const char **outPartId);

    // ---- assembly instances -------------------------------------------------
    CharacterHandle (*createAssembly)(void *self, const char *profileId, CharacterError *error);
    void (*destroyAssembly)(void *self, CharacterHandle character);

    InstanceId (*addPart)(void *self, CharacterHandle character, const char *partId, const char *instanceName,
        CharacterError *error);
    bool (*removePart)(void *self, CharacterHandle character, InstanceId instance);
    // Swaps the asset of an existing instance, keeping its name and
    // connections; validation reports missing connectors afterwards.
    bool (*replacePart)(void *self, CharacterHandle character, InstanceId instance, const char *newPartId,
        CharacterError *error);

    // Establishes the structural attachment parent -> child. Re-parents the
    // child when it already had a parent; rejects cycles immediately.
    bool (*connect)(void *self, CharacterHandle character, InstanceId parent, const char *parentConnector,
        InstanceId child, const char *childConnector, const float *offsetTranslation, const float *offsetRotation,
        CharacterError *error);
    bool (*disconnect)(void *self, CharacterHandle character, InstanceId child);

    // World placement of the assembly root (translation + quaternion).
    bool (*setAssemblyTransform)(void *self, CharacterHandle character, const float *translation,
        const float *rotation);

    // Updates the authored correction offset of child's incoming connection.
    // Per-frame callers (appendage animation drivers) go through this; it
    // marks only transforms dirty, never the structure.
    bool (*setConnectionOffset)(void *self, CharacterHandle character, InstanceId child,
        const float *offsetTranslation, const float *offsetRotation);

    // ---- parameters ---------------------------------------------------------
    // Sets a parameter, clamped to its registered range; marks deformation
    // and dependent anchors dirty.
    bool (*setParameter)(void *self, CharacterHandle character, const char *parameter, float value,
        CharacterError *error);
    bool (*getParameter)(void *self, CharacterHandle character, const char *parameter, float *outValue);

    // ---- evaluation, validation, queries ------------------------------------
    // Validates the assembly and recomputes rig poses, connector-aligned
    // transforms, and bounds. Returns false when validation failed; the issue
    // list from validate() explains why. Stale transforms are left untouched
    // on failure.
    bool (*evaluate)(void *self, CharacterHandle character, CharacterError *error);

    // Runs the full validation pass and writes up to capacity issues,
    // returning the total issue count (may exceed capacity).
    std::uint32_t (*validate)(void *self, CharacterHandle character, ValidationIssue *outIssues,
        std::uint32_t capacity);

    std::uint32_t (*instanceCount)(void *self, CharacterHandle character);
    InstanceId (*instanceId)(void *self, CharacterHandle character, std::uint32_t index);
    // Fills outName with the instance name and outPartId with its asset id
    // (borrowed pointers, valid until the assembly changes); parent receives
    // the structural parent or invalidInstance.
    bool (*instanceInfo)(void *self, CharacterHandle character, InstanceId instance, const char **outName,
        const char **outPartId, InstanceId *outParent);

    // Column-major 4x4 world matrix of an instance (rigid transform with the
    // part's parameter scale applied). Requires a successful evaluate().
    bool (*instanceTransform)(void *self, CharacterHandle character, InstanceId instance, float *outMatrix16);

    // Union of all transformed part bounds (center + radius in world space).
    bool (*bounds)(void *self, CharacterHandle character, float *outCenter3, float *outRadius);

    bool (*statistics)(void *self, CharacterHandle character, CharacterStatistics *outStatistics);

    // ---- serialization (versioned assembly recipes) -------------------------
    bool (*saveRecipe)(void *self, CharacterHandle character, const char *filePath, CharacterError *error);
    // Replaces the assembly's content (parts, connections, parameters) with
    // the recipe; the profile metadata is validated when present.
    bool (*loadRecipe)(void *self, CharacterHandle character, const char *filePath, CharacterError *error);

    // Last module-level error text (parse failures, I/O errors); borrowed,
    // valid until the next capability call. Empty when nothing failed.
    const char *(*lastErrorMessage)(void *self);
};

} // namespace engine::character

#endif
