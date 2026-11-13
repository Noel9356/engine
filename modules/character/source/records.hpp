// Internal data model of the character module.
//
// Runtime snapshots of the registered content (parts, rigs, parameters,
// rules, profiles) plus the assembly records (instances, connections,
// evaluated transforms, cached evaluation stages). Registration copies
// user-provided definitions so borrowed strings never escape the
// registration call.
//
// Derived data is layered behind dirty flags so the per-frame path (assembly
// transforms, connection offsets) never repeats parameter or structure work:
//
//   structure   -> resolved topology (connections -> part/connector indices,
//                  root list, cached parents, adjacency)
//   parameters  -> per-part scale, rig poses, resolved socket frames
//   transforms  -> connector-aligned rigid walk, bounds, statistics
//
// The registry is append-only for parameters, rigs, and rules, so records
// cache stable indices (parameter/rig/joint/connector) instead of repeating
// hash lookups during evaluation.

#ifndef ENGINE_CHARACTER_RECORDS_HPP
#define ENGINE_CHARACTER_RECORDS_HPP

#include <engine/character/capabilities.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace engine::character
{
namespace internal
{

// Transparent hashing/equality so unordered maps keyed by std::string accept
// string_view and const char* lookups without temporary allocations.
struct StringHash
{
    using is_transparent = void;

    std::size_t operator()(std::string_view text) const noexcept
    {
        return std::hash<std::string_view>{}(text);
    }
};

struct StringEqual
{
    using is_transparent = void;

    bool operator()(std::string_view left, std::string_view right) const noexcept
    {
        return left == right;
    }
};

template<typename Value>
using StringMap = std::unordered_map<std::string, Value, StringHash, StringEqual>;

// One connector of a stored part.
struct ConnectorRecord
{
    std::string id;
    std::string role;

    AnchorSpace anchorSpace = AnchorSpace::PartLocal;
    std::string anchorJointSemantic;

    // Joint index of anchorJointSemantic inside the owning part's rig,
    // resolved at registration; -1 for part-local anchors. Rigs are
    // append-only, so the index stays valid for the module's lifetime.
    std::int32_t anchorJointIndex = -1;

    glm::vec3 anchorTranslation{0.0f};
    glm::quat anchorRotation{1.0f, 0.0f, 0.0f, 0.0f};

    std::string compatibilityRule;

    std::uint32_t maxConnections = 1;
};

// One parameter binding of a stored part. The parameter index and the
// joint/connector target index are resolved at registration so evaluation
// never hashes strings; targets are checked for existence at registration.
struct BindingRecord
{
    std::string parameter;

    // Position in Registry::parameters; parameters are append-only.
    std::uint32_t parameterIndex = 0;

    ParameterBindingTarget target = ParameterBindingTarget::PartScaleX;

    std::string targetId;

    // JointTranslation: joint index in the part's rig. ConnectorOffset
    // Translation: connector index on the part. Unused for scale targets.
    std::int32_t targetIndex = -1;

    float scale = 0.0f;

    glm::vec3 offset{0.0f};
};

// One material slot of a stored part.
struct MaterialSlotRecord
{
    std::string id;
    glm::vec3   baseColor{1.0f};
    float       roughness = 0.8f;
    float       metallic  = 0.0f;
};

// Sentinel for PartRecord::rigIndex ("static part without a rig fragment").
inline constexpr std::uint32_t noRigIndex = std::numeric_limits<std::uint32_t>::max();

// Stored body part asset (registration-time snapshot).
struct PartRecord
{
    std::string id;
    std::string displayName;
    std::string geometryId;
    std::string rigId; // empty = static part

    // Position in Registry::rigs; noRigIndex for static parts.
    std::uint32_t rigIndex = noRigIndex;

    std::vector<ConnectorRecord>    connectors;
    std::vector<BindingRecord>      bindings;
    std::vector<MaterialSlotRecord> materialSlots;
    std::vector<std::string>        tags;

    glm::vec3     boundsCenter{0.0f};
    float         boundsRadius  = 1.0f;
    std::uint32_t triangleCount = 0;
};

// Stored rig fragment: joints in parent-before-child order.
struct RigJointRecord
{
    std::string  semantic;
    std::int32_t parentIndex = -1;

    glm::vec3 translation{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

struct RigRecord
{
    std::string                 id;
    std::vector<RigJointRecord> joints;
};

// Stored compatibility rule.
struct CompatibilityRuleRecord
{
    std::string id;
    std::string parentRole;
    std::string childRole;
    bool        allowed              = true;
    float       minimumRelativeScale = 0.25f;
    float       maximumRelativeScale = 4.0f;
    std::string requiredParentTag;
    std::string requiredChildTag;
};

// Stored global parameter.
struct ParameterRecord
{
    std::string id;
    std::string displayName;
    float       minimum  = 0.0f;
    float       maximum  = 1.0f;
    float       defaultV = 0.0f;
};

// Stored assembly profile (topology contract).
struct ProfileRecord
{
    std::string id;
    std::string displayName;

    struct Part
    {
        std::string name;
        std::string partId;
    };
    struct Connection
    {
        std::string parentName;
        std::string parentConnector;
        std::string childName;
        std::string childConnector;
        glm::vec3   offsetTranslation{0.0f};
        glm::quat   offsetRotation{1.0f, 0.0f, 0.0f, 0.0f};
    };

    std::vector<Part>       parts;
    std::vector<Connection> connections;
};

// One body part instance of an assembly. The part snapshot is copied at
// add/replace time, so the assembly stays valid even if the asset is
// unregistered later.
struct PartInstance
{
    InstanceId  id = invalidInstance;
    std::string name;
    std::string partId;
    PartRecord  part;

    // Structural parent, cached when the topology is (re)built;
    // invalidInstance for the assembly root.
    InstanceId parent = invalidInstance;

    // Parameter scale (pre-multiplied bindings), applied to the mesh below
    // the connector frames.
    glm::vec3 scale{1.0f};

    // Cached parameter-dependent evaluation stages, refreshed only when the
    // structure or the parameters changed - never on the per-frame transform
    // path. rigPoses holds part-local joint world matrices (empty for static
    // parts); sockets mirrors part.connectors with the resolved anchor frame.
    struct Socket
    {
        glm::vec3 position{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    std::vector<glm::mat4> rigPoses;
    std::vector<Socket>    sockets;

    // Evaluated world transform: rigid alignment (T) composed with the
    // parameter scale (S). Column-major, matching the renderer.
    glm::mat4 rigidWorld{1.0f};
    glm::mat4 world{1.0f};
    bool      transformValid = false;
};

// One structural attachment. The child's transform is derived entirely from
// the connector relationship; offset is an explicit authored correction.
struct ConnectionRecord
{
    InstanceId  parent = invalidInstance;
    std::string parentConnector;

    InstanceId  child = invalidInstance;
    std::string childConnector;

    glm::vec3 offsetTranslation{0.0f};
    glm::quat offsetRotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Structure-resolved view of one connection: positions in the parts vector
// and connector indices inside the part snapshots. Rebuilt only when the
// structure changes; noRigIndex-style sentinels mark unresolvable entries so
// the walk can fail the assembly the same way the validation pass would.
inline constexpr std::uint32_t unresolvedIndex = std::numeric_limits<std::uint32_t>::max();

struct ResolvedConnection
{
    std::uint32_t parentIndex         = unresolvedIndex;
    std::uint32_t childIndex          = unresolvedIndex;
    std::uint32_t parentConnectorSlot = unresolvedIndex;
    std::uint32_t childConnectorSlot  = unresolvedIndex;
};

// One live character assembly.
struct CharacterRecord
{
    CharacterHandle id = invalidCharacter;

    std::string profileId;

    std::vector<PartInstance>     parts;
    std::vector<ConnectionRecord> connections;

    StringMap<InstanceId> instancesByName;

    // Instance id -> position in parts, maintained alongside instancesByName
    // so capability queries (the per-frame renderer path) avoid linear scans.
    std::unordered_map<InstanceId, std::uint32_t> instanceIndicesById;

    // Instance ids are per-assembly, monotonically increasing and never
    // reused, so external references stay stable across removals.
    InstanceId nextInstanceId = 1;

    // Assembly root placement (world).
    glm::vec3 rootTranslation{0.0f};
    glm::quat rootRotation{1.0f, 0.0f, 0.0f, 0.0f};

    // Parameter values for this assembly, aligned with Registry::parameters
    // (the registry is append-only; registerParameter appends the default to
    // every live assembly). Unregistered parameters are rejected at
    // setParameter time.
    std::vector<float> parameterValues;

    // Dirty flags per subsystem (independent invalidation). validationPending
    // tracks whether the validated surface (structure or parameters) changed
    // since the last validation pass; transformsDirty covers the per-frame
    // path (root placement, connection offsets); topologyValid marks the
    // structure-resolved cache current.
    bool structureDirty    = true;
    bool parametersDirty   = true;
    bool transformsDirty   = true;
    bool validationPending = true;
    bool topologyValid     = false;

    // Structure-resolved topology, rebuilt only after structure changes so
    // the transform walk is O(parts + connections) with no string or hash
    // work. resolvedConnections is parallel to connections; childConnections
    // lists topology positions per part.
    std::vector<ResolvedConnection>       resolvedConnections;
    std::vector<std::vector<std::size_t>> childConnections;
    std::vector<std::uint32_t>            rootIndices;

    // Last evaluation results.
    CharacterStatistics          statistics{};
    std::vector<ValidationIssue> lastIssues;
    glm::vec3                    boundsCenter{0.0f};
    float                        boundsRadius = 0.0f;
    bool                         boundsValid  = false;
};

// Registration-time content store shared by the module, the serializer, and
// the evaluation stages. Vectors own the records; the index maps give
// id -> vector position. Parameters, rigs, and rules are append-only, which
// makes the indices cached in records stable for the module's lifetime.
struct Registry
{
    std::vector<ParameterRecord> parameters;
    StringMap<std::uint32_t>     parameterIndices;

    std::vector<CompatibilityRuleRecord> rules;

    // id -> rules position; wildcardRules lists positions of rules with at
    // least one "*" role in registration order (the fallback scan order).
    StringMap<std::uint32_t>   ruleIdIndices;
    std::vector<std::uint32_t> wildcardRules;

    std::vector<RigRecord>   rigs;
    StringMap<std::uint32_t> rigIndices;

    std::vector<PartRecord>  parts;
    StringMap<std::uint32_t> partIndices;

    std::vector<ProfileRecord> profiles;
    StringMap<std::uint32_t>   profileIndices;

    // Indexes a rule after it was appended to rules (id map + wildcard list).
    void indexRule(std::uint32_t position);
};

// Registration budgets (from the [character] config section).
struct Budgets
{
    std::size_t maximumCharacters              = 64;
    std::size_t maximumRegisteredParts         = 256;
    std::size_t maximumPartsPerCharacter       = 256;
    std::size_t maximumConnectionsPerCharacter = 512;
    std::size_t maximumConnectorsPerPart       = 32;
    std::size_t maximumBindingsPerPart         = 64;
    std::size_t maximumMaterialSlotsPerPart    = 8;
    std::size_t maximumTagsPerPart             = 16;
    std::size_t maximumParameters              = 256;
    std::size_t maximumRules                   = 256;
    std::size_t maximumRigs                    = 128;
    std::size_t maximumJointsPerRig            = 256;
    std::size_t maximumProfiles                = 64;
    bool        validateOnEvaluate             = true;
};

// ---- serialization (versioned TOML assembly recipes) ----------------------

// Writes record as a recipe file (parameter ids resolve through the
// registry). Returns false with errorMessage set on I/O or serialization
// failure.
bool saveRecipeToFile(const Registry &registry, const CharacterRecord &record, const std::string &filePath,
    std::string &errorMessage);

// Replaces record's content (parts, connections, parameters, root placement)
// from the recipe file. Registry entries are resolved by id; unknown part,
// profile, or parameter ids fail the load, as do connections naming unknown
// instances, missing connectors, or disallowed role pairs, and content that
// exceeds the per-assembly budgets. The assembly is only modified when every
// check passed. Returns false with errorMessage set otherwise.
bool loadRecipeFromFile(const Registry &registry, const Budgets &budgets, CharacterRecord &record,
    const std::string &filePath, std::string &errorMessage);

} // namespace internal
} // namespace engine::character

#endif
