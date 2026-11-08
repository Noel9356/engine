// Internal data model of the character module.
//
// Runtime snapshots of the registered content (parts, rigs, parameters,
// rules, profiles) plus the assembly records (instances, connections,
// evaluated transforms). Registration copies user-provided definitions so
// borrowed strings never escape the registration call.

#ifndef ENGINE_CHARACTER_MODULE_INTERNAL_HPP
#define ENGINE_CHARACTER_MODULE_INTERNAL_HPP

#include <engine/character/capabilities.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::character
{
namespace internal
{

// One connector of a stored part.
struct ConnectorRecord
{
    std::string id;
    std::string role;

    AnchorSpace anchorSpace = AnchorSpace::PartLocal;
    std::string anchorJointSemantic;
    glm::vec3   anchorTranslation{0.0f};
    glm::quat   anchorRotation{1.0f, 0.0f, 0.0f, 0.0f};

    std::string compatibilityRule;

    std::uint32_t maxConnections = 1;
};

// One parameter binding of a stored part.
struct BindingRecord
{
    std::string parameter;

    ParameterBindingTarget target = ParameterBindingTarget::PartScaleX;

    std::string targetId;

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

// Stored body part asset (registration-time snapshot).
struct PartRecord
{
    std::string id;
    std::string displayName;
    std::string geometryId;
    std::string rigId; // empty = static part

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

    // Parameter scale (pre-multiplied bindings), applied to the mesh below
    // the connector frames.
    glm::vec3 scale{1.0f};

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

// One live character assembly.
struct CharacterRecord
{
    CharacterHandle id = invalidCharacter;

    std::string profileId;

    std::vector<PartInstance>     parts;
    std::vector<ConnectionRecord> connections;

    std::unordered_map<std::string, InstanceId> instancesByName;

    // Assembly root placement (world).
    glm::vec3 rootTranslation{0.0f};
    glm::quat rootRotation{1.0f, 0.0f, 0.0f, 0.0f};

    // Parameter values for this assembly (unregistered parameters rejected at
    // setParameter time).
    std::unordered_map<std::string, float> parameterValues;

    // Dirty flags per subsystem (independent invalidation).
    bool structureDirty  = true;
    bool parametersDirty = true;
    bool transformsDirty = true;
    bool boundsDirty     = true;

    // Child connection lists per part (positions in `connections`), rebuilt
    // lazily after structure changes so the transform walk is O(V + E).
    std::vector<std::vector<std::size_t>> childConnections;
    bool                                  adjacencyValid = false;

    // Last evaluation results.
    CharacterStatistics          statistics{};
    std::vector<ValidationIssue> lastIssues;
    glm::vec3                    boundsCenter{0.0f};
    float                        boundsRadius = 0.0f;
    bool                         boundsValid  = false;
};

// Registration-time content store shared by the module and the recipe
// serializer. Vectors own the records; the index maps give id -> vector
// position.
struct Registry
{
    std::vector<ParameterRecord>                   parameters;
    std::unordered_map<std::string, std::uint32_t> parameterIndices;

    std::vector<CompatibilityRuleRecord> rules;

    std::vector<RigRecord>                         rigs;
    std::unordered_map<std::string, std::uint32_t> rigIndices;

    std::vector<PartRecord>                        parts;
    std::unordered_map<std::string, std::uint32_t> partIndices;

    std::vector<ProfileRecord>                     profiles;
    std::unordered_map<std::string, std::uint32_t> profileIndices;
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

// TRS matrix helper shared with the serializer.
glm::mat4 makeTrsMatrix(const glm::vec3 &translation, const glm::quat &rotation);

// Connector lookup by id; returns nullptr when absent.
const ConnectorRecord *findConnector(const PartRecord &part, const std::string &id);

// ---- serialization (versioned TOML assembly recipes) ----------------------

// Writes record as a recipe file. Returns false with errorMessage set on I/O
// or serialization failure.
bool saveRecipeToFile(
    const Registry &registry, const CharacterRecord &record, const std::string &filePath, std::string &errorMessage);

// Replaces record's content (parts, connections, parameters, root placement)
// from the recipe file. Registry entries are resolved by id; unknown part
// ids fail the load. Returns false with errorMessage set on parse or
// resolution failure.
bool loadRecipeFromFile(
    const Registry &registry, CharacterRecord &record, const std::string &filePath, std::string &errorMessage);

} // namespace internal
} // namespace engine::character

#endif
