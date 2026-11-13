// Registry construction and lookup helpers of the character module.
//
// Copies validated definitions into the module-owned records and provides
// the lookups shared by registration, validation, evaluation, and the
// recipe serializer. Rule lookup is indexed: exact role pairs resolve
// through a hash map, the wildcard fallback scans only wildcard rules in
// registration order - matching the linear-scan precedence (exact pair
// first, then first wildcard match) without the O(rules) walk.

#ifndef ENGINE_CHARACTER_REGISTRY_HPP
#define ENGINE_CHARACTER_REGISTRY_HPP

#include "records.hpp"

#include <engine/character/capabilities.hpp>

#include <string>

namespace engine::character
{
namespace internal
{

// Registration sanity cap for one connector's attachment slots.
inline constexpr std::uint32_t kMaximumConnectionsPerConnector = 64;

// ---- registry lookups ------------------------------------------------------

// Position of id in registry.parameters, or npos-like sentinel.
std::uint32_t findParameterIndex(const Registry &registry, std::string_view id);

const PartRecord *findPartRecord(const Registry &registry, std::string_view id);

const ConnectorRecord *findConnector(const PartRecord &part, const std::string &id);

std::int32_t rigJointIndex(const RigRecord &rig, const std::string &semantic);

bool partHasTag(const PartRecord &part, const std::string &tag);

// Resolves the compatibility rule for a connection: the connector's explicit
// rule id when present, otherwise the (parentRole, childRole) role pair with
// exact-pair precedence over wildcards. Returns nullptr when nothing matches.
const CompatibilityRuleRecord *resolveCompatibilityRule(
    const Registry &registry, const ConnectorRecord &parentConnector, const ConnectorRecord &childConnector);

// ---- definition copying (registration-time validation) ---------------------

// Copies one connector definition into a stored record. Returns false with
// the error out-parameter set when the definition is malformed.
bool copyConnectorDefinition(const ConnectorDefinition &definition, ConnectorRecord &record, CharacterError *error);

// Resolves anchor and binding references of a stored part against the
// registry: joint-space anchors and JointTranslation bindings must name
// joints of the part's rig, ConnectorOffsetTranslation bindings must name
// connectors of the same part, and every binding parameter must be
// registered. Fills the cached indices used by evaluation and validation.
// Returns false with the error out-parameter set on an unresolved reference.
bool resolvePartReferences(const Registry &registry, PartRecord &part, CharacterError *error);

} // namespace internal
} // namespace engine::character

#endif
