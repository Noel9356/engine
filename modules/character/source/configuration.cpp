// Configuration reading - see configuration.hpp.

#include "configuration.hpp"

#include <cstdint>

namespace engine::character
{
namespace internal
{

void readBudgets(ParticipantContext &context, Budgets &budgets)
{
    budgets              = Budgets{};
    const auto readCount = [&](const char *key, std::size_t &target, std::size_t maximum)
    {
        if (const auto value = context.configurationIntegerInSection("character", key);
            value && *value > 0 && static_cast<std::uint64_t>(*value) <= maximum)
        {
            target = static_cast<std::size_t>(*value);
        }
    };
    readCount("maximum_characters", budgets.maximumCharacters, 65536);
    readCount("maximum_parts", budgets.maximumRegisteredParts, 65536);
    readCount("maximum_parts_per_character", budgets.maximumPartsPerCharacter, 65536);
    readCount("maximum_connections_per_character", budgets.maximumConnectionsPerCharacter, 262144);
    readCount("maximum_connectors_per_part", budgets.maximumConnectorsPerPart, 1024);
    readCount("maximum_bindings_per_part", budgets.maximumBindingsPerPart, 1024);
    readCount("maximum_material_slots_per_part", budgets.maximumMaterialSlotsPerPart, 256);
    readCount("maximum_tags_per_part", budgets.maximumTagsPerPart, 256);
    readCount("maximum_parameters", budgets.maximumParameters, 65536);
    readCount("maximum_rules", budgets.maximumRules, 65536);
    readCount("maximum_rigs", budgets.maximumRigs, 65536);
    readCount("maximum_joints_per_rig", budgets.maximumJointsPerRig, 65536);
    readCount("maximum_profiles", budgets.maximumProfiles, 65536);
    if (const auto validate = context.configurationBooleanInSection("character", "validate_on_evaluate"))
    {
        budgets.validateOnEvaluate = *validate;
    }
}

} // namespace internal
} // namespace engine::character
