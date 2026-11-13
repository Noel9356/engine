// Configuration reading of the character module.
//
// Reads the [character] section into the registration budgets. Values outside
// their documented range are ignored so the built-in default stays active;
// nothing is clamped. Called once per construct() and again whenever a
// ConfigurationReloadedEvent touches character.* keys.

#ifndef ENGINE_CHARACTER_CONFIGURATION_HPP
#define ENGINE_CHARACTER_CONFIGURATION_HPP

#include "records.hpp"

#include <engine/kernel/participant_interface.hpp>

namespace engine::character
{
namespace internal
{

// Replaces budgets with the built-in defaults, then applies every valid
// [character] key found in the participant configuration.
void readBudgets(ParticipantContext &context, Budgets &budgets);

} // namespace internal
} // namespace engine::character

#endif
