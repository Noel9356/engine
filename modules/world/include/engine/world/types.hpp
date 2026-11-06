// Handle types shared by the world module's ecs and entity_world layers.
// EntityHandle packs slot + 1 in the low 32 bits and a generation counter in
// the high 32 bits, so stale handles are detected after entity reuse.

#ifndef ENGINE_WORLD_TYPES_HPP
#define ENGINE_WORLD_TYPES_HPP

#include <cstdint>

namespace engine::world
{

using WorldHandle                           = std::uint32_t;
using EntityHandle                          = std::uint64_t;
inline constexpr WorldHandle  invalidWorld  = 0;
inline constexpr EntityHandle invalidEntity = 0;

} // namespace engine::world

#endif
