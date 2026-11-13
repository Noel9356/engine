// Evaluation stages of the character module.
//
// Connector-aligned transform evaluation over the attachment tree, split
// into independently invalidated stages so the per-frame animation path
// (assembly transform / connection offsets) repeats neither the parameter
// application nor the topology resolution:
//
//   prepareTopology   structure -> resolved connections, roots, adjacency
//   applyParameters   structure or parameters -> per-part scale, rig poses,
//                     resolved socket frames (positions carry the part
//                     scale, rotations stay rigid)
//   evaluateTransforms transforms -> rigid connector-aligned walk
//   updateBoundsAndStatistics  bounds union + aggregate statistics
//
// All functions assume the module mutex is held and that validation either
// passed or was disabled; unresolvable topology fails the walk instead of
// activating an invalid assembly.

#ifndef ENGINE_CHARACTER_EVALUATION_HPP
#define ENGINE_CHARACTER_EVALUATION_HPP

#include "records.hpp"

namespace engine::character
{
namespace internal
{

// Rebuilds the structure-resolved topology (connection -> part/connector
// indices, root list, per-part child adjacency, cached parents). Called when
// record.structureDirty is set.
void prepareTopology(const Registry &registry, CharacterRecord &record);

// Recomputes per-part parameter scale, rig poses, and resolved socket frames
// from the assembly's parameter values. Called when the structure or the
// parameters changed.
void applyParameters(const Registry &registry, CharacterRecord &record);

// Walks the attachment tree from the roots, composing rigid child frames
// from the cached socket frames and the connection offsets, and writes the
// per-instance world matrices. Returns false when the topology references a
// missing instance or connector (the walk is aborted; stale transforms stay
// untouched).
bool evaluateTransforms(const Registry &registry, CharacterRecord &record);

// Union of the transformed part bound spheres plus aggregate statistics.
void updateBoundsAndStatistics(const Registry &registry, CharacterRecord &record);

} // namespace internal
} // namespace engine::character

#endif
