// Validation pass of the character module.
//
// One mandatory check suite over a live assembly, shared by evaluate() and
// validate(): instance names, connection references, root uniqueness,
// reachability, connector capacity, role compatibility (exact rule pair,
// then wildcard), relative-scale range, required tags, joint references,
// and parameter ranges. Issues are capped so a pathological assembly cannot
// grow the report without bound.

#ifndef ENGINE_CHARACTER_VALIDATION_HPP
#define ENGINE_CHARACTER_VALIDATION_HPP

#include "records.hpp"

#include <vector>

namespace engine::character
{
namespace internal
{

// Maximum issues retained per validation pass; further issues are dropped.
inline constexpr std::size_t kMaximumIssues = 256;

// Runs the full validation pass and appends every failed condition to
// issues (cleared first). Assumes the module mutex is held.
void collectIssues(const Registry &registry, const CharacterRecord &record, std::vector<ValidationIssue> &issues);

} // namespace internal
} // namespace engine::character

#endif
