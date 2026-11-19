// Shared integer-lattice arithmetic for the chunk grid.
//
// floorDiv is the floored division the conforming-boundary machinery in
// meshing.cpp and the octree's coarse-cell bookkeeping in octree.cpp both
// need: C++ integer division truncates toward zero, so a negative numerator
// needs one corrective step to floor. One definition keeps the two
// consumers bit-identical by construction.

#pragma once

#include <cstdint>

namespace engine
{
namespace planet
{
namespace detail
{

// Floored division: the quotient rounds toward negative infinity, with the
// sign rules handling every numerator/divisor combination.
inline std::int64_t floorDiv(std::int64_t value, std::int64_t divisor)
{
    std::int64_t quotient = value / divisor;
    if ((value % divisor) != 0 && ((value < 0) != (divisor < 0)))
    {
        --quotient;
    }
    return quotient;
}

} // namespace detail
} // namespace planet
} // namespace engine
