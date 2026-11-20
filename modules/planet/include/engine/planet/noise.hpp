// OpenSimplex2 noise (2D and 3D), C++ port.
//
// Ported from K.jpg's OpenSimplex2 reference implementation (C# /
// Java, https://github.com/KdotJPG/OpenSimplex2), which is released under
// CC0 1.0 Universal. This port keeps the reference's lattice walks,
// constants and gradient tables so the output matches the canonical
// algorithm; only the API surface is engine-style.
//
// The implementation is stateless: lattice vertices are hashed directly
// through 64-bit integer primes mixed with the seed, so every call is
// thread-safe and bit-for-bit reproducible across call orders and threads.
// Only the 3D evaluator the planet core needs is ported (the 2D and 4D
// variants and the alternative orientations are omitted).

#pragma once

#include <cstdint>

namespace engine
{
namespace planet
{
namespace noise
{

// 3D OpenSimplex2 with better visual isotropy in the (X, Y) plane. Used for
// planet-space (non-terrain-shaped) fields where no axis is special.
// Range is roughly [-1, 1].
double simplex3ImproveXY(std::int64_t seed, double x, double y, double z);

// 3D Worley (cellular) noise, first-feature distance F1 normalized so one
// lattice cell spans [0, 1]: zero at a feature point, one at the farthest
// point of its cell. Stateless like the simplex port - feature points are
// hashed per lattice cell from the seed, so every call is thread-safe and
// bit-for-bit reproducible. Used for the billow shapes cloud banks need
// (inverted, it packs rounded cumulus lobes the way smooth fBm cannot).
double worley3(std::int64_t seed, double x, double y, double z);

} // namespace noise
} // namespace planet
} // namespace engine

