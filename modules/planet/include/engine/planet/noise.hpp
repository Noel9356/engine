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
// Only the 2D and 3D evaluators the planet core needs are ported (the 4D
// variants and the alternative orientations are omitted).

#ifndef ENGINE_PLANET_NOISE_HPP
#define ENGINE_PLANET_NOISE_HPP

#include <cstdint>

namespace engine
{
namespace planet
{
namespace noise
{

// 2D OpenSimplex2, standard lattice orientation. Range is roughly [-1, 1].
double simplex2(std::int64_t seed, double x, double y);

// 3D OpenSimplex2 with better visual isotropy in the (X, Z) plane, which is
// the recommended orientation for terrain: the Y axis is the "different"
// coordinate (vertical in engine world space). Range is roughly [-1, 1].
double simplex3ImproveXZ(std::int64_t seed, double x, double y, double z);

// 3D OpenSimplex2 with better visual isotropy in the (X, Y) plane. Used for
// planet-space (non-terrain-shaped) fields where no axis is special.
double simplex3ImproveXY(std::int64_t seed, double x, double y, double z);

} // namespace noise
} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_NOISE_HPP
