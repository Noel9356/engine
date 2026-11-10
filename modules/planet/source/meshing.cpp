// Chunk meshing implementation (see meshing.hpp).
//
// Marching-cubes tables: the classic 256-entry edge and triangle tables by
// Paul Bourke (public domain, http://paulbourke.net/geometry/polygonise/),
// with the standard corner numbering (corner i at x = i&1, y = (i>>1)&1,
// z = (i>>2)&1).

#include <engine/planet/meshing.hpp>

#include <engine/planet/noise.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace engine
{
namespace planet
{
namespace
{

// Orientation of marching-cubes triangles relative to the density gradient.
// The Bourke tables produce triangles that face the NEGATIVE gradient (air)
// for d > 0 = inside with the corner ordering above; the demo's --self-test
// verifies this constant against the geometry and fails loudly on mismatch.
constexpr bool kTerrainFlipWinding = true;

constexpr std::uint32_t kEdgeTable[256] = {
    0x0,   0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99,  0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33,  0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa,  0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66,  0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff,  0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55,  0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,  0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0xcc,  0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x55,  0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0xff,  0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x66,  0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa,  0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33,  0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99,  0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0,
};

constexpr std::int8_t kTriTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};

// The 12 cube edges as (cornerA, cornerB), matching the edge bits of the
// Bourke edge table.
// The 12 cube edges as (cornerA, cornerB), Bourke ring numbering:
// corners 0..3 ring the z=0 face, 4..7 ring the z=1 face, edges 8..11 are
// the verticals. Bit e of the edge table refers to edge e here.
constexpr std::uint32_t kEdgeCorners[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom ring (z = 0)
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top ring (z = 1)
    {0, 4}, {1, 5}, {2, 6}, {3, 7}, // verticals
};

// Bourke corner numbering: 0..3 ring the low face counterclockwise, 4..7 the
// high face. Bit c of a cell's cube index refers to this corner.
inline void cornerOffset(std::uint32_t corner, std::uint32_t &dx, std::uint32_t &dy, std::uint32_t &dz)
{
    switch (corner)
    {
    case 0: dx = 0; dy = 0; dz = 0; break;
    case 1: dx = 1; dy = 0; dz = 0; break;
    case 2: dx = 1; dy = 1; dz = 0; break;
    case 3: dx = 0; dy = 1; dz = 0; break;
    case 4: dx = 0; dy = 0; dz = 1; break;
    case 5: dx = 1; dy = 0; dz = 1; break;
    case 6: dx = 1; dy = 1; dz = 1; break;
    default: dx = 0; dy = 1; dz = 1; break; // 7
    }
}

// Dedup key for an edge between two padded-grid points a step apart on
// `axis` (local to one chunk's build).
inline std::uint64_t edgeKey(std::uint32_t ax, std::uint32_t ay, std::uint32_t az, std::uint32_t axis)
{
    return (static_cast<std::uint64_t>(axis) << 60) | (static_cast<std::uint64_t>(ax) << 40)
         | (static_cast<std::uint64_t>(ay) << 20) | static_cast<std::uint64_t>(az);
}

// Canonical endpoints of a marching-cubes edge: lexicographic minimum first.
// Every cell that shares the edge (in this chunk or the neighbor's) sees the
// same (a, b) pair, so interpolated positions and dedup keys agree.
inline void canonicalEdge(std::uint32_t pax, std::uint32_t pay, std::uint32_t paz, std::uint32_t pbx, std::uint32_t pby,
    std::uint32_t pbz, std::uint32_t &ax, std::uint32_t &ay, std::uint32_t &az, std::uint32_t &bx, std::uint32_t &by,
    std::uint32_t &bz, std::uint32_t &axis)
{
    ax = pax; ay = pay; az = paz;
    bx = pbx; by = pby; bz = pbz;
    if (pbx < pax || (pbx == pax && pby < pay) || (pbx == pax && pby == pay && pbz < paz))
    {
        ax = pbx; ay = pby; az = pbz;
        bx = pax; by = pay; bz = paz;
    }
    axis = ax != bx ? 0u : (ay != by ? 1u : 2u);
}

} // namespace

Vec3 chunkOriginMeters(const PlanetGenerationParams &params, const ChunkCoord &coord)
{
    // Deterministic node corner: root minimum + global lattice index * step,
    // matching the sampling formula in buildChunkMesh.
    const double step = chunkStepMeters(params, coord.depth);
    const double half = params.derivedRootHalfMeters;
    return Vec3{-half + step * static_cast<double>(static_cast<std::int64_t>(coord.x) * static_cast<std::int64_t>(params.leafGridN)),
        -half + step * static_cast<double>(static_cast<std::int64_t>(coord.y) * static_cast<std::int64_t>(params.leafGridN)),
        -half + step * static_cast<double>(static_cast<std::int64_t>(coord.z) * static_cast<std::int64_t>(params.leafGridN))};
}

ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord,
    const ConformMask &conformMask)
{
    ChunkMesh mesh;

    const std::uint32_t n        = params.leafGridN;
    const std::uint32_t paddedN  = n + 3; // 1-voxel apron on every side
    const std::uint32_t strideY  = paddedN;
    const std::uint32_t strideZ  = paddedN * paddedN;
    const std::size_t   gridSize = static_cast<std::size_t>(paddedN) * paddedN * paddedN;
    const double        nodeSize = chunkSizeMeters(params, coord.depth);
    const double        step     = chunkStepMeters(params, coord.depth);
    const double        half     = params.derivedRootHalfMeters;

    // Every sample position is derived from ONE global lattice index along
    // each axis: p = -half + step * (coord * n + innerIndex). Neighbor chunks
    // (same depth or across depths) evaluate the exact same expression with
    // the exact same integers for shared points, so their boundary geometry
    // matches bit-for-bit - no floating point drift, no cracks.
    const auto axisCoordinate = [&](std::uint32_t axisIndex, std::int64_t innerIndex)
    {
        const std::int64_t globalIndex = static_cast<std::int64_t>(static_cast<std::size_t>(axisIndex) * n) + innerIndex;
        return half * -1.0 + step * static_cast<double>(globalIndex);
    };

    // Chunk-local origin for the float vertices: the node center. Consumers
    // place chunks with a model matrix; floats stay small at any planet radius.
    const Vec3 center{axisCoordinate(coord.x, n / 2), axisCoordinate(coord.y, n / 2), axisCoordinate(coord.z, n / 2)};

    // ---- conforming boundary sampling (LOD seam elimination) --------------------------
    //
    // Wherever the material beyond a boundary-plane segment is a rendered leaf
    // one level coarser (flagged per coarse cell in conformMask), the fine
    // chunk's boundary-plane lattice points sample the COARSE LATTICE's
    // trilinear interpolant restricted to the plane (a bilinear over the (up
    // to) four surrounding on-plane coarse samples) instead of the raw field.
    // The plane sits on a coarse lattice plane, and at coarse lattice points
    // the bilinear degenerates to the coarse sample itself - so the fine
    // isoline and the coarse neighbor's chords share their zero crossings on
    // every coarse grid line, and the two surfaces meet without cracks at
    // those crossings.
    //
    // The residual between the crossings (the coarse chord sagitta) is covered
    // by the EXTENSION: the chunk also marches the apron cells just beyond
    // every conforming face, carrying the true surface INTO the coarse
    // neighbor's territory where the neighbor's own surface overlaps it - the
    // two interpenetrate, so no viewing ray can slip between them (see the
    // ConformMask comment in meshing.hpp).
    //
    // Consistency rules (they make the value a global function of the point):
    //  * the far-side coarse cell is derived from GLOBAL indices (floor of the
    //    tangential position, diagonal ring included), so every chunk touching
    //    the point looks up the same cell and decides identically;
    //  * when several planes touch a point (chunk edges/corners) the lowest
    //    conforming AXIS wins, and the bilinear arithmetic only depends on
    //    that axis - identical for every chunk around the point;
    //  * points at even tangential GLOBAL indices are coarse lattice points -
    //    chunk corners (coord * n) are always such points, which pins every
    //    corner to the same value from both sides (with per-depth band
    //    limiting the value is the coarse side's band-limited sample, see
    //    below).
    const double coarseStep  = coord.depth > 0 ? chunkStepMeters(params, coord.depth - 1) : 0.0;
    const bool   conforming  = coord.depth > 0 && conformMask.any();

    // Per-depth band limiting: a marching-cubes lattice resolves at most ~2
    // samples per wavelength, so the field evaluated FOR this chunk must
    // carry no content finer than its own voxel - content below
    // voxel * lodBandLimitFactor fades smoothly to zero (full strength at
    // twice that wavelength). Without this, coarse LOD chunks sample the
    // sub-voxel detail octaves as uncorrelated random values and marching
    // cubes reconstructs them as random spikes - the "spiky globe".
    // The cutoff follows the lattice, not the chunk: any lattice point at
    // depth d is ALWAYS evaluated with cutoff(d), so shared points agree
    // bit-for-bit no matter which chunk samples them. The coarse side of a
    // depth transition uses cutoff(d-1) = 2 * cutoff(d) (its voxel is twice
    // ours), and the conforming machinery copies exactly those values onto
    // the shared plane - including at coarse lattice points, which is why
    // the even-even skip below moved behind the mask check.
    const double ownCutoff    = step * params.lodBandLimitFactor;
    const double coarseCutoff = coord.depth > 0 ? coarseStep * params.lodBandLimitFactor : 0.0;

    const auto setAxisComponent = [](Vec3 &vector, std::uint32_t axis, double value)
    {
        if (axis == 0)
        {
            vector.x = value;
        }
        else if (axis == 1)
        {
            vector.y = value;
        }
        else
        {
            vector.z = value;
        }
    };

    const auto floorDiv = [](std::int64_t value, std::int64_t divisor)
    {
        std::int64_t quotient = value / divisor;
        if ((value % divisor) != 0 && ((value < 0) != (divisor < 0)))
        {
            --quotient;
        }
        return quotient;
    };

    // Bilinear evaluation of the coarse lattice interpolant on one boundary
    // plane. (axis, maxSide) identify the plane; perpGlobalFine is the point's
    // global fine index along that axis (always a multiple of n, hence a
    // coarse lattice plane); tangentialGlobalU/V are the other two axes. At
    // coarse lattice points both weights vanish and the bilinear degenerates
    // to the coarse sample itself.
    const auto coarsePlaneDensity = [&](std::uint32_t axis, std::int64_t perpGlobalFine, std::int64_t tangentialGlobalU,
                                        std::int64_t tangentialGlobalV) -> double
    {
        const std::int64_t perpCoarse = perpGlobalFine >> 1;
        const std::int64_t coarseU0   = tangentialGlobalU >> 1;
        const std::int64_t coarseV0   = tangentialGlobalV >> 1;
        const double       weightU    = (tangentialGlobalU & 1) != 0 ? 0.5 : 0.0;
        const double       weightV    = (tangentialGlobalV & 1) != 0 ? 0.5 : 0.0;

        // The four surrounding on-plane coarse lattice points.
        Vec3 point00{0.0, 0.0, 0.0};
        Vec3 point10{0.0, 0.0, 0.0};
        Vec3 point01{0.0, 0.0, 0.0};
        Vec3 point11{0.0, 0.0, 0.0};
        const double perpValue = -half + coarseStep * static_cast<double>(perpCoarse);
        const double u0Value   = -half + coarseStep * static_cast<double>(coarseU0);
        const double u1Value   = -half + coarseStep * static_cast<double>(coarseU0 + ((tangentialGlobalU & 1) != 0 ? 1 : 0));
        const double v0Value   = -half + coarseStep * static_cast<double>(coarseV0);
        const double v1Value   = -half + coarseStep * static_cast<double>(coarseV0 + ((tangentialGlobalV & 1) != 0 ? 1 : 0));
        const std::uint32_t axisU   = axis == 0 ? 1u : 0u;
        const std::uint32_t axisV   = axis == 2 ? 1u : 2u;
        setAxisComponent(point00, axis, perpValue);
        setAxisComponent(point00, axisU, u0Value);
        setAxisComponent(point00, axisV, v0Value);
        setAxisComponent(point10, axis, perpValue);
        setAxisComponent(point10, axisU, u1Value);
        setAxisComponent(point10, axisV, v0Value);
        setAxisComponent(point01, axis, perpValue);
        setAxisComponent(point01, axisU, u0Value);
        setAxisComponent(point01, axisV, v1Value);
        setAxisComponent(point11, axis, perpValue);
        setAxisComponent(point11, axisU, u1Value);
        setAxisComponent(point11, axisV, v1Value);

        if (weightV == 0.0)
        {
            const double d0 = field.densityAt(point00, coarseCutoff);
            const double d1 = field.densityAt(point10, coarseCutoff);
            return d0 + (d1 - d0) * weightU;
        }
        if (weightU == 0.0)
        {
            const double d0 = field.densityAt(point00, coarseCutoff);
            const double d1 = field.densityAt(point01, coarseCutoff);
            return d0 + (d1 - d0) * weightV;
        }
        const double d00 = field.densityAt(point00, coarseCutoff);
        const double d10 = field.densityAt(point10, coarseCutoff);
        const double d01 = field.densityAt(point01, coarseCutoff);
        const double d11 = field.densityAt(point11, coarseCutoff);
        return d00 * (1.0 - weightU) * (1.0 - weightV) + d10 * weightU * (1.0 - weightV)
             + d01 * (1.0 - weightU) * weightV + d11 * weightU * weightV;
    };

    // Trilinear evaluation of the coarse lattice interpolant at an arbitrary
    // planet-local point - the natural continuation of the conforming field
    // BEYOND the boundary plane, used for the apron points of the extension
    // strip. Its far boundary then matches the coarse neighbor's chords to
    // within the chord error of the smooth bilinear field, instead of the
    // (much larger) chord error of the raw field.
    const auto coarseTrilinear = [&](const Vec3 &p) -> double
    {
        const double scaled[3] = {(p.x + half) / coarseStep, (p.y + half) / coarseStep, (p.z + half) / coarseStep};
        const std::int64_t base[3] = {static_cast<std::int64_t>(std::floor(scaled[0])),
            static_cast<std::int64_t>(std::floor(scaled[1])), static_cast<std::int64_t>(std::floor(scaled[2]))};
        const double fraction[3] = {scaled[0] - static_cast<double>(base[0]), scaled[1] - static_cast<double>(base[1]),
            scaled[2] - static_cast<double>(base[2])};
        double sum = 0.0;
        for (std::uint32_t corner = 0; corner < 8; ++corner)
        {
            const std::int64_t offset[3] = {corner & 1u, (corner >> 1) & 1u, (corner >> 2) & 1u};
            const double weight = (offset[0] != 0 ? fraction[0] : 1.0 - fraction[0])
                                * (offset[1] != 0 ? fraction[1] : 1.0 - fraction[1])
                                * (offset[2] != 0 ? fraction[2] : 1.0 - fraction[2]);
            const Vec3 cornerPoint{-half + coarseStep * static_cast<double>(base[0] + offset[0]),
                -half + coarseStep * static_cast<double>(base[1] + offset[1]),
                -half + coarseStep * static_cast<double>(base[2] + offset[2])};
            sum += weight * field.densityAt(cornerPoint, coarseCutoff);
        }
        return sum;
    };

    // One padded lattice point: local indices in [-1, n+1] per axis plus the
    // point's position. Conforming planes are probed in ascending face order
    // (0:-x 1:+x 2:-y 3:+y 4:-z 5:+z); the first conforming hit - which is
    // also the lowest conforming axis - defines the value. Apron points one
    // step beyond a conforming face carry the coarse trilinear field: they
    // feed the extension strip whose far boundary must meet the coarse
    // neighbor's chords (see coarseTrilinear below).
    const auto samplePoint = [&](std::int64_t lx, std::int64_t ly, std::int64_t lz, const Vec3 &p) -> double
    {
        if (!conforming)
        {
            return field.densityAt(p, ownCutoff);
        }
        const std::int64_t globals[3] = {static_cast<std::int64_t>(coord.x) * n + lx,
            static_cast<std::int64_t>(coord.y) * n + ly, static_cast<std::int64_t>(coord.z) * n + lz};
        // Face probe order: axis-major, min side before max side.
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const std::int64_t chunkBase  = static_cast<std::int64_t>(axis == 0 ? coord.x : axis == 1 ? coord.y : coord.z)
                                          * static_cast<std::int64_t>(n);
            const std::int64_t local      = globals[axis] - chunkBase;
            const bool         minPlane   = local == 0;
            const bool         maxPlane   = local == static_cast<std::int64_t>(n);
            const bool         beyond     = local == -1 || local == static_cast<std::int64_t>(n + 1);
            if (!minPlane && !maxPlane && !beyond)
            {
                continue;
            }
            const std::uint32_t face     = axis * 2u + ((maxPlane || local == static_cast<std::int64_t>(n + 1)) ? 1u : 0u);
            // Tangential axes: x planes span (y, z), y planes span (x, z),
            // z planes span (x, y).
            const std::uint32_t axisU    = axis == 0 ? 1u : 0u;
            const std::uint32_t axisV    = axis == 2 ? 1u : 2u;
            const std::int64_t  globalU  = globals[axisU];
            const std::int64_t  globalV  = globals[axisV];
            if (!minPlane && !maxPlane)
            {
                // Apron point one step beyond a conforming face: it lies in
                // the coarse neighbor's territory and feeds the extension
                // strip, so it carries the coarse trilinear field - the
                // extension's far boundary then meets the coarse chords to
                // within the bilinear chord error.
                const std::uint32_t apronFace = face;
                const std::int64_t apronU = globalU;
                const std::int64_t apronV = globalV;
                const std::int64_t apronChunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
                const std::int64_t apronChunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
                const int apronOffsetU = static_cast<int>(floorDiv(apronU, 2 * n) - apronChunkU / 2);
                const int apronOffsetV = static_cast<int>(floorDiv(apronV, 2 * n) - apronChunkV / 2);
                if (apronOffsetU >= -1 && apronOffsetU <= 1 && apronOffsetV >= -1 && apronOffsetV <= 1
                    && conformBit(conformMask, apronFace, apronOffsetU, apronOffsetV))
                {
                    return coarseTrilinear(p);
                }
                continue;
            }
            if ((globalU & 1) != 0 || (globalV & 1) != 0)
            {
                // The depth-(d-1) octree cell beyond this plane containing the
                // point: floor(global / (2n)) per axis, expressed as an offset
                // from the chunk's own coarse-cell block (a depth-(d-1) cell
                // spans 2n fine steps). Offset 0 governs the face interior;
                // +-1 are the diagonal cells reached by apron overshoot.
                const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
                const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
                const int offsetU = static_cast<int>(floorDiv(globalU, 2 * n) - chunkU / 2);
                const int offsetV = static_cast<int>(floorDiv(globalV, 2 * n) - chunkV / 2);
                if (offsetU < -1 || offsetU > 1 || offsetV < -1 || offsetV > 1)
                {
                    continue; // outside the 3x3 ring (cannot happen for plane points)
                }
                if (!conformBit(conformMask, face, offsetU, offsetV))
                {
                    continue; // material beyond this segment is not a coarse leaf
                }
                return coarsePlaneDensity(axis, globals[axis], globalU, globalV);
            }
            // Coarse lattice point. On a conforming segment this must be the
            // COARSE side's value (the coarse neighbor evaluates its own
            // lattice with cutoff(d-1); with per-depth band limiting that is
            // no longer "raw exactly"), and coarsePlaneDensity degenerates to
            // exactly that sample. On a non-conforming segment the point is
            // shared with same-depth neighbors that evaluate it at cutoff(d)
            // - the fallthrough below does the same.
            const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
            const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
            const int offsetU = static_cast<int>(floorDiv(globalU, 2 * n) - chunkU / 2);
            const int offsetV = static_cast<int>(floorDiv(globalV, 2 * n) - chunkV / 2);
            if (offsetU >= -1 && offsetU <= 1 && offsetV >= -1 && offsetV <= 1
                && conformBit(conformMask, face, offsetU, offsetV))
            {
                return coarsePlaneDensity(axis, globals[axis], globalU, globalV);
            }
        }
        return field.densityAt(p, ownCutoff);
    };

    // ---- sample the padded grids -------------------------------------------------
    std::vector<double> densities(gridSize);
    std::vector<double> waters;
    const bool          buildWater = params.waterEnabled;
    if (buildWater)
    {
        waters.resize(gridSize);
    }
    for (std::uint32_t pz = 0; pz < paddedN; ++pz)
    {
        for (std::uint32_t py = 0; py < paddedN; ++py)
        {
            for (std::uint32_t px = 0; px < paddedN; ++px)
            {
                const std::int64_t ix = static_cast<std::int64_t>(px) - 1;
                const std::int64_t iy = static_cast<std::int64_t>(py) - 1;
                const std::int64_t iz = static_cast<std::int64_t>(pz) - 1;
                const Vec3 p{axisCoordinate(coord.x, ix), axisCoordinate(coord.y, iy), axisCoordinate(coord.z, iz)};
                const std::size_t index  = px + strideY * py + strideZ * pz;
                densities[index]         = samplePoint(ix, iy, iz, p);
                if (buildWater)
                {
                    waters[index] = params.derivedSeaRadiusMeters - length(p);
                }
            }
        }
    }

    const auto sampleGrid = [&](const std::vector<double> &grid, std::uint32_t px, std::uint32_t py, std::uint32_t pz)
    { return grid[static_cast<std::size_t>(px) + strideY * py + strideZ * pz]; };

    // ---- marching cubes over the inner cells --------------------------------------
    std::vector<PlanetVertex>                        &vertices = mesh.terrainVertices;
    std::vector<std::uint32_t>                       &indices  = mesh.terrainIndices;
    std::unordered_map<std::uint64_t, std::uint32_t> vertexForEdge;
    vertexForEdge.reserve(4096);

    // Gradient of the trilinear interpolant at an edge, averaged over the (up
    // to) four cells adjacent to the edge on the canonical side. A single-cell
    // gradient is constant per cell: neighboring vertices then carry different
    // constant normals, and every steep crease speckles with dark cell-sized
    // facets (visible as dotted dark lines along ridges). Averaging the four
    // adjacent cells makes the normal field continuous across cell borders.
    // The canonical min-corner bookkeeping keeps the average a pure function
    // of the edge's global position: the sampled density block (2x3x3 around
    // the canonical corner) always lies inside the 1-voxel apron, so two
    // chunks sharing a boundary edge - at the same depth or across depths -
    // evaluate the exact same numbers and carry bit-identical normals.
    const auto edgeGradient = [&](std::uint32_t ax, std::uint32_t ay, std::uint32_t az, std::uint32_t axis, double outGradient[3])
    {
        const std::uint32_t axisB = axis == 0 ? 1u : 0u; // first non-edge axis
        const std::uint32_t axisC = axis == 2 ? 1u : 2u; // second non-edge axis
        double       sum[3]    = {0.0, 0.0, 0.0};
        std::uint32_t cells    = 0;
        for (std::uint32_t cell = 0; cell < 4; ++cell)
        {
            const std::int64_t offsetB = (cell & 1u) != 0u ? 0 : -1;
            const std::int64_t offsetC = (cell & 2u) != 0u ? 0 : -1;
            double corners[2][2][2];
            bool   valid = true;
            for (std::uint32_t i = 0; i < 2 && valid; ++i)
            {
                for (std::uint32_t j = 0; j < 2 && valid; ++j)
                {
                    for (std::uint32_t k = 0; k < 2 && valid; ++k)
                    {
                        const std::int64_t mapped[3] = {static_cast<std::int64_t>(i), static_cast<std::int64_t>(j) + offsetB,
                            static_cast<std::int64_t>(k) + offsetC};
                        std::int64_t address[3] = {static_cast<std::int64_t>(ax), static_cast<std::int64_t>(ay),
                            static_cast<std::int64_t>(az)};
                        address[axis] += mapped[0];
                        address[axisB] += mapped[1];
                        address[axisC] += mapped[2];
                        if (address[0] < 0 || address[1] < 0 || address[2] < 0 || address[0] > std::int64_t(paddedN - 1)
                            || address[1] > std::int64_t(paddedN - 1) || address[2] > std::int64_t(paddedN - 1))
                        {
                            // The cell block reaches past the padded grid
                            // (edges of the own cells sit one step inside the
                            // apron; edges of EXTENDED apron cells sit on the
                            // grid border): skip this neighbor cell. The skip
                            // pattern is a pure function of the canonical
                            // corner, so every chunk evaluating the same edge
                            // skips identically.
                            valid = false;
                            break;
                        }
                        corners[i][j][k] = sampleGrid(densities, static_cast<std::uint32_t>(address[0]),
                            static_cast<std::uint32_t>(address[1]), static_cast<std::uint32_t>(address[2]));
                    }
                }
            }
            if (!valid)
            {
                continue;
            }
            // Central differences of the trilinear interpolant at the cell
            // center (face-averaged directional derivatives).
            double derivative[3] = {0.0, 0.0, 0.0};
            derivative[axis] = 0.25 * ((corners[1][0][0] + corners[1][0][1] + corners[1][1][0] + corners[1][1][1])
                                     - (corners[0][0][0] + corners[0][0][1] + corners[0][1][0] + corners[0][1][1]));
            derivative[axisB] = 0.25 * ((corners[0][1][0] + corners[0][1][1] + corners[1][1][0] + corners[1][1][1])
                                      - (corners[0][0][0] + corners[0][0][1] + corners[1][0][0] + corners[1][0][1]));
            derivative[axisC] = 0.25 * ((corners[0][0][1] + corners[0][1][1] + corners[1][0][1] + corners[1][1][1])
                                      - (corners[0][0][0] + corners[0][1][0] + corners[1][0][0] + corners[1][1][0]));
            sum[axis]   += derivative[axis];
            sum[axisB]  += derivative[axisB];
            sum[axisC]  += derivative[axisC];
            cells       += 1;
        }
        if (cells == 0)
        {
            outGradient[0] = 0.0;
            outGradient[1] = 0.0;
            outGradient[2] = 0.0;
            return;
        }
        outGradient[axis]  = sum[axis] / (static_cast<double>(cells) * step);
        outGradient[axisB] = sum[axisB] / (static_cast<double>(cells) * step);
        outGradient[axisC] = sum[axisC] / (static_cast<double>(cells) * step);
    };

    // ---- marching cubes -------------------------------------------------------------
    //
    // Own cells [0, n)^3 always march. Apron cells (any index at -1 or n)
    // march ONLY when they touch a conforming face: their surface EXTENDS the
    // chunk one cell layer past the transition into the coarse neighbor's
    // territory, where the neighbor's own surface overlaps it - the
    // interpenetration is what hides the sub-voxel boundary mismatch (see the
    // ConformMask comment in meshing.hpp). Apron cells past non-conforming
    // faces stay unmarched: same-depth neighbors already weld bit-identically,
    // and duplicate geometry there would only cost fill rate.
    const auto cellNeedsMarching = [&](std::int64_t cx, std::int64_t cy, std::int64_t cz) -> bool
    {
        if (cx >= 0 && cx < static_cast<std::int64_t>(n) && cy >= 0 && cy < static_cast<std::int64_t>(n) && cz >= 0
            && cz < static_cast<std::int64_t>(n))
        {
            return true; // own cell
        }
        const std::int64_t cell[3] = {cx, cy, cz};
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const bool minApron = cell[axis] == -1;
            const bool maxApron = cell[axis] == static_cast<std::int64_t>(n);
            if (!minApron && !maxApron)
            {
                continue;
            }
            const std::uint32_t face = axis * 2u + (maxApron ? 1u : 0u);
            const std::uint32_t axisU = axis == 0 ? 1u : 0u;
            const std::uint32_t axisV = axis == 2 ? 1u : 2u;
            // The apron cell lies within ONE coarse-cell segment per
            // tangential axis; use its minimum corner to find which.
            const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
            const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
            const std::int64_t globalUMin = chunkU * n + cell[axisU];
            const std::int64_t globalVMin = chunkV * n + cell[axisV];
            const int offsetU = static_cast<int>(floorDiv(globalUMin, 2 * n) - chunkU / 2);
            const int offsetV = static_cast<int>(floorDiv(globalVMin, 2 * n) - chunkV / 2);
            if (offsetU >= -1 && offsetU <= 1 && offsetV >= -1 && offsetV <= 1
                && conformBit(conformMask, face, offsetU, offsetV))
            {
                return true;
            }
        }
        return false;
    };

    for (std::int64_t cz = -1; cz <= static_cast<std::int64_t>(n); ++cz)
    {
        for (std::int64_t cy = -1; cy <= static_cast<std::int64_t>(n); ++cy)
        {
            for (std::int64_t cx = -1; cx <= static_cast<std::int64_t>(n); ++cx)
            {
                if (!cellNeedsMarching(cx, cy, cz))
                {
                    continue;
                }
                double          cornerDensity[8];
                std::uint32_t   cubeIndex = 0;
                for (std::uint32_t corner = 0; corner < 8; ++corner)
                {
                    std::uint32_t dx, dy, dz;
                    cornerOffset(corner, dx, dy, dz);
                    cornerDensity[corner] = sampleGrid(densities, cx + 1 + dx, cy + 1 + dy, cz + 1 + dz);
                    if (cornerDensity[corner] > 0.0)
                    {
                        cubeIndex |= 1u << corner;
                    }
                }
                if (cubeIndex == 0 || cubeIndex == 255)
                {
                    continue;
                }

                std::uint32_t slotVertex[12];
                for (std::uint32_t slot = 0; slot < 12; ++slot)
                {
                    slotVertex[slot] = 0xFFFFFFFFu;
                }

                const std::uint32_t edges = kEdgeTable[cubeIndex];
                for (std::uint32_t slot = 0; slot < 12; ++slot)
                {
                    if ((edges & (1u << slot)) == 0)
                    {
                        continue;
                    }
                    std::uint32_t adx, ady, adz, bdx, bdy, bdz;
                    cornerOffset(kEdgeCorners[slot][0], adx, ady, adz);
                    cornerOffset(kEdgeCorners[slot][1], bdx, bdy, bdz);
                    std::uint32_t aX, aY, aZ, bX, bY, bZ, edgeAxis;
                    canonicalEdge(cx + 1 + adx, cy + 1 + ady, cz + 1 + adz, cx + 1 + bdx, cy + 1 + bdy, cz + 1 + bdz,
                        aX, aY, aZ, bX, bY, bZ, edgeAxis);

                    const std::uint64_t key = edgeKey(aX, aY, aZ, edgeAxis);
                    auto                it  = vertexForEdge.find(key);
                    if (it == vertexForEdge.end())
                    {
                        const double densityA = sampleGrid(densities, aX, aY, aZ);
                        const double densityB = sampleGrid(densities, bX, bY, bZ);
                        // The conforming boundary values can make the two
                        // endpoint densities identical (locally flat coarse
                        // field): the raw division would yield inf/NaN and
                        // the degenerate triangle then spans the whole screen
                        // in the software rasterizer. Clamp the crossing into
                        // the edge; legit crossings already sit in [0, 1].
                        const double denominator = densityA - densityB;
                        const double t        = std::abs(denominator) < 1.0e-12
                                                  ? 0.5
                                                  : std::clamp(densityA / denominator, 0.0, 1.0);

                        // aX/aY/aZ can legitimately be 0 for EXTENDED
                        // apron-cell edges; cast before subtracting so the
                        // index never underflows into an astronomically
                        // distant position.
                        const Vec3 pointA{axisCoordinate(coord.x, static_cast<std::int64_t>(aX) - 1),
                            axisCoordinate(coord.y, static_cast<std::int64_t>(aY) - 1),
                            axisCoordinate(coord.z, static_cast<std::int64_t>(aZ) - 1)};
                        const Vec3 pointB{axisCoordinate(coord.x, static_cast<std::int64_t>(bX) - 1),
                            axisCoordinate(coord.y, static_cast<std::int64_t>(bY) - 1),
                            axisCoordinate(coord.z, static_cast<std::int64_t>(bZ) - 1)};
                        const Vec3 point = pointA + (pointB - pointA) * t;

                        double gradient[3] = {0.0, 0.0, 0.0};
                        edgeGradient(aX, aY, aZ, edgeAxis, gradient);
                        // d > 0 = inside: the gradient points into the body,
                        // surface normals point outward against it.
                        double nx = -gradient[0], ny = -gradient[1], nz = -gradient[2];
                        const double normalLength = std::sqrt(nx * nx + ny * ny + nz * nz);

                        const double radius     = std::max(1.0e-9, length(point));
                        const Vec3   unitRadial = point * (1.0 / radius);
                        if (normalLength > 1.0e-12)
                        {
                            nx /= normalLength;
                            ny /= normalLength;
                            nz /= normalLength;
                        }
                        else
                        {
                            nx = unitRadial.x;
                            ny = unitRadial.y;
                            nz = unitRadial.z;
                        }

                        const double slope01 = std::clamp(1.0 - (nx * unitRadial.x + ny * unitRadial.y + nz * unitRadial.z), 0.0, 1.0);
                        const ClimateSample climate = field.climate().sample(unitRadial, radius, slope01);

                        PlanetVertex vertex{};
                        vertex.position[0] = static_cast<float>(point.x - center.x);
                        vertex.position[1] = static_cast<float>(point.y - center.y);
                        vertex.position[2] = static_cast<float>(point.z - center.z);
                        vertex.normal[0]   = static_cast<float>(nx);
                        vertex.normal[1]   = static_cast<float>(ny);
                        vertex.normal[2]   = static_cast<float>(nz);
                        vertex.color[0]    = climate.color[0];
                        vertex.color[1]    = climate.color[1];
                        vertex.color[2]    = climate.color[2];
                        const double longitude = std::atan2(unitRadial.z, unitRadial.x) * 0.15915494309189535 + 0.5;
                        const double latitude  = 0.5 - std::asin(std::clamp(unitRadial.y, -1.0, 1.0)) * 0.3183098861837907;
                        vertex.uv[0] = static_cast<float>(longitude);
                        vertex.uv[1] = static_cast<float>(latitude);

                        it = vertexForEdge.emplace(key, static_cast<std::uint32_t>(vertices.size())).first;
                        vertices.push_back(vertex);
                    }
                    slotVertex[slot] = it->second;
                }

                for (std::uint32_t triangle = 0; triangle < 5; ++triangle)
                {
                    const std::int8_t v0 = kTriTable[cubeIndex][triangle * 3 + 0];
                    if (v0 < 0)
                    {
                        break;
                    }
                    const std::int8_t v1 = kTriTable[cubeIndex][triangle * 3 + 1];
                    const std::int8_t v2 = kTriTable[cubeIndex][triangle * 3 + 2];
                    if (!kTerrainFlipWinding)
                    {
                        indices.insert(indices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                            slotVertex[static_cast<std::uint32_t>(v1)], slotVertex[static_cast<std::uint32_t>(v2)]});
                    }
                    else
                    {
                        indices.insert(indices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                            slotVertex[static_cast<std::uint32_t>(v2)], slotVertex[static_cast<std::uint32_t>(v1)]});
                    }
                    mesh.triangleCount += 1;
                }
            }
        }
    }

    // ---- chunk-boundary continuity ----------------------------------------------------
    //
    // No skirt walls: same-depth neighbors sample a shared global lattice, so
    // their boundary vertices are bit-identical and the surface is watertight
    // by construction. Across depth changes, the octree enforces a 2:1 balance
    // (see octree.hpp), which keeps the T-junction residual between the coarse
    // chord and the fine isoline far below one voxel - invisible, and far
    // cheaper than painted-over crack walls.
    // ---- water surface patch --------------------------------------------------------
    if (buildWater)
    {
        // Nodes whose bounding sphere misses the sea shell have no water.
        const double centerRadius  = length(center);
        const double nodeBound     = 0.5 * nodeSize * std::sqrt(3.0);
        const double seaRadius     = params.derivedSeaRadiusMeters;
        if (centerRadius - nodeBound <= seaRadius + step && centerRadius + nodeBound >= seaRadius - step)
        {
            std::vector<PlanetVertex>                        &waterVertices = mesh.waterVertices;
            std::vector<std::uint32_t>                       &waterIndices  = mesh.waterIndices;
            std::unordered_map<std::uint64_t, std::uint32_t> waterForEdge;
            waterForEdge.reserve(1024);

            for (std::uint32_t cz = 0; cz < n; ++cz)
            {
                for (std::uint32_t cy = 0; cy < n; ++cy)
                {
                    for (std::uint32_t cx = 0; cx < n; ++cx)
                    {
                        std::uint32_t cubeIndex = 0;
                        double minCornerDensity = 1.0e30;
                        for (std::uint32_t corner = 0; corner < 8; ++corner)
                        {
                            std::uint32_t dx, dy, dz;
                            cornerOffset(corner, dx, dy, dz);
                            if (sampleGrid(waters, cx + 1 + dx, cy + 1 + dy, cz + 1 + dz) > 0.0)
                            {
                                cubeIndex |= 1u << corner;
                            }
                            minCornerDensity = std::min(minCornerDensity, sampleGrid(densities, cx + 1 + dx, cy + 1 + dy, cz + 1 + dz));
                        }
                        if (cubeIndex == 0 || cubeIndex == 255)
                        {
                            continue;
                        }
                        if (minCornerDensity > 0.0)
                        {
                            // Fully buried water: the terrain surface is above
                            // the sea everywhere in this cell. The patch here
                            // is invisible anyway - except as grazing-angle
                            // slivers where it grazes the terrain surface.
                            // Skip it.
                            continue;
                        }

                        std::uint32_t slotVertex[12];
                        for (std::uint32_t slot = 0; slot < 12; ++slot)
                        {
                            slotVertex[slot] = 0xFFFFFFFFu;
                        }

                        const std::uint32_t edges = kEdgeTable[cubeIndex];
                        for (std::uint32_t slot = 0; slot < 12; ++slot)
                        {
                            if ((edges & (1u << slot)) == 0)
                            {
                                continue;
                            }
                            std::uint32_t adx, ady, adz, bdx, bdy, bdz;
                            cornerOffset(kEdgeCorners[slot][0], adx, ady, adz);
                            cornerOffset(kEdgeCorners[slot][1], bdx, bdy, bdz);
                            std::uint32_t aX, aY, aZ, bX, bY, bZ, edgeAxis;
                            canonicalEdge(cx + 1 + adx, cy + 1 + ady, cz + 1 + adz, cx + 1 + bdx, cy + 1 + bdy,
                                cz + 1 + bdz, aX, aY, aZ, bX, bY, bZ, edgeAxis);
                            const std::uint64_t key = edgeKey(aX, aY, aZ, edgeAxis);
                            auto                it  = waterForEdge.find(key);
                            if (it == waterForEdge.end())
                            {
                                const double waterA = sampleGrid(waters, aX, aY, aZ);
                                const double waterB = sampleGrid(waters, bX, bY, bZ);
                                const double wDenominator = waterA - waterB;
                                const double t      = std::abs(wDenominator) < 1.0e-12
                                                          ? 0.5
                                                          : std::clamp(waterA / wDenominator, 0.0, 1.0);
                                const Vec3 pointA{axisCoordinate(coord.x, aX - 1), axisCoordinate(coord.y, aY - 1),
                                    axisCoordinate(coord.z, aZ - 1)};
                                const Vec3 pointB{axisCoordinate(coord.x, bX - 1), axisCoordinate(coord.y, bY - 1),
                                    axisCoordinate(coord.z, bZ - 1)};
                                const Vec3   point  = pointA + (pointB - pointA) * t;
                                const double radius = std::max(1.0e-9, length(point));
                                const Vec3   radial = point * (1.0 / radius);

                                // Water depth = the exact terrain density (meters)
                                // at the sea surface, straight from the field.
                                // The per-chunk trilinear grid would make the
                                // tint LOD-dependent: neighboring chunks at
                                // different depths disagree, painting straight
                                // color edges into open water. The exact field
                                // is one pure function - every chunk, at any
                                // depth, agrees bit-for-bit.
                                const double terrainDepth = std::max(0.0, field.densityAt(point));
                                const double depthTint    = std::clamp(terrainDepth / std::max(1.0, params.waterDepthTintMeters), 0.0, 1.0);
                                const double foam         = smoothstep(2.5, 0.0, terrainDepth) * 0.35;

                                PlanetVertex vertex{};
                                vertex.position[0] = static_cast<float>(point.x - center.x);
                                vertex.position[1] = static_cast<float>(point.y - center.y);
                                vertex.position[2] = static_cast<float>(point.z - center.z);
                                vertex.normal[0]   = static_cast<float>(radial.x);
                                vertex.normal[1]   = static_cast<float>(radial.y);
                                vertex.normal[2]   = static_cast<float>(radial.z);
                                for (std::uint32_t channel = 0; channel < 3; ++channel)
                                {
                                    const float shallow = params.waterColorShallow[channel];
                                    const float deep    = params.waterColorDeep[channel];
                                    float blended       = shallow + static_cast<float>((deep - shallow) * depthTint);
                                    blended             = static_cast<float>(blended + (0.92 - blended) * foam);
                                    vertex.color[channel] = blended;
                                }
                                vertex.uv[0] = 0.0f;
                                vertex.uv[1] = 0.0f;
                                it = waterForEdge.emplace(key, static_cast<std::uint32_t>(waterVertices.size())).first;
                                waterVertices.push_back(vertex);
                            }
                            slotVertex[slot] = it->second;
                        }

                        for (std::uint32_t triangle = 0; triangle < 5; ++triangle)
                        {
                            const std::int8_t v0 = kTriTable[cubeIndex][triangle * 3 + 0];
                            if (v0 < 0)
                            {
                                break;
                            }
                            const std::int8_t v1 = kTriTable[cubeIndex][triangle * 3 + 1];
                            const std::int8_t v2 = kTriTable[cubeIndex][triangle * 3 + 2];
                            // The sea surface only ever faces away from the
                            // center: auto-orient each triangle radially.
                            const PlanetVertex &pv0 = waterVertices[slotVertex[static_cast<std::uint32_t>(v0)]];
                            const PlanetVertex &pv1 = waterVertices[slotVertex[static_cast<std::uint32_t>(v1)]];
                            const PlanetVertex &pv2 = waterVertices[slotVertex[static_cast<std::uint32_t>(v2)]];
                            const Vec3 p0{pv0.position[0], pv0.position[1], pv0.position[2]};
                            const Vec3 p1{pv1.position[0], pv1.position[1], pv1.position[2]};
                            const Vec3 p2{pv2.position[0], pv2.position[1], pv2.position[2]};
                            // Orient against the TRUE radial direction: the
                            // centroid in planet-local space (chunk-local
                            // vertices alone point no particular way - the
                            // water sheet is flat inside its chunk).
                            const Vec3 centroidWorld = (p0 + p1 + p2) * (1.0 / 3.0) + center;
                            const bool flip = dot(cross(p1 - p0, p2 - p0), normalized(centroidWorld)) < 0.0;
                            if (!flip)
                            {
                                waterIndices.insert(waterIndices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                                    slotVertex[static_cast<std::uint32_t>(v1)], slotVertex[static_cast<std::uint32_t>(v2)]});
                            }
                            else
                            {
                                waterIndices.insert(waterIndices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                                    slotVertex[static_cast<std::uint32_t>(v2)], slotVertex[static_cast<std::uint32_t>(v1)]});
                            }
                            mesh.triangleCount += 1;
                        }
                    }
                }
            }
        }
    }

    // ---- hash (determinism self-tests + change detection) -----------------------------
    std::uint64_t hash = 0xCBF29CE484222325ull;
    hash = fnv1a64(mesh.terrainVertices.data(), mesh.terrainVertices.size() * sizeof(PlanetVertex), hash);
    hash = fnv1a64(mesh.terrainIndices.data(), mesh.terrainIndices.size() * sizeof(std::uint32_t), hash);
    hash = fnv1a64(mesh.waterVertices.data(), mesh.waterVertices.size() * sizeof(PlanetVertex), hash);
    hash = fnv1a64(mesh.waterIndices.data(), mesh.waterIndices.size() * sizeof(std::uint32_t), hash);
    mesh.hash = hash;
    return mesh;
}

ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord)
{
    // Unconstrained build: no boundary segment conforms to a coarser lattice.
    return buildChunkMesh(params, field, coord, ConformMask{});
}

} // namespace planet
} // namespace engine
