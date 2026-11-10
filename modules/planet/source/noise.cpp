// OpenSimplex2 noise implementation (2D + 3D).
//
// Faithful C++ port of the CC0-licensed reference by KdotJPG
// (https://github.com/KdotJPG/OpenSimplex2, csharp/OpenSimplex2.cs).
// Constants, gradient tables and lattice walks match the reference so the
// characteristic OpenSimplex2 look and quality carry over 1:1. The
// reference evaluates its falloff terms in single precision on the hot
// path; this port keeps that behavior (it is part of the algorithm's
// performance profile, and float math is deterministic).

#include <engine/planet/noise.hpp>

#include <array>
#include <cmath>

namespace engine
{
namespace planet
{
namespace noise
{
namespace
{

constexpr std::int64_t primeX          = 0x5205402B9270C86Fll;
constexpr std::int64_t primeY          = 0x598CD327003817B5ll;
constexpr std::int64_t primeZ          = 0x5BCC226E9FA0BACBll;
constexpr std::int64_t hashMultiplier  = 0x53A3F72DEEC546F5ll;
constexpr std::int64_t seedFlip3D      = -0x52D547B2E96ED629ll;

constexpr double root2Over2 = 0.7071067811865476;
constexpr double skew2D     = 0.366025403784439;
constexpr double unskew2D   = -0.21132486540518713;

constexpr double root3Over3             = 0.577350269189626;
constexpr double rotate3DOrthogonalizer = unskew2D; // per the reference

constexpr int nGrads2DExponent = 7;
constexpr int nGrads3DExponent = 8;
constexpr int nGrads2D         = 1 << nGrads2DExponent;
constexpr int nGrads3D         = 1 << nGrads3DExponent;

constexpr double normalizer2D = 0.01001634121365712;
constexpr double normalizer3D = 0.07969837668935331;

constexpr float rsquared2D = 0.5f;
constexpr float rsquared3D = 0.6f;

int fastFloor(double value)
{
    const int truncated = static_cast<int>(value);
    return value < static_cast<double>(truncated) ? truncated - 1 : truncated;
}

int fastRound(double value)
{
    return value < 0.0 ? static_cast<int>(value - 0.5) : static_cast<int>(value + 0.5);
}

// Gradient tables: the reference's base gradients (unscaled), replicated
// cyclically to fill 2^exponent entries, then divided by the normalizer so
// the final range is about [-1, 1].
struct GradientTables
{
    std::array<float, nGrads2D * 2> gradients2D{};
    std::array<float, nGrads3D * 4> gradients3D{};
};

GradientTables makeGradientTables()
{
    GradientTables tables{};

    static constexpr float base2D[] = {
        0.38268343236509f,   0.923879532511287f,  0.923879532511287f,  0.38268343236509f,
        0.923879532511287f,  -0.38268343236509f,  0.38268343236509f,   -0.923879532511287f,
        -0.38268343236509f,  -0.923879532511287f, -0.923879532511287f, -0.38268343236509f,
        -0.923879532511287f,  0.38268343236509f,  -0.38268343236509f,   0.923879532511287f,
        //--------------------------------------------------------------//
        0.130526192220052f,  0.99144486137381f,   0.608761429008721f,  0.793353340291235f,
        0.793353340291235f,  0.608761429008721f,  0.99144486137381f,   0.130526192220051f,
        0.99144486137381f,   -0.130526192220051f, 0.793353340291235f,  -0.60876142900872f,
        0.608761429008721f,  -0.793353340291235f, 0.130526192220052f,  -0.99144486137381f,
        -0.130526192220052f, -0.99144486137381f,  -0.608761429008721f, -0.793353340291235f,
        -0.793353340291235f, -0.608761429008721f, -0.99144486137381f,  -0.130526192220052f,
        -0.99144486137381f,  0.130526192220051f,  -0.793353340291235f,  0.608761429008721f,
        -0.608761429008721f, 0.793353340291235f,  -0.130526192220052f,  0.99144486137381f,
    };

    static constexpr float base3D[] = {
        2.22474487139f,      2.22474487139f,      -1.0f,                0.0f,
        2.22474487139f,      2.22474487139f,       1.0f,                0.0f,
        3.0862664687972017f, 1.1721513422464978f,  0.0f,                0.0f,
        1.1721513422464978f, 3.0862664687972017f,  0.0f,                0.0f,
        -2.22474487139f,     2.22474487139f,      -1.0f,                0.0f,
        -2.22474487139f,     2.22474487139f,       1.0f,                0.0f,
        -1.1721513422464978f, 3.0862664687972017f, 0.0f,                0.0f,
        -3.0862664687972017f, 1.1721513422464978f, 0.0f,                0.0f,
        -1.0f,               -2.22474487139f,     -2.22474487139f,      0.0f,
        1.0f,                -2.22474487139f,     -2.22474487139f,      0.0f,
        0.0f,                -3.0862664687972017f, -1.1721513422464978f, 0.0f,
        0.0f,                -1.1721513422464978f, -3.0862664687972017f, 0.0f,
        -1.0f,               -2.22474487139f,      2.22474487139f,      0.0f,
        1.0f,                -2.22474487139f,      2.22474487139f,      0.0f,
        0.0f,                -1.1721513422464978f,  3.0862664687972017f, 0.0f,
        0.0f,                -3.0862664687972017f,  1.1721513422464978f, 0.0f,
        //---------------------------------------------------------------//
        -2.22474487139f,     -2.22474487139f,     -1.0f,                 0.0f,
        -2.22474487139f,     -2.22474487139f,      1.0f,                 0.0f,
        -3.0862664687972017f, -1.1721513422464978f, 0.0f,                0.0f,
        -1.1721513422464978f, -3.0862664687972017f, 0.0f,                0.0f,
        -2.22474487139f,     -1.0f,               -2.22474487139f,       0.0f,
        -2.22474487139f,      1.0f,               -2.22474487139f,       0.0f,
        -1.1721513422464978f,  0.0f,               -3.0862664687972017f, 0.0f,
        -3.0862664687972017f,  0.0f,               -1.1721513422464978f, 0.0f,
        -2.22474487139f,     -1.0f,                2.22474487139f,       0.0f,
        -2.22474487139f,      1.0f,                2.22474487139f,       0.0f,
        -3.0862664687972017f,  0.0f,                1.1721513422464978f, 0.0f,
        -1.1721513422464978f,  0.0f,                3.0862664687972017f, 0.0f,
        -1.0f,                2.22474487139f,      -2.22474487139f,       0.0f,
        1.0f,                 2.22474487139f,      -2.22474487139f,       0.0f,
        0.0f,                 1.1721513422464978f, -3.0862664687972017f, 0.0f,
        0.0f,                 3.0862664687972017f, -1.1721513422464978f, 0.0f,
        -1.0f,                2.22474487139f,       2.22474487139f,       0.0f,
        1.0f,                 2.22474487139f,       2.22474487139f,       0.0f,
        0.0f,                 3.0862664687972017f,  1.1721513422464978f, 0.0f,
        0.0f,                 1.1721513422464978f,  3.0862664687972017f, 0.0f,
        2.22474487139f,      -2.22474487139f,     -1.0f,                 0.0f,
        2.22474487139f,      -2.22474487139f,      1.0f,                 0.0f,
        1.1721513422464978f, -3.0862664687972017f,  0.0f,                0.0f,
        3.0862664687972017f, -1.1721513422464978f,  0.0f,                0.0f,
        2.22474487139f,      -1.0f,               -2.22474487139f,       0.0f,
        2.22474487139f,       1.0f,               -2.22474487139f,       0.0f,
        3.0862664687972017f,  0.0f,               -1.1721513422464978f, 0.0f,
        1.1721513422464978f,  0.0f,               -3.0862664687972017f, 0.0f,
        2.22474487139f,      -1.0f,                2.22474487139f,       0.0f,
        2.22474487139f,       1.0f,                2.22474487139f,       0.0f,
        1.1721513422464978f,  0.0f,                3.0862664687972017f, 0.0f,
        3.0862664687972017f,  0.0f,                1.1721513422464978f, 0.0f,
    };

    for (std::size_t index = 0; index < tables.gradients2D.size(); ++index)
    {
        tables.gradients2D[index] = base2D[index % (sizeof(base2D) / sizeof(base2D[0]))] / static_cast<float>(normalizer2D);
    }
    for (std::size_t index = 0; index < tables.gradients3D.size(); ++index)
    {
        tables.gradients3D[index] = base3D[index % (sizeof(base3D) / sizeof(base3D[0]))] / static_cast<float>(normalizer3D);
    }
    return tables;
}

const GradientTables &gradientTables()
{
    static const GradientTables tables = makeGradientTables();
    return tables;
}

inline float grad2(std::int64_t seed, std::int64_t xsvp, std::int64_t ysvp, float dx, float dy)
{
    std::int64_t hash = seed ^ xsvp ^ ysvp;
    hash *= hashMultiplier;
    hash ^= hash >> (64 - nGrads2DExponent + 1);
    const int gradientIndex = static_cast<int>(hash) & ((nGrads2D - 1) << 1);
    const auto &table       = gradientTables().gradients2D;
    return table[static_cast<std::size_t>(gradientIndex | 0)] * dx + table[static_cast<std::size_t>(gradientIndex | 1)] * dy;
}

inline float grad3(std::int64_t seed, std::int64_t xrvp, std::int64_t yrvp, std::int64_t zrvp, float dx, float dy, float dz)
{
    std::int64_t hash = (seed ^ xrvp) ^ (yrvp ^ zrvp);
    hash *= hashMultiplier;
    hash ^= hash >> (64 - nGrads3DExponent + 2);
    const int gradientIndex = static_cast<int>(hash) & ((nGrads3D - 1) << 2);
    const auto &table       = gradientTables().gradients3D;
    return table[static_cast<std::size_t>(gradientIndex | 0)] * dx + table[static_cast<std::size_t>(gradientIndex | 1)] * dy
         + table[static_cast<std::size_t>(gradientIndex | 2)] * dz;
}

float noise2UnskewedBase(std::int64_t seed, double xs, double ys)
{
    // Base cell and fractional offset inside it.
    const int  xsb = fastFloor(xs);
    const int  ysb = fastFloor(ys);
    const float xi = static_cast<float>(xs - static_cast<double>(xsb));
    const float yi = static_cast<float>(ys - static_cast<double>(ysb));

    // Prime pre-multiplication for the hash.
    const std::int64_t xsbp = static_cast<std::int64_t>(xsb) * primeX;
    const std::int64_t ysbp = static_cast<std::int64_t>(ysb) * primeY;

    // Unskew.
    const float t    = (xi + yi) * static_cast<float>(unskew2D);
    const float dx0  = xi + t;
    const float dy0  = yi + t;

    // First vertex.
    float value = 0.0f;
    float a0    = rsquared2D - dx0 * dx0 - dy0 * dy0;
    if (a0 > 0.0f)
    {
        value = (a0 * a0) * (a0 * a0) * grad2(seed, xsbp, ysbp, dx0, dy0);
    }

    // Second vertex.
    const float a1 = static_cast<float>(2.0 * (1.0 + 2.0 * unskew2D) * (1.0 / unskew2D + 2.0)) * t
                   + (static_cast<float>(-2.0 * (1.0 + 2.0 * unskew2D) * (1.0 + 2.0 * unskew2D)) + a0);
    if (a1 > 0.0f)
    {
        const float dx1 = dx0 - static_cast<float>(1.0 + 2.0 * unskew2D);
        const float dy1 = dy0 - static_cast<float>(1.0 + 2.0 * unskew2D);
        value += (a1 * a1) * (a1 * a1) * grad2(seed, xsbp + primeX, ysbp + primeY, dx1, dy1);
    }

    // Third vertex: the nearer of the two side simplex corners.
    if (dy0 > dx0)
    {
        const float dx2 = dx0 - static_cast<float>(unskew2D);
        const float dy2 = dy0 - static_cast<float>(unskew2D + 1.0);
        const float a2  = rsquared2D - dx2 * dx2 - dy2 * dy2;
        if (a2 > 0.0f)
        {
            value += (a2 * a2) * (a2 * a2) * grad2(seed, xsbp, ysbp + primeY, dx2, dy2);
        }
    }
    else
    {
        const float dx2 = dx0 - static_cast<float>(unskew2D + 1.0);
        const float dy2 = dy0 - static_cast<float>(unskew2D);
        const float a2  = rsquared2D - dx2 * dx2 - dy2 * dy2;
        if (a2 > 0.0f)
        {
            value += (a2 * a2) * (a2 * a2) * grad2(seed, xsbp + primeX, ysbp, dx2, dy2);
        }
    }

    return value;
}

float noise3UnrotatedBase(std::int64_t seed, double xr, double yr, double zr)
{
    // Base BCC lattice points and offsets.
    const int xrb = fastRound(xr);
    const int yrb = fastRound(yr);
    const int zrb = fastRound(zr);
    float xri = static_cast<float>(xr - static_cast<double>(xrb));
    float yri = static_cast<float>(yr - static_cast<double>(yrb));
    float zri = static_cast<float>(zr - static_cast<double>(zrb));

    // -1 if positive, 1 if negative.
    int xNSign = static_cast<int>(-1.0f - xri) | 1;
    int yNSign = static_cast<int>(-1.0f - yri) | 1;
    int zNSign = static_cast<int>(-1.0f - zri) | 1;

    // Absolute values via the sign shortcut.
    float ax0 = static_cast<float>(xNSign) * -xri;
    float ay0 = static_cast<float>(yNSign) * -yri;
    float az0 = static_cast<float>(zNSign) * -zri;

    // Prime pre-multiplication for the hash.
    std::int64_t xrbp = static_cast<std::int64_t>(xrb) * primeX;
    std::int64_t yrbp = static_cast<std::int64_t>(yrb) * primeY;
    std::int64_t zrbp = static_cast<std::int64_t>(zrb) * primeZ;

    // Two overlapping cubic lattices (BCC): pick an edge on each copy.
    float value = 0.0f;
    float a     = (rsquared3D - xri * xri) - (yri * yri + zri * zri);
    for (int latticeIndex = 0;; ++latticeIndex)
    {
        // Closest point on the cube.
        if (a > 0.0f)
        {
            value += (a * a) * (a * a) * grad3(seed, xrbp, yrbp, zrbp, xri, yri, zri);
        }

        // Second-closest point on the same lattice copy.
        if (ax0 >= ay0 && ax0 >= az0)
        {
            float b = a + ax0 + ax0;
            if (b > 1.0f)
            {
                b -= 1.0f;
                value += (b * b) * (b * b)
                       * grad3(seed, xrbp - static_cast<std::int64_t>(xNSign) * primeX, yrbp, zrbp,
                           xri + static_cast<float>(xNSign), yri, zri);
            }
        }
        else if (ay0 > ax0 && ay0 >= az0)
        {
            float b = a + ay0 + ay0;
            if (b > 1.0f)
            {
                b -= 1.0f;
                value += (b * b) * (b * b)
                       * grad3(seed, xrbp, yrbp - static_cast<std::int64_t>(yNSign) * primeY, zrbp,
                           xri, yri + static_cast<float>(yNSign), zri);
            }
        }
        else
        {
            float b = a + az0 + az0;
            if (b > 1.0f)
            {
                b -= 1.0f;
                value += (b * b) * (b * b)
                       * grad3(seed, xrbp, yrbp, zrbp - static_cast<std::int64_t>(zNSign) * primeZ,
                           xri, yri, zri + static_cast<float>(zNSign));
            }
        }

        // Both lattice copies evaluated.
        if (latticeIndex == 1)
        {
            break;
        }

        // Move to the second lattice copy.
        ax0 = 0.5f - ax0;
        ay0 = 0.5f - ay0;
        az0 = 0.5f - az0;

        xri = static_cast<float>(xNSign) * ax0;
        yri = static_cast<float>(yNSign) * ay0;
        zri = static_cast<float>(zNSign) * az0;

        a += (0.75f - ax0) - (ay0 + az0);

        xrbp += (static_cast<std::int64_t>(xNSign) >> 1) & primeX;
        yrbp += (static_cast<std::int64_t>(yNSign) >> 1) & primeY;
        zrbp += (static_cast<std::int64_t>(zNSign) >> 1) & primeZ;

        xNSign = -xNSign;
        yNSign = -yNSign;
        zNSign = -zNSign;

        seed ^= seedFlip3D;
    }

    return value;
}

} // namespace

double simplex2(std::int64_t seed, double x, double y)
{
    // Skew the input space to the A2* lattice.
    const double s  = skew2D * (x + y);
    const double xs = x + s;
    const double ys = y + s;
    return static_cast<double>(noise2UnskewedBase(seed, xs, ys));
}

double simplex3ImproveXY(std::int64_t seed, double x, double y, double z)
{
    // Re-orient the cubic lattices without skewing so Z points up the main
    // lattice diagonal (orthonormal rotation, per the reference).
    const double xy = x + y;
    const double s2 = xy * rotate3DOrthogonalizer;
    const double zz = z * root3Over3;
    const double xr = x + s2 + zz;
    const double yr = y + s2 + zz;
    const double zr = xy * -root3Over3 + zz;
    return static_cast<double>(noise3UnrotatedBase(seed, xr, yr, zr));
}

double simplex3ImproveXZ(std::int64_t seed, double x, double y, double z)
{
    // Re-orient the cubic lattices without skewing so Y points up the main
    // lattice diagonal (orthonormal rotation, per the reference). This is
    // the terrain-friendly orientation: horizontal (X, Z) looks isotropic.
    const double xz = x + z;
    const double s2 = xz * rotate3DOrthogonalizer;
    const double yy = y * root3Over3;
    const double xr = x + s2 + yy;
    const double zr = z + s2 + yy;
    const double yr = xz * -root3Over3 + yy;
    return static_cast<double>(noise3UnrotatedBase(seed, xr, yr, zr));
}

} // namespace noise
} // namespace planet
} // namespace engine
