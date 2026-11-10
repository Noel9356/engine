// Quaternion operations shared by engine modules.
//
// Quaternions use (x, y, z, w) component order and rotate column vectors. The
// functions are dependency-free and operate on caller-owned four/three-float
// arrays so capability-facing data structures remain unchanged.

#ifndef ENGINE_SHARED_MATH_QUATERNION_HPP
#define ENGINE_SHARED_MATH_QUATERNION_HPP

#include <cmath>

namespace engine::math
{

// Normalizes in double precision. Invalid or near-zero inputs are left intact
// and reported to the caller; no allocation or hidden fallback is performed.
inline bool normalizeQuaternion(float value[4]) noexcept
{
    const double lengthSquared = static_cast<double>(value[0]) * value[0]
        + static_cast<double>(value[1]) * value[1] + static_cast<double>(value[2]) * value[2]
        + static_cast<double>(value[3]) * value[3];
    if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-16)
        return false;

    const double inverseLength = 1.0 / std::sqrt(lengthSquared);
    for (int index = 0; index < 4; ++index)
        value[index] = static_cast<float>(static_cast<double>(value[index]) * inverseLength);
    return true;
}

inline float quaternionDot(const float left[4], const float right[4]) noexcept
{
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2] + left[3] * right[3];
}

// Computes left * right. Temporaries make output aliasing either input safe.
inline void multiplyQuaternion(const float left[4], const float right[4], float output[4]) noexcept
{
    const float x = left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1];
    const float y = left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0];
    const float z = left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3];
    const float w = left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2];
    output[0] = x;
    output[1] = y;
    output[2] = z;
    output[3] = w;
}

// Rotates a vector by a unit quaternion. Local copies make value/output
// aliasing safe and preserve the module-wide (x, y, z, w) convention.
inline void rotateVector(const float quaternion[4], const float value[3], float output[3]) noexcept
{
    const float qx = quaternion[0];
    const float qy = quaternion[1];
    const float qz = quaternion[2];
    const float qw = quaternion[3];
    const float x = value[0];
    const float y = value[1];
    const float z = value[2];

    const float tx = 2.0f * (qy * z - qz * y);
    const float ty = 2.0f * (qz * x - qx * z);
    const float tz = 2.0f * (qx * y - qy * x);
    output[0] = x + qw * tx + (qy * tz - qz * ty);
    output[1] = y + qw * ty + (qz * tx - qx * tz);
    output[2] = z + qw * tz + (qx * ty - qy * tx);
}

} // namespace engine::math

#endif // ENGINE_SHARED_MATH_QUATERNION_HPP
