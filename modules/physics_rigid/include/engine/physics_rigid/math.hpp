// math.hpp - public double-precision vector, quaternion, and matrix
// vocabulary over the physics_rigid capability's plain-data types. Consumers
// of the capability (dependent simulation modules, applications) use the same
// helpers as the module internals, so every consumer agrees on one
// implementation. Operators live in the vector types' own namespace so
// argument-dependent lookup finds them anywhere; named functions live in the
// math namespace. Everything here is pure and allocation-free.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <algorithm>
#include <cmath>

// Operators on the capability's Vec3 vocabulary: argument-dependent lookup
// resolves them wherever Vec3 operands appear.
namespace engine::physics_rigid
{

inline Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator-(const Vec3 &a)
{
    return Vec3{-a.x, -a.y, -a.z};
}

inline Vec3 operator*(const Vec3 &a, double scalar)
{
    return Vec3{a.x * scalar, a.y * scalar, a.z * scalar};
}

inline Vec3 operator*(double scalar, const Vec3 &a)
{
    return a * scalar;
}

inline Vec3 &operator+=(Vec3 &a, const Vec3 &b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vec3 &operator-=(Vec3 &a, const Vec3 &b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Vec3 &operator*=(Vec3 &a, double scalar)
{
    a.x *= scalar;
    a.y *= scalar;
    a.z *= scalar;
    return a;
}

inline Quaternion operator+(const Quaternion &a, const Quaternion &b)
{
    return Quaternion{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

inline Quaternion operator*(const Quaternion &a, const Quaternion &b)
{
    return Quaternion{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

} // namespace engine::physics_rigid

namespace engine::physics_rigid::math
{

// The math namespace operates on the capability's plain-data vocabulary.
using Vec3           = ::engine::physics_rigid::Vec3;
using Quaternion     = ::engine::physics_rigid::Quaternion;
using RigidTransform = ::engine::physics_rigid::RigidTransform;

constexpr double kPi = 3.14159265358979323846;

// Component access by axis index (0 = x, 1 = y, 2 = z), for the axis-driven
// loops the shapes and solver run. Out-of-range axes clamp to z. The mutable
// overload writes one component without any pointer arithmetic between
// members.
inline double &component(Vec3 &vector, std::uint32_t axis)
{
    return axis == 0u ? vector.x : (axis == 1u ? vector.y : vector.z);
}

inline double component(const Vec3 &vector, std::uint32_t axis)
{
    return axis == 0u ? vector.x : (axis == 1u ? vector.y : vector.z);
}

inline double dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double lengthSquared(const Vec3 &a)
{
    return dot(a, a);
}

inline double length(const Vec3 &a)
{
    return std::sqrt(lengthSquared(a));
}

inline double distanceSquared(const Vec3 &a, const Vec3 &b)
{
    return lengthSquared(b - a);
}

// Unit-length vector; a zero-length input yields the zero vector.
inline Vec3 normalized(const Vec3 &a)
{
    const double magnitude = length(a);
    return magnitude > 0.0 ? a * (1.0 / magnitude) : Vec3{};
}

inline bool isFinite(const Vec3 &a)
{
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

inline Vec3 lerp(const Vec3 &a, const Vec3 &b, double alpha)
{
    return a + (b - a) * alpha;
}

inline double dot(const Quaternion &a, const Quaternion &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Quaternion normalize(const Quaternion &a)
{
    const double magnitude = std::sqrt(dot(a, a));
    if (magnitude <= 0.0)
    {
        return Quaternion{};
    }
    return Quaternion{a.x / magnitude, a.y / magnitude, a.z / magnitude, a.w / magnitude};
}

inline Quaternion conjugate(const Quaternion &a)
{
    return Quaternion{-a.x, -a.y, -a.z, a.w};
}

// Rotates v by the unit quaternion a.
inline Vec3 rotate(const Quaternion &a, const Vec3 &v)
{
    const Vec3 q{a.x, a.y, a.z};
    const Vec3 crossQv = cross(q, v);
    // v' = v + 2 * w * (q x v) + 2 * (q x (q x v))
    return v + (crossQv * (2.0 * a.w)) + cross(q, crossQv) * 2.0;
}

// Rotates v by the inverse of the unit quaternion a.
inline Vec3 inverseRotate(const Quaternion &a, const Vec3 &v)
{
    return rotate(conjugate(a), v);
}


// Rotation vector (axis-angle log map) of a quaternion: the rotation as a
// 3-vector whose direction is the rotation axis and whose length is the
// angle in radians. Small angles use the linearized form so the result is
// continuous at identity; the input normalizes internally, so unnormalized
// quaternions are accepted.
inline Vec3 rotationVectorRadians(const Quaternion &rotation)
{
    const Quaternion normalized = normalize(rotation);
    const double     sine       = std::sqrt(
        normalized.x * normalized.x + normalized.y * normalized.y + normalized.z * normalized.z);
    if (sine < 1.0e-9)
    {
        return Vec3{2.0 * normalized.x, 2.0 * normalized.y, 2.0 * normalized.z};
    }
    const double angle = 2.0 * std::atan2(sine, normalized.w);
    return Vec3{normalized.x / sine, normalized.y / sine, normalized.z / sine} * angle;
}

// Column-major 3x3 rotation matrix stored row-of-columns: element (row,
// column) at elements[row * 3 + column].
struct Mat3
{
    double elements[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

inline Mat3 makeRotationMatrix(const Quaternion &a)
{
    const double x  = a.x;
    const double y  = a.y;
    const double z  = a.z;
    const double w  = a.w;
    const double xx = x + x;
    const double yy = y + y;
    const double zz = z + z;
    const double xy = x * yy;
    const double xz = x * zz;
    const double xw = x * w;
    const double yz = y * zz;
    const double yw = y * w;
    const double zw = z * w;
    Mat3         matrix;
    matrix.elements[0] = 1.0 - (yy + zz);
    matrix.elements[1] = xy - zw;
    matrix.elements[2] = xz + yw;
    matrix.elements[3] = xy + zw;
    matrix.elements[4] = 1.0 - (xx + zz);
    matrix.elements[5] = yz - xw;
    matrix.elements[6] = xz - yw;
    matrix.elements[7] = yz + xw;
    matrix.elements[8] = 1.0 - (xx + yy);
    return matrix;
}

inline Vec3 multiplyMatrixVector(const Mat3 &m, const Vec3 &v)
{
    return Vec3{
        m.elements[0] * v.x + m.elements[1] * v.y + m.elements[2] * v.z,
        m.elements[3] * v.x + m.elements[4] * v.y + m.elements[5] * v.z,
        m.elements[6] * v.x + m.elements[7] * v.y + m.elements[8] * v.z};
}

// v^T * M, equivalently M^T * v for a symmetric tensor application.
inline Vec3 multiplyMatrixTransposedVector(const Mat3 &m, const Vec3 &v)
{
    return Vec3{
        m.elements[0] * v.x + m.elements[3] * v.y + m.elements[6] * v.z,
        m.elements[1] * v.x + m.elements[4] * v.y + m.elements[7] * v.z,
        m.elements[2] * v.x + m.elements[5] * v.y + m.elements[8] * v.z};
}

// Quaternion that rotates unit vector from onto unit vector to (shortest arc).
inline Quaternion makeRotationFromTo(const Vec3 &from, const Vec3 &to)
{
    const double cosine = dot(from, to);
    if (cosine > 1.0 - 1.0e-9)
    {
        return Quaternion{};
    }
    if (cosine < -(1.0 - 1.0e-9))
    {
        // Antiparallel: rotate by pi around any axis perpendicular to from.
        Vec3 axis = cross(from, Vec3{1.0, 0.0, 0.0});
        if (lengthSquared(axis) < 1.0e-9)
        {
            axis = cross(from, Vec3{0.0, 0.0, 1.0});
        }
        axis = normalized(axis);
        return Quaternion{axis.x, axis.y, axis.z, 0.0};
    }
    const Vec3 axis = cross(from, to);
    // q = [axisUnit * sin(theta/2), cos(theta/2)] with cos(theta/2) =
    // sqrt((1 + cos)/2) and sin(theta/2) = sqrt((1 - cos)/2); |axis| =
    // sin(theta) guarantees a non-degenerate axis here.
    const Vec3   axisUnit   = normalized(axis);
    const double halfSine   = std::sqrt(0.5 * (1.0 - cosine));
    const double halfCosine = std::sqrt(0.5 * (1.0 + cosine));
    return Quaternion{axisUnit.x * halfSine, axisUnit.y * halfSine, axisUnit.z * halfSine, halfCosine};
}

// Swing-twist decomposition of a unit quaternion along the unit axis: returns
// the twist component (rotation about the axis); the remainder is the swing.
inline Quaternion twistComponent(const Quaternion &rotation, const Vec3 &axis)
{
    const Vec3   vector{rotation.x, rotation.y, rotation.z};
    const double projection = dot(vector, axis);
    // Twist quaternion: axis * projection + w, then normalized.
    const double magnitude = std::sqrt(projection * projection + rotation.w * rotation.w);
    if (magnitude <= 1.0e-12)
    {
        return Quaternion{};
    }
    return Quaternion{
        axis.x * projection / magnitude,
        axis.y * projection / magnitude,
        axis.z * projection / magnitude,
        rotation.w / magnitude};
}

// Signed angle in [-pi, pi] of the twist component about the axis.
inline double twistAngleRadians(const Quaternion &rotation, const Vec3 &axis)
{
    const Quaternion twist = twistComponent(rotation, axis);
    double           angle = 2.0 * std::atan2(dot(Vec3{twist.x, twist.y, twist.z}, axis), twist.w);
    if (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    if (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

// Full swing angle (deviation of the rotation direction from the axis).
inline double swingAngleRadians(const Quaternion &rotation, const Vec3 &axis)
{
    const Quaternion twist = twistComponent(rotation, axis);
    const Quaternion swing = conjugate(twist) * rotation;
    return 2.0 * std::atan2(length(Vec3{swing.x, swing.y, swing.z}), swing.w);
}

} // namespace engine::physics_rigid::math
