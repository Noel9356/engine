// Pose math helpers shared by skeletal_animation translation units.
//
// Pure free functions operating on caller-owned arrays. The shared
// engine::math quaternion helpers are reused for normalization, dot product,
// vector rotation, and quaternion composition so this header stays small and
// consistent with the rest of the engine.

#ifndef ENGINE_SKELETAL_ANIMATION_MATH_UTILS_HPP
#define ENGINE_SKELETAL_ANIMATION_MATH_UTILS_HPP

#include <engine/math/quaternion.hpp>
#include <engine/skeletal_animation/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine::skeletal_animation
{

using engine::math::multiplyQuaternion;
using engine::math::normalizeQuaternion;
using engine::math::quaternionDot;
using engine::math::rotateVector;

using Pose = JointDescription;

// Sentinel stored in per-track sampling cursors when no interval is cached.
inline constexpr std::uint32_t invalidKeyframeCursor = std::numeric_limits<std::uint32_t>::max();

// Precomputed coefficients for one slerp interval between two adjacent
// keyframes. `signedAngle` is the angle between the (hemisphere-aligned)
// quaternions with the sign of the cosine; `inverseSine` is 1 / sin(angle).
// When `signedAngle` is zero the interval is treated as a no-op and the
// sampling path falls back to nlerp.
struct QuaternionInterval
{
    float signedAngle  = 0.0f;
    float inverseSine = 0.0f;
};

// Verifies a pose has finite translation/rotation/scale and a non-zero
// rotation quaternion. The rotation length check uses double precision to
// avoid spurious overflow for very large inputs.
inline bool finitePose(const Pose &pose) noexcept
{
    for (float value : pose.translation)
        if (!std::isfinite(value))
            return false;
    for (float value : pose.rotation)
        if (!std::isfinite(value))
            return false;
    for (float value : pose.scale)
        if (!std::isfinite(value))
            return false;

    const double lengthSquared = static_cast<double>(pose.rotation[0]) * pose.rotation[0]
        + static_cast<double>(pose.rotation[1]) * pose.rotation[1]
        + static_cast<double>(pose.rotation[2]) * pose.rotation[2]
        + static_cast<double>(pose.rotation[3]) * pose.rotation[3];
    return lengthSquared > 1.0e-16 && std::isfinite(lengthSquared);
}

// Normalized linear interpolation between two quaternions. The hemisphere of
// `b` is aligned to `a` so the result follows the shorter arc; the resulting
// quaternion is renormalized to compensate for the linear shortcut.
inline void nlerp(const float a[4], const float bInput[4], float t, float output[4]) noexcept
{
    const float sign = quaternionDot(a, bInput) < 0.0f ? -1.0f : 1.0f;
    for (std::uint32_t index = 0; index < 4; ++index)
        output[index] = a[index] + (bInput[index] * sign - a[index]) * t;
    normalizeQuaternion(output);
}

// Computes a weighted rotation delta in the same hemisphere as `deltaInput`
// and applies the angle-scaling form of quaternion exponentiation. For very
// small angles the result degrades to a linear scale of the imaginary part
// (with the real part pinned to 1) before renormalization, which matches the
// slerp short-arc convention used by the sampling path.
inline void weightedQuaternionDelta(const float deltaInput[4], float weight, float output[4]) noexcept
{
    float delta[4] = {deltaInput[0], deltaInput[1], deltaInput[2], deltaInput[3]};
    if (delta[3] < 0.0f)
    {
        for (float &value : delta)
            value = -value;
    }

    const float halfAngle = std::acos(std::clamp(delta[3], -1.0f, 1.0f));
    const float sine = std::sin(halfAngle);
    if (std::abs(sine) > 1.0e-6f)
    {
        const float factor = std::sin(halfAngle * weight) / sine;
        output[0] = delta[0] * factor;
        output[1] = delta[1] * factor;
        output[2] = delta[2] * factor;
        output[3] = std::cos(halfAngle * weight);
    }
    else
    {
        output[0] = delta[0] * weight;
        output[1] = delta[1] * weight;
        output[2] = delta[2] * weight;
        output[3] = 1.0f;
    }
    normalizeQuaternion(output);
}

// Builds a column-major 4x4 transform from a joint pose (TRS). The rotation is
// renormalized defensively; the matrix layout matches what `buildGlobalTransforms`
// and consumers expect for parent composition.
inline void makeTransformMatrix(const Pose &pose, float output[16]) noexcept
{
    float rotation[4] = {pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
    normalizeQuaternion(rotation);
    const float x = rotation[0];
    const float y = rotation[1];
    const float z = rotation[2];
    const float w = rotation[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - z * w);
    const float r02 = 2.0f * (x * z + y * w);
    const float r10 = 2.0f * (x * y + z * w);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - x * w);
    const float r20 = 2.0f * (x * z - y * w);
    const float r21 = 2.0f * (y * z + x * w);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    output[0] = r00 * pose.scale[0];
    output[1] = r10 * pose.scale[0];
    output[2] = r20 * pose.scale[0];
    output[3] = 0.0f;
    output[4] = r01 * pose.scale[1];
    output[5] = r11 * pose.scale[1];
    output[6] = r21 * pose.scale[1];
    output[7] = 0.0f;
    output[8] = r02 * pose.scale[2];
    output[9] = r12 * pose.scale[2];
    output[10] = r22 * pose.scale[2];
    output[11] = 0.0f;
    output[12] = pose.translation[0];
    output[13] = pose.translation[1];
    output[14] = pose.translation[2];
    output[15] = 1.0f;
}

// Multiplies two column-major 4x4 matrices: `output = left * right`. The
// result is computed into a temporary so `output` may alias either input.
inline void multiplyColumnMajorMatrices(const float left[16], const float right[16], float output[16]) noexcept
{
    float result[16];
    for (std::uint32_t column = 0; column < 4; ++column)
    {
        for (std::uint32_t row = 0; row < 4; ++row)
        {
            result[column * 4 + row] = left[row] * right[column * 4]
                + left[4 + row] * right[column * 4 + 1]
                + left[8 + row] * right[column * 4 + 2]
                + left[12 + row] * right[column * 4 + 3];
        }
    }
    std::copy(result, result + 16, output);
}

// Inverts a column-major 4x4 affine transform (TRS) analytically. The matrix
// is assumed to be the composition of a rotation, scale, and translation as
// produced by `makeTransformMatrix`; the inverse is computed in closed form
// using the rotation transpose and the negated, scale-adjusted translation.
// Returns false for degenerate (non-invertible) inputs.
inline bool invertAffineMatrix(const float matrix[16], float output[16]) noexcept
{
    // Extract the 3x3 rotation/scale block (column-major).
    const float a00 = matrix[0];
    const float a10 = matrix[1];
    const float a20 = matrix[2];
    const float a01 = matrix[4];
    const float a11 = matrix[5];
    const float a21 = matrix[6];
    const float a02 = matrix[8];
    const float a12 = matrix[9];
    const float a22 = matrix[10];

    const float determinant = a00 * (a11 * a22 - a21 * a12) - a01 * (a10 * a22 - a20 * a12)
        + a02 * (a10 * a21 - a20 * a11);
    if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-20f)
        return false;

    const float inverseDeterminant = 1.0f / determinant;

    // Adjugate of the 3x3 block (row-major adjugate transposed back to column major).
    const float b00 = (a11 * a22 - a21 * a12) * inverseDeterminant;
    const float b10 = -(a10 * a22 - a20 * a12) * inverseDeterminant;
    const float b20 = (a10 * a21 - a20 * a11) * inverseDeterminant;
    const float b01 = -(a01 * a22 - a21 * a02) * inverseDeterminant;
    const float b11 = (a00 * a22 - a20 * a02) * inverseDeterminant;
    const float b21 = -(a00 * a21 - a20 * a01) * inverseDeterminant;
    const float b02 = (a01 * a12 - a11 * a02) * inverseDeterminant;
    const float b12 = -(a00 * a12 - a10 * a02) * inverseDeterminant;
    const float b22 = (a00 * a11 - a10 * a01) * inverseDeterminant;

    const float t0 = matrix[12];
    const float t1 = matrix[13];
    const float t2 = matrix[14];

    output[0] = b00;
    output[1] = b10;
    output[2] = b20;
    output[3] = 0.0f;
    output[4] = b01;
    output[5] = b11;
    output[6] = b21;
    output[7] = 0.0f;
    output[8] = b02;
    output[9] = b12;
    output[10] = b22;
    output[11] = 0.0f;
    // Inverse translation: -R^-1 * t.
    output[12] = -(b00 * t0 + b01 * t1 + b02 * t2);
    output[13] = -(b10 * t0 + b11 * t1 + b12 * t2);
    output[14] = -(b20 * t0 + b21 * t1 + b22 * t2);
    output[15] = 1.0f;
    return true;
}

// Copies a keyframe's TRS values into a pose, attaching the parent index.
inline void copyKeyframe(const AnimationKeyframe &key, std::int32_t parentIndex, Pose &output) noexcept
{
    output.parentIndex = parentIndex;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        output.translation[axis] = key.translation[axis];
        output.scale[axis]       = key.scale[axis];
    }
    for (std::uint32_t axis = 0; axis < 4; ++axis)
        output.rotation[axis] = key.rotation[axis];
}

// Interpolates between two keyframes into a pose. `t` is the normalized
// interval fraction. Slerp uses the precomputed `rotationInterval` to skip
// the per-sample acos/sin work; nlerp and step bypass it. `t <= 0` returns the
// left keyframe and `t >= 1` returns the right keyframe, which keeps boundary
// sampling stable without a redundant epsilon check at the call site.
inline void interpolateKeyframes(const AnimationKeyframe &left, const AnimationKeyframe &right, float t,
    AnimationInterpolation interpolation, const QuaternionInterval *rotationInterval, std::int32_t parentIndex,
    Pose &output) noexcept
{
    if (interpolation == AnimationInterpolation::Step || t <= 0.0f)
    {
        copyKeyframe(left, parentIndex, output);
        return;
    }
    if (t >= 1.0f)
    {
        copyKeyframe(right, parentIndex, output);
        return;
    }

    output.parentIndex = parentIndex;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        output.translation[axis] = left.translation[axis] + (right.translation[axis] - left.translation[axis]) * t;
        output.scale[axis]       = left.scale[axis] + (right.scale[axis] - left.scale[axis]) * t;
    }

    if (interpolation == AnimationInterpolation::Nlerp)
    {
        nlerp(left.rotation, right.rotation, t, output.rotation);
    }
    else if (rotationInterval == nullptr || rotationInterval->signedAngle == 0.0f)
    {
        nlerp(left.rotation, right.rotation, t, output.rotation);
    }
    else
    {
        const float angle = std::abs(rotationInterval->signedAngle);
        const float leftWeight = std::sin((1.0f - t) * angle) * rotationInterval->inverseSine;
        const float rightWeight = std::sin(t * rotationInterval->signedAngle) * rotationInterval->inverseSine;
        for (std::uint32_t axis = 0; axis < 4; ++axis)
            output.rotation[axis] = left.rotation[axis] * leftWeight + right.rotation[axis] * rightWeight;
    }
}

} // namespace engine::skeletal_animation

#endif
