#ifndef ENGINE_MODULES_RENDERER_SCENE_MATH_HPP
#define ENGINE_MODULES_RENDERER_SCENE_MATH_HPP

// Minimal column-major 4x4 math for the scene camera and object transforms.
// Hand rolled instead of pulling a linear algebra dependency: the renderer
// needs perspective, lookAt, multiply and inverse, nothing else, and the
// storage layout (float[16], column major) is exactly what the shaders
// consume - no conversion step, no second source of truth.

#include <array>
#include <cmath>

namespace engine
{

// Shared angle constants: one definition because the C++ sites previously
// inlined the literals at two different precisions.
inline constexpr float pi    = 3.14159265358979323846f;
inline constexpr float twoPi = 2.0f * pi;

inline constexpr float degreesToRadians(
    float degrees
)
{
    return degrees * (pi / 180.0f);
}

using Mat4 = std::array<float, 16>; // column major: m[col * 4 + row]

inline Mat4 multiplyMatrix(
    const Mat4 &left, const Mat4 &right
)
{
    Mat4 result{};
    for (std::size_t column = 0; column < 4; ++column)
    {
        for (std::size_t row = 0; row < 4; ++row)
        {
            float sum = 0.0f;
            for (std::size_t inner = 0; inner < 4; ++inner)
            {
                sum += left[inner * 4 + row] * right[column * 4 + inner];
            }
            result[column * 4 + row] = sum;
        }
    }
    return result;
}

// Vulkan convention: depth maps to [0, 1], Y points down in clip space.
inline Mat4 perspectiveMatrix(
    float verticalFieldOfViewRadians, float aspect, float nearPlane, float farPlane
)
{
    const float focal = 1.0f / std::tan(verticalFieldOfViewRadians * 0.5f);
    Mat4        result{};
    result[0]  = focal / aspect;
    result[5]  = -focal;
    result[10] = farPlane / (nearPlane - farPlane);
    result[11] = -1.0f;
    result[14] = nearPlane * farPlane / (nearPlane - farPlane);
    return result;
}

inline Mat4 lookAtMatrix(
    const float eye[3], const float center[3], const float up[3]
)
{
    float       forward[3]    = {center[0] - eye[0], center[1] - eye[1], center[2] - eye[2]};
    const float forwardLength = std::sqrt(forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2]);
    for (float &component : forward)
    {
        component /= forwardLength;
    }
    float right[3] = {
        forward[1] * up[2] - forward[2] * up[1],
        forward[2] * up[0] - forward[0] * up[2],
        forward[0] * up[1] - forward[1] * up[0]
    };
    const float rightLength = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    for (float &component : right)
    {
        component /= rightLength;
    }
    const float cameraUp[3] = {
        right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0]
    };

    // View matrix: basis vectors in the rows of the upper 3x3, translation in
    // the last column.
    Mat4 result{};
    result[0]  = right[0];
    result[4]  = right[1];
    result[8]  = right[2];
    result[12] = -(right[0] * eye[0] + right[1] * eye[1] + right[2] * eye[2]);
    result[1]  = cameraUp[0];
    result[5]  = cameraUp[1];
    result[9]  = cameraUp[2];
    result[13] = -(cameraUp[0] * eye[0] + cameraUp[1] * eye[1] + cameraUp[2] * eye[2]);
    result[2]  = -forward[0];
    result[6]  = -forward[1];
    result[10] = -forward[2];
    result[14] = (forward[0] * eye[0] + forward[1] * eye[1] + forward[2] * eye[2]);
    result[15] = 1.0f;
    return result;
}

// General 4x4 inverse through Gauss-Jordan elimination with partial pivoting.
// Straightforward and obviously correct; called twice per frame at most, so
// the asymptotically faster adjugate expansion is not worth its index risk.
inline Mat4 inverseMatrix(
    const Mat4 &matrix
)
{
    // Work on a row-major augmented copy: [matrix | identity].
    float augmented[4][8] = {};
    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 4; ++column)
        {
            augmented[row][column] = matrix[column * 4 + row];
        }
        augmented[row][4 + row] = 1.0f;
    }

    for (std::size_t column = 0; column < 4; ++column)
    {
        // Partial pivot: swap the row with the largest absolute value in.
        std::size_t pivotRow       = column;
        float       pivotMagnitude = std::abs(augmented[column][column]);
        for (std::size_t candidate = column + 1; candidate < 4; ++candidate)
        {
            if (std::abs(augmented[candidate][column]) > pivotMagnitude)
            {
                pivotMagnitude = std::abs(augmented[candidate][column]);
                pivotRow       = candidate;
            }
        }
        float pivotRowStorage[8];
        for (std::size_t index = 0; index < 8; ++index)
        {
            pivotRowStorage[index]     = augmented[column][index];
            augmented[column][index]   = augmented[pivotRow][index];
            augmented[pivotRow][index] = pivotRowStorage[index];
        }
        const float pivot = augmented[column][column];
        for (std::size_t index = 0; index < 8; ++index)
        {
            augmented[column][index] /= pivot;
        }
        for (std::size_t row = 0; row < 4; ++row)
        {
            if (row == column)
            {
                continue;
            }
            const float factor = augmented[row][column];
            if (factor == 0.0f)
            {
                continue;
            }
            for (std::size_t index = 0; index < 8; ++index)
            {
                augmented[row][index] -= factor * augmented[column][index];
            }
        }
    }

    Mat4 result{};
    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 4; ++column)
        {
            result[column * 4 + row] = augmented[row][4 + column];
        }
    }
    return result;
}

// Translation * rotationY * scale.
inline Mat4 modelMatrix(
    const float position[3], float rotationYRadians, const float scale[3]
)
{
    const float s = std::sin(rotationYRadians);
    const float c = std::cos(rotationYRadians);
    Mat4        result{};
    result[0]  = c * scale[0];
    result[1]  = 0.0f;
    result[2]  = -s * scale[0];
    result[3]  = 0.0f;
    result[4]  = 0.0f;
    result[5]  = scale[1];
    result[6]  = 0.0f;
    result[7]  = 0.0f;
    result[8]  = s * scale[2];
    result[9]  = 0.0f;
    result[10] = c * scale[2];
    result[11] = 0.0f;
    result[12] = position[0];
    result[13] = position[1];
    result[14] = position[2];
    result[15] = 1.0f;
    return result;
}

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_SCENE_MATH_HPP
