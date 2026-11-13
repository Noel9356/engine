// Shared math and string helpers of the character module.
//
// Small, dependency-free utilities used by the registry, the validation
// pass, the evaluation walk, and the recipe serializer. Everything here
// assumes the module mutex is held by the caller.

#ifndef ENGINE_CHARACTER_MATH_UTILS_HPP
#define ENGINE_CHARACTER_MATH_UTILS_HPP

#include "records.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <string>

namespace engine::character
{
namespace internal
{

inline bool finiteFloat(float value)
{
    return std::isfinite(value);
}

inline bool finiteVector3(const float *values)
{
    return values != nullptr && finiteFloat(values[0]) && finiteFloat(values[1]) && finiteFloat(values[2]);
}

inline bool finiteQuaternion(const float *values)
{
    return values != nullptr && finiteFloat(values[0]) && finiteFloat(values[1]) && finiteFloat(values[2])
        && finiteFloat(values[3]);
}

// A quaternion that carries rotation information (rejects the zero quat
// before normalization would divide by ~zero).
inline bool nonZeroQuaternion(const float *values)
{
    const float squaredLength
        = values[0] * values[0] + values[1] * values[1] + values[2] * values[2] + values[3] * values[3];
    return squaredLength > 1.0e-8f;
}

inline glm::vec3 copyVector3(const float *values)
{
    return glm::vec3(values[0], values[1], values[2]);
}

inline glm::quat copyQuaternion(const float *values)
{
    glm::quat   rotation(values[3], values[0], values[1], values[2]);
    const float length = glm::length(rotation);
    if (length > 1.0e-8f)
    {
        rotation = rotation / length;
    }
    else
    {
        rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    return rotation;
}

inline std::string copyString(const char *text)
{
    return text != nullptr ? std::string(text) : std::string();
}

inline bool stringNonEmpty(const char *text)
{
    return text != nullptr && text[0] != '\0';
}

// TRS matrix helper shared with the serializer.
inline glm::mat4 makeTrsMatrix(const glm::vec3 &translation, const glm::quat &rotation)
{
    return glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);
}

// Parameter-scale clamp bounds applied to the composed per-part scale.
inline constexpr float kMinimumPartScale = 0.01f;
inline constexpr float kMaximumPartScale = 100.0f;

} // namespace internal
} // namespace engine::character

#endif
