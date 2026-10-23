#include "scene_description.hpp"

#include "scene_math.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace engine
{

namespace
{

bool parseVec3(
    ComponentContext &context, const std::string &path, std::array<float, 3> &out
)
{
    const std::vector<double> values = context.configurationFloatingArrayInSection("scene", path);
    if (values.size() != 3)
    {
        return false;
    }
    for (std::size_t index = 0; index < 3; ++index)
    {
        out[index] = static_cast<float>(values[index]);
    }
    return true;
}

} // namespace

bool SceneDescription::parseFromConfiguration(
    ComponentContext &context, SceneDescription &scene, std::string &error
)
{
    scene = SceneDescription{};

    if (const std::vector<double> sky = context.configurationFloatingArrayInSection("scene", "sky_radiance");
        sky.size() == 3)
    {
        for (std::size_t channel = 0; channel < 3; ++channel)
        {
            scene.skyRadiance[channel] = static_cast<float>(sky[channel]);
        }
    }

    // [scene.sky] gradient: unspecified components default to the flat
    // sky_radiance value, so legacy configurations render exactly as before.
    const auto parseSkyColor =
        [&context](const char *path, std::array<float, 3> &target, const std::array<float, 3> &fallback)
    {
        std::array<float, 3>      value      = fallback;
        const std::vector<double> configured = context.configurationFloatingArrayInSection("scene", path);
        if (configured.size() == 3)
        {
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                value[channel] = static_cast<float>(configured[channel]);
            }
        }
        target = value;
    };
    parseSkyColor("sky.zenith", scene.sky.zenith, scene.skyRadiance);
    parseSkyColor("sky.horizon", scene.sky.horizon, scene.skyRadiance);
    parseSkyColor("sky.ground", scene.sky.ground, scene.skyRadiance);
    scene.sky.sunDisk            = context.configurationBooleanInSection("scene", "sky.sun_disk").value_or(true);
    scene.sky.sunDiskSizeDegrees = static_cast<float>(
        context.configurationFloatingInSection("scene", "sky.sun_disk_size_degrees").value_or(0.6)
    );
    if (scene.sky.sunDiskSizeDegrees <= 0.0f || scene.sky.sunDiskSizeDegrees > 10.0f)
    {
        error = "scene sky sun_disk_size_degrees must be within (0, 10]";
        return false;
    }

    parseVec3(context, "camera.position", scene.camera.position);
    parseVec3(context, "camera.look_at", scene.camera.lookAt);

    // [scene.sun]: shared by the pbr technique's light state and the
    // environment's sun disk.
    parseVec3(context, "sun.direction", scene.sun.direction);
    parseVec3(context, "sun.color", scene.sun.color);
    if (const auto intensity = context.configurationFloatingInSection("scene", "sun.intensity"))
    {
        scene.sun.intensity = static_cast<float>(*intensity);
    }
    if (const auto fieldOfView = context.configurationFloatingInSection("scene", "camera.fov"))
    {
        scene.camera.verticalFieldOfViewDegrees = static_cast<float>(*fieldOfView);
    }
    if (const auto nearPlane = context.configurationFloatingInSection("scene", "camera.near"))
    {
        scene.camera.nearPlane = static_cast<float>(*nearPlane);
    }
    if (const auto farPlane = context.configurationFloatingInSection("scene", "camera.far"))
    {
        scene.camera.farPlane = static_cast<float>(*farPlane);
    }
    scene.camera.freeFly = context.configurationBooleanInSection("scene", "camera.free_fly").value_or(true);
    if (const auto speed = context.configurationFloatingInSection("scene", "camera.fly_speed"))
    {
        scene.camera.flySpeed = static_cast<float>(*speed);
    }
    if (const auto boost = context.configurationFloatingInSection("scene", "camera.fly_boost"))
    {
        scene.camera.flyBoost = static_cast<float>(*boost);
    }
    if (const auto sensitivity = context.configurationFloatingInSection("scene", "camera.look_sensitivity"))
    {
        scene.camera.lookSensitivity = static_cast<float>(*sensitivity);
    }
    if (scene.camera.flySpeed <= 0.0f || scene.camera.flyBoost < 1.0f || scene.camera.lookSensitivity <= 0.0f)
    {
        error = "scene camera fly controls need fly_speed > 0, fly_boost >= 1 and look_sensitivity > 0";
        return false;
    }

    const std::size_t objectCount = context.configurationArraySizeInSection("scene", "objects");
    if (objectCount > maxSceneObjects)
    {
        error = "scene configuration holds " + std::to_string(objectCount) + " objects; the limit is "
              + std::to_string(maxSceneObjects) + " (analytic SDF tracing cost is linear in the object count)";
        return false;
    }
    scene.objects.reserve(objectCount);
    for (std::size_t object = 0; object < objectCount; ++object)
    {
        const std::string prefix = "objects[" + std::to_string(object) + "]";
        SceneObject       parsed;
        const std::string typeName = context.configurationStringInSection("scene", prefix + ".type").value_or("box");
        if (typeName == "box" || typeName == "plane")
        {
            // "plane" is authoring sugar for a thin box: one primitive shape
            // per entry keeps the rasterized mesh and the analytic SDF of the
            // GI tracer exactly consistent.
            parsed.type = SceneObjectType::Box;
            if (typeName == "plane")
            {
                parsed.size[1] = 0.01f;
            }
        }
        else if (typeName == "sphere")
        {
            parsed.type = SceneObjectType::Sphere;
        }
        else
        {
            error = "scene object " + std::to_string(object) + " has unknown type '" + typeName
                  + "' (known: box, sphere, plane)";
            return false;
        }
        if (!parseVec3(context, prefix + ".position", parsed.position))
        {
            error = "scene object " + std::to_string(object) + " needs position = [x, y, z]";
            return false;
        }
        if (const std::vector<double> size = context.configurationFloatingArrayInSection("scene", prefix + ".size");
            size.size() == 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                parsed.size[axis] = static_cast<float>(size[axis]);
            }
        }
        if (const auto rotation = context.configurationFloatingInSection("scene", prefix + ".rotation_y"))
        {
            parsed.rotationYRadians = degreesToRadians(static_cast<float>(*rotation));
        }
        if (const std::vector<double> albedo = context.configurationFloatingArrayInSection("scene", prefix + ".albedo");
            albedo.size() == 3)
        {
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                parsed.albedo[channel] = static_cast<float>(albedo[channel]);
            }
        }
        if (const auto roughness = context.configurationFloatingInSection("scene", prefix + ".roughness"))
        {
            parsed.roughness = static_cast<float>(*roughness);
        }
        if (const auto metallic = context.configurationFloatingInSection("scene", prefix + ".metallic"))
        {
            parsed.metallic = static_cast<float>(*metallic);
        }
        if (const auto emissive = context.configurationFloatingInSection("scene", prefix + ".emissive_strength"))
        {
            parsed.emissiveStrength = static_cast<float>(*emissive);
        }
        parsed.textureSet = context.configurationStringInSection("scene", prefix + ".texture").value_or(std::string());
        if (const auto uvScale = context.configurationFloatingInSection("scene", prefix + ".uv_scale"))
        {
            parsed.uvScale = static_cast<float>(*uvScale);
        }
        if (parsed.uvScale <= 0.0f)
        {
            error = "scene object " + std::to_string(object) + " needs uv_scale > 0";
            return false;
        }
        scene.objects.push_back(parsed);
    }

    // Texture set registry: first-use order, so layer indices are stable and
    // independent of object order changes elsewhere.
    for (const SceneObject &object : scene.objects)
    {
        if (object.textureSet.empty())
        {
            continue;
        }
        const bool registered = std::any_of(
            scene.textureSets.begin(),
            scene.textureSets.end(),
            [&object](const std::string &registeredName) { return registeredName == object.textureSet; }
        );
        if (!registered)
        {
            scene.textureSets.push_back(object.textureSet);
        }
    }
    return true;
}

} // namespace engine
