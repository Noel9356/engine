#include "persistence.hpp"

#include <toml.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "engine/kernel/filesystem_utilities.hpp"

namespace engine
{

namespace
{

// One field per object row, named exactly like the [scene] section's keys so
// a save file reads as documentation and a hand-tuned save behaves like a
// hand-tuned scene (the row schema is the [scene.objects] schema plus the
// mesh name; the [scene] header comment documents the same fields).
const char *objectTypeToString(std::uint32_t type)
{
    switch (type)
    {
        case scene_object_type::sphere:
            return "sphere";
        case scene_object_type::mesh:
            return "mesh";
        case scene_object_type::box:
        default:
            return "box";
    }
}

toml::array floatArray(
    const std::array<float, 3> &values
)
{
    toml::array array;
    for (const float value : values)
    {
        array.push_back(static_cast<double>(value));
    }
    return array;
}

std::array<float, 3> floatArray3(
    const toml::table &row, const char *key, const std::array<float, 3> &fallback
)
{
    const toml::node *value = row.get(key);
    if (value == nullptr || !value->is_array())
    {
        return fallback;
    }
    const toml::array *array = value->as_array();
    if (array == nullptr || array->size() < 3)
    {
        return fallback;
    }
    return {
        static_cast<float>(array->get(0)->value_or(fallback[0])),
        static_cast<float>(array->get(1)->value_or(fallback[1])),
        static_cast<float>(array->get(2)->value_or(fallback[2])),
    };
}

std::filesystem::path resolveAgainstExecutable(
    const std::string &path
)
{
    const std::filesystem::path candidate(path);
    return candidate.is_relative() ? (executableDirectory() / candidate) : candidate;
}

} // namespace

PersistenceModule::PersistenceModule()
{
    componentManifest.name = "persistence";
    componentManifest.kind = ComponentKind::Module;
    // The implementation-bound member of the "persistence" family: a future
    // binary or streamed-world save format declares its own member and the
    // [providers] configuration picks between them.
    componentManifest.providedCapabilities = {"persistence.toml"};
    componentManifest.requiredCapabilities = {"logging", "scene"};
    // Saving writes a file (filesystem.write); loading reads one
    // (filesystem.read). Both are deniable in configuration like every grant.
    componentManifest.requiredPermissions  = {"capability.logging", "capability.scene", "filesystem.write", "filesystem.read"};
}

PersistenceModule::~PersistenceModule()
{
    stop();
}

void PersistenceModule::construct(
    ComponentContext &context
)
{
    boundContext    = &context;
    logger          = &context.acquireCapability<LoggerCapability>();
    sceneCapability = &context.acquireCapability<SceneCapability>();
}

void PersistenceModule::start()
{
    logger->logMessage(LogLevel::Information, "persistence module started");
}

void PersistenceModule::stop()
{
    boundContext    = nullptr;
    logger          = nullptr;
    sceneCapability = nullptr;
}

void *PersistenceModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "persistence.toml")
    {
        return static_cast<PersistenceCapability *>(this);
    }
    return nullptr;
}

bool PersistenceModule::saveScene(
    const std::string &path, std::string &errorReport
)
{
    const std::vector<SceneObjectDescriptor> snapshot = sceneCapability->objectSnapshot();
    toml::table              file;
    file.emplace("saved_objects", static_cast<std::int64_t>(snapshot.size()));

    toml::array objects;
    for (const SceneObjectDescriptor &object : snapshot)
    {
        toml::table row;
        row.emplace("type", objectTypeToString(object.type));
        row.emplace("position", floatArray(object.position));
        // Box: half extents; sphere: radius in x; mesh: the bbox half extents
        // (recomputed from the registered mesh at load time - the row's size
        // documents the proxy, the registry's mesh data is authoritative).
        row.emplace("size", floatArray(object.size));
        row.emplace("rotation_y", static_cast<double>(object.rotationYRadians));
        row.emplace("albedo", floatArray(object.albedo));
        row.emplace("roughness", static_cast<double>(object.roughness));
        row.emplace("metallic", static_cast<double>(object.metallic));
        row.emplace("emissive_strength", static_cast<double>(object.emissiveStrength));
        if (!object.textureSet.empty())
        {
            row.emplace("texture", object.textureSet);
        }
        row.emplace("uv_scale", static_cast<double>(object.uvScale));
        if (object.type == scene_object_type::mesh)
        {
            const std::string meshName = sceneCapability->meshName(object.meshHandle);
            if (meshName.empty())
            {
                errorReport = "persistence module: object " + std::to_string(object.meshHandle)
                            + " instances a mesh the registry does not know; save refused";
                return false;
            }
            row.emplace("mesh", meshName);
        }
        objects.push_back(std::move(row));
    }
    file.emplace("objects", std::move(objects));

    const std::filesystem::path resolved = resolveAgainstExecutable(path);
    std::error_code             directoryError;
    std::filesystem::create_directories(resolved.parent_path(), directoryError);
    std::ofstream output(resolved, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        errorReport = "persistence module: cannot open '" + resolved.string() + "' for writing"
                    + " (is filesystem.write granted to this caller's component?)";
        return false;
    }
    output << "# Saved by the engine's persistence module: " << snapshot.size() << " object(s).\n"
           << "# Loading spawns these as NEW registry objects (handles are runtime values).\n"
           << "# Mesh rows reference registry meshes by NAME - the mesh must be registered\n"
           << "# (or re-imported) before load. The row schema is the [scene.objects] schema.\n"
           << file << "\n";
    if (!output)
    {
        errorReport = "persistence module: writing '" + resolved.string() + "' failed";
        return false;
    }
    logger->logMessage(
        LogLevel::Information,
        "persistence module: saved " + std::to_string(snapshot.size()) + " object(s) to '" + path + "'"
    );
    return true;
}

std::uint32_t PersistenceModule::loadScene(
    const std::string &path, std::string &errorReport
)
{
    const std::filesystem::path resolved = resolveAgainstExecutable(path);
    std::error_code             fileError;
    if (!std::filesystem::is_regular_file(resolved, fileError))
    {
        errorReport = "persistence module: '" + path + "' does not name a readable file (resolved '"
                    + resolved.string() + "')";
        return 0;
    }
    toml::parse_result parsed;
    try
    {
        parsed = toml::parse_file(resolved.string());
    }
    catch (const toml::parse_error &error)
    {
        std::ostringstream description;
        description << "persistence module: '" << path << "' is not valid TOML: " << error.description()
                    << " (line " << error.source().begin.line << ")";
        errorReport = description.str();
        return 0;
    }

    const toml::node *objectsNode = parsed.get("objects");
    if (objectsNode == nullptr || !objectsNode->is_array())
    {
        errorReport = "persistence module: '" + path + "' defines no [[objects]] array - nothing to load";
        return 0;
    }
    const toml::array *objects = objectsNode->as_array();
    if (objects == nullptr)
    {
        errorReport = "persistence module: '" + path + "'s objects array is not an array of tables";
        return 0;
    }

    std::uint32_t spawned = 0;
    for (const toml::node &entry : *objects)
    {
        const toml::table *row = entry.as_table();
        if (row == nullptr)
        {
            continue;
        }
        SceneObjectDescriptor object;
        const std::string     type = row->get("type") != nullptr ? row->get("type")->value_or(std::string("box")) : "box";
        if (type == "sphere")
        {
            object.type = scene_object_type::sphere;
        }
        else if (type == "mesh")
        {
            const std::string meshName =
                row->get("mesh") != nullptr ? row->get("mesh")->value_or(std::string()) : std::string();
            if (meshName.empty())
            {
                if (logger != nullptr)
                {
                    logger->logMessage(
                        LogLevel::Warning,
                        "persistence module: '" + path + "' has a mesh row without a mesh name; row skipped"
                    );
                }
                continue;
            }
            const SceneCapability::MeshHandle handle = sceneCapability->meshHandle(meshName);
            if (handle == SceneCapability::invalidMesh)
            {
                if (logger != nullptr)
                {
                    logger->logMessage(
                        LogLevel::Warning,
                        "persistence module: mesh '" + meshName
                            + "' is not registered (import it first); row skipped"
                    );
                }
                continue;
            }
            object.type       = scene_object_type::mesh;
            object.meshHandle = handle;
        }
        object.position         = floatArray3(*row, "position", object.position);
        object.size             = floatArray3(*row, "size", object.size);
        object.rotationYRadians = static_cast<float>(row->get("rotation_y") != nullptr
                                                         ? row->get("rotation_y")->value_or(0.0)
                                                         : 0.0);
        object.albedo           = floatArray3(*row, "albedo", object.albedo);
        object.roughness        = static_cast<float>(
            row->get("roughness") != nullptr ? row->get("roughness")->value_or(object.roughness) : object.roughness);
        object.metallic        = static_cast<float>(
            row->get("metallic") != nullptr ? row->get("metallic")->value_or(object.metallic) : object.metallic);
        object.emissiveStrength = static_cast<float>(row->get("emissive_strength") != nullptr
                                                         ? row->get("emissive_strength")->value_or(object.emissiveStrength)
                                                         : object.emissiveStrength);
        object.textureSet =
            row->get("texture") != nullptr ? row->get("texture")->value_or(std::string()) : std::string();
        object.uvScale =
            static_cast<float>(row->get("uv_scale") != nullptr ? row->get("uv_scale")->value_or(object.uvScale)
                                                               : object.uvScale);
        if (sceneCapability->addObject(object) == SceneCapability::invalidObject)
        {
            // The registry is full (its refusal is logged precisely there):
            // report what arrived, like the asset importer.
            return spawned;
        }
        ++spawned;
    }
    logger->logMessage(
        LogLevel::Information,
        "persistence module: loaded " + std::to_string(spawned) + " object(s) from '" + path + "'"
    );
    return spawned;
}

bool PersistenceModule::clearScene(
    std::string &errorReport
)
{
    (void) errorReport;
    // Meshes stay: they are assets shared by instances, and a load that
    // follows typically re-references them by name. Handles are read in one
    // ascending pass and removed after (removal never reuses handles, and a
    // live iteration over a mutating map would be the wrong shape here).
    const std::vector<SceneCapability::ObjectHandle> handles = sceneCapability->objectHandles();
    for (const SceneCapability::ObjectHandle handle : handles)
    {
        sceneCapability->removeObject(handle);
    }
    logger->logMessage(
        LogLevel::Information,
        "persistence module: scene registry cleared (" + std::to_string(handles.size()) + " object(s) removed)"
    );
    return true;
}

} // namespace engine
