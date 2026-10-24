#include "asset.hpp"

#include "gltf_import.hpp"
#include "engine/kernel/filesystem_utilities.hpp"

#include <fstream>
#include <sstream>

namespace engine
{

AssetModule::AssetModule()
{
    componentManifest.name = "asset";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"asset.gltf"};
    // The scene registry is the import target (hard edge); reading content
    // files is the importer's whole job, so the read permission is declared.
    componentManifest.requiredCapabilities = {"logging", "scene"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.scene", "filesystem.read"};
}

AssetModule::~AssetModule()
{
    stop();
}

void AssetModule::construct(
    ComponentContext &context
)
{
    boundContext    = &context;
    logger          = &context.acquireCapability<LoggerCapability>();
    sceneCapability = &context.acquireCapability<SceneCapability>();
}

void AssetModule::start()
{
    logger->logMessage(LogLevel::Information, "asset module started");
}

void AssetModule::stop()
{
    boundContext    = nullptr;
    logger          = nullptr;
    sceneCapability = nullptr;
}

void *AssetModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "asset.gltf")
    {
        return static_cast<AssetCapability *>(this);
    }
    return nullptr;
}

std::uint32_t AssetModule::importGltfIntoScene(
    const std::string &path, std::uint32_t fidelity, std::string &errorReport
)
{
    // Read the file. Relative paths resolve against the executable directory
    // (the engine's path convention), which is where the runtime layout puts
    // mod packages - so "mods/my_mod/assets/level.gltf" just works from a
    // script.
    const std::filesystem::path resolved =
        std::filesystem::path(path).is_relative() ? (executableDirectory() / path)
                                                  : std::filesystem::path(path);
    std::error_code  fileError;
    const bool       exists = std::filesystem::is_regular_file(resolved, fileError);
    if (!exists)
    {
        errorReport = "asset module: '" + path + "' does not name a readable file (resolved '" + resolved.string()
                    + "')";
        return 0;
    }
    // The .glb container wraps the JSON in a binary header; this importer
    // speaks the JSON form only, and refusing precisely beats a garbled
    // parse of the binary section.
    if (resolved.extension() == ".glb")
    {
        errorReport = "asset module: '" + path
                    + "' is a .glb container; import the .gltf JSON form (exporters offer both)";
        return 0;
    }

    std::ifstream file(resolved, std::ios::binary);
    if (!file)
    {
        errorReport = "asset module: failed to open '" + resolved.string() + "'";
        return 0;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    std::string              parseError;
    std::unique_ptr<JsonValue> document = jsonParse(text, parseError);
    if (document == nullptr)
    {
        errorReport = "asset module: '" + path + "' is not valid JSON: " + parseError;
        return 0;
    }

    if (fidelity != asset_fidelity::bounding_proxies && fidelity != asset_fidelity::full_mesh)
    {
        errorReport = "asset module: unknown import fidelity " + std::to_string(fidelity)
                    + " (0 = bounding proxies, 1 = full mesh)";
        return 0;
    }

    std::uint32_t spawned =
        gltfImportIntoScene(*document, *sceneCapability, resolved.filename().string(), fidelity, errorReport);
    if (!errorReport.empty())
    {
        errorReport = "asset module: " + errorReport;
        return 0;
    }
    logger->logMessage(
        LogLevel::Information,
        "asset module: imported " + std::to_string(spawned) + " object(s) from '" + path + "'"
    );
    return spawned;
}

} // namespace engine
