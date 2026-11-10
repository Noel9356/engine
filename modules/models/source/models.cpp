// models module implementation: the ModuleInterface binding, configuration
// handling, and the "models" capability (load, inspect, destroy loaded models).
//
// Format parsing lives in the per-format importers and mesh optimization in
// model_optimizer.cpp; this file owns the model registry and turns ModelRecords
// into renderer::MeshDescription views for consumers.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/models/capabilities.hpp>

#include "model_import_internal.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <optional>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{
namespace models
{
namespace
{

using detail::makeTransform;
using detail::maximumTransformMagnitude;
using detail::ModelMaterialRecord;
using detail::ModelPart;
using detail::ModelRecord;
using detail::ModelTransform;
using detail::reportError;

constexpr std::size_t maximumModelPathLength = 16384;

std::size_t boundedStringLength(const char *value, std::size_t maximum) noexcept
{
    if (value == nullptr)
    {
        return 0;
    }
    std::size_t length = 0;
    while (length < maximum && value[length] != '\0')
    {
        ++length;
    }
    return length;
}

// Module implementation. Owns every loaded model (handle -> record), the
// configured default options, and the capability struct handed to consumers.
class ModelsModule final : public ModuleInterface
{
public:
    ModelsModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads the [models] defaults once and watches for configuration reloads.
    void construct(ParticipantContext &context) override
    {
        context_ = &context;
        readConfiguration();
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<ModelsModule, &ModelsModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    // Drops the reload subscription and every loaded model; all outstanding
    // MeshDescription pointers handed out via part() become invalid.
    void stop() override
    {
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        models_.clear();
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == ModelsCapability::capabilityName)
        {
            return &capability_;
        }
        return nullptr;
    }

private:
    // Reloads every [models] key into the module defaults. Values outside their
    // validated range are ignored so the built-in default stays active; only
    // normal_mode and overdraw_threshold additionally print a diagnostic.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;
        defaultOptions_             = ModelLoadOptions{};
        maximumFileSizeBytes_       = 512ull * bytesPerMegabyte;
        maximumLoadedModels_        = 256;
        optimizationOptions_        = detail::ModelOptimizationOptions{};
        materialSearchPath_.clear();

        // models.search_paths: directories probed in order for relative model paths.
        searchPaths_ = context.configurationStringArrayInSection("models", "search_paths");

        // models.scale: per-axis factor; each kept component must be finite, |v| <= 1e12.
        const std::vector<double> scale = context.configurationFloatingArrayInSection("models", "scale");
        if (scale.size() >= 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (std::isfinite(scale[axis]) && std::abs(scale[axis]) <= maximumTransformMagnitude)
                {
                    defaultOptions_.scale[axis] = static_cast<float>(scale[axis]);
                }
            }
        }
        // models.translation: post-transform offset; same per-component validation.
        const std::vector<double> translation = context.configurationFloatingArrayInSection("models", "translation");
        if (translation.size() >= 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (std::isfinite(translation[axis]) && std::abs(translation[axis]) <= maximumTransformMagnitude)
                {
                    defaultOptions_.translation[axis] = static_cast<float>(translation[axis]);
                }
            }
        }
        // models.pivot: point positions are rotated/scaled around; same validation.
        const std::vector<double> pivot = context.configurationFloatingArrayInSection("models", "pivot");
        if (pivot.size() >= 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (std::isfinite(pivot[axis]) && std::abs(pivot[axis]) <= maximumTransformMagnitude)
                {
                    defaultOptions_.pivot[axis] = static_cast<float>(pivot[axis]);
                }
            }
        }
        // models.uv_scale / uv_offset: 2-component UV transform, validated like scale.
        const std::vector<double> uvScale = context.configurationFloatingArrayInSection("models", "uv_scale");
        if (uvScale.size() >= 2)
        {
            for (std::size_t axis = 0; axis < 2; ++axis)
            {
                if (std::isfinite(uvScale[axis]) && std::abs(uvScale[axis]) <= maximumTransformMagnitude)
                {
                    defaultOptions_.uvScale[axis] = static_cast<float>(uvScale[axis]);
                }
            }
        }
        const std::vector<double> uvOffset = context.configurationFloatingArrayInSection("models", "uv_offset");
        if (uvOffset.size() >= 2)
        {
            for (std::size_t axis = 0; axis < 2; ++axis)
            {
                if (std::isfinite(uvOffset[axis]) && std::abs(uvOffset[axis]) <= maximumTransformMagnitude)
                {
                    defaultOptions_.uvOffset[axis] = static_cast<float>(uvOffset[axis]);
                }
            }
        }
        // models.rotation: unit quaternion (x y z w). Rejected as a whole unless all
        // four components are finite and the length exceeds 1e-8, then normalized.
        const std::vector<double> rotation = context.configurationFloatingArrayInSection("models", "rotation");
        if (rotation.size() >= 4)
        {
            bool validRotation = true;
            for (std::size_t component = 0; component < 4; ++component)
            {
                validRotation = validRotation && std::isfinite(rotation[component])
                             && std::abs(rotation[component]) <= std::numeric_limits<float>::max();
            }
            const double rotationLength
                = std::hypot(std::hypot(rotation[0], rotation[1]), std::hypot(rotation[2], rotation[3]));
            if (validRotation && std::isfinite(rotationLength) && rotationLength > 1e-8)
            {
                for (std::size_t component = 0; component < 4; ++component)
                {
                    defaultOptions_.rotation[component] = static_cast<float>(rotation[component] / rotationLength);
                }
            }
        }
        // models.normal_mode: preserve_smooth_missing | preserve | smooth | flat.
        const std::optional<std::string> normalMode = context.configurationStringInSection("models", "normal_mode");
        if (normalMode.has_value())
        {
            if (*normalMode == "preserve_smooth_missing")
            {
                defaultOptions_.normalMode = ModelNormalMode::PreserveOrGenerateSmooth;
            }
            else if (*normalMode == "preserve")
            {
                defaultOptions_.normalMode = ModelNormalMode::PreserveOnly;
            }
            else if (*normalMode == "smooth")
            {
                defaultOptions_.normalMode = ModelNormalMode::GenerateSmooth;
            }
            else if (*normalMode == "flat")
            {
                defaultOptions_.normalMode = ModelNormalMode::GenerateFlat;
            }
            else
            {
                std::fprintf(stderr,
                    "models module: unknown normal_mode '%s'; using preserve_smooth_missing\n",
                    normalMode->c_str());
            }
        }
        // The remaining boolean keys keep their built-in default when absent;
        // no value validation is needed beyond presence.
        if (const auto value = context.configurationBooleanInSection("models", "flip_texture_v"))
        {
            defaultOptions_.flipTextureV = *value;
        }
        if (const auto value = context.configurationBooleanInSection("models", "import_vertex_colors"))
        {
            defaultOptions_.importVertexColors = *value;
        }
        if (const auto value = context.configurationBooleanInSection("models", "weld_vertices"))
        {
            defaultOptions_.weldVertices = *value;
        }
        if (const auto value = context.configurationBooleanInSection("models", "import_materials"))
        {
            defaultOptions_.importMaterials = *value;
        }
        if (const auto value = context.configurationBooleanInSection("models", "optimize_mesh"))
        {
            defaultOptions_.optimizeMesh = *value;
        }
        // Optimization pass switches default to true even when the key is absent.
        optimizationOptions_.vertexCache
            = context.configurationBooleanInSection("models", "optimize_vertex_cache").value_or(true);
        optimizationOptions_.overdraw
            = context.configurationBooleanInSection("models", "optimize_overdraw").value_or(true);
        optimizationOptions_.vertexFetch
            = context.configurationBooleanInSection("models", "optimize_vertex_fetch").value_or(true);
        // models.overdraw_threshold: finite value in 1.0..2.0; outside the range a
        // warning is printed and 1.05 stays active.
        if (const auto threshold = context.configurationFloatingInSection("models", "overdraw_threshold"))
        {
            if (std::isfinite(*threshold) && *threshold >= 1.0 && *threshold <= 2.0)
            {
                optimizationOptions_.overdrawThreshold = static_cast<float>(*threshold);
            }
            else
            {
                std::fprintf(stderr,
                    "models module: ignoring [models] overdraw_threshold; expected a finite value in 1.0..2.0\n");
            }
        }
        // models.format: auto | obj | stl | ply | off | gltf | glb. Unknown names are
        // silently ignored, leaving "auto" (importer chosen from the extension).
        if (const auto value = context.configurationStringInSection("models", "format"))
        {
            if (*value == "obj")
            {
                defaultOptions_.format = ModelFormat::Obj;
            }
            else if (*value == "stl")
            {
                defaultOptions_.format = ModelFormat::Stl;
            }
            else if (*value == "ply")
            {
                defaultOptions_.format = ModelFormat::Ply;
            }
            else if (*value == "off")
            {
                defaultOptions_.format = ModelFormat::Off;
            }
            else if (*value == "gltf" || *value == "glb")
            {
                defaultOptions_.format = ModelFormat::Gltf;
            }
            else if (*value == "auto")
            {
                defaultOptions_.format = ModelFormat::Auto;
            }
        }
        // models.winding_mode: automatic | preserve | reverse; unknown names ignored.
        if (const auto value = context.configurationStringInSection("models", "winding_mode"))
        {
            if (*value == "automatic")
            {
                defaultOptions_.windingMode = ModelWindingMode::Automatic;
            }
            else if (*value == "preserve")
            {
                defaultOptions_.windingMode = ModelWindingMode::Preserve;
            }
            else if (*value == "reverse")
            {
                defaultOptions_.windingMode = ModelWindingMode::Reverse;
            }
        }
        if (const auto value = context.configurationStringInSection("models", "material_search_path"))
        {
            materialSearchPath_ = *value;
        }
        // models.max_loaded_models: 0..2^24 concurrent models; 0 disables the cap.
        if (const auto value = context.configurationIntegerInSection("models", "max_loaded_models"))
        {
            if (*value >= 0 && static_cast<std::uint64_t>(*value) <= maximumLoadedModelCount)
            {
                maximumLoadedModels_ = static_cast<std::uint32_t>(*value);
            }
        }
        // models.max_file_size_mb: 0..2^20 MiB per file; 0 disables the limit.
        if (const auto value = context.configurationIntegerInSection("models", "max_file_size_mb"))
        {
            if (*value >= 0 && static_cast<std::uint64_t>(*value) <= maximumFileSizeMegabytes)
            {
                maximumFileSizeBytes_ = static_cast<std::uint64_t>(*value) * bytesPerMegabyte;
            }
        }
    }

    // Re-reads the defaults only when at least one changed key is in [models].
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool modelsChanged = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("models.", 0) == 0; });
        if (modelsChanged)
        {
            readConfiguration();
        }
    }

    // Configuration bounds: MiB conversion, largest accepted max_file_size_mb, and
    // the total handle space available to max_loaded_models.
    static constexpr std::uint64_t bytesPerMegabyte         = 1024ull * 1024ull;
    static constexpr std::uint64_t maximumFileSizeMegabytes = 1024ull * 1024ull;
    static constexpr std::uint64_t maximumLoadedModelCount  = 1ull << 24;

    // In-memory manifest handed to the kernel; mirrors module.toml.
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "models",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"models"},
            .requiredCapabilities = {}};
    }

    // Resolution order: absolute paths must exist as given; relative paths try
    // every configured search path first, then the working directory.
    std::optional<std::filesystem::path> resolvePath(const std::string &path) const
    {
        const std::filesystem::path given(path);
        std::error_code             filesystemError;
        if (given.is_absolute())
        {
            return std::filesystem::exists(given, filesystemError) && !filesystemError
                     ? std::optional<std::filesystem::path>(given)
                     : std::nullopt;
        }
        for (const std::string &searchPath : searchPaths_)
        {
            filesystemError.clear();
            const std::filesystem::path candidate = std::filesystem::path(searchPath) / given;
            if (std::filesystem::exists(candidate, filesystemError) && !filesystemError)
            {
                return candidate;
            }
        }
        filesystemError.clear();
        if (std::filesystem::exists(given, filesystemError) && !filesystemError)
        {
            return given;
        }
        return std::nullopt;
    }

    ModelHandle loadModelImpl(const char *path, ModelLoadError *error)
    {
        return loadModelWithOptionsImpl(path, nullptr, error);
    }

    // The load pipeline: validate options, resolve and size-limit the file, pick
    // an importer (explicit format or file extension), optimize, then register the
    // finished record under a fresh handle.
    ModelHandle loadModelWithOptionsImpl(const char *path, const ModelLoadOptions *options, ModelLoadError *error)
    {
        const std::size_t pathLength = boundedStringLength(path, maximumModelPathLength + 1u);
        if (pathLength == 0 || pathLength > maximumModelPathLength)
        {
            reportError(error, ModelLoadError::InvalidOptions);
            return invalidModel;
        }
        try
        {
        // models.max_loaded_models (0 disables the cap).
        if (maximumLoadedModels_ != 0 && models_.size() >= maximumLoadedModels_)
        {
            reportError(error, ModelLoadError::TooManyModels);
            return invalidModel;
        }
        // Caller options replace the configured defaults wholesale; they are
        // validated (and the rotation normalized) on the local copy below.
        ModelLoadOptions selectedOptions = options != nullptr ? *options : defaultOptions_;
        if (!validOptions(selectedOptions))
        {
            reportError(error, ModelLoadError::InvalidOptions);
            return invalidModel;
        }
        const ModelTransform                       transform = makeTransform(selectedOptions);
        const std::optional<std::filesystem::path> resolved  = resolvePath(path);
        if (!resolved.has_value())
        {
            reportError(error, ModelLoadError::FileNotFound);
            return invalidModel;
        }

        std::error_code     fileError;
        const std::uint64_t fileSize = std::filesystem::file_size(*resolved, fileError);
        if (fileError)
        {
            reportError(error, ModelLoadError::ParseFailed);
            return invalidModel;
        }
        // models.max_file_size_mb (0 disables the limit).
        if (maximumFileSizeBytes_ != 0 && fileSize > maximumFileSizeBytes_)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return invalidModel;
        }

        ModelRecord record;
        std::string extension = resolved->extension().string();
        std::transform(extension.begin(),
            extension.end(),
            extension.begin(),
            [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        // Auto format: choose the importer from the lowercased file extension.
        const ModelFormat format = selectedOptions.format != ModelFormat::Auto
                                     ? selectedOptions.format
                                     : (extension == ".obj"                                ? ModelFormat::Obj
                                           : extension == ".stl"                           ? ModelFormat::Stl
                                           : extension == ".ply"                           ? ModelFormat::Ply
                                           : extension == ".off"                           ? ModelFormat::Off
                                           : (extension == ".gltf" || extension == ".glb") ? ModelFormat::Gltf
                                                                                           : ModelFormat::Auto);
        if (format == ModelFormat::Obj)
        {
            if (!detail::importObjFile(*resolved, record, selectedOptions, transform, materialSearchPath_, error))
            {
                return invalidModel;
            }
        }
        else if (format == ModelFormat::Stl)
        {
            if (!detail::importStlFile(*resolved, record, selectedOptions, transform, error))
            {
                return invalidModel;
            }
        }
        else if (format == ModelFormat::Ply)
        {
            if (!detail::importPlyFile(*resolved, record, selectedOptions, transform, error))
            {
                return invalidModel;
            }
        }
        else if (format == ModelFormat::Off)
        {
            if (!detail::importOffFile(*resolved, record, selectedOptions, transform, error))
            {
                return invalidModel;
            }
        }
        else if (format == ModelFormat::Gltf)
        {
            if (!detail::importGltfFile(*resolved, record, selectedOptions, transform, error))
            {
                return invalidModel;
            }
        }
        else
        {
            std::fprintf(stderr,
                "models module: unsupported model format '%s' (%s)\n",
                resolved->extension().string().c_str(),
                path);
            reportError(error, ModelLoadError::ParseFailed);
            return invalidModel;
        }

        // Optimization uses the module-configured pass set, not per-load options.
        if (selectedOptions.optimizeMesh)
        {
            detail::optimizeModelRecord(record, optimizationOptions_);
        }

        // Statistics describe the post-optimization geometry.
        record.statistics.sourceFileSizeBytes = fileSize;
        record.statistics.partCount           = static_cast<std::uint32_t>(record.parts.size());
        record.statistics.materialCount       = static_cast<std::uint32_t>(record.materials.size());
        for (const ModelPart &part : record.parts)
        {
            record.statistics.vertexCount += part.vertices.size() / renderer::meshVertexFloatCount;
            record.statistics.indexCount += part.indices.size();
        }

        const ModelHandle id = allocateModelHandle();
        if (id == invalidModel)
        {
            reportError(error, ModelLoadError::TooManyModels);
            return invalidModel;
        }
        // Both failure paths report TooManyModels: the counter wrapped (handle space
        // exhausted) or, unexpectedly, the id is already taken.
        const auto inserted = models_.emplace(id, std::move(record));
        if (!inserted.second)
        {
            reportError(error, ModelLoadError::TooManyModels);
            return invalidModel;
        }
        reportError(error, ModelLoadError::None);
        return id;
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return invalidModel;
        }
        catch (const std::length_error &)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return invalidModel;
        }
        catch (...)
        {
            reportError(error, ModelLoadError::ParseFailed);
            return invalidModel;
        }
    }

    // Checks a caller-supplied option set in place: enum ranges, material search
    // path length, finite magnitudes bounded by 1e12, and a renormalized unit
    // rotation quaternion (which is why the parameter is mutable).
    static bool validOptions(ModelLoadOptions &options)
    {
        if (options.normalMode > ModelNormalMode::GenerateFlat || options.format > ModelFormat::Gltf
            || options.windingMode > ModelWindingMode::Reverse
            || (options.materialSearchPath != nullptr
                && boundedStringLength(options.materialSearchPath, 4097u) > 4096u))
        {
            return false;
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(options.scale[axis]) || !std::isfinite(options.translation[axis])
                || !std::isfinite(options.pivot[axis]) || std::abs(options.scale[axis]) > maximumTransformMagnitude
                || std::abs(options.translation[axis]) > maximumTransformMagnitude
                || std::abs(options.pivot[axis]) > maximumTransformMagnitude)
            {
                return false;
            }
        }
        for (std::size_t axis = 0; axis < 2; ++axis)
        {
            if (!std::isfinite(options.uvScale[axis]) || !std::isfinite(options.uvOffset[axis])
                || std::abs(options.uvScale[axis]) > maximumTransformMagnitude
                || std::abs(options.uvOffset[axis]) > maximumTransformMagnitude)
            {
                return false;
            }
        }
        for (const float component : options.rotation)
        {
            if (!std::isfinite(component))
            {
                return false;
            }
        }
        const double rotationLength
            = std::hypot(std::hypot(static_cast<double>(options.rotation[0]), static_cast<double>(options.rotation[1])),
                std::hypot(static_cast<double>(options.rotation[2]), static_cast<double>(options.rotation[3])));
        if (!std::isfinite(rotationLength) || rotationLength <= 1e-8)
        {
            return false;
        }
        for (float &component : options.rotation)
        {
            component = static_cast<float>(component / rotationLength);
        }
        return true;
    }

    void destroyModelImpl(ModelHandle model)
    {
        models_.erase(model);
    }

    std::uint32_t partCountImpl(ModelHandle model) const
    {
        const auto found = models_.find(model);
        return found != models_.end() ? static_cast<std::uint32_t>(found->second.parts.size()) : 0;
    }

    // Hands out pointers into the record's own buffers; they stay valid until
    // destroyModel() or module stop (no copy is made for the renderer).
    bool partImpl(ModelHandle model, std::uint32_t partIndex, renderer::MeshDescription *outDescription) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || outDescription == nullptr || partIndex >= found->second.parts.size())
        {
            return false;
        }
        const ModelPart &part = found->second.parts[partIndex];
        *outDescription       = renderer::MeshDescription{
            .vertices    = part.vertices.data(),
            .vertexCount = static_cast<std::uint32_t>(part.vertices.size() / renderer::meshVertexFloatCount),
            .indices     = part.indices.data(),
            .indexCount  = static_cast<std::uint32_t>(part.indices.size())};
        return true;
    }

    const char *partNameImpl(ModelHandle model, std::uint32_t partIndex) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || partIndex >= found->second.parts.size())
        {
            return nullptr;
        }
        return found->second.parts[partIndex].name.c_str();
    }

    // Bounds accumulated by the importers, after the option transform.
    bool boundsImpl(ModelHandle model, float *outMinimum, float *outMaximum) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || found->second.parts.empty())
        {
            return false;
        }
        if (outMinimum != nullptr)
        {
            std::copy(found->second.minimum, found->second.minimum + 3, outMinimum);
        }
        if (outMaximum != nullptr)
        {
            std::copy(found->second.maximum, found->second.maximum + 3, outMaximum);
        }
        return true;
    }

    std::uint32_t materialCountImpl(ModelHandle model) const
    {
        const auto found = models_.find(model);
        return found != models_.end() ? static_cast<std::uint32_t>(found->second.materials.size()) : 0;
    }

    bool materialImpl(ModelHandle model, std::uint32_t materialIndex, ModelMaterial *outMaterial) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || outMaterial == nullptr || materialIndex >= found->second.materials.size())
        {
            return false;
        }
        *outMaterial = found->second.materials[materialIndex].factors;
        return true;
    }

    const char *materialNameImpl(ModelHandle model, std::uint32_t materialIndex) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || materialIndex >= found->second.materials.size())
        {
            return nullptr;
        }
        return found->second.materials[materialIndex].name.c_str();
    }

    const char *materialTexturePathImpl(
        ModelHandle model, std::uint32_t materialIndex, ModelMaterialTextureSlot slot) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || materialIndex >= found->second.materials.size()
            || slot >= ModelMaterialTextureSlot::Count)
        {
            return nullptr;
        }
        const std::string &path = found->second.materials[materialIndex].texturePaths[static_cast<std::size_t>(slot)];
        return path.empty() ? nullptr : path.c_str();
    }

    bool materialTextureOptionsImpl(ModelHandle model, std::uint32_t materialIndex, ModelMaterialTextureSlot slot,
        ModelMaterialTextureOptions *outOptions) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || outOptions == nullptr || materialIndex >= found->second.materials.size()
            || slot >= ModelMaterialTextureSlot::Count)
        {
            return false;
        }
        *outOptions = found->second.materials[materialIndex].textureOptions[static_cast<std::size_t>(slot)];
        return true;
    }

    // The per-triangle array is null when the part carries no material assignments;
    // the reported triangle count matches the part's index list.
    bool partTriangleMaterialsImpl(ModelHandle model, std::uint32_t partIndex, const std::uint32_t **outMaterialIndices,
        std::uint32_t *outTriangleCount) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || partIndex >= found->second.parts.size()
            || (outMaterialIndices == nullptr && outTriangleCount == nullptr))
        {
            return false;
        }
        const ModelPart &part = found->second.parts[partIndex];
        if (outMaterialIndices != nullptr)
        {
            *outMaterialIndices = part.triangleMaterials.empty() ? nullptr : part.triangleMaterials.data();
        }
        if (outTriangleCount != nullptr)
        {
            *outTriangleCount = static_cast<std::uint32_t>(part.indices.size() / 3);
        }
        return true;
    }

    bool statisticsImpl(ModelHandle model, ModelStatistics *outStatistics) const
    {
        const auto found = models_.find(model);
        if (found == models_.end() || outStatistics == nullptr)
        {
            return false;
        }
        *outStatistics = found->second.statistics;
        return true;
    }

    // Binds the C-style capability entries to the impl methods; `self` carries the
    // module instance through the void* first parameter.
    void assignCapabilityPointers()
    {
        capability_.self      = this;
        capability_.loadModel = [](void *self, const char *path, ModelLoadError *error)
        { return static_cast<ModelsModule *>(self)->loadModelImpl(path, error); };
        capability_.destroyModel
            = [](void *self, ModelHandle model) { static_cast<ModelsModule *>(self)->destroyModelImpl(model); };
        capability_.partCount = [](void *self, ModelHandle model)
        { return static_cast<const ModelsModule *>(self)->partCountImpl(model); };
        capability_.part
            = [](void *self, ModelHandle model, std::uint32_t partIndex, renderer::MeshDescription *outDescription)
        { return static_cast<const ModelsModule *>(self)->partImpl(model, partIndex, outDescription); };
        capability_.partName = [](void *self, ModelHandle model, std::uint32_t partIndex)
        { return static_cast<const ModelsModule *>(self)->partNameImpl(model, partIndex); };
        capability_.bounds = [](void *self, ModelHandle model, float *outMinimum, float *outMaximum)
        { return static_cast<const ModelsModule *>(self)->boundsImpl(model, outMinimum, outMaximum); };
        capability_.loadModelWithOptions
            = [](void *self, const char *path, const ModelLoadOptions *options, ModelLoadError *error)
        { return static_cast<ModelsModule *>(self)->loadModelWithOptionsImpl(path, options, error); };
        capability_.materialCount = [](void *self, ModelHandle model)
        { return static_cast<const ModelsModule *>(self)->materialCountImpl(model); };
        capability_.material = [](void *self, ModelHandle model, std::uint32_t index, ModelMaterial *out)
        { return static_cast<const ModelsModule *>(self)->materialImpl(model, index, out); };
        capability_.materialName = [](void *self, ModelHandle model, std::uint32_t index)
        { return static_cast<const ModelsModule *>(self)->materialNameImpl(model, index); };
        capability_.materialTexturePath
            = [](void *self, ModelHandle model, std::uint32_t index, ModelMaterialTextureSlot slot)
        { return static_cast<const ModelsModule *>(self)->materialTexturePathImpl(model, index, slot); };
        capability_.materialTextureOptions = [](void                         *self,
                                                 ModelHandle                  model,
                                                 std::uint32_t                index,
                                                 ModelMaterialTextureSlot     slot,
                                                 ModelMaterialTextureOptions *out)
        { return static_cast<const ModelsModule *>(self)->materialTextureOptionsImpl(model, index, slot, out); };
        capability_.partTriangleMaterials
            = [](void *self, ModelHandle model, std::uint32_t part, const std::uint32_t **indices, std::uint32_t *count)
        { return static_cast<const ModelsModule *>(self)->partTriangleMaterialsImpl(model, part, indices, count); };
        capability_.statistics = [](void *self, ModelHandle model, ModelStatistics *out)
        { return static_cast<const ModelsModule *>(self)->statisticsImpl(model, out); };
    }

    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    // Monotonic handle counter that skips 0 and live ids, wrapping at 2^32;
    // invalidModel is returned only when every handle is in use.
    ModelHandle allocateModelHandle()
    {
        const ModelHandle firstCandidate = nextModelId_ == invalidModel ? 1 : nextModelId_;
        do
        {
            ModelHandle candidate = nextModelId_++;
            if (nextModelId_ == invalidModel)
            {
                nextModelId_ = 1;
            }
            if (candidate != invalidModel && models_.find(candidate) == models_.end())
            {
                return candidate;
            }
        }
        while (nextModelId_ != firstCandidate);
        return invalidModel;
    }

    // Configuration state, re-read on construct and on [models.*] reloads.
    std::vector<std::string>         searchPaths_;
    std::string                      materialSearchPath_;
    ModelLoadOptions                 defaultOptions_{};
    std::uint64_t                    maximumFileSizeBytes_ = 512ull * bytesPerMegabyte;
    std::uint32_t                    maximumLoadedModels_  = 256;
    detail::ModelOptimizationOptions optimizationOptions_{};

    // Registry of loaded models; erased per handle by destroyModel(), cleared in stop().
    std::unordered_map<ModelHandle, ModelRecord> models_;
    ModelHandle                                  nextModelId_ = 1;

    ParticipantManifest manifest_;

    ModelsCapability capability_{};
};

} // namespace
} // namespace models

using models::ModelsModule;

} // namespace engine

// Exported module entry table: what the kernel loads the .so for. The module
// provides the "models" capability, requires none, and declares no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"models"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "models",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::models::ModelsModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
