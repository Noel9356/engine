// pbr module implementation.
//
// Owns the shared PbrEnvironment and the model-material conversion options,
// applies the [pbr] configuration section (including live reload through
// ConfigurationReloadedEvent), and binds the "pbr" capability entries. The
// module performs no synchronization of its own.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/pbr/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace engine
{
namespace pbr
{
namespace
{

// Validates a config-provided direction (finite, non-degenerate) and writes
// it normalized. Returns false without touching outDirection otherwise.
bool normalizedDirection(const double *values, float *outDirection)
{
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2]))
    {
        return false;
    }
    const double length = std::hypot(std::hypot(values[0], values[1]), values[2]);
    if (!std::isfinite(length) || length <= 1e-8)
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        outDirection[axis] = static_cast<float>(values[axis] / length);
    }
    return true;
}

// Copies a color, clamping to the representable float range; non-finite
// components are skipped so the destination keeps its previous value.
void copyColor(const double *values, float *outColor)
{
    for (std::size_t index = 0; index < 3; ++index)
    {
        if (std::isfinite(values[index]))
        {
            outColor[index] = static_cast<float>(
                std::clamp(values[index], 0.0, static_cast<double>(std::numeric_limits<float>::max())));
        }
    }
}

// Validates a whole environment and writes the normalized form: unit sun
// direction, negative color components clamped to 0, and range checks on
// intensities/exposure/tone mapping. Returns false for invalid input, with
// destination only partially written in that case.
bool normalizeEnvironment(const PbrEnvironment &source, PbrEnvironment &destination)
{
    destination = source;
    if (!std::isfinite(source.sun.direction[0]) || !std::isfinite(source.sun.direction[1])
        || !std::isfinite(source.sun.direction[2]))
    {
        return false;
    }
    const double directionLength = std::hypot(
        std::hypot(static_cast<double>(source.sun.direction[0]), static_cast<double>(source.sun.direction[1])),
        static_cast<double>(source.sun.direction[2]));
    if (!std::isfinite(directionLength) || directionLength <= 1e-8)
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        destination.sun.direction[axis] = static_cast<float>(source.sun.direction[axis] / directionLength);
        if (!std::isfinite(source.sun.color[axis]) || !std::isfinite(source.ambient.skyColor[axis])
            || !std::isfinite(source.ambient.groundColor[axis]))
        {
            return false;
        }
        destination.sun.color[axis]           = std::max(0.0f, source.sun.color[axis]);
        destination.ambient.skyColor[axis]    = std::max(0.0f, source.ambient.skyColor[axis]);
        destination.ambient.groundColor[axis] = std::max(0.0f, source.ambient.groundColor[axis]);
    }
    if (!std::isfinite(source.sun.intensity) || source.sun.intensity < 0.0f || !std::isfinite(source.ambient.intensity)
        || source.ambient.intensity < 0.0f || !std::isfinite(source.exposure) || source.exposure <= 0.0f
        || source.toneMapping > ToneMapping::AcesFilmic)
    {
        return false;
    }
    return true;
}

// Exact equality used for change detection; only real changes bump versions.
bool sameEnvironment(const PbrEnvironment &left, const PbrEnvironment &right)
{
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (left.sun.direction[axis] != right.sun.direction[axis] || left.sun.color[axis] != right.sun.color[axis]
            || left.ambient.skyColor[axis] != right.ambient.skyColor[axis]
            || left.ambient.groundColor[axis] != right.ambient.groundColor[axis])
        {
            return false;
        }
    }
    return left.sun.intensity == right.sun.intensity && left.sun.enabled == right.sun.enabled
        && left.ambient.intensity == right.ambient.intensity && left.exposure == right.exposure
        && left.toneMapping == right.toneMapping;
}

// Interpolates two normalized environments at t in [0, 1]: colors and
// intensities lerp linearly, exposure in log space, and the sun direction is
// renormalized (nearest endpoint wins if the blend degenerates to zero).
bool blendEnvironmentValues(const PbrEnvironment &from, const PbrEnvironment &to, float t, PbrEnvironment &out)
{
    if (!std::isfinite(t) || t < 0.0f || t > 1.0f)
    {
        return false;
    }
    PbrEnvironment a{};
    PbrEnvironment b{};
    if (!normalizeEnvironment(from, a) || !normalizeEnvironment(to, b))
    {
        return false;
    }
    const float inverseT = 1.0f - t;
    auto        lerp     = [t, inverseT](float left, float right) { return left * inverseT + right * t; };

    PbrEnvironment result{};
    float          direction[3]
        = {lerp(a.sun.direction[0], b.sun.direction[0]),
            lerp(a.sun.direction[1], b.sun.direction[1]),
            lerp(a.sun.direction[2], b.sun.direction[2])};
    const double directionLength
        = std::hypot(std::hypot(static_cast<double>(direction[0]), static_cast<double>(direction[1])),
            static_cast<double>(direction[2]));
    if (directionLength > 1.0e-8 && std::isfinite(directionLength))
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            result.sun.direction[axis] = static_cast<float>(direction[axis] / directionLength);
        }
    }
    else
    {
        const PbrEnvironment &nearest = t < 0.5f ? a : b;
        std::copy(nearest.sun.direction, nearest.sun.direction + 3, result.sun.direction);
    }

    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.sun.color[axis]           = lerp(a.sun.color[axis], b.sun.color[axis]);
        result.ambient.skyColor[axis]    = lerp(a.ambient.skyColor[axis], b.ambient.skyColor[axis]);
        result.ambient.groundColor[axis] = lerp(a.ambient.groundColor[axis], b.ambient.groundColor[axis]);
    }
    const float fromSunIntensity = a.sun.enabled ? a.sun.intensity : 0.0f;
    const float toSunIntensity   = b.sun.enabled ? b.sun.intensity : 0.0f;
    result.sun.intensity         = lerp(fromSunIntensity, toSunIntensity);
    result.sun.enabled           = result.sun.intensity > 0.0f;
    result.ambient.intensity     = lerp(a.ambient.intensity, b.ambient.intensity);
    result.exposure
        = static_cast<float>(std::exp(std::log(static_cast<double>(a.exposure)) * static_cast<double>(inverseT)
                                      + std::log(static_cast<double>(b.exposure)) * static_cast<double>(t)));
    result.toneMapping = t < 0.5f ? a.toneMapping : b.toneMapping;
    out                = result;
    return true;
}

// Conversion options must be finite and non-negative; minimumRoughness is
// additionally capped at 1. baseColorScale components have no upper bound.
bool validConversionOptions(const ModelMaterialConversionOptions &options)
{
    if (!std::isfinite(options.emissiveScale) || options.emissiveScale < 0.0f || !std::isfinite(options.metallicScale)
        || options.metallicScale < 0.0f || !std::isfinite(options.roughnessScale) || options.roughnessScale < 0.0f
        || !std::isfinite(options.minimumRoughness) || options.minimumRoughness < 0.0f
        || options.minimumRoughness > 1.0f)
    {
        return false;
    }
    for (float value : options.baseColorScale)
    {
        if (!std::isfinite(value) || value < 0.0f)
        {
            return false;
        }
    }
    return true;
}

// Standard sRGB transfer function: linear segment below 0.04045, gamma 2.4
// curve above.
float srgbToLinear(float value)
{
    const double encoded = static_cast<double>(value);
    const double linear  = encoded <= 0.04045 ? encoded / 12.92 : std::pow((encoded + 0.055) / 1.055, 2.4);
    return static_cast<float>(std::clamp(linear,
        -static_cast<double>(std::numeric_limits<float>::max()),
        static_cast<double>(std::numeric_limits<float>::max())));
}

bool sameConversionOptions(const ModelMaterialConversionOptions &left, const ModelMaterialConversionOptions &right)
{
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (left.baseColorScale[axis] != right.baseColorScale[axis])
        {
            return false;
        }
    }
    return left.emissiveScale == right.emissiveScale && left.metallicScale == right.metallicScale
        && left.roughnessScale == right.roughnessScale && left.minimumRoughness == right.minimumRoughness
        && left.useSpecularAsMetallic == right.useSpecularAsMetallic
        && left.decodeBaseColorSrgb == right.decodeBaseColorSrgb && left.decodeEmissiveSrgb == right.decodeEmissiveSrgb;
}

// Converts one legacy material: optional sRGB decode, scale and clamp every
// channel, floor roughness at options.minimumRoughness, and mark materials
// with illuminationModel == 0 as unlit. Returns false for non-finite source
// data, leaving outMaterial untouched.
bool convertModelMaterialValues(
    const models::ModelMaterial &source, const ModelMaterialConversionOptions &options, ConvertedModelMaterial &out)
{
    if (!validConversionOptions(options))
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(source.baseColor[axis]) || !std::isfinite(source.ambient[axis])
            || !std::isfinite(source.emissive[axis]) || !std::isfinite(source.specular[axis])
            || !std::isfinite(source.transmittance[axis]))
        {
            return false;
        }
    }
    if (!std::isfinite(source.metallic) || !std::isfinite(source.roughness) || !std::isfinite(source.opacity)
        || !std::isfinite(source.indexOfRefraction) || !std::isfinite(source.shininess) || !std::isfinite(source.sheen)
        || !std::isfinite(source.clearcoat) || !std::isfinite(source.clearcoatRoughness)
        || !std::isfinite(source.anisotropy) || !std::isfinite(source.anisotropyRotation))
    {
        return false;
    }

    ConvertedModelMaterial result{};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const float sourceBaseColor
            = options.decodeBaseColorSrgb ? srgbToLinear(source.baseColor[axis]) : source.baseColor[axis];
        const float sourceEmissive
            = options.decodeEmissiveSrgb ? srgbToLinear(source.emissive[axis]) : source.emissive[axis];
        const double baseColor = static_cast<double>(sourceBaseColor) * options.baseColorScale[axis];
        const double emissive  = static_cast<double>(sourceEmissive) * options.emissiveScale;
        result.material.baseColor[axis]
            = static_cast<float>(std::clamp(baseColor, 0.0, static_cast<double>(std::numeric_limits<float>::max())));
        result.material.emissiveColor[axis]
            = static_cast<float>(std::clamp(emissive, 0.0, static_cast<double>(std::numeric_limits<float>::max())));
        result.ambientColor[axis]  = std::max(0.0f, source.ambient[axis]);
        result.transmittance[axis] = std::clamp(source.transmittance[axis], 0.0f, 1.0f);
    }
    float metallic
        = static_cast<float>(std::clamp(static_cast<double>(source.metallic) * options.metallicScale, 0.0, 1.0));
    if (options.useSpecularAsMetallic && !source.hasPbrFactors && metallic == 0.0f)
    {
        metallic
            = std::clamp(std::max(source.specular[0], std::max(source.specular[1], source.specular[2])), 0.0f, 1.0f);
    }
    result.material.metallicFactor = metallic;
    result.material.roughnessFactor
        = static_cast<float>(std::clamp(static_cast<double>(source.roughness) * options.roughnessScale,
            static_cast<double>(options.minimumRoughness),
            1.0));
    result.material.unlit     = source.unlit || source.illuminationModel == 0u;
    result.opacity            = std::clamp(source.opacity, 0.0f, 1.0f);
    result.indexOfRefraction  = std::clamp(source.indexOfRefraction, 1.0e-4f, 1.0e4f);
    result.shininess          = std::max(0.0f, source.shininess);
    result.sheen              = std::clamp(source.sheen, 0.0f, 1.0f);
    result.clearcoat          = std::clamp(source.clearcoat, 0.0f, 1.0f);
    result.clearcoatRoughness = std::clamp(source.clearcoatRoughness, 0.0f, 1.0f);
    result.anisotropy         = std::clamp(source.anisotropy, 0.0f, 1.0f);
    result.anisotropyRotation = source.anisotropyRotation;

    out = result;
    return true;
}

// Module singleton backing PbrCapability. Keeps live and configured
// snapshots of both the environment and the conversion options; versions
// increment only on observed changes so consumers can poll cheaply.
class PbrModule final : public ModuleInterface
{
public:
    PbrModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Applies the [pbr] configuration section and follows configuration
    // reloads so the environment can be tuned without a restart.
    void construct(ParticipantContext &context) override
    {
        context_ = &context;
        readConfiguration();

        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<PbrModule, &PbrModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    // Drop the event subscription before the context goes away.
    void stop() override
    {
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == PbrCapability::capabilityName)
        {
            return &capability_;
        }
        return nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "pbr",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"pbr"},
            .requiredCapabilities = {}};
    }

    // Reads every [pbr.*] key into fresh state and publishes it. Absent,
    // mistyped, or out-of-range values keep their built-in defaults; unknown
    // enum strings warn on stderr and fall back. Versions bump only on
    // actual changes.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext            &context = *context_;
        PbrEnvironment                 nextEnvironment{};
        ModelMaterialConversionOptions nextConversionOptions{};

        const std::vector<double> sunDirection = context.configurationFloatingArrayInSection("pbr", "sun_direction");
        if (sunDirection.size() >= 3)
        {
            normalizedDirection(sunDirection.data(), nextEnvironment.sun.direction);
        }
        const std::vector<double> sunColor = context.configurationFloatingArrayInSection("pbr", "sun_color");
        if (sunColor.size() >= 3)
        {
            copyColor(sunColor.data(), nextEnvironment.sun.color);
        }
        const std::optional<double> sunIntensity = context.configurationFloatingInSection("pbr", "sun_intensity");
        if (sunIntensity.has_value() && std::isfinite(*sunIntensity) && *sunIntensity >= 0.0
            && *sunIntensity <= std::numeric_limits<float>::max())
        {
            nextEnvironment.sun.intensity = static_cast<float>(*sunIntensity);
        }
        const std::optional<bool> sunEnabled = context.configurationBooleanInSection("pbr", "sun_enabled");
        if (sunEnabled.has_value())
        {
            nextEnvironment.sun.enabled = *sunEnabled;
        }

        const std::vector<double> skyColor = context.configurationFloatingArrayInSection("pbr", "ambient_sky_color");
        if (skyColor.size() >= 3)
        {
            copyColor(skyColor.data(), nextEnvironment.ambient.skyColor);
        }
        const std::vector<double> groundColor
            = context.configurationFloatingArrayInSection("pbr", "ambient_ground_color");
        if (groundColor.size() >= 3)
        {
            copyColor(groundColor.data(), nextEnvironment.ambient.groundColor);
        }
        const std::optional<std::string> ambientMode = context.configurationStringInSection("pbr", "ambient_mode");
        if (ambientMode.has_value())
        {
            if (*ambientMode == "uniform")
            {
                const std::vector<double> ambientColor
                    = context.configurationFloatingArrayInSection("pbr", "ambient_color");
                if (ambientColor.size() >= 3)
                {
                    copyColor(ambientColor.data(), nextEnvironment.ambient.skyColor);
                    copyColor(ambientColor.data(), nextEnvironment.ambient.groundColor);
                }
                else
                {
                    std::copy(nextEnvironment.ambient.skyColor,
                        nextEnvironment.ambient.skyColor + 3,
                        nextEnvironment.ambient.groundColor);
                }
            }
            else if (*ambientMode != "hemisphere")
            {
                std::fprintf(stderr,
                    "pbr module: unknown [pbr] ambient_mode '%s'; using \"hemisphere\"\n",
                    ambientMode->c_str());
            }
        }
        const std::optional<double> ambientIntensity
            = context.configurationFloatingInSection("pbr", "ambient_intensity");
        if (ambientIntensity.has_value() && std::isfinite(*ambientIntensity) && *ambientIntensity >= 0.0
            && *ambientIntensity <= std::numeric_limits<float>::max())
        {
            nextEnvironment.ambient.intensity = static_cast<float>(*ambientIntensity);
        }

        const std::optional<double> exposure = context.configurationFloatingInSection("pbr", "exposure");
        if (exposure.has_value() && std::isfinite(*exposure) && *exposure > 0.0
            && *exposure <= std::numeric_limits<float>::max())
        {
            nextEnvironment.exposure = static_cast<float>(*exposure);
        }

        const std::vector<double> baseColorScale
            = context.configurationFloatingArrayInSection("pbr", "model_base_color_scale");
        if (baseColorScale.size() >= 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (std::isfinite(baseColorScale[axis]) && baseColorScale[axis] >= 0.0
                    && baseColorScale[axis] <= std::numeric_limits<float>::max())
                {
                    nextConversionOptions.baseColorScale[axis] = static_cast<float>(baseColorScale[axis]);
                }
            }
        }
        auto readNonNegativeScale = [&](const char *key, float &destination)
        {
            const auto value = context.configurationFloatingInSection("pbr", key);
            if (value.has_value() && std::isfinite(*value) && *value >= 0.0
                && *value <= std::numeric_limits<float>::max())
            {
                destination = static_cast<float>(*value);
            }
        };
        readNonNegativeScale("model_emissive_scale", nextConversionOptions.emissiveScale);
        readNonNegativeScale("model_metallic_scale", nextConversionOptions.metallicScale);
        readNonNegativeScale("model_roughness_scale", nextConversionOptions.roughnessScale);
        if (const auto minimum = context.configurationFloatingInSection("pbr", "model_minimum_roughness"))
        {
            if (std::isfinite(*minimum) && *minimum >= 0.0 && *minimum <= 1.0)
            {
                nextConversionOptions.minimumRoughness = static_cast<float>(*minimum);
            }
        }
        if (const auto useSpecular = context.configurationBooleanInSection("pbr", "model_use_specular_as_metallic"))
        {
            nextConversionOptions.useSpecularAsMetallic = *useSpecular;
        }
        if (const auto decodeBaseColor = context.configurationBooleanInSection("pbr", "model_decode_base_color_srgb"))
        {
            nextConversionOptions.decodeBaseColorSrgb = *decodeBaseColor;
        }
        if (const auto decodeEmissive = context.configurationBooleanInSection("pbr", "model_decode_emissive_srgb"))
        {
            nextConversionOptions.decodeEmissiveSrgb = *decodeEmissive;
        }

        const std::optional<std::string> toneMapping = context.configurationStringInSection("pbr", "tone_mapping");
        if (toneMapping.has_value())
        {
            if (*toneMapping == "none")
            {
                nextEnvironment.toneMapping = ToneMapping::None;
            }
            else if (*toneMapping == "reinhard")
            {
                nextEnvironment.toneMapping = ToneMapping::Reinhard;
            }
            else if (*toneMapping == "aces")
            {
                nextEnvironment.toneMapping = ToneMapping::AcesFilmic;
            }
            else
            {
                std::fprintf(
                    stderr, "pbr module: unknown [pbr] tone_mapping '%s'; using \"aces\"\n", toneMapping->c_str());
            }
        }

        PbrEnvironment normalized{};
        if (!normalizeEnvironment(nextEnvironment, normalized))
        {
            normalized = PbrEnvironment{};
        }
        configuredEnvironment_ = normalized;
        if (!environmentInitialized_ || !sameEnvironment(environment_, normalized))
        {
            environment_ = normalized;
            ++environmentVersion_;
            environmentInitialized_ = true;
        }
        configuredModelConversionOptions_ = nextConversionOptions;
        if (!modelConversionOptionsInitialized_
            || !sameConversionOptions(modelConversionOptions_, configuredModelConversionOptions_))
        {
            modelConversionOptions_ = configuredModelConversionOptions_;
            ++modelConversionVersion_;
            modelConversionOptionsInitialized_ = true;
        }
    }

    // Ignore reload events that did not touch a pbr.* key.
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool pbrChanged = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("pbr.", 0) == 0; });
        if (pbrChanged)
        {
            readConfiguration();
        }
    }

    const PbrEnvironment *environmentImpl() const
    {
        return &environment_;
    }

    std::uint64_t environmentVersionImpl() const
    {
        return environmentVersion_;
    }

    // Normalize first so callers cannot store invalid state; identical
    // updates leave the version untouched.
    bool setEnvironmentImpl(const PbrEnvironment *environment)
    {
        if (environment == nullptr)
        {
            return false;
        }
        PbrEnvironment normalized{};
        if (!normalizeEnvironment(*environment, normalized))
        {
            return false;
        }
        if (!sameEnvironment(environment_, normalized))
        {
            environment_ = normalized;
            ++environmentVersion_;
        }
        return true;
    }

    void resetEnvironmentImpl()
    {
        if (!sameEnvironment(environment_, configuredEnvironment_))
        {
            environment_ = configuredEnvironment_;
            ++environmentVersion_;
        }
    }

    bool copyEnvironmentImpl(PbrEnvironment *outEnvironment, std::uint64_t *outVersion) const
    {
        if (outEnvironment == nullptr && outVersion == nullptr)
        {
            return false;
        }
        if (outEnvironment != nullptr)
        {
            *outEnvironment = environment_;
        }
        if (outVersion != nullptr)
        {
            *outVersion = environmentVersion_;
        }
        return true;
    }

    bool blendEnvironmentsImpl(
        const PbrEnvironment *from, const PbrEnvironment *to, float t, PbrEnvironment *outEnvironment) const
    {
        return from != nullptr && to != nullptr && outEnvironment != nullptr
            && blendEnvironmentValues(*from, *to, t, *outEnvironment);
    }

    // options == nullptr selects the module-configured options.
    bool convertModelMaterialImpl(const models::ModelMaterial *source, const ModelMaterialConversionOptions *options,
        ConvertedModelMaterial *outMaterial) const
    {
        if (source == nullptr || outMaterial == nullptr)
        {
            return false;
        }
        const ModelMaterialConversionOptions &selected = options != nullptr ? *options : modelConversionOptions_;
        return convertModelMaterialValues(*source, selected, *outMaterial);
    }

    // Empty batch is a no-op success; stops at the first invalid material.
    bool convertModelMaterialsImpl(const models::ModelMaterial *sources, std::uint32_t count,
        const ModelMaterialConversionOptions *options, ConvertedModelMaterial *outMaterials) const
    {
        if (count == 0)
        {
            return true;
        }
        if (sources == nullptr || outMaterials == nullptr)
        {
            return false;
        }
        const ModelMaterialConversionOptions &selected = options != nullptr ? *options : modelConversionOptions_;
        if (!validConversionOptions(selected))
        {
            return false;
        }
        for (std::uint32_t index = 0; index < count; ++index)
        {
            if (!convertModelMaterialValues(sources[index], selected, outMaterials[index]))
            {
                return false;
            }
        }
        return true;
    }

    bool setModelMaterialConversionOptionsImpl(const ModelMaterialConversionOptions *options)
    {
        if (options == nullptr || !validConversionOptions(*options))
        {
            return false;
        }
        if (!sameConversionOptions(modelConversionOptions_, *options))
        {
            modelConversionOptions_ = *options;
            ++modelConversionVersion_;
        }
        return true;
    }

    void resetModelMaterialConversionOptionsImpl()
    {
        if (!sameConversionOptions(modelConversionOptions_, configuredModelConversionOptions_))
        {
            modelConversionOptions_ = configuredModelConversionOptions_;
            ++modelConversionVersion_;
        }
    }

    bool copyModelMaterialConversionOptionsImpl(
        ModelMaterialConversionOptions *outOptions, std::uint64_t *outVersion) const
    {
        if (outOptions == nullptr && outVersion == nullptr)
        {
            return false;
        }
        if (outOptions != nullptr)
        {
            *outOptions = modelConversionOptions_;
        }
        if (outVersion != nullptr)
        {
            *outVersion = modelConversionVersion_;
        }
        return true;
    }

    // Bind every capability entry to its member implementation.
    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.environment = [](void *self) { return static_cast<const PbrModule *>(self)->environmentImpl(); };
        capability_.environmentVersion
            = [](void *self) { return static_cast<PbrModule *>(self)->environmentVersionImpl(); };
        capability_.setEnvironment = [](void *self, const PbrEnvironment *environment)
        { return static_cast<PbrModule *>(self)->setEnvironmentImpl(environment); };
        capability_.resetEnvironment = [](void *self) { static_cast<PbrModule *>(self)->resetEnvironmentImpl(); };
        capability_.copyEnvironment  = [](void *self, PbrEnvironment *outEnvironment, std::uint64_t *outVersion)
        { return static_cast<const PbrModule *>(self)->copyEnvironmentImpl(outEnvironment, outVersion); };
        capability_.blendEnvironments
            = [](void *self, const PbrEnvironment *from, const PbrEnvironment *to, float t, PbrEnvironment *out)
        { return static_cast<const PbrModule *>(self)->blendEnvironmentsImpl(from, to, t, out); };
        capability_.convertModelMaterial = [](void                                  *self,
                                               const models::ModelMaterial          *source,
                                               const ModelMaterialConversionOptions *options,
                                               ConvertedModelMaterial               *out)
        { return static_cast<const PbrModule *>(self)->convertModelMaterialImpl(source, options, out); };
        capability_.convertModelMaterials = [](void                                  *self,
                                                const models::ModelMaterial          *sources,
                                                std::uint32_t                         count,
                                                const ModelMaterialConversionOptions *options,
                                                ConvertedModelMaterial               *out)
        { return static_cast<const PbrModule *>(self)->convertModelMaterialsImpl(sources, count, options, out); };
        capability_.setModelMaterialConversionOptions = [](void *self, const ModelMaterialConversionOptions *options)
        { return static_cast<PbrModule *>(self)->setModelMaterialConversionOptionsImpl(options); };
        capability_.resetModelMaterialConversionOptions
            = [](void *self) { static_cast<PbrModule *>(self)->resetModelMaterialConversionOptionsImpl(); };
        capability_.copyModelMaterialConversionOptions
            = [](void *self, ModelMaterialConversionOptions *out, std::uint64_t *version)
        { return static_cast<const PbrModule *>(self)->copyModelMaterialConversionOptionsImpl(out, version); };
    }

    ParticipantContext *context_ = nullptr; // kernel-provided, valid until stop()

    // Live state returned by the capability and the configured snapshot that
    // the reset*() entries restore.
    PbrEnvironment                 environment_{};
    PbrEnvironment                 configuredEnvironment_{};
    ModelMaterialConversionOptions modelConversionOptions_{};
    ModelMaterialConversionOptions configuredModelConversionOptions_{};
    // Monotonic change counters polled by consumers.
    std::uint64_t environmentVersion_                = 0;
    std::uint64_t modelConversionVersion_            = 0;
    bool          environmentInitialized_            = false;
    bool          modelConversionOptionsInitialized_ = false;

    SubscriptionToken reloadSubscription_ = 0;

    ParticipantManifest manifest_;

    PbrCapability capability_{};
};

} // namespace
} // namespace pbr

} // namespace engine

// Module entry table the kernel loads from the .so: factory plus the
// provided/required capability lists (kept in sync with module.toml).
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"pbr"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "pbr",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::pbr::PbrModule(); },
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
