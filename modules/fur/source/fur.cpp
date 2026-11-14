// fur module implementation.
//
// Owns the fur look presets, the [fur] configuration section (including live
// reload through ConfigurationReloadedEvent), the procedural strand-texture
// generator and the screen-size LOD evaluator, and binds the "fur"
// capability entries. The module owns no GPU state: consumers upload the
// generated pixels through the renderer's createTexture and draw through the
// renderer's drawFur with the parameters produced here. The module performs
// no synchronization of its own.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/fur/capabilities.hpp>

#include "fur_internal.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace engine
{
namespace fur
{
namespace
{

// Presets: three shipped fur looks spanning the density/length/variance
// range the demo's species picker cycles through. All parameters ride
// FurLayerParams / FurTextureRequest unchanged, so presets are content, not
// code paths.
FurParameters presetTable(std::uint32_t presetIndex)
{
    switch (static_cast<FurPreset>(presetIndex))
    {
    case FurPreset::Shaggy:
        // Long, uneven strands; soft edges so the length spread reads as
        // scruff rather than spikes.
        return FurParameters{
            .strandLength    = 0.075f,
            .density         = 0.62f,
            .lengthVariance  = 0.75f,
            .colorVariation  = 0.22f,
            .patternScale    = 4.5f,
            .gravityStrength = 0.05f,
            .densityBoost    = 5.0f,
            .seed            = 7u,
        };
    case FurPreset::Velvety:
        // Very short, dense, even coat with a subtle sheen-friendly crispness.
        return FurParameters{
            .strandLength    = 0.022f,
            .density         = 0.9f,
            .lengthVariance  = 0.2f,
            .colorVariation  = 0.1f,
            .patternScale    = 9.0f,
            .gravityStrength = 0.0f,
            .densityBoost    = 8.0f,
            .seed            = 11u,
        };
    case FurPreset::Plush:
    default:
        // Short-medium dense coat, moderate variance: the friendly default.
        return FurParameters{
            .strandLength    = 0.038f,
            .density         = 0.74f,
            .lengthVariance  = 0.45f,
            .colorVariation  = 0.16f,
            .patternScale    = 6.0f,
            .gravityStrength = 0.0f,
            .densityBoost    = 6.0f,
            .seed            = 3u,
        };
    }
}

// Parses "plush"/"shaggy"/"velvety"; unknown names warn and fall back to
// Plush.
FurPreset presetFromName(const std::string &name)
{
    if (name == "shaggy")
        return FurPreset::Shaggy;
    if (name == "velvety")
        return FurPreset::Velvety;
    if (name != "plush")
    {
        std::fprintf(stderr, "fur module: unknown [fur] default_preset '%s'; using \"plush\"\n", name.c_str());
    }
    return FurPreset::Plush;
}

class FurModule final : public ModuleInterface
{
public:
    FurModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        context_ = &context;
        readConfiguration();
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<FurModule, &FurModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

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
        if (capabilityName == FurCapability::capabilityName)
        {
            return &furCapability_;
        }
        return nullptr;
    }

private:
    // Resets every setting to its built-in default, then overlays whatever
    // [fur] keys are present. Out-of-range values are clamped with a stderr
    // warning, matching the renderer module's configuration style.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;

        defaultPreset_ = presetFromName(
            context.configurationStringInSection("fur", "default_preset").value_or("plush"));

        if (const auto textureSize = context.configurationIntegerInSection("fur", "texture_size");
            textureSize.has_value())
        {
            if (*textureSize < 16 || *textureSize > 2048)
            {
                std::fprintf(stderr,
                    "fur module: clamping [fur] texture_size %lld to range 16..2048\n",
                    static_cast<long long>(*textureSize));
            }
            defaultTextureSize_ =
                static_cast<std::uint32_t>(std::clamp<std::int64_t>(*textureSize, 16, 2048));
        }

        if (const auto maximumLayers = context.configurationIntegerInSection("fur", "max_layers");
            maximumLayers.has_value())
        {
            if (*maximumLayers < 1 || *maximumLayers > 64)
            {
                std::fprintf(stderr,
                    "fur module: clamping [fur] max_layers %lld to range 1..64\n",
                    static_cast<long long>(*maximumLayers));
            }
            lodBands_.maximumLayers = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*maximumLayers, 1, 64));
        }

        if (const auto fadeFraction = context.configurationFloatingInSection("fur", "lod_fade_fraction");
            fadeFraction.has_value())
        {
            if (!std::isfinite(*fadeFraction) || *fadeFraction <= 0.0 || *fadeFraction > 1.0)
            {
                std::fprintf(stderr,
                    "fur module: clamping [fur] lod_fade_fraction %g to range (0..1]\n",
                    *fadeFraction);
            }
            lodBands_.fadeFraction = static_cast<float>(std::clamp(*fadeFraction, 0.01, 1.0));
        }

        // Layer budgets per LOD band: five values, most detailed first.
        const std::vector<double> layerCounts =
            context.configurationFloatingArrayInSection("fur", "lod_layer_counts");
        if (layerCounts.size() >= LodBands::bandCount)
        {
            for (std::size_t band = 0; band < LodBands::bandCount; ++band)
            {
                const long long value = static_cast<long long>(layerCounts[band]);
                if (value < 0 || value > 64)
                {
                    std::fprintf(stderr,
                        "fur module: clamping [fur] lod_layer_counts band %zu (%lld) to range 0..64\n",
                        band,
                        value);
                }
                lodBands_.layerCounts[band] =
                    static_cast<std::uint32_t>(std::clamp<std::int64_t>(value, 0, 64));
            }
        }

        // Angular-size thresholds: four values, descending; bands[i] begins
        // at thresholds[i] and the fifth band catches everything smaller.
        const std::vector<double> thresholds =
            context.configurationFloatingArrayInSection("fur", "lod_screen_sizes");
        if (thresholds.size() >= 4)
        {
            float previous = thresholds[0] + 1.0f;
            for (std::size_t band = 0; band < 4; ++band)
            {
                double value = thresholds[band];
                if (!std::isfinite(value) || value <= 0.0 || value >= previous)
                {
                    std::fprintf(stderr,
                        "fur module: [fur] lod_screen_sizes band %zu (%g) must be finite, > 0 and descending; "
                        "keeping the default\n",
                        band,
                        value);
                    // Restore the built-in table for this reload.
                    lodBands_.thresholds[0] = 0.32f;
                    lodBands_.thresholds[1] = 0.16f;
                    lodBands_.thresholds[2] = 0.08f;
                    lodBands_.thresholds[3] = 0.03f;
                    break;
                }
                lodBands_.thresholds[band] = static_cast<float>(value);
                previous                   = static_cast<float>(value);
            }
        }

        // Strand-height scales per band: shrinking the fur length at
        // distance keeps the reduced layer stacks from reading as sparse
        // wire spikes.
        const std::vector<double> heightScales =
            context.configurationFloatingArrayInSection("fur", "lod_height_scales");
        if (heightScales.size() >= LodBands::bandCount)
        {
            for (std::size_t band = 0; band < LodBands::bandCount; ++band)
            {
                double value = heightScales[band];
                if (!std::isfinite(value) || value <= 0.0)
                {
                    std::fprintf(stderr,
                        "fur module: [fur] lod_height_scales band %zu (%g) must be finite and > 0; keeping 1.0\n",
                        band,
                        value);
                    value = 1.0;
                }
                lodBands_.heightScales[band] = static_cast<float>(std::clamp(value, 0.1, 2.0));
            }
        }
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &)
    {
        readConfiguration();
    }

    // ---- capability implementations ----------------------------------------

    static FurParameters presetParametersImpl(void *self, FurPreset preset)
    {
        (void)self; // presets are static content
        const std::uint32_t index = static_cast<std::uint32_t>(preset);
        return presetTable(index >= static_cast<std::uint32_t>(FurPreset::count) ? 0u : index);
    }

    static bool generateFurTextureImpl(
        void *self, const FurTextureRequest *request, std::uint8_t *outPixelsRgba, std::size_t outCapacityBytes)
    {
        (void)self;
        if (request == nullptr || outPixelsRgba == nullptr)
        {
            std::fprintf(stderr, "fur module: generateFurTexture called with null request or buffer\n");
            return false;
        }
        if (request->width < 16u || request->width > 2048u || request->height < 16u || request->height > 2048u)
        {
            std::fprintf(stderr,
                "fur module: fur texture size %ux%u outside 16..2048\n",
                request->width,
                request->height);
            return false;
        }
        const std::size_t requiredBytes = static_cast<std::size_t>(request->width) * request->height * 4u;
        if (outCapacityBytes < requiredBytes)
        {
            std::fprintf(stderr,
                "fur module: fur texture buffer too small (%zu < %zu bytes)\n",
                outCapacityBytes,
                requiredBytes);
            return false;
        }
        generateFurTexturePixels(request->parameters, request->width, request->height, outPixelsRgba);
        return true;
    }

    FurLodDecision evaluateFurLodImpl(void *self, float angularSize) const
    {
        (void)self;
        return evaluateFurLodDecision(lodBands_, angularSize);
    }

    void assignCapabilityPointers()
    {
        furCapability_.self = this;
        furCapability_.presetParameters = &FurModule::presetParametersImpl;
        furCapability_.generateFurTexture = &FurModule::generateFurTextureImpl;
        furCapability_.evaluateFurLod = [](void *self, float angularSize)
        { return static_cast<const FurModule *>(self)->evaluateFurLodImpl(self, angularSize); };
    }

    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "fur",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"fur"},
            .requiredCapabilities = {}};
    }

    ParticipantManifest manifest_;
    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    FurParameters defaultPresetParameters_ = presetTable(static_cast<std::uint32_t>(FurPreset::Plush));
    FurPreset     defaultPreset_           = FurPreset::Plush;
    std::uint32_t defaultTextureSize_      = 256u;
    LodBands      lodBands_{};

    FurCapability furCapability_{};
};

} // namespace
} // namespace fur

using fur::FurModule;

} // namespace engine

// The kernel's only required export: module metadata + factory. The lists
// here must match makeManifest() and module.toml.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"fur"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "fur",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::fur::FurModule(); },
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
