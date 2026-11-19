// physics_fur module implementation.
//
// Owns the fur look presets, the [physics_fur] configuration section (including
// live reload through ConfigurationReloadedEvent), the procedural
// strand-texture generator and the screen-size LOD evaluator, and binds the
// "physics_fur" capability entries. The module owns no GPU state: consumers upload the
// generated pixels through the renderer's createTexture and draw through the
// renderer's drawFur with the parameters produced here. The module performs
// no synchronization of its own.

#include <cmath>
#include <engine/physics_fur/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <engine/kernel/event_bus.hpp>
#include "internal.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace engine
{
namespace physics_fur
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
        std::fprintf(stderr, "physics_fur module: unknown [physics_fur] default_preset '%s'; using \"plush\"\n", name.c_str());
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
    // [physics_fur] keys are present. Out-of-range values are clamped with a
    // stderr warning, matching the renderer module's configuration style.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;

        // Start from the built-in defaults on every read (including live
        // reloads): a key removed from the configuration reverts to its
        // default instead of keeping the previously overlaid value.
        defaultPreset_      = FurPreset::Plush;
        defaultTextureSize_ = 256u;
        lodBands_           = LodBands{};

        defaultPreset_ = presetFromName(
            context.configurationStringInSection("physics_fur", "default_preset").value_or("plush"));

        if (const auto textureSize = context.configurationIntegerInSection("physics_fur", "texture_size");
            textureSize.has_value())
        {
            if (*textureSize < 16 || *textureSize > 2048)
            {
                std::fprintf(stderr,
                    "physics_fur module: clamping [physics_fur] texture_size %lld to range 16..2048\n",
                    static_cast<long long>(*textureSize));
            }
            defaultTextureSize_ =
                static_cast<std::uint32_t>(std::clamp<std::int64_t>(*textureSize, 16, 2048));
        }

        if (const auto maximumLayers = context.configurationIntegerInSection("physics_fur", "max_layers");
            maximumLayers.has_value())
        {
            if (*maximumLayers < 1 || *maximumLayers > 64)
            {
                std::fprintf(stderr,
                    "physics_fur module: clamping [physics_fur] max_layers %lld to range 1..64\n",
                    static_cast<long long>(*maximumLayers));
            }
            lodBands_.maximumLayers = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*maximumLayers, 1, 64));
        }

        if (const auto fadeFraction = context.configurationFloatingInSection("physics_fur", "lod_fade_fraction");
            fadeFraction.has_value())
        {
            if (!std::isfinite(*fadeFraction) || *fadeFraction <= 0.0 || *fadeFraction > 1.0)
            {
                std::fprintf(stderr,
                    "physics_fur module: clamping [physics_fur] lod_fade_fraction %g to range (0..1]\n",
                    *fadeFraction);
            }
            lodBands_.fadeFraction = static_cast<float>(std::clamp(*fadeFraction, 0.01, 1.0));
        }

        // Layer budgets per LOD band: five values, most detailed first.
        const std::vector<double> layerCounts =
            context.configurationFloatingArrayInSection("physics_fur", "lod_layer_counts");
        if (layerCounts.size() >= LodBands::bandCount)
        {
            for (std::size_t band = 0; band < LodBands::bandCount; ++band)
            {
                const long long value = static_cast<long long>(layerCounts[band]);
                if (value < 0 || value > 64)
                {
                    std::fprintf(stderr,
                        "physics_fur module: clamping [physics_fur] lod_layer_counts band %zu (%lld) to range 0..64\n",
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
            context.configurationFloatingArrayInSection("physics_fur", "lod_screen_sizes");
        if (thresholds.size() >= 4)
        {
            float previous = thresholds[0] + 1.0f;
            for (std::size_t band = 0; band < 4; ++band)
            {
                double value = thresholds[band];
                if (!std::isfinite(value) || value <= 0.0 || value >= previous)
                {
                    std::fprintf(stderr,
                        "physics_fur module: [physics_fur] lod_screen_sizes band %zu (%g) must be finite, > 0 and descending; "
                        "keeping the default\n",
                        band,
                        value);
                    // Restore the built-in table for this reload (the same
                    // defaults the LodBands struct ships).
                    const LodBands defaults;
                    for (std::size_t restored = 0; restored < 4; ++restored)
                    {
                        lodBands_.thresholds[restored] = defaults.thresholds[restored];
                    }
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
            context.configurationFloatingArrayInSection("physics_fur", "lod_height_scales");
        if (heightScales.size() >= LodBands::bandCount)
        {
            for (std::size_t band = 0; band < LodBands::bandCount; ++band)
            {
                double value = heightScales[band];
                if (!std::isfinite(value) || value <= 0.0)
                {
                    std::fprintf(stderr,
                        "physics_fur module: [physics_fur] lod_height_scales band %zu (%g) must be finite and > 0; keeping 1.0\n",
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
        FurModule *module = static_cast<FurModule *>(self);
        // Out-of-range preset ids resolve to the module's configured default
        // preset (itself Plush unless [physics_fur] default_preset says
        // otherwise), so consumers probing an unknown id get a coherent look.
        const std::uint32_t index = static_cast<std::uint32_t>(preset);
        if (index >= static_cast<std::uint32_t>(FurPreset::count))
        {
            return presetTable(static_cast<std::uint32_t>(module->defaultPreset_));
        }
        return presetTable(index);
    }

    static bool generateFurTextureImpl(
        void *self, const FurTextureRequest *request, std::uint8_t *outPixelsRgba, std::size_t outCapacityBytes)
    {
        FurModule *module = static_cast<FurModule *>(self);
        if (request == nullptr || outPixelsRgba == nullptr)
        {
            std::fprintf(stderr, "physics_fur module: generateFurTexture called with null request or buffer\n");
            return false;
        }
        // A zero edge length selects the module's configured default texture
        // size ([physics_fur] texture_size, 256 by default); explicit sizes
        // validate against the same 16..2048 range.
        const std::uint32_t width = request->width != 0u ? request->width : module->defaultTextureSize_;
        const std::uint32_t height = request->height != 0u ? request->height : module->defaultTextureSize_;
        if (width < 16u || width > 2048u || height < 16u || height > 2048u)
        {
            std::fprintf(stderr,
                "physics_fur module: fur texture size %ux%u outside 16..2048\n",
                width,
                height);
            return false;
        }
        const std::size_t requiredBytes = static_cast<std::size_t>(width) * height * 4u;
        if (outCapacityBytes < requiredBytes)
        {
            std::fprintf(stderr,
                "physics_fur module: fur texture buffer too small (%zu < %zu bytes)\n",
                outCapacityBytes,
                requiredBytes);
            return false;
        }
        generateFurTexturePixels(request->parameters, width, height, outPixelsRgba);
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
            .name                 = "physics_fur",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_fur"},
            .requiredCapabilities = {}};
    }

    ParticipantManifest manifest_;
    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    FurPreset     defaultPreset_           = FurPreset::Plush;
    std::uint32_t defaultTextureSize_      = 256u;
    LodBands      lodBands_{};

    FurCapability furCapability_{};
};

} // namespace
} // namespace physics_fur

using physics_fur::FurModule;

} // namespace engine

// The kernel's only required export: module metadata + factory. The lists
// here must match makeManifest() and module.toml.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"physics_fur"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "physics_fur",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::physics_fur::FurModule(); },
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
