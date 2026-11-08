// Frame-budget controllers: dynamic resolution scaling and adaptive quality
// presets. Both are self-contained, deterministic, and driven by CPU/GPU
// timing samples fed in once per frame by the caller (typically the
// FrameStatistics reported by the renderer capability).

#ifndef ENGINE_RENDERER_DYNAMIC_RESOLUTION_HPP
#define ENGINE_RENDERER_DYNAMIC_RESOLUTION_HPP

#include <engine/renderer/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace engine::renderer
{

// All times are milliseconds; scales are multipliers of the full resolution
// (1.0 = native). Validated in setSettings, which throws on invalid input.
struct DynamicResolutionSettings
{
    double        targetGpuMilliseconds = 16.667; // GPU time budget per frame (~60 Hz)
    float         minimumScale          = 0.5f;   // never render below half resolution
    float         maximumScale          = 1.0f;   // never above native
    float         scaleStep             = 0.05f;  // per-adjustment scale change
    double        smoothingTimeSeconds  = 0.25;   // exponential-smoothing time constant
    float         hysteresisFraction    = 0.05f;  // dead-band around the budget (avoids oscillation)
    std::uint32_t recoveryFrames        = 8;      // consecutive under-budget frames before scaling up
};

// One update() result: the new scale, the smoothed GPU time it was based on,
// and whether the scale actually changed this frame (callers recreate render
// targets only when it did).
struct DynamicResolutionSample
{
    float  scale                   = 1.0f;
    double smoothedGpuMilliseconds = 0.0;
    bool   changed                 = false;
};

// Scales render resolution to hold a GPU frame-time budget. GPU samples are
// smoothed with an exponential filter (alpha from smoothingTimeSeconds), and
// the scale steps down when the smoothed time leaves the hysteresis band
// above the budget, or steps back up after recoveryFrames of headroom.
class DynamicResolutionController final
{
public:
    explicit DynamicResolutionController(DynamicResolutionSettings settings = {})
    {
        setSettings(settings);
        scale_ = settings_.maximumScale;
    }

    // Throws std::invalid_argument when any setting is non-finite/out of
    // range (scales must be within (0, 1], minimum <= maximum, etc.).
    void setSettings(DynamicResolutionSettings settings)
    {
        const bool valid = std::isfinite(settings.targetGpuMilliseconds) && settings.targetGpuMilliseconds > 0.0
                        && std::isfinite(settings.minimumScale) && std::isfinite(settings.maximumScale)
                        && settings.minimumScale > 0.0f && settings.maximumScale >= settings.minimumScale
                        && settings.maximumScale <= 1.0f && std::isfinite(settings.scaleStep)
                        && settings.scaleStep > 0.0f && std::isfinite(settings.smoothingTimeSeconds)
                        && settings.smoothingTimeSeconds > 0.0 && std::isfinite(settings.hysteresisFraction)
                        && settings.hysteresisFraction >= 0.0f && settings.hysteresisFraction < 0.5f
                        && settings.recoveryFrames > 0;
        if (!valid)
            throw std::invalid_argument("invalid dynamic-resolution settings");
        settings_ = settings;
        scale_    = std::clamp(scale_, settings_.minimumScale, settings_.maximumScale);
        resetFilter();
    }

    [[nodiscard]] const DynamicResolutionSettings &settings() const noexcept
    {
        return settings_;
    }
    [[nodiscard]] float scale() const noexcept
    {
        return scale_;
    }
    [[nodiscard]] double smoothedGpuMilliseconds() const noexcept
    {
        return smoothedGpuMilliseconds_;
    }

    void reset(float scale = 1.0f) noexcept
    {
        if (!std::isfinite(scale))
            scale = settings_.maximumScale;
        scale_ = std::clamp(scale, settings_.minimumScale, settings_.maximumScale);
        resetFilter();
    }

    // Feeds one frame's GPU time; non-finite or non-positive samples are
    // ignored and return the current state unchanged.
    [[nodiscard]] DynamicResolutionSample update(double gpuMilliseconds, double deltaSeconds)
    {
        if (!std::isfinite(gpuMilliseconds) || gpuMilliseconds <= 0.0 || !std::isfinite(deltaSeconds)
            || deltaSeconds <= 0.0)
            return {scale_, smoothedGpuMilliseconds_, false};

        if (!hasSample_)
        {
            smoothedGpuMilliseconds_ = gpuMilliseconds;
            hasSample_               = true;
        }
        else
        {
            const double alpha = 1.0 - std::exp(-deltaSeconds / settings_.smoothingTimeSeconds);
            smoothedGpuMilliseconds_ += (gpuMilliseconds - smoothedGpuMilliseconds_) * alpha;
        }

        const double budget        = settings_.targetGpuMilliseconds;
        const double upper         = budget * (1.0 + settings_.hysteresisFraction);
        const double lower         = budget * (1.0 - settings_.hysteresisFraction);
        const float  previousScale = scale_;
        if (smoothedGpuMilliseconds_ > upper)
        {
            scale_           = std::max(settings_.minimumScale, scale_ - settings_.scaleStep);
            recoveryCounter_ = 0;
        }
        else if (smoothedGpuMilliseconds_ < lower)
        {
            if (++recoveryCounter_ >= settings_.recoveryFrames)
            {
                scale_           = std::min(settings_.maximumScale, scale_ + settings_.scaleStep);
                recoveryCounter_ = 0;
            }
        }
        else
        {
            recoveryCounter_ = 0;
        }
        return {scale_, smoothedGpuMilliseconds_, scale_ != previousScale};
    }

private:
    void resetFilter() noexcept
    {
        smoothedGpuMilliseconds_ = 0.0;
        recoveryCounter_         = 0;
        hasSample_               = false;
    }

    DynamicResolutionSettings settings_{};
    float                     scale_                   = 1.0f;
    double                    smoothedGpuMilliseconds_ = 0.0;
    std::uint32_t             recoveryCounter_         = 0;
    bool                      hasSample_               = false;
};

// Coarse classification of which side of the pipeline is the bottleneck.
enum class PerformanceBottleneck : std::uint8_t
{
    Unknown = 0,
    Cpu,
    Gpu,
    Combined
};

// Quality presets step between QualityPreset::Low and High/Ultra based on
// CPU and GPU frame times. Presets are engine-defined (see
// shared_types/include/engine/renderer/device_profile.hpp); Auto is not a
// valid running preset and is resolved to Medium.
struct AdaptiveQualitySettings
{
    double        targetCpuMilliseconds = 16.667;
    double        targetGpuMilliseconds = 16.667;
    double        overloadFraction      = 0.08; // above budget * (1 + this) counts as overloaded
    double        headroomFraction      = 0.18; // below budget * (1 - this) counts as headroom
    QualityPreset minimumPreset         = QualityPreset::Low;
    QualityPreset maximumPreset         = QualityPreset::Ultra;
    std::uint32_t degradeAfterFrames    = 4;  // consecutive overloaded frames before stepping down
    std::uint32_t recoverAfterFrames    = 60; // consecutive headroom frames before stepping up
};

// One update() result: current preset, detected bottleneck, per-side budget
// flags, and whether the preset changed this frame.
struct AdaptiveQualitySample
{
    QualityPreset         preset        = QualityPreset::Medium;
    PerformanceBottleneck bottleneck    = PerformanceBottleneck::Unknown;
    bool                  cpuOverBudget = false;
    bool                  gpuOverBudget = false;
    bool                  changed       = false;
};

// Steps the quality preset down under sustained overload and back up after
// sustained headroom. GPU samples are optional: when gpuTimingAvailable is
// false the controller judges only the CPU side. All timing inputs are
// milliseconds.
class AdaptiveQualityController final
{
public:
    explicit AdaptiveQualityController(
        AdaptiveQualitySettings settings = {}, QualityPreset initial = QualityPreset::Medium)
    {
        setSettings(settings);
        reset(initial);
    }

    // Throws std::invalid_argument for non-finite/non-positive targets,
    // fractions outside [0, 1), or an inverted preset range.
    void setSettings(const AdaptiveQualitySettings &settings)
    {
        const auto validPreset
            = [](QualityPreset value) { return value >= QualityPreset::Low && value <= QualityPreset::Ultra; };
        if (!std::isfinite(settings.targetCpuMilliseconds) || settings.targetCpuMilliseconds <= 0.0
            || !std::isfinite(settings.targetGpuMilliseconds) || settings.targetGpuMilliseconds <= 0.0
            || !std::isfinite(settings.overloadFraction) || settings.overloadFraction < 0.0
            || settings.overloadFraction >= 1.0 || !std::isfinite(settings.headroomFraction)
            || settings.headroomFraction <= 0.0 || settings.headroomFraction >= 1.0
            || !validPreset(settings.minimumPreset) || !validPreset(settings.maximumPreset)
            || settings.minimumPreset > settings.maximumPreset || settings.degradeAfterFrames == 0
            || settings.recoverAfterFrames == 0)
            throw std::invalid_argument("invalid adaptive-quality settings");
        settings_       = settings;
        preset_         = clampPreset(preset_);
        overloadFrames_ = recoveryFrames_ = 0;
    }

    // Auto (or out-of-range) presets resolve to Medium.
    void reset(QualityPreset initial) noexcept
    {
        if (initial == QualityPreset::Auto || initial < QualityPreset::Low || initial > QualityPreset::Ultra)
            initial = QualityPreset::Medium;
        preset_         = clampPreset(initial);
        overloadFrames_ = recoveryFrames_ = 0;
    }

    [[nodiscard]] QualityPreset preset() const noexcept
    {
        return preset_;
    }
    [[nodiscard]] const AdaptiveQualitySettings &settings() const noexcept
    {
        return settings_;
    }

    [[nodiscard]] AdaptiveQualitySample update(
        double cpuMilliseconds, double gpuMilliseconds, bool gpuTimingAvailable) noexcept
    {
        const bool cpuValid = std::isfinite(cpuMilliseconds) && cpuMilliseconds > 0.0;
        const bool gpuValid = gpuTimingAvailable && std::isfinite(gpuMilliseconds) && gpuMilliseconds > 0.0;
        if (!cpuValid && !gpuValid)
            return {preset_, PerformanceBottleneck::Unknown, false, false, false};

        const bool cpuOver
            = cpuValid && cpuMilliseconds > settings_.targetCpuMilliseconds * (1.0 + settings_.overloadFraction);
        const bool gpuOver
            = gpuValid && gpuMilliseconds > settings_.targetGpuMilliseconds * (1.0 + settings_.overloadFraction);
        const bool cpuHeadroom
            = !cpuValid || cpuMilliseconds < settings_.targetCpuMilliseconds * (1.0 - settings_.headroomFraction);
        const bool gpuHeadroom
            = !gpuValid || gpuMilliseconds < settings_.targetGpuMilliseconds * (1.0 - settings_.headroomFraction);

        PerformanceBottleneck bottleneck = PerformanceBottleneck::Unknown;
        if (cpuOver && gpuOver)
            bottleneck = PerformanceBottleneck::Combined;
        else if (cpuOver)
            bottleneck = PerformanceBottleneck::Cpu;
        else if (gpuOver)
            bottleneck = PerformanceBottleneck::Gpu;
        else if (cpuValid && gpuValid)
            bottleneck = cpuMilliseconds >= gpuMilliseconds ? PerformanceBottleneck::Cpu : PerformanceBottleneck::Gpu;
        else if (cpuValid)
            bottleneck = PerformanceBottleneck::Cpu;
        else if (gpuValid)
            bottleneck = PerformanceBottleneck::Gpu;

        const QualityPreset previous = preset_;
        if (cpuOver || gpuOver)
        {
            recoveryFrames_ = 0;
            if (++overloadFrames_ >= settings_.degradeAfterFrames)
            {
                preset_         = stepDown(preset_);
                overloadFrames_ = 0;
            }
        }
        else if (cpuHeadroom && gpuHeadroom)
        {
            overloadFrames_ = 0;
            if (++recoveryFrames_ >= settings_.recoverAfterFrames)
            {
                preset_         = stepUp(preset_);
                recoveryFrames_ = 0;
            }
        }
        else
        {
            overloadFrames_ = recoveryFrames_ = 0;
        }
        return {preset_, bottleneck, cpuOver, gpuOver, previous != preset_};
    }

private:
    QualityPreset clampPreset(QualityPreset value) const noexcept
    {
        return static_cast<QualityPreset>(std::clamp(static_cast<std::uint32_t>(value),
            static_cast<std::uint32_t>(settings_.minimumPreset),
            static_cast<std::uint32_t>(settings_.maximumPreset)));
    }
    QualityPreset stepDown(QualityPreset value) const noexcept
    {
        if (value <= settings_.minimumPreset)
            return settings_.minimumPreset;
        return static_cast<QualityPreset>(static_cast<std::uint32_t>(value) - 1u);
    }
    QualityPreset stepUp(QualityPreset value) const noexcept
    {
        if (value >= settings_.maximumPreset)
            return settings_.maximumPreset;
        return static_cast<QualityPreset>(static_cast<std::uint32_t>(value) + 1u);
    }

    AdaptiveQualitySettings settings_{};
    QualityPreset           preset_         = QualityPreset::Medium;
    std::uint32_t           overloadFrames_ = 0;
    std::uint32_t           recoveryFrames_ = 0;
};

} // namespace engine::renderer

#endif
