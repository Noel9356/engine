// physics_foliage module implementation.
//
// Per-instance bend springs driven by wind: every instance carries a 2D
// horizontal bend vector with a spring-damper toward the wind-derived
// target plus a phase-offset sway cycle; radial disturbances inject bend
// velocity that springs back. Wind comes from the physics_fields capability
// when a wind handle is bound, else from the configured ambient breeze.
// Stepping happens in whole fixed steps, so results depend on the step
// count only. Simulation LOD: every field carries an importance score;
// fields below the Full fidelity band update on a staggered cadence whose
// due steps cover the whole interval of wall time, and Cached fields freeze.
// Per-instance wind samples amortize over a configurable stride so large
// fields do not call the wind capability once per instance per step.

#include "records.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_foliage/capabilities.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_foliage
{
namespace
{

class FoliageModule final : public ModuleInterface
{
public:
    FoliageModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }


    // Refreshes the pre-normalized ambient breeze velocity after a direction
    // or speed change.
    void normalizeAmbientWind()
    {
        const double length = std::sqrt(ambientWindDirection_[0] * ambientWindDirection_[0]
                                        + ambientWindDirection_[1] * ambientWindDirection_[1]);
        if (length < 1.0e-9)
        {
            ambientWindVelocity_[0] = 0.0;
            ambientWindVelocity_[1] = 0.0;
            return;
        }
        ambientWindVelocity_[0] = ambientWindDirection_[0] / length * ambientWindSpeed_;
        ambientWindVelocity_[1] = ambientWindDirection_[1] / length * ambientWindSpeed_;
    }

    void construct(ParticipantContext &context) override
    {
        if (const auto value = context.configurationIntegerInSection("physics_foliage", "maximum_fields");
            value && *value > 0 && *value <= 4096)
            settings_.maximumFields = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_foliage", "maximum_instances");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumInstances = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_foliage", "maximum_instances_per_field");
            value && *value > 0 && *value <= 1'000'000)
            settings_.maximumInstancesPerField = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_foliage", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_foliage", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingArrayInSection("physics_foliage", "ambient_wind_direction");
            value.size() == 2 && std::isfinite(value[0]) && std::isfinite(value[1]))
        {
            ambientWindDirection_[0] = value[0];
            ambientWindDirection_[1] = value[1];
        }
        if (const auto value = context.configurationFloatingInSection("physics_foliage", "ambient_wind_speed");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            ambientWindSpeed_ = *value;
        normalizeAmbientWind();
        if (const auto value = context.configurationFloatingInSection("physics_foliage", "sway_amplitude");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.swayAmplitude = *value;
        if (const auto value = context.configurationIntegerInSection("physics_foliage", "wind_sample_stride");
            value && *value >= 1 && *value <= 65536)
            settings_.windSampleStride = static_cast<std::uint32_t>(*value);
        readLodSettings(context);

        fields_ = context.tryAcquireCapability<engine::physics_fields::FieldsCapability>();
        if (fields_ != nullptr && fields_->abi != engine::physics_fields::physicsFieldsCapabilityAbi)
        {
            fields_ = nullptr;
        }
    }

    void start() override
    {}

    void stop() override
    {
        fieldRecords_.clear();
        fieldIndexByHandle_.clear();
        nextFieldHandle_ = 1;
        totalInstances_  = 0;
        statistics_      = FoliageStatistics{};
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == FoliageCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_foliage",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_foliage"},
            .requiredCapabilities = {},
            .optionalCapabilities = {"physics_fields"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_foliage");
    }

    static void setError(FoliageError *output, FoliageError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }



    FieldRecord *findField(FieldHandle handle)
    {
        const auto iterator = fieldIndexByHandle_.find(handle);
        return iterator != fieldIndexByHandle_.end() ? &fieldRecords_[iterator->second] : nullptr;
    }

    // Wind velocity sample for one instance: the bound wind field when
    // available, else the ambient breeze. A dead or unknown wind-field
    // handle (the fields module may have destroyed it) falls back to the
    // ambient breeze instead of freezing the last cached sample forever.
    bool sampleWindAt(const FieldRecord &field, const double position[3], double outVelocity[3]) const
    {
        if (fields_ != nullptr && field.windFieldHandle != 0
            && fields_->sampleWindVelocity(fields_->self, field.windFieldHandle, position, field.swayClock, outVelocity))
        {
            return true;
        }
        outVelocity[0] = ambientWindVelocity_[0];
        outVelocity[1] = 0.0;
        outVelocity[2] = ambientWindVelocity_[1];
        return true;
    }

    void stepField(FieldRecord &field, double stepDeltaSeconds, std::uint64_t stepCounter)
    {
        // Per-instance spring-damper toward the wind target with the phase
        // cycle; semi-implicit Euler at the fixed step. Wind samples
        // amortize over the configured stride: each instance refreshes its
        // cached sample when (stepCounter + instance index) mod stride hits
        // zero, so large fields spread the wind calls across steps while the
        // bend target stays a bounded number of steps stale.
        const std::uint32_t stride = settings_.windSampleStride;
        for (std::size_t instanceIndex = 0; instanceIndex < field.instances.size(); ++instanceIndex)
        {
            InstanceState &instance = field.instances[instanceIndex];
            if (stride <= 1 || !instance.windCached
                || (stepCounter + static_cast<std::uint64_t>(instanceIndex)) % stride == 0u)
            {
                sampleWindAt(field, instance.description.position, instance.cachedWind);
                instance.windCached = true;
            }
            const double(&windVelocity)[3] = instance.cachedWind;
            const double windSpeedHorizontal
                = std::sqrt(windVelocity[0] * windVelocity[0] + windVelocity[2] * windVelocity[2]);

            // Wind target: tip displacement grows with wind speed, shrinks
            // with stiffness, and scales with the square of the height
            // (taller instances lever further).
            const double windFactor = (1.0 - std::clamp(instance.description.stiffness, 0.0, 1.0)) * field.description.bendScale;
            const double targetMagnitude = windSpeedHorizontal * windFactor * instance.description.height * instance.description.height;

            // Phase-offset sway: a traveling wave across the field so
            // instances do not move in unison.
            const double phase = instance.description.phaseSeed
                                 + field.swayClock * field.description.swayFrequency * 6.283185307179586;
            const double swayMagnitude
                = std::sin(phase) * settings_.swayAmplitude * windSpeedHorizontal * windFactor
                  * instance.description.swayScale;

            double target[2] = {0.0, 0.0};
            if (windSpeedHorizontal > 1.0e-9 && std::isfinite(windSpeedHorizontal))
            {
                // The horizontal speed of a validated wind sample is finite
                // but its square can still overflow for extreme speeds; the
                // isfinite guard keeps the normalized direction and the
                // target magnitude from producing 0 x Inf = NaN bends.
                const double directionX = windVelocity[0] / windSpeedHorizontal;
                const double directionZ = windVelocity[2] / windSpeedHorizontal;
                const double magnitude = targetMagnitude + swayMagnitude;
                if (std::isfinite(magnitude))
                {
                    target[0] = directionX * magnitude;
                    target[1] = directionZ * magnitude;
                }
            }

            // Stability-aware spring gains: a LOD due step covers interval x
            // the caller's delta. Semi-implicit Euler on the damped spring is
            // stable inside the JOINT Jury region h^2 k + 2 h c < 4 - clamping
            // each term to its own bound (h^2 k < 3.2 and h c < 1.8)
            // satisfies neither bound jointly (3.2 + 2 x 1.8 = 6.8 > 4) and
            // lets contract-valid gains diverge. Stiffness clamps first,
            // then damping clamps to the residual the stiffness left, so the
            // pair always lands inside the region with margin (sum <= 3.6).
            const double stiffness = std::min(std::max(field.description.springStiffness, 1.0e-6),
                3.2 / (stepDeltaSeconds * stepDeltaSeconds));
            const double dampingStabilityBudget = 3.6 - stiffness * stepDeltaSeconds * stepDeltaSeconds;
            const double damping = std::min(std::max(field.description.springDamping, 0.0),
                std::min(1.8 / stepDeltaSeconds, dampingStabilityBudget / (2.0 * stepDeltaSeconds)));
            for (std::uint32_t axis = 0; axis < 2; ++axis)
            {
                const double acceleration
                    = (target[axis] - instance.bend[axis]) * stiffness - instance.bendVelocity[axis] * damping;
                instance.bendVelocity[axis] += acceleration * stepDeltaSeconds;
                instance.bend[axis] += instance.bendVelocity[axis] * stepDeltaSeconds;
            }

            // Clamp the bend to half the height: foliage bends, never folds.
            const double bendLengthSquared = instance.bend[0] * instance.bend[0] + instance.bend[1] * instance.bend[1];
            const double maximumBend       = instance.description.height * 0.5;
            if (bendLengthSquared > maximumBend * maximumBend)
            {
                const double scale = maximumBend / std::sqrt(bendLengthSquared);
                instance.bend[0] *= scale;
                instance.bend[1] *= scale;
            }
        }
        field.swayClock += stepDeltaSeconds;
    }

    // ---- capability implementations ---------------------------------------

    FieldHandle createFieldImpl(
        const FoliageFieldDescription *description, std::uint32_t windFieldHandle, FoliageError *error)
    {
        setError(error, FoliageError::None);
        if (description == nullptr || (description->instances == nullptr && description->instanceCount > 0))
        {
            setError(error, FoliageError::InvalidDescription);
            return invalidField;
        }
        if (fieldRecords_.size() >= settings_.maximumFields || totalInstances_ >= settings_.maximumInstances
            || description->instanceCount > settings_.maximumInstancesPerField
            || totalInstances_ + description->instanceCount > settings_.maximumInstances)
        {
            setError(error, FoliageError::CapacityExceeded);
            return invalidField;
        }
        if (!std::isfinite(description->swayFrequency) || description->swayFrequency < 0.0
            || !std::isfinite(description->bendScale) || description->bendScale < 0.0
            || !std::isfinite(description->springStiffness) || description->springStiffness <= 0.0
            || !std::isfinite(description->springDamping) || description->springDamping < 0.0)
        {
            setError(error, FoliageError::InvalidDescription);
            return invalidField;
        }
        for (std::uint32_t instance = 0; instance < description->instanceCount; ++instance)
        {
            const FoliageInstance &candidate = description->instances[instance];
            if (!std::isfinite(candidate.position[0]) || !std::isfinite(candidate.position[1])
                || !std::isfinite(candidate.position[2]) || !std::isfinite(candidate.height)
                || candidate.height <= 0.0 || !std::isfinite(candidate.stiffness) || candidate.stiffness < 0.0
                || candidate.stiffness > 1.0 || !std::isfinite(candidate.phaseSeed)
                || !std::isfinite(candidate.swayScale) || candidate.swayScale < 0.0)
            {
                setError(error, FoliageError::InvalidDescription);
                return invalidField;
            }
        }
        if (nextFieldHandle_ == invalidField)
        {
            setError(error, FoliageError::CapacityExceeded);
            return invalidField;
        }
        FieldRecord field;
        field.handle          = nextFieldHandle_;
        nextFieldHandle_      = nextFieldHandle_ == std::numeric_limits<FieldHandle>::max()
                                    ? invalidField
                                    : nextFieldHandle_ + 1;
        field.windFieldHandle = windFieldHandle;
        field.description     = *description;
        field.instances.resize(description->instanceCount);
        for (std::uint32_t instance = 0; instance < description->instanceCount; ++instance)
        {
            field.instances[instance].description = description->instances[instance];
        }
        // The record owns the instance copies; the stored description keeps
        // no view of the caller's array (the rope/cloth/planter records drop
        // borrowed pointers the same way).
        field.description.instances    = nullptr;
        field.description.instanceCount = 0;
        totalInstances_ += description->instanceCount;
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so fields sharing an interval spread their work across
        // steps.
        field.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        fieldRecords_.push_back(std::move(field));
        fieldIndexByHandle_.emplace(fieldRecords_.back().handle, fieldRecords_.size() - 1);
        statistics_.fieldCount    = static_cast<std::uint32_t>(fieldRecords_.size());
        statistics_.instanceCount = static_cast<std::uint32_t>(totalInstances_);
        return fieldRecords_.back().handle;
    }

    void destroyFieldImpl(FieldHandle handle)
    {
        const auto iterator = fieldIndexByHandle_.find(handle);
        if (iterator == fieldIndexByHandle_.end())
        {
            return;
        }
        totalInstances_ -= fieldRecords_[iterator->second].instances.size();
        const std::size_t index = iterator->second;
        fieldIndexByHandle_.erase(iterator);
        if (index + 1 != fieldRecords_.size())
        {
            fieldRecords_[index]                                   = fieldRecords_.back();
            fieldIndexByHandle_[fieldRecords_[index].handle]       = index;
        }
        fieldRecords_.pop_back();
        statistics_.fieldCount    = static_cast<std::uint32_t>(fieldRecords_.size());
        statistics_.instanceCount = static_cast<std::uint32_t>(totalInstances_);
    }

    bool copyBendsImpl(FieldHandle handle, FoliageBend *outBends, std::uint32_t capacity, std::uint32_t *outFilled,
        std::uint32_t *outRequired)
    {
        FieldRecord *field = findField(handle);
        if (field == nullptr || outRequired == nullptr)
        {
            return false;
        }
        const std::uint32_t total = static_cast<std::uint32_t>(field->instances.size());
        *outRequired = total;
        if (outFilled != nullptr)
        {
            *outFilled = 0;
        }
        if (outBends == nullptr)
        {
            return false;
        }
        std::uint32_t filled = 0;
        for (std::uint32_t index = 0; index < total && filled < capacity; ++index)
        {
            const InstanceState &instance = field->instances[index];
            const double bendLength = std::sqrt(instance.bend[0] * instance.bend[0] + instance.bend[1] * instance.bend[1]);
            FoliageBend bend;
            if (bendLength > 1.0e-9)
            {
                bend.bendDirectionX   = instance.bend[0] / bendLength;
                bend.bendDirectionZ   = instance.bend[1] / bendLength;
                bend.bendAngleRadians = std::atan2(bendLength, instance.description.height);
            }
            outBends[filled++] = bend;
        }
        if (outFilled != nullptr)
        {
            *outFilled = filled;
        }
        return true;
    }

    bool applyRadialDisturbanceImpl(FieldHandle handle, const double position[3], double radius, double strength)
    {
        FieldRecord *field = findField(handle);
        if (field == nullptr || position == nullptr || !std::isfinite(radius) || radius <= 0.0
            || !std::isfinite(strength))
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        for (InstanceState &instance : field->instances)
        {
            const double offsetX = instance.description.position[0] - position[0];
            const double offsetZ = instance.description.position[2] - position[2];
            const double distance = std::sqrt(offsetX * offsetX + offsetZ * offsetZ);
            if (distance > radius)
            {
                continue;
            }
            const double falloff = 1.0 - distance / radius;
            double directionX = 0.0;
            double directionZ = 1.0;
            if (distance > 1.0e-6)
            {
                directionX = offsetX / distance;
                directionZ = offsetZ / distance;
            }
            // Disturbances inject bend velocity scaled by the instance
            // flexibility; stiffer instances barely move.
            const double flexibility = 1.0 - std::clamp(instance.description.stiffness, 0.0, 1.0);
            instance.bendVelocity[0] += directionX * strength * falloff * flexibility;
            instance.bendVelocity[1] += directionZ * strength * falloff * flexibility;
        }
        field->disturbanceTotal += 1;
        statistics_.disturbanceTotal += 1;
        return true;
    }

    bool stepImpl(double fixedDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        bool diverged = false;
        // The wind-sample stride rides a module-level step counter so the
        // refresh phases advance every step whether LOD is on or off (a
        // per-field cadence counter would freeze at zero with lod disabled
        // and hold most cached samples forever).
        const std::uint64_t stepCounter = stepCounter_++;
        for (FieldRecord &field : fieldRecords_)
        {
            // Simulation LOD: resolve the band, refresh the staggered
            // cadence, and let one due step cover the whole interval of
            // wall time. Cached fields freeze. Without lod_enabled every
            // field runs Full every step.
            const std::uint32_t interval = engine::util::simulationLodResolvePass(field.lodBand,
                field.cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                field.importance,
                &field.lodSkippedSteps);
            if (interval == 0u)
            {
                continue;
            }
            stepField(field, fixedDeltaSeconds * static_cast<double>(interval), stepCounter);
            for (InstanceState &instance : field.instances)
            {
                if (!std::isfinite(instance.bend[0]) || !std::isfinite(instance.bend[1])
                    || !std::isfinite(instance.bendVelocity[0]) || !std::isfinite(instance.bendVelocity[1]))
                {
                    // A diverging bend resets to upright instead of
                    // poisoning the field; the step still reports failure.
                    instance.bend[0]         = 0.0;
                    instance.bend[1]         = 0.0;
                    instance.bendVelocity[0] = 0.0;
                    instance.bendVelocity[1] = 0.0;
                    instance.windCached      = false;
                    diverged = true;
                }
            }
        }
        return !diverged;
    }

    std::uint32_t simulateImpl(double elapsedSeconds)
    {
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return accumulator_.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, fixedDelta]() { return stepImpl(fixedDelta); });
    }

    bool setFieldImportanceImpl(FieldHandle handle, double importance)
    {
        FieldRecord *field = findField(handle);
        if (field == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        field->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool foliageStatisticsImpl(FoliageStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        statistics_.lodSkippedSteps = 0;
        for (const FieldRecord &field : fieldRecords_)
        {
            statistics_.lodSkippedSteps += field.lodSkippedSteps;
        }
        *outStatistics = statistics_;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsFoliageCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_FOLIAGE_BIND(field, method)                                                                     \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<FoliageModule *>(self)->method(args...); }
        ENGINE_PHYSICS_FOLIAGE_BIND(createField, createFieldImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(destroyField, destroyFieldImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(copyBends, copyBendsImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(applyRadialDisturbance, applyRadialDisturbanceImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(step, stepImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(foliageStatistics, foliageStatisticsImpl);
        ENGINE_PHYSICS_FOLIAGE_BIND(setFieldImportance, setFieldImportanceImpl);
#undef ENGINE_PHYSICS_FOLIAGE_BIND
    }


    ParticipantManifest manifest_;
    FoliageCapability   capability_{};

    engine::physics_fields::FieldsCapability *fields_ = nullptr;

    FoliageSettings settings_;

    FoliageStatistics statistics_{};

    // Ambient breeze velocity (normalized direction x speed), normalized
    // once here instead of per instance per step.
    double ambientWindVelocity_[2] = {0.0, 0.0};
    double ambientWindDirection_[2] = {1.0, 0.0};
    double ambientWindSpeed_        = 0.0;

    engine::util::StepAccumulator accumulator_;

    std::size_t totalInstances_ = 0;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;

    // Module-level fixed-step counter for the per-instance wind-sample
    // stride (advances every stepImpl call, LOD or not).
    std::uint64_t stepCounter_ = 0;

    std::vector<FieldRecord>                       fieldRecords_;
    std::unordered_map<FieldHandle, std::size_t>   fieldIndexByHandle_;
    FieldHandle                                    nextFieldHandle_ = 1;
};

} // namespace
} // namespace engine::physics_foliage

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_foliage capability, declaring no requirements, and optionally
// using physics_fields for wind sampling.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const    provided[] = {"physics_foliage"};
    static const char *const    optional[] = {"physics_fields"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_foliage",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_foliage::FoliageModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = optional,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
