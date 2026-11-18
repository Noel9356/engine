// physics_foliage module implementation.
//
// Per-instance bend springs driven by wind: every instance carries a 2D
// horizontal bend vector with a spring-damper toward the wind-derived
// target plus a phase-offset sway cycle; radial disturbances inject bend
// velocity that springs back. Wind comes from the physics_fields capability
// when a wind handle is bound, else from the configured ambient breeze.
// Stepping happens in whole fixed steps, so results depend on the step
// count only.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_foliage/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>

#include <algorithm>
#include <chrono>
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

namespace math = engine::physics_rigid::math;

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
            .requiredCapabilities = {"physics_rigid"},
            .optionalCapabilities = {"physics_fields"}};
    }

    static void setError(FoliageError *output, FoliageError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
    }

    struct InstanceState
    {
        FoliageInstance description{};

        // Bend state: horizontal displacement of the tip (x, z) with its
        // velocity, integrated by the spring-damper.
        double bend[2]         = {0.0, 0.0};
        double bendVelocity[2] = {0.0, 0.0};
    };

    struct FieldRecord
    {
        FieldHandle handle = invalidField;

        std::uint32_t windFieldHandle = 0; // physics_fields wind handle, 0 = ambient breeze

        FoliageFieldDescription description{};

        std::vector<InstanceState> instances;

        double swayClock = 0.0;

        std::uint64_t disturbanceTotal = 0;
    };

    FieldRecord *findField(FieldHandle handle)
    {
        const auto iterator = fieldIndexByHandle_.find(handle);
        return iterator != fieldIndexByHandle_.end() ? &fieldRecords_[iterator->second] : nullptr;
    }

    // Wind velocity sample for one instance: the bound wind field when
    // available, else the ambient breeze.
    bool sampleWindAt(const FieldRecord &field, const double position[3], double outVelocity[3]) const
    {
        if (fields_ != nullptr && field.windFieldHandle != 0)
        {
            return fields_->sampleWindVelocity(fields_->self, field.windFieldHandle, position, field.swayClock, outVelocity);
        }
        const double length = std::sqrt(ambientWindDirection_[0] * ambientWindDirection_[0]
                                        + ambientWindDirection_[1] * ambientWindDirection_[1]);
        if (length < 1.0e-9)
        {
            outVelocity[0] = 0.0;
            outVelocity[1] = 0.0;
            outVelocity[2] = 0.0;
            return true;
        }
        outVelocity[0] = ambientWindDirection_[0] / length * ambientWindSpeed_;
        outVelocity[1] = 0.0;
        outVelocity[2] = ambientWindDirection_[1] / length * ambientWindSpeed_;
        return true;
    }

    void stepField(FieldRecord &field, double fixedDeltaSeconds)
    {
        // Per-instance spring-damper toward the wind target with the phase
        // cycle; semi-implicit Euler at the fixed step.
        for (InstanceState &instance : field.instances)
        {
            double windVelocity[3] = {0.0, 0.0, 0.0};
            sampleWindAt(field, instance.description.position, windVelocity);
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
                = std::sin(phase) * 0.15 * windSpeedHorizontal * windFactor * instance.description.swayScale;

            double target[2] = {0.0, 0.0};
            if (windSpeedHorizontal > 1.0e-9)
            {
                target[0] = windVelocity[0] / windSpeedHorizontal * (targetMagnitude + swayMagnitude);
                target[1] = windVelocity[2] / windSpeedHorizontal * (targetMagnitude + swayMagnitude);
            }

            const double stiffness = std::max(field.description.springStiffness, 1.0e-6);
            const double damping   = std::max(field.description.springDamping, 0.0);
            for (std::uint32_t axis = 0; axis < 2; ++axis)
            {
                const double acceleration
                    = (target[axis] - instance.bend[axis]) * stiffness - instance.bendVelocity[axis] * damping;
                instance.bendVelocity[axis] += acceleration * fixedDeltaSeconds;
                instance.bend[axis] += instance.bendVelocity[axis] * fixedDeltaSeconds;
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
        field.swayClock += fixedDeltaSeconds;
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
        totalInstances_ += description->instanceCount;
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
        for (FieldRecord &field : fieldRecords_)
        {
            stepField(field, fixedDeltaSeconds);
            for (const InstanceState &instance : field.instances)
            {
                if (!std::isfinite(instance.bend[0]) || !std::isfinite(instance.bend[1]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    std::uint32_t simulateImpl(double elapsedSeconds)
    {
        if (!std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        simulatedTimeBudget_ += elapsedSeconds;
        std::uint32_t executed = 0;
        while (simulatedTimeBudget_ >= fixedDelta && executed < settings_.maximumStepsPerSimulate)
        {
            simulatedTimeBudget_ -= fixedDelta;
            if (!stepImpl(fixedDelta))
            {
                break;
            }
            ++executed;
        }
        if (executed == settings_.maximumStepsPerSimulate)
        {
            simulatedTimeBudget_ = 0.0;
        }
        return executed;
    }

    bool foliageStatisticsImpl(FoliageStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
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
#undef ENGINE_PHYSICS_FOLIAGE_BIND
    }

    struct FoliageSettings
    {
        double        fixedRateHz               = 60.0;
        std::uint32_t maximumStepsPerSimulate   = 8;
        std::uint32_t maximumFields             = 64;
        std::uint32_t maximumInstances          = 2000000;
        std::uint32_t maximumInstancesPerField  = 500000;
    };

    ParticipantManifest manifest_;
    FoliageCapability   capability_{};

    engine::physics_fields::FieldsCapability *fields_ = nullptr;

    FoliageSettings settings_;

    FoliageStatistics statistics_{};

    double ambientWindDirection_[2] = {1.0, 0.0};
    double ambientWindSpeed_        = 0.0;

    double simulatedTimeBudget_ = 0.0;

    std::size_t totalInstances_ = 0;

    std::vector<FieldRecord>                       fieldRecords_;
    std::unordered_map<FieldHandle, std::size_t>   fieldIndexByHandle_;
    FieldHandle                                    nextFieldHandle_ = 1;
};

} // namespace
} // namespace engine::physics_foliage

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_foliage capability, requiring physics_rigid, and optionally
// using physics_fields for wind sampling.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const    provided[] = {"physics_foliage"};
    static const char *const    required[] = {"physics_rigid"};
    static const char *const    optional[] = {"physics_fields"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_foliage",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_foliage::FoliageModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = optional,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
