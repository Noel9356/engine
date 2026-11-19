// physics_fluid module implementation.
//
// Smoothed-particle hydrodynamics over a physics_rigid world. The SPH
// pipeline itself lives in sph_step.cpp; this file owns the fluid records,
// the capability surface, and the module lifecycle. Stepping happens in
// whole fixed steps with internal substeps, so the simulation is
// independent of frame rate; simulation LOD reduces the substep count for
// unimportant fluids and a reduced-cadence due step integrates the
// accumulated fixed deltas in one pass, so fidelity changes never change
// the wall-clock speed of the fluid.

#include "records.hpp"
#include "sph_step.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
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

namespace engine::physics_fluid
{
namespace
{

class FluidModule final : public ModuleInterface
{
public:
    FluidModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_fluids");
            value && *value > 0 && *value <= 4096)
            settings_.maximumFluids = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_particles");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumParticles = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_particles_per_fluid");
            value && *value > 0 && *value <= 1'000'000)
            settings_.maximumParticlesPerFluid = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "substeps_per_step");
            value && *value > 0 && *value <= 64)
            settings_.substepsPerStep = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "maximum_velocity");
            value && *value > 0.0 && *value <= 10'000.0 && std::isfinite(*value))
            settings_.maximumVelocity = *value;
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_neighbors_per_particle");
            value && *value >= 8 && *value <= 4096)
            settings_.maximumNeighborsPerParticle = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "bounds_restitution");
            value && *value >= 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.boundsRestitution = *value;
        readLodSettings(context);

        rigid_ = context.tryAcquireCapability<engine::physics_rigid::PhysicsRigidCapability>();
        if (rigid_ != nullptr && rigid_->abi != engine::physics_rigid::physicsRigidCapabilityAbi)
        {
            rigid_ = nullptr;
        }
    }

    void start() override
    {}

    void stop() override
    {
        fluids_.clear();
        fluidIndexByHandle_.clear();
        nextFluidHandle_ = 1;
        totalParticles_  = 0;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == FluidCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_fluid",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_fluid"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_fluid");
    }

    static void setError(FluidError *output, FluidError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    FluidRecord *findFluid(FluidHandle handle)
    {
        const auto iterator = fluidIndexByHandle_.find(handle);
        return iterator != fluidIndexByHandle_.end() ? &fluids_[iterator->second] : nullptr;
    }

    // ---- capability implementations ---------------------------------------

    FluidHandle createFluidImpl(
        engine::physics_rigid::WorldHandle world, const FluidDescription *description, FluidError *error)
    {
        setError(error, FluidError::None);
        if (rigid_ == nullptr)
        {
            setError(error, FluidError::InvalidHandle);
            return invalidFluid;
        }
        if (description == nullptr)
        {
            setError(error, FluidError::InvalidDescription);
            return invalidFluid;
        }
        if (fluids_.size() >= settings_.maximumFluids || totalParticles_ >= settings_.maximumParticles)
        {
            setError(error, FluidError::CapacityExceeded);
            return invalidFluid;
        }
        if (!std::isfinite(description->interactionRadius) || description->interactionRadius <= 0.0
            || !std::isfinite(description->particleRadius) || description->particleRadius <= 0.0
            || !std::isfinite(description->particleMass) || description->particleMass <= 0.0
            || !std::isfinite(description->restDensity) || description->restDensity <= 0.0
            || !std::isfinite(description->gasStiffness) || description->gasStiffness < 0.0
            || !std::isfinite(description->viscosity) || description->viscosity < 0.0
            || !std::isfinite(description->damping) || description->damping < 0.0
            || (description->useBounds
                && (!std::isfinite(description->boundsMinimum[0]) || !std::isfinite(description->boundsMinimum[1])
                    || !std::isfinite(description->boundsMinimum[2]) || !std::isfinite(description->boundsMaximum[0])
                    || !std::isfinite(description->boundsMaximum[1]) || !std::isfinite(description->boundsMaximum[2])
                    || description->boundsMinimum[0] >= description->boundsMaximum[0]
                    || description->boundsMinimum[1] >= description->boundsMaximum[1]
                    || description->boundsMinimum[2] >= description->boundsMaximum[2]
                    // Every axis must span at least the particle diameter, or
                    // the two inset clamps would cross and fight on a thinner
                    // box.
                    || description->boundsMaximum[0] - description->boundsMinimum[0] < 2.0 * description->particleRadius
                    || description->boundsMaximum[1] - description->boundsMinimum[1] < 2.0 * description->particleRadius
                    || description->boundsMaximum[2] - description->boundsMinimum[2] < 2.0 * description->particleRadius)))
        {
            setError(error, FluidError::InvalidDescription);
            return invalidFluid;
        }
        if (nextFluidHandle_ == invalidFluid)
        {
            setError(error, FluidError::CapacityExceeded);
            return invalidFluid;
        }
        FluidRecord fluid;
        fluid.handle = nextFluidHandle_;
        nextFluidHandle_
            = nextFluidHandle_ == std::numeric_limits<FluidHandle>::max() ? invalidFluid : nextFluidHandle_ + 1;
        fluid.world       = world;
        fluid.description = *description;
        // Capturing the gravity doubles as the world-handle validation: a
        // world the rigid module does not know is rejected here instead of
        // silently falling back to the default gravity.
        if (!rigid_->copyWorldGravity(rigid_->self, world, fluid.gravity))
        {
            setError(error, FluidError::InvalidHandle);
            return invalidFluid;
        }
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so fluids sharing an interval spread their work across
        // steps.
        fluid.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        fluids_.push_back(std::move(fluid));
        fluidIndexByHandle_.emplace(fluids_.back().handle, fluids_.size() - 1);
        return fluids_.back().handle;
    }

    void destroyFluidImpl(FluidHandle handle)
    {
        const auto iterator = fluidIndexByHandle_.find(handle);
        if (iterator == fluidIndexByHandle_.end())
        {
            return;
        }
        totalParticles_ -= fluids_[iterator->second].positions.size() / 3;
        const std::size_t index = iterator->second;
        fluidIndexByHandle_.erase(iterator);
        if (index + 1 != fluids_.size())
        {
            fluids_[index]                                   = fluids_.back();
            fluidIndexByHandle_[fluids_[index].handle]       = index;
        }
        fluids_.pop_back();
    }

    bool fluidInfoImpl(FluidHandle handle, FluidInfo *outInfo)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outInfo == nullptr)
        {
            return false;
        }
        outInfo->particleCount     = static_cast<std::uint32_t>(fluid->positions.size() / 3);
        outInfo->interactionRadius = fluid->description.interactionRadius;
        return true;
    }

    std::uint32_t addParticlesImpl(FluidHandle handle, const double *positions, const double *velocities,
        std::uint32_t count, FluidError *error)
    {
        setError(error, FluidError::None);
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr)
        {
            setError(error, FluidError::InvalidHandle);
            return 0u;
        }
        if (positions == nullptr && count > 0)
        {
            setError(error, FluidError::InvalidDescription);
            return 0u;
        }
        // Budget checks compute in 64-bit so a huge caller-supplied count
        // cannot wrap past them.
        const std::size_t current = fluid->positions.size() / 3;
        if (current + static_cast<std::size_t>(count) > settings_.maximumParticlesPerFluid
            || totalParticles_ + static_cast<std::size_t>(count) > settings_.maximumParticles)
        {
            setError(error, FluidError::CapacityExceeded);
            return 0u;
        }
        // The batch is atomic: every particle validates before anything
        // appends, so a rejected call can never leave the position, velocity,
        // and state arrays at inconsistent sizes (which the stepping passes
        // index into).
        for (std::uint32_t particle = 0; particle < count; ++particle)
        {
            const std::size_t source = static_cast<std::size_t>(particle) * 3;
            if (!std::isfinite(positions[source]) || !std::isfinite(positions[source + 1])
                || !std::isfinite(positions[source + 2])
                || (velocities != nullptr
                    && (!std::isfinite(velocities[source]) || !std::isfinite(velocities[source + 1])
                        || !std::isfinite(velocities[source + 2]))))
            {
                setError(error, FluidError::InvalidDescription);
                return 0u;
            }
        }
        for (std::uint32_t particle = 0; particle < count; ++particle)
        {
            const std::size_t source = static_cast<std::size_t>(particle) * 3;
            fluid->positions.push_back(positions[source]);
            fluid->positions.push_back(positions[source + 1]);
            fluid->positions.push_back(positions[source + 2]);
            if (velocities != nullptr)
            {
                fluid->velocities.push_back(velocities[source]);
                fluid->velocities.push_back(velocities[source + 1]);
                fluid->velocities.push_back(velocities[source + 2]);
            }
            else
            {
                fluid->velocities.push_back(0.0);
                fluid->velocities.push_back(0.0);
                fluid->velocities.push_back(0.0);
            }
        }
        const std::size_t particleCount = fluid->positions.size() / 3;
        // Only the appended entries initialize; existing particles keep their
        // last density, pressure, and acceleration state.
        fluid->densities.resize(particleCount, fluid->description.restDensity);
        fluid->pressures.resize(particleCount, 0.0);
        fluid->accelerations.resize(particleCount * 3, 0.0);
        totalParticles_ += count;
        return static_cast<std::uint32_t>(current);
    }

    bool copyParticlePositionsImpl(
        FluidHandle handle, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outPositions == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(fluid->positions.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(fluid->positions.begin(), fluid->positions.end(), outPositions);
        return true;
    }

    bool copyParticleVelocitiesImpl(
        FluidHandle handle, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outVelocities == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(fluid->velocities.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(fluid->velocities.begin(), fluid->velocities.end(), outVelocities);
        return true;
    }

    // Internal substep count for one fidelity band: lower bands reduce the
    // substep budget (Full = substepsPerStep, Simplified = half, Procedural
    // = quarter, Animation = one).
    std::uint32_t substepsForBand(engine::util::SimulationFidelity band) const
    {
        return engine::util::simulationLodSubstepsForBand(band, settings_.substepsPerStep);
    }

    bool stepImpl(FluidHandle handle, double fixedDeltaSeconds)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence. A due
        // step integrates the accumulated fixed deltas (fixedDelta x interval)
        // in one pass on the band's substep budget, so a reduced-cadence
        // fluid keeps advancing at wall time. Cached fluids freeze. Without
        // lod_enabled every fluid runs Full every step.
        const std::uint32_t interval = engine::util::simulationLodResolvePass(fluid->lodBand,
            fluid->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            fluid->importance,
            &fluid->lodSkippedSteps);
        if (interval == 0u)
        {
            return true;
        }
        return stepFluid(
            *fluid, settings_, rigid_, fixedDeltaSeconds * static_cast<double>(interval), substepsForBand(fluid->lodBand));
    }

    std::uint32_t simulateImpl(FluidHandle handle, double elapsedSeconds)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return fluid->accumulator.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, handle, fixedDelta]() { return stepImpl(handle, fixedDelta); });
    }

    bool setFluidImportanceImpl(FluidHandle handle, double importance)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        fluid->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool fluidStatisticsImpl(FluidHandle handle, FluidStatistics *outStatistics)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        outStatistics->stepsSimulated       = fluid->stepsSimulated;
        outStatistics->substepsExecuted     = fluid->substepsExecuted;
        outStatistics->lastContactCount     = fluid->lastContactCount;
        outStatistics->lastAverageDensity   = fluid->lastAverageDensity;
        outStatistics->lastStepMilliseconds = fluid->lastStepMilliseconds;
        outStatistics->lodSkippedSteps      = fluid->lodSkippedSteps;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsFluidCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_FLUID_BIND(field, method)                                                                       \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<FluidModule *>(self)->method(args...); }
        ENGINE_PHYSICS_FLUID_BIND(createFluid, createFluidImpl);
        ENGINE_PHYSICS_FLUID_BIND(destroyFluid, destroyFluidImpl);
        ENGINE_PHYSICS_FLUID_BIND(fluidInfo, fluidInfoImpl);
        ENGINE_PHYSICS_FLUID_BIND(addParticles, addParticlesImpl);
        ENGINE_PHYSICS_FLUID_BIND(copyParticlePositions, copyParticlePositionsImpl);
        ENGINE_PHYSICS_FLUID_BIND(copyParticleVelocities, copyParticleVelocitiesImpl);
        ENGINE_PHYSICS_FLUID_BIND(step, stepImpl);
        ENGINE_PHYSICS_FLUID_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_FLUID_BIND(fluidStatistics, fluidStatisticsImpl);
        ENGINE_PHYSICS_FLUID_BIND(setFluidImportance, setFluidImportanceImpl);
#undef ENGINE_PHYSICS_FLUID_BIND
    }

    ParticipantManifest manifest_;
    FluidCapability     capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    FluidSettings settings_;

    std::vector<FluidRecord>                     fluids_;
    std::unordered_map<FluidHandle, std::size_t> fluidIndexByHandle_;
    FluidHandle                                  nextFluidHandle_ = 1;
    std::size_t                                  totalParticles_  = 0;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;
};

} // namespace
} // namespace engine::physics_fluid

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_fluid capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_fluid"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_fluid",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_fluid::FluidModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
