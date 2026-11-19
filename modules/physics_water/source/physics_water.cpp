// physics_water module implementation.
//
// Gerstner ocean over a physics_rigid world: the surface is a steepness-
// weighted deep-water wave sum with true horizontal crest displacement
// (heights at fixed positions recovered by a fixed inverse iteration),
// superposed expanding ripple rings, automatic wake ripples for fast
// registered bodies, and a uniform current the buoyancy drag opposes;
// buoyancy applies per-body submersion-sampled forces, drag, and angular
// damping through the rigid capability's apply entry points inside the
// consumer's fixed update, so water physics composes deterministically with
// every other force source.

#include "records.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/physics_water/capabilities.hpp>
#include <engine/util/collider_metrics.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_water
{
namespace
{

namespace math = engine::physics_rigid::math;

class WaterModule final : public ModuleInterface
{
public:
    WaterModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_water", "maximum_oceans");
            value && *value > 0 && *value <= 4096)
            settings_.maximumOceans = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_water", "maximum_ripples_per_ocean");
            value && *value > 0 && *value <= 4096)
            settings_.maximumRipplesPerOcean = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_water", "maximum_registered_bodies");
            value && *value > 0 && *value <= 100'000)
            settings_.maximumRegisteredBodies = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_water", "water_density");
            value && *value > 0.0 && *value <= 20000.0 && std::isfinite(*value))
            settings_.waterDensity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_speed");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.rippleSpeed = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_decay");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.rippleDecay = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_attenuation");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.rippleAttenuation = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_wavelength");
            value && *value > 1.0e-4 && *value <= 1000.0 && std::isfinite(*value))
            settings_.rippleWavelength = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "buoyancy_scale");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.buoyancyScale = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "quadratic_drag_coefficient");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.quadraticDragCoefficient = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "angular_drag_coefficient");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.angularDragCoefficient = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "wake_speed_threshold");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.wakeSpeedThreshold = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "wake_spacing_meters");
            value && *value > 0.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.wakeSpacingMeters = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "wake_strength_scale");
            value && *value >= 0.0 && *value <= 10.0 && std::isfinite(*value))
            settings_.wakeStrengthScale = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "wake_submerged_fraction");
            value && *value > 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.wakeSubmergedFraction = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_retire_envelope");
            value && *value > 0.0 && *value < 1.0 && std::isfinite(*value))
            settings_.rippleRetireEnvelope = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "wave_gravity");
            value && *value > 0.1 && *value <= 100.0 && std::isfinite(*value))
            settings_.waveGravity = *value;
        if (const auto value = context.configurationIntegerInSection("physics_water", "gerstner_iterations");
            value && *value >= 1 && *value <= 32)
            settings_.gerstnerIterations = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_water", "normal_sample_epsilon");
            value && *value > 1.0e-6 && *value <= 1.0 && std::isfinite(*value))
            settings_.normalSampleEpsilon = *value;

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
        oceans_.clear();
        oceanIndexByHandle_.clear();
        nextOceanHandle_ = 1;
        statistics_      = WaterStatistics{};
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == WaterCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_water",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_water"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_water");
    }

    static void setError(WaterError *output, WaterError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    // Refreshes the module-wide registration count: the statistic reports
    // every ocean's registrations together, not whichever ocean was touched
    // last.
    void refreshRegisteredBodyStatistics()
    {
        std::size_t total = 0;
        for (const OceanRecord &ocean : oceans_)
        {
            total += ocean.registeredBodies.size();
        }
        statistics_.registeredBodies = static_cast<std::uint32_t>(total);
    }

    // Refreshes the module-wide active-ripple count the same way.
    void refreshActiveRippleStatistics()
    {
        std::size_t active = 0;
        for (const OceanRecord &ocean : oceans_)
        {
            for (const Ripple &ripple : ocean.ripples)
            {
                active += ripple.active ? 1u : 0u;
            }
        }
        statistics_.activeRipples = static_cast<std::uint32_t>(active);
    }

    bool setOceanImportanceImpl(OceanHandle handle, double importance)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        ocean->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    // Uniform horizontal current the buoyancy drag opposes: the y component
    // is forced to zero (the description stays validated as finite first).
    static math::Vec3 effectiveCurrentOf(const OceanDescription &description)
    {
        return math::Vec3{description.current[0], 0.0, description.current[2]};
    }

    OceanRecord *findOcean(OceanHandle handle)
    {
        const auto iterator = oceanIndexByHandle_.find(handle);
        return iterator != oceanIndexByHandle_.end() ? &oceans_[iterator->second] : nullptr;
    }

    bool validateDescription(const OceanDescription &description) const
    {
        if (description.waveCount > maximumWavesPerOcean || !std::isfinite(description.waterLevel))
        {
            return false;
        }
        for (std::uint32_t wave = 0; wave < description.waveCount; ++wave)
        {
            const WaveDescription &candidate = description.waves[wave];
            if (!std::isfinite(candidate.directionX) || !std::isfinite(candidate.directionZ)
                || !std::isfinite(candidate.wavelength) || candidate.wavelength <= 0.0
                || !std::isfinite(candidate.amplitude) || candidate.amplitude < 0.0
                || !std::isfinite(candidate.steepness))
            {
                return false;
            }
        }
        return std::isfinite(description.current[0]) && std::isfinite(description.current[1])
               && std::isfinite(description.current[2]);
    }

    void prepareWaves(OceanRecord &ocean)
    {
        ocean.waves.clear();
        ocean.waves.reserve(ocean.description.waveCount);
        // GPU Gems 1 chapter 1 steepness factor: Q_i = steepness / (k_i *
        // amplitude_i * waveCount), so Q_i * k_i * amplitude_i = steepness /
        // waveCount per wave and the sum over the set stays <= 1 (no looping
        // crests). The per-wave horizontal displacement amplitude is
        // Q_i * amplitude_i = steepness / (k_i * waveCount); zero-amplitude
        // components contribute no displacement at all.
        const double waveCount = static_cast<double>(ocean.description.waveCount);
        for (std::uint32_t wave = 0; wave < ocean.description.waveCount; ++wave)
        {
            const WaveDescription &candidate = ocean.description.waves[wave];
            PreparedWave prepared;
            const double length = std::sqrt(candidate.directionX * candidate.directionX
                                            + candidate.directionZ * candidate.directionZ);
            if (length < 1.0e-9)
            {
                prepared.direction = math::Vec3{1.0, 0.0, 0.0};
            }
            else
            {
                prepared.direction = math::Vec3{candidate.directionX / length, 0.0, candidate.directionZ / length};
            }
            prepared.waveNumber = 2.0 * math::kPi / candidate.wavelength;
            prepared.amplitude  = candidate.amplitude;
            const double steepness = std::clamp(candidate.steepness, 0.0, 1.0);
            prepared.horizontalAmplitude = steepness > 0.0 && candidate.amplitude > 0.0
                                               ? steepness / (prepared.waveNumber * waveCount)
                                               : 0.0;
            prepared.omega = std::sqrt(settings_.waveGravity * prepared.waveNumber);
            ocean.waves.push_back(prepared);
        }
    }

    // Displaced Gerstner surface position for a horizontal seed (seedX,
    // seedZ): the seed sum of horizontal crest displacements plus the wave
    // height. Pure function of (seed, time); with every steepness at 0 the
    // horizontal terms vanish and this reduces to the plain sinusoid sum.
    math::Vec3 displacedPosition(const OceanRecord &ocean, double seedX, double seedZ, double timeSeconds) const
    {
        double displacedX = seedX;
        double displacedZ = seedZ;
        double height     = ocean.description.waterLevel;
        for (const PreparedWave &wave : ocean.waves)
        {
            const double phase = wave.waveNumber * (wave.direction.x * seedX + wave.direction.z * seedZ)
                                 - wave.omega * timeSeconds;
            displacedX += wave.horizontalAmplitude * wave.direction.x * std::cos(phase);
            displacedZ += wave.horizontalAmplitude * wave.direction.z * std::cos(phase);
            height     += wave.amplitude * std::sin(phase);
        }
        return math::Vec3{displacedX, height, displacedZ};
    }

    // Surface point whose horizontal image lands on (x, z): the Gerstner
    // displacement moves sample points horizontally, so the height at a
    // fixed horizontal position is recovered by inverse iteration. The seed
    // starts at the query point and is corrected by seed += target - P.xz a
    // fixed number of times (each pass contracts the seed error by at most
    // the total steepness factor sum(Q_i * k_i * amplitude_i) <= 1), then
    // the final seed is evaluated once and returned.
    math::Vec3 displacedSurfacePoint(const OceanRecord &ocean, double x, double z, double timeSeconds) const
    {
        double seedX = x;
        double seedZ = z;
        for (std::uint32_t iteration = 0; iteration < settings_.gerstnerIterations; ++iteration)
        {
            const math::Vec3 displaced = displacedPosition(ocean, seedX, seedZ, timeSeconds);
            seedX += x - displaced.x;
            seedZ += z - displaced.z;
        }
        return displacedPosition(ocean, seedX, seedZ, timeSeconds);
    }

    // Full surface height (waves through the Gerstner inverse mapping plus
    // every active ripple, sampled at the query point exactly as before) at
    // a world (x, z).
    double surfaceHeight(const OceanRecord &ocean, double x, double z, double timeSeconds) const
    {
        return displacedSurfacePoint(ocean, x, z, timeSeconds).y + rippleHeight(ocean, x, z);
    }

    // Ripple height contribution at a world (x, z): one expanding, decaying
    // ring per active ripple source.
    double rippleHeight(const OceanRecord &ocean, double x, double z) const
    {
        double height = 0.0;
        for (const Ripple &ripple : ocean.ripples)
        {
            if (!ripple.active)
            {
                continue;
            }
            const double dx        = x - ripple.centerX;
            const double dz        = z - ripple.centerZ;
            const double radius    = std::sqrt(dx * dx + dz * dz);
            const double ringFront = ripple.age * settings_.rippleSpeed;
            const double distanceToRing = std::abs(radius - ringFront);
            if (distanceToRing > settings_.rippleWavelength)
            {
                continue;
            }
            const double ringPhase = 2.0 * math::kPi * (radius - ringFront) / settings_.rippleWavelength;
            const double envelope  = std::exp(-ripple.age * settings_.rippleDecay - radius * settings_.rippleAttenuation)
                                    * (1.0 - distanceToRing / settings_.rippleWavelength);
            height += ripple.strength * envelope * std::cos(ringPhase);
        }
        return height;
    }

    // Builds the shape-dependent sample set from an already fetched body
    // state and description (the buoyancy pass fetches them batched; the
    // single-body queries fetch directly). The sets come from the shared
    // engine::util::collider_metrics vocabulary, which covers every solid
    // shape including cylinders.
    void collectSamplePoints(const engine::physics_rigid::BodyState &state,
        const engine::physics_rigid::BodyDescription &description, std::vector<math::Vec3> &outPoints) const
    {
        outPoints.clear();
        engine::util::appendColliderSamplePoints(description.shape, state.transform, outPoints);
    }

    double bodyMassOf(const engine::physics_rigid::BodyDescription &description) const
    {
        return description.density > 0.0 ? description.density * engine::util::colliderVolume(description.shape)
                                         : description.mass;
    }

    // ---- capability implementations ---------------------------------------

    OceanHandle createOceanImpl(
        engine::physics_rigid::WorldHandle world, const OceanDescription *description, WaterError *error)
    {
        setError(error, WaterError::None);
        if (rigid_ == nullptr || description == nullptr)
        {
            setError(error, WaterError::InvalidDescription);
            return invalidOcean;
        }
        if (oceans_.size() >= settings_.maximumOceans)
        {
            setError(error, WaterError::CapacityExceeded);
            return invalidOcean;
        }
        if (!validateDescription(*description))
        {
            setError(error, WaterError::InvalidDescription);
            return invalidOcean;
        }
        if (nextOceanHandle_ == invalidOcean)
        {
            setError(error, WaterError::CapacityExceeded);
            return invalidOcean;
        }
        OceanRecord ocean;
        ocean.handle = nextOceanHandle_;
        nextOceanHandle_
            = nextOceanHandle_ == std::numeric_limits<OceanHandle>::max() ? invalidOcean : nextOceanHandle_ + 1;
        ocean.world       = world;
        ocean.description = *description;
        ocean.currentVelocity = effectiveCurrentOf(ocean.description);
        ocean.ripples.assign(settings_.maximumRipplesPerOcean, Ripple{});
        prepareWaves(ocean);
        // The buoyancy pass pushes opposite to this ocean's world gravity;
        // capturing the magnitude doubles as the world-handle validation: a
        // world the rigid module does not know is rejected instead of
        // silently falling back to the default gravity.
        double gravity[3] = {0.0, -9.81, 0.0};
        if (!rigid_->copyWorldGravity(rigid_->self, world, gravity))
        {
            setError(error, WaterError::InvalidHandle);
            return invalidOcean;
        }
        ocean.gravityMagnitude = std::abs(gravity[1]) > 1.0e-6 ? std::abs(gravity[1])
                                                               : std::sqrt(gravity[0] * gravity[0]
                                                                           + gravity[1] * gravity[1]
                                                                           + gravity[2] * gravity[2]);
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so oceans sharing an interval spread their passes across
        // steps.
        const auto phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        ocean.rippleCadence.phase   = phase;
        ocean.buoyancyCadence.phase = phase;
        oceans_.push_back(std::move(ocean));
        oceanIndexByHandle_.emplace(oceans_.back().handle, oceans_.size() - 1);
        statistics_.oceanCount = static_cast<std::uint32_t>(oceans_.size());
        return oceans_.back().handle;
    }

    void destroyOceanImpl(OceanHandle handle)
    {
        const auto iterator = oceanIndexByHandle_.find(handle);
        if (iterator == oceanIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        oceanIndexByHandle_.erase(iterator);
        if (index + 1 != oceans_.size())
        {
            oceans_[index]                                  = oceans_.back();
            oceanIndexByHandle_[oceans_[index].handle]      = index;
        }
        oceans_.pop_back();
        statistics_.oceanCount = static_cast<std::uint32_t>(oceans_.size());
        refreshRegisteredBodyStatistics();
        refreshActiveRippleStatistics();
    }

    bool setOceanDescriptionImpl(OceanHandle handle, const OceanDescription *description)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || description == nullptr || !validateDescription(*description))
        {
            return false;
        }
        ocean->description = *description;
        // An explicit runtime current survives a description update; without
        // one the current re-derives from the description.
        if (!ocean->currentExplicit)
        {
            ocean->currentVelocity = effectiveCurrentOf(ocean->description);
        }
        prepareWaves(*ocean);
        return true;
    }

    bool sampleSurfaceImpl(
        OceanHandle handle, double x, double z, double timeSeconds, double *outHeight, double outNormal[3])
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || outHeight == nullptr || outNormal == nullptr || !std::isfinite(timeSeconds))
        {
            return false;
        }
        const double epsilon  = settings_.normalSampleEpsilon;
        const double height   = surfaceHeight(*ocean, x, z, timeSeconds);
        const double heightX0 = surfaceHeight(*ocean, x - epsilon, z, timeSeconds);
        const double heightX1 = surfaceHeight(*ocean, x + epsilon, z, timeSeconds);
        const double heightZ0 = surfaceHeight(*ocean, x, z - epsilon, timeSeconds);
        const double heightZ1 = surfaceHeight(*ocean, x, z + epsilon, timeSeconds);
        *outHeight = height;
        const math::Vec3 normal = math::normalized(math::Vec3{-(heightX1 - heightX0) / (2.0 * epsilon), 1.0,
            -(heightZ1 - heightZ0) / (2.0 * epsilon)});
        outNormal[0] = normal.x;
        outNormal[1] = normal.y;
        outNormal[2] = normal.z;
        return true;
    }

    // Plants a validated ripple into the ocean's fixed slot array: the first
    // inactive slot or, when every slot is active, the oldest one. Shared by
    // addRipple and the automatic wake emission; allocation-free.
    void plantRipple(OceanRecord &ocean, double centerX, double centerZ, double strength)
    {
        std::size_t slot = ocean.ripples.size();
        for (std::size_t index = 0; index < ocean.ripples.size(); ++index)
        {
            if (!ocean.ripples[index].active)
            {
                slot = index;
                break;
            }
            if (slot == ocean.ripples.size() || ocean.ripples[index].age > ocean.ripples[slot].age)
            {
                slot = index;
            }
        }
        if (slot == ocean.ripples.size())
        {
            return; // unreachable while the ocean owns at least one slot
        }
        Ripple &ripple  = ocean.ripples[slot];
        ripple.active   = true;
        ripple.centerX  = centerX;
        ripple.centerZ  = centerZ;
        ripple.age      = 0.0;
        ripple.strength = strength;
    }

    bool addRippleImpl(OceanHandle handle, const double center[3], double strength)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || center == nullptr || !std::isfinite(strength) || strength == 0.0)
        {
            return false;
        }
        if (!std::isfinite(center[0]) || !std::isfinite(center[1]) || !std::isfinite(center[2]))
        {
            return false;
        }
        plantRipple(*ocean, center[0], center[2], strength);
        return true;
    }

    // Submersion fraction with the querySubmersion sample scheme, evaluated
    // against the surface at the ocean's simulated time; used by the wake
    // gate.
    double bodySubmersionFraction(OceanRecord &ocean, engine::physics_rigid::BodyHandle body)
    {
        engine::physics_rigid::BodyState state;
        engine::physics_rigid::BodyDescription description;
        if (!rigid_->copyBodyState(rigid_->self, ocean.world, body, &state)
            || !rigid_->copyBodyDescription(rigid_->self, ocean.world, body, &description))
        {
            return 0.0;
        }
        collectSamplePoints(state, description, samplePoints_);
        if (samplePoints_.empty())
        {
            return 0.0;
        }
        std::uint32_t below = 0;
        for (const math::Vec3 &point : samplePoints_)
        {
            if (surfaceHeight(ocean, point.x, point.z, ocean.simulatedTime) > point.y)
            {
                ++below;
            }
        }
        return static_cast<double>(below) / static_cast<double>(samplePoints_.size());
    }

    // Automatic wakes for one fixed step: registered bodies walked in
    // ascending body-handle order accumulate travelled horizontal distance;
    // past the configured spacing at a speed above the threshold, a ripple
    // is planted at the body's position with amplitude proportional to
    // speed. Fully submerged bodies stay silent, the accumulator resets on
    // each spawn, and no allocations happen anywhere in the walk.
    void emitWakes(OceanRecord &ocean)
    {
        if (rigid_ == nullptr || settings_.wakeStrengthScale <= 0.0)
        {
            return;
        }
        // One batched state read per pass instead of one call per body; the
        // resolved count prunes registrations whose body died in the rigid
        // world (their stale velocity would otherwise plant a phantom wake
        // at the last seen position).
        wakeStatesScratch_.resize(ocean.registeredBodies.size());
        const std::uint32_t resolved = rigid_->copyBodyStates(rigid_->self, ocean.world,
            ocean.registeredBodies.data(), static_cast<std::uint32_t>(ocean.registeredBodies.size()),
            wakeStatesScratch_.data());
        for (std::size_t index = ocean.registeredBodies.size(); index > resolved; --index)
        {
            const std::size_t position = index - 1;
            ocean.registeredBodies.erase(ocean.registeredBodies.begin() + static_cast<std::ptrdiff_t>(position));
            ocean.wakeStates.erase(ocean.wakeStates.begin() + static_cast<std::ptrdiff_t>(position));
            if (position < ocean.bodyVolumes.size())
            {
                ocean.bodyVolumes.erase(ocean.bodyVolumes.begin() + static_cast<std::ptrdiff_t>(position));
            }
            refreshRegisteredBodyStatistics();
        }
        for (std::size_t index = 0; index < ocean.registeredBodies.size(); ++index)
        {
            WakeState &wake = ocean.wakeStates[index];
            const engine::physics_rigid::BodyState &state = wakeStatesScratch_[index];
            const double deltaX     = state.transform.position.x - wake.lastPositionX;
            const double deltaZ     = state.transform.position.z - wake.lastPositionZ;
            wake.lastPositionX      = state.transform.position.x;
            wake.lastPositionZ      = state.transform.position.z;
            wake.travelledDistance += std::sqrt(deltaX * deltaX + deltaZ * deltaZ);
            const double speed      = std::sqrt(state.linearVelocity.x * state.linearVelocity.x
                                                + state.linearVelocity.z * state.linearVelocity.z);
            if (speed < settings_.wakeSpeedThreshold || wake.travelledDistance < settings_.wakeSpacingMeters)
            {
                continue;
            }
            if (bodySubmersionFraction(ocean, ocean.registeredBodies[index]) >= settings_.wakeSubmergedFraction)
            {
                continue;
            }
            plantRipple(ocean, state.transform.position.x, state.transform.position.z,
                speed * settings_.wakeStrengthScale);
            wake.travelledDistance = 0.0;
        }
    }

    bool stepRipplesImpl(OceanHandle handle, double fixedDeltaSeconds)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0
            || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the ripple pass's own
        // staggered cadence; a due step covers interval x the caller's
        // delta (ripple aging is wave math over wall time, so the enlarged
        // delta keeps the pacing exact), and Cached oceans freeze. Without
        // lod_enabled every ocean runs Full every step. The buoyancy pass
        // keeps its own cadence, so a consumer driving both passes per
        // fixed update staggers each one exactly once per step.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(ocean->lodBand,
            ocean->rippleCadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            ocean->importance,
            &ocean->lodSkippedSteps);
        if (lodInterval == 0u)
        {
            return true;
        }
        const double stepDelta = fixedDeltaSeconds * static_cast<double>(lodInterval);
        ocean->simulatedTime += stepDelta;
        emitWakes(*ocean);
        std::uint32_t active = 0;
        for (Ripple &ripple : ocean->ripples)
        {
            if (!ripple.active)
            {
                continue;
            }
            ripple.age += stepDelta;
            // A ripple retires when its envelope has decayed below the
            // configured amplitude fraction of its initial strength.
            if (std::exp(-ripple.age * settings_.rippleDecay) < settings_.rippleRetireEnvelope)
            {
                ripple.active = false;
            }
            else
            {
                ++active;
            }
        }
        (void)active; // per-ocean count folded into the aggregate below
        refreshActiveRippleStatistics();
        return true;
    }

    bool registerBodyImpl(OceanHandle handle, engine::physics_rigid::BodyHandle body, WaterError *error)
    {
        setError(error, WaterError::None);
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || rigid_ == nullptr || body == engine::physics_rigid::invalidBody)
        {
            setError(error, WaterError::InvalidHandle);
            return false;
        }
        engine::physics_rigid::BodyState state;
        if (!rigid_->copyBodyState(rigid_->self, ocean->world, body, &state))
        {
            setError(error, WaterError::InvalidHandle);
            return false;
        }
        if (ocean->registeredBodies.size() >= settings_.maximumRegisteredBodies)
        {
            setError(error, WaterError::CapacityExceeded);
            return false;
        }
        // Registrations stay sorted by body handle so every pass (wake
        // emission, buoyancy) walks them in one fixed order; the wake
        // bookkeeping vector is kept index-parallel to this list.
        const auto entry = std::lower_bound(ocean->registeredBodies.begin(), ocean->registeredBodies.end(), body);
        if (entry != ocean->registeredBodies.end() && *entry == body)
        {
            return true; // already registered
        }
        const auto position = static_cast<std::ptrdiff_t>(entry - ocean->registeredBodies.begin());
        ocean->registeredBodies.insert(ocean->registeredBodies.begin() + position, body);
        WakeState wake;
        wake.lastPositionX = state.transform.position.x;
        wake.lastPositionZ = state.transform.position.z;
        ocean->wakeStates.insert(ocean->wakeStates.begin() + position, wake);
        // The volume cache stays index-parallel: a fresh slot starts without
        // a cached volume (-1 recomputes it on the next buoyancy pass).
        ocean->bodyVolumes.insert(ocean->bodyVolumes.begin() + position, -1.0);
        refreshRegisteredBodyStatistics();
        return true;
    }

    bool unregisterBodyImpl(OceanHandle handle, engine::physics_rigid::BodyHandle body)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr)
        {
            return false;
        }
        const auto entry = std::lower_bound(ocean->registeredBodies.begin(), ocean->registeredBodies.end(), body);
        if (entry == ocean->registeredBodies.end() || *entry != body)
        {
            return false;
        }
        const auto position = static_cast<std::ptrdiff_t>(entry - ocean->registeredBodies.begin());
        ocean->registeredBodies.erase(entry);
        ocean->wakeStates.erase(ocean->wakeStates.begin() + position);
        if (position < static_cast<std::ptrdiff_t>(ocean->bodyVolumes.size()))
        {
            ocean->bodyVolumes.erase(ocean->bodyVolumes.begin() + position);
        }
        refreshRegisteredBodyStatistics();
        return true;
    }

    bool querySubmersionImpl(OceanHandle handle, engine::physics_rigid::BodyHandle body, double timeSeconds,
        double *outFraction, double *outDepth)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || rigid_ == nullptr || outFraction == nullptr || outDepth == nullptr
            || !std::isfinite(timeSeconds))
        {
            return false;
        }
        engine::physics_rigid::BodyState state;
        engine::physics_rigid::BodyDescription description;
        if (!rigid_->copyBodyState(rigid_->self, ocean->world, body, &state)
            || !rigid_->copyBodyDescription(rigid_->self, ocean->world, body, &description))
        {
            // An unknown body sits fully above the surface: zero fraction,
            // zero depth.
            *outFraction = 0.0;
            *outDepth    = 0.0;
            return true;
        }
        collectSamplePoints(state, description, samplePoints_);
        if (samplePoints_.empty())
        {
            // A shape without sample points reports nothing submerged: the
            // outputs stay defined (zeros) so a failed query never leaves
            // uninitialized values behind.
            *outFraction = 0.0;
            *outDepth    = 0.0;
            return false;
        }
        std::uint32_t below = 0;
        double deepest = -std::numeric_limits<double>::max();
        for (const math::Vec3 &point : samplePoints_)
        {
            const double surface = surfaceHeight(*ocean, point.x, point.z, timeSeconds);
            const double depth   = surface - point.y;
            if (depth > 0.0)
            {
                ++below;
            }
            deepest = std::max(deepest, depth);
        }
        *outFraction = static_cast<double>(below) / static_cast<double>(samplePoints_.size());
        *outDepth    = deepest;
        return true;
    }

    bool applyBuoyancyImpl(OceanHandle handle, double timeSeconds, double fixedDeltaSeconds)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || rigid_ == nullptr || !std::isfinite(timeSeconds) || !std::isfinite(fixedDeltaSeconds)
            || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the buoyancy pass's own
        // staggered cadence; a due pass scales the applied forces by the
        // interval so the impulse per wall time matches the full-rate
        // schedule, and Cached oceans apply nothing. Without lod_enabled
        // every ocean runs Full every pass.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(ocean->lodBand,
            ocean->buoyancyCadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            ocean->importance,
            &ocean->lodSkippedSteps);
        if (lodInterval == 0u)
        {
            return true;
        }
        const double forceScale = static_cast<double>(lodInterval);
        std::uint64_t applied = 0;
        // Batched reads: two calls per pass instead of two per body.
        buoyancyStates_.resize(ocean->registeredBodies.size());
        buoyancyDescriptions_.resize(ocean->registeredBodies.size());
        const std::uint32_t resolvedStates = rigid_->copyBodyStates(rigid_->self, ocean->world,
            ocean->registeredBodies.data(), static_cast<std::uint32_t>(ocean->registeredBodies.size()),
            buoyancyStates_.data());
        const std::uint32_t resolvedDescriptions = rigid_->copyBodyDescriptions(rigid_->self, ocean->world,
            ocean->registeredBodies.data(), static_cast<std::uint32_t>(ocean->registeredBodies.size()),
            buoyancyDescriptions_.data());
        ocean->bodyVolumes.resize(ocean->registeredBodies.size(), -1.0);
        for (std::size_t index = 0; index < ocean->registeredBodies.size();)
        {
            const engine::physics_rigid::BodyHandle body = ocean->registeredBodies[index];
            const bool resolved = index < resolvedStates && index < resolvedDescriptions;
            if (!resolved)
            {
                // The body died in the rigid world: free the registration.
                ocean->registeredBodies.erase(ocean->registeredBodies.begin()
                                              + static_cast<std::ptrdiff_t>(index));
                ocean->wakeStates.erase(ocean->wakeStates.begin() + static_cast<std::ptrdiff_t>(index));
                ocean->bodyVolumes.erase(ocean->bodyVolumes.begin() + static_cast<std::ptrdiff_t>(index));
                refreshRegisteredBodyStatistics();
                continue;
            }
            const engine::physics_rigid::BodyState &state = buoyancyStates_[index];
            const engine::physics_rigid::BodyDescription &description = buoyancyDescriptions_[index];
            const std::size_t registrationIndex = index;
            ++index;
            if (!state.awake)
            {
                continue;
            }
            collectSamplePoints(state, description, samplePoints_);
            if (samplePoints_.empty())
            {
                continue;
            }
            std::uint32_t below = 0;
            math::Vec3 submergedCentroid{};
            double deepest = -std::numeric_limits<double>::max();
            for (const math::Vec3 &point : samplePoints_)
            {
                const double surface = surfaceHeight(*ocean, point.x, point.z, timeSeconds);
                const double depth   = surface - point.y;
                if (depth > 0.0)
                {
                    ++below;
                    submergedCentroid = submergedCentroid + point;
                }
                deepest = std::max(deepest, depth);
            }
            if (below == 0)
            {
                continue;
            }
            submergedCentroid = submergedCentroid * (1.0 / static_cast<double>(below));
            const double fraction = static_cast<double>(below) / static_cast<double>(samplePoints_.size());
            // The shape volume caches per registered body (shapes are
            // immutable per handle); the buoyancy factor scales with the
            // pass interval so the impulse per wall time stays constant.
            // The volume vector stays index-parallel to the registrations,
            // which the dead-body pruning above already compacted.
            double shapeVolumeMeters = ocean->bodyVolumes[registrationIndex];
            if (!(shapeVolumeMeters >= 0.0))
            {
                shapeVolumeMeters = engine::util::colliderVolume(description.shape);
                ocean->bodyVolumes[registrationIndex] = shapeVolumeMeters;
            }
            const double volume   = shapeVolumeMeters * fraction * settings_.buoyancyScale;
            const double mass     = bodyMassOf(description);
            if (mass <= 0.0 || volume <= 0.0)
            {
                continue;
            }
            // Gravity magnitude from the ocean's own world; buoyancy pushes
            // opposite to it.
            const double buoyancyForce[3]
                = {0.0, settings_.waterDensity * volume * ocean->gravityMagnitude * forceScale, 0.0};
            const double centroidPoint[3] = {submergedCentroid.x, submergedCentroid.y, submergedCentroid.z};
            rigid_->applyForceAtPoint(rigid_->self, ocean->world, body, buoyancyForce, centroidPoint);

            // Quadratic water drag on the submerged part: opposite the body
            // velocity relative to the ocean's uniform current, scaled by the
            // submerged fraction and shape size.
            const math::Vec3 relativeVelocity = state.linearVelocity - ocean->currentVelocity;
            const double     speed            = math::length(relativeVelocity);
            if (speed > 1.0e-6)
            {
                const double dragArea  = std::cbrt(shapeVolumeMeters * shapeVolumeMeters);
                const double magnitude = 0.5 * settings_.waterDensity * settings_.quadraticDragCoefficient * dragArea
                                         * fraction * speed * speed * forceScale;
                const math::Vec3 direction = relativeVelocity * (-1.0 / speed);
                const double dragForce[3]  = {direction.x * magnitude, direction.y * magnitude, direction.z * magnitude};
                rigid_->applyForceAtPoint(rigid_->self, ocean->world, body, dragForce, centroidPoint);
            }

            // Angular damping torque proportional to the submerged fraction.
            const double angularSpeed = math::length(state.angularVelocity);
            if (angularSpeed > 1.0e-6 && settings_.angularDragCoefficient > 0.0)
            {
                const double torqueMagnitude = settings_.angularDragCoefficient * fraction * mass * angularSpeed * 0.01;
                const math::Vec3 torque      = state.angularVelocity * (-torqueMagnitude / angularSpeed);
                const double torqueVector[3] = {torque.x, torque.y, torque.z};
                rigid_->applyTorque(rigid_->self, ocean->world, body, torqueVector);
            }
            ++applied;
        }
        statistics_.buoyancyApplications = applied;
        return true;
    }

    bool waterStatisticsImpl(WaterStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        statistics_.lodSkippedSteps = 0;
        for (const OceanRecord &ocean : oceans_)
        {
            statistics_.lodSkippedSteps += ocean.lodSkippedSteps;
        }
        *outStatistics = statistics_;
        return true;
    }

    bool sampleSurfaceDisplacedImpl(
        OceanHandle handle, const double position[3], double timeSeconds, double outDisplacement[3])
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || position == nullptr || outDisplacement == nullptr || !std::isfinite(timeSeconds))
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        const math::Vec3 point = displacedSurfacePoint(*ocean, position[0], position[2], timeSeconds);
        outDisplacement[0]     = point.x - position[0];
        outDisplacement[1]     = point.y + rippleHeight(*ocean, position[0], position[2]);
        outDisplacement[2]     = point.z - position[2];
        return true;
    }

    bool setCurrentVelocityImpl(OceanHandle handle, const double current[3])
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || current == nullptr || !std::isfinite(current[0]) || !std::isfinite(current[1])
            || !std::isfinite(current[2]))
        {
            return false;
        }
        ocean->currentVelocity = math::Vec3{current[0], 0.0, current[2]};
        ocean->currentExplicit = true;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsWaterCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_WATER_BIND(field, method)                                                                       \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<WaterModule *>(self)->method(args...); }
        ENGINE_PHYSICS_WATER_BIND(createOcean, createOceanImpl);
        ENGINE_PHYSICS_WATER_BIND(destroyOcean, destroyOceanImpl);
        ENGINE_PHYSICS_WATER_BIND(setOceanDescription, setOceanDescriptionImpl);
        ENGINE_PHYSICS_WATER_BIND(sampleSurface, sampleSurfaceImpl);
        ENGINE_PHYSICS_WATER_BIND(addRipple, addRippleImpl);
        ENGINE_PHYSICS_WATER_BIND(stepRipples, stepRipplesImpl);
        ENGINE_PHYSICS_WATER_BIND(registerBody, registerBodyImpl);
        ENGINE_PHYSICS_WATER_BIND(unregisterBody, unregisterBodyImpl);
        ENGINE_PHYSICS_WATER_BIND(querySubmersion, querySubmersionImpl);
        ENGINE_PHYSICS_WATER_BIND(applyBuoyancy, applyBuoyancyImpl);
        ENGINE_PHYSICS_WATER_BIND(waterStatistics, waterStatisticsImpl);
        ENGINE_PHYSICS_WATER_BIND(setOceanImportance, setOceanImportanceImpl);
        ENGINE_PHYSICS_WATER_BIND(sampleSurfaceDisplaced, sampleSurfaceDisplacedImpl);
        ENGINE_PHYSICS_WATER_BIND(setCurrentVelocity, setCurrentVelocityImpl);
#undef ENGINE_PHYSICS_WATER_BIND
    }

    ParticipantManifest manifest_;
    WaterCapability     capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    WaterSettings settings_;

    WaterStatistics statistics_{};


    std::vector<OceanRecord>                     oceans_;

    // Reused scratch: wake-state and buoyancy batch reads, plus the
    // registration ordinal for LOD stagger phases.
    std::vector<engine::physics_rigid::BodyState>        wakeStatesScratch_;
    std::vector<engine::physics_rigid::BodyState>        buoyancyStates_;
    std::vector<engine::physics_rigid::BodyDescription>  buoyancyDescriptions_;
    std::uint64_t                                        registrationOrdinal_ = 0;
    std::unordered_map<OceanHandle, std::size_t> oceanIndexByHandle_;
    OceanHandle                                  nextOceanHandle_ = 1;

    // Per-pass scratch.
    std::vector<math::Vec3> samplePoints_;
};

} // namespace
} // namespace engine::physics_water

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_water capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_water"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_water",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_water::WaterModule(); },
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
