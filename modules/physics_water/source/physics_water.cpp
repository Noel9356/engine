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

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/physics_water/capabilities.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace engine::physics_water
{
namespace
{

namespace math = engine::physics_rigid::math;

constexpr double kPi            = 3.14159265358979323846;
constexpr double kGravityForWaves = 9.81; // deep-water dispersion reference gravity

constexpr int    kGerstnerIterations    = 4;    // fixed inverse-iteration count for the displaced height

double shapeVolumeOf(const engine::physics_rigid::ShapeDescription &shape)
{
    using engine::physics_rigid::ShapeKind;
    switch (shape.kind)
    {
    case ShapeKind::Sphere:
        return (4.0 / 3.0) * kPi * shape.sphereRadius * shape.sphereRadius * shape.sphereRadius;
    case ShapeKind::Box:
        return 8.0 * shape.boxHalfExtents[0] * shape.boxHalfExtents[1] * shape.boxHalfExtents[2];
    case ShapeKind::Capsule: {
        const double r = shape.capsuleRadius;
        return kPi * r * r * (2.0 * shape.capsuleHalfLength) + (4.0 / 3.0) * kPi * r * r * r;
    }
    case ShapeKind::Plane:
    default:
        return 0.0;
    }
}

struct PreparedWave
{
    math::Vec3 direction;           // unit, XZ plane
    double     waveNumber           = 0.0;  // k = 2*pi / wavelength
    double     amplitude            = 0.0;
    double     horizontalAmplitude  = 0.0;  // Q * amplitude: Gerstner horizontal crest displacement (meters)
    double     omega                = 0.0;  // deep-water dispersion: omega = sqrt(g * k)
};

struct Ripple
{
    bool   active    = false;
    double centerX   = 0.0;
    double centerZ   = 0.0;
    double age       = 0.0;
    double strength  = 0.0; // initial ring amplitude (meters)
};

// Per-registration wake bookkeeping, kept parallel to the ocean's
// registeredBodies vector: the body's last seen position and the horizontal
// distance travelled since its last emitted wake ripple.
struct WakeState
{
    double lastPositionX     = 0.0;
    double lastPositionZ     = 0.0;
    double travelledDistance = 0.0; // meters since the last wake ripple
};

struct OceanRecord
{
    OceanHandle handle = invalidOcean;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    OceanDescription description{};

    // Effective uniform current (meters per second); the y component is
    // always 0. Re-derived from the description on create and
    // setOceanDescription, updatable at runtime through setCurrentVelocity.
    math::Vec3 currentVelocity{};

    // timeSeconds of the most recent applyBuoyancy pass on this ocean; the
    // wake submersion check samples the wave surface at this clock reading
    // (0 before the first pass).
    double lastBuoyancyTime = 0.0;

    std::vector<PreparedWave> waves;

    std::vector<Ripple> ripples;

    // Ascending body-handle order; wakeStates and bodyVolumes stay
    // index-parallel to it. bodyVolumes caches each registered body's shape
    // volume (shapes are immutable per body handle; a negative value marks
    // "not yet computed"), so the buoyancy pass does not re-derive it from
    // the description every step.
    std::vector<engine::physics_rigid::BodyHandle> registeredBodies;

    std::vector<WakeState> wakeStates;
    std::vector<double>    bodyVolumes;

    // Gravity magnitude of this ocean's rigid world, captured at creation
    // (the buoyancy pass pushes opposite to it).
    double gravityMagnitude = 9.81;

    // Simulation LOD: importance-driven fidelity band and staggered cadence
    // (the phase derives from the registration ordinal). Due passes age the
    // ripples by the interval delta and scale the buoyancy forces by the
    // interval.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct WaterSettings
{
    double        fixedRateHz             = 60.0;
    std::uint32_t maximumOceans           = 16;
    std::uint32_t maximumRipplesPerOcean  = 64;
    std::uint32_t maximumRegisteredBodies = 4096;
    double        waterDensity            = 1000.0; // kilograms per cubic meter
    double        rippleSpeed             = 4.0;    // ring expansion (meters per second)
    double        rippleDecay             = 1.2;    // exponential decay per second
    double        rippleAttenuation       = 0.35;   // amplitude falloff per meter of radius
    double        rippleWavelength        = 1.6;    // ring crest spacing (meters)
    double        buoyancyScale           = 1.0;    // global buoyancy multiplier
    double        linearDragCoefficient   = 1.8;    // quadratic water drag per square meter of drag area
    double        angularDragCoefficient  = 3.0;    // angular damping torque per (rad/s) of submerged body
    double        wakeSpeedThreshold      = 1.2;    // minimum horizontal speed for wake emission (meters per second)
    double        wakeSpacingMeters       = 1.5;    // travelled distance between wake ripples (meters)
    double        wakeStrengthScale       = 0.35;   // wake ripple amplitude per (meter per second) of speed
    double        wakeSubmergedFraction   = 0.95;   // bodies at or above this submersion emit no wakes

    // A ripple retires when its exponential envelope decays below this
    // amplitude fraction of its initial strength.
    double        rippleRetireEnvelope    = 1.0e-3;

    // Simulation LOD: oceans below the Full band run their passes on a
    // staggered cadence; the per-band interval says how many fixed steps one
    // due pass covers (0 freezes that band, which is the Cached default).
    bool                              lodEnabled   = false;
    engine::util::SimulationFidelityThresholds lodThresholds{};
    std::uint32_t                     lodIntervals[engine::util::kSimulationFidelityBandCount] = {0u, 8u, 4u, 2u, 1u};
};

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
        if (const auto value = context.configurationFloatingInSection("physics_water", "linear_drag_coefficient");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.linearDragCoefficient = *value;
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
        if (const auto value = context.configurationFloatingInSection("physics_water", "wake_submerged_fraction");
            value && *value > 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.wakeSubmergedFraction = *value;
        if (const auto value = context.configurationFloatingInSection("physics_water", "ripple_retire_envelope");
            value && *value > 0.0 && *value < 1.0 && std::isfinite(*value))
            settings_.rippleRetireEnvelope = *value;
        settings_.lodEnabled = context.configurationBooleanInSection("physics_water", "lod_enabled").value_or(false);
        const auto readThreshold = [this, &context](const char *key, double &target)
        {
            if (const auto value = context.configurationFloatingInSection("physics_water", key);
                value && *value >= 0.0 && *value <= 1.0 && std::isfinite(*value))
                target = *value;
        };
        readThreshold("lod_full_threshold", settings_.lodThresholds.fullThreshold);
        readThreshold("lod_simplified_threshold", settings_.lodThresholds.simplifiedThreshold);
        readThreshold("lod_procedural_threshold", settings_.lodThresholds.proceduralThreshold);
        readThreshold("lod_animation_threshold", settings_.lodThresholds.animationThreshold);
        readThreshold("lod_demotion_margin", settings_.lodThresholds.demotionMargin);
        if (!engine::util::simulationFidelityThresholdsValid(settings_.lodThresholds))
        {
            settings_.lodThresholds = engine::util::SimulationFidelityThresholds{};
        }
        const auto readInterval = [this, &context](const char *key, std::uint32_t &target)
        {
            if (const auto value = context.configurationIntegerInSection("physics_water", key);
                value && *value >= 0 && *value <= 65536)
                target = static_cast<std::uint32_t>(*value);
        };
        readInterval("lod_interval_simplified", settings_.lodIntervals[engine::util::simulationFidelityIndex(engine::util::SimulationFidelity::Simplified)]);
        readInterval("lod_interval_procedural", settings_.lodIntervals[engine::util::simulationFidelityIndex(engine::util::SimulationFidelity::Procedural)]);
        readInterval("lod_interval_animation", settings_.lodIntervals[engine::util::simulationFidelityIndex(engine::util::SimulationFidelity::Animation)]);
        readInterval("lod_interval_cached", settings_.lodIntervals[engine::util::simulationFidelityIndex(engine::util::SimulationFidelity::Cached)]);
    }

    static void setError(WaterError *output, WaterError value) noexcept
    {
        engine::util::setModuleError(output, value);
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
            prepared.waveNumber = 2.0 * kPi / candidate.wavelength;
            prepared.amplitude  = candidate.amplitude;
            const double steepness = std::clamp(candidate.steepness, 0.0, 1.0);
            prepared.horizontalAmplitude = steepness > 0.0 && candidate.amplitude > 0.0
                                               ? steepness / (prepared.waveNumber * waveCount)
                                               : 0.0;
            prepared.omega = std::sqrt(kGravityForWaves * prepared.waveNumber);
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
        for (int iteration = 0; iteration < kGerstnerIterations; ++iteration)
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
            const double ringPhase = 2.0 * kPi * (radius - ringFront) / settings_.rippleWavelength;
            const double envelope  = std::exp(-ripple.age * settings_.rippleDecay - radius * settings_.rippleAttenuation)
                                    * (1.0 - distanceToRing / settings_.rippleWavelength);
            height += ripple.strength * envelope * std::cos(ringPhase);
        }
        return height;
    }

    // Builds the shape-dependent sample set from an already fetched body
    // state and description (the buoyancy pass fetches them batched; the
    // single-body queries fetch directly).
    void collectSamplePoints(const engine::physics_rigid::BodyState &state,
        const engine::physics_rigid::BodyDescription &description, std::vector<math::Vec3> &outPoints) const
    {
        outPoints.clear();
        using engine::physics_rigid::ShapeKind;
        const math::Vec3 center = state.transform.position;
        switch (description.shape.kind)
        {
        case ShapeKind::Sphere: {
            const double r = description.shape.sphereRadius * 0.5;
            outPoints.push_back(center);
            outPoints.push_back(center + math::Vec3{r, 0.0, 0.0});
            outPoints.push_back(center + math::Vec3{-r, 0.0, 0.0});
            outPoints.push_back(center + math::Vec3{0.0, r, 0.0});
            outPoints.push_back(center + math::Vec3{0.0, -r, 0.0});
            outPoints.push_back(center + math::Vec3{0.0, 0.0, r});
            outPoints.push_back(center + math::Vec3{0.0, 0.0, -r});
            break;
        }
        case ShapeKind::Box: {
            const double hx = description.shape.boxHalfExtents[0] * 0.7;
            const double hy = description.shape.boxHalfExtents[1] * 0.7;
            const double hz = description.shape.boxHalfExtents[2] * 0.7;
            for (const double sx : {-1.0, 1.0})
            {
                for (const double sy : {-1.0, 1.0})
                {
                    for (const double sz : {-1.0, 1.0})
                    {
                        outPoints.push_back(center + math::rotate(state.transform.rotation, math::Vec3{hx * sx, hy * sy, hz * sz}));
                    }
                }
            }
            outPoints.push_back(center);
            break;
        }
        case ShapeKind::Capsule: {
            // Segment samples along the capsule axis (five stations) plus
            // lateral offsets at the center, so the submersion fraction
            // varies smoothly as the body rides the waves.
            const math::Vec3 axis = math::rotate(state.transform.rotation,
                math::Vec3{0.0, description.shape.capsuleHalfLength, 0.0});
            for (const double fraction : {-1.0, -0.5, 0.0, 0.5, 1.0})
            {
                outPoints.push_back(center + axis * fraction);
            }
            const double lateral = description.shape.capsuleRadius * 0.5;
            outPoints.push_back(center + math::Vec3{lateral, 0.0, 0.0});
            outPoints.push_back(center + math::Vec3{-lateral, 0.0, 0.0});
            outPoints.push_back(center + math::Vec3{0.0, 0.0, lateral});
            outPoints.push_back(center + math::Vec3{0.0, 0.0, -lateral});
            break;
        }
        case ShapeKind::Plane:
        default:
            break;
        }
    }

    double bodyMassOf(const engine::physics_rigid::BodyDescription &description) const
    {
        return description.density > 0.0 ? description.density * shapeVolumeOf(description.shape) : description.mass;
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
        // capture the magnitude once per ocean so worlds with different
        // gravity settings stay independent.
        double gravity[3] = {0.0, -9.81, 0.0};
        if (rigid_->copyWorldGravity(rigid_->self, world, gravity))
        {
            ocean.gravityMagnitude = std::abs(gravity[1]) > 1.0e-6 ? std::abs(gravity[1])
                                                                    : std::sqrt(gravity[0] * gravity[0]
                                                                                + gravity[1] * gravity[1]
                                                                                + gravity[2] * gravity[2]);
        }
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so oceans sharing an interval spread their passes across
        // steps.
        ocean.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
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
    }

    bool setOceanDescriptionImpl(OceanHandle handle, const OceanDescription *description)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || description == nullptr || !validateDescription(*description))
        {
            return false;
        }
        ocean->description = *description;
        ocean->currentVelocity = effectiveCurrentOf(ocean->description);
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
        const double epsilon  = 0.05;
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
    // against the surface at the ocean's last buoyancy clock reading (0
    // before the first pass); used by the wake gate.
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
            if (surfaceHeight(ocean, point.x, point.z, ocean.lastBuoyancyTime) > point.y)
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
        // One batched state read per pass instead of one call per body.
        wakeStatesScratch_.resize(ocean.registeredBodies.size());
        rigid_->copyBodyStates(rigid_->self, ocean.world, ocean.registeredBodies.data(),
            static_cast<std::uint32_t>(ocean.registeredBodies.size()), wakeStatesScratch_.data());
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
        // Simulation LOD: resolve the band and staggered cadence; a due step
        // covers interval x the caller's delta (ripple aging is wave math
        // over wall time, so the enlarged delta keeps the pacing exact), and
        // Cached oceans freeze. Without lod_enabled every ocean runs Full
        // every step.
        double stepDelta = fixedDeltaSeconds;
        if (settings_.lodEnabled)
        {
            ocean->lodBand = engine::util::simulationFidelityForImportance(
                settings_.lodThresholds, ocean->importance, ocean->lodBand);
            const std::uint32_t interval
                = settings_.lodIntervals[engine::util::simulationFidelityIndex(ocean->lodBand)];
            if (interval == 0u)
            {
                ocean->cadence.interval = 1u;
                ocean->lodSkippedSteps += 1;
                return true;
            }
            ocean->cadence.interval = interval;
            if (!engine::util::simulationCadenceAdvance(ocean->cadence))
            {
                ocean->lodSkippedSteps += 1;
                return true;
            }
            stepDelta = fixedDeltaSeconds * static_cast<double>(interval);
        }
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
        statistics_.activeRipples = active;
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
        statistics_.registeredBodies = static_cast<std::uint32_t>(ocean->registeredBodies.size());
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
        statistics_.registeredBodies = static_cast<std::uint32_t>(ocean->registeredBodies.size());
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
            *outFraction = 0.0;
            return true;
        }
        collectSamplePoints(state, description, samplePoints_);
        if (samplePoints_.empty())
        {
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
            || fixedDeltaSeconds <= 0.0)
        {
            return false;
        }
        // Simulation LOD: resolve the band and staggered cadence; a due
        // pass scales the applied forces by the interval so the impulse per
        // wall time matches the full-rate schedule, and Cached oceans apply
        // nothing. Without lod_enabled every ocean runs Full every pass.
        double forceScale = 1.0;
        if (settings_.lodEnabled)
        {
            ocean->lodBand = engine::util::simulationFidelityForImportance(
                settings_.lodThresholds, ocean->importance, ocean->lodBand);
            const std::uint32_t interval
                = settings_.lodIntervals[engine::util::simulationFidelityIndex(ocean->lodBand)];
            if (interval == 0u)
            {
                ocean->cadence.interval = 1u;
                ocean->lodSkippedSteps += 1;
                return true;
            }
            ocean->cadence.interval = interval;
            if (!engine::util::simulationCadenceAdvance(ocean->cadence))
            {
                ocean->lodSkippedSteps += 1;
                return true;
            }
            forceScale = static_cast<double>(interval);
        }
        std::uint64_t applied = 0;
        // Remember the pass clock: the wake submersion gate samples the
        // surface at this reading until the next pass.
        ocean->lastBuoyancyTime = timeSeconds;
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
                statistics_.registeredBodies
                    = static_cast<std::uint32_t>(ocean->registeredBodies.size());
                continue;
            }
            const engine::physics_rigid::BodyState &state = buoyancyStates_[index];
            const engine::physics_rigid::BodyDescription &description = buoyancyDescriptions_[index];
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
            double shapeVolumeMeters = ocean->bodyVolumes[std::min(index, ocean->bodyVolumes.size()) - 1];
            if (!(shapeVolumeMeters >= 0.0))
            {
                shapeVolumeMeters = shapeVolumeOf(description.shape);
                ocean->bodyVolumes[std::min(index, ocean->bodyVolumes.size()) - 1] = shapeVolumeMeters;
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
                const double magnitude = 0.5 * settings_.waterDensity * settings_.linearDragCoefficient * dragArea
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
