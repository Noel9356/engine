// physics_fields module implementation.
//
// Force fields over a physics_rigid world. The module owns no simulation
// state: wind fields are analytic functions of position and time, force
// volumes are placement records, and every application drives forces and
// impulses through the physics_rigid capability's apply* entry points inside
// the consumer's fixed update, so fields compose deterministically with any
// other force source.

#include "records.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/collider_metrics.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_fields
{
namespace
{

namespace math = engine::physics_rigid::math;

// Shared-vector helpers for the flat double[3] samples the wind model reads.
double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double length3(const double v[3])
{
    return std::sqrt(dot3(v, v));
}

// Deterministic 3D value noise: integer lattice hashing with cubic-smooth
// interpolation between lattice values. The hash mixes three coordinates
// into one pseudo-random lattice value; no state, fully repeatable.

// Largest lattice coordinate the int64 walk accepts: callers feed wind and
// turbulence coordinates derived from user positions and base speeds, which
// can overflow to Inf or leave the int64 range; casting such a double is
// undefined behaviour. Non-finite coordinates sample the neutral zero
// lattice point; out-of-range finite coordinates clamp to the boundary
// (the noise is a hash - the boundary sample is as good as any). The limit
// keeps room for the +1 corner step inside int64.
constexpr double kLatticeCoordinateLimit = 4611686018427387904.0; // 2^62

std::int64_t latticeCoordinate(double floored)
{
    if (!std::isfinite(floored))
    {
        return 0;
    }
    if (floored >= kLatticeCoordinateLimit)
    {
        return static_cast<std::int64_t>(kLatticeCoordinateLimit) - 1;
    }
    if (floored <= -kLatticeCoordinateLimit)
    {
        return -static_cast<std::int64_t>(kLatticeCoordinateLimit);
    }
    return static_cast<std::int64_t>(floored);
}

double hashLattice(std::int64_t x, std::int64_t y, std::int64_t z)
{
    std::uint64_t h = static_cast<std::uint64_t>(x) * 0x9E3779B185EBCA87ull;
    h ^= static_cast<std::uint64_t>(y) * 0xC2B2AE3D27D4EB4Full;
    h ^= static_cast<std::uint64_t>(z) * 0x165667B19E3779F9ull;
    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    // Map into [-1, 1).
    return static_cast<double>(h >> 11) / static_cast<double>(1ull << 53) * 2.0 - 1.0;
}

double valueNoise3(double x, double y, double z)
{
    const double floorX = std::floor(x);
    const double floorY = std::floor(y);
    const double floorZ = std::floor(z);
    const double fx = x - floorX;
    const double fy = y - floorY;
    const double fz = z - floorZ;
    const double sx = fx * fx * (3.0 - 2.0 * fx);
    const double sy = fy * fy * (3.0 - 2.0 * fy);
    const double sz = fz * fz * (3.0 - 2.0 * fz);
    const std::int64_t ix = latticeCoordinate(floorX);
    const std::int64_t iy = latticeCoordinate(floorY);
    const std::int64_t iz = latticeCoordinate(floorZ);
    double value = 0.0;
    for (int corner = 0; corner < 8; ++corner)
    {
        const double weightX = (corner & 1) != 0 ? sx : 1.0 - sx;
        const double weightY = (corner & 2) != 0 ? sy : 1.0 - sy;
        const double weightZ = (corner & 4) != 0 ? sz : 1.0 - sz;
        value += weightX * weightY * weightZ
                 * hashLattice(ix + ((corner & 1) != 0 ? 1 : 0), iy + ((corner & 2) != 0 ? 1 : 0),
                     iz + ((corner & 4) != 0 ? 1 : 0));
    }
    return value;
}

// Closed-form solid volume, reference drag area, and drag coefficient of a
// validated rigid shape come from the shared engine::util::collider_metrics
// vocabulary (the same formulas the water module's buoyancy uses, covering
// every solid shape including cylinders).


struct ForceVolumeRecord
{
    ForceVolumeHandle       handle = 0;
    engine::physics_rigid::WorldHandle world = 0;
    ForceVolumeDescription  description{};

    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct FieldsSettings
{
    std::uint32_t maximumWindFields   = 64;
    std::uint32_t maximumForceVolumes = 256;
    double        airDensity          = 1.225; // kilograms per cubic meter
    double        windDragScale       = 1.0;   // global drag multiplier

    // Simulation LOD: fields below the Full band run their passes on a
    // staggered cadence; the per-band interval says how many fixed steps one
    // due pass covers, and the applied force scales with it (0 freezes that
    // band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

class FieldsModule final : public ModuleInterface
{
public:
    FieldsModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_fields", "maximum_wind_fields");
            value && *value > 0 && *value <= 4096)
            settings_.maximumWindFields = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fields", "maximum_force_volumes");
            value && *value > 0 && *value <= 4096)
            settings_.maximumForceVolumes = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_fields", "air_density");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.airDensity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_fields", "wind_drag_scale");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.windDragScale = *value;

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
        winds_.clear();
        windIndexByHandle_.clear();
        forceVolumes_.clear();
        forceVolumeIndexByHandle_.clear();
        nextWindHandle_        = 1;
        nextForceVolumeHandle_ = 1;
        statistics_            = FieldsStatistics{};
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == FieldsCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_fields",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_fields"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_fields");
    }

    static void setError(FieldsError *output, FieldsError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    bool setWindFieldImportanceImpl(WindHandle handle, double importance)
    {
        WindFieldRecord *wind = findWind(handle);
        if (wind == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        wind->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool setForceVolumeImportanceImpl(ForceVolumeHandle handle, double importance)
    {
        ForceVolumeRecord *volume = findForceVolume(handle);
        if (volume == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        volume->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    WindFieldRecord *findWind(WindHandle handle)
    {
        const auto iterator = windIndexByHandle_.find(handle);
        return iterator != windIndexByHandle_.end() ? &winds_[iterator->second] : nullptr;
    }

    ForceVolumeRecord *findForceVolume(ForceVolumeHandle handle)
    {
        const auto iterator = forceVolumeIndexByHandle_.find(handle);
        return iterator != forceVolumeIndexByHandle_.end() ? &forceVolumes_[iterator->second] : nullptr;
    }

    // Body list scratch: refreshed per apply call from the rigid world.
    // Only awake dynamic bodies collect: static bodies never move, and
    // kinematic bodies are driven by their mover rather than by fields
    // (matching the apply entry points' documentation).
    bool collectDynamicBodies(engine::physics_rigid::WorldHandle world,
        std::vector<engine::physics_rigid::BodyHandle> &outHandles,
        std::vector<engine::physics_rigid::BodyState> &outStates,
        std::vector<engine::physics_rigid::BodyDescription> &outDescriptions)
    {
        outHandles.clear();
        outStates.clear();
        outDescriptions.clear();
        if (rigid_ == nullptr)
        {
            return false;
        }
        const std::uint32_t count = rigid_->bodyCount(rigid_->self, world);
        if (count == 0)
        {
            return true;
        }
        bodyHandles_.resize(count);
        std::uint32_t required = 0;
        if (!rigid_->copyBodyHandles(rigid_->self, world, bodyHandles_.data(), count, &required) || required > count)
        {
            return false;
        }
        // Batched reads: one call per world instead of two per body for the
        // awake-body scan.
        bodyStatesScratch_.resize(count);
        bodyDescriptionsScratch_.resize(count);
        const std::uint32_t resolvedStates = rigid_->copyBodyStates(rigid_->self, world, bodyHandles_.data(), count,
            bodyStatesScratch_.data());
        const std::uint32_t resolvedDescriptions = rigid_->copyBodyDescriptions(rigid_->self, world,
            bodyHandles_.data(), count, bodyDescriptionsScratch_.data());
        for (std::uint32_t index = 0; index < count; ++index)
        {
            if (index >= resolvedStates || index >= resolvedDescriptions)
            {
                break;
            }
            if (bodyStatesScratch_[index].awake
                && bodyDescriptionsScratch_[index].kind == engine::physics_rigid::BodyKind::Dynamic)
            {
                outHandles.push_back(bodyHandles_[index]);
                outStates.push_back(bodyStatesScratch_[index]);
                outDescriptions.push_back(bodyDescriptionsScratch_[index]);
            }
        }
        return true;
    }

    // Force-scaling mass of one body: the density-sourced shapes derive it
    // from the shared shape volume, the explicit-mass shapes carry their
    // description mass.
    static double bodyMass(const engine::physics_rigid::BodyDescription &description)
    {
        return engine::util::colliderMass(description);
    }

    // ---- wind field evaluation ----------------------------------------------

    void evaluateWind(const WindFieldRecord &wind, const double position[3], double timeSeconds, double outVelocity[3])
        const
    {
        const WindFieldDescription &description = wind.description;
        // Creation rejects non-positive wavelengths; the floor keeps a
        // legacy record (one predating the validation) well-defined anyway.
        const double wavelength = description.turbulenceWavelength > 1.0e-6 ? description.turbulenceWavelength : 1.0;

        // Global gust cycle plus a traveling front sampled along the wind
        // direction; the front moves downwind at the base speed.
        const double cyclePhase
            = 2.0 * math::kPi * (description.gustPeriodSeconds > 1.0e-6 ? timeSeconds / description.gustPeriodSeconds : 0.0);
        const double frontCoordinate
            = (dot3(position, wind.direction) - description.baseSpeed * timeSeconds) / wavelength;
        const double gust = description.gustStrength
                            * (0.6 * std::sin(cyclePhase) + 0.4 * valueNoise3(frontCoordinate * 0.5, 0.0, 17.31));

        // Two spatial turbulence octaves drifting downwind.
        double turbulence = 0.0;
        if (description.turbulenceStrength > 0.0)
        {
            const double drift = description.baseSpeed * timeSeconds / wavelength;
            const double x = (position[0] - wind.direction[0] * drift) / wavelength;
            const double y = (position[1] - wind.direction[1] * drift) / wavelength;
            const double z = (position[2] - wind.direction[2] * drift) / wavelength;
            turbulence = description.turbulenceStrength
                         * (valueNoise3(x, y, z) * 0.65 + valueNoise3(x * 2.7 + 5.13, y * 2.7, z * 2.7) * 0.35);
        }

        const double speed = description.baseSpeed * std::max(1.0 + gust + turbulence, 0.0);
        outVelocity[0] = wind.direction[0] * speed;
        outVelocity[1] = wind.direction[1] * speed;
        outVelocity[2] = wind.direction[2] * speed;
    }

    // ---- capability implementations -----------------------------------------

    WindHandle createWindFieldImpl(
        engine::physics_rigid::WorldHandle world, const WindFieldDescription *description, FieldsError *error)
    {
        setError(error, FieldsError::None);
        if (rigid_ == nullptr)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidWind;
        }
        if (description == nullptr)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidWind;
        }
        if (winds_.size() >= settings_.maximumWindFields)
        {
            setError(error, FieldsError::CapacityExceeded);
            return invalidWind;
        }
        if (!std::isfinite(description->direction[0]) || !std::isfinite(description->direction[1])
            || !std::isfinite(description->direction[2]) || !std::isfinite(description->baseSpeed)
            || !std::isfinite(description->gustStrength) || !std::isfinite(description->gustPeriodSeconds)
            || !std::isfinite(description->turbulenceWavelength) || description->turbulenceWavelength <= 0.0
            || !std::isfinite(description->turbulenceStrength)
            || description->baseSpeed < 0.0 || description->gustStrength < 0.0
            || description->gustStrength > 2.0 || description->gustPeriodSeconds < 0.0
            || description->turbulenceStrength < 0.0 || description->turbulenceStrength > 2.0
            // Wind beyond any physical scale (the strongest hurricanes are
            // ~1e2 m/s) is rejected so every downstream product - gust and
            // turbulence multipliers up to 5x, consumers squaring the speed -
            // stays finite.
            || description->baseSpeed > 1.0e6)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidWind;
        }
        const double length = length3(description->direction);
        if (length < 1.0e-9)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidWind;
        }
        if (nextWindHandle_ == invalidWind)
        {
            setError(error, FieldsError::CapacityExceeded);
            return invalidWind;
        }
        // The wind's world handle validates through a gravity probe: a
        // world the rigid module does not know is rejected here.
        double gravityProbe[3] = {0.0, 0.0, 0.0};
        if (!rigid_->copyWorldGravity(rigid_->self, world, gravityProbe))
        {
            setError(error, FieldsError::InvalidHandle);
            return invalidWind;
        }
        WindFieldRecord record;
        record.handle      = nextWindHandle_;
        nextWindHandle_    = nextWindHandle_ == std::numeric_limits<WindHandle>::max() ? invalidWind : nextWindHandle_ + 1;
        record.world       = world;
        record.description = *description;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            record.direction[axis] = description->direction[axis] / length;
        }
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal.
        record.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        winds_.push_back(record);
        windIndexByHandle_.emplace(record.handle, winds_.size() - 1);
        statistics_.windFieldCount = static_cast<std::uint32_t>(winds_.size());
        return record.handle;
    }

    void destroyWindFieldImpl(WindHandle handle)
    {
        const auto iterator = windIndexByHandle_.find(handle);
        if (iterator == windIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        windIndexByHandle_.erase(iterator);
        if (index + 1 != winds_.size())
        {
            winds_[index]                            = winds_.back();
            windIndexByHandle_[winds_[index].handle] = index;
        }
        winds_.pop_back();
        statistics_.windFieldCount = static_cast<std::uint32_t>(winds_.size());
    }

    bool sampleWindVelocityImpl(WindHandle handle, const double position[3], double timeSeconds, double outVelocity[3])
    {
        const WindFieldRecord *wind = findWind(handle);
        if (wind == nullptr || position == nullptr || outVelocity == nullptr || !std::isfinite(timeSeconds))
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        evaluateWind(*wind, position, timeSeconds, outVelocity);
        return true;
    }

    bool applyWindDragImpl(WindHandle handle, double timeSeconds, double fixedDeltaSeconds)
    {
        WindFieldRecord *wind = findWind(handle);
        if (wind == nullptr || rigid_ == nullptr || !std::isfinite(timeSeconds) || !std::isfinite(fixedDeltaSeconds)
            || fixedDeltaSeconds <= 0.0)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a due
        // pass scales the applied drag by the interval so the impulse per
        // wall time matches the full-rate schedule, and Cached winds apply
        // nothing. Without lod_enabled every wind runs Full every pass.
        const std::uint32_t forceInterval = engine::util::simulationLodResolvePass(wind->lodBand,
            wind->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            wind->importance,
            &wind->lodSkippedSteps);
        if (forceInterval == 0u)
        {
            return true;
        }
        const double forceScale = static_cast<double>(forceInterval);
        const bool bodiesCollected = collectDynamicBodies(wind->world, applyHandles_, applyStates_, applyDescriptions_);
        if (!bodiesCollected)
        {
            // The collection grew past this pass's capacity (the world gained
            // bodies mid-step): apply nothing this pass rather than applying
            // to a partial read.
            return false;
        }
        for (std::size_t index = 0; index < applyHandles_.size(); ++index)
        {
            const double samplePoint[3] = {applyStates_[index].transform.position.x,
                applyStates_[index].transform.position.y,
                applyStates_[index].transform.position.z};
            double windVelocity[3] = {0.0, 0.0, 0.0};
            evaluateWind(*wind, samplePoint, timeSeconds, windVelocity);
            const double relative[3] = {windVelocity[0] - applyStates_[index].linearVelocity.x,
                windVelocity[1] - applyStates_[index].linearVelocity.y,
                windVelocity[2] - applyStates_[index].linearVelocity.z};
            const double relativeSpeed = length3(relative);
            if (relativeSpeed < 1.0e-6)
            {
                continue;
            }
            double area            = 0.0;
            double dragCoefficient = 0.0;
            const engine::util::ColliderAerodynamics aerodynamics
                = engine::util::colliderAerodynamics(applyDescriptions_[index].shape);
            area            = aerodynamics.referenceArea;
            dragCoefficient = aerodynamics.dragCoefficient;
            if (area <= 0.0)
            {
                continue;
            }
            // Interval-scaled force: a pass that covers N fixed steps
            // applies roughly N times the drag so the impulse per wall time
            // matches the full-rate schedule.
            const double scale = 0.5 * settings_.airDensity * dragCoefficient * area * relativeSpeed * settings_.windDragScale
                                 * forceScale;
            const double force[3] = {relative[0] * scale, relative[1] * scale, relative[2] * scale};
            const double centerPoint[3] = {applyStates_[index].transform.position.x,
                applyStates_[index].transform.position.y,
                applyStates_[index].transform.position.z};
            if (rigid_->applyForceAtPoint(rigid_->self, wind->world, applyHandles_[index], force, centerPoint))
            {
                ++statistics_.dragApplications;
            }
        }
        return true;
    }

    ForceVolumeHandle createForceVolumeImpl(
        engine::physics_rigid::WorldHandle world, const ForceVolumeDescription *description, FieldsError *error)
    {
        setError(error, FieldsError::None);
        if (rigid_ == nullptr)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidForceVolume;
        }
        if (description == nullptr)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidForceVolume;
        }
        if (forceVolumes_.size() >= settings_.maximumForceVolumes)
        {
            setError(error, FieldsError::CapacityExceeded);
            return invalidForceVolume;
        }
        if (!std::isfinite(description->center[0]) || !std::isfinite(description->center[1])
            || !std::isfinite(description->center[2]) || !std::isfinite(description->halfExtents[0])
            || !std::isfinite(description->halfExtents[1]) || !std::isfinite(description->halfExtents[2])
            || !std::isfinite(description->radius) || !std::isfinite(description->acceleration[0])
            || !std::isfinite(description->acceleration[1]) || !std::isfinite(description->acceleration[2])
            || !std::isfinite(description->radialStrength) || description->halfExtents[0] <= 0.0
            || description->halfExtents[1] <= 0.0 || description->halfExtents[2] <= 0.0 || description->radius < 0.0)
        {
            setError(error, FieldsError::InvalidDescription);
            return invalidForceVolume;
        }
        if (nextForceVolumeHandle_ == invalidForceVolume)
        {
            setError(error, FieldsError::CapacityExceeded);
            return invalidForceVolume;
        }
        // The volume's world handle validates through a gravity probe: a
        // world the rigid module does not know is rejected here.
        double gravityProbe[3] = {0.0, 0.0, 0.0};
        if (!rigid_->copyWorldGravity(rigid_->self, world, gravityProbe))
        {
            setError(error, FieldsError::InvalidHandle);
            return invalidForceVolume;
        }
        ForceVolumeRecord record;
        record.handle            = nextForceVolumeHandle_;
        nextForceVolumeHandle_   = nextForceVolumeHandle_ == std::numeric_limits<ForceVolumeHandle>::max()
                                       ? invalidForceVolume
                                       : nextForceVolumeHandle_ + 1;
        record.world             = world;
        record.description       = *description;
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal.
        record.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        forceVolumes_.push_back(record);
        forceVolumeIndexByHandle_.emplace(record.handle, forceVolumes_.size() - 1);
        statistics_.forceVolumeCount = static_cast<std::uint32_t>(forceVolumes_.size());
        return record.handle;
    }

    void destroyForceVolumeImpl(ForceVolumeHandle handle)
    {
        const auto iterator = forceVolumeIndexByHandle_.find(handle);
        if (iterator == forceVolumeIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        forceVolumeIndexByHandle_.erase(iterator);
        if (index + 1 != forceVolumes_.size())
        {
            forceVolumes_[index]                                  = forceVolumes_.back();
            forceVolumeIndexByHandle_[forceVolumes_[index].handle] = index;
        }
        forceVolumes_.pop_back();
        statistics_.forceVolumeCount = static_cast<std::uint32_t>(forceVolumes_.size());
    }

    bool setForceVolumeImpl(ForceVolumeHandle handle, const ForceVolumeDescription *description)
    {
        ForceVolumeRecord *record = findForceVolume(handle);
        if (record == nullptr || description == nullptr)
        {
            return false;
        }
        if (!std::isfinite(description->center[0]) || !std::isfinite(description->center[1])
            || !std::isfinite(description->center[2]) || !std::isfinite(description->halfExtents[0])
            || !std::isfinite(description->halfExtents[1]) || !std::isfinite(description->halfExtents[2])
            || !std::isfinite(description->radius) || !std::isfinite(description->acceleration[0])
            || !std::isfinite(description->acceleration[1]) || !std::isfinite(description->acceleration[2])
            || !std::isfinite(description->radialStrength) || description->halfExtents[0] <= 0.0
            || description->halfExtents[1] <= 0.0 || description->halfExtents[2] <= 0.0 || description->radius < 0.0)
        {
            return false;
        }
        record->description = *description;
        return true;
    }

    bool applyForceVolumesImpl(double fixedDeltaSeconds)
    {
        // The delta carries no physics here: volumes apply accelerations
        // scaled by the simulation-LOD interval (the impulse-per-wall-time
        // compensation), so the caller's fixed delta is accepted for
        // interface consistency only.
        (void)fixedDeltaSeconds;
        if (rigid_ == nullptr)
        {
            return false;
        }
        // Body collection memo for the pass: volumes sharing a world (the
        // common case) collect its awake dynamic bodies once instead of
        // once per volume.
        engine::physics_rigid::WorldHandle collectedWorld = engine::physics_rigid::invalidWorld;
        for (ForceVolumeRecord &volume : forceVolumes_)
        {
            if (!volume.description.enabled)
            {
                continue;
            }
            // Simulation LOD: resolve the band and the staggered cadence; a
            // due pass scales the applied forces by the interval so the
            // impulse per wall time matches the full-rate schedule, and
            // Cached volumes apply nothing. Without lod_enabled every
            // volume runs Full every pass.
            const std::uint32_t forceInterval = engine::util::simulationLodResolvePass(volume.lodBand,
                volume.cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                volume.importance,
                &volume.lodSkippedSteps);
            if (forceInterval == 0u)
            {
                continue;
            }
            // Interval-scaled forces: a pass covering N fixed steps applies
            // roughly N times the force so the impulse per wall time matches
            // the full-rate schedule.
            const double passScale = static_cast<double>(forceInterval);
            if (volume.world != collectedWorld)
            {
                if (!collectDynamicBodies(volume.world, applyHandles_, applyStates_, applyDescriptions_))
                {
                    continue; // the read could not resolve every body; skip the volume this pass
                }
                collectedWorld = volume.world;
            }
            for (std::size_t index = 0; index < applyHandles_.size(); ++index)
            {
                const engine::physics_rigid::BodyState &state = applyStates_[index];
                double forceScale = 0.0;
                double direction[3] = {0.0, 0.0, 0.0};
                if (volume.description.kind == ForceVolumeDescription::Kind::Uniform)
                {
                    const double offset[3] = {state.transform.position.x - volume.description.center[0],
                        state.transform.position.y - volume.description.center[1],
                        state.transform.position.z - volume.description.center[2]};
                    if (std::abs(offset[0]) > volume.description.halfExtents[0]
                        || std::abs(offset[1]) > volume.description.halfExtents[1]
                        || std::abs(offset[2]) > volume.description.halfExtents[2])
                    {
                        continue;
                    }
                    forceScale = bodyMass(applyDescriptions_[index]);
                    direction[0] = volume.description.acceleration[0];
                    direction[1] = volume.description.acceleration[1];
                    direction[2] = volume.description.acceleration[2];
                }
                else
                {
                    if (volume.description.radius <= 0.0 || volume.description.radialStrength == 0.0)
                    {
                        continue;
                    }
                    const double offset[3] = {state.transform.position.x - volume.description.center[0],
                        state.transform.position.y - volume.description.center[1],
                        state.transform.position.z - volume.description.center[2]};
                    const double distance = length3(offset);
                    if (distance > volume.description.radius || distance < 1.0e-6)
                    {
                        continue;
                    }
                    const double falloff = (1.0 - distance / volume.description.radius)
                                           / std::max(distance * distance, 1.0);
                    forceScale = bodyMass(applyDescriptions_[index]) * volume.description.radialStrength * falloff;
                    direction[0] = offset[0] / distance;
                    direction[1] = offset[1] / distance;
                    direction[2] = offset[2] / distance;
                }
                const double force[3] = {direction[0] * forceScale, direction[1] * forceScale, direction[2] * forceScale};
                const double centerPoint[3] = {state.transform.position.x, state.transform.position.y,
                    state.transform.position.z};
                const double scaledForce[3] = {force[0] * passScale, force[1] * passScale, force[2] * passScale};
                if (rigid_->applyForceAtPoint(rigid_->self, volume.world, applyHandles_[index], scaledForce, centerPoint))
                {
                    ++statistics_.volumeApplications;
                }
            }
        }
        return true;
    }

    bool applyRadialImpulseImpl(engine::physics_rigid::WorldHandle world, const double position[3],
        double impulseMagnitude, double radius, double upwardBias)
    {
        if (rigid_ == nullptr || position == nullptr || !std::isfinite(impulseMagnitude)
            || !std::isfinite(radius) || !std::isfinite(upwardBias) || impulseMagnitude < 0.0 || radius <= 0.0)
        {
            return false;
        }
        if (!collectDynamicBodies(world, applyHandles_, applyStates_, applyDescriptions_))
        {
            return false; // the read could not resolve every body; apply nothing this pass
        }
        for (std::size_t index = 0; index < applyHandles_.size(); ++index)
        {
            const engine::physics_rigid::BodyState &state = applyStates_[index];
            const double offset[3] = {state.transform.position.x - position[0],
                state.transform.position.y - position[1],
                state.transform.position.z - position[2]};
            const double distance = length3(offset);
            if (distance > radius)
            {
                continue;
            }
            const double falloff = 1.0 - distance / radius;
            // The blast direction biases along the world's up axis (+Y, the
            // engine's gravity convention).
            double direction[3] = {0.0, 1.0, 0.0};
            if (distance > 1.0e-6)
            {
                direction[0] = offset[0] / distance;
                direction[1] = offset[1] / distance + upwardBias;
                direction[2] = offset[2] / distance;
            }
            else
            {
                direction[1] = 1.0 + upwardBias;
            }
            const double directionLength = length3(direction);
            if (directionLength < 1.0e-9)
            {
                continue;
            }
            const double scale = impulseMagnitude * falloff / directionLength;
            if (!std::isfinite(scale))
            {
                // The magnitude x falloff product overflowed; the rigid
                // entry point rejects non-finite impulses anyway - skip the
                // call instead of reporting a blast that never happened.
                continue;
            }
            const double impulse[3] = {direction[0] * scale, direction[1] * scale, direction[2] * scale};
            const double centerPoint[3] = {state.transform.position.x, state.transform.position.y,
                state.transform.position.z};
            rigid_->applyImpulseAtPoint(rigid_->self, world, applyHandles_[index], impulse, centerPoint);
        }
        return true;
    }

    bool fieldsStatisticsImpl(FieldsStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        statistics_.lodSkippedSteps = 0;
        for (const WindFieldRecord &wind : winds_)
        {
            statistics_.lodSkippedSteps += wind.lodSkippedSteps;
        }
        for (const ForceVolumeRecord &volume : forceVolumes_)
        {
            statistics_.lodSkippedSteps += volume.lodSkippedSteps;
        }
        *outStatistics = statistics_;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsFieldsCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_FIELDS_BIND(field, method)                                                                      \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<FieldsModule *>(self)->method(args...); }
        ENGINE_PHYSICS_FIELDS_BIND(createWindField, createWindFieldImpl);
        ENGINE_PHYSICS_FIELDS_BIND(destroyWindField, destroyWindFieldImpl);
        ENGINE_PHYSICS_FIELDS_BIND(sampleWindVelocity, sampleWindVelocityImpl);
        ENGINE_PHYSICS_FIELDS_BIND(applyWindDrag, applyWindDragImpl);
        ENGINE_PHYSICS_FIELDS_BIND(createForceVolume, createForceVolumeImpl);
        ENGINE_PHYSICS_FIELDS_BIND(destroyForceVolume, destroyForceVolumeImpl);
        ENGINE_PHYSICS_FIELDS_BIND(setForceVolume, setForceVolumeImpl);
        ENGINE_PHYSICS_FIELDS_BIND(applyForceVolumes, applyForceVolumesImpl);
        ENGINE_PHYSICS_FIELDS_BIND(applyRadialImpulse, applyRadialImpulseImpl);
        ENGINE_PHYSICS_FIELDS_BIND(fieldsStatistics, fieldsStatisticsImpl);
        ENGINE_PHYSICS_FIELDS_BIND(setWindFieldImportance, setWindFieldImportanceImpl);
        ENGINE_PHYSICS_FIELDS_BIND(setForceVolumeImportance, setForceVolumeImportanceImpl);
#undef ENGINE_PHYSICS_FIELDS_BIND
    }

    ParticipantManifest manifest_;
    FieldsCapability    capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    FieldsSettings settings_;

    FieldsStatistics statistics_{};

    std::vector<WindFieldRecord>                             winds_;
    std::unordered_map<WindHandle, std::size_t>              windIndexByHandle_;
    WindHandle                                               nextWindHandle_ = 1;
    std::vector<ForceVolumeRecord>                           forceVolumes_;
    std::unordered_map<ForceVolumeHandle, std::size_t>       forceVolumeIndexByHandle_;
    ForceVolumeHandle                                        nextForceVolumeHandle_ = 1;

    // Per-apply scratch shared across the apply entry points.
    std::vector<engine::physics_rigid::BodyHandle>       applyHandles_;
    std::vector<engine::physics_rigid::BodyState>        applyStates_;
    std::vector<engine::physics_rigid::BodyDescription>  applyDescriptions_;
    std::vector<engine::physics_rigid::BodyHandle>       bodyHandles_;
    std::vector<engine::physics_rigid::BodyState>        bodyStatesScratch_;
    std::vector<engine::physics_rigid::BodyDescription>  bodyDescriptionsScratch_;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;
};

} // namespace
} // namespace engine::physics_fields

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_fields capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_fields"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_fields",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_fields::FieldsModule(); },
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
