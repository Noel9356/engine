// physics_water module implementation.
//
// Gerstner ocean over a physics_rigid world: the surface is an analytic
// deep-water wave sum with superposed expanding ripple rings; buoyancy
// applies per-body submersion-sampled forces, drag, and angular damping
// through the rigid capability's apply entry points inside the consumer's
// fixed update, so water physics composes deterministically with every
// other force source.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/physics_water/capabilities.hpp>

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
    math::Vec3 direction;      // unit, XZ plane
    double     waveNumber      = 0.0;  // k = 2*pi / wavelength
    double     amplitude       = 0.0;
    double     steepness       = 0.0;
    double     omega           = 0.0;  // deep-water dispersion: omega = sqrt(g * k)
};

struct Ripple
{
    bool   active    = false;
    double centerX   = 0.0;
    double centerZ   = 0.0;
    double age       = 0.0;
    double strength  = 0.0; // initial ring amplitude (meters)
};

struct OceanRecord
{
    OceanHandle handle = invalidOcean;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    OceanDescription description{};

    std::vector<PreparedWave> waves;

    std::vector<Ripple> ripples;

    std::vector<engine::physics_rigid::BodyHandle> registeredBodies;
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

    static void setError(WaterError *output, WaterError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
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
                || !std::isfinite(candidate.steepness) || candidate.steepness < 0.0 || candidate.steepness >= 1.0)
            {
                return false;
            }
        }
        return true;
    }

    void prepareWaves(OceanRecord &ocean)
    {
        ocean.waves.clear();
        ocean.waves.reserve(ocean.description.waveCount);
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
            prepared.steepness  = candidate.steepness;
            prepared.omega      = std::sqrt(kGravityForWaves * prepared.waveNumber);
            ocean.waves.push_back(prepared);
        }
    }

    // Wave sum height at a world (x, z) and time.
    double waveHeight(const OceanRecord &ocean, double x, double z, double timeSeconds) const
    {
        double height = ocean.description.waterLevel;
        for (const PreparedWave &wave : ocean.waves)
        {
            const double phase = wave.waveNumber * (wave.direction.x * x + wave.direction.z * z) - wave.omega * timeSeconds;
            height += wave.amplitude * std::sin(phase);
        }
        return height;
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

    double surfaceHeight(const OceanRecord &ocean, double x, double z, double timeSeconds) const
    {
        return waveHeight(ocean, x, z, timeSeconds) + rippleHeight(ocean, x, z);
    }

    void collectSamplePoints(const OceanRecord &ocean, engine::physics_rigid::BodyHandle body,
        std::vector<math::Vec3> &outPoints)
    {
        outPoints.clear();
        if (rigid_ == nullptr)
        {
            return;
        }
        engine::physics_rigid::BodyState state;
        engine::physics_rigid::BodyDescription description;
        if (!rigid_->copyBodyState(rigid_->self, ocean.world, body, &state)
            || !rigid_->copyBodyDescription(rigid_->self, ocean.world, body, &description))
        {
            return;
        }
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
        ocean.ripples.assign(settings_.maximumRipplesPerOcean, Ripple{});
        prepareWaves(ocean);
        // The buoyancy pass pushes opposite to the world's gravity; capture
        // the gravity magnitude from this world once per module.
        double gravity[3] = {0.0, -9.81, 0.0};
        if (rigid_->copyWorldGravity(rigid_->self, world, gravity))
        {
            gravityMagnitude_ = std::abs(gravity[1]) > 1.0e-6 ? std::abs(gravity[1])
                                                                : std::sqrt(gravity[0] * gravity[0]
                                                                              + gravity[1] * gravity[1]
                                                                              + gravity[2] * gravity[2]);
        }
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
        // Ring-buffer placement: the oldest slot (or first inactive slot).
        std::size_t slot = ocean->ripples.size();
        for (std::size_t index = 0; index < ocean->ripples.size(); ++index)
        {
            if (!ocean->ripples[index].active)
            {
                slot = index;
                break;
            }
            if (slot == ocean->ripples.size() || ocean->ripples[index].age > ocean->ripples[slot].age)
            {
                slot = index;
            }
        }
        if (slot == ocean->ripples.size())
        {
            return false;
        }
        Ripple &ripple    = ocean->ripples[slot];
        ripple.active     = true;
        ripple.centerX    = center[0];
        ripple.centerZ    = center[2];
        ripple.age        = 0.0;
        ripple.strength   = strength;
        return true;
    }

    bool stepRipplesImpl(OceanHandle handle, double fixedDeltaSeconds)
    {
        OceanRecord *ocean = findOcean(handle);
        if (ocean == nullptr || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0
            || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        std::uint32_t active = 0;
        for (Ripple &ripple : ocean->ripples)
        {
            if (!ripple.active)
            {
                continue;
            }
            ripple.age += fixedDeltaSeconds;
            // A ripple retires when its envelope has decayed below a
            // thousandth of its initial amplitude.
            if (std::exp(-ripple.age * settings_.rippleDecay) < 1.0e-3)
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
        if (std::find(ocean->registeredBodies.begin(), ocean->registeredBodies.end(), body)
            != ocean->registeredBodies.end())
        {
            return true; // already registered
        }
        ocean->registeredBodies.push_back(body);
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
        const auto entry = std::find(ocean->registeredBodies.begin(), ocean->registeredBodies.end(), body);
        if (entry == ocean->registeredBodies.end())
        {
            return false;
        }
        ocean->registeredBodies.erase(entry);
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
        collectSamplePoints(*ocean, body, samplePoints_);
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
        std::uint64_t applied = 0;
        for (std::size_t index = 0; index < ocean->registeredBodies.size();)
        {
            const engine::physics_rigid::BodyHandle body = ocean->registeredBodies[index];
            engine::physics_rigid::BodyState state;
            engine::physics_rigid::BodyDescription description;
            if (!rigid_->copyBodyState(rigid_->self, ocean->world, body, &state)
                || !rigid_->copyBodyDescription(rigid_->self, ocean->world, body, &description))
            {
                // The body died in the rigid world: free the registration.
                ocean->registeredBodies.erase(ocean->registeredBodies.begin()
                                              + static_cast<std::ptrdiff_t>(index));
                statistics_.registeredBodies
                    = static_cast<std::uint32_t>(ocean->registeredBodies.size());
                continue;
            }
            ++index;
            if (!state.awake)
            {
                continue;
            }
            collectSamplePoints(*ocean, body, samplePoints_);
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
            const double volume   = shapeVolumeOf(description.shape) * fraction * settings_.buoyancyScale;
            const double mass     = bodyMassOf(description);
            if (mass <= 0.0 || volume <= 0.0)
            {
                continue;
            }
            // Gravity magnitude from the world (read once per pass below);
            // buoyancy pushes opposite to it.
            const double buoyancyForce[3] = {0.0, settings_.waterDensity * volume * gravityMagnitude_, 0.0};
            const double centroidPoint[3] = {submergedCentroid.x, submergedCentroid.y, submergedCentroid.z};
            rigid_->applyForceAtPoint(rigid_->self, ocean->world, body, buoyancyForce, centroidPoint);

            // Quadratic water drag on the submerged part: opposite the body
            // velocity, scaled by the submerged fraction and shape size.
            const double speed = math::length(state.linearVelocity);
            if (speed > 1.0e-6)
            {
                const double dragArea  = std::cbrt(shapeVolumeOf(description.shape) * shapeVolumeOf(description.shape));
                const double magnitude = 0.5 * settings_.waterDensity * settings_.linearDragCoefficient * dragArea
                                         * fraction * speed * speed;
                const math::Vec3 direction = state.linearVelocity * (-1.0 / speed);
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
        *outStatistics = statistics_;
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
#undef ENGINE_PHYSICS_WATER_BIND
    }

    ParticipantManifest manifest_;
    WaterCapability     capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    WaterSettings settings_;

    WaterStatistics statistics_{};

    double gravityMagnitude_ = 9.81; // read from the first world at creation of the first ocean

    std::vector<OceanRecord>                     oceans_;
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
