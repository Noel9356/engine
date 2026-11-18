// physics_particles module implementation.
//
// Deterministic particle pools for rain, snow, dust, and fire. Every slot's
// lifetime, jitter, sway phase, and size derive from an integer hash of
// (slot, generation), so identical emitters replay identically with no
// random-number state. Particles integrate per fixed step against the world
// gravity plus a uniform wind acceleration, optionally collide with the
// rigid world through collideSphere, and recycle their slots FIFO.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_particles/capabilities.hpp>
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

namespace engine::physics_particles
{
namespace
{

namespace math = engine::physics_rigid::math;

// Deterministic hash noise in [0, 1) from an integer seed.
double hashFraction(std::uint64_t seed)
{
    seed ^= seed >> 33;
    seed *= 0xFF51AFD7ED558CCDull;
    seed ^= seed >> 33;
    seed *= 0xC4CEB9FE1A85EC53ull;
    seed ^= seed >> 33;
    return static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53);
}

struct SlotState
{
    double position[3]  = {0.0, 0.0, 0.0};
    double velocity[3]  = {0.0, 0.0, 0.0};
    double age          = 0.0;
    double lifetime     = 1.0;
    double size         = 0.1;
    double baseSize     = 0.1;
    double swayPhase    = 0.0;
    double jitter[3]    = {0.0, 0.0, 0.0};
    bool   settled      = false; // snow resting on geometry
    std::uint32_t seed  = 0;
    bool   alive        = false;
};

struct EmitterRecord
{
    EmitterHandle handle = invalidEmitter;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    EmitterDescription description{};

    std::vector<SlotState> slots;
    std::uint32_t          spawnCursor = 0; // FIFO recycling cursor
    double                 spawnDebt   = 0.0; // fractional particle accumulation
    std::uint64_t          generationCounter_ = 0; // per-slot hash generation
    double                 swayClock_  = 0.0; // global emitter clock for sway/flicker phases

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    double windAcceleration[3] = {0.0, 0.0, 0.0};

    std::uint64_t spawnedTotal = 0;
    std::uint64_t expiredTotal = 0;

    double simulatedTimeBudget = 0.0;
};

struct ParticlesSettings
{
    double        fixedRateHz            = 60.0;
    std::uint32_t maximumStepsPerSimulate = 8;
    std::uint32_t maximumEmitters        = 128;
    std::uint32_t maximumParticles       = 1000000;
    std::uint32_t maximumParticlesPerEmitter = 100000;
};

class ParticlesModule final : public ModuleInterface
{
public:
    ParticlesModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_particles", "maximum_emitters");
            value && *value > 0 && *value <= 4096)
            settings_.maximumEmitters = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_particles", "maximum_particles");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumParticles = static_cast<std::uint32_t>(*value);
        if (const auto value
            = context.configurationIntegerInSection("physics_particles", "maximum_particles_per_emitter");
            value && *value > 0 && *value <= 1'000'000)
            settings_.maximumParticlesPerEmitter = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_particles", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_particles", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;

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
        emitters_.clear();
        emitterIndexByHandle_.clear();
        nextEmitterHandle_ = 1;
        statistics_        = ParticlesStatistics{};
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ParticlesCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_particles",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_particles"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    static void setError(ParticlesError *output, ParticlesError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
    }

    EmitterRecord *findEmitter(EmitterHandle handle)
    {
        const auto iterator = emitterIndexByHandle_.find(handle);
        return iterator != emitterIndexByHandle_.end() ? &emitters_[iterator->second] : nullptr;
    }

    bool validateDescription(const EmitterDescription &description) const
    {
        if (!std::isfinite(description.origin[0]) || !std::isfinite(description.origin[1])
            || !std::isfinite(description.origin[2]) || !std::isfinite(description.halfExtents[0])
            || !std::isfinite(description.halfExtents[1]) || !std::isfinite(description.halfExtents[2])
            || description.halfExtents[0] < 0.0 || description.halfExtents[1] < 0.0
            || description.halfExtents[2] < 0.0 || !std::isfinite(description.emitRate)
            || description.emitRate < 0.0 || !std::isfinite(description.minimumLifetime)
            || !std::isfinite(description.maximumLifetime) || description.minimumLifetime <= 0.0
            || description.maximumLifetime < description.minimumLifetime
            || !std::isfinite(description.initialVelocity[0]) || !std::isfinite(description.initialVelocity[1])
            || !std::isfinite(description.initialVelocity[2]) || !std::isfinite(description.velocityJitter)
            || !std::isfinite(description.gravityScale) || !std::isfinite(description.airDrag)
            || description.airDrag < 0.0 || !std::isfinite(description.swayAmplitude)
            || !std::isfinite(description.swayFrequency) || !std::isfinite(description.particleRadius)
            || description.particleRadius < 0.0 || !std::isfinite(description.minimumSize)
            || !std::isfinite(description.maximumSize) || description.minimumSize <= 0.0
            || description.maximumSize < description.minimumSize || !std::isfinite(description.flicker)
            || description.flicker < 0.0 || description.flicker > 2.0 || !std::isfinite(description.groundHeight))
        {
            return false;
        }
        return true;
    }

    void spawnSlot(EmitterRecord &emitter, SlotState &slot, std::uint32_t slotIndex)
    {
        const EmitterDescription &description = emitter.description;
        const std::uint64_t generation = ++emitter.generationCounter_;
        const std::uint64_t baseSeed   = generation * 0x9E3779B97F4A7C15ull + slotIndex;
        const std::uint32_t seed       = static_cast<std::uint32_t>(baseSeed ^ (baseSeed >> 32));

        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double fraction = hashFraction(baseSeed + 0x1000ull * (axis + 1)) * 2.0 - 1.0;
            slot.position[axis]   = description.origin[axis] + fraction * description.halfExtents[axis];
        }
        const double lifetimeFraction = hashFraction(baseSeed + 0x2000ull);
        slot.lifetime = description.minimumLifetime
                        + (description.maximumLifetime - description.minimumLifetime) * lifetimeFraction;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double jitter = 1.0 + (hashFraction(baseSeed + 0x3000ull * (axis + 1)) * 2.0 - 1.0)
                                        * description.velocityJitter;
            slot.velocity[axis] = description.initialVelocity[axis] * jitter;
        }
        const double sizeFraction = hashFraction(baseSeed + 0x4000ull);
        slot.baseSize = description.minimumSize + (description.maximumSize - description.minimumSize) * sizeFraction;
        slot.size     = slot.baseSize;
        slot.swayPhase = hashFraction(baseSeed + 0x5000ull) * 6.283185307179586;
        slot.jitter[0] = hashFraction(baseSeed + 0x6000ull) * 2.0 - 1.0;
        slot.jitter[1] = hashFraction(baseSeed + 0x7000ull) * 2.0 - 1.0;
        slot.jitter[2] = hashFraction(baseSeed + 0x8000ull) * 2.0 - 1.0;
        slot.age       = 0.0;
        slot.seed      = seed;
        slot.settled   = false;
        slot.alive     = true;
        emitter.spawnedTotal += 1;
    }

    void stepEmitter(EmitterRecord &emitter, double fixedDeltaSeconds)
    {
        const EmitterDescription &description = emitter.description;

        // Spawn from the emit rate plus recycle expired slots FIFO.
        emitter.spawnDebt += description.emitRate * fixedDeltaSeconds;
        while (emitter.spawnDebt >= 1.0 && static_cast<std::uint32_t>(emitter.slots.size())
                                              < std::min(description.maximumParticles, settings_.maximumParticlesPerEmitter))
        {
            SlotState slot;
            spawnSlot(emitter, slot, static_cast<std::uint32_t>(emitter.slots.size()));
            emitter.slots.push_back(slot);
            emitter.spawnDebt -= 1.0;
        }
        while (emitter.spawnDebt >= 1.0 && emitter.spawnCursor < emitter.slots.size())
        {
            spawnSlot(emitter, emitter.slots[emitter.spawnCursor], emitter.spawnCursor);
            emitter.spawnCursor = (emitter.spawnCursor + 1) % static_cast<std::uint32_t>(emitter.slots.size());
            emitter.spawnDebt -= 1.0;
        }
        if (emitter.spawnDebt >= 1.0)
        {
            emitter.spawnDebt = std::fmod(emitter.spawnDebt, 1.0);
        }

        // Integrate.
        const double dragFactor = std::exp(-description.airDrag * fixedDeltaSeconds);
        for (SlotState &slot : emitter.slots)
        {
            if (!slot.alive)
            {
                continue;
            }
            slot.age += fixedDeltaSeconds;
            if (slot.age >= slot.lifetime)
            {
                slot.alive = false;
                emitter.expiredTotal += 1;
                continue;
            }

            if (slot.settled)
            {
                // Snow resting on geometry: keep it until its lifetime ends
                // or the surface moves away (checked by the collision pass).
                slot.velocity[0] = 0.0;
                slot.velocity[1] = 0.0;
                slot.velocity[2] = 0.0;
                slot.size        = slot.baseSize * (1.0 - 0.5 * slot.age / slot.lifetime);
                continue;
            }

            // Gravity (scaled, may be negative for buoyant fire) plus the
            // wind target blended by the air drag.
            double acceleration[3] = {emitter.gravity[0] * description.gravityScale + emitter.windAcceleration[0],
                emitter.gravity[1] * description.gravityScale + emitter.windAcceleration[1],
                emitter.gravity[2] * description.gravityScale + emitter.windAcceleration[2]};
            // Horizontal sway around the wind direction.
            const double swayPhase = slot.swayPhase + emitter.swayClock_ * description.swayFrequency * 6.283185307179586;
            acceleration[0] += std::cos(swayPhase) * description.swayAmplitude * slot.jitter[0];
            acceleration[2] += std::sin(swayPhase * 1.3 + slot.jitter[1]) * description.swayAmplitude * slot.jitter[2];

            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                slot.velocity[axis] = slot.velocity[axis] * dragFactor + acceleration[axis] * fixedDeltaSeconds;
                slot.position[axis] += slot.velocity[axis] * fixedDeltaSeconds;
            }

            // Fire flicker: size oscillates fast; smoke-free burn ends by
            // shrinking over the last third of the lifetime.
            if (description.kind == EmitterKind::Fire)
            {
                const double flickerPhase = emitter.swayClock_ * 9.0 + slot.swayPhase * 7.0;
                const double flickerScale = 1.0 + description.flicker * 0.4 * std::sin(flickerPhase)
                                            + description.flicker * 0.2 * std::sin(flickerPhase * 2.7 + slot.jitter[0] * 3.0);
                const double burnOut = slot.age > slot.lifetime * 0.66
                                         ? std::max(0.0, 1.0 - (slot.age - slot.lifetime * 0.66) / (slot.lifetime * 0.34))
                                         : 1.0;
                slot.size = slot.baseSize * flickerScale * burnOut;
            }
            else
            {
                slot.size = slot.baseSize;
            }

            // Ground-plane fallback kill (rain ending on the floor).
            if (description.useGroundPlane && slot.position[1] <= description.groundHeight)
            {
                slot.alive = false;
                emitter.expiredTotal += 1;
            }
        }

        // World collision.
        if (description.collideWithWorld && rigid_ != nullptr)
        {
            for (SlotState &slot : emitter.slots)
            {
                if (!slot.alive || slot.settled || description.kind == EmitterKind::Dust)
                {
                    continue;
                }
                double                            normal[3] = {0.0, 0.0, 0.0};
                double                            depth     = 0.0;
                engine::physics_rigid::BodyHandle body      = engine::physics_rigid::invalidBody;
                if (!rigid_->collideSphere(rigid_->self, emitter.world, slot.position, description.particleRadius,
                        normal, &depth, &body)
                    || depth <= 0.0)
                {
                    if (slot.settled)
                    {
                        slot.settled = false;
                    }
                    continue;
                }
                slot.position[0] += normal[0] * depth;
                slot.position[1] += normal[1] * depth;
                slot.position[2] += normal[2] * depth;
                if (description.kind == EmitterKind::Rain)
                {
                    slot.alive = false;
                    emitter.expiredTotal += 1;
                }
                else
                {
                    slot.settled = true;
                }
            }
        }

        emitter.swayClock_ += fixedDeltaSeconds;
    }

    // ---- capability implementations ---------------------------------------

    EmitterHandle createEmitterImpl(
        engine::physics_rigid::WorldHandle world, const EmitterDescription *description, ParticlesError *error)
    {
        setError(error, ParticlesError::None);
        if (rigid_ == nullptr || description == nullptr)
        {
            setError(error, ParticlesError::InvalidDescription);
            return invalidEmitter;
        }
        if (emitters_.size() >= settings_.maximumEmitters)
        {
            setError(error, ParticlesError::CapacityExceeded);
            return invalidEmitter;
        }
        if (description->maximumParticles > settings_.maximumParticlesPerEmitter)
        {
            setError(error, ParticlesError::InvalidDescription);
            return invalidEmitter;
        }
        if (!validateDescription(*description))
        {
            setError(error, ParticlesError::InvalidDescription);
            return invalidEmitter;
        }
        if (nextEmitterHandle_ == invalidEmitter)
        {
            setError(error, ParticlesError::CapacityExceeded);
            return invalidEmitter;
        }
        EmitterRecord emitter;
        emitter.handle = nextEmitterHandle_;
        nextEmitterHandle_ = nextEmitterHandle_ == std::numeric_limits<EmitterHandle>::max()
                               ? invalidEmitter
                               : nextEmitterHandle_ + 1;
        emitter.world       = world;
        emitter.description = *description;
        if (!rigid_->copyWorldGravity(rigid_->self, world, emitter.gravity))
        {
            emitter.gravity[0] = 0.0;
            emitter.gravity[1] = -9.81;
            emitter.gravity[2] = 0.0;
        }
        emitter.slots.reserve(description->maximumParticles);
        emitters_.push_back(std::move(emitter));
        emitterIndexByHandle_.emplace(emitters_.back().handle, emitters_.size() - 1);
        statistics_.emitterCount = static_cast<std::uint32_t>(emitters_.size());
        return emitters_.back().handle;
    }

    void destroyEmitterImpl(EmitterHandle handle)
    {
        const auto iterator = emitterIndexByHandle_.find(handle);
        if (iterator == emitterIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        emitterIndexByHandle_.erase(iterator);
        if (index + 1 != emitters_.size())
        {
            emitters_[index]                                    = emitters_.back();
            emitterIndexByHandle_[emitters_[index].handle]      = index;
        }
        emitters_.pop_back();
        statistics_.emitterCount = static_cast<std::uint32_t>(emitters_.size());
    }

    bool setEmitterDescriptionImpl(EmitterHandle handle, const EmitterDescription *description)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || description == nullptr)
        {
            return false;
        }
        if (description->maximumParticles > settings_.maximumParticlesPerEmitter || !validateDescription(*description))
        {
            return false;
        }
        emitter->description = *description;
        if (emitter->slots.size() > description->maximumParticles)
        {
            emitter->slots.resize(description->maximumParticles);
            emitter->spawnCursor = 0;
        }
        return true;
    }

    bool setWindAccelerationImpl(EmitterHandle handle, const double wind[3])
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || wind == nullptr)
        {
            return false;
        }
        if (!std::isfinite(wind[0]) || !std::isfinite(wind[1]) || !std::isfinite(wind[2]))
        {
            return false;
        }
        emitter->windAcceleration[0] = wind[0];
        emitter->windAcceleration[1] = wind[1];
        emitter->windAcceleration[2] = wind[2];
        return true;
    }

    bool burstImpl(EmitterHandle handle, const double position[3], std::uint32_t count, double speed)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || position == nullptr || !std::isfinite(speed) || speed < 0.0)
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        for (std::uint32_t burst = 0; burst < count; ++burst)
        {
            // Prefer an expired slot; fall back to the FIFO cursor.
            std::size_t slotIndex = emitter->slots.size();
            for (std::size_t candidate = 0; candidate < emitter->slots.size(); ++candidate)
            {
                const std::size_t probe = (emitter->spawnCursor + candidate) % emitter->slots.size();
                if (!emitter->slots[probe].alive)
                {
                    slotIndex = probe;
                    break;
                }
            }
            if (slotIndex == emitter->slots.size()
                && emitter->slots.size() >= std::min(emitter->description.maximumParticles, settings_.maximumParticlesPerEmitter))
            {
                slotIndex = emitter->spawnCursor;
                emitter->spawnCursor = (emitter->spawnCursor + 1) % static_cast<std::uint32_t>(emitter->slots.size());
            }
            if (slotIndex == emitter->slots.size())
            {
                SlotState slot;
                emitter->slots.push_back(slot);
            }
            SlotState &slot = emitter->slots[slotIndex];
            spawnSlot(*emitter, slot, static_cast<std::uint32_t>(slotIndex));
            const std::uint64_t burstSeed = emitter->generationCounter_;
            const double azimuth   = hashFraction(burstSeed + 0xA000ull) * 6.283185307179586;
            const double elevation = (hashFraction(burstSeed + 0xB000ull) * 2.0 - 1.0);
            const double horizontal = std::sqrt(std::max(0.0, 1.0 - elevation * elevation));
            slot.position[0] = position[0];
            slot.position[1] = position[1];
            slot.position[2] = position[2];
            slot.velocity[0] = (std::cos(azimuth) * horizontal) * speed;
            slot.velocity[1] = elevation * speed;
            slot.velocity[2] = (std::sin(azimuth) * horizontal) * speed;
        }
        return true;
    }

    bool copyParticleStatesImpl(EmitterHandle handle, ParticleState *outStates, std::uint32_t capacity,
        std::uint32_t *outFilled, std::uint32_t *outRequired)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || outRequired == nullptr)
        {
            return false;
        }
        const std::uint32_t total = static_cast<std::uint32_t>(emitter->slots.size());
        *outRequired = total;
        if (outFilled != nullptr)
        {
            *outFilled = 0;
        }
        if (outStates == nullptr)
        {
            return false;
        }
        std::uint32_t filled = 0;
        for (std::uint32_t index = 0; index < total && filled < capacity; ++index)
        {
            const SlotState &slot = emitter->slots[index];
            ParticleState &target = outStates[filled++];
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                target.position[axis] = slot.position[axis];
                target.velocity[axis] = slot.velocity[axis];
            }
            target.age      = slot.age;
            target.lifetime = slot.lifetime;
            target.size     = slot.size;
            target.seed     = slot.seed;
            target.alive    = slot.alive;
        }
        if (outFilled != nullptr)
        {
            *outFilled = filled;
        }
        return true;
    }

    bool stepImpl(EmitterHandle handle, double fixedDeltaSeconds)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0
            || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        stepEmitter(*emitter, fixedDeltaSeconds);
        // Finite guard: a diverging emitter stops reporting alive particles.
        for (const SlotState &slot : emitter->slots)
        {
            if (!std::isfinite(slot.position[0]) || !std::isfinite(slot.position[1])
                || !std::isfinite(slot.position[2]))
            {
                return false;
            }
        }
        return true;
    }

    std::uint32_t simulateImpl(EmitterHandle handle, double elapsedSeconds)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        emitter->simulatedTimeBudget += elapsedSeconds;
        std::uint32_t executed = 0;
        while (emitter->simulatedTimeBudget >= fixedDelta && executed < settings_.maximumStepsPerSimulate)
        {
            emitter->simulatedTimeBudget -= fixedDelta;
            if (!stepImpl(handle, fixedDelta))
            {
                break;
            }
            ++executed;
        }
        if (executed == settings_.maximumStepsPerSimulate)
        {
            emitter->simulatedTimeBudget = 0.0;
        }
        return executed;
    }

    bool particlesStatisticsImpl(ParticlesStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        std::uint32_t alive = 0;
        std::uint64_t spawned = 0;
        std::uint64_t expired = 0;
        for (const EmitterRecord &emitter : emitters_)
        {
            for (const SlotState &slot : emitter.slots)
            {
                if (slot.alive)
                {
                    ++alive;
                }
            }
            spawned += emitter.spawnedTotal;
            expired += emitter.expiredTotal;
        }
        statistics_.aliveParticles = alive;
        statistics_.spawnedTotal   = spawned;
        statistics_.expiredTotal   = expired;
        *outStatistics = statistics_;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsParticlesCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_PARTICLES_BIND(field, method)                                                                   \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<ParticlesModule *>(self)->method(args...); }
        ENGINE_PHYSICS_PARTICLES_BIND(createEmitter, createEmitterImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(destroyEmitter, destroyEmitterImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(setEmitterDescription, setEmitterDescriptionImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(setWindAcceleration, setWindAccelerationImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(burst, burstImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(copyParticleStates, copyParticleStatesImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(step, stepImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_PARTICLES_BIND(particlesStatistics, particlesStatisticsImpl);
#undef ENGINE_PHYSICS_PARTICLES_BIND
    }

    ParticipantManifest manifest_;
    ParticlesCapability capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    ParticlesSettings settings_;

    ParticlesStatistics statistics_{};

    std::vector<EmitterRecord>                     emitters_;
    std::unordered_map<EmitterHandle, std::size_t> emitterIndexByHandle_;
    EmitterHandle                                  nextEmitterHandle_ = 1;
};

} // namespace
} // namespace engine::physics_particles

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_particles capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_particles"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_particles",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_particles::ParticlesModule(); },
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
