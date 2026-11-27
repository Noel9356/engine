// physics_particles module implementation.
//
// Deterministic particle pools for rain, snow, dust, fire, hail, ash,
// leaves, pollen, and insects. Every slot's lifetime, jitter, sway phase,
// and size derive from an integer hash of (slot, generation), so identical
// emitters replay identically with no random-number state. Particles
// integrate per fixed step against the world gravity plus a uniform wind
// acceleration, optionally collide with the rigid world through the batched
// sphere-overlap query, and recycle their pool slots FIFO.

#include "records.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_particles/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

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

// Settled slots release after this many consecutive collision-pass misses.
// The push-out above lands exactly at the contact distance, so a static
// surface classifies as a miss by rounding noise alone; a genuine
// moved-away surface misses every step.
inline constexpr std::uint32_t kSettleReleaseMisses = 2u;

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
        readLodSettings(context);
        readBehaviorSettings(context);

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
        registrationOrdinal_ = 0;
        totalSlots_          = 0;
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

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_particles");
    }

    void readBehaviorSettings(ParticipantContext &context)
    {
        const auto readPositive = [this, &context](const char *key, double &target, double maximum)
        {
            if (const auto value = context.configurationFloatingInSection("physics_particles", key);
                value && *value > 0.0 && *value <= maximum && std::isfinite(*value))
                target = *value;
        };
        readPositive("leaf_settle_seconds", settings_.leafSettleSeconds, 60.0);
        readPositive("insect_wander_radius", settings_.insectWanderRadius, 1000.0);
        readPositive("insect_steer_gain", settings_.insectSteerGain, 1000.0);
        // Snow slide friction is a Coulomb coefficient: zero (instant stop)
        // is valid, so its read allows the closed lower bound.
        if (const auto value = context.configurationFloatingInSection("physics_particles", "snow_slide_friction");
            value && *value >= 0.0 && *value <= 4.0 && std::isfinite(*value))
            settings_.snowSlideFriction = *value;
        readPositive("maximum_velocity", settings_.maximumVelocity, 10'000.0);
    }

    static void setError(ParticlesError *output, ParticlesError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    EmitterRecord *findEmitter(EmitterHandle handle)
    {
        const auto iterator = emitterIndexByHandle_.find(handle);
        return iterator != emitterIndexByHandle_.end() ? &emitters_[iterator->second] : nullptr;
    }

    bool validateDescription(const EmitterDescription &description) const
    {
        if (description.maximumParticles == 0
            || !std::isfinite(description.origin[0]) || !std::isfinite(description.origin[1])
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
        const std::uint64_t generation = ++emitter.generationCounter;
        const std::uint64_t baseSeed   = generation * 0x9E3779B97F4A7C15ull + slotIndex;
        const std::uint32_t seed       = static_cast<std::uint32_t>(baseSeed ^ (baseSeed >> 32));
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double fraction = hashFraction(baseSeed + 0x1000ull * (axis + 1)) * 2.0 - 1.0;
            slot.position[axis]   = description.origin[axis] + fraction * description.halfExtents[axis];
        }
        slot.spawnPosition[0] = slot.position[0];
        slot.spawnPosition[1] = slot.position[1];
        slot.spawnPosition[2] = slot.position[2];
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
        slot.swayPhase = hashFraction(baseSeed + 0x5000ull) * math::kTwoPi;
        slot.jitter[0] = hashFraction(baseSeed + 0x6000ull) * 2.0 - 1.0;
        slot.jitter[1] = hashFraction(baseSeed + 0x7000ull) * 2.0 - 1.0;
        slot.jitter[2] = hashFraction(baseSeed + 0x8000ull) * 2.0 - 1.0;
        slot.age       = 0.0;
        slot.seed      = seed;
        slot.settled   = false;
        slot.settleMisses = 0u;
        slot.settledTime = 0.0;
        slot.alive     = true;
        emitter.spawnedTotal += 1;
    }

    void stepEmitter(EmitterRecord &emitter, double stepDeltaSeconds)
    {
        const EmitterDescription &description = emitter.description;

        // Spawn from the emit rate, filling the pool while both budgets
        // allow: the per-emitter pool cap and the module-wide pool budget
        // across all emitters. Once both are full the loop recycles slots
        // FIFO.
        // Emission debt accrues against wall time, but the pool can only
        // accept a bounded number of spawns per step (fresh slots up to the
        // pool budget, then at most one FIFO recycle per existing slot -
        // recycling a slot twice in one step just overwrites the earlier
        // spawn). Debt beyond what this step can possibly spend is dropped
        // up front exactly like the fmod tail below drops it, so an extreme
        // rate (or an LOD interval merge multiplying the delta) cannot turn
        // the recycle loop into an unbounded spin.
        emitter.spawnDebt += description.emitRate * stepDeltaSeconds;
        {
            const std::uint32_t moduleRemaining = settings_.maximumParticles > totalSlots_
                ? static_cast<std::uint32_t>(settings_.maximumParticles - totalSlots_)
                : 0u;
            const std::uint32_t poolBudget
                = std::min(description.maximumParticles, std::min(settings_.maximumParticlesPerEmitter, moduleRemaining));
            const double spendableDebt = static_cast<double>(poolBudget)
                + static_cast<double>(static_cast<std::uint32_t>(emitter.slots.size())) + 1.0;
            if (!std::isfinite(emitter.spawnDebt) || emitter.spawnDebt > spendableDebt)
            {
                emitter.spawnDebt = spendableDebt;
            }
        }
        while (emitter.spawnDebt >= 1.0 && static_cast<std::uint32_t>(emitter.slots.size())
                                              < std::min(description.maximumParticles, settings_.maximumParticlesPerEmitter)
                                              && totalSlots_ < settings_.maximumParticles)
        {
            SlotState slot;
            spawnSlot(emitter, slot, static_cast<std::uint32_t>(emitter.slots.size()));
            emitter.slots.push_back(slot);
            totalSlots_ += 1;
            emitter.spawnDebt -= 1.0;
        }
        // At most one full FIFO generation per step (see the debt note
        // above); the fmod below drops anything the pool could not spend.
        {
            const std::size_t recycleBudget = emitter.slots.size();
            std::size_t recycled = 0;
            while (emitter.spawnDebt >= 1.0 && emitter.spawnCursor < emitter.slots.size()
                && recycled < recycleBudget)
            {
                spawnSlot(emitter, emitter.slots[emitter.spawnCursor], emitter.spawnCursor);
                emitter.spawnCursor = (emitter.spawnCursor + 1) % static_cast<std::uint32_t>(emitter.slots.size());
                emitter.spawnDebt -= 1.0;
                ++recycled;
            }
        }
        if (emitter.spawnDebt >= 1.0)
        {
            emitter.spawnDebt = std::fmod(emitter.spawnDebt, 1.0);
        }

        // Integrate.
        const double dragFactor = std::exp(-description.airDrag * stepDeltaSeconds);
        for (SlotState &slot : emitter.slots)
        {
            if (!slot.alive)
            {
                continue;
            }
            slot.age += stepDeltaSeconds;
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
                // Settled snow slides: a Coulomb friction coefficient
                // (snow_slide_friction, 0.2 by default) decelerates the
                // retained horizontal velocity at friction x |g|, so flakes
                // that landed with drift keep creeping downhill for a moment
                // instead of stopping on the spot.
                if (description.kind == EmitterKind::Snow)
                {
                    const double gravityMagnitude = std::sqrt(emitter.gravity[0] * emitter.gravity[0]
                        + emitter.gravity[1] * emitter.gravity[1]
                        + emitter.gravity[2] * emitter.gravity[2]);
                    const double deceleration = settings_.snowSlideFriction * gravityMagnitude;
                    slot.velocity[1] = 0.0;
                    const double slideSpeed = std::sqrt(slot.velocity[0] * slot.velocity[0]
                        + slot.velocity[2] * slot.velocity[2]);
                    if (slideSpeed <= deceleration * stepDeltaSeconds)
                    {
                        slot.velocity[0] = 0.0;
                        slot.velocity[2] = 0.0;
                    }
                    else
                    {
                        const double retained = (slideSpeed - deceleration * stepDeltaSeconds) / slideSpeed;
                        slot.velocity[0] *= retained;
                        slot.velocity[2] *= retained;
                        slot.position[0] += slot.velocity[0] * stepDeltaSeconds;
                        slot.position[2] += slot.velocity[2] * stepDeltaSeconds;
                    }
                    slot.size = slot.baseSize * (1.0 - 0.5 * slot.age / slot.lifetime);
                }
                else
                {
                    slot.velocity[0] = 0.0;
                    slot.velocity[1] = 0.0;
                    slot.velocity[2] = 0.0;
                }
                if (description.kind != EmitterKind::Snow)
                {
                    // Leaves settle briefly on impact and then expire (they
                    // hold the contact for the configured settle window). The
                    // settled branch only reaches non-snow kinds that settle,
                    // i.e. leaves: the kinds that expire on contact (rain,
                    // hail, fire) never enter it.
                    slot.settledTime += stepDeltaSeconds;
                    if (slot.settledTime >= settings_.leafSettleSeconds)
                    {
                        slot.alive = false;
                        emitter.expiredTotal += 1;
                        continue;
                    }
                }
                continue;
            }

            // Gravity (scaled, may be negative for buoyant fire) plus the
            // wind target blended by the air drag.
            double acceleration[3] = {emitter.gravity[0] * description.gravityScale + emitter.windAcceleration[0],
                emitter.gravity[1] * description.gravityScale + emitter.windAcceleration[1],
                emitter.gravity[2] * description.gravityScale + emitter.windAcceleration[2]};
            // Horizontal sway around the wind direction.
            const double swayPhase = slot.swayPhase + emitter.swayClock * description.swayFrequency * math::kTwoPi;
            acceleration[0] += std::cos(swayPhase) * description.swayAmplitude * slot.jitter[0];
            acceleration[2] += std::sin(swayPhase * 1.3 + slot.jitter[1]) * description.swayAmplitude * slot.jitter[2];

            // Insects hover and wander: gravity, wind, and sway are replaced
            // by a velocity target that is a stateless function of the slot
            // hash and the emitter clock (exact replays), steered toward at
            // a fixed rate, with a soft pull that keeps each insect inside a
            // small 1.5 meter wander radius around its spawn point.
            if (description.kind == EmitterKind::Insects)
            {
                const double wanderPhase = emitter.swayClock * math::kPi; // slow 0.5 Hz wander phase
                double targetVelocity[3] = {
                    0.9 * std::sin(wanderPhase + slot.swayPhase + slot.jitter[0] * 2.4),
                    0.35 * std::sin(wanderPhase * 1.7 + slot.swayPhase * 2.0 + slot.jitter[1] * 3.1),
                    0.9 * std::cos(wanderPhase * 1.3 + slot.swayPhase * 0.5 + slot.jitter[2] * 2.9)};
                const double offsetX = slot.position[0] - slot.spawnPosition[0];
                const double offsetY = slot.position[1] - slot.spawnPosition[1];
                const double offsetZ = slot.position[2] - slot.spawnPosition[2];
                const double wanderDistance = std::sqrt(offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ);
                if (wanderDistance > settings_.insectWanderRadius)
                {
                    const double excess = (wanderDistance - settings_.insectWanderRadius) / wanderDistance;
                    targetVelocity[0] -= offsetX * excess * 2.5;
                    targetVelocity[1] -= offsetY * excess * 2.5;
                    targetVelocity[2] -= offsetZ * excess * 2.5;
                }
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    acceleration[axis] = settings_.insectSteerGain * (targetVelocity[axis] - slot.velocity[axis]);
                }
            }

            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                slot.velocity[axis] = slot.velocity[axis] * dragFactor + acceleration[axis] * stepDeltaSeconds;
            }
            // Stability clamp: a mis-tuned steer gain or wind burst cannot
            // integrate unbounded speeds; the clamp preserves direction. A
            // non-finite velocity (an input magnitude the integrator
            // overflowed) is zeroed instead - rescaling it would multiply
            // Inf by 0 and produce NaN.
            {
                const double speedSquared = slot.velocity[0] * slot.velocity[0]
                    + slot.velocity[1] * slot.velocity[1]
                    + slot.velocity[2] * slot.velocity[2];
                const double maximum = settings_.maximumVelocity;
                if (!std::isfinite(speedSquared))
                {
                    slot.velocity[0] = 0.0;
                    slot.velocity[1] = 0.0;
                    slot.velocity[2] = 0.0;
                }
                else if (speedSquared > maximum * maximum)
                {
                    const double scale = maximum / std::sqrt(speedSquared);
                    slot.velocity[0] *= scale;
                    slot.velocity[1] *= scale;
                    slot.velocity[2] *= scale;
                }
            }
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                slot.position[axis] += slot.velocity[axis] * stepDeltaSeconds;
            }

            // Fire flicker: size oscillates fast; smoke-free burn ends by
            // shrinking over the last third of the lifetime.
            if (description.kind == EmitterKind::Fire)
            {
                const double flickerPhase = emitter.swayClock * 9.0 + slot.swayPhase * 7.0;
                const double flickerScale = 1.0 + description.flicker * 0.4 * std::sin(flickerPhase)
                                            + description.flicker * 0.2 * std::sin(flickerPhase * 2.7 + slot.jitter[0] * 3.0);
                const double burnOut = slot.age > slot.lifetime * 0.66
                                         ? std::max(0.0, 1.0 - (slot.age - slot.lifetime * 0.66) / (slot.lifetime * 0.34))
                                         : 1.0;
                slot.size = slot.baseSize * flickerScale * burnOut;
            }
            else if (description.kind == EmitterKind::Ash)
            {
                // Ash shrinks toward the end of its life (a quiet fade-out
                // over the final 35% of the lifetime, no flicker).
                const double fadeStart = slot.lifetime * 0.65;
                const double fade      = slot.age > fadeStart
                                           ? std::max(0.0, 1.0 - (slot.age - fadeStart) / (slot.lifetime * 0.35))
                                           : 1.0;
                slot.size = slot.baseSize * fade;
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

        // World collision: one batched sphere query per step instead of one
        // capability call per particle. Settled slots take part so a surface
        // that moves away releases them back into free fall.
        if (description.collideWithWorld && rigid_ != nullptr
            && (description.kind != EmitterKind::Dust && description.kind != EmitterKind::Ash
                && description.kind != EmitterKind::Pollen && description.kind != EmitterKind::Insects))
        {
            sphereSlotIndices_.clear();
            sphereQueries_.clear();
            for (std::size_t slotIndex = 0; slotIndex < emitter.slots.size(); ++slotIndex)
            {
                const SlotState &slot = emitter.slots[slotIndex];
                if (!slot.alive)
                {
                    continue;
                }
                engine::physics_rigid::SphereOverlapQuery query;
                query.center[0] = slot.position[0];
                query.center[1] = slot.position[1];
                query.center[2] = slot.position[2];
                query.radius    = description.particleRadius;
                sphereSlotIndices_.push_back(slotIndex);
                sphereQueries_.push_back(query);
            }
            if (!sphereQueries_.empty())
            {
                sphereResults_.resize(sphereQueries_.size());
                const std::uint32_t resolved = rigid_->collideSpheres(rigid_->self, emitter.world,
                    sphereQueries_.data(), static_cast<std::uint32_t>(sphereQueries_.size()), sphereResults_.data());
                // A short count means the rigid world is gone (the provider
                // writes nothing in that case). resize() keeps the previous
                // step's contents, so every entry must be neutralized before
                // the walk - otherwise stale hits settle the wrong slots.
                if (resolved < sphereQueries_.size())
                {
                    for (engine::physics_rigid::SphereOverlapResult &result : sphereResults_)
                    {
                        result = engine::physics_rigid::SphereOverlapResult{};
                    }
                }
                for (std::size_t entry = 0; entry < sphereSlotIndices_.size(); ++entry)
                {
                    SlotState &slot = emitter.slots[sphereSlotIndices_[entry]];
                    const engine::physics_rigid::SphereOverlapResult &result = sphereResults_[entry];
                    if (!result.hit || result.depth <= 0.0)
                    {
                        // A settled slot releases only on a persistent miss:
                        // the push-out lands exactly at the contact distance,
                        // so a static surface can classify as a miss by
                        // rounding noise alone, and re-entering free fall for
                        // that one step would break the documented settle
                        // semantics (the slide rate, the leaf hold window,
                        // and the resting position all assume the settle
                        // holds while the surface stays).
                        if (slot.settled)
                        {
                            if (++slot.settleMisses >= kSettleReleaseMisses)
                            {
                                slot.settled      = false;
                                slot.settleMisses = 0u;
                            }
                        }
                        continue;
                    }
                    slot.settleMisses = 0u;
                    slot.position[0] += result.normal[0] * result.depth;
                    slot.position[1] += result.normal[1] * result.depth;
                    slot.position[2] += result.normal[2] * result.depth;
                    if (description.kind == EmitterKind::Rain || description.kind == EmitterKind::Hail
                        || description.kind == EmitterKind::Fire)
                    {
                        // Precipitation dies on impact; a flame that reaches
                        // geometry burns out instead of settling.
                        slot.alive = false;
                        emitter.expiredTotal += 1;
                    }
                    else
                    {
                        slot.settled = true;
                    }
                }
            }
        }

        emitter.swayClock += stepDeltaSeconds;
    }

    // ---- capability implementations ---------------------------------------

    EmitterHandle createEmitterImpl(
        engine::physics_rigid::WorldHandle world, const EmitterDescription *description, ParticlesError *error)
    {
        setError(error, ParticlesError::None);
        if (rigid_ == nullptr)
        {
            setError(error, ParticlesError::InvalidHandle);
            return invalidEmitter;
        }
        if (description == nullptr)
        {
            setError(error, ParticlesError::InvalidDescription);
            return invalidEmitter;
        }
        if (emitters_.size() >= settings_.maximumEmitters)
        {
            setError(error, ParticlesError::CapacityExceeded);
            return invalidEmitter;
        }
        if (description->maximumParticles == 0u
            || description->maximumParticles > settings_.maximumParticlesPerEmitter)
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
        // Stagger phase from the registration ordinal: emitters sharing an
        // update interval spread their due steps evenly across the cadence.
        emitter.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        // Capturing the gravity doubles as the world-handle validation: a
        // world the rigid module does not know is rejected here instead of
        // silently falling back to the default gravity.
        if (!rigid_->copyWorldGravity(rigid_->self, world, emitter.gravity))
        {
            setError(error, ParticlesError::InvalidHandle);
            return invalidEmitter;
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
        totalSlots_ -= emitters_[iterator->second].slots.size();
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
            totalSlots_ -= emitter->slots.size() - description->maximumParticles;
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
        // A burst larger than the emitter's hard pool cap can only recycle
        // or drop particles (the pool never grows past the cap), so every
        // particle beyond it is pure scan overhead - an unbounded count
        // turned the call into an O(count x pool) stall on the calling
        // thread. Clamp to the cap: the excess drops exactly like a full
        // pool drops spawn attempts.
        const std::uint32_t poolCap
            = std::min(emitter->description.maximumParticles, settings_.maximumParticlesPerEmitter);
        const std::uint32_t boundedCount = std::min(count, poolCap);
        for (std::uint32_t burst = 0; burst < boundedCount; ++burst)
        {
            // Prefer an expired slot; fall back to the FIFO cursor.
            std::size_t slotIndex = emitter->slots.size();
            for (std::size_t candidate = 0; candidate < emitter->slots.size(); ++candidate)
            {
                const std::size_t probe = (emitter->spawnCursor + candidate) % emitter->slots.size();
                if (!emitter->slots[probe].alive)
                {
                    slotIndex = probe;
                    // Continue the next burst particle's search from here so
                    // a burst scans the pool once overall instead of once per
                    // particle.
                    emitter->spawnCursor = (probe + 1) % static_cast<std::uint32_t>(emitter->slots.size());
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
                // Growing the pool needs both budgets: the per-emitter pool
                // cap and the module-wide pool budget. When either is full
                // the burst particle drops (its spawn attempt still advanced
                // the deterministic generation hash, so replays stay exact).
                const bool poolCapacityAvailable
                    = emitter->slots.size()
                      < std::min(emitter->description.maximumParticles, settings_.maximumParticlesPerEmitter);
                if (poolCapacityAvailable && totalSlots_ < settings_.maximumParticles)
                {
                    emitter->slots.push_back(SlotState{});
                    totalSlots_ += 1;
                }
                else
                {
                    continue;
                }
            }
            SlotState &slot = emitter->slots[slotIndex];
            spawnSlot(*emitter, slot, static_cast<std::uint32_t>(slotIndex));
            const std::uint64_t burstSeed = emitter->generationCounter;
            const double azimuth   = hashFraction(burstSeed + 0xA000ull) * math::kTwoPi;
            const double elevation = (hashFraction(burstSeed + 0xB000ull) * 2.0 - 1.0);
            const double horizontal = std::sqrt(std::max(0.0, 1.0 - elevation * elevation));
            slot.position[0] = position[0];
            slot.position[1] = position[1];
            slot.position[2] = position[2];
            slot.spawnPosition[0] = position[0]; // bursts re-anchor the insect wander
            slot.spawnPosition[1] = position[1];
            slot.spawnPosition[2] = position[2];
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
        // Simulation LOD: resolve the fidelity band from the importance
        // score, refresh the staggered cadence, and let one due step cover
        // the whole interval of wall time. Cached emitters freeze. Without
        // lod_enabled every emitter runs Full every step.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(emitter->lodBand,
            emitter->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            emitter->importance,
            &statistics_.lodSkippedSteps);
        if (lodInterval == 0u)
        {
            return true;
        }
        const double stepDelta = fixedDeltaSeconds * static_cast<double>(lodInterval);
        stepEmitter(*emitter, stepDelta);
        // Finite guard: a diverging particle retires immediately instead of
        // poisoning its pool; the step still reports the failure.
        bool diverged = false;
        for (SlotState &slot : emitter->slots)
        {
            if (!slot.alive)
            {
                continue;
            }
            const bool finite = std::isfinite(slot.position[0]) && std::isfinite(slot.position[1])
                && std::isfinite(slot.position[2]) && std::isfinite(slot.velocity[0]) && std::isfinite(slot.velocity[1])
                && std::isfinite(slot.velocity[2]);
            if (!finite)
            {
                slot.alive = false;
                emitter->expiredTotal += 1;
                diverged = true;
            }
        }
        return !diverged;
    }

    std::uint32_t simulateImpl(EmitterHandle handle, double elapsedSeconds)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return emitter->accumulator.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, handle, fixedDelta]() { return stepImpl(handle, fixedDelta); });
    }

    bool setEmitterImportanceImpl(EmitterHandle handle, double importance)
    {
        EmitterRecord *emitter = findEmitter(handle);
        if (emitter == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        emitter->importance = engine::util::simulationImportanceClamped(importance);
        return true;
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
        ENGINE_PHYSICS_PARTICLES_BIND(setEmitterImportance, setEmitterImportanceImpl);
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
    std::uint64_t                                  registrationOrdinal_ = 0;

    // Module-wide pool budget bookkeeping: the total number of allocated
    // slots across all emitters, bounded by settings_.maximumParticles.
    std::size_t totalSlots_ = 0;

    // Reused scratch for the batched sphere-overlap queries.
    std::vector<std::size_t>                             sphereSlotIndices_;
    std::vector<engine::physics_rigid::SphereOverlapQuery>  sphereQueries_;
    std::vector<engine::physics_rigid::SphereOverlapResult> sphereResults_;
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
