// records.hpp - the physics_particles module's plain-data runtime records:
// slot and emitter records (particle pools with deterministic per-slot hash
// state) plus the shared ParticlesSettings.

#pragma once

#include <engine/physics_particles/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_particles
{
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
    double spawnPosition[3] = {0.0, 0.0, 0.0}; // spawn anchor for the insect wander
    bool   settled      = false; // snow resting on geometry
    double settledTime  = 0.0;   // total settle seconds (leaves' brief settle)
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
    std::uint64_t          generationCounter = 0; // per-slot hash generation
    double                 swayClock  = 0.0; // global emitter clock for sway/flicker phases

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    double windAcceleration[3] = {0.0, 0.0, 0.0};

    std::uint64_t spawnedTotal = 0;
    std::uint64_t expiredTotal = 0;

    // Simulation LOD: the importance-driven fidelity band and the staggered
    // update cadence. The phase derives from the registration ordinal so
    // emitters sharing an interval spread their work across steps.
    double                            importance = 1.0;
    engine::util::SimulationFidelity  lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence   cadence{};

    // Fixed-step budget bookkeeping for simulate().
    engine::util::StepAccumulator accumulator;
};

struct ParticlesSettings
{
    double        fixedRateHz            = 60.0;
    std::uint32_t maximumStepsPerSimulate = 8;
    std::uint32_t maximumEmitters        = 128;
    std::uint32_t maximumParticles       = 1000000;
    std::uint32_t maximumParticlesPerEmitter = 100000;

    // Simulation LOD: emitters below the Full band update on a staggered
    // cadence; the per-band interval says how many fixed steps one due step
    // covers (0 freezes the band's emitters). The whole block comes from the
    // shared fidelity-ladder vocabulary.
    engine::util::SimulationLodSettings lod{};

    // Behavior tuning: the leaves' brief contact hold, and the insect
    // wander controller's radius and steering gain.
    double leafSettleSeconds   = 0.8;
    double insectWanderRadius  = 1.5;
    double insectSteerGain     = 8.0;

    // Settled-snow Coulomb slide coefficient (0 = stop on contact) and the
    // stability clamp on particle speeds (meters per second).
    double snowSlideFriction   = 0.2;
    double maximumVelocity     = 30.0;
};

} // namespace engine::physics_particles
