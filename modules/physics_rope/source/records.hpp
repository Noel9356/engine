// records.hpp - the physics_rope module's plain-data runtime records: ropes
// (a chain of nodes with stretch and bending constraints plus end
// attachments) and the shared RopeSettings.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rope/capabilities.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_rope
{

// Attachment state of one node: pinned to a world position, or following a
// rigid body's local anchor. Attached nodes carry no mass.
enum class NodeAttachment : std::uint8_t
{
    Free = 0,
    Pinned,
    Body
};

struct RopeRecord
{
    RopeHandle handle = invalidRope;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    std::uint32_t segments = 0;

    std::vector<double>       positions;         // x,y,z per node
    std::vector<double>       velocities;        // x,y,z per node
    std::vector<double>       previousPositions; // x,y,z per node
    std::vector<double>       inverseMasses;     // zero for attached nodes
    std::vector<NodeAttachment> attachments;
    std::vector<double>       pinPositions;      // x,y,z per node attachment target
    std::vector<engine::physics_rigid::BodyHandle> attachedBodies;
    std::vector<double>       attachedLocalAnchors; // x,y,z per node

    struct StretchConstraint
    {
        std::uint32_t nodeA = 0;
        std::uint32_t nodeB = 0;
        double        restLength = 0.0;
        double        compliance = 0.0;
    };
    struct BendConstraint
    {
        std::uint32_t nodeA = 0;
        std::uint32_t nodeB = 0;
        double        restLength = 0.0;
        double        compliance = 0.0;
    };

    std::vector<StretchConstraint> stretchConstraints;
    std::vector<BendConstraint>    bendConstraints;

    // Strain-breaking state: one flag per segment (index matches the
    // stretch-constraint index). A broken segment stops solving and releases
    // the bend rows that span it; the count is cumulative.
    std::vector<std::uint8_t> brokenSegments;
    std::uint32_t             brokenSegmentCount = 0;

    double nodeMass = 0.0;
    double nodeRadius = 0.0;
    double damping = 0.0;
    double friction = 0.6;
    double breakStrain = 0.0; // elongation ratio tolerance beyond rest; 0 disables breaking
    bool   collideWithWorld = true;

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    double windAcceleration[3] = {0.0, 0.0, 0.0};

    // Tension readout bookkeeping: the accumulator sums each segment's
    // |lambda| over the running step (reset at step start); when the step
    // completes, the values convert to force units (divided by the fixed
    // step delta squared) into lastTension and maximumTension tracks the
    // largest published value.
    std::vector<double> tensionAccumulator; // |lambda| per segment, current step
    std::vector<double> lastTension;        // force units per segment, last completed step
    double              maximumTension = 0.0;

    std::uint64_t stepsSimulated   = 0;
    std::uint64_t substepsExecuted = 0;
    std::uint32_t lastContactCount = 0;
    double        lastStepMilliseconds = 0.0;
    std::uint64_t lodSkippedSteps  = 0;

    // Fixed-step budget bookkeeping for simulate().
    engine::util::StepAccumulator accumulator;

    // Simulation LOD: importance-driven fidelity band and staggered cadence
    // (the phase derives from the registration ordinal).
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};

    // Per-step contact scratch (one entry per node): 0 none, 1 contact, plus
    // the push-out normal. Lives on the record so ropes of different sizes
    // reuse their own warmed-up buffers.
    std::vector<std::uint8_t> contactFlags;
    std::vector<double>       contactNormals;

    // Pre-step copies of the node state, restored when a step diverges into
    // non-finite values (reused buffers; capture is one copy per step).
    std::vector<double> preStepPositions;
    std::vector<double> preStepVelocities;
};

struct RopeSettings
{
    double        fixedRateHz          = 60.0;
    std::uint32_t maximumStepsPerSimulate = 8;
    std::uint32_t substepsPerStep     = 8;
    std::uint32_t maximumRopes        = 256;
    std::uint32_t maximumNodes        = 200000;
    std::uint32_t maximumNodesPerRope = 4096;

    // Velocity clamp applied to the derived per-substep node velocities.
    double maximumVelocity = 30.0;

    // Simulation LOD: fidelity bands keep the fixed step size and reduce the
    // internal substep count instead (Full = substepsPerStep, Simplified =
    // half, Procedural = quarter, Animation = one); the interval table says
    // how many fixed steps each band updates on (0 freezes that band, which
    // is the Cached default).
    bool                              lodEnabled   = false;
    engine::util::SimulationFidelityThresholds lodThresholds{};
    std::uint32_t                     lodIntervals[engine::util::kSimulationFidelityBandCount] = {0u, 8u, 4u, 2u, 1u};
};

} // namespace engine::physics_rope
