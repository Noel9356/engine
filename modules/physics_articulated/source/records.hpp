// records.hpp - the articulated module's figure records. A figure mirrors
// its links into a physics_rigid world: one rigid body per link, one rigid
// joint per link-parent connection, plus drive configuration and the
// assembly-frame reference rotations the drives and angle queries measure
// against. The module owns only this bookkeeping; simulation happens in the
// rigid core.

#pragma once

#include <engine/physics_articulated/capabilities.hpp>
#include <engine/util/simulation_lod.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_articulated
{

// One link of a live figure.
struct LinkRecord
{
    engine::physics_rigid::BodyHandle body = engine::physics_rigid::invalidBody;

    std::int32_t parentIndex = -1;

    ArticulatedJointKind jointToParent = ArticulatedJointKind::Hinge;

    // Rigid joint connecting this link to its parent (invalidJoint for the
    // root and for parentless dynamic roots).
    engine::physics_rigid::JointHandle joint = engine::physics_rigid::invalidJoint;

    // Hinge axis in the parent's local frame (hinge joints only).
    engine::physics_rigid::Vec3 hingeAxisLocalParent = {0.0, 1.0, 0.0};

    // Relative rotation (parent^-1 * link) captured at the assembly pose;
    // drives and angle queries report offsets from it.
    engine::physics_rigid::Quaternion referenceRelativeRotation = {};

    JointDrive drive;
};

// One live figure.
struct FigureRecord
{
    FigureHandle handle = invalidFigure;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    std::vector<LinkRecord> links;

    bool selfCollision = false;

    // Collision category bit reserved from the module pool (1..31); mask
    // excludes the figure's own bit when selfCollision is false.
    std::uint32_t categoryBit = 0;

    bool sleepingEnabled = true;

    // Simulation LOD: the consumer-written importance score resolves the
    // fidelity band (with demotion hysteresis, the shared ladder every
    // simulation module uses); the band gates the drive refresh cadence -
    // a skipped refresh keeps the rigid motors on their last target, so
    // the figure keeps holding its pose at a fraction of the refresh cost.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

// Module-wide budgets and clamps read from the [physics_articulated] section.
struct ArticulatedSettings
{
    std::uint32_t maximumFigures        = 256;
    std::uint32_t maximumLinksPerFigure = 128;

    double maximumDriveVelocity = 100.0;
    double maximumDriveTorque   = 10000.0;

    // Simulation LOD: the shared ladder plus the per-band drive-refresh
    // intervals (0 freezes that band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_articulated
