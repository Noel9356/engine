# physics_articulated module

physics_articulated provides articulated-body figures: trees of rigid links connected by hinge or ball-socket joints with angle limits, plus per-joint PD drives that hold pose targets. It is the articulated layer over the physics_rigid module's solver: every link is a rigid body in a caller-created physics_rigid world and every connection is a rigid joint, so ray casts, sphere queries, forces, grabbing, and stepping all run through the rigid capability while this module supplies the figure model, joint measurement, drive control, and per-figure collision isolation.

## Capability

The module provides the `physics_articulated` capability (ABI version constant `physicsArticulatedCapabilityAbi = 1`, see `include/engine/physics_articulated/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the capability permission from the manifest's `requires` list. Figure handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidFigure`) means "none". Angles are signed radians measured against the figure's assembly pose (the transforms the links carried at `createFigure`).

| Group | Functions | Purpose |
| --- | --- | --- |
| Figure lifetime | `createFigure(world, description, outError)`, `destroyFigure(figure)` | Creates one rigid body per link and one rigid joint per link-parent connection in the given rigid world; destroys the whole set. |
| Inspection | `figureInfo`, `figureStatistics`, `copyLinkBodyHandles`, `linkBodyHandle` | Link and joint counts, active drive count, the reserved collision category bit, and the rigid body handle of each link (for rendering, grabbing, and force application through the rigid capability). |
| Drives | `setJointDrive(figure, linkIndex, drive)`, `copyJointDrive`, `updateDrives(figure, fixedDeltaSeconds)` | Per-joint PD control: `updateDrives` measures each driven joint's angle relative to the assembly frame, derives a velocity target from the stiffness and damping gains (clamped to the drive's velocity limit), and pushes it into the rigid hinge motor with the drive's torque budget. Call once per fixed step. |
| Measurement | `copyJointState(figure, linkIndex, outState)` | Signed joint angle relative to the assembly frame: hinge twist for hinge joints, swing and twist for ball-socket joints. |

**Error convention:** creation functions fill the `ArticulatedError` out-parameter (`InvalidDescription`, `CapacityExceeded`, `Unsupported`); the remaining calls return `false` for unknown figures, out-of-range link indices, or inapplicable requests (for example a drive on a parentless root link). A figure whose bodies or joints fail mid-creation is rolled back completely: the world never keeps a partial figure.

## Figure model

A `FigureDescription` lists links in assembly order: index 0 is the root, and every other link's `parentIndex` must reference an earlier link, which makes each figure a tree. The root link's `relativeTransform` places it in world space; a child link's places it in its parent's frame. `rootDynamic = false` pins the root as a static rigid body (pendulums, anchored ragdolls); the default creates a dynamic root. Each link carries its own shape, mass source (density over explicit mass, mirroring `BodyDescription`), and material.

The link-parent connection is a `Hinge` (rotation about the parent-frame axis only, with optional fold limits relative to the assembly pose) or a `BallSocket` (coincident anchors with optional cone and twist limits). Joint anchors are expressed in each link's local frame and must coincide in world space at the assembly pose; the rigid module normalizes the hinge axis and validates the limit ranges at joint creation.

**Collision isolation.** By default links of one figure never collide with each other: each figure reserves one collision category bit from a pool of thirty-one and its links' masks exclude their own bit, while everything else in the rigid world collides with the links normally. `selfCollision = true` skips the reservation so figure-internal links collide with each other (useful for loose chains); the rigid category/mask rules decide the behavior. When the category pool is exhausted, `createFigure` reports `CapacityExceeded`.

**Drives.** A drive is PD control over the joint's twist angle: `updateDrives` measures the current twist about the hinge axis against the assembly reference, computes `stiffness x (target - angle) - damping x angular speed`, clamps the result to the drive's velocity limit, and applies it as the rigid hinge motor's target velocity with the drive's torque budget. Ball-socket drives power the twist component; swing remains limit-controlled. Because drives only retune rigid motors, they are frame-rate independent: the control law runs at the fixed step the consumer calls `updateDrives` with, and the motor executes inside the rigid solver's iterations.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_articulated.cpp` (they agree):

- **Provided:** `physics_articulated`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_articulated]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_articulated` (a `.so` under `runtime/modules/physics_articulated/`) and copies `module.toml` next to it. |
| `include/engine/physics_articulated/capabilities.hpp` | Public capability types: ABI constant, figure handles, link and figure descriptions, drive configuration, joint state, capability struct. |
| `source/records.hpp` | Figure and link records (rigid handle mirrors, assembly reference rotations, drive state) and the module settings. |
| `source/physics_articulated.cpp` | Module class (lifecycle, configuration, figure creation with rollback, drive update, measurement, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public double-precision math vocabulary shared by this module's angle measurement. |

## How it works

**Creation.** `createFigure` validates the parent tree, reserves a collision category unless `selfCollision` is set, and then walks the links in order: each link's world transform is composed from its parent's freshly created body and the relative placement, the body is created through the rigid capability with the figure's category and mask, and the joint to the parent is created with the description's anchors, axis, and limits. The relative rotation `parent^-1 x link` at the assembly pose is stored per link as the reference for drives and angle queries. Any failure rolls back every body and joint created so far.

**Drive update.** `updateDrives` iterates the figure's links in assembly order (deterministic), reads each driven joint's description and the two body states from the rigid capability, decomposes the relative rotation with a swing-twist split about the hinge axis, and applies the PD law. The derived velocity target lands in the rigid joint's motor, so the next rigid step executes it with the solver's torque budget. Consumers that pause their simulation simply stop calling `updateDrives`; the motors keep their last targets until retuned or disabled.

**Measurement.** `copyJointState` reports the same decomposition the drives use: signed twist about the hinge axis (hinge joints) or swing magnitude plus twist (ball-socket joints), each relative to the assembly frame. Recording a figure's pose over time therefore reads through one call per joint per step.

## Threading and lifetime

The module has no locks; serialize calls from one thread. Figures hold rigid handles owned by the target world, so destroying the world (or a link body through the rigid capability) invalidates the figure's mirrors: the module does not observe foreign destruction and subsequent drive or measurement calls fail through the rigid capability's unknown-handle checks. `stop()` destroys every remaining figure's bodies and joints through the rigid capability and frees the category pool. Handles are monotonic and never reused.

A consumer creates an anchored two-link pendulum and drives its elbow:

```cpp
engine::physics_articulated::ArticulatedCapability &articulation =
    context.acquireCapability<engine::physics_articulated::ArticulatedCapability>();
if (articulation.abi != engine::physics_articulated::physicsArticulatedCapabilityAbi)
{
    return;
}
engine::physics_articulated::FigureDescription description{};
engine::physics_articulated::LinkDescription links[2] = {};
links[0].parentIndex = -1; // static root: description.rootDynamic = false
links[1].parentIndex = 0;
links[1].jointToParent = engine::physics_articulated::ArticulatedJointKind::Hinge;
links[1].limitsEnabled = true;
description.links = links;
description.linkCount = 2;
description.rootDynamic = false;
const engine::physics_articulated::FigureHandle figure =
    articulation.createFigure(articulation.self, rigidWorld, &description, nullptr);

engine::physics_articulated::JointDrive drive{};
drive.enabled = true;
drive.stiffness = 40.0;
drive.damping = 4.0;
drive.targetAngleRadians = 0.7;
drive.maximumTorque = 120.0;
articulation.setJointDrive(articulation.self, figure, 1, &drive);
// per fixed step:
articulation.updateDrives(articulation.self, figure, fixedDeltaSeconds);
rigid.step(rigid.self, rigidWorld, fixedDeltaSeconds);
```
