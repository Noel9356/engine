# physics_rope module

physics_rope provides rope and cable simulation: XPBD node chains held together by stretch distance constraints and two-apart bending distance constraints, stepped with fixed-timestep substepping, colliding against a physics_rigid world's bodies and planes through that module's sphere-overlap query, and publishing a per-segment tension readout, a grappling-style rest-length reel (`setRopeLength`), and optional strain breaking that snaps over-stretched segments so the rope falls into independent pieces. Ropes hang from pins and cranes, tow behind moving rigid bodies via body-local anchors, drape over static scenery, and react to a uniform wind acceleration, while the rigid capability supplies gravity, body transforms, and every contact.

## Capability

The module provides the `physics_rope` capability (ABI version constant `physicsRopeCapabilityAbi = 3`, see `include/engine/physics_rope/capabilities.hpp`, whose ABI history comment records what each bump appended; strain breaking, tension readout, and reeling arrived in ABI 2, the simulation-LOD surface (`setRopeImportance` and the `lodSkippedSteps` counter) arrived in ABI 3, and consumers check `abi >= 3` before touching those additions) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Rope handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidRope`) means "none". Node positions and velocities are world-space meters and meters per second packed `x, y, z` per node in chain order (node 0 first); the copy functions report the required capacity through their out-parameter (the node copies need `segments + 1` triples, `copyRopeTension` needs `segments` doubles, one per segment).

### Frame-rate independence contract

Like the rigid module, rope advances in whole fixed steps only. `step(rope, fixedDeltaSeconds)` performs exactly one step (values in (0, 0.1] are accepted) and must be called with a constant delta; `simulate(rope, elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, dropping backlog beyond `maximum_steps_per_simulate` and returning the number of steps executed. Each step internally runs `substeps_per_step` XPBD substeps — the small-steps schedule that keeps constraint stiffness iteration-independent — so the result depends only on the step count.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createRope(world, description, outError)`, `destroyRope(rope)` | Builds the node chain (a straight line from `start` to `end`), the stretch and bending constraint sets, and the description's end attachments against the given rigid world; the rope adopts that world's gravity. |
| Inspection | `ropeInfo(rope, outInfo)`, `copyNodePositions(rope, outPositions, capacity, outRequired)`, `copyNodeVelocities(rope, outVelocities, capacity, outRequired)`, `copyRopeTension(rope, outTension, capacity, outRequired)`, `ropeStatistics(rope, outStatistics)` | Segment, node, and constraint counts, packed node state, the most recent completed step's per-segment tension in force units, and the cumulative step, substep, contact, timing, broken-segment, and maximum-tension counters. |
| Node attachment | `setNodePinned(rope, nodeIndex, pinned)`, `pinNodeToPosition(rope, nodeIndex, position)`, `attachNodeToBody(rope, nodeIndex, body, localAnchor)`, `detachNode(rope, nodeIndex)`, `isNodeAttached(rope, nodeIndex, outAttached)` | Pins a node at its current position or an explicit world point, follows a rigid body's local anchor each substep, releases any pin or attachment; attached nodes carry no mass (the rigid body drives them). |
| Reeling | `setRopeLength(rope, length)` | Grappling-style rest-length control: sets the rope's total rest length immediately (every segment's rest becomes `length / segmentCount`, bend rests follow at twice that); validates finite `length > 0` and takes effect on the next step. |
| Wind | `setWindAcceleration(rope, wind)` | Uniform wind acceleration (meters per second squared) added to every free node's velocity each substep; consumers can bridge a wind field's local sample here per step. |
| Stepping | `step(rope, fixedDeltaSeconds)`, `simulate(rope, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |

**Error convention:** `createRope` fills the `RopeError` out-parameter (`InvalidDescription` for degenerate chains, non-finite values, or friction outside [0, 1], `CapacityExceeded` beyond the rope, per-rope node, or shared node budgets, `InvalidHandle` when an attached end references a body the rigid world does not know). The remaining entry points report unknown rope handles and out-of-range node indices through their `false` return; `step` reports the `SimulationFailure` class of failure through its `false` return when any node position becomes non-finite, aborting the step before the statistics counters advance.

## Simulation model

**Chain and constraints.** `createRope` places `segments + 1` nodes on the straight line from `start` to `end` and builds two distance-constraint families: stretch constraints between neighboring nodes (rest length `restLength / segments`, the description's `stretchCompliance`) and — for chains of two or more segments — bending constraints between two-apart neighbors (rest length twice the segment length, the `bendingCompliance` — the distance-based bending model that resists kinking without dihedral calculations). Compliances are the XPBD `alpha` values from the description; 0 is perfectly rigid and higher values are floppier.

**XPBD stepping.** Each substep: body-attached targets refresh from the rigid world (body position plus the rotated local anchor); attached nodes snap to their targets and carry no mass; free nodes integrate gravity (adopted from the world at creation) plus the wind acceleration, apply damping (`1 / (1 + dt x damping)`), and advance positions (previous positions recorded); distance constraints then solve once in construction order — all stretch constraints followed by all bend constraints — with the compliance-scaled correction `lambda = (-length + restLength) / (inverseMassSum + alpha / dt^2)` (a constraint touching a zero-mass node steers its free counterpart); free nodes resolve against the rigid world; and finally velocities are recomputed from the position change. Because each substep uses `fixedDeltaSeconds / substeps_per_step`, doubling the substep count halves the per-substep displacement while the XPBD compliance scaling keeps stiffness constant — the time-step independence the compliance term exists for.

**Collision.** Per substep, every free node takes part in one batched `collideSpheres` query against the rigid world with the rope's node radius and is projected out along the push-out direction. The final velocity pass splits a contacted node's velocity into normal and tangential components: the approach velocity along the normal is removed and the tangential part scales by the friction retention — `friction^substepDelta`, the per-second "sliding velocity kept" factor the capability documents, so the same wall-clock contact time removes the same tangential speed at any step rate (0 = full stick, 1 = frictionless). `collideWithWorld = false` skips the queries entirely. Ropes therefore slide realistically over ramps and get carried by moving rigid bodies without any rope-specific solver state inside the rigid module.

**Attachments.** Every node is `Free`, `Pinned` (a world position), or `Body` (a rigid body plus local anchor). Pinned and attached nodes have zero inverse mass, skip integration, and sit exactly at their targets; `detachNode` restores their mass. Body-attached targets recompute every substep from the body transform, so cranes, tow lines, and chain links follow moving machinery, and a body destroyed in the rigid world freezes its node at the last computed anchor. The description's `pinStart` / `pinEnd` / `attachStartToBody` / `attachEndToBody` flags set the initial end states at creation.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_rope.cpp` (they agree):

- **Provided:** `physics_rope`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_rope]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_rope` (a `.so` under `runtime/modules/physics_rope/`) and copies `module.toml` next to it. |
| `include/engine/physics_rope/capabilities.hpp` | Public capability types: ABI constant, rope handle, rope description, info and statistics, capability struct. |
| `source/records.hpp` | Rope record (nodes, stretch and bend constraints, attachment state, breaking flags, tension bookkeeping, statistics) and module settings. |
| `source/physics_rope.cpp` | Module class (lifecycle, configuration, chain construction, XPBD stepping, collision resolution, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used for body-anchor transforms. |

## How it works

**Creation.** `createRope` validates the chain (at least one segment), rest length, node radius, mass, compliances, damping, friction range, and break strain, probes attached ends through `copyBodyState` when either end attaches to a body, reads the target world's gravity through `copyWorldGravity` (falling back to `0, -9.81, 0` when the read fails), builds the node arrays, generates the constraint list, and flags the description's pinned or attached ends (zeroing their inverse masses and seeding the anchors) before the handle is published.

**Stepping.** `stepRope` runs the substep loop described above and aborts (returning false without counting the step) when any node position becomes non-finite. The statistics counters accumulate steps, substeps, the contact count of the previous step, and the previous step's wall-clock cost. `simulate` mirrors the rigid module's accumulator: whole steps at the configured rate, budget clamp, and zero-budget reset when the step cap trips so a stalled consumer never accumulates debt.

**Tension.** Every stretch solve adds its row's |lambda| to the owning segment's accumulator (bend rows do not count); the accumulators reset at step start and, once the step completes without failure, divide by `fixedDeltaSeconds^2` to publish one force value per segment — lambda carries mass-times-length units (the applied correction is inverse-mass x lambda), so the division yields newtons. `copyRopeTension` copies the most recent completed step's values with the same capacity/`outRequired` idiom as the node copies (zero before the first step), `RopeStatistics.maximumTension` carries the largest segment value of that step, and a segment that breaks mid-step keeps the |lambda| it accumulated up to the break.

**Reeling.** `setRopeLength(rope, length)` validates finite `length > 0` and retargets the rest lengths immediately: every stretch constraint takes `length / segmentCount` and every two-apart bend constraint follows at twice that (the relation creation builds), so reeling is never fought by stale bend rests. Nothing is woken or warmed — the module has no sleep and XPBD has no warm start — the nodes simply respond from the next step on as the solver drives current lengths toward the new rests, which makes the call the whole reel-in/reel-out primitive for grappling hooks, winches, and tow lines.

**Breaking.** With `RopeDescription.breakStrain` above zero, each substep checks every intact segment before the solve and breaks it permanently when its current length exceeds `restLength x (1 + breakStrain)` (stretch only — a compressed segment never snaps; 0 disables the check). Broken segments are skipped from then on and counted in `RopeStatistics.brokenSegmentCount`, and a bending constraint is released as soon as either of the two stretch segments it spans breaks, because holding the two-apart distance across a break would drag the separated pieces back together — the rule that lets the rope hang as two or more independent pieces sharing the node pool while pins and body attachments stay untouched.

**Interaction with rigid bodies.** Rope reads the rigid module's world gravity, body state copies, and the batched `collideSpheres` query, and writes nothing back: static ground planes catch draped chains, kinematic carriers drag them by their anchors, and dynamic hooks swing them — the rope never pushes a body, so any body kind works the same and rope is one-way coupled by design (see configuration.md for the cost trade-off this represents).

## Simulation LOD and robustness

Every rope carries an importance score (`setRopeImportance`, ABI 3) resolved through the five-band fidelity ladder with demotion hysteresis. Bands below Full keep the rope's fixed step size and cut the internal XPBD substep count instead (Simplified half, Procedural a quarter, Animation one), and Cached ropes freeze until the score rises again; with `lod_enabled` on, ropes below Full also update on a staggered cadence phase-spread by registration order, and `lodSkippedSteps` reports the skipped steps. Node collision resolves through one batched sphere-overlap query per substep (the rigid capability's `collideSpheres`), the per-node contact scratch lives on the rope record so ropes of different sizes reuse their own warmed-up buffers, and a step whose state would turn non-finite rolls back to its pre-step node state and reports `SimulationFailure`, leaving the rope untouched. The derived node velocities clamp to `maximum_velocity` so solver noise cannot escalate.

## Threading and lifetime

The module has no locks; serialize calls from one thread. Node arrays live in the rope record and die with it: `destroyRope` releases the arrays and returns the node budget to the shared counter; `stop()` clears every rope and resets the handle and node counters. Handles are monotonic and never reused. The batched `collideSpheres` queries and `copyBodyState` reads are read-only against the rigid world, so rope steps may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct; in the common single-world case the consumer steps the rigid world and every rope sequentially within one fixed update.

A consumer hangs a cable from a fixed point onto a moving hook body:

```cpp
engine::physics_rope::RopeCapability &ropeCapability =
    context.acquireCapability<engine::physics_rope::RopeCapability>();
if (ropeCapability.abi != engine::physics_rope::physicsRopeCapabilityAbi)
{
    return;
}
engine::physics_rope::RopeDescription description{};
description.segments = 24;
description.restLength = 6.0;
description.pinStart = true;        // node 0 holds the start point
description.attachEndToBody = true; // the last node follows the hook's local anchor
description.endBody = hookBody;
const engine::physics_rope::RopeHandle rope =
    ropeCapability.createRope(ropeCapability.self, rigidWorld, &description, nullptr);
const double wind[3] = {0.6, 0.0, 0.0};
ropeCapability.setWindAcceleration(ropeCapability.self, rope, wind);
// per fixed step:
ropeCapability.step(ropeCapability.self, rope, fixedDeltaSeconds);
```
