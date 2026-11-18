# physics_rigid module

physics_rigid provides deterministic double-precision rigid body simulation: static, dynamic, and kinematic bodies with sphere, box, capsule, and plane shapes; a spatial-hash broadphase; a warm-started sequential impulse solver for contacts and joints; island-based sleeping; and the query surface (ray casts, deepest sphere overlap, statistics) that other simulation modules and applications build on. The module owns no rendering state; consumers read body states and feed them to their own presentation layer.

## Capability

The module provides the `physics_rigid` capability (ABI version constant `physicsRigidCapabilityAbi = 1`, see `include/engine/physics_rigid/capabilities.hpp`). World, body, and joint handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidWorld` / `invalidBody` / `invalidJoint`) means "none". All positions, extents, and velocities are in meters, seconds, and radians; transforms are position plus an x/y/z/w quaternion.

### Frame-rate independence contract

The world advances in whole fixed steps only. `step(world, fixedDeltaSeconds)` performs exactly one step and must be called with a constant delta across a running simulation (values in (0, 0.1] are accepted; anything else is rejected). A delta above the configured `substep_maximum_delta` (1/120 seconds by default) is split into equal internal substeps, so a consumer that steps at 30 or 60 Hz integrates through the same canonical passes as one stepping at 120 Hz — identical inputs produce identical results at any of those rates, which the demo's self-test verifies. `simulate(world, elapsedSeconds)` is the accumulator variant: it adds elapsed wall time to the world's budget, runs whole fixed steps at the configured `fixed_rate_hz`, and drops backlog beyond `maximum_steps_per_simulate`, so a stalled consumer can never spiral. Application loops that follow the engine's FrameClock (fixed update count plus interpolation alpha) call `step` once per fixed update and render with interpolated transforms; the simulation result then depends only on the step count, not on how frames were paced.

| Group | Functions | Purpose |
| --- | --- | --- |
| World lifetime | `createWorld(description, outError)`, `destroyWorld(handle)`, `setWorldGravity`, `copyWorldGravity` | Worlds own bodies, joints, manifolds, and the accumulator budget. `WorldDescription.useConfiguredGravity` adopts the module's `[physics_rigid] gravity`. |
| Body creation | `createBody(world, description, outError)`, `destroyBody(world, body)` | One body per call; planes must be static; mass comes from density (shape volume) or an explicit mass with a shape-derived inertia. |
| Body inspection | `bodyCount`, `copyBodyHandles`, `copyBodyDescription`, `copyBodyState` | Creation-time description echo (shape, kind, material, filter bits, `userTag`) and the runtime snapshot (transform, velocities, awake flag). |
| Body mutation | `setBodyTransform`, `setBodyKind`, `setBodyLinearVelocity`, `setBodyAngularVelocity`, `setBodyMaterial`, `setBodyAwake` | Teleports (optionally waking), retargets the motion model (mass properties recomputed), or overrides velocities and materials; sleeping bodies zero their velocities when moved without waking. |
| Forces | `applyForceAtPoint`, `applyTorque`, `applyImpulseAtPoint`, `applyAngularImpulse` | Forces accumulate for exactly one step and clear afterwards; impulses apply immediately. Non-dynamic bodies reject both. |
| Joints | `createJoint`, `destroyJoint`, `copyJointDescription`, `setJointMotor` | Distance, ball-socket (optional cone/twist limits), and hinge (anti-swing, optional angle limits, optional velocity motor) constraints. |
| Stepping | `step(world, fixedDeltaSeconds)`, `simulate(world, elapsedSeconds)` | The deterministic fixed-step primitive and the accumulator convenience; see the contract above. |
| Queries | `rayCast`, `collideSphere`, `worldStatistics` | Closest-hit ray cast (hit point, surface normal, distance), deepest sphere overlap (push-out normal, depth, responsible body), cumulative counters. |

**Error convention:** functions returning `bool` fail on unknown handles or null outputs; creation functions also fill the `PhysicsError` out-parameter (`InvalidDescription`, `CapacityExceeded`, `Unsupported`, `SimulationFailure`). A `SimulationFailure` restores the pre-step state: when any body enters a non-finite state during a step, the whole step is rolled back and reported instead of propagating NaNs.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_rigid.cpp` (they agree):

- **Provided:** `physics_rigid`.
- **Required:** none.
- **Optional:** none.
- **Declared permissions:** none.

Consumers acquire the capability through `ParticipantContext::acquireCapability` with the kernel permission `capability.physics_rigid`, which the manifest's `requires` list grants to dependent modules.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_rigid]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_rigid` (a `.so` under `runtime/modules/physics_rigid/`) and copies `module.toml` next to it. |
| `include/engine/physics_rigid/capabilities.hpp` | Public capability types: ABI constant, handles, shape/body/joint descriptions, state snapshots, statistics, capability struct. |
| `source/math.hpp` | Double-precision vector, quaternion, and matrix helpers (rotation, swing-twist decomposition, transform composition). |
| `source/records.hpp` | Plain-data records owned by the module: bodies, joints, persistent manifolds, worlds, and the shared `SimulationSettings`. |
| `source/shapes.hpp` / `source/shapes.cpp` | Shape validation, mass properties (exact solid capsule inertia), world-space AABBs, the narrowphase pair routines (sphere/box/capsule SAT plus reference-face clipping, plane half-spaces), ray casts, and sphere-overlap resolution. |
| `source/broadphase.hpp` / `source/broadphase.cpp` | Uniform spatial hash over solid body AABBs; deduplicated, index-sorted pair output with motion-kind, sleep, and collision-filter rejection. |
| `source/solver.hpp` / `source/solver.cpp` | Sequential impulse constraints: contact points with Baumgarte/restitution bias and two-tangent friction, distance/ball-socket/hinge joint rows with limits and motors, warm-start impulse application. |
| `source/world.hpp` / `source/world.cpp` | Step orchestration (forces, broadphase, narrowphase, solve, integrate, island sleeping, statistics), body/joint lifecycle, ray casts, sphere overlap, and the non-finite guard. |
| `source/physics_rigid.cpp` | Module class (lifecycle, configuration, budgets, capability binding) and entry table. |

## How it works

**Bodies and shapes.** `createBody` validates the shape (positive extents, finite values, planes static-only), normalizes the rotation, derives mass properties, and stores the body in the world's dense array with a monotonic handle. Density above zero multiplies the exact shape volume (sphere, box, or the solid capsule cylinder-plus-hemispheres volume); otherwise the explicit mass is used with the shape-derived principal-axis inertia. Sphere, box, and capsule inertia are the closed-form formulas about the shape's local axes; the world-frame inverse inertia is reconstructed per solve as `R diag(1/I) R^T`. Static and kinematic bodies carry zero inverse mass; kinematic bodies integrate their velocities every step but ignore forces, and both kinds push dynamic bodies through contacts without being pushed. `setBodyKind` re-derives the mass properties from the stored mass source, so a body can be promoted from static scenery into a dynamic actor (and back) without recreating it.

**Collision pipeline.** Every step, the broadphase inserts each solid body's world-space AABB into a uniform spatial hash keyed by the configured `spatial_hash_cell_size`; dynamic and kinematic bodies probe their own cells for candidates. Candidate pairs are deduplicated by an unordered index pair, rejected when both bodies are static, both sleeping, or mutually excluded by the category/mask filter, then AABB-tested, and the survivors are sorted by dense index so narrowphase and the solver see a deterministic order. Planes are unbounded and never hashed: every awake non-static solid body is tested against every plane directly and merged into the same manifold store with canonical pair ordering. Narrowphase dispatches per pair shape combination within the configured `contact_margin`: sphere tests are analytic; box pairs run a full 15-axis separating-axis test with reference-face clipping against the incident face polygon (up to four points, edge axes preferred only when they beat the best face axis by more than five percent so stacks rest on face contacts); capsules contribute segment-swept sphere tests (two interior samples against boxes, exact closest-segment for capsule pairs); planes clip box corners and sample capsule endpoints. Separations may be positive up to the margin (speculative contacts), which keeps resting manifolds alive instead of detaching and re-impacting every step.

**Manifolds and warm starting.** Narrowphase results land in persistent manifolds keyed by the unordered body-handle pair. Fresh points match persisted points by anchor proximity and inherit their accumulated normal and tangent impulses plus the tangent basis they were accumulated along, so a resting stack starts each step from the previous solution instead of re-converging from zero. Manifolds that narrowphase did not refresh are dropped the same step. After the final solve iteration the current tangent basis is written back into the persisted points so the next step's warm start applies impulses in the same frame.

**Canonical substepping.** `stepWorld` validates the delta, splits it into `ceil(delta / substep_maximum_delta)` equal internal passes (capped at 16), and runs the pipeline once per pass. Every pass re-runs broadphase, narrowphase, solving, and integration on the shared state, so warm starting carries across passes and the sleeping timers accumulate real time. The split count depends only on the delta and the configured target, which keeps coarse-rate callers bit-identical to fine-rate callers over the same wall-clock span.

**Solving.** Constraints solve in fixed order: contacts sorted by pair key, then joints in creation order, iterated `solver_velocity_iterations` times per step. Each contact point runs a block solve: the normal row targets restitution (applied only to approach speeds beyond `restitution_velocity_threshold`) plus the speculative gap - a positive separation must close within the step, which stops approaching bodies before they touch and removes impact overshoot; the accumulated impulse may never open a speculative gap faster than the gap allows. Two tangent rows follow, clamped to the Coulomb cone `friction x accumulated normal impulse` with the pair coefficient `sqrt(fA x fB)`. Joints contribute: distance rows along the current anchor separation; ball-socket rows as three anchor-coincidence components on an anchor-derived orthonormal basis; hinge rows as the same anchor rows plus two anti-swing angular rows that drive the measured swing tilt back to zero, with Baumgarte bias velocities for the joint position error. Hinge limits clamp the twist angle (measured against the relative frame captured at creation through a swing-twist decomposition) into the configured range; ball-socket limits clamp the swing cone half-angle and the twist half-angle; the hinge motor tracks a target angular velocity about the hinge axis within a torque budget. All accumulated impulses live in the world-owned joint and manifold records, which is what makes warm starting exact across steps.

**Position solving.** After the velocity iterations, `solver_position_iterations` passes move the transforms directly to remove the remaining contact penetration: each contact point whose separation exceeds `contact_slop` applies a positional displacement (`position_correction_factor` of the error, clamped by `maximum_linear_correction`) along the contact normal through the point's mass matrix. Splitting position repair from the velocity solve means penetration leaves without injecting the bounce energy a bias velocity would, so stacks settle instead of oscillating.

**Sleeping.** After integration, islands are formed over the contact and joint graphs (union-find over dynamic bodies). An island whose dynamic members all stay below the linear and angular sleep tolerances accumulates sleep time; once it passes `time_to_sleep` every member sleeps (velocities zeroed, broadphase skipped for sleeping-sleeping pairs). Any island motion above tolerance resets timers and wakes the island, so a body nudged by a falling object or an opened joint drive rejoins the simulation immediately. Kinematic and static bodies never sleep and never block an island from sleeping.

**Queries.** `rayCast` walks every body and plane with per-shape analytic tests (sphere quadratic, box slab test in local space, capsule as the minimum over both caps and the clipped cylinder surface, plane half-space) and reports the closest hit with the surface normal opposing the ray. `collideSphere` reports the deepest penetration over bodies and planes with the push-out direction for the query sphere; it is the collision entry point for consumers that resolve their own particles (for example the physics_cloth module). `worldStatistics` reports body counts per kind, awake and sleeping counts, joint count, contact pair and point counts, the cumulative step count, and the previous step's cost in milliseconds.

## Threading and lifetime

The module has no locks. Object creation and destruction, `stop()`, and the stepping functions must be serialized from one thread (the main or simulation thread). Read-only queries (`copyBodyState`, `rayCast`, `collideSphere`, `worldStatistics`) may run concurrently with each other while no object-lifecycle call or step is in flight. Records live in the world's dense arrays and die with the world: `destroyWorld` releases every body, joint, and manifold; `stop()` clears all worlds and resets the handle counters. Destroying a body destroys its joints and drops every manifold that references it. Handles are monotonic and never reused, so a stale handle is rejected rather than aliasing a newer object.

A consumer drives one fixed update per FrameClock step and renders interpolated states:

```cpp
engine::physics_rigid::PhysicsRigidCapability &physics =
    context.acquireCapability<engine::physics_rigid::PhysicsRigidCapability>();
if (physics.abi != engine::physics_rigid::physicsRigidCapabilityAbi)
{
    return;
}
const engine::physics_rigid::WorldHandle world = physics.createWorld(physics.self, &description, nullptr);
for (std::uint32_t step = 0; step < timing.fixedUpdateCount; ++step)
{
    physics.step(physics.self, world, fixedDeltaSeconds);
}
```
