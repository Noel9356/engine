# physics_rigid configuration

All keys live in the `[physics_rigid]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_rigid]` keys are silently accepted.

Every value is read once in `construct()` and copied into each world at `createWorld`; the module does not follow configuration reloads. Changing a world's gravity after creation is done through `setWorldGravity`. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_rigid.angular_sleep_tolerance` | number | > 0 | `0.14` | Angular speed (radians per second) below which an island member counts as motionless for sleeping. |
| `physics_rigid.contact_margin` | number | [0, 0.1) | `0.02` | Speculative contact margin (meters): shapes within this distance count as touching, so resting contacts persist instead of detaching and re-impacting each step, and the velocity solve closes speculative gaps within the step (which removes impact-impulse overshoot). |
| `physics_rigid.ccd_velocity_threshold` | number | [0, 1000] | `1.5` | Speed (meters per second) above which a continuous-collision-enabled body is swept from its pre-integration transform to the integrated one, so fast movers stop at thin geometry instead of tunneling; slower bodies integrate without the sweep. |
| `physics_rigid.contact_slop` | number | (0, 1) | `0.005` | Penetration (meters) the position solver tolerates before correcting; resting bodies keep up to this much overlap so manifolds persist. |
| `physics_rigid.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_rigid.gravity` | number[3] | finite vector | `[0.0, -9.81, 0.0]` | Gravity acceleration adopted by worlds created with `WorldDescription.useConfiguredGravity` and the fallback for non-finite configured values. |
| `physics_rigid.joint_baumgarte_factor` | number | (0, 1] | `0.2` | Fraction of the remaining joint position error (anchor offsets, swing tilt, limit violations) converted into bias velocity per step. Higher values tighten joints faster at the cost of added energy; contact penetration never uses a bias velocity (the position solver handles it). |
| `physics_rigid.linear_sleep_tolerance` | number | > 0 | `0.08` | Linear speed (meters per second) below which an island member counts as motionless for sleeping. |
| `physics_rigid.maximum_angular_velocity` | number | >= 1 | `200.0` | Angular speed clamp (radians per second) applied after integration to keep rotation integrators stable. |
| `physics_rigid.maximum_bodies` | integer | 1 - 10000000 | `100000` | Largest body count accepted per world; `createBody` reports `CapacityExceeded` beyond it. |
| `physics_rigid.maximum_joints` | integer | 1 - 10000000 | `50000` | Largest joint count accepted per world; `createJoint` reports `CapacityExceeded` beyond it. |
| `physics_rigid.maximum_linear_correction` | number | (0, 10) | `0.4` | Upper bound (meters for the position solver's per-iteration displacement, meters per second for joint bias velocities) so deep overlaps correct over several steps instead of popping. |
| `physics_rigid.maximum_linear_velocity` | number | >= 1 | `500.0` | Linear speed clamp (meters per second) applied after integration as a tunneling safety net. |
| `physics_rigid.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped instead of accumulating into a catch-up spiral. |
| `physics_rigid.maximum_substeps` | integer | 1 - 64 | `16` | Cap on how many canonical substeps one step may split into when the delta exceeds `substep_maximum_delta`; a delta whose required count exceeds the bound integrates at a coarser canonical step instead of spending an unbounded pass count. |
| `physics_rigid.maximum_trigger_events` | integer | 1 - 1000000 | `1024` | Bound on the per-world pending trigger-event queue; once full, the oldest event is dropped with the overflow flag set (visible through `triggerEventCount`) until the consumer drains the queue. |
| `physics_rigid.maximum_triggers` | integer | 1 - 100000 | `256` | Largest trigger count accepted per world; `createTrigger` reports `CapacityExceeded` beyond it. |
| `physics_rigid.maximum_worlds` | integer | 1 - 4096 | `16` | Maximum concurrently existing worlds. |
| `physics_rigid.position_correction_factor` | number | (0, 1] | `0.4` | Fraction of the contact penetration the position solver removes per iteration. Higher values de-penetrate faster; the per-iteration correction stays clamped by `maximum_linear_correction`. |
| `physics_rigid.restitution_velocity_threshold` | number | [0, 100) | `0.3` | Approach speed (meters per second) below which restitution is suppressed so resting contact does not jitter. |
| `physics_rigid.solver_position_iterations` | integer | 1 - 1000 | `4` | Position-solver passes per step that move transforms directly to remove contact penetration beyond `contact_slop` (split impulse scheme; no bias velocities, no bounce). |
| `physics_rigid.solver_velocity_iterations` | integer | 1 - 1000 | `20` | Sequential impulse iterations per step over contacts and then joints. |
| `physics_rigid.spatial_hash_cell_size` | number | (0.0001, 1000000] | `2.0` | Broadphase cell edge (meters). Cells roughly the size of the largest common shape balance insertion cost against candidate count. |
| `physics_rigid.substep_maximum_delta` | number | [0, 0.1] | `0.008333333333333333` (1/120) | Internal canonical substep target: a `step` delta above it is split into equal internal passes, so callers at coarse rates integrate on the same step the fine rates use and results converge across frame pacing. Zero disables splitting. |
| `physics_rigid.time_to_sleep` | number | [0, 60] | `0.6` | Seconds an island must stay motionless before its members sleep. |
| `physics_rigid.warm_starting` | boolean | `true` / `false` | `true` | Applies the previous step's accumulated contact and joint impulses before iterating. Disabling it makes the solver re-converge from zero every step (useful for A/B comparisons). |
## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite arrays are ignored and the built-in default stays active. Budgets are enforced when objects are created: `createWorld` beyond `maximum_worlds`, `createBody` beyond `maximum_bodies`, `createJoint` beyond `maximum_joints`, and `createTrigger` beyond `maximum_triggers` report `CapacityExceeded`. Lowering a configured limit affects new allocations; it does not evict existing objects. `stop()` releases all worlds and resets the handle counters.

## Where the settings show up

- The solver block (`solver_velocity_iterations`, `contact_slop`, `contact_margin`, `joint_baumgarte_factor`, `position_correction_factor`, `maximum_linear_correction`, `restitution_velocity_threshold`, `warm_starting`) shapes contact and joint convergence: the velocity solve enforces restitution and speculative gap closing, the position pass removes penetration directly, and the defaults target stable stacking at the 60 Hz default rate.
- The clamps (`maximum_linear_velocity`, `maximum_angular_velocity`) are the tunneling and spin safety net applied after every integration.
- `substep_maximum_delta` is the frame-rate independence guarantee made internal: the demo's 30/60/120 Hz comparison runs finish bit-identical because every rate integrates through the same canonical substeps; see [overview.md](overview.md) for the splitting rule.
- The sleep block (`linear_sleep_tolerance`, `angular_sleep_tolerance`, `time_to_sleep`) trades idle CPU for wake latency; sleeping islands skip broadphase pairs and solve nothing until touched.
- `spatial_hash_cell_size` and the budgets (`maximum_worlds`, `maximum_bodies`, `maximum_joints`, `maximum_triggers`) bound memory: bodies, joints, triggers, and manifolds are dense arrays sized by the live object counts, and the broadphase scratch reuses its cell buckets across steps. See [overview.md](overview.md) for the stepping and sleeping paths.
- `maximum_trigger_events` bounds the overlap-event queue; consumers that poll less often than the event rate raise it so transitions are never dropped (the overflow flag reports a drop either way).
