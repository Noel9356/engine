# physics_builtin module - the built-in rigid body provider

## What this module provides

Capability `physics` (implementation-bound member `physics.builtin`, published in `physics_capability.hpp`): rigid bodies that drive scene registry objects. A body does not own geometry - the bound object's kind (sphere radius, box half extents) IS the collider, read live from the registry: resize the object and the collider follows, remove the object and the body drops.

```lua
local ball = engine.spawn_object({ type = "sphere", position = {1.5, 3.0, 0.0}, radius = 0.3 })
local body = engine.add_physics_body(ball, { restitution = 0.6 })
engine.set_body_velocity(body, 0.5, 0.0, -0.2)
```

## The simulation contract

The module subscribes to the frame event, integrates every dynamic body with **semi-implicit Euler** under the configured gravity (velocity first, then position - the stable order for gravity-driven motion under restitution), resolves contacts, and writes the resulting transforms back through the scene registry. The registry stays the one source of truth: the renderer's incremental rebuild picks moved objects up with no physics-specific renderer path, animation tweens on other objects compose freely, and scripts can teleport a body's object between frames (the integrator adopts the new pose).

Contact set of the built-in provider:

| pair                | response                                                        |
| ------------------- | --------------------------------------------------------------- |
| body vs ground plane | push out along Y, reflect vertical velocity per restitution, rest below 0.15 m/s |
| sphere vs sphere    | positional correction split half/half, normal impulse with the pair's min restitution |
| sphere vs box       | closest-point response (AABB; degenerate inside case exits through the shallowest face) |
| box vs box          | skipped - rotated box colliders need the OBB path a heavier provider brings |

Frame timing: the frame delta is clamped to `[physics].max_step_seconds` (default 1/30 s) - a stalled frame must not teleport bodies through geometry.

Pair culling runs through the `bvh` module's hierarchy (capability member `bvh.binary`) over the bodies' world bounds, rebuilt per step (at the registry cap that is microseconds). The same capability a future CPU SDF bake would consume - one spatial index, two consumers, and physics demands the family (`bvh`), so a different spatial index could win through `[providers]` without this module noticing.

## Honest scope

This is a compact, correct-for-the-analytic-scene simulator: gravity, restitution, damping, sphere/sphere and sphere/box contacts, AABB box-box contacts, a ground plane, raycasts against every body shape (hit position and normal, nearest-first), and automatic sleeping - a body whose linear and angular speed stay under the thresholds for the configured quiet time stops integrating (and wakes on contact or an explicit impulse), which is what keeps a settled scene from burning cycles forever. It still has no joints, no continuous collision detection and no rotational dynamics; and it collides mesh objects as their bounding-box proxy (the scene_registry module's documented contract). Where real games would hit those remaining walls, the family pattern is the answer: a provider wrapping Jolt (or any engine) declares `physics.jolt`, implements the same `PhysicsCapability`, and wins through `[providers] physics = "physics.jolt"` - mods keep demanding `physics` and notice nothing but better behavior.

## Configuration

```toml
[physics]
# gravity = [0.0, -9.81, 0.0]     # world gravity, shared by every body
# ground_enabled = true           # the infinite ground plane at ground_height
# ground_height = 0.0
# max_step_seconds = 0.03333      # frame delta clamp
# sleep_speed = 0.15              # linear/angular speed under which a body counts as quiet
# sleep_delay = 2.0               # quiet seconds before a body falls asleep
```

## Files

- `physics_capability.hpp` - the published capability surface
- `physics.hpp` / `physics.cpp` - the module, integrator, contacts
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_PHYSICS_BUILTIN`)
