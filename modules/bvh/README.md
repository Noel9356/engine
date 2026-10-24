# bvh — the spatial index module

The provider of the `bvh` capability (implementation member `bvh.binary`): CPU-side
"what is near what" answers over axis-aligned boxes, without touching the GPU. The
implementation is a binary bounding volume hierarchy built by median split on the
longest axis — deterministic, which keeps simulation runs reproducible, and cheap
enough at the registry's object caps (a 32-box rebuild is microseconds) that the
capability is a stateless service: every call takes the full box set and answers
from a hierarchy built over it. No invalidation protocol, no stale-hierarchy bug
class.

Why a module: several consumers need the same spatial index, and the capability
model gives them one provider with no compile-time coupling. The physics_builtin module
culls collision pairs through it every step; a future CPU SDF bake (the renderer's
cost model is linear in the object count; a baked distance field needs fast
neighborhood queries) would rest on the same answers. A run whose mods do not pull
a consumer of the family never loads this module.

## Capability surface (`bvh_capability.hpp`)

| Member | Notes |
|---|---|
| `AxisAlignedBoundingBox` | the vocabulary box: plain `float[3]` min/max, a `center()` helper |
| `boxOverlaps(a, b)` | the exact overlap test, shared by provider and consumers |
| `BvhCapability::overlappingPairs(boxes, out)` | one-shot broadphase: for each box, every box overlapping it (self included; unordered per index) |
| `BvhCapability::queryOverlaps(boxes, query, out)` | one query against the given box set |

Both calls rebuild the hierarchy over `boxes` first — the caller's list is always
the newest truth there is. Results are exact (no false positives, no missed pairs
that a wider broadphase would report).

## Configuration

None. The implementation has no tuning surface: median split on the longest axis,
deterministic tiebreaks. A structure with knobs is a structure whose callers can
be handed a slow configuration.

## Files

- `bvh_capability.hpp` - the published capability surface (box vocabulary + the two queries)
- `binary_bvh.hpp` - the hierarchy implementation (module-private)
- `bvh.hpp` / `bvh.cpp` - the module, the capability implementation
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_BVH`)
