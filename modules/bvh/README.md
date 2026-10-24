# bvh — the spatial index module

The provider of the `bvh` capability (implementation member `bvh.binary`): CPU-side
"what is near what" answers over axis-aligned boxes, without touching the GPU. The
implementation is a binary bounding volume hierarchy whose split strategy is
configurable (see Configuration below) — every strategy keeps deterministic
tiebreaks, so simulation runs stay reproducible, and every build is cheap enough
at the registry's object caps (a 32-box rebuild is microseconds) that the
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

The module ships these values in its own `config.toml` (the lowest configuration layer — `config/engine.toml` and staged overrides win over them, see `docs/configuration.md`):

| Key | Type | Default | Meaning |
|---|---|---|---|
| `split` | string | `"midpoint"` | the split strategy of every hierarchy the module builds: `"median"` (longest axis at the object median — perfectly balanced, deterministic down to the index tiebreak), `"midpoint"` (longest axis at the range's centroid midpoint) or `"sah"` (binned surface area heuristic — the best query throughput at larger box counts, at a markedly higher build cost). An unknown spelling fails startup precisely |

The shipped default is the benchmark winner (`benchmark.cpp`): the engine's
contract is build-per-call, and the midpoint tree's build + query TOTAL is the
lowest at every measured box count and scene shape — spread and clustered
scenes at the registry cap of 32 boxes and at the 256/1024 stress counts
(e.g. 256 spread boxes: midpoint ~29 µs vs median ~56 µs vs SAH ~269 µs per
build+batch call). SAH remains the right choice for a consumer that builds
once and queries many times; median remains available where perfect balance
matters. Query results are identical under every strategy (the overlap test
is exact), so the choice only moves traversal cost around, never correctness.

## Files

- `bvh_capability.hpp` - the published capability surface (box vocabulary + the two queries)
- `binary_bvh.hpp` - the hierarchy implementation and its split strategies (module-private)
- `benchmark.cpp` - the standalone split-strategy benchmark that fixes the shipped default (no dependencies; `g++ -std=c++20 -O2 -o split_bench modules/bvh/benchmark.cpp`)
- `config.toml` - the module's shipped defaults (the lowest configuration layer)
- `bvh.hpp` / `bvh.cpp` - the module, the capability implementation
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_BVH`)
