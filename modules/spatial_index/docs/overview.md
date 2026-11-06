# spatial_index module

spatial_index is a header-only geometry acceleration library: an axis-aligned bounding box (AABB) primitive, an incrementally updated dynamic AABB tree for moving objects, a statically built bounding volume hierarchy (BVH) for static geometry, and a combined scene index that queries both. It is not a loadable kernel module — there is no `.so`, no capability struct, and no `engineQueryModuleEntryTable()`; consumers simply include the headers and link the INTERFACE CMake target.

## Capability

None. spatial_index does not provide kernel capabilities and is not discovered or loaded by the kernel. It exposes plain C++ classes in `engine::spatial` from `include/engine/spatial_index/spatial_index.hpp` (umbrella header). There is no ABI version constant because the types are compiled into each consumer.

## Provided / required / optional capabilities

None, on all three counts. The module directory has no `module.toml` and no module entry table; the CMake target `engine_module_spatial_index` (alias `engine_spatial_index`) is an INTERFACE library that only forwards its include directory and requires C++20. Because it is header-only, nothing is published on the kernel's capability resolver, and the permission system is not involved.

## Architecture

| File | Purpose |
| --- | --- |
| `CMakeLists.txt` | INTERFACE target `engine_module_spatial_index` (alias `engine_spatial_index`): include directory + C++20 requirement only. |
| `include/engine/spatial_index/types.hpp` | Shared value types: `invalidNode`, generation-tagged `ProxyHandle`, `QueryStatistics`, `SpatialHit`, `DynamicTreeStatistics`. |
| `include/engine/spatial_index/aabb.hpp` | `Aabb` value type plus `merge`/`expanded` helpers used by all builders. |
| `include/engine/spatial_index/dynamic_aabb_tree.hpp` | `DynamicAabbTree`: incrementally maintained, rotation-balanced BVH over moveable proxies. |
| `include/engine/spatial_index/static_bvh.hpp` | `StaticBvh`: build-once BVH with median or binned-SAH splits and refit support. |
| `include/engine/spatial_index/scene_spatial_index.hpp` | `SceneSpatialIndex`: facade combining the static BVH and dynamic tree, with tagged hits. |
| `include/engine/spatial_index/spatial_index.hpp` | Umbrella header pulling in all of the above. |

## How it works

**AABB primitive.** `Aabb` stores per-axis min/max. `valid()` rejects NaNs and inverted boxes; `surfaceArea()` (computed in double) is the cost metric for both builders; `intersects()` is inclusive on touching edges.

**Dynamic AABB tree** (`DynamicAabbTree`). Each proxy is a leaf holding two boxes: a fat bound used for traversal and a tight bound used to confirm hits. The fat bound is the tight bound expanded by `fatMargin` plus the predicted motion (`displacement * displacementMultiplier`, per axis, sign-aware), so slow movers never trigger reinsertion.

- *Insert/update:* `update()` keeps the leaf where it is if the new tight bound still fits the old fat bound; otherwise the leaf is removed and reinserted. Insertion descends from the root choosing the child with the smallest inherited surface-area increase, with an early stop when pairing the leaf with the current node is cheapest; a new parent node is spliced in above the chosen sibling.
- *Balancing:* after every structural change `refitAndBalance()` walks to the root, recomputing bounds/masks/heights and applying AVL-style single rotations (`balance()`) wherever sibling heights differ by more than one.
- *Removal:* the leaf's parent is replaced by the leaf's sibling, the parent slot is freed onto a free list, and the slot's generation counter is bumped.
- *Handles:* `ProxyHandle` packs (generation, slot + 1); `resolve()` rejects stale generations, freed slots, and internal nodes, so using an old handle after removal fails cleanly instead of corrupting the tree.
- *Rebuild:* `rebuild()` collects all leaves, resets them, reuses the previous internal-node slots as the new internal nodes, and builds a fresh median-split tree bottom-up; `needsRebuild()` hints when height or the surface-area ratio (default threshold `8 * log2(n + 1)`) degraded too far.

**Static BVH** (`StaticBvh`). Built once from `SpatialItem`s (bounds + category mask + user payload). Splits pick the widest centroid axis; the `BinnedSah` strategy evaluates binned split costs on all three axes and accepts the cheapest partition unless either side would get fewer than a quarter of the items, in which case a deterministic median split (`nth_element`, ties broken by item index) is used. The structure never changes after build:

- `refit()` recomputes every node bound bottom-up for a wholesale bounds update.
- `refitItems()` applies per-item updates and refits only the affected root paths, deduplicating shared ancestors with generation stamps and processing them child-first (nodes are stored child-before-parent).
- `query()` is an iterative traversal with a fixed 128-entry stack; a tree deeper than 127 levels aborts the query with partial results rather than overflowing.

**Category masks.** Every node stores the OR of its subtree's category masks, so a query with a mask can skip whole subtrees with one AND — the same filtering applies again per item/proxy at the leaves. A query mask of 0 disables the query.

**Scene index** (`SceneSpatialIndex`). Holds one `StaticBvh` and one `DynamicAabbTree`. `query()` runs both traversals into caller-owned scratch buffers, then appends static hits (item index) followed by dynamic hits (proxy handle) to the output list, each tagged with `SceneMobility`. The returned `QueryStatistics` sums both traversals.

## Threading and lifetime

No class in this module contains locks or atomics. Each tree/index instance is single-threaded: one thread may query while nobody mutates, but concurrent mutation and querying of the same instance is undefined. Instances are plain C++ objects — create them where you use them, and destroy all proxies before destroying the tree if you want clean ownership (destroying the tree is safe at any time; outstanding handles simply dangle until `resolve()` is next called, at which point they fail). Item indices in `StaticBvh` stay valid across `refit`/`refitItems`; `build()` invalidates everything.

## Configuration

Not configurable. The module has no `config.toml`, reads no configuration keys, and has no kernel construct phase; tuning happens through the typed options structs (`DynamicAabbTreeOptions`, `StaticBvhBuildOptions`) passed to the classes.

## Integration

No events are published or subscribed (the module never touches the kernel event bus) and no permissions are declared or required.

Consumer usage — a static world plus moving characters, queried once per frame:

```cpp
#include <engine/spatial_index/spatial_index.hpp>

namespace spatial = engine::spatial;

spatial::SceneSpatialIndex scene;

// Static world geometry: one item per collider.
std::vector<spatial::SpatialItem> items = /* fill bounds/masks/payloads */;
scene.buildStatic(items, spatial::StaticBvhBuildOptions{.leafSize = 4,
    .strategy = spatial::StaticBvhSplitStrategy::BinnedSah, .bucketCount = 12});

// Dynamic objects: one proxy each; keep the handles.
const spatial::ProxyHandle player
    = scene.insertDynamic(playerBounds, userData, categoryMask);
if (player == spatial::invalidProxy) { /* invalid bounds or allocation failure */ }

// Each frame: move the proxy, then query an area.
scene.updateDynamic(player, newBounds, velocity);
std::vector<spatial::SceneSpatialHit> hits;
spatial::SceneSpatialQueryScratch scratch;
const spatial::QueryStatistics stats = scene.query(area, hits, scratch, categoryMask);
for (const spatial::SceneSpatialHit &hit : hits)
{
    if (hit.mobility == spatial::SceneMobility::Static)
        use(items[hit.staticIndex]);       // static item payload
    else
        use(scene.dynamicTree());          // or hit.dynamicHandle
}
scene.removeDynamic(player);

// Optional quality maintenance for the dynamic half:
if (scene.needsDynamicRebuild())
    scene.rebuildDynamic();
```

For a purely static scene, use `StaticBvh` directly (`build`, `query`, `item`); for purely dynamic sets, use `DynamicAabbTree` directly.
