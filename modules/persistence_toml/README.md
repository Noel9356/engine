# persistence_toml module - the scene registry's save/load (TOML)

## What this module provides

Capability `persistence` (implementation-bound member `persistence.toml`, published in `persistence_capability.hpp`): the runtime save/load for the scene registry. One call writes every live object to a TOML file; one call spawns the rows of a file back as new registry objects. The registry stays the single source of truth - a save captures exactly what the renderer renders, physics steps and scripts address at the moment of the call.

```lua
engine.save_scene("mods/my_mod/saves/checkpoint.toml")   -- every live object to one file
engine.load_scene("mods/my_mod/saves/checkpoint.toml")   -- spawn the rows back
engine.clear_scene()                                     -- remove every live object (meshes stay)
```

## The file format

One `[[objects]]` array of tables, one row per object, with the SAME field names the `[scene.objects]` schema uses - a save file reads as documentation and a hand-tuned save behaves like a hand-tuned scene:

```toml
saved_objects = 2

[[objects]]
type = "sphere"                  # box | sphere | mesh
position = [1.4, 2.5, 0.6]
size = [0.25, 0.25, 0.25]        # half extents / radius in x / mesh bbox
rotation_y = 0.0
albedo = [0.85, 0.3, 0.2]
roughness = 0.35
metallic = 0.1
emissive_strength = 0.0
uv_scale = 1.0
# texture = "Metal049A"          # present only when the object references a set

[[objects]]
type = "mesh"
position = [0.0, 1.0, 0.0]
size = [0.8, 1.0, 1.1]           # the bbox half extents (the tracing proxy)
mesh = "Dragon#n0p0"             # the registry mesh NAME (see below)
```

## Handles are runtime values; meshes are saved by name

Registry handles are per-run values, so a file never carries them: loading spawns NEW objects with fresh handles (the count is what the caller checks; the registry's own precise-capacity refusal applies). Mesh objects reference their registry mesh by NAME: loading requires the named meshes to be registered at load time - the same run, or an import that ran first (glTF imports register meshes under deterministic names and registration is idempotent by name, so re-importing the same file before a load restores every mesh reference exactly). A row naming an unregistered mesh is skipped with one precise log; the count reflects what arrived.

`clear_scene()` removes every live object but leaves the mesh registry untouched: meshes are assets, not instances, and the load that follows usually re-references them.

## Permissions

The module declares `filesystem.write` (save) and `filesystem.read` (load) plus the derived capability grants; both file permissions are deniable in configuration like every grant. A save into a directory the run may not write fails with a precise message, never a partial file.

## Configuration

None. Paths resolve relative to the executable directory (the engine's path convention), so mod-internal saves are addressed as `mods/<id>/saves/<file>.toml` from scripts.

## Files

- `persistence_capability.hpp` - the published capability surface
- `persistence.hpp` / `persistence.cpp` - the module, the TOML writer/reader
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_PERSISTENCE_TOML`)
