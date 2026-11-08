# character lua bridge

The character module installs a `character.*` Lua function namespace into every script runtime the engine hosts. Scripts create and destroy assemblies, add, remove, and replace parts, connect them through validated connectors, drive global parameters, evaluate and validate, read instance transforms, and save or load versioned recipes. The bridge is optional: without `script_runtime` the C++ `character` capability works unchanged and scripts never see the `character` table.

## Installation

The module declares `optional_capabilities = ["script_runtime"]`. In `construct()` (source/character.cpp) it acquires the capability when present and registers `initializeCharacterLuaRuntime` (source/lua_bridge.cpp) through `registerRuntimeInitializer`. script_runtime runs the initializer once per runtime - including runtimes created before registration - and each call registers the twenty `character.*` native functions for that runtime. Unregistering at module shutdown stops future applications.

## Values and errors

- Assembly handles and instance ids are numbers; `0` means "none" and is never handed out. `character.create` returns `0` when it cannot create an assembly, and `character.add_part` returns `0` when it cannot add the part.
- Numbers must be finite. A malformed call (wrong argument count, wrong types) fails the call instead of the engine: it returns `false` (or no values) and leaves assembly state untouched.
- Functions that report a matrix or bounds append a trailing `ok` boolean; see the reference below for the exact shapes.
- The underlying capability reports errors through `CharacterError`; the bridge flattens them to the return conventions above. `character.last_error` returns the module's last error text (or nil) for diagnostics.

## Function reference

Functions are listed alphabetically.

| Function | Arguments | Returns | Notes |
| --- | --- | --- | --- |
| `character.add_part` | handle, part_id, instance_name | instance id (0 on failure) | Instance names are unique per assembly. |
| `character.bounds` | handle | cx, cy, cz, radius, ok | Union of the transformed part bounds after a successful `evaluate`. |
| `character.connect` | handle, parent, parent_connector, child, child_connector | ok | Establishes the structural parent; rejects cycles and incompatible role pairs. The authored offset correction stays identity; animate attachments through `set_connection_offset`. |
| `character.create` | profile_or_nil | handle (0 on failure) | Instantiates a registered profile's default recipe, or an empty assembly for nil. |
| `character.destroy` | handle | (none) | |
| `character.disconnect` | handle, child | ok | |
| `character.evaluate` | handle | ok | Validates first (unless `character.validate_on_evaluate` is disabled) and recomputes rig poses, connector-aligned transforms, and bounds. |
| `character.get_parameter` | handle, name | value, ok | |
| `character.instance_count` | handle | count | |
| `character.instance_id` | handle, index | instance id (0 when out of range) | `index` is 1-based; pairs with `instance_count` to enumerate an assembly. |
| `character.instance_info` | handle, instance | name, part_id, parent, ok | `parent` is `0` for the assembly root. |
| `character.instance_transform` | handle, instance | m00..m32 | The 16 world-matrix values in column-major order (`m00` first, translation in elements 13..15). Returns no values when the instance has no evaluated transform - script_runtime caps native results at 16, so there is no trailing flag. |
| `character.last_error` | (none) | message or nil | Module-level error text (recipe parse failures, I/O errors); nil when nothing failed. |
| `character.load` | handle, path | ok | Replaces the assembly's content with a saved recipe file (format 1). |
| `character.remove_part` | handle, instance | ok | Detaches and removes the instance and its connections. |
| `character.replace_part` | handle, instance, part_id | ok | Swaps the asset, keeping the instance name and connections; validation reports missing connectors afterwards. |
| `character.save` | handle, path | ok | Writes the versioned recipe (parts, attachments with offsets, root placement, parameters). |
| `character.set_connection_offset` | handle, child, x, y, z | ok | Authored correction on the child's incoming connection; per-frame animation drivers go through this (rotation offsets are not exposed to Lua). |
| `character.set_parameter` | handle, name, value | ok | Clamped to the parameter's registered range. |
| `character.validate` | handle | issue_count | Runs the validation pass without evaluating. |

## Limits

- script_runtime caps apply per runtime: at most `script_runtime.maximum_native_functions` (default 256) registered functions - the bridge registers 20 - at most `maximum_native_arguments` (default 32) per call (the widest is `character.connect` with 5) and at most `maximum_native_results` (default 16) per call (the widest is `character.instance_transform` with exactly 16).
- Bridge calls serialize on the bridge context mutex, so script calls are orderly among themselves; direct C++ capability calls must still follow the module's single-thread rule (see [overview.md](overview.md), "Threading and lifetime").
- Budgets (`character.maximum_characters`, `maximum_parts_per_character`, ...) apply to script calls exactly as to capability calls; over-budget calls fail with `0` / `false`.

## Example

```lua
-- Build and inspect a two-part character (the demo content registry must be
-- registered first; the character_lab mod defers to the first frame).
local assembly = character.create(nil)
local torso = character.add_part(assembly, "torso_humanoid", "torso")
local head  = character.add_part(assembly, "head_dragon", "head")
character.connect(assembly, torso, "neck.base", head, "head.root")
character.set_parameter(assembly, "body.height", 1.2)
character.evaluate(assembly)
for index = 1, character.instance_count(assembly) do
    local id = character.instance_id(assembly, index)
    local name, part, parent = character.instance_info(assembly, id)
    local m00 = character.instance_transform(assembly, id)
    print(name, part, parent, m00)
end
character.save(assembly, "my_character.toml")
character.destroy(assembly)
```
