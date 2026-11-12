# character module

character provides data-driven character customisation for the engine: a content registry of body part assets (connectors, parameter bindings, material slots, tags, bounds), independent rig fragments, connector compatibility rules, and assembly profiles, plus a validated attachment-tree assembly runtime. Parts attach through connector-aligned transforms (parameter scale rides on socket positions, never on the inherited frames), global parameters deform parts through per-asset bindings, and assemblies save and reload as versioned TOML recipes. A Lua bridge (`character.*`) exposes the validated operations to plugins. Defaults for new assemblies come from the `[character]` configuration section and hot-reload when the kernel publishes a configuration-reloaded event.

## Capability

The module provides the `character` capability (ABI version constant `characterCapabilityAbi = 1`, see `include/engine/character/capabilities.hpp`). Assembly handles and instance ids are `std::uint32_t`; `0` (`invalidCharacter` / `invalidInstance`) means "none" and handles are never reused. Most entries report errors through an optional `CharacterError*` out-parameter; queries return plain values.

Entry groups: content registration (`registerParameter`, `registerCompatibilityRule`, `registerRig`, `registerPart`, `registerProfile`, `unregisterPart`, `parameterCount`/`parameterInfo`, `partCount`/`partId`), assembly instances (`createAssembly`, `destroyAssembly`, `addPart`, `removePart`, `replacePart`, `connect`, `disconnect`, `setAssemblyTransform`, `setConnectionOffset`), parameters (`setParameter`, `getParameter`), evaluation (`evaluate`, `validate`, `instanceCount`, `instanceId`, `instanceInfo`, `instanceTransform`, `bounds`, `statistics`), and serialization (`saveRecipe`, `loadRecipe`, `lastErrorMessage`).

## Provided / required / optional capabilities

- `module.toml`: `provides = ["character"]`, `requires = []`, `optional_capabilities = ["script_runtime"]`.
- Entry table (bottom of `source/character.cpp`): `providedCapabilities = {"character"}`, `requiredCapabilities = nullptr` (count 0), `optionalCapabilities = {"script_runtime"}` (count 1), `declaredPermissions = nullptr` (count 0).

The module optionally acquires `script_runtime` to register its Lua bridge. The character capability remains usable without a scripting backend and declares no permissions.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (14 keys). Read by kernel module discovery. |
| `config.toml` | Default values for the `[character]` configuration section (see [configuration.md](configuration.md)). |
| `include/engine/character/capabilities.hpp` | Public capability types: ABI, handles, error codes, anchor spaces, binding targets, asset definitions, validation codes, change events, function-pointer table. |
| `source/character.cpp` | The implementation: content registries, assembly records, the mandatory validation pass, connector-aligned evaluation, parameter application, recipe serialization, configuration and hot-reload, capability binding, module entry table. |
| `source/character_module.hpp` | Internal records (`PartRecord`, `RigRecord`, `CharacterRecord`, budgets) shared between the implementation and the Lua bridge. |
| `source/lua_bridge.cpp`, `source/lua_bridge.hpp` | Optional `script_runtime` bridge over the validated operations (`character.*`, see [lua_bridge.md](lua_bridge.md)). |
| `docs/` | This documentation. |

## How it works

- **Content registry.** Parts, rigs, parameters, compatibility rules, and profiles are copied into module-owned records at registration (strings included; callers may release their memory afterwards). Registration validates references immediately: parameter bindings must name registered parameters, joint-translation targets must exist in the part's rig, connector-offset targets must exist on the same part, and compatibility rules need a valid role pair and scale range. Every registry is capped by its `[character]` budget.
- **Assemblies.** `createAssembly` instantiates a profile's default recipe (or an empty assembly for a null profile). Instances keep a full snapshot of their part definition, so unregistering a part never corrupts live assemblies. `connect` establishes the single structural parent of a child and rejects cycles immediately; `replacePart` swaps the asset of an existing instance, keeping its name and connections (validation reports missing connectors afterwards).
- **Validation.** `evaluate()` validates whenever the validated surface changed (structure or parameters): unknown assets or connectors, duplicate instance names, multiple parents, missing/multiple roots, cycles, connector capacity, role compatibility (exact rule pair, then wildcard), relative-scale range, required tags, joint references, parameter ranges, and non-finite transforms. An invalid assembly never becomes active; `validate()` returns the same issues on demand. A pure transform refresh (per-frame animation offsets) reuses the last issue list instead of re-running the pass.
- **Evaluation.** Per part, parameter bindings resolve the instance scale (`S`), and the part's rig resolves its joint pose. The attachment walk then composes rigid child frames from connector sockets: the parent socket's world position is lifted with the parent's parameter scale (deformed geometry lives in the scaled space, so anchors follow it), rotations stay rigid (parameter scale never shears attachment frames), and the connection offset sits between the two socket frames. `M_instance_world = T_instance_rigid * S_parameters` is what `instanceTransform` returns and what consumers draw. The walk is O(parts + connections) over child adjacency that is rebuilt only after structure changes, and `evaluate()` early-outs entirely when no dirty flag is set.
- **Parameters.** One global parameter registry; every part maps parameters onto its own targets. Transform-level targets only: part scale per axis, joint rest translation (connector anchors follow the joint), and connector offset translation. Scale bindings multiply (`1 + (value - default) * scale`), translation bindings add.
- **Serialization.** `saveRecipe` writes format version 1 (parts, structural attachments with offsets, root placement, parameter values); `loadRecipe` accepts the same version, rejects anything newer, and replaces the assembly's content after validation.
- **Change events.** Mutations publish `CharacterAssemblyChangedEvent` (structure / parameters / offsets / evaluated) on the engine event bus so consumers can invalidate only what is affected. An `evaluate()` that only refreshed transforms publishes nothing - the offset change that made it necessary already published `OffsetsChanged` - so per-frame animation does not spam the bus.

## Transform model

Column vectors, column-major matrices, right-handed world space:

```
T_child_world    = T_socket(parent, scale-applied) * T_offset * inverse(T_socket(child, scale-applied))
M_instance_world = T_instance_rigid * S_parameters
```

`T_offset` is the authored correction on the connection, not an independent attachment transform: an attached part derives its entire transform from its connector relationship. Socket positions carry the owning part's parameter scale; socket rotations and the inherited frames stay rigid, so a scaled torso drags its head, arms, and wings with it without shearing them.

### Authoring rule: one driver per deformation, one source of truth per junction

Because socket positions are scaled by the owning part's parameter scale *after* the rig pose and anchor offset are resolved, every anchor already follows the part scale. Content must therefore author anchors and rig joints at the **undeformed rest positions** and use `JointTranslation` / `ConnectorOffsetTranslation` bindings only for shifts the affine part scale cannot express. A binding that re-tracks what the scale already applies (for example a neck joint shifted up in step with `body.height` while the anchor rides the torso scale) moves that attachment twice and opens a gap that grows with the parameter - the classic "parts drift apart at non-default parameter values" bug.

When a part deforms by baked mesh morphs instead of part scale (the demo's torso and heads), the same rule takes a stronger form - the **junction contract** documented in [customization_design.md](customization_design.md): every interface ring away from a part's origin is moved by exactly one mechanism, and the mesh morph's interface deltas and the anchor bindings are derived from one shared set of constants (the generated metrics header) so geometry and anchors cannot drift apart. Parts whose only ring sits at their origin (limbs, tails, wings) may deform by plain part scale - the ring cannot leave its anchor.

## Deliberate limits in this engine build

- **No GPU skinning / morph targets on the renderer**: the renderer has no joint-palette or morph path (the glTF importer rejects skinned meshes), so this module's own deformation is transform-level only. Mesh-space morph targets live in the content layer: a consumer may apply baked per-vertex morph deltas when parameters change and re-upload the mesh (the demo does exactly this - see [customization_design.md](customization_design.md)). Parts render as independently drawn components (mesh merging is an optional optimisation this module does not perform).
- **No physics**: there is no physics module to consume collision data, so collision profiles are not part of the data model yet.
- **No retargeting yet**: rigs expose semantic joint ids; retarget profiles belong with an animation integration.

## Lua bridge

When `script_runtime` is present, each Lua runtime receives the `character.*` functions: `create`, `destroy`, `add_part`, `remove_part`, `replace_part`, `connect`, `disconnect`, `set_parameter`, `get_parameter`, `set_connection_offset`, `evaluate`, `validate`, `instance_count`, `instance_id`, `instance_info`, `instance_transform`, `bounds`, `save`, `load`, and `last_error`. The bridge serializes its own calls, but direct C++ capability calls must still obey the module's single-thread rule. The complete function reference lives in [lua_bridge.md](lua_bridge.md).

## Threading and lifetime

Capability and Lua bridge entry points serialize on one module mutex; the expected caller is the main/simulation thread. Handles stay valid until `destroyAssembly`; instance ids stay stable across parameter, transform, and offset changes. Registry strings are copied at registration, so callers may free their definitions immediately.

## Reference demo

The engine demo ships a complete character customisation screen (`--view character`, the default) built on this capability: an orbit-camera viewport with appendage animation drivers, and a panel that swaps part variants (nine heads, five tail chains, two wing styles), edits the global parameters (thirteen, including the `neck.length`, `tail.curl`, and `skin.tone` axes), lists validation issues, and saves/loads recipes. Its content layer (`demos/character_customisation/source/demo/character_content.cpp`) registers a hybrid-biped profile (humanoid torso, humanoid arms and legs, two procedural wing variants, a procedural reptile tail chain, nine swappable heads) and demonstrates the junction contract described above. Part meshes are authored by `scripts/build_character_assets.py` into `demos/character_customisation/assets/character_parts/` (see the LICENSE.md there); the Quaternius-sourced torso/arm/leg surfaces are de-slivered pack anatomy, and every head, tail, and wing plus the neck transition piece is procedural.

For automated verification the screen exposes two flags: `--character-test` runs the design document's assembly test operations (head swap, leg proportions, every head/tail variant swap, the full widened-range extreme sweep, second tail with straddled symmetric spread, wing removal, torso width, independent tail animation, save/reload, capacity and compatibility enforcement, junction contract at parameter extremes) and prints a PASS/FAIL report; `--character-drive <preset>` applies a named configuration (`human`, `dragon`, `serpent`, `orc`, `yeti`, `alien`, `bluedemon`, `skull`, `spadetail`, `clubtail`, `fangtail`, `fantail`, `curltail`, `featherwings`, `tall`, `broad`, `longtail`, `compact`, `noappend`) to the visible character, which combined with `--screenshot` captures the preset for visual regression checks. `--camera-yaw`, `--camera-pitch`, `--orbit-distance`, and `--hide-panel` make those captures deterministic.

The design rationale - the industry research behind the junction contract, the morph-based deformation split, and the software-rasterizer performance model - lives in [customization_design.md](customization_design.md).
