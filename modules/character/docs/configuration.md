# character configuration

All keys live in the `[character]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[character]` keys are silently accepted.

Every value is read once per `readConfiguration()` (in `construct()`, and again whenever a `ConfigurationReloadedEvent` touches `character.*` keys). A value outside its documented range is ignored - the built-in default stays active; nothing is clamped. Keys are listed alphabetically.

| Key | Type | Valid range (validated in `readConfiguration()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `character.maximum_bindings_per_part` | integer | 1 - 65536 | `64` | Maximum parameter bindings accepted per registered part. |
| `character.maximum_characters` | integer | 1 - 65536 | `64` | Maximum simultaneously live assemblies. Existing assemblies survive a lower hot-reloaded limit; new ones are refused until the live count drops below it. |
| `character.maximum_connectors_per_part` | integer | 1 - 65536 | `32` | Maximum connectors accepted per registered part. |
| `character.maximum_connections_per_character` | integer | 1 - 65536 | `512` | Maximum attachments per assembly. |
| `character.maximum_joints_per_rig` | integer | 1 - 65536 | `256` | Maximum joints accepted per registered rig fragment. |
| `character.maximum_material_slots_per_part` | integer | 1 - 65536 | `8` | Maximum material slots accepted per registered part. |
| `character.maximum_parameters` | integer | 1 - 65536 | `256` | Maximum registered global parameters. |
| `character.maximum_parts` | integer | 1 - 65536 | `256` | Maximum registered body part assets. |
| `character.maximum_parts_per_character` | integer | 1 - 65536 | `256` | Maximum part instances per assembly. |
| `character.maximum_profiles` | integer | 1 - 65536 | `64` | Maximum registered assembly profiles. |
| `character.maximum_rigs` | integer | 1 - 65536 | `128` | Maximum registered rig fragments. |
| `character.maximum_rules` | integer | 1 - 65536 | `256` | Maximum registered compatibility rules. |
| `character.maximum_tags_per_part` | integer | 1 - 65536 | `16` | Maximum tags accepted per registered part. |
| `character.validate_on_evaluate` | boolean | any | `true` | When true, `evaluate` runs the full validation pass before activating an assembly whenever its structure or parameters changed, and refuses to activate an invalid one; when false, evaluation never validates (the assembly must be validated through `validate` instead). A transform-only refresh (per-frame animation offsets) never re-validates under either setting. |

## Out-of-range / invalid values

Each count key is validated with an explicit upper bound (65536, except `maximum_connections_per_character`, which allows up to 262144); failures leave the built-in default in place silently. Lowering a budget hot-reloads cleanly: content that already exists is kept, and only new registrations beyond the reduced budget are refused.

Note that these are module-wide budgets, not per-assembly values; a hot reload refreshes them, but assemblies that already exist keep their data.
