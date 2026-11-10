# Game mods

A game mod is a directory under a configured mods directory containing `mod.toml`. The manifest declares the mod identity, engine API version, kind, entry file, requested capabilities, permissions, and optional configuration keys.

## Manifest

`mod.toml` uses TOML and is limited to 1 MiB. Manifest strings are limited to 4096 bytes and each string-array field accepts at most 256 non-empty entries. `id` contains 1–128 ASCII letters, digits, or underscores. `engine_api` is an unsigned 32-bit integer and must match `engineGameModInterfaceVersion`.

A native mod uses `type = "native"`; `entry` is a library base name without path separators. Its compiled module descriptor supplies the factory and runtime capabilities.

A script mod uses `type = "script"`; `entry` names a `.lua` file inside the package. It declares `host_capability`, which must also appear in `capabilities`. Script mods do not provide engine capabilities.

An optional `scene` names a file inside the package. `config_keys` uses `<section>.<dotted path>` names. Required and optional capabilities are listed separately; permissions are declared explicitly in `permissions`.

## Package paths

Script entry and scene paths are resolved relative to the package directory. Absolute paths and paths resolving outside the package are not accepted. Native library entries are names only; the loader applies the platform-specific library prefix and suffix.

## Script hosting

The script runtime creates a separate Lua state for each script mod. The manifest entry is executed during hosting, followed by optional `on_construct`, `on_start`, `on_frame`, `on_fixed_update`, `on_key_pressed`, and `on_shutdown` callbacks. Native functionality is exposed through explicitly registered capabilities and Lua bridges.
