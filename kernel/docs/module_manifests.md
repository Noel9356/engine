# Module manifests

A module may ship a `module.toml` sidecar beside its library. The kernel reads this descriptor during discovery without executing the library.

## Manifest fields

- `name`: required identifier containing ASCII letters, digits, or underscores.
- `version`: optional version string.
- `library`: optional library file name; path separators are not accepted.
- `abi`: unsigned 32-bit module ABI version; it must match `engineModuleAbiVersion`.
- `provides`, `requires`, `optional_capabilities`, `permissions`: optional arrays of non-empty strings.

## Limits

The manifest file is limited to 1 MiB. Individual string values are limited to 4096 bytes. Each string-array field accepts at most 256 non-empty entries. The descriptor cache is advisory, limited to 64 MiB, 65,536 records, 4096-byte lines, and 256 values per descriptor list.
