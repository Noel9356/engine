# Optional planet demo assets

The planet demo remains runnable without this asset package. Without the
package, terrain uses its vertex colours and vegetation uses procedural
fallback sprites. No demo features are disabled.

To enable the photographic terrain and vegetation textures, extract
`engine_planet_assets.zip` into the directory containing the `engine/` folder.
The archive preserves the original path `engine/demos/planet/assets/`.

Extract the package before configuring CMake. If the project has already been
configured, re-run CMake configuration after extraction so the application
runtime staging step detects and copies the assets.

## Texture formats

The terrain and vegetation textures ship as KTX 2 files (`.ktx2`, Basis
UASTC with zstd supercompression and a full mip chain). The renderer loads
them through its KTX path; machines whose GPU samples BC formats natively
transcode to BC7 at load, and every other machine decodes on the CPU - no
separate asset variant is needed.
