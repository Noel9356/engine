# test_scene — the content mod

The one mod package the engine ships. It owns the demo world end to end: the scene definition (`scene.toml`), the PBR texture sets the scene references (`assets/textures/<set>/`), the `[renderer].texture_directory` default that points the renderer at them (`config.toml`), and a one-line script (`script.lua`) that keeps the script-mod discovery path exercised in every run. The engine itself ships no content — deleting this package leaves a working engine that renders whatever the configuration defines (an assembly `[scene]`, a `--scene` file, or an empty sky).

The package is a script mod hosted by `lua_scripting` because the content needs no native code; the scene is data, not behavior. The manifest's `scene = "scene.toml"` key is what makes this the *world provider*: the kernel applies the file wholesale when no `--scene` flag is given and the configuration defines no `[scene]` of its own (precedence and the two-content-mods error are documented in `mods/README.md`).

Texture sets are CC0 from ambientCG (1K): `Bricks070`, `Concrete034`, `Metal049A`, `Plaster001`, `WoodFloor055`. Each is a folder with `Color.png`, `NormalGL.png`, `Roughness.png` and optional `Metalness.png`; all sets used by one scene must share one resolution (they upload into shared texture arrays), and every set here is 1024x1024.
