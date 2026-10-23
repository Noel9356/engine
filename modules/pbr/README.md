# pbr module

The analytic direct-lighting technique. The pbr module attaches a compute pass to the renderer through the technique interface (`modules/renderer/render_interface.hpp`), owns the scene light state behind the `LightingCapability` (capability name `lighting`, published in `lighting_capability.hpp` next to this module — the light-state vocabulary travels with its provider), and writes the DIRECT radiance layer the composite adds. It is named after its algorithm — physically based rendering direct transport — because technique modules are named after algorithms, never after rendering categories (`docs/architecture.md`).

## The LightingCapability

The pbr module is the capability's provider: the sun is set with `setSunLight`/`clearSunLight`/`hasSunLight`/`sunLight(const)`, and point lights are managed with `addPointLight`/`removePointLight`/`clearPointLights`/`pointLightCount`/`pointLight(index)`. The state is consumed once per frame by the lighting passes; `[scene.sun]` and `[[scene.point_lights]]` in the configuration only SEED it. Live point lights are capped at 16; `addPointLight` returns the invalid handle when the cap is reached, so a caller can react instead of silently overwriting an existing light. Light descriptions are plain primitive values (directions, colors, radii), so the published header stays implementation-, platform- and vendor-agnostic like every other capability surface. C++ mods acquire this capability to drive lights at runtime; script mods interact with lighting through configuration today, because the Lua bridge grows deliberately.

## The direct pass

`shaders/pbr_lighting.comp` evaluates Cook-Torrance GGX for every G-buffer pixel against the sun and all point lights, with analytic SDF soft shadows: shadow rays sphere-trace the same analytic scene SDF the renderer rasterizes against (one geometry truth — the shared parameterization in `scene_common.glsl`), and the penumbra factor shapes the soft shadow width. The dark-side details that make it robust:

- Shadow rays leave the surface through a guarded origin: the G-buffer normal carries the normal map, and when the offset point sits inside the analytic SDF the origin re-offsets along the SDF's geometric normal — a normal-mapped normal pushed origins inside the geometry and self-shadowed whole patches until the guard went in.
- Point-light shadow rays terminate AT the light. A radius-cutoff bound let them fly past the light and hit whatever sat behind it, carving phantom shadow disks out of surfaces beyond the light.
- When the run also demands `indirect_specular`, this pass dims its analytic specular by the same Fresnel weight the reflection layer carries, so the two passes SPLIT the specular energy instead of drawing the sun twice. The hand-over weight is the shared `[lighting].max_roughness` key, read by both layers so the two sides of the hand-off can never drift apart.
- The relight of first-bounce hit points (shared with GI and reflections through `surfaceOutgoingRadiance` in `relight_common.glsl`) stores physically scaled LAMBERTIAN radiance, so every stage of the transport chain carries the same energy unit.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `lighting`, `direct_diffuse` |
| required capabilities | `logging`, `renderer` |
| required permissions | `capability.logging`, `capability.renderer` |

The module provides two capabilities: `lighting` (the scene light state every lighting pass reads) and `direct_diffuse` (its radiance layer). It is activated exactly when an active mod demands one of them. A run that demands only `lighting` gets the state with an idle pass; demanding `direct_diffuse` attaches the analytic pass.

## Configuration

The sun and point lights are seeded from `[scene.sun]` and `[[scene.point_lights]]` (see `docs/configuration.md`, section "[scene]").

| Key | Type | Default | Meaning |
|---|---|---|---|
| `shadows` | bool | `true` | analytic SDF soft shadows for sun and point lights |
| `penumbra` | float | `12.0` | soft shadow penumbra factor (higher = sharper) |
| `shadow_steps` | integer | `24` | sphere-tracing steps per shadow ray. Keep this value in step with the GI module's `shadow_steps` — an under-budgeted relight shadow ray returns partial light inside sealed rooms and the probe grid amplifies the error into blocky patches |

## Files

- `lighting_capability.hpp` — the published capability surface every consumer includes
- `pbr.hpp` / `pbr.cpp` — the technique module and the `LightingCapability` provider
- `light_state.hpp` — the scene light state behind the capability
- `shaders/pbr_lighting.comp` — the direct-lighting compute pass
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_PBR`)
