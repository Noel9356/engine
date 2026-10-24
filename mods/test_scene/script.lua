-- test_scene: the content package's script side.
--
-- The scene itself is data (scene.toml); this script only proves, in every
-- run that loads the package, that a script mod is discovered, hosted by the
-- provider of the manifest's host capability, and constructed after it.
-- The runtime-spawn below additionally exercises the scene capability: one
-- emissive sphere joins the configured world at startup, and the renderer's
-- incremental rebuild makes it visible from the first frame. The full
-- scripting bridge (input polling, scene objects, lights, camera, timers)
-- is documented in mods/README.md.

engine.log_information("test_scene loaded: the demo world comes from scene.toml")

-- Spawn a small glowing orb above the center table: exercises the scene
-- registry end to end (spawn -> event -> renderer rebuild).
local orb = engine.spawn_object({
    type = "sphere",
    position = { 0.0, 1.9, -0.4 },
    radius = 0.12,
    albedo = { 1.0, 0.85, 0.4 },
    roughness = 0.4,
    metallic = 0.0,
    emissive = 6.0,
})
if orb ~= 0 then
    engine.log_information("test_scene: spawned demo orb (handle " .. orb .. ")")
    -- A gentle ping-pong bob: exercises the animation capability end to end
    -- (tween -> scene registry -> renderer incremental rebuild).
    engine.animate_object(orb, {
        channel = 0,                      -- position
        from_position = { 0.0, 1.8, -0.4 },
        to_position   = { 0.0, 2.2, -0.4 },
        duration = 2.0,
        easing = 3,                       -- ease in/out
        mode = 2,                         -- ping-pong
    })
else
    engine.log_warning("test_scene: scene registry full, demo orb skipped")
end

-- A dropped ball under the builtin physics provider: falls, bounces on the
-- floor, rolls to rest. Exercises physics -> scene registry -> renderer.
local ball = engine.spawn_object({
    type = "sphere",
    position = { 1.4, 2.5, 0.6 },
    radius = 0.25,
    albedo = { 0.85, 0.3, 0.2 },
    roughness = 0.35,
    metallic = 0.1,
})
if ball ~= 0 then
    local body = engine.add_physics_body(ball, { restitution = 0.55 })
    engine.set_body_velocity(body, 0.4, 0.0, -0.1)
    engine.log_information("test_scene: dropped physics ball (handle " .. ball .. ", body " .. body .. ")")
end

-- The Stanford dragon: the mesh pipeline's real-geometry proof. The scan
-- (The Stanford 3D Scanning Repository, Stanford University Computer
-- Graphics Laboratory) ships in this package as an embedded-buffer glTF;
-- full-mesh import registers it as a registry mesh (deterministic name
-- "Dragon#n0p0") and spawns the mesh object at the node's placement -
-- 1.15 m tall, feet on the floor, beside the metal sphere. The analytic
-- consumers (GI, reflections, shadows) see the mesh through its documented
-- bounding-box proxy.
local dragonSpawned = engine.import_gltf("mods/test_scene/assets/models/dragon.gltf", "mesh")
if dragonSpawned > 0 then
    engine.log_information("test_scene: imported the Stanford dragon ("
        .. dragonSpawned .. " object, mesh Dragon#n0p0)")
else
    engine.log_warning("test_scene: dragon import failed or the asset_gltf module is absent")
end
