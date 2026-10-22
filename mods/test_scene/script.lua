-- test_scene: the content package's script side.
--
-- The scene itself is data (scene.toml); this script only proves, in every
-- run that loads the package (the demo, the headless smoke and the leak
-- test), that a script mod is discovered, hosted by lua_scripting and
-- constructed after it. The full scripting bridge (key events, screenshots,
-- frame subscription) is documented in mods/README.md and exercised by the
-- QA packages under scripts/qa/.

engine.log_information("test_scene loaded: the demo world comes from scene.toml")
