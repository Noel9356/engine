-- test_scene: the content package's script side.
--
-- The scene itself is data (scene.toml); this script only proves, in every
-- run that loads the package, that a script mod is discovered, hosted by the
-- provider of the manifest's host capability, and constructed after it. The
-- full scripting bridge (key events, screenshots, frame subscription) is
-- documented in mods/README.md.

engine.log_information("test_scene loaded: the demo world comes from scene.toml")
