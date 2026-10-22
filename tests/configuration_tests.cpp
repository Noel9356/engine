// Layered configuration tests: the store's layer stack (assembly, shipped
// defaults, scene swap, command line overrides), the provenance records and
// the known-key warnings, plus the kernel-level integration of all of them.
// Everything here runs without a window or a Vulkan instance.

#include <catch2/catch_test_macros.hpp>

#include "engine/kernel/configuration.hpp"
#include "engine/kernel/engine_kernel.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{

std::filesystem::path writeTextFile(const std::filesystem::path& path, const std::string& contents)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::trunc);
    file << contents;
    return path;
}

std::size_t uniqueFileCounter()
{
    static std::size_t counter = 0;
    return ++counter;
}

std::filesystem::path uniqueTempPath(const std::string& prefix)
{
    return std::filesystem::temp_directory_path() /
           (prefix + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
            std::to_string(uniqueFileCounter()));
}

bool contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

} // namespace

// ---- the store's layer stack -------------------------------------------------

TEST_CASE("the assembly layer loads and records its provenance", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml",
                                                     "[window]\ntitle = \"assembly\"\nwidth = 800\n");
    REQUIRE(store.loadFromFile(path, error));
    REQUIRE(store.isLoaded());
    REQUIRE(store.stringValue("window", "title") == "assembly");

    const std::optional<engine::ConfigProvenance> titleProvenance = store.provenanceFor("window", "title");
    REQUIRE(titleProvenance.has_value());
    REQUIRE(titleProvenance->layer == engine::ConfigProvenance::Layer::EngineConfig);
    REQUIRE(titleProvenance->source == path.string());

    // Every provided leaf is tracked; the report names layer and source.
    const std::string report = store.provenanceReport();
    REQUIRE(contains(report, "window.title <- engine configuration"));
    REQUIRE(contains(report, "window.width <- engine configuration"));
}

TEST_CASE("assembly parse errors stay precise", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path =
        writeTextFile(uniqueTempPath("engine_config_test_") / "broken.toml", "[window\ntitle = \n");
    REQUIRE_FALSE(store.loadFromFile(path, error));
    REQUIRE(contains(error, "configuration parse error"));
    REQUIRE(contains(error, "line "));
    REQUIRE_FALSE(store.isLoaded());
}

TEST_CASE("shipped defaults fill gaps and never override the assembly", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path assemblyPath = writeTextFile(
        uniqueTempPath("engine_config_test_") / "engine.toml",
        "[window]\ntitle = \"assembly\"\n");
    REQUIRE(store.loadFromFile(assemblyPath, error));

    // A defaults file providing existing keys, new keys and a whole section.
    // [scene] is deliberately absent: world content has a dedicated channel
    // (the assembly's [scene] or a mod's declared scene file) and a defaults
    // file that defines it is rejected - see the authority test below.
    const std::filesystem::path defaultsPath = writeTextFile(
        uniqueTempPath("engine_config_test_") / "config.toml",
        "[window]\nwidth = 640\ntitle = \"from defaults\"\n"
        "\n[example_mod]\nlevel = 3\n");
    REQUIRE(store.mergeDefaultsFile(defaultsPath, engine::ConfigProvenance::Layer::ModDefaults, "example_mod",
                                    error));

    // The assembly wins wherever it spoke; defaults only fill the gaps.
    REQUIRE(store.stringValue("window", "title") == "assembly");
    REQUIRE(store.integerValue("window", "width") == 640);
    REQUIRE(store.integerAtPath("example_mod", "level") == 3);

    // Provenance tells the two apart.
    REQUIRE(store.provenanceFor("window", "title")->layer == engine::ConfigProvenance::Layer::EngineConfig);
    REQUIRE(store.provenanceFor("window", "width")->layer == engine::ConfigProvenance::Layer::ModDefaults);
    REQUIRE(store.provenanceFor("window", "width")->source == "example_mod");
}

TEST_CASE("the first defaults file wins when two defaults provide one key", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path first = writeTextFile(uniqueTempPath("engine_config_test_") / "a.toml",
                                                      "[shared]\nvalue = \"module\"\n");
    const std::filesystem::path second = writeTextFile(uniqueTempPath("engine_config_test_") / "b.toml",
                                                       "[shared]\nvalue = \"mod\"\nother = \"mod only\"\n");
    REQUIRE(store.mergeDefaultsFile(first, engine::ConfigProvenance::Layer::ModuleDefaults, "module_a", error));
    REQUIRE(store.mergeDefaultsFile(second, engine::ConfigProvenance::Layer::ModDefaults, "mod_b", error));
    REQUIRE(store.stringValue("shared", "value") == "module");
    REQUIRE(store.provenanceFor("shared", "value")->layer == engine::ConfigProvenance::Layer::ModuleDefaults);
    REQUIRE(store.stringValue("shared", "other") == "mod only");
}

TEST_CASE("shipped defaults may not define assembly or host sections", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;

    SECTION("[engine] belongs to the assembly")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml",
                                                         "[engine]\nmodules = [\"evil\"]\n");
        REQUIRE_FALSE(store.mergeDefaultsFile(path, engine::ConfigProvenance::Layer::ModDefaults, "package", error));
        REQUIRE(contains(error, "must not define the 'engine' section"));
    }
    SECTION("[diagnostics] belongs to the host")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml",
                                                         "[diagnostics]\nlatency_report = true\n");
        REQUIRE_FALSE(store.mergeDefaultsFile(path, engine::ConfigProvenance::Layer::ModDefaults, "package", error));
        REQUIRE(contains(error, "must not define the 'diagnostics' section"));
    }
    SECTION("[scene] belongs to the assembly or a mod's scene file")
    {
        // World content has a dedicated channel. A defaults file defining
        // [scene] would drip its keys into every configuration that does not
        // define the same keys - including sealed-room test configurations
        // whose zero-sky guarantee depends on an untouched scene.
        const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml",
                                                         "[scene]\nsky_radiance = [0.1, 0.1, 0.1]\n");
        REQUIRE_FALSE(store.mergeDefaultsFile(path, engine::ConfigProvenance::Layer::ModDefaults, "package", error));
        REQUIRE(contains(error, "must not define the 'scene' section"));
        REQUIRE(contains(error, "scene file"));
    }
    SECTION("bare top-level values are not sections")
    {
        const std::filesystem::path path =
            writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml", "loose = 1\n");
        REQUIRE_FALSE(store.mergeDefaultsFile(path, engine::ConfigProvenance::Layer::ModDefaults, "package", error));
        REQUIRE(contains(error, "must only define sections"));
    }
    SECTION("only the defaults layers may merge defaults")
    {
        const std::filesystem::path path =
            writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml", "[window]\nwidth = 1\n");
        REQUIRE_FALSE(store.mergeDefaultsFile(path, engine::ConfigProvenance::Layer::EngineConfig, "x", error));
    }
}

TEST_CASE("the scene file replaces the whole [scene] section", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path assemblyPath = writeTextFile(
        uniqueTempPath("engine_config_test_") / "engine.toml",
        "[window]\ntitle = \"assembly\"\n"
        "\n[scene]\nsky_radiance = [1.0, 1.0, 1.0]\n"
        "\n[[scene.objects]]\ntype = \"box\"\nposition = [0.0, 0.0, 0.0]\n");
    REQUIRE(store.loadFromFile(assemblyPath, error));

    const std::filesystem::path scenePath =
        writeTextFile(uniqueTempPath("engine_config_test_") / "scene.toml",
                      "[scene.camera]\nfov = 100.0\nposition = [1.0, 2.0, 3.0]\n"
                      "\n[[scene.objects]]\ntype = \"sphere\"\n");
    REQUIRE(store.applySceneFile(scenePath, error));

    // Wholesale swap: the scene file's world is the whole world. Assembly
    // scene keys do not leak in, and the scene file's own object list is the
    // object list - no merge with the demo room.
    REQUIRE(store.floatingAtPath("scene", "camera.fov") == 100.0);
    REQUIRE(store.arraySizeAtPath("scene", "objects") == 1);
    REQUIRE(store.stringAtPath("scene", "objects[0].type") == "sphere");
    REQUIRE(store.floatingArrayAtPath("scene", "sky_radiance").empty());
    REQUIRE(store.stringValue("window", "title") == "assembly"); // untouched

    REQUIRE(store.provenanceFor("scene", "camera.fov")->layer == engine::ConfigProvenance::Layer::SceneFile);
    REQUIRE(store.provenanceFor("scene", "camera.fov")->source == scenePath.string());
    REQUIRE(store.provenanceFor("window", "title")->layer == engine::ConfigProvenance::Layer::EngineConfig);
}

TEST_CASE("scene files are rejected when they define more than [scene]", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;

    SECTION("a foreign section")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "scene.toml",
                                                         "[scene]\n\n[window]\ntitle = \"surprise\"\n");
        REQUIRE_FALSE(store.applySceneFile(path, error));
        REQUIRE(contains(error, "may only define the [scene] section"));
        REQUIRE(contains(error, "'window'"));
    }
    SECTION("no [scene] at all: an empty file")
    {
        // Any non-empty top-level key is rejected as a foreign section before
        // this diagnostic can fire, so the missing-[scene] error belongs to
        // the empty (or comments-only) file.
        const std::filesystem::path path =
            writeTextFile(uniqueTempPath("engine_config_test_") / "scene.toml", "# a scene file with no content\n");
        REQUIRE_FALSE(store.applySceneFile(path, error));
        REQUIRE(contains(error, "does not define a [scene] section"));
    }
    SECTION("a bare [scene] value")
    {
        const std::filesystem::path path =
            writeTextFile(uniqueTempPath("engine_config_test_") / "scene.toml", "scene = 3\n");
        REQUIRE_FALSE(store.applySceneFile(path, error));
        REQUIRE(contains(error, "must define [scene] as a table"));
    }
}

TEST_CASE("--set assignments parse TOML values and bare words", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;

    // Assembly context so the overrides have something to fight with.
    const std::filesystem::path assemblyPath =
        writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml",
                      "[window]\ntitle = \"assembly\"\n"
                      "\n[scene]\nsky_radiance = [1.0, 1.0, 1.0]\ncamera.fov = 62.0\n");
    REQUIRE(store.loadFromFile(assemblyPath, error));

    REQUIRE(store.stageCommandLineOverride("scene.camera.fov=90", error));
    REQUIRE(store.stageCommandLineOverride("scene.sky_radiance=[0.1, 0.2, 0.3]", error));
    REQUIRE(store.stageCommandLineOverride("window.title=\"Quoted Title\"", error));
    REQUIRE(store.stageCommandLineOverride("window.subtitle=My Room", error));
    REQUIRE(store.stageCommandLineOverride("diagnostics.latency_report=true", error));
    REQUIRE(store.stageCommandLineOverride("deep.nested.key=5", error));
    REQUIRE(store.applyCommandLineOverrides(error));

    // Integers are accepted for floating keys, exactly like in files.
    REQUIRE(store.floatingAtPath("scene", "camera.fov") == 90.0);
    REQUIRE(store.floatingArrayAtPath("scene", "sky_radiance") == std::vector<double>{0.1, 0.2, 0.3});
    REQUIRE(store.stringValue("window", "title") == "Quoted Title");
    REQUIRE(store.stringValue("window", "subtitle") == "My Room");
    REQUIRE(store.booleanValue("diagnostics", "latency_report") == true);
    REQUIRE(store.integerAtPath("deep", "nested.key") == 5);

    // The command line is the highest layer, everywhere it spoke.
    REQUIRE(store.provenanceFor("scene", "camera.fov")->layer == engine::ConfigProvenance::Layer::CommandLine);
    REQUIRE(store.provenanceFor("window", "subtitle")->layer == engine::ConfigProvenance::Layer::CommandLine);
    REQUIRE(contains(store.provenanceReport(), "scene.camera.fov <- command line (--set)"));
}

TEST_CASE("--set rejects malformed assignments at staging time", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    REQUIRE_FALSE(store.stageCommandLineOverride("noequals", error));
    REQUIRE(contains(error, "no '='"));
    REQUIRE_FALSE(store.stageCommandLineOverride("=1", error));
    REQUIRE_FALSE(store.stageCommandLineOverride("window..title=1", error));
    REQUIRE_FALSE(store.stageCommandLineOverride(".window.title=1", error));
    REQUIRE_FALSE(store.stageCommandLineOverride("window.title.=1", error));
    REQUIRE_FALSE(store.stageCommandLineOverride("scene.objects[0]=1", error));
    REQUIRE(contains(error, "dotted path"));
}

TEST_CASE("--set refuses to traverse through a non-table", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path assemblyPath =
        writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml", "[window]\ntitle = \"assembly\"\n");
    REQUIRE(store.loadFromFile(assemblyPath, error));
    REQUIRE(store.stageCommandLineOverride("window.title.sub=1", error));
    REQUIRE_FALSE(store.applyCommandLineOverrides(error));
    REQUIRE(contains(error, "'window.title' is not a table"));
}

TEST_CASE("reset returns the store to its just-constructed state", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path =
        writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml", "[window]\ntitle = \"x\"\n");
    REQUIRE(store.loadFromFile(path, error));
    REQUIRE(store.isLoaded());
    store.reset();
    REQUIRE_FALSE(store.isLoaded());
    REQUIRE(store.provenanceReport().empty());
}

// ---- known keys and unknown-key warnings ---------------------------------------

TEST_CASE("unknown keys are warned with their source", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path =
        writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml",
                      "[window]\ntitle = \"known\"\ntital = \"typo\"\n"
                      "\n[nothere]\nkey = 1\n");
    REQUIRE(store.loadFromFile(path, error));

    // Nothing is declared yet: every leaf warns, each with the section-level
    // note. Section order is a detail of the table's bookkeeping, so the
    // assertions are order-independent.
    std::vector<std::string> warnings = store.unknownKeyWarnings();
    REQUIRE(warnings.size() == 3);
    const std::string joined = warnings[0] + "|" + warnings[1] + "|" + warnings[2];
    REQUIRE(contains(joined, "unknown configuration key 'window.title'"));
    REQUIRE(contains(joined, "unknown configuration key 'window.tital'"));
    REQUIRE(contains(joined, "unknown configuration key 'nothere.key'"));
    REQUIRE(contains(joined, "no component declares keys in '[nothere]'"));
    REQUIRE(contains(joined, "no component declares keys in '[window]'"));

    // Declaring the one real key removes exactly that warning; the section is
    // now declared, so its remaining typo warns WITHOUT the section note.
    REQUIRE(store.declareKnownConfigKey("window.title"));
    warnings = store.unknownKeyWarnings();
    REQUIRE(warnings.size() == 2);
    const std::string joinedAfter = warnings[0] + "|" + warnings[1];
    REQUIRE(contains(joinedAfter, "unknown configuration key 'window.tital'"));
    REQUIRE(contains(joinedAfter, "from '" + path.string() + "'"));
    // The section is declared now, so its typo warns WITHOUT the note.
    REQUIRE_FALSE(contains(joinedAfter, "no component declares keys in '[window]'"));
    REQUIRE(contains(joinedAfter, "unknown configuration key 'nothere.key'"));
}

TEST_CASE("shipped-default keys are exempt from the unknown-key warnings", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path assemblyPath =
        writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml", "[custom]\nfrom_assembly = 1\n");
    REQUIRE(store.loadFromFile(assemblyPath, error));
    const std::filesystem::path defaultsPath =
        writeTextFile(uniqueTempPath("engine_config_test_") / "config.toml", "[custom]\nfrom_defaults = 1\n");
    REQUIRE(store.mergeDefaultsFile(defaultsPath, engine::ConfigProvenance::Layer::ModDefaults, "package", error));

    const std::vector<std::string> warnings = store.unknownKeyWarnings();
    REQUIRE(warnings.size() == 1);
    REQUIRE(contains(warnings[0], "custom.from_assembly"));
    REQUIRE(contains(warnings[0], "no component declares keys in '[custom]'"));
}

TEST_CASE("whole-section declarations cover dynamic key sets", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path = writeTextFile(uniqueTempPath("engine_config_test_") / "engine.toml",
                                                     "[providers]\nanything = \"some_provider\"\n");
    REQUIRE(store.loadFromFile(path, error));
    REQUIRE(store.declareKnownConfigKey("providers"));
    REQUIRE(store.unknownKeyWarnings().empty());
}

TEST_CASE("the assembly-owned sections are declared in one call", "[configuration]")
{
    engine::ConfigurationStore store;
    std::string error;
    const std::filesystem::path path = writeTextFile(
        uniqueTempPath("engine_config_test_") / "engine.toml",
        "[engine]\nmodules = []\n"
        "\n[providers]\nwindow = \"glfw_window\"\n"
        "\n[permissions]\ndenied = []\n"
        "\n[diagnostics]\nlatency_report = false\n"
        "\n[leak_test]\nmax_luminance = 0.02\nregion = [0.0, 0.0, 0.1, 0.1]\n");
    REQUIRE(store.loadFromFile(path, error));
    store.declareAssemblyKnownKeys();
    REQUIRE(store.unknownKeyWarnings().empty());
}

TEST_CASE("malformed known-key declarations are rejected", "[configuration]")
{
    engine::ConfigurationStore store;
    REQUIRE_FALSE(store.declareKnownConfigKey(""));
    REQUIRE_FALSE(store.declareKnownConfigKey("nodot but words"));
    REQUIRE_FALSE(store.declareKnownConfigKey("section."));
    REQUIRE_FALSE(store.declareKnownConfigKey(".section"));
    REQUIRE_FALSE(store.declareKnownConfigKey("section..key"));
    REQUIRE_FALSE(store.declareKnownConfigKey("section..key"));
    REQUIRE_FALSE(store.declareKnownConfigKey("section.objects[0]"));
    REQUIRE_FALSE(store.declareKnownConfigKey("section.bad key"));
    REQUIRE(store.declareKnownConfigKey("section.key"));
    REQUIRE(store.declareKnownConfigKey("section.nested.key"));
    REQUIRE(store.declareKnownConfigKey("providers"));
}

// ---- kernel-level integration ---------------------------------------------------

class FakeHostModule final : public engine::ModuleInterface
{
public:
    FakeHostModule()
    {
        manifest_.name = "fake_host";
        manifest_.providedCapabilities = {};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext&) override {}
    void start() override {}
    void stop() override {}
    void* capabilityPointerFor(const std::string&) override
    {
        return nullptr;
    }

private:
    engine::ComponentManifest manifest_;
};

TEST_CASE("the kernel assembles every layer and warns without failing", "[configuration][kernel]")
{
    const std::filesystem::path directory = uniqueTempPath("engine_config_kernel_");
    const std::filesystem::path assemblyPath = writeTextFile(
        directory / "engine.toml",
        "[engine]\nmodules = [\"fake_host\"]\n"
        "\n[example_defaults_mod]\nmessage = \"from the assembly\"\n"
        "\n[window]\ntital = \"typo\"\n");

    // A script mod package with shipped defaults and declared config keys.
    const std::filesystem::path package = directory / "mods" / "example_defaults_mod";
    writeTextFile(package / "mod.toml",
                  "id = \"example_defaults_mod\"\n"
                  "engine_api = 1\n"
                  "type = \"script\"\n"
                  "entry = \"script.lua\"\n"
                  "host = \"fake_host\"\n"
                  "capabilities = []\n"
                  "config_keys = [\"example_defaults_mod.message\"]\n");
    writeTextFile(package / "config.toml", "[example_defaults_mod]\nmessage = \"shipped default\"\n");

    const std::filesystem::path scenePath = writeTextFile(
        directory / "scene.toml", "[scene.camera]\nfov = 100.0\n");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(assemblyPath);
    kernel.setModsDirectory(package.parent_path()); // the temp mods directory
    kernel.setSceneFile(scenePath);
    kernel.registerModuleFactory("fake_host", []() { return std::make_unique<FakeHostModule>(); });
    std::string overrideError;
    REQUIRE(kernel.addCommandLineOverride("example_defaults_mod.message=from the command line", overrideError));
    REQUIRE(kernel.addCommandLineOverride("scene.camera.fov=110", overrideError));

    std::string report;
    REQUIRE(kernel.validate(report));

    // Layered precedence: the command line beat the assembly, the assembly
    // beat the shipped defaults, the scene file replaced the world content.
    REQUIRE(kernel.configuration().stringValue("example_defaults_mod", "message") == "from the command line");
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov") == 110.0);
    REQUIRE(kernel.configuration().provenanceFor("example_defaults_mod", "message")->layer ==
            engine::ConfigProvenance::Layer::CommandLine);

    // Without the --set, the assembly would have won over the shipped
    // default. Re-run the same composition without it to pin that rule.
    engine::EngineKernel withoutOverride;
    withoutOverride.setConfigurationFile(assemblyPath);
    withoutOverride.setModsDirectory(package.parent_path());
    withoutOverride.setSceneFile(scenePath);
    withoutOverride.registerModuleFactory("fake_host", []() { return std::make_unique<FakeHostModule>(); });
    std::string report2;
    REQUIRE(withoutOverride.validate(report2));
    REQUIRE(withoutOverride.configuration().stringValue("example_defaults_mod", "message") == "from the assembly");
    REQUIRE(withoutOverride.configuration().provenanceFor("example_defaults_mod", "message")->layer ==
            engine::ConfigProvenance::Layer::EngineConfig);

    // The declared config key produced no warning; the typo did. Warnings
    // never fail validation.
    REQUIRE(contains(report, "warning: unknown configuration key 'window.tital'"));
    REQUIRE_FALSE(contains(report, "example_defaults_mod.message"));

    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
}

TEST_CASE("a malformed manifest key declaration is a startup problem", "[configuration][kernel]")
{
    class BadDeclarer final : public engine::ModuleInterface
    {
    public:
        BadDeclarer()
        {
            manifest_.name = "bad_declarer";
            manifest_.knownConfigKeys = {"no dot anywhere"};
        }

        const engine::ComponentManifest& manifest() const override
        {
            return manifest_;
        }

        void construct(engine::ComponentContext&) override {}
        void start() override {}
        void stop() override {}
        void* capabilityPointerFor(const std::string&) override
        {
            return nullptr;
        }

    private:
        engine::ComponentManifest manifest_;
    };

    const std::filesystem::path assemblyPath =
        writeTextFile(uniqueTempPath("engine_config_kernel_") / "engine.toml", "[engine]\nmodules = [\"bad_declarer\"]\n");
    engine::EngineKernel kernel;
    kernel.setConfigurationFile(assemblyPath);
    kernel.setModsDirectory("mods_isolated_out_for_configuration_tests");
    kernel.registerModuleFactory("bad_declarer", []() { return std::make_unique<BadDeclarer>(); });

    std::string report;
    REQUIRE_FALSE(kernel.validate(report));
    REQUIRE(contains(report, "malformed configuration key"));

    std::error_code ignored;
    std::filesystem::remove(assemblyPath, ignored);
}
