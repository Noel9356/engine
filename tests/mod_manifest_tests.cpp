#include <catch2/catch_test_macros.hpp>

#include "engine/capabilities/logging_contract.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/mod_manifest.hpp"

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

std::filesystem::path uniqueTempDirectory(const std::string& prefix)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        (prefix + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    return directory;
}

} // namespace

TEST_CASE("mod manifests parse and validate", "[mods]")
{
    engine::ModManifestLoader loader;
    engine::ModManifest manifest;
    std::string error;

    SECTION("a valid native manifest parses with defaults")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"example_mod\"\n"
            "type = \"native\"\n"
            "entry = \"example_mod\"\n"
            "capabilities = [\"logging\", \"input\"]\n"
            "permissions = [\"capability.logging\"]\n");
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.id == "example_mod");
        REQUIRE(manifest.isNative());
        REQUIRE_FALSE(manifest.isScript());
        REQUIRE(manifest.version == "0.0.0");       // default
        REQUIRE(manifest.displayName == "example_mod"); // falls back to the id
        REQUIRE(manifest.host == "lua_scripting");  // default, irrelevant for native mods
        REQUIRE(manifest.capabilities == std::vector<std::string>{"logging", "input"});
        REQUIRE(manifest.permissions == std::vector<std::string>{"capability.logging"});
        REQUIRE(manifest.directory == path.parent_path());
    }

    SECTION("a valid script manifest parses")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"script_mod\"\n"
            "version = \"2.1.0\"\n"
            "engine_api = 1\n"
            "type = \"script\"\n"
            "entry = \"main.lua\"\n"
            "host = \"my_scripting\"\n"
            "display_name = \"Script Mod\"\n"
            "requires = []\n");
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.isScript());
        REQUIRE(manifest.version == "2.1.0");
        REQUIRE(manifest.entry == "main.lua");
        REQUIRE(manifest.host == "my_scripting");
        REQUIRE(manifest.displayName == "Script Mod");
    }

    SECTION("a script mod may declare requires")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"script_mod\"\n"
            "type = \"script\"\n"
            "entry = \"main.lua\"\n"
            "requires = [\"window\", \"renderer\"]\n");
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.requiresComponents == std::vector<std::string>{"window", "renderer"});
    }

    SECTION("a missing required key is a precise error")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"example_mod\"\ntype = \"native\"\n"); // no entry
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("'entry'") != std::string::npos);
    }

    SECTION("an unknown type is a precise error")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"example_mod\"\ntype = \"widget\"\nentry = \"x\"\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("expected 'native' or 'script'") != std::string::npos);
    }

    SECTION("a wrong engine_api is a precise error")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"example_mod\"\nengine_api = 999\ntype = \"native\"\nentry = \"x\"\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("engine_api 999") != std::string::npos);
        REQUIRE(error.find("this engine provides 1") != std::string::npos);
    }

    SECTION("an invalid id is a precise error")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = \"not a valid id!\"\ntype = \"native\"\nentry = \"x\"\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("letters, digits and underscores") != std::string::npos);
    }

    SECTION("a malformed toml file reports the parser location")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            "id = [unclosed\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("could not be parsed") != std::string::npos);
        REQUIRE(error.find("line 1") != std::string::npos);
    }
}

namespace
{

class FakeLoggingModule final : public engine::ModuleInterface, public engine::LoggerCapability
{
public:
    FakeLoggingModule()
    {
        manifest_.name = "fake_logging";
        manifest_.providedCapabilities = {"logging"};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext&) override {}
    void start() override {}
    void stop() override {}

    void* capabilityPointerFor(const std::string& capabilityName) override
    {
        if (capabilityName == engine::LoggerCapability::capabilityName)
        {
            return static_cast<engine::LoggerCapability*>(this);
        }
        return nullptr;
    }

    void logMessage(engine::LogLevel, const std::string&) override {}

private:
    engine::ComponentManifest manifest_;
};

class FakeScriptHostModule final : public engine::ModuleInterface, public engine::ScriptModHost
{
public:
    FakeScriptHostModule()
    {
        manifest_.name = "fake_script_host";
        manifest_.requiredCapabilities = {"logging"};
        manifest_.requiredPermissions = {"capability.logging"};
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

    engine::ScriptModHost* scriptModHost() override
    {
        return this;
    }

    void hostScriptMod(const engine::ScriptModDescription& description) override
    {
        hostedMods.push_back(description);
    }

    std::vector<engine::ScriptModDescription> hostedMods;

private:
    engine::ComponentManifest manifest_;
};

std::filesystem::path writeConfiguration(const std::string& contents)
{
    const std::filesystem::path path = uniqueTempDirectory("engine_mods_cfg_") / "engine.toml";
    std::ofstream file(path, std::ios::trunc);
    file << contents;
    return path;
}

} // namespace

TEST_CASE("script mods are discovered through mod.toml and hosted through the contract", "[mods]")
{
    const std::filesystem::path modsDirectory = uniqueTempDirectory("engine_mods_dir_");
    writeTextFile(modsDirectory / "example_script" / "mod.toml",
                  "id = \"example_script\"\n"
                  "type = \"script\"\n"
                  "entry = \"main.lua\"\n"
                  "host = \"fake_script_host\"\n"
                  "capabilities = [\"logging\"]\n"
                  "permissions = []\n");
    writeTextFile(modsDirectory / "example_script" / "main.lua", "-- script content\n");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeConfiguration(
        "[engine]\nmodules = [\"fake_logging\", \"fake_script_host\"]\n"));
    kernel.setModsDirectory(modsDirectory);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });
    kernel.registerModuleFactory("fake_script_host", []() { return std::make_unique<FakeScriptHostModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(report.empty());

    REQUIRE(kernel.construct(report));
    kernel.start();
    kernel.stop();
}

TEST_CASE("a missing script host is a precise validation failure", "[mods]")
{
    const std::filesystem::path modsDirectory = uniqueTempDirectory("engine_mods_dir_");
    writeTextFile(modsDirectory / "orphan_script" / "mod.toml",
                  "id = \"orphan_script\"\n"
                  "type = \"script\"\n"
                  "entry = \"main.lua\"\n"
                  "host = \"not_compiled\"\n");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(
        writeConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsDirectory);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE_FALSE(kernel.validate(report));
    REQUIRE(report.find("orphan_script") != std::string::npos);
    REQUIRE(report.find("not_compiled") != std::string::npos);
}

TEST_CASE("disabled mods are skipped during discovery", "[mods]")
{
    const std::filesystem::path modsDirectory = uniqueTempDirectory("engine_mods_dir_");
    writeTextFile(modsDirectory / "example_script" / "mod.toml",
                  "id = \"example_script\"\n"
                  "type = \"script\"\n"
                  "entry = \"main.lua\"\n"
                  "host = \"fake_script_host\"\n");
    writeTextFile(modsDirectory / "example_script" / "main.lua", "-- script content\n");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeConfiguration(
        "[engine]\nmodules = [\"fake_logging\", \"fake_script_host\"]\ndisabled_mods = [\"example_script\"]\n"));
    kernel.setModsDirectory(modsDirectory);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });
    kernel.registerModuleFactory("fake_script_host", []() { return std::make_unique<FakeScriptHostModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));

    // The construct phase hands over nothing: the mod is disabled on purpose.
    REQUIRE(kernel.construct(report));
    kernel.start();
    kernel.stop();
}

TEST_CASE("duplicate mod ids are diagnosed", "[mods]")
{
    const std::filesystem::path modsDirectory = uniqueTempDirectory("engine_mods_dir_");
    writeTextFile(modsDirectory / "package_one" / "mod.toml",
                  "id = \"same_id\"\ntype = \"script\"\nentry = \"a.lua\"\n");
    writeTextFile(modsDirectory / "package_two" / "mod.toml",
                  "id = \"same_id\"\ntype = \"script\"\nentry = \"b.lua\"\n");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(
        writeConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsDirectory);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE_FALSE(kernel.validate(report));
    REQUIRE(report.find("duplicate mod id 'same_id'") != std::string::npos);
}

TEST_CASE("config_keys parse into the manifest and are validated", "[mods]")
{
    engine::ModManifestLoader loader;
    engine::ModManifest manifest;
    std::string error;
    const std::string manifestPrefix = "id = \"configurable_mod\"\ntype = \"script\"\nentry = \"main.lua\"\n";

    SECTION("a valid config_keys list parses")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            manifestPrefix + "config_keys = [\"configurable_mod.level\", \"scene.camera.fov\"]\n");
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.configKeys == std::vector<std::string>{"configurable_mod.level", "scene.camera.fov"});
    }

    SECTION("a malformed entry names the manifest and the entry")
    {
        const std::filesystem::path path = writeTextFile(
            uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
            manifestPrefix + "config_keys = [\"level\"]\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("config_keys entry 'level'") != std::string::npos);
        REQUIRE(error.find("'<section>.<dotted path>'") != std::string::npos);
    }

    SECTION("config_keys is optional")
    {
        const std::filesystem::path path =
            writeTextFile(uniqueTempDirectory("engine_manifest_test_") / "mod.toml", manifestPrefix);
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.configKeys.empty());
    }
}

TEST_CASE("the manifest scene declaration is parsed and validated", "[mods]")
{
    engine::ModManifestLoader loader;
    engine::ModManifest manifest;
    std::string error;
    const std::string manifestPrefix =
        "id = \"content_mod\"\ntype = \"script\"\nentry = \"main.lua\"\nhost = \"lua_scripting\"\n";

    SECTION("a package-relative scene file parses")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
                                                         manifestPrefix + "scene = \"scene.toml\"\n");
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.scene == "scene.toml");
    }

    SECTION("scene is optional")
    {
        const std::filesystem::path path =
            writeTextFile(uniqueTempDirectory("engine_manifest_test_") / "mod.toml", manifestPrefix);
        REQUIRE(loader.loadFromFile(path, manifest, error));
        REQUIRE(manifest.scene.empty());
    }

    SECTION("an absolute scene path is rejected")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
                                                         manifestPrefix + "scene = \"/etc/engine/world.toml\"\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("must be a file path inside the mod package") != std::string::npos);
    }

    SECTION("a scene path climbing out of the package is rejected")
    {
        const std::filesystem::path path = writeTextFile(uniqueTempDirectory("engine_manifest_test_") / "mod.toml",
                                                         manifestPrefix + "scene = \"../other/world.toml\"\n");
        REQUIRE_FALSE(loader.loadFromFile(path, manifest, error));
        REQUIRE(error.find("must be a file path inside the mod package") != std::string::npos);
    }
}
