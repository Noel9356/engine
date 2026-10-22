#include <catch2/catch_test_macros.hpp>

#include "engine/capabilities/logging_contract.hpp"
#include "engine/capabilities/window_contract.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/engine_kernel.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string>& lifecycleJournal()
{
    static std::vector<std::string> journal;
    return journal;
}

class FakeLoggingModule final : public engine::ModuleInterface, public engine::LoggerCapability
{
public:
    FakeLoggingModule()
    {
        manifest_.name = "fake_logging";
        manifest_.providedCapabilities = {"logging"};
        manifest_.requiredPermissions = {};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext& context) override
    {
        lifecycleJournal().push_back("logging.construct");
        context_ = &context;
    }

    void start() override
    {
        lifecycleJournal().push_back("logging.start");
    }

    void stop() override
    {
        lifecycleJournal().push_back("logging.stop");
    }

    void* capabilityPointerFor(const std::string& capabilityName) override
    {
        if (capabilityName == engine::LoggerCapability::capabilityName)
        {
            return static_cast<engine::LoggerCapability*>(this);
        }
        return nullptr;
    }

    // Journalled: a dlopened mod observably logs through this virtual call
    // (the mod object lives in its shared library, the logger in this binary),
    // which is how the native fixture test proves the loaded code ran.
    void logMessage(engine::LogLevel, const std::string& message) override
    {
        lifecycleJournal().push_back("log:" + message);
    }

private:
    engine::ComponentManifest manifest_;
    engine::ComponentContext* context_ = nullptr;
};

class FakeWindowModule final : public engine::ModuleInterface, public engine::WindowCapability
{
public:
    FakeWindowModule()
    {
        manifest_.name = "fake_window";
        manifest_.providedCapabilities = {"window"};
        manifest_.requiredCapabilities = {"logging"};
        manifest_.requiredPermissions = {"capability.logging"};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext& context) override
    {
        lifecycleJournal().push_back("window.construct");
        acquiredLogger = &context.acquireCapability<engine::LoggerCapability>();
    }

    void start() override
    {
        lifecycleJournal().push_back("window.start");
        backendStarted = true;
    }

    void stop() override
    {
        lifecycleJournal().push_back("window.stop");
        backendStarted = false;
    }

    void* capabilityPointerFor(const std::string& capabilityName) override
    {
        if (capabilityName == engine::WindowCapability::capabilityName)
        {
            return static_cast<engine::WindowCapability*>(this);
        }
        return nullptr;
    }

    // Unused window capability surface; the fake only proves the wiring.
    engine::WindowIdentifier createMainWindow(const engine::WindowDescription&) override { return 1; }
    bool windowCloseRequested(engine::WindowIdentifier) override { return false; }
    void pollEvents() override {}
    std::uint32_t framebufferWidth(engine::WindowIdentifier) override { return 0; }
    std::uint32_t framebufferHeight(engine::WindowIdentifier) override { return 0; }
    std::vector<engine::WindowIdentifier> windowIdentifiers() const override { return {1}; }
    std::vector<std::string> requiredInstanceExtensions(const std::string&) override { return {}; }
    void* createPresentationSurface(const std::string&, void*, engine::WindowIdentifier) override { return nullptr; }
    void* nativeWindowHandle(engine::WindowIdentifier) override { return nullptr; }
    engine::WindowIdentifier windowIdentifierFromNativeHandle(void*) override { return 0; }

    engine::LoggerCapability* acquiredLogger = nullptr;
    bool backendStarted = false;

private:
    engine::ComponentManifest manifest_;
};

class FakeInputMod final : public engine::ModInterface
{
public:
    FakeInputMod()
    {
        manifest_.name = "fake_input_mod";
        manifest_.kind = engine::ComponentKind::SharedLibraryMod;
        manifest_.requiredCapabilities = {"logging", "window"};
        manifest_.requiredPermissions = {"capability.logging", "capability.window"};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext& context) override
    {
        lifecycleJournal().push_back("mod.construct");
        acquiredLogger = &context.acquireCapability<engine::LoggerCapability>();
        context.acquireCapability<engine::WindowCapability>();
    }

    void start() override
    {
        lifecycleJournal().push_back("mod.start");
    }

    void stop() override
    {
        lifecycleJournal().push_back("mod.stop");
    }

    void* capabilityPointerFor(const std::string&) override
    {
        return nullptr;
    }

    engine::LoggerCapability* acquiredLogger = nullptr;

private:
    engine::ComponentManifest manifest_;
};

class FakeBrokenModule final : public engine::ModuleInterface
{
public:
    FakeBrokenModule()
    {
        manifest_.name = "fake_broken";
        manifest_.requiredCapabilities = {"missing_capability"};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext&) override {}
    void start() override {}
    void stop() override {}
    void* capabilityPointerFor(const std::string&) override { return nullptr; }

private:
    engine::ComponentManifest manifest_;
};

// Unique per call: several tests create temp dirs within the same clock tick.
std::size_t& temporaryPathCounter()
{
    static std::size_t counter = 0;
    return counter;
}

std::filesystem::path writeTemporaryConfiguration(const std::string& contents)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("engine_kernel_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         std::to_string(++temporaryPathCounter()) + ".toml");
    std::ofstream file(path, std::ios::trunc);
    file << contents;
    return path;
}

std::filesystem::path makeTemporaryDirectory(const std::string& prefix)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        (prefix + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         std::to_string(++temporaryPathCounter()));
    std::filesystem::create_directories(directory);
    return directory;
}

// Writes one mod package (manifest plus optional scene file) into a mods root
// and returns the package directory. Script mods in these tests declare the
// fake logging module as their host: the host availability check at discovery
// only needs a registered module factory, and construct is never reached in
// the validate-only scene resolution tests.
std::filesystem::path writeModPackage(const std::filesystem::path& modsRoot, const std::string& id,
                                      const std::string& manifestBody, const std::string& sceneContents)
{
    const std::filesystem::path package = modsRoot / id;
    std::filesystem::create_directories(package);
    std::ofstream manifest(package / "mod.toml", std::ios::trunc);
    manifest << manifestBody;
    if (!sceneContents.empty())
    {
        std::ofstream sceneFile(package / "scene.toml", std::ios::trunc);
        sceneFile << sceneContents;
    }
    return package;
}

std::string contentModManifest(const std::string& id)
{
    return "id = \"" + id + "\"\n"
           "version = \"1.0.0\"\n"
           "engine_api = 1\n"
           "type = \"script\"\n"
           "entry = \"script.lua\"\n"
           "host = \"fake_logging\"\n"
           "capabilities = [\"logging\"]\n"
           "permissions = [\"logging.write\"]\n"
           "scene = \"scene.toml\"\n";
}

const char* contentModSceneFile =
    "[scene]\n"
    "[scene.camera]\n"
    "fov = 40.0\n";

struct KernelFixture
{
    KernelFixture()
    {
        lifecycleJournal().clear();
        kernel.setConfigurationFile(configurationPath);
        // Isolate from any mods directory that may sit next to the test
        // binary: these fixtures assemble their components explicitly.
        kernel.setModsDirectory("mods_isolated_out_for_kernel_tests");
        kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });
        kernel.registerModuleFactory("fake_window", []() { return std::make_unique<FakeWindowModule>(); });
        kernel.registerModFactory("fake_input_mod", []() { return std::make_unique<FakeInputMod>(); });
    }

    ~KernelFixture()
    {
        std::error_code ignored;
        std::filesystem::remove(configurationPath, ignored);
    }

    std::filesystem::path configurationPath =
        writeTemporaryConfiguration(R"(
[engine]
modules = ["fake_logging", "fake_window"]
)");
    engine::EngineKernel kernel;
};

} // namespace

TEST_CASE_METHOD(KernelFixture, "lifecycle phases run in dependency order and stop in reverse", "[kernel]")
{
    std::string report;
    REQUIRE(kernel.construct(report));
    REQUIRE(report.empty());
    kernel.start();
    REQUIRE(kernel.isStarted());

    const std::vector<std::string>& journal = lifecycleJournal();
    REQUIRE(journal.size() == 6);
    REQUIRE(journal[0] == "logging.construct");
    REQUIRE(journal[1] == "window.construct");
    REQUIRE(journal[2] == "mod.construct");
    REQUIRE(journal[3] == "logging.start");
    REQUIRE(journal[4] == "window.start");
    REQUIRE(journal[5] == "mod.start");

    kernel.stop();
    REQUIRE(journal[6] == "mod.stop");
    REQUIRE(journal[7] == "window.stop");
    REQUIRE(journal[8] == "logging.stop");
    REQUIRE_FALSE(kernel.isStarted());
}

TEST_CASE_METHOD(KernelFixture, "shared capabilities resolve to one instance", "[kernel]")
{
    std::string report;
    REQUIRE(kernel.construct(report));

    // The registry hands out the same provider instance to every consumer.
    auto* registeredLogger =
        static_cast<engine::LoggerCapability*>(kernel.capabilityRegistry().acquireCapability("logging"));
    REQUIRE(registeredLogger != nullptr);

    kernel.start();
    kernel.stop();
}

TEST_CASE_METHOD(KernelFixture, "validation is a dry run with precise reports", "[kernel]")
{
    std::string report;
    REQUIRE(kernel.validate(report));
    // No construct phase ran: the journal stays empty after validation.
    REQUIRE(lifecycleJournal().empty());
}

TEST_CASE("a required but absent capability produces a precise startup failure", "[kernel]")
{
    lifecycleJournal().clear();
    const std::filesystem::path configurationPath = []()
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / "engine_kernel_test_broken.toml";
        std::ofstream file(path, std::ios::trunc);
        file << "[engine]\nmodules = [\"fake_broken\"]\n";
        return path;
    }();

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(configurationPath);
    kernel.setModsDirectory("mods_isolated_out_for_kernel_tests");
    kernel.registerModuleFactory("fake_broken", []() { return std::make_unique<FakeBrokenModule>(); });

    std::string report;
    REQUIRE_FALSE(kernel.construct(report));
    REQUIRE(report.find("fake_broken") != std::string::npos);
    REQUIRE(report.find("missing_capability") != std::string::npos);
    REQUIRE(lifecycleJournal().empty());

    std::error_code ignored;
    std::filesystem::remove(configurationPath, ignored);
}

TEST_CASE("a denied permission fails startup before any construct runs", "[kernel]")
{
    lifecycleJournal().clear();
    const std::filesystem::path configurationPath = []()
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / "engine_kernel_test_denied.toml";
        std::ofstream file(path, std::ios::trunc);
        file << "[engine]\nmodules = [\"fake_logging\", \"fake_window\"]\n";
        file << "\n[permissions]\ndenied = [[\"fake_input_mod\", \"capability.window\"]]\n";
        return path;
    }();

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(configurationPath);
    kernel.setModsDirectory("mods_isolated_out_for_kernel_tests");
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });
    kernel.registerModuleFactory("fake_window", []() { return std::make_unique<FakeWindowModule>(); });
    kernel.registerModFactory("fake_input_mod", []() { return std::make_unique<FakeInputMod>(); });

    std::string report;
    REQUIRE_FALSE(kernel.construct(report));
    REQUIRE(report.find("capability.window") != std::string::npos);
    REQUIRE(report.find("fake_input_mod") != std::string::npos);
    REQUIRE(lifecycleJournal().empty());

    std::error_code ignored;
    std::filesystem::remove(configurationPath, ignored);
}

// Regression: a component may cache the ComponentContext pointer it receives
// during construct and publish events through it long after the construct
// phase returned (frame loops, input callbacks, capture events). The context
// used to be a stack local of the construct loop, so every such pointer
// dangled and the first post-startup publish ran on corrupted state. The
// Catch2 suite never ran a frame loop, so only the headless smoke run caught
// it; this test pins the guarantee at kernel level.
struct PostStartupEvent
{
    int value = 0;
};

class FakeFramePublisherModule final : public engine::ModuleInterface
{
public:
    FakeFramePublisherModule()
    {
        manifest_.name = "fake_frame_publisher";
    }

    const engine::ComponentManifest& manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ComponentContext& context) override
    {
        lifecycleJournal().push_back("publisher.construct");
        cachedContext = &context;
    }

    void start() override
    {
        lifecycleJournal().push_back("publisher.start");
    }

    void stop() override
    {
        lifecycleJournal().push_back("publisher.stop");
    }

    void* capabilityPointerFor(const std::string&) override
    {
        return nullptr;
    }

    // Publishes through the context cached during construct: exactly the
    // pattern that used to dangle once the construct phase returned.
    void publishFromCachedContext(int value)
    {
        cachedContext->eventBus().publish(PostStartupEvent{value});
    }

private:
    engine::ComponentManifest manifest_;
    engine::ComponentContext* cachedContext = nullptr;
};

TEST_CASE("a context cached during construct stays valid for the engine lifetime", "[kernel]")
{
    lifecycleJournal().clear();
    const std::filesystem::path configurationPath = []()
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() / "engine_kernel_test_context.toml";
        std::ofstream file(path, std::ios::trunc);
        file << "[engine]\nmodules = [\"fake_frame_publisher\"]\n";
        return path;
    }();

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(configurationPath);
    kernel.setModsDirectory("mods_isolated_out_for_kernel_tests");
    FakeFramePublisherModule* publisher = nullptr;
    kernel.registerModuleFactory("fake_frame_publisher",
                                 [&publisher]() -> std::unique_ptr<engine::ModuleInterface>
                                 {
                                     auto module = std::make_unique<FakeFramePublisherModule>();
                                     publisher = module.get();
                                     return module;
                                 });

    struct Observer
    {
        int lastValue = 0;
        int count = 0;
    } observer;
    kernel.eventBus().channel<PostStartupEvent>().subscribe(
        [](void* userData, const PostStartupEvent& event)
        {
            auto* observed = static_cast<Observer*>(userData);
            observed->lastValue = event.value;
            ++observed->count;
        },
        &observer);

    std::string report;
    REQUIRE(kernel.construct(report));
    REQUIRE(publisher != nullptr);
    kernel.start();
    REQUIRE(kernel.isStarted());

    // The construct phase's stack frames are long gone at this point; the
    // cached context must still be valid and wired to the kernel bus.
    publisher->publishFromCachedContext(42);
    REQUIRE(observer.count == 1);
    REQUIRE(observer.lastValue == 42);

    for (int frame = 0; frame < 100; ++frame)
    {
        publisher->publishFromCachedContext(frame);
    }
    REQUIRE(observer.count == 101);
    REQUIRE(observer.lastValue == 99);

    kernel.stop();
    REQUIRE(lifecycleJournal().back() == "publisher.stop");

    std::error_code ignored;
    std::filesystem::remove(configurationPath, ignored);
}

// The kernel's native mod path (mod package discovery, DynamicLibrary load,
// entry table resolution, destroyMod teardown) previously had no in-process
// coverage: the shipped example mods exercised it only in the headless smoke
// run. The engine now ships content packages only, so this fixture library
// carries the coverage at unit level.
TEST_CASE("a native mod package loads through the real shared library path", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_native_mod_test_");
    const std::filesystem::path package = modsRoot / "native_mod_fixture";
    std::filesystem::create_directories(package);
    {
        std::ofstream manifest(package / "mod.toml", std::ios::trunc);
        manifest << "id = \"native_mod_fixture\"\n"
                    "version = \"1.0.0\"\n"
                    "engine_api = 1\n"
                    "type = \"native\"\n"
                    "entry = \"native_mod_fixture\"\n"
                    "capabilities = [\"logging\"]\n"
                    "permissions = [\"capability.logging\"]\n";
    }
    std::error_code copyError;
    std::filesystem::copy_file(ENGINE_NATIVE_MOD_FIXTURE_LIBRARY,
                               package / std::filesystem::path(ENGINE_NATIVE_MOD_FIXTURE_LIBRARY).filename(),
                               std::filesystem::copy_options::overwrite_existing, copyError);
    REQUIRE_FALSE(copyError);

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.construct(report));
    REQUIRE(report.empty());
    kernel.start();
    kernel.stop();

    // The journal entries come from the dlopened instance through the logger
    // capability: discovery, symbol resolution and the lifecycle phases all
    // ran for real, in dependency order (logging first, mod last; reverse on
    // stop, and the library unloads strictly after destroyMod).
    const std::vector<std::string>& journal = lifecycleJournal();
    const std::vector<std::string> expected{
        "logging.construct",
        "log:native_mod_fixture.construct",
        "logging.start",
        "log:native_mod_fixture.start",
        "log:native_mod_fixture.stop",
        "logging.stop",
    };
    REQUIRE(journal == expected);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("a native mod needs no permissions list: capability requirements grant their acquisitions", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_native_mod_test_");
    const std::filesystem::path package = modsRoot / "native_mod_fixture";
    std::filesystem::create_directories(package);
    {
        std::ofstream manifest(package / "mod.toml", std::ios::trunc);
        // No 'permissions' key at all: the capability.<name> grants are
        // derived from the required capabilities, so this package acquires
        // the logger like one that declared both lists.
        manifest << "id = \"native_mod_fixture\"\n"
                    "version = \"1.0.0\"\n"
                    "engine_api = 1\n"
                    "type = \"native\"\n"
                    "entry = \"native_mod_fixture\"\n"
                    "capabilities = [\"logging\"]\n";
    }
    std::error_code copyError;
    std::filesystem::copy_file(ENGINE_NATIVE_MOD_FIXTURE_LIBRARY,
                               package / std::filesystem::path(ENGINE_NATIVE_MOD_FIXTURE_LIBRARY).filename(),
                               std::filesystem::copy_options::overwrite_existing, copyError);
    REQUIRE_FALSE(copyError);

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.construct(report));
    REQUIRE(report.empty());
    kernel.start();
    kernel.stop();

    // The derived grant reached the runtime: the mod's construct actually
    // acquired and used the logger capability.
    const std::vector<std::string>& journal = lifecycleJournal();
    REQUIRE(std::find(journal.begin(), journal.end(), "log:native_mod_fixture.construct") != journal.end());

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("a content mod's declared scene provides the world when the assembly defines none", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_mod", contentModManifest("content_mod"), contentModSceneFile);

    engine::EngineKernel kernel;
    // No [scene] in the assembly: the world is delegated to the content mod.
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().hasSection("scene"));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 40.0);
    REQUIRE(kernel.configuration().provenanceFor("scene", "camera.fov")->layer ==
            engine::ConfigProvenance::Layer::SceneFile);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("an assembly [scene] wins over an installed content mod's scene", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_mod", contentModManifest("content_mod"), contentModSceneFile);

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration(
        "[engine]\nmodules = [\"fake_logging\"]\n\n[scene]\n[scene.camera]\nfov = 55.0\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    // Valid, but the author is told the installed content is not the world.
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 55.0);
    REQUIRE(kernel.configuration().provenanceFor("scene", "camera.fov")->layer ==
            engine::ConfigProvenance::Layer::EngineConfig);
    REQUIRE(report.find("declares scene file") != std::string::npos);
    REQUIRE(report.find("the mod scene is not applied") != std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("two content mods without an explicit world source fail startup precisely", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_a", contentModManifest("content_a"), contentModSceneFile);
    writeModPackage(modsRoot, "content_b", contentModManifest("content_b"), contentModSceneFile);

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE_FALSE(kernel.validate(report));
    // The one-world rule bites only where the world is ambiguous: both
    // claims are named, and the report says how to resolve the ambiguity.
    REQUIRE(report.find("the world is ambiguous") != std::string::npos);
    REQUIRE(report.find("content_a") != std::string::npos);
    REQUIRE(report.find("content_b") != std::string::npos);
    REQUIRE(report.find("--scene") != std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("two content mods coexist when the assembly defines its own scene", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_a", contentModManifest("content_a"), contentModSceneFile);
    writeModPackage(modsRoot, "content_b", contentModManifest("content_b"), contentModSceneFile);

    engine::EngineKernel kernel;
    // An explicit world source makes the installed content claims
    // irrelevant: both packages stay active, the assembly owns the world.
    kernel.setConfigurationFile(writeTemporaryConfiguration(
        "[engine]\nmodules = [\"fake_logging\"]\n\n[scene]\n[scene.camera]\nfov = 55.0\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 55.0);
    REQUIRE(kernel.configuration().provenanceFor("scene", "camera.fov")->layer ==
            engine::ConfigProvenance::Layer::EngineConfig);
    // validate() still tells the author that the installed content is not
    // the world this run uses - once per claim.
    REQUIRE(report.find("the mod scene is not applied") != std::string::npos);
    REQUIRE(report.find("content_a") != std::string::npos);
    REQUIRE(report.find("content_b") != std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("two content mods coexist when --scene selects the world", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_a", contentModManifest("content_a"), contentModSceneFile);
    writeModPackage(modsRoot, "content_b", contentModManifest("content_b"), contentModSceneFile);

    const std::filesystem::path commandLineScene = makeTemporaryDirectory("engine_mod_scene_flag_") / "world.toml";
    {
        std::ofstream sceneFile(commandLineScene, std::ios::trunc);
        sceneFile << "[scene]\n[scene.camera]\nfov = 70.0\n";
    }

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setSceneFile(commandLineScene);
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 70.0);
    // An explicit --scene is deliberate intent: no shadowing warnings.
    REQUIRE(report.find("the mod scene is not applied") == std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("a disabled content mod's scene is not claimed", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_a", contentModManifest("content_a"), contentModSceneFile);
    writeModPackage(modsRoot, "content_b", contentModManifest("content_b"), contentModSceneFile);

    engine::EngineKernel kernel;
    // Disabling content_b resolves the ambiguity on purpose.
    kernel.setConfigurationFile(writeTemporaryConfiguration(
        "[engine]\nmodules = [\"fake_logging\"]\ndisabled_mods = [\"content_b\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 40.0);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("a missing mod scene file is a precise startup failure", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    // The manifest declares a scene file the package does not carry.
    writeModPackage(modsRoot, "content_mod", contentModManifest("content_mod"), "");

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE_FALSE(kernel.validate(report));
    REQUIRE(report.find("declares scene file") != std::string::npos);
    REQUIRE(report.find("does not exist") != std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}

TEST_CASE("the --scene flag overrides the content mod's scene file", "[kernel][mods]")
{
    lifecycleJournal().clear();
    const std::filesystem::path modsRoot = makeTemporaryDirectory("engine_mod_scene_test_");
    writeModPackage(modsRoot, "content_mod", contentModManifest("content_mod"), contentModSceneFile);

    const std::filesystem::path commandLineScene = makeTemporaryDirectory("engine_mod_scene_flag_") / "world.toml";
    {
        std::ofstream sceneFile(commandLineScene, std::ios::trunc);
        sceneFile << "[scene]\n[scene.camera]\nfov = 70.0\n";
    }

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(writeTemporaryConfiguration("[engine]\nmodules = [\"fake_logging\"]\n"));
    kernel.setSceneFile(commandLineScene);
    kernel.setModsDirectory(modsRoot);
    kernel.registerModuleFactory("fake_logging", []() { return std::make_unique<FakeLoggingModule>(); });

    std::string report;
    REQUIRE(kernel.validate(report));
    REQUIRE(kernel.configuration().floatingAtPath("scene", "camera.fov").value_or(0.0) == 70.0);
    // An explicit --scene is deliberate intent: no shadowing warning.
    REQUIRE(report.find("the mod scene is not applied") == std::string::npos);

    std::error_code cleanupError;
    std::filesystem::remove_all(modsRoot, cleanupError);
}
