// Test fixture: the smallest native mod that exercises the real shared
// library loading path (DynamicLibrary dlopen, engine_query_mod_entry_table
// resolution, interface version check, destroyMod teardown). The shipped
// mods are content packages; this fixture lives with the tests so the
// kernel's native mod path keeps end-to-end coverage without the engine
// shipping a second mod.
//
// Observability crosses the DSO boundary through the logging capability: the
// fixture acquires the LoggerCapability registered by the test binary's fake
// logging module and calls logMessage - a virtual call into test-owned code,
// so the lifecycle journal records the mod's phases.

#include "engine/capabilities/logging_contract.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/mod_api.hpp"

#include <memory>
#include <string>

namespace
{

class FixtureMod final : public engine::ModInterface
{
public:
    FixtureMod()
    {
        componentManifest.name = "native_mod_fixture";
        componentManifest.kind = engine::ComponentKind::SharedLibraryMod;
        componentManifest.requiredCapabilities = {"logging"};
        componentManifest.requiredPermissions = {"capability.logging"};
    }

    const engine::ComponentManifest& manifest() const override
    {
        return componentManifest;
    }

    void construct(engine::ComponentContext& context) override
    {
        logger = &context.acquireCapability<engine::LoggerCapability>();
        logger->logMessage(engine::LogLevel::Information, "native_mod_fixture.construct");
    }

    void start() override
    {
        logger->logMessage(engine::LogLevel::Information, "native_mod_fixture.start");
    }

    void stop() override
    {
        logger->logMessage(engine::LogLevel::Information, "native_mod_fixture.stop");
        logger = nullptr;
    }

    void* capabilityPointerFor(const std::string& capabilityName) override
    {
        // The fixture provides nothing; it only consumes the logger.
        (void)capabilityName;
        return nullptr;
    }

private:
    engine::ComponentManifest componentManifest;
    engine::LoggerCapability* logger = nullptr;
};

engine::ModInterface* createFixtureMod()
{
    return new FixtureMod();
}

void destroyFixtureMod(engine::ModInterface* mod)
{
    delete mod;
}

const EngineModEntryTable entryTable{
    engine::engineModInterfaceVersion,
    &createFixtureMod,
    &destroyFixtureMod,
};

} // namespace

// The one symbol the kernel resolves when loading a shared library mod.
extern "C" const EngineModEntryTable* engine_query_mod_entry_table()
{
    return &entryTable;
}
