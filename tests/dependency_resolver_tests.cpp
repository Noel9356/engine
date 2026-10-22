#include <catch2/catch_test_macros.hpp>

#include "engine/kernel/dependency_resolver.hpp"

#include <algorithm>

namespace
{

engine::ComponentManifest makeManifest(const std::string& name, engine::ComponentKind kind,
                                       std::vector<std::string> provides, std::vector<std::string> requiresCapabilities)
{
    engine::ComponentManifest manifest;
    manifest.name = name;
    manifest.kind = kind;
    manifest.providedCapabilities = std::move(provides);
    manifest.requiredCapabilities = std::move(requiresCapabilities);
    return manifest;
}

bool orderedBefore(const engine::DependencyResolver::ResolutionResult& result,
                   const std::vector<engine::ComponentManifest>& manifests, const std::string& first,
                   const std::string& second)
{
    std::size_t firstPosition = manifests.size();
    std::size_t secondPosition = manifests.size();
    for (std::size_t index = 0; index < result.constructionOrder.size(); ++index)
    {
        const std::string& name = manifests[result.constructionOrder[index]].name;
        if (name == first)
        {
            firstPosition = index;
        }
        if (name == second)
        {
            secondPosition = index;
        }
    }
    return firstPosition < secondPosition;
}

} // namespace

TEST_CASE("a linear chain resolves providers before consumers", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("consumer", engine::ComponentKind::Module, {}, {"middle_capability"}),
        makeManifest("middle", engine::ComponentKind::Module, {"middle_capability"}, {"logging"}),
        makeManifest("logging", engine::ComponentKind::Module, {"logging"}, {}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE(result.successful);
    REQUIRE(result.diagnostics.empty());
    REQUIRE(result.constructionOrder.size() == 3);
    REQUIRE(orderedBefore(result, manifests, "logging", "middle"));
    REQUIRE(orderedBefore(result, manifests, "middle", "consumer"));
}

TEST_CASE("two components requiring the same capability share one provider", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("first_consumer", engine::ComponentKind::Module, {}, {"shared"}),
        makeManifest("second_consumer", engine::ComponentKind::SharedLibraryMod, {}, {"shared"}),
        makeManifest("provider", engine::ComponentKind::Module, {"shared"}, {}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE(result.successful);
    REQUIRE(orderedBefore(result, manifests, "provider", "first_consumer"));
    REQUIRE(orderedBefore(result, manifests, "provider", "second_consumer"));
}

TEST_CASE("a module may depend on a capability provided by a mod and vice versa", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("module_consumer", engine::ComponentKind::Module, {}, {"mod_capability"}),
        makeManifest("some_mod", engine::ComponentKind::SharedLibraryMod, {"mod_capability"}, {"module_capability"}),
        makeManifest("provider_module", engine::ComponentKind::Module, {"module_capability"}, {}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE(result.successful);
    REQUIRE(orderedBefore(result, manifests, "provider_module", "some_mod"));
    REQUIRE(orderedBefore(result, manifests, "some_mod", "module_consumer"));
}

TEST_CASE("an unknown capability is reported with the requester", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("wants_missing", engine::ComponentKind::Module, {}, {"does_not_exist"}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE_FALSE(result.successful);
    REQUIRE(result.diagnostics.size() == 1);
    REQUIRE(result.diagnostics.front().kind == engine::DependencyResolver::ResolutionDiagnostic::Kind::UnknownCapability);
    REQUIRE(result.diagnostics.front().message.find("wants_missing") != std::string::npos);
    REQUIRE(result.diagnostics.front().message.find("does_not_exist") != std::string::npos);
}

TEST_CASE("several providers without a selection are ambiguous", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("consumer", engine::ComponentKind::Module, {}, {"window"}),
        makeManifest("glfw_window", engine::ComponentKind::Module, {"window"}, {}),
        makeManifest("sdl_window", engine::ComponentKind::Module, {"window"}, {}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult ambiguous = resolver.resolve();
    REQUIRE_FALSE(ambiguous.successful);
    REQUIRE(ambiguous.diagnostics.front().kind ==
            engine::DependencyResolver::ResolutionDiagnostic::Kind::AmbiguousCapability);

    engine::DependencyResolver selecting(manifests, {{"window", "sdl_window"}});
    const engine::DependencyResolver::ResolutionResult selected = selecting.resolve();
    REQUIRE(selected.successful);
    REQUIRE(orderedBefore(selected, manifests, "sdl_window", "consumer"));
}

TEST_CASE("cycles are detected with every participant", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("alpha", engine::ComponentKind::Module, {"alpha_capability"}, {"gamma_capability"}),
        makeManifest("beta", engine::ComponentKind::Module, {"beta_capability"}, {"alpha_capability"}),
        makeManifest("gamma", engine::ComponentKind::Module, {"gamma_capability"}, {"beta_capability"}),
    };

    engine::DependencyResolver resolver(manifests, {});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE_FALSE(result.successful);
    REQUIRE(result.diagnostics.size() == 1);
    REQUIRE(result.diagnostics.front().kind ==
            engine::DependencyResolver::ResolutionDiagnostic::Kind::CyclicDependency);
    REQUIRE(result.diagnostics.front().involvementChain.size() == 3);

    const std::string& message = result.diagnostics.front().message;
    REQUIRE(message.find("alpha") != std::string::npos);
    REQUIRE(message.find("beta") != std::string::npos);
    REQUIRE(message.find("gamma") != std::string::npos);
    REQUIRE(message.find("->") != std::string::npos);
}

TEST_CASE("component name requirements order script mods after their host", "[dependency_resolver]")
{
    const std::vector<engine::ComponentManifest> manifests = {
        makeManifest("example_lua_mod", engine::ComponentKind::ScriptMod, {}, {"logging"}),
        makeManifest("lua_scripting", engine::ComponentKind::Module, {}, {"logging"}),
        makeManifest("logging", engine::ComponentKind::Module, {"logging"}, {}),
    };

    engine::DependencyResolver resolver(manifests, {{"logging", "logging"}});
    const engine::DependencyResolver::ResolutionResult result = resolver.resolve();

    REQUIRE(result.successful);
    REQUIRE(orderedBefore(result, manifests, "logging", "lua_scripting"));
}
