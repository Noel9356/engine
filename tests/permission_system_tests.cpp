#include <catch2/catch_test_macros.hpp>

#include "engine/kernel/permission_system.hpp"

TEST_CASE("declared permissions are granted, undeclared are not", "[permission_system]")
{
    engine::PermissionSystem permissions;
    permissions.declarePermissions("screenshot", {"capability.logging", "capability.renderer", "filesystem.write"});

    REQUIRE(permissions.isGranted("screenshot", "capability.logging"));
    REQUIRE(permissions.isGranted("screenshot", "filesystem.write"));
    REQUIRE_FALSE(permissions.isGranted("screenshot", "capability.window"));
    REQUIRE_FALSE(permissions.isGranted("unknown_component", "capability.logging"));
}

TEST_CASE("configuration denials override declarations", "[permission_system]")
{
    engine::PermissionSystem permissions;
    permissions.declarePermissions("example_cpp_mod", {"capability.logging", "capability.input"});
    permissions.denyPermission("example_cpp_mod", "capability.input");

    REQUIRE(permissions.isGranted("example_cpp_mod", "capability.logging"));
    REQUIRE_FALSE(permissions.isGranted("example_cpp_mod", "capability.input"));

    const std::vector<std::string> missing =
        permissions.missingPermissions("example_cpp_mod", {"capability.logging", "capability.input"});
    REQUIRE(missing.size() == 1);
    REQUIRE(missing.front() == "capability.input");
}

TEST_CASE("trusted components keep every permission", "[permission_system]")
{
    engine::PermissionSystem permissions;
    permissions.grantAllPermissions("<host>");

    REQUIRE(permissions.isGranted("<host>", "capability.anything"));
    REQUIRE(permissions.isGranted("<host>", "filesystem.write"));

    permissions.denyPermission("<host>", "capability.anything");
    REQUIRE_FALSE(permissions.isGranted("<host>", "capability.anything"));
}
