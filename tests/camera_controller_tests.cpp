// Fly camera controller tests. camera_controller.hpp is header only and
// Vulkan-free; a stub InputCapability drives it deterministically, so the
// binding and integration math is pinned without any window system.

#include "camera_controller.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using Catch::Approx;

#include <cmath>
#include <set>

using namespace engine;

namespace
{

// Records what the controller does and feeds scripted state back. consume*
// drains a queued delta exactly like the real module's accumulators.
class StubInput final : public InputCapability
{
public:
    bool isKeyDown(KeyCode keyCode) override
    {
        return keys.count(keyCode) != 0;
    }

    bool isMouseButtonDown(MouseButton button) override
    {
        return buttons.count(button) != 0;
    }

    void consumeMouseDelta(double& deltaX, double& deltaY) override
    {
        deltaX = mouseX;
        deltaY = mouseY;
        mouseX = 0.0;
        mouseY = 0.0;
    }

    void consumeScrollDelta(double& deltaX, double& deltaY) override
    {
        deltaX = scrollX;
        deltaY = scrollY;
        scrollX = 0.0;
        scrollY = 0.0;
    }

    void setCursorCaptured(bool captured) override
    {
        cursorCaptured = captured;
    }

    bool isCursorCaptured() const override
    {
        return cursorCaptured;
    }

    std::set<KeyCode> keys;
    std::set<MouseButton> buttons;
    double mouseX = 0.0;
    double mouseY = 0.0;
    double scrollX = 0.0;
    double scrollY = 0.0;
    bool cursorCaptured = false;
};

struct NullLogger final : public LoggerCapability
{
    void logMessage(LogLevel, const std::string&) override
    {
    }
};

SceneCamera demoCamera()
{
    SceneCamera camera;
    camera.position = {0.0f, 1.55f, 2.9f};
    camera.lookAt = {0.0f, 1.05f, -0.4f};
    camera.freeFly = true;
    camera.flySpeed = 4.0f;
    camera.flyBoost = 4.0f;
    camera.lookSensitivity = 0.0022f;
    return camera;
}

} // namespace

TEST_CASE("fly controller preserves the configured pose without input", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    controller.syncFromCamera(camera);
    controller.applyToCamera(camera);

    // applyToCamera re-expresses look_at as position + unit forward: the view
    // DIRECTION must be preserved exactly, the stored point may normalize.
    const auto direction = [](const SceneCamera& c)
    {
        const float dx = c.lookAt[0] - c.position[0];
        const float dy = c.lookAt[1] - c.position[1];
        const float dz = c.lookAt[2] - c.position[2];
        const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
        return std::array<float, 3>{dx / length, dy / length, dz / length};
    };
    const std::array<float, 3> configured = direction(demoCamera());
    const std::array<float, 3> applied = direction(camera);
    for (int axis = 0; axis < 3; ++axis)
    {
        REQUIRE(camera.position[axis] == Catch::Approx(demoCamera().position[axis]).margin(1e-5f));
        REQUIRE(applied[axis] == Catch::Approx(configured[axis]).margin(1e-4f));
    }

    // A frame with no input changes nothing.
    controller.update(&input, 0.016, logger);
    controller.applyToCamera(camera);
    for (int axis = 0; axis < 3; ++axis)
    {
        REQUIRE(camera.position[axis] == Catch::Approx(demoCamera().position[axis]).margin(1e-5f));
    }
}

TEST_CASE("fly controller moves along the view direction with W", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    // Normalize the start: look straight down -Z from the origin.
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);

    input.keys.insert(KeyCode::W);
    controller.update(&input, 0.5, logger); // 4 u/s * 0.5 s = 2 units
    controller.applyToCamera(camera);

    REQUIRE(camera.position[0] == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(camera.position[1] == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(camera.position[2] == Catch::Approx(-2.0f).margin(1e-4f));
}

TEST_CASE("fly controller strafes level independent of pitch", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);

    input.keys.insert(KeyCode::D);
    controller.update(&input, 0.25, logger); // 1 unit to the right (+X)
    controller.applyToCamera(camera);

    REQUIRE(camera.position[0] == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(camera.position[1] == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(camera.position[2] == Catch::Approx(0.0f).margin(1e-5f));
}

TEST_CASE("fly controller rises with Space and sinks with Control", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);

    input.keys.insert(KeyCode::Space);
    controller.update(&input, 1.0, logger);
    controller.applyToCamera(camera);
    REQUIRE(camera.position[1] == Catch::Approx(4.0f).margin(1e-4f));

    input.keys.clear();
    input.keys.insert(KeyCode::LeftControl);
    controller.update(&input, 1.0, logger);
    controller.applyToCamera(camera);
    REQUIRE(camera.position[1] == Catch::Approx(0.0f).margin(1e-4f));
}

TEST_CASE("fly controller boost multiplies speed", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);

    input.keys.insert(KeyCode::W);
    input.keys.insert(KeyCode::LeftShift);
    controller.update(&input, 0.5, logger); // 4 * 4 * 0.5 = 8 units
    controller.applyToCamera(camera);
    REQUIRE(camera.position[2] == Catch::Approx(-8.0f).margin(1e-4f));
}

TEST_CASE("mouse look turns the view right and up", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);

    input.setCursorCaptured(true);
    input.mouseX = 100.0; // right: yaw increases, view turns towards +X
    input.mouseY = -100.0; // up: pitch increases
    controller.update(&input, 0.016, logger);
    controller.applyToCamera(camera);

    const float deltaX = camera.lookAt[0] - camera.position[0];
    const float deltaY = camera.lookAt[1] - camera.position[1];
    const float deltaZ = camera.lookAt[2] - camera.position[2];
    REQUIRE(deltaX > 0.05f); // turned towards +X
    REQUIRE(deltaY > 0.05f); // pitched up
    // Looking right of -Z tilts forward z negative with a magnitude below 1.
    REQUIRE(deltaZ < -0.05f);
}

TEST_CASE("mouse deltas are consumed exactly once", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    controller.syncFromCamera(camera);
    input.setCursorCaptured(true);

    input.mouseX = 100.0;
    input.mouseY = 0.0;
    controller.update(&input, 0.016, logger);
    controller.applyToCamera(camera);
    const float yawAfterFirst = std::atan2(camera.lookAt[0], -camera.lookAt[2]);

    // Second frame with no new events: the view must not keep turning.
    controller.update(&input, 0.016, logger);
    controller.applyToCamera(camera);
    const float yawAfterSecond = std::atan2(camera.lookAt[0], -camera.lookAt[2]);
    REQUIRE(yawAfterSecond == Catch::Approx(yawAfterFirst).margin(1e-6f));
}

TEST_CASE("wheel scales the fly speed within bounds", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.lookAt = {0.0f, 0.0f, -1.0f};
    camera.flySpeed = 4.0f;
    controller.syncFromCamera(camera);

    input.scrollY = 1.0;
    controller.update(&input, 0.016, logger); // 4 * 1.15
    input.keys.insert(KeyCode::W);
    controller.update(&input, 1.0, logger);
    controller.applyToCamera(camera);
    REQUIRE(camera.position[2] == Catch::Approx(-4.0f * 1.15f).margin(0.01f));
}

TEST_CASE("capture toggles through F and Escape and click recaptures", "[camera]")
{
    StubInput input;
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    controller.syncFromCamera(camera);

    // F press captures.
    input.keys.insert(KeyCode::F);
    controller.update(&input, 0.016, logger);
    REQUIRE(input.isCursorCaptured());
    // Holding F across frames must not toggle again (edge triggered).
    controller.update(&input, 0.016, logger);
    REQUIRE(input.isCursorCaptured());
    // Release F, press Escape: releases.
    input.keys.clear();
    input.keys.insert(KeyCode::Escape);
    controller.update(&input, 0.016, logger);
    REQUIRE_FALSE(input.isCursorCaptured());
    // Click recaptures.
    input.keys.clear();
    input.buttons.insert(MouseButton::Left);
    controller.update(&input, 0.016, logger);
    REQUIRE(input.isCursorCaptured());
}

TEST_CASE("null input keeps the camera exactly where it is", "[camera]")
{
    NullLogger logger;
    FlyCameraController controller;

    SceneCamera camera = demoCamera();
    controller.syncFromCamera(camera);
    controller.update(nullptr, 0.016, logger);
    controller.applyToCamera(camera);
    for (int axis = 0; axis < 3; ++axis)
    {
        REQUIRE(camera.position[axis] == Catch::Approx(demoCamera().position[axis]).margin(1e-5f));
    }
}
