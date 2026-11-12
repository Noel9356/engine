// User-configurable camera keyboard mapping for the character demo.
#ifndef ENGINE_DEMO_CAMERA_BINDINGS_HPP
#define ENGINE_DEMO_CAMERA_BINDINGS_HPP

#include <engine/kernel/participant_interface.hpp>
#include <engine/window/capabilities.hpp>

#include <cstdint>

namespace demo
{
struct DemoCameraKeyBindings
{
    engine::window::Key moveForward = engine::window::Key::W;
    engine::window::Key moveBackward = engine::window::Key::S;
    engine::window::Key moveLeft = engine::window::Key::A;
    engine::window::Key moveRight = engine::window::Key::D;
    engine::window::Key moveUp = engine::window::Key::Space;
    engine::window::Key moveDown = engine::window::Key::LeftControl;
    engine::window::Key orbitZoomIn = engine::window::Key::Space;
    engine::window::Key orbitZoomOut = engine::window::Key::LeftControl;
    engine::window::Key sprint = engine::window::Key::LeftShift;
    engine::window::Key lookLeft = engine::window::Key::Left;
    engine::window::Key lookRight = engine::window::Key::Right;
    engine::window::Key lookUp = engine::window::Key::Up;
    engine::window::Key lookDown = engine::window::Key::Down;
};

std::uint64_t keyMaskBit(engine::window::Key key) noexcept;
const char *windowKeyName(engine::window::Key key) noexcept;
DemoCameraKeyBindings loadCameraKeyBindings(engine::ParticipantContext &context);
} // namespace demo

#endif
