// diagnostics.hpp - shared diagnostic helpers for the demo application.
//
// Maps capability error codes and device classes to human-readable names and
// provides the capability ABI sanity check used during demo startup.

#ifndef ENGINE_DEMO_DIAGNOSTICS_HPP
#define ENGINE_DEMO_DIAGNOSTICS_HPP

#include <engine/camera/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <cstdint>

namespace demo
{

// Error-code to printable-name lookups; unknown values return "unknown".
[[nodiscard]] const char *windowErrorName(engine::window::WindowError error);
[[nodiscard]] const char *renderErrorName(engine::renderer::RenderError error);
[[nodiscard]] const char *cameraErrorName(engine::camera::CameraError error);
[[nodiscard]] const char *renderGraphErrorName(engine::render_graph::RenderGraphError error);
// DeviceClass to display name (e.g. "discrete GPU"); used in the startup report.
[[nodiscard]] const char *deviceClassName(engine::renderer::DeviceClass deviceClass);
// Returns true when the acquired capability matches the ABI the demo compiled
// against; otherwise logs the mismatch to stderr and returns false.
[[nodiscard]] bool capabilityAbiMatches(const char *name, std::uint32_t actual, std::uint32_t expected);

} // namespace demo

#endif
