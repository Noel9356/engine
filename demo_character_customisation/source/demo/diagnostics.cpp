// diagnostics.cpp - printable names for module error codes and device classes.
//
// Pure lookup tables over the capability enums so error paths in main.cpp can
// log something readable; nothing here owns state.

#include "diagnostics.hpp"

#include <cstdint>
#include <iostream>

namespace demo
{

namespace window       = engine::window;
namespace renderer       = engine::renderer;
namespace camera       = engine::camera;
namespace render_graph = engine::render_graph;

const char *windowErrorName(window::WindowError error)
{
    switch (error)
    {
    case window::WindowError::None:
        return "none";
    case window::WindowError::BackendUnavailable:
        return "backend unavailable";
    case window::WindowError::CreationFailed:
        return "creation failed";
    case window::WindowError::InvalidHandle:
        return "invalid handle";
    case window::WindowError::Unsupported:
        return "unsupported";
    }
    return "unknown";
}

const char *renderErrorName(renderer::RenderError error)
{
    switch (error)
    {
    case renderer::RenderError::None:
        return "none";
    case renderer::RenderError::BackendUnavailable:
        return "backend unavailable";
    case renderer::RenderError::CreationFailed:
        return "creation failed";
    case renderer::RenderError::InvalidHandle:
        return "invalid handle";
    case renderer::RenderError::InvalidMesh:
        return "invalid mesh";
    case renderer::RenderError::InvalidTexture:
        return "invalid texture";
    case renderer::RenderError::Unsupported:
        return "unsupported";
    }
    return "unknown";
}

const char *cameraErrorName(camera::CameraError error)
{
    switch (error)
    {
    case camera::CameraError::None:
        return "none";
    case camera::CameraError::CreationFailed:
        return "creation failed";
    case camera::CameraError::InvalidHandle:
        return "invalid handle";
    }
    return "unknown";
}

const char *renderGraphErrorName(render_graph::RenderGraphError error)
{
    switch (error)
    {
    case render_graph::RenderGraphError::None:
        return "none";
    case render_graph::RenderGraphError::InvalidInput:
        return "invalid input";
    case render_graph::RenderGraphError::CapacityExceeded:
        return "capacity exceeded";
    case render_graph::RenderGraphError::DependencyCycle:
        return "dependency cycle";
    case render_graph::RenderGraphError::BudgetExceeded:
        return "budget exceeded";
    case render_graph::RenderGraphError::ExecutionFailed:
        return "execution failed";
    }
    return "unknown";
}

const char *deviceClassName(renderer::DeviceClass deviceClass)
{
    switch (deviceClass)
    {
    case renderer::DeviceClass::Unknown:
        return "unknown";
    case renderer::DeviceClass::DiscreteGpu:
        return "discrete GPU";
    case renderer::DeviceClass::IntegratedGpu:
        return "integrated GPU";
    case renderer::DeviceClass::VirtualGpu:
        return "virtual GPU";
    case renderer::DeviceClass::Cpu:
        return "CPU";
    }
    return "unknown";
}

// Reports an ABI mismatch on stderr (the demo aborts startup when it fails).
bool capabilityAbiMatches(const char *name, std::uint32_t actual, std::uint32_t expected)
{
    if (actual == expected)
    {
        return true;
    }
    std::cerr << "demo: capability '" << name << "' has ABI " << actual << ", expected " << expected << "\n";
    return false;
}

} // namespace demo
