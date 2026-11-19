// Optional-error out-parameter convention shared by the renderer module's
// capability entries and helpers (nullptr is always allowed).

#pragma once

#include <engine/renderer/capabilities.hpp>

namespace engine
{
namespace renderer
{

// Stores an error code through an optional out-parameter (nullptr allowed).
inline void reportError(RenderError *error, RenderError value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

} // namespace renderer
} // namespace engine
