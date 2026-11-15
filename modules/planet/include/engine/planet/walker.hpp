// Kinematic surface walking for the planet module.
//
// SurfaceWalker moves a point-feet character across the planet's terrain
// with planet-centric gravity: "down" is always toward the planet center,
// "up" is the radial direction at the character's position, and horizontal
// movement lives in the tangent plane (the classic approach for spherical
// worlds - see how gravity, the up axis and the movement axes are derived
// below). Terrain collision queries the SAME pure density field the mesher
// samples: a short march-and-bisect along the local radial direction finds
// the ground directly beneath the feet, which keeps overhang ledges and
// (optional) cave floors walkable without any collision proxy geometry.
//
// This answers "do you need a physics module to walk on the planet?": no.
// A full rigid-body stage (contact manifolds, solvers, broadphase) solves a
// problem the field answers directly with a handful of pure evaluations per
// frame. The controller here is deliberately KINEMATIC - velocities are
// integrated against the field, not pushed out by contact forces - which
// keeps it deterministic (fixed inputs produce bit-identical walks, so the
// self-test can assert on the whole path) and independent of any other
// module.
//
// The walk parameters come from PlanetGenerationParams (the [planet]
// configuration section, walker_* keys), so tuning needs no code changes.

#pragma once

#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>


namespace engine
{
namespace planet
{

// One frame of movement intent, in the walker's local frame. `forward` and
// `right` are axis weights in [-1, 1]; `heading` is the WORLD-space direction
// the forward axis should point along (any non-degenerate vector - it is
// projected onto the tangent plane at the walker's position, so a camera view
// direction can be passed as is). `jump` requests a jump from the ground.
struct WalkIntent
{
    double forward      = 0.0;
    double right        = 0.0;
    Vec3   heading      = {};
    bool   jump         = false;
    bool   sprint       = false;
};

// State of the walker after an update; consumers read position for the LOD
// eye, eyePosition for the camera, and the flags for feedback/UI.
struct WalkState
{
    Vec3  positionMeters = {};        // feet position, planet-local
    Vec3  up             = {0.0, 1.0, 0.0}; // radial up at the feet
    Vec3  eyePosition    = {};        // feet + up * eyeHeight
    double groundRadius  = 0.0;       // radius of the ground under the feet (last known)
    double speed         = 0.0;       // horizontal speed this frame (meters/second)
    bool   grounded      = false;     // standing on terrain
    bool   swimming      = false;     // floating at the sea surface
};

class SurfaceWalker final
{
public:
    // params must be validate()d; field must be built from the same params.
    // The walker only reads both - any number of walkers can share one field.
    SurfaceWalker(const PlanetGenerationParams &params, const DensityField &field);

    // Places the feet on the OUTERMOST surface along `direction` (the terrain
    // top, not an overhang underside or cave floor) and clears velocity.
    void placeOnSurface(const Vec3 &unitDirection);

    // Places the feet at an arbitrary planet-local point (buried positions
    // pop out to the first surface above them on the next update).
    void teleport(const Vec3 &position);

    // Advances one frame. deltaSeconds is clamped to a safe range so a
    // stalled host cannot teleport the character through terrain.
    void update(double deltaSeconds, const WalkIntent &intent);

    const WalkState &state() const
    {
        return state_;
    }

    // Highest terrain radius along a unit direction (the first crossing a
    // descending probe from open sky hits). Exposed for consumers that need
    // to place objects or cameras on the terrain top.
    double topSurfaceRadius(const Vec3 &unitDirection) const;

private:
    // Ground search: walks along -up from `position` and returns the radius
    // of the first solid surface within `maxDepthMeters` (false = open
    // sky below, keep falling). Positions inside solid rock search outward
    // instead and report the first surface above.
    bool groundRadiusWithin(const Vec3 &position, double maxDepthMeters, double &outRadius) const;

    // One integration slice of update() (see kMaxDeltaSeconds there).
    void step(double deltaSeconds, const WalkIntent &intent);

    const PlanetGenerationParams &params_;
    const DensityField           &field_;
    WalkState                     state_{};
    double                        verticalSpeed_ = 0.0; // along up (positive = rising)
};

} // namespace planet
} // namespace engine

