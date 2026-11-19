// Shared collider metric vocabulary for the engine's simulation modules.
//
// Modules that push forces onto rigid bodies (wind drag, buoyancy, radial
// blasts) re-derive the same per-shape metrics: a volume estimate, a
// reference drag area, and - for surface-sampled forces - a small fixed
// set of body sample points. This header keeps one canonical definition of
// each metric, covering every solid shape the physics_rigid capability
// ships (sphere, box, capsule, cylinder; planes carry no volume and never
// receive surface-sampled forces). Header-only and dependency-free apart
// from the physics_rigid capability vocabulary; everything is pure and
// allocation-free apart from the caller-owned sample vector.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>

#include <cmath>
#include <vector>

namespace engine::util
{

// Volume of one solid shape in cubic meters (0 for planes). The formulas
// match the rigid module's exact shape volumes.
inline double colliderVolume(const engine::physics_rigid::ShapeDescription &shape)
{
    using engine::physics_rigid::ShapeKind;
    const double pi = engine::physics_rigid::math::kPi;
    switch (shape.kind)
    {
    case ShapeKind::Sphere:
        return (4.0 / 3.0) * pi * shape.sphereRadius * shape.sphereRadius * shape.sphereRadius;
    case ShapeKind::Box:
        return 8.0 * shape.boxHalfExtents[0] * shape.boxHalfExtents[1] * shape.boxHalfExtents[2];
    case ShapeKind::Capsule: {
        const double radius = shape.capsuleRadius;
        return pi * radius * radius * (2.0 * shape.capsuleHalfLength) + (4.0 / 3.0) * pi * radius * radius * radius;
    }
    case ShapeKind::Cylinder:
        return 2.0 * shape.cylinderHalfHeight * pi * shape.cylinderRadius * shape.cylinderRadius;
    case ShapeKind::Plane:
    default:
        return 0.0;
    }
}

// Reference area and drag coefficient of one solid shape for the quadratic
// drag law (force = 0.5 * density * coefficient * area * speed^2). The
// coefficients follow the standard bluff-body table: a sphere crosses
// smooth flow at 0.47, a box presents roughly constant separated flow at
// 1.05, and rounded elongated bodies (capsules, cylinders) sit near 0.82.
// The reference area is the shape's average projected cross-section.
struct ColliderAerodynamics
{
    double referenceArea   = 0.0;
    double dragCoefficient = 0.0;
};

inline ColliderAerodynamics colliderAerodynamics(const engine::physics_rigid::ShapeDescription &shape)
{
    using engine::physics_rigid::ShapeKind;
    const double pi = engine::physics_rigid::math::kPi;
    ColliderAerodynamics aerodynamics;
    switch (shape.kind)
    {
    case ShapeKind::Sphere:
        aerodynamics.referenceArea   = pi * shape.sphereRadius * shape.sphereRadius;
        aerodynamics.dragCoefficient = 0.47;
        break;
    case ShapeKind::Box: {
        // Average projected face area of the full box edges.
        const double width  = shape.boxHalfExtents[0] * 2.0;
        const double height = shape.boxHalfExtents[1] * 2.0;
        const double depth  = shape.boxHalfExtents[2] * 2.0;
        aerodynamics.referenceArea   = (width * height + height * depth + width * depth) / 3.0;
        aerodynamics.dragCoefficient = 1.05;
        break;
    }
    case ShapeKind::Capsule:
        aerodynamics.referenceArea = pi * shape.capsuleRadius * shape.capsuleRadius
                                     + 2.0 * shape.capsuleRadius * (2.0 * shape.capsuleHalfLength);
        aerodynamics.dragCoefficient = 0.82;
        break;
    case ShapeKind::Cylinder:
        aerodynamics.referenceArea = pi * shape.cylinderRadius * shape.cylinderRadius
                                     + 2.0 * shape.cylinderRadius * (2.0 * shape.cylinderHalfHeight);
        aerodynamics.dragCoefficient = 0.82;
        break;
    case ShapeKind::Plane:
    default:
        break;
    }
    return aerodynamics;
}

// Appends the body sample points the surface-sampled force passes evaluate
// (submersion fractions, submerged centroids). The sets are small, fixed,
// and deterministic: a sphere keeps its center and six half-radius axis
// points; a box its center and eight rotated corner points pulled in to
// 0.7 of the half extents; a capsule five stations along its axis plus four
// half-radius lateral offsets; a cylinder five stations along its axis,
// four lateral offsets, and four rim points per cap at quarter angles.
// Planes append nothing (unbounded half-spaces receive no surface-sampled
// forces).
inline void appendColliderSamplePoints(const engine::physics_rigid::ShapeDescription &shape,
    const engine::physics_rigid::RigidTransform &transform,
    std::vector<engine::physics_rigid::math::Vec3> &outPoints)
{
    using engine::physics_rigid::ShapeKind;
    namespace math = engine::physics_rigid::math;
    const math::Vec3 center = transform.position;
    switch (shape.kind)
    {
    case ShapeKind::Sphere: {
        const double radius = shape.sphereRadius * 0.5;
        outPoints.push_back(center);
        outPoints.push_back(center + math::Vec3{radius, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{-radius, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{0.0, radius, 0.0});
        outPoints.push_back(center + math::Vec3{0.0, -radius, 0.0});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, radius});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, -radius});
        break;
    }
    case ShapeKind::Box: {
        const double hx = shape.boxHalfExtents[0] * 0.7;
        const double hy = shape.boxHalfExtents[1] * 0.7;
        const double hz = shape.boxHalfExtents[2] * 0.7;
        for (const double sx : {-1.0, 1.0})
        {
            for (const double sy : {-1.0, 1.0})
            {
                for (const double sz : {-1.0, 1.0})
                {
                    outPoints.push_back(center + math::rotate(transform.rotation, math::Vec3{hx * sx, hy * sy, hz * sz}));
                }
            }
        }
        outPoints.push_back(center);
        break;
    }
    case ShapeKind::Capsule: {
        const math::Vec3 axis
            = math::rotate(transform.rotation, math::Vec3{0.0, shape.capsuleHalfLength, 0.0});
        for (const double fraction : {-1.0, -0.5, 0.0, 0.5, 1.0})
        {
            outPoints.push_back(center + axis * fraction);
        }
        const double lateral = shape.capsuleRadius * 0.5;
        outPoints.push_back(center + math::Vec3{lateral, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{-lateral, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, lateral});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, -lateral});
        break;
    }
    case ShapeKind::Cylinder: {
        const math::Vec3 axis
            = math::rotate(transform.rotation, math::Vec3{0.0, shape.cylinderHalfHeight, 0.0});
        for (const double fraction : {-1.0, -0.5, 0.0, 0.5, 1.0})
        {
            outPoints.push_back(center + axis * fraction);
        }
        const double lateral = shape.cylinderRadius * 0.5;
        outPoints.push_back(center + math::Vec3{lateral, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{-lateral, 0.0, 0.0});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, lateral});
        outPoints.push_back(center + math::Vec3{0.0, 0.0, -lateral});
        // Four rim points per cap at quarter angles keep the caps weighted,
        // so a cylinder standing upright samples its submerged base before
        // the axis reaches the surface.
        const math::Vec3 localAxis{0.0, shape.cylinderHalfHeight, 0.0};
        const double     rimRadius = shape.cylinderRadius * 0.7;
        for (const double capSign : {-1.0, 1.0})
        {
            for (const double rimX : {-rimRadius, rimRadius})
            {
                for (const double rimZ : {-rimRadius, rimRadius})
                {
                    outPoints.push_back(
                        center + math::rotate(transform.rotation, math::Vec3{rimX, localAxis.y * capSign, rimZ}));
                }
            }
        }
        break;
    }
    case ShapeKind::Plane:
    default:
        break;
    }
}

} // namespace engine::util
