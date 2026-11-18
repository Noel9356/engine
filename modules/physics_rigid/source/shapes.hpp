// shapes.hpp - runtime shape representation, mass properties, world-space
// AABBs, and the narrowphase pair routines. A Shape is the validated copy of
// a ShapeDescription; all collision routines take world-space transforms and
// fill a ContactManifold whose normal points from B toward A (the direction
// that pushes A out of B) with separation values in meters (<= 0 touching).

#pragma once

#include "math.hpp"
#include <engine/physics_rigid/capabilities.hpp>

namespace engine::physics_rigid
{

// Validated shape data derived from a ShapeDescription at body creation.
struct Shape
{
    ShapeKind kind = ShapeKind::Sphere;

    double sphereRadius = 0.5;

    double boxHalfExtents[3] = {0.5, 0.5, 0.5};

    double capsuleRadius     = 0.25;
    double capsuleHalfLength = 0.5;

    double cylinderRadius     = 0.5;
    double cylinderHalfHeight = 0.5;

    // Plane fields in world space; the body transform is ignored for planes.
    math::Vec3 planeNormal;
    double     planeOffset = 0.0;
};

// Validates and copies a description into a runtime shape; returns false and
// leaves the output untouched on non-finite or negative extents.
bool makeShape(const ShapeDescription &description, Shape &outShape);

// One contact point: world-space anchors on each body. pointA and pointB
// coincide in the ideal solve; the solver pulls them together.
struct ContactPoint
{
    math::Vec3 pointA;
    math::Vec3 pointB;
};

// A narrowphase result for one body pair.
struct ContactManifold
{
    math::Vec3    normal; // from B to A, unit length
    std::uint32_t pointCount = 0;
    ContactPoint  points[8];
};

// Mass properties of a solid shape: total mass and the local-frame inertia
// tensor diagonal (the sphere, box, and capsule principal axes coincide with
// the shape's local axes). density > 0 scales the shape volume; otherwise the
// explicit mass is used. Returns false for non-positive mass results.
bool computeMassProperties(
    const Shape &shape, double density, double explicitMass, double &outMass, math::Vec3 &outInertiaDiagonal);

// Shape volume in cubic meters (0 for planes).
double computeShapeVolume(const Shape &shape);

// World-space AABB of a solid shape; planes report an inverted (empty) box.
void computeShapeAabb(
    const Shape &shape, const RigidTransform &transform, math::Vec3 &outMinimum, math::Vec3 &outMaximum);

// World-space capsule segment endpoints (transformed local +Y segment).
void computeCapsuleSegment(
    const Shape &shape, const RigidTransform &transform, math::Vec3 &outEndpointA, math::Vec3 &outEndpointB);

// Narrowphase dispatcher for two solid shapes (never planes). Returns true
// when the shapes are within margin of each other (at least one contact
// point); separations may be positive up to the margin (speculative
// contacts). A margin of zero keeps exact touching only.
bool collideShapes(const Shape &shapeA, const RigidTransform &transformA, const Shape &shapeB,
    const RigidTransform &transformB, double margin, ContactManifold &outManifold);

// Plane (half-space x dot normal = offset, normal unit length) against a
// solid shape within margin. The plane acts as body B; the manifold normal
// points from the plane toward the solid body.
bool collidePlaneSolid(const math::Vec3 &planeNormal, double planeOffset, const Shape &solidShape,
    const RigidTransform &solidTransform, double margin, ContactManifold &outManifold);

// Closest-hit ray casts. Return the entry distance in (0, maximumDistance]
// through outDistance when the ray hits; direction need not be normalized.
bool rayCastSphere(const RigidTransform &transform, double radius, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance);
bool rayCastBox(const RigidTransform &transform, const double halfExtents[3], const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance);
bool rayCastCapsule(const RigidTransform &transform, double radius, double halfLength, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance);
bool rayCastCylinder(const RigidTransform &transform, double radius, double halfHeight, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance);
bool rayCastPlane(const math::Vec3 &planeNormal, double planeOffset, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance);

// Bounding sphere radius of a solid shape (0 for planes): the conservative
// sweep proxy radius used by continuous collision detection and sphere casts.
double computeShapeBoundingRadius(const Shape &shape);

// Sphere sweeps (linear shape casts). A sphere of the given radius moves from
// `start` along `delta` (the sweep is clipped to that segment); on contact the
// functions write the travelled distance in (0, length(delta)] and the world
// contact normal facing back against the motion. The box sweep tests the box
// inflated by the query radius (a conservative rounded-corner approximation),
// so a sweep may stop slightly before grazing a corner but never tunnels.
bool sweepSphereSolid(const math::Vec3 &start, double radius, const math::Vec3 &delta, const Shape &solidShape,
    const RigidTransform &solidTransform, double &outDistance, math::Vec3 &outNormal);
bool sweepSpherePlane(const math::Vec3 &start, double radius, const math::Vec3 &delta, const math::Vec3 &planeNormal,
    double planeOffset, double &outDistance, math::Vec3 &outNormal);

// Deepest sphere-overlap resolution against one solid shape: writes the push
// direction (away from the shape, toward the query center) and penetration
// depth when they overlap. For planes use collideSpherePlane below.
bool collideSphereSolid(const math::Vec3 &center, double radius, const Shape &solidShape,
    const RigidTransform &solidTransform, math::Vec3 &outNormal, double &outDepth);
bool collideSpherePlane(const math::Vec3 &center, double radius, const math::Vec3 &planeNormal, double planeOffset,
    math::Vec3 &outNormal, double &outDepth);

} // namespace engine::physics_rigid
