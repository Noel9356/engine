// shapes.cpp - shape validation, mass properties, AABBs, and the narrowphase
// pair routines. Collision conventions: manifold normals point from body B
// toward body A, and every routine is allocation-free so stepping stays
// deterministic.

#include "shapes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace engine::physics_rigid
{
namespace
{

constexpr double kPi      = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

bool finiteExtents(const double *values, std::uint32_t count)
{
    for (std::uint32_t index = 0; index < count; ++index)
    {
        if (!std::isfinite(values[index]) || values[index] <= 0.0)
        {
            return false;
        }
    }
    return true;
}

bool validateShape(const ShapeDescription &description)
{
    switch (description.kind)
    {
    case ShapeKind::Sphere:
        return std::isfinite(description.sphereRadius) && description.sphereRadius > 0.0;
    case ShapeKind::Box:
        return finiteExtents(description.boxHalfExtents, 3);
    case ShapeKind::Capsule:
        return std::isfinite(description.capsuleRadius) && description.capsuleRadius > 0.0
            && std::isfinite(description.capsuleHalfLength) && description.capsuleHalfLength > 0.0;
    case ShapeKind::Cylinder:
        return std::isfinite(description.cylinderRadius) && description.cylinderRadius > 0.0
            && std::isfinite(description.cylinderHalfHeight) && description.cylinderHalfHeight > 0.0;
    case ShapeKind::Plane: {
        const math::Vec3 normal{description.planeNormal[0], description.planeNormal[1], description.planeNormal[2]};
        return math::isFinite(normal) && math::lengthSquared(normal) > kEpsilon
            && std::isfinite(description.planeOffset);
    }
    default:
        return false;
    }
}

// Closest point between the segments p1 + s * d1 (s in [0, 1]) and
// p2 + t * d2 (t in [0, 1]); writes both points. For parallel segments the
// first segment's start is used as the sample, which callers tolerate.
void closestSegmentSegment(const math::Vec3 &p1, const math::Vec3 &d1, const math::Vec3 &p2, const math::Vec3 &d2,
    math::Vec3 &outPoint1, math::Vec3 &outPoint2)
{
    const math::Vec3 offset      = p2 - p1;
    const double     a           = math::dot(d1, d1);
    const double     e           = math::dot(d2, d2);
    const double     f           = math::dot(d2, offset);
    const double     c           = math::dot(d1, offset);
    const double     b           = math::dot(d1, d2);
    const double     denominator = a * e - b * b;
    double           s           = denominator > kEpsilon ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0.0;
    double           t           = (b * s + f) / e;
    if (t < 0.0)
    {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
    }
    else if (t > 1.0)
    {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
    }
    outPoint1 = p1 + d1 * s;
    outPoint2 = p2 + d2 * t;
}

// Keeps the deepest maximumPoints contacts (most negative separation).
void trimManifold(ContactManifold &manifold, std::uint32_t maximumPoints)
{
    if (manifold.pointCount <= maximumPoints)
    {
        return;
    }
    double separations[8];
    for (std::uint32_t index = 0; index < manifold.pointCount; ++index)
    {
        separations[index] = math::dot(manifold.points[index].pointA - manifold.points[index].pointB, manifold.normal);
    }
    for (std::uint32_t sorted = 0; sorted < maximumPoints; ++sorted)
    {
        std::uint32_t deepest = sorted;
        for (std::uint32_t index = sorted + 1; index < manifold.pointCount; ++index)
        {
            if (separations[index] < separations[deepest])
            {
                deepest = index;
            }
        }
        std::swap(manifold.points[sorted], manifold.points[deepest]);
        std::swap(separations[sorted], separations[deepest]);
    }
    manifold.pointCount = maximumPoints;
}

void appendContact(ContactManifold &manifold, const math::Vec3 &pointA, const math::Vec3 &pointB)
{
    if (manifold.pointCount < 8)
    {
        manifold.points[manifold.pointCount].pointA = pointA;
        manifold.points[manifold.pointCount].pointB = pointB;
        ++manifold.pointCount;
    }
}

// Flips a manifold produced with the roles of A and B exchanged into the
// canonical B-toward-A convention.
void flipManifold(ContactManifold &manifold)
{
    manifold.normal = manifold.normal * -1.0;
    for (std::uint32_t index = 0; index < manifold.pointCount; ++index)
    {
        std::swap(manifold.points[index].pointA, manifold.points[index].pointB);
    }
}

bool sphereSphere(const RigidTransform &transformA, double radiusA, const RigidTransform &transformB, double radiusB,
    double margin, ContactManifold &manifold)
{
    const math::Vec3 delta           = transformA.position - transformB.position;
    const double     combined        = radiusA + radiusB + margin;
    const double     distanceSquared = math::lengthSquared(delta);
    if (distanceSquared >= combined * combined)
    {
        return false;
    }
    const double     distance = std::sqrt(distanceSquared);
    const math::Vec3 normal   = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
    manifold.normal           = normal;
    manifold.pointCount       = 0;
    const math::Vec3 midpoint = transformB.position + normal * (radiusB - (combined - distance) * 0.5);
    appendContact(manifold, midpoint, midpoint);
    return true;
}

bool sphereBox(const RigidTransform &transformA, double radiusA, const RigidTransform &transformB,
    const double halfExtents[3], double margin, ContactManifold &manifold)
{
    const math::Vec3 local = math::inverseRotate(transformB.rotation, transformA.position - transformB.position);
    const math::Vec3 clamped{
        std::clamp(local.x, -halfExtents[0], halfExtents[0]),
        std::clamp(local.y, -halfExtents[1], halfExtents[1]),
        std::clamp(local.z, -halfExtents[2], halfExtents[2])};
    math::Vec3 localNormal;
    double     depth = 0.0;
    math::Vec3 closestPointLocal;
    if (math::distanceSquared(local, clamped) <= kEpsilon * kEpsilon)
    {
        // Sphere center inside the box: push out through the nearest face.
        const double faceDistances[3] = {
            halfExtents[0] - std::abs(local.x), halfExtents[1] - std::abs(local.y), halfExtents[2] - std::abs(local.z)};
        std::uint32_t nearestAxis = 0;
        for (std::uint32_t axis = 1; axis < 3; ++axis)
        {
            if (faceDistances[axis] < faceDistances[nearestAxis])
            {
                nearestAxis = axis;
            }
        }
        localNormal                           = math::Vec3{};
        math::component(localNormal, nearestAxis) = math::component(local, nearestAxis) >= 0.0 ? 1.0 : -1.0;
        depth                                 = radiusA + faceDistances[nearestAxis];
        closestPointLocal                     = localNormal * halfExtents[nearestAxis];
    }
    else
    {
        const math::Vec3 delta           = local - clamped;
        const double     distanceSquared = math::lengthSquared(delta);
        if (distanceSquared >= (radiusA + margin) * (radiusA + margin))
        {
            return false;
        }
        const double distance = std::sqrt(distanceSquared);
        localNormal           = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
        depth                 = radiusA - distance;
        closestPointLocal     = clamped;
    }
    manifold.normal         = math::rotate(transformB.rotation, localNormal);
    manifold.pointCount     = 0;
    const math::Vec3 pointB = transformB.position + math::rotate(transformB.rotation, closestPointLocal);
    const math::Vec3 pointA = pointB + manifold.normal * depth;
    appendContact(manifold, pointA, pointB);
    return true;
}

// Sphere against the sphere swept along a capsule's segment; the segment
// parameter runs in [0, 1] from endpoint -Y to +Y.
bool sphereCapsuleSegment(const RigidTransform &transformA, double radiusA, const math::Vec3 &start,
    const math::Vec3 &direction, double radiusB, double margin, ContactManifold &manifold)
{
    const double t
        = std::clamp(math::dot(transformA.position - start, direction) / math::dot(direction, direction), 0.0, 1.0);
    RigidTransform segmentTransform{};
    segmentTransform.position = start + direction * t;
    return sphereSphere(transformA, radiusA, segmentTransform, radiusB, margin, manifold);
}

bool sphereCapsule(const RigidTransform &transformA, double radiusA, const RigidTransform &transformB, double radiusB,
    double halfLengthB, double margin, ContactManifold &manifold)
{
    const math::Vec3 axisB = math::rotate(transformB.rotation, math::Vec3{0.0, halfLengthB, 0.0});
    return sphereCapsuleSegment(
        transformA, radiusA, transformB.position - axisB, axisB * 2.0, radiusB, margin, manifold);
}

bool spherePlane(const RigidTransform &transformA, double radiusA, const math::Vec3 &planeNormal, double planeOffset,
    double margin, ContactManifold &manifold)
{
    const double signedDistance = math::dot(transformA.position, planeNormal) - planeOffset;
    if (signedDistance >= radiusA + margin)
    {
        return false;
    }
    manifold.normal                = planeNormal;
    manifold.pointCount            = 0;
    const math::Vec3 pointOnPlane  = transformA.position - planeNormal * signedDistance;
    const math::Vec3 pointOnSphere = transformA.position - planeNormal * radiusA;
    appendContact(manifold, pointOnSphere, pointOnPlane);
    return true;
}

// Full 3D box-box SAT over the six face axes and nine edge axes, then
// reference-face clipping against the incident face polygon. Edge axes are
// only preferred when their separation beats the best face axis by more than
// 5%, which keeps stacks resting on face contacts.
bool boxBox(const RigidTransform &transformA, const double halfExtentsA[3], const RigidTransform &transformB,
    const double halfExtentsB[3], double margin, ContactManifold &manifold)
{
    const math::Mat3 rotationA   = math::makeRotationMatrix(transformA.rotation);
    const math::Mat3 rotationB   = math::makeRotationMatrix(transformB.rotation);
    const math::Vec3 centerDelta = transformB.position - transformA.position;

    const math::Vec3 axesA[3]
        = {math::Vec3{rotationA.elements[0], rotationA.elements[1], rotationA.elements[2]},
            math::Vec3{rotationA.elements[3], rotationA.elements[4], rotationA.elements[5]},
            math::Vec3{rotationA.elements[6], rotationA.elements[7], rotationA.elements[8]}};
    const math::Vec3 axesB[3]
        = {math::Vec3{rotationB.elements[0], rotationB.elements[1], rotationB.elements[2]},
            math::Vec3{rotationB.elements[3], rotationB.elements[4], rotationB.elements[5]},
            math::Vec3{rotationB.elements[6], rotationB.elements[7], rotationB.elements[8]}};

    auto projectRadius = [](const math::Vec3 axes[3], const double extents[3], const math::Vec3 &axis)
    {
        return std::abs(math::dot(axes[0], axis)) * extents[0] + std::abs(math::dot(axes[1], axis)) * extents[1]
             + std::abs(math::dot(axes[2], axis)) * extents[2];
    };

    struct SatResult
    {
        math::Vec3    axis; // signed to point from A toward B
        double        separation = -1.0;
        std::uint32_t source     = 0; // 0..2 face of A, 3..5 face of B, 6..14 edge pair
    };

    auto faceOverlap = [&](std::uint32_t boxIndex, std::uint32_t axisIndex, double &outOverlap)
    {
        const math::Vec3 axis      = boxIndex == 0 ? axesA[axisIndex] : axesB[axisIndex];
        const double     radiusOwn = boxIndex == 0 ? halfExtentsA[axisIndex] : halfExtentsB[axisIndex];
        const double     radiusOther
            = boxIndex == 0 ? projectRadius(axesB, halfExtentsB, axis) : projectRadius(axesA, halfExtentsA, axis);
        const double distance = math::dot(centerDelta, axis);
        outOverlap            = radiusOwn + radiusOther - std::abs(distance);
        return outOverlap < -margin ? false : true;
    };

    SatResult faceBest;
    for (std::uint32_t boxIndex = 0; boxIndex < 2; ++boxIndex)
    {
        for (std::uint32_t axisIndex = 0; axisIndex < 3; ++axisIndex)
        {
            double overlap = 0.0;
            if (!faceOverlap(boxIndex, axisIndex, overlap))
            {
                return false;
            }
            if (faceBest.separation < 0.0 || overlap < faceBest.separation)
            {
                const math::Vec3 axis = boxIndex == 0 ? axesA[axisIndex] : axesB[axisIndex];
                faceBest.axis = math::dot(centerDelta, axis) >= 0.0 ? axis : math::Vec3{-axis.x, -axis.y, -axis.z};
                faceBest.separation = overlap;
                faceBest.source     = boxIndex * 3 + axisIndex;
            }
        }
    }

    SatResult edgeBest;
    for (std::uint32_t axisAIndex = 0; axisAIndex < 3; ++axisAIndex)
    {
        for (std::uint32_t axisBIndex = 0; axisBIndex < 3; ++axisBIndex)
        {
            const math::Vec3 rawAxis = math::cross(axesA[axisAIndex], axesB[axisBIndex]);
            if (math::lengthSquared(rawAxis) < 1.0e-8)
            {
                continue; // near-parallel faces: their face axes already cover them
            }
            const math::Vec3 axis = rawAxis * (1.0 / std::sqrt(math::lengthSquared(rawAxis)));
            const double overlap  = projectRadius(axesA, halfExtentsA, axis) + projectRadius(axesB, halfExtentsB, axis)
                                  - std::abs(math::dot(centerDelta, axis));
            if (overlap < -margin)
            {
                return false;
            }
            if (edgeBest.separation < 0.0 || overlap < edgeBest.separation)
            {
                edgeBest.axis = math::dot(centerDelta, axis) >= 0.0 ? axis : math::Vec3{-axis.x, -axis.y, -axis.z};
                edgeBest.separation = overlap;
                edgeBest.source     = 6 + axisAIndex * 3 + axisBIndex;
            }
        }
    }

    const bool      useEdgeAxis = edgeBest.separation >= 0.0 && edgeBest.separation < faceBest.separation * 1.05;
    const SatResult chosen      = useEdgeAxis ? edgeBest : faceBest;
    // For edge axes either box works as the clipping reference; A is used.
    const bool referenceIsA = chosen.source < 3 || chosen.source >= 6;

    const math::Mat3       &referenceRotation = referenceIsA ? rotationA : rotationB;
    const double           *referenceExtents  = referenceIsA ? halfExtentsA : halfExtentsB;
    const math::Vec3       &referenceCenter   = referenceIsA ? transformA.position : transformB.position;
    const math::Vec3       &incidentCenter    = referenceIsA ? transformB.position : transformA.position;
    const double           *incidentExtents   = referenceIsA ? halfExtentsB : halfExtentsA;
    const math::Quaternion &incidentRotation  = referenceIsA ? transformB.rotation : transformA.rotation;

    // Outward reference normal: points from the reference box toward the
    // incident box.
    math::Vec3 referenceNormal
        = referenceIsA ? chosen.axis : math::Vec3{-chosen.axis.x, -chosen.axis.y, -chosen.axis.z};
    if (math::dot(referenceNormal, incidentCenter - referenceCenter) < 0.0)
    {
        referenceNormal = referenceNormal * -1.0;
    }

    // Reference face axes: the normal axis plus two in-face tangent axes.
    const math::Vec3 referenceLocalNormal
        = math::inverseRotate(referenceIsA ? transformA.rotation : transformB.rotation, referenceNormal);
    std::uint32_t normalAxis = 0;
    {
        const double magnitudes[3]
            = {std::abs(referenceLocalNormal.x), std::abs(referenceLocalNormal.y), std::abs(referenceLocalNormal.z)};
        normalAxis = magnitudes[0] > magnitudes[1] ? (magnitudes[0] > magnitudes[2] ? 0u : 2u)
                                                   : (magnitudes[1] > magnitudes[2] ? 1u : 2u);
    }
    const std::uint32_t tangentAxis   = (normalAxis + 1) % 3;
    const std::uint32_t bitangentAxis = (normalAxis + 2) % 3;

    // Incident face: the incident box's local axis most anti-parallel to the
    // reference normal, plus that box's own two in-face axes.
    const math::Vec3 incidentLocalAntiNormal = math::inverseRotate(incidentRotation, referenceNormal * -1.0);
    std::uint32_t    incidentAxis            = 0;
    {
        const double magnitudes[3]
            = {std::abs(incidentLocalAntiNormal.x),
                std::abs(incidentLocalAntiNormal.y),
                std::abs(incidentLocalAntiNormal.z)};
        incidentAxis = magnitudes[0] > magnitudes[1] ? (magnitudes[0] > magnitudes[2] ? 0u : 2u)
                                                     : (magnitudes[1] > magnitudes[2] ? 1u : 2u);
    }
    const std::uint32_t incidentTangent   = (incidentAxis + 1) % 3;
    const std::uint32_t incidentBitangent = (incidentAxis + 2) % 3;

    math::Vec3 incidentCorners[4];
    {
        const double faceOffset
            = incidentExtents[incidentAxis] * (math::component(incidentLocalAntiNormal, incidentAxis) >= 0.0 ? 1.0 : -1.0);
        std::uint32_t cornerIndex = 0;
        for (std::uint32_t edgeU = 0; edgeU < 2; ++edgeU)
        {
            for (std::uint32_t edgeV = 0; edgeV < 2; ++edgeV)
            {
                math::Vec3 local{};
                math::component(local, incidentAxis)      = faceOffset;
                math::component(local, incidentTangent)   = incidentExtents[incidentTangent] * (edgeU == 0 ? 1.0 : -1.0);
                math::component(local, incidentBitangent) = incidentExtents[incidentBitangent] * (edgeV == 0 ? 1.0 : -1.0);
                incidentCorners[cornerIndex++]        = incidentCenter + math::rotate(incidentRotation, local);
            }
        }
    }

    // Sutherland-Hodgman clip of the incident polygon against the four side
    // planes of the reference face. A convex polygon gains at most two
    // vertices per clip plane, so four planes can grow the quad to twelve
    // vertices before the manifold trim caps the contact count; the clip
    // buffers are sized for that worst case.
    constexpr std::uint32_t kClipCapacity = 16;
    math::Vec3    clipped[kClipCapacity];
    std::uint32_t clippedCount = 4;
    for (std::uint32_t index = 0; index < 4; ++index)
    {
        clipped[index] = incidentCorners[index];
    }
    const math::Vec3 sideAxes[2]
        = {[&]()
            {
                math::Vec3 axis{};
                math::component(axis, tangentAxis) = 1.0;
                return math::multiplyMatrixVector(referenceRotation, axis);
            }(),
            [&]()
            {
                math::Vec3 axis{};
                math::component(axis, bitangentAxis) = 1.0;
                return math::multiplyMatrixVector(referenceRotation, axis);
            }()};
    const double sideExtents[2] = {referenceExtents[tangentAxis], referenceExtents[bitangentAxis]};
    for (std::uint32_t side = 0; side < 2 && clippedCount > 0; ++side)
    {
        for (const double sign : {1.0, -1.0})
        {
            const math::Vec3 clipNormal = sideAxes[side] * sign;
            const double     clipExtent = sideExtents[side];
            math::Vec3       output[kClipCapacity];
            std::uint32_t    outputCount = 0;
            for (std::uint32_t index = 0; index < clippedCount; ++index)
            {
                const math::Vec3 &current  = clipped[index];
                const math::Vec3 &previous = clipped[(index + clippedCount - 1) % clippedCount];
                // Side-plane distances are measured from the reference box's
                // center, never from the world origin.
                const double currentDistance  = math::dot(current - referenceCenter, clipNormal) - clipExtent;
                const double previousDistance = math::dot(previous - referenceCenter, clipNormal) - clipExtent;
                const bool   currentInside    = currentDistance <= 0.0;
                const bool   previousInside   = previousDistance <= 0.0;
                if (currentInside)
                {
                    output[outputCount++] = current;
                }
                if (currentInside != previousInside)
                {
                    const double alpha    = previousDistance / (previousDistance - currentDistance);
                    output[outputCount++] = previous + (current - previous) * alpha;
                }
            }
            for (std::uint32_t index = 0; index < outputCount; ++index)
            {
                clipped[index] = output[index];
            }
            clippedCount = outputCount;
            if (clippedCount == 0)
            {
                break;
            }
        }
    }

    // Manifold normal from B toward A: the outward reference normal runs
    // reference-to-incident, so it flips when A is the incident box.
    manifold.normal
        = referenceIsA ? math::Vec3{-referenceNormal.x, -referenceNormal.y, -referenceNormal.z} : referenceNormal;
    manifold.pointCount                  = 0;
    const math::Vec3 referenceFaceCenter = referenceCenter + referenceNormal * referenceExtents[normalAxis];
    for (std::uint32_t index = 0; index < clippedCount; ++index)
    {
        const double separation = math::dot(clipped[index] - referenceFaceCenter, referenceNormal);
        if (separation <= margin)
        {
            const math::Vec3 projected = clipped[index] - referenceNormal * separation;
            if (referenceIsA)
            {
                appendContact(manifold, projected, clipped[index]);
            }
            else
            {
                appendContact(manifold, clipped[index], projected);
            }
        }
    }
    if (manifold.pointCount == 0)
    {
        return false;
    }
    trimManifold(manifold, 4);
    return true;
}

// Samples the capsule segment as two interior spheres against the box; the
// two-point manifolds keep lying capsules from rocking.
bool boxCapsule(const RigidTransform &transformA, const double halfExtents[3], const RigidTransform &transformB,
    double radiusB, double halfLengthB, double margin, ContactManifold &manifold)
{
    manifold.pointCount       = 0;
    manifold.normal           = math::Vec3{0.0, 1.0, 0.0};
    bool             touching = false;
    const math::Vec3 halfAxis = math::rotate(transformB.rotation, math::Vec3{0.0, halfLengthB * 0.5, 0.0});
    for (std::uint32_t sample = 0; sample < 2; ++sample)
    {
        RigidTransform sphereTransform{};
        sphereTransform.position = transformB.position + (sample == 0 ? halfAxis * -1.0 : halfAxis);
        ContactManifold sampleManifold;
        if (sphereBox(sphereTransform, radiusB, transformA, halfExtents, margin, sampleManifold))
        {
            touching        = true;
            manifold.normal = sampleManifold.normal * -1.0;
            for (std::uint32_t point = 0; point < sampleManifold.pointCount; ++point)
            {
                appendContact(manifold, sampleManifold.points[point].pointB, sampleManifold.points[point].pointA);
            }
        }
    }
    if (!touching)
    {
        return false;
    }
    trimManifold(manifold, 2);
    return true;
}

bool boxPlane(const RigidTransform &transformA, const double halfExtents[3], const math::Vec3 &planeNormal,
    double planeOffset, double margin, ContactManifold &manifold)
{
    const math::Mat3 rotation = math::makeRotationMatrix(transformA.rotation);
    manifold.normal           = planeNormal;
    manifold.pointCount       = 0;
    for (std::uint32_t cornerIndex = 0; cornerIndex < 8; ++cornerIndex)
    {
        const math::Vec3 localCorner{
            cornerIndex & 1u ? halfExtents[0] : -halfExtents[0],
            cornerIndex & 2u ? halfExtents[1] : -halfExtents[1],
            cornerIndex & 4u ? halfExtents[2] : -halfExtents[2]};
        const math::Vec3 corner         = transformA.position + math::multiplyMatrixVector(rotation, localCorner);
        const double     signedDistance = math::dot(corner, planeNormal) - planeOffset;
        if (signedDistance < margin)
        {
            appendContact(manifold, corner, corner - planeNormal * std::min(signedDistance, 0.0));
        }
    }
    if (manifold.pointCount == 0)
    {
        return false;
    }
    trimManifold(manifold, 4);
    return true;
}

bool capsuleCapsule(const RigidTransform &transformA, double radiusA, double halfLengthA,
    const RigidTransform &transformB, double radiusB, double halfLengthB, double margin, ContactManifold &manifold)
{
    const math::Vec3 axisA = math::rotate(transformA.rotation, math::Vec3{0.0, halfLengthA, 0.0});
    const math::Vec3 axisB = math::rotate(transformB.rotation, math::Vec3{0.0, halfLengthB, 0.0});
    math::Vec3       pointA;
    math::Vec3       pointB;
    closestSegmentSegment(
        transformA.position - axisA, axisA * 2.0, transformB.position - axisB, axisB * 2.0, pointA, pointB);
    RigidTransform sphereTransformA{};
    sphereTransformA.position = pointA;
    RigidTransform sphereTransformB{};
    sphereTransformB.position = pointB;
    return sphereSphere(sphereTransformA, radiusA, sphereTransformB, radiusB, margin, manifold);
}

bool capsulePlane(const RigidTransform &transformA, double radiusA, double halfLengthA, const math::Vec3 &planeNormal,
    double planeOffset, double margin, ContactManifold &manifold)
{
    const math::Vec3 axis = math::rotate(transformA.rotation, math::Vec3{0.0, halfLengthA, 0.0});
    manifold.normal       = planeNormal;
    manifold.pointCount   = 0;
    for (std::uint32_t endpoint = 0; endpoint < 2; ++endpoint)
    {
        const math::Vec3 center         = transformA.position + (endpoint == 0 ? axis * -1.0 : axis);
        const double     signedDistance = math::dot(center, planeNormal) - planeOffset;
        if (signedDistance < radiusA + margin)
        {
            appendContact(manifold, center - planeNormal * radiusA, center - planeNormal * std::min(signedDistance, 0.0));
        }
    }
    return manifold.pointCount > 0;
}

// World-space cylinder cap endpoints (transformed local +Y axis).
void cylinderEndpoints(
    const RigidTransform &transform, double halfHeight, math::Vec3 &outEndpointA, math::Vec3 &outEndpointB)
{
    const math::Vec3 axis = math::rotate(transform.rotation, math::Vec3{0.0, halfHeight, 0.0});
    outEndpointA          = transform.position - axis;
    outEndpointB          = transform.position + axis;
}

// Deterministic orthonormal pair spanning the disc plane perpendicular to the
// unit `axis`; the seed branch on the smallest component keeps the basis
// stable for a given axis direction, so sampling order never depends on the
// transform's rotation representation.
void cylinderDiscBasis(const math::Vec3 &axis, math::Vec3 &outTangentU, math::Vec3 &outTangentV)
{
    const math::Vec3 seed = std::abs(axis.x) <= std::abs(axis.y) && std::abs(axis.x) <= std::abs(axis.z)
        ? math::Vec3{1.0, 0.0, 0.0}
        : (std::abs(axis.y) <= std::abs(axis.z) ? math::Vec3{0.0, 1.0, 0.0} : math::Vec3{0.0, 0.0, 1.0});
    outTangentU = math::normalized(math::cross(axis, seed));
    outTangentV = math::cross(axis, outTangentU);
}

// Surface sample points of a cylinder: the two cap centers followed by four
// rim points on each cap rim at fixed quarter angles. The fixed sampling is
// the deterministic primitive approximation for cylinder pairs and plane
// contacts, matching the sampled-sphere idiom used for capsule pairs.
void appendCylinderSurfaceSamples(const RigidTransform &transform, double radius, double halfHeight,
    math::Vec3 outPoints[10])
{
    math::Vec3 endpointA;
    math::Vec3 endpointB;
    cylinderEndpoints(transform, halfHeight, endpointA, endpointB);
    const math::Vec3 axis        = endpointB - endpointA;
    const double     axisLength  = math::length(axis);
    math::Vec3       tangentU;
    math::Vec3       tangentV;
    if (axisLength > kEpsilon)
    {
        cylinderDiscBasis(axis * (1.0 / axisLength), tangentU, tangentV);
    }
    else
    {
        tangentU = math::Vec3{1.0, 0.0, 0.0};
        tangentV = math::Vec3{0.0, 0.0, 1.0};
    }
    std::uint32_t index = 0;
    outPoints[index++]  = endpointA;
    outPoints[index++]  = endpointB;
    for (std::uint32_t rim = 0; rim < 2; ++rim)
    {
        const math::Vec3 capCenter = rim == 0 ? endpointA : endpointB;
        for (std::uint32_t quadrant = 0; quadrant < 4; ++quadrant)
        {
            const double     angle = static_cast<double>(quadrant) * kPi * 0.5;
            const math::Vec3 radial = tangentU * (std::cos(angle) * radius) + tangentV * (std::sin(angle) * radius);
            outPoints[index++]      = capCenter + radial;
        }
    }
}

// Closest point on the local-frame solid cylinder (axis along local +Y,
// centered at the origin) to `localPoint`. Inside points return themselves
// with outInside set; the projection clamps the axis coordinate and the
// radial distance independently, which is exact for the slab/cylinder
// intersection region.
math::Vec3 closestPointOnSolidCylinderLocal(
    const math::Vec3 &localPoint, double radius, double halfHeight, bool &outInside)
{
    const double radialSquared = localPoint.x * localPoint.x + localPoint.z * localPoint.z;
    outInside = radialSquared <= radius * radius && localPoint.y >= -halfHeight && localPoint.y <= halfHeight;
    if (outInside)
    {
        return localPoint;
    }
    math::Vec3 clamped   = localPoint;
    clamped.y            = std::clamp(clamped.y, -halfHeight, halfHeight);
    const double radial  = std::sqrt(radialSquared);
    const double radialScale = radial > radius && radial > kEpsilon ? radius / radial : 1.0;
    clamped.x *= radialScale;
    clamped.z *= radialScale;
    return clamped;
}

// Nearest-surface push-out for a point inside the local-frame solid cylinder:
// the outLocalNormal is the unit surface normal at the escape point and the
// return value is the distance to that surface (cap plane or side wall).
double insideSolidCylinderEscape(
    const math::Vec3 &localPoint, double radius, double halfHeight, math::Vec3 &outLocalNormal)
{
    const double radial        = std::sqrt(localPoint.x * localPoint.x + localPoint.z * localPoint.z);
    const double faceDistance  = halfHeight - std::abs(localPoint.y);
    const double sideDistance  = radius - radial;
    if (faceDistance <= sideDistance)
    {
        outLocalNormal           = math::Vec3{};
        outLocalNormal.y         = localPoint.y >= 0.0 ? 1.0 : -1.0;
        return faceDistance;
    }
    if (radial > kEpsilon)
    {
        outLocalNormal = math::Vec3{localPoint.x / radial, 0.0, localPoint.z / radial};
    }
    else
    {
        outLocalNormal = math::Vec3{0.0, 0.0, 1.0};
    }
    return sideDistance;
}

// Sphere against a solid cylinder (sphere A, cylinder B). Mirrors sphereBox:
// the query runs in the cylinder's local frame, with an inside push-out
// through the nearest surface and a closest-point test otherwise.
bool sphereCylinder(const RigidTransform &transformA, double radiusA, const RigidTransform &transformB, double radiusB,
    double halfHeightB, double margin, ContactManifold &manifold)
{
    const math::Vec3 local = math::inverseRotate(transformB.rotation, transformA.position - transformB.position);
    bool             inside  = false;
    const math::Vec3 closest = closestPointOnSolidCylinderLocal(local, radiusB, halfHeightB, inside);
    math::Vec3       localNormal;
    double           depth        = 0.0;
    math::Vec3       surfaceLocal;
    if (inside)
    {
        const double escape = insideSolidCylinderEscape(local, radiusB, halfHeightB, localNormal);
        depth               = radiusA + escape;
        surfaceLocal        = local + localNormal * escape;
    }
    else
    {
        const math::Vec3 delta           = local - closest;
        const double     distanceSquared = math::lengthSquared(delta);
        if (distanceSquared >= (radiusA + margin) * (radiusA + margin))
        {
            return false;
        }
        const double distance = std::sqrt(distanceSquared);
        localNormal           = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
        depth                 = radiusA - distance;
        surfaceLocal          = closest;
    }
    manifold.normal         = math::rotate(transformB.rotation, localNormal);
    manifold.pointCount     = 0;
    const math::Vec3 pointB = transformB.position + math::rotate(transformB.rotation, surfaceLocal);
    const math::Vec3 pointA = pointB + manifold.normal * depth;
    appendContact(manifold, pointA, pointB);
    return true;
}

// Box against solid cylinder: the box's eight corners are projected onto the
// cylinder and the cylinder's ten surface samples onto the box; the deepest
// contact's normal becomes the manifold normal (the sampled-sphere idiom).
bool boxCylinder(const RigidTransform &transformA, const double halfExtents[3], const RigidTransform &transformB,
    double radiusB, double halfHeightB, double margin, ContactManifold &manifold)
{
    manifold.pointCount       = 0;
    manifold.normal           = math::Vec3{0.0, 1.0, 0.0};
    bool             touching = false;
    double           bestDepth = 0.0;

    const auto offerContact = [&](const math::Vec3 &pointA, const math::Vec3 &pointB, const math::Vec3 &normal,
                              double depth)
    {
        if (!touching || depth > bestDepth)
        {
            bestDepth      = depth;
            manifold.normal = normal;
        }
        touching = true;
        appendContact(manifold, pointA, pointB);
    };

    // Box corners against the cylinder solid.
    for (std::uint32_t cornerIndex = 0; cornerIndex < 8; ++cornerIndex)
    {
        const math::Vec3 localCorner{
            cornerIndex & 1u ? halfExtents[0] : -halfExtents[0],
            cornerIndex & 2u ? halfExtents[1] : -halfExtents[1],
            cornerIndex & 4u ? halfExtents[2] : -halfExtents[2]};
        const math::Vec3 corner = transformA.position + math::rotate(transformA.rotation, localCorner);
        const math::Vec3 local  = math::inverseRotate(transformB.rotation, corner - transformB.position);
        bool             inside  = false;
        const math::Vec3 closest = closestPointOnSolidCylinderLocal(local, radiusB, halfHeightB, inside);
        if (inside)
        {
            math::Vec3 localNormal;
            const double escape = insideSolidCylinderEscape(local, radiusB, halfHeightB, localNormal);
            const math::Vec3 normal = math::rotate(transformB.rotation, localNormal);
            const math::Vec3 surface = transformB.position + math::rotate(transformB.rotation, local + localNormal * escape);
            offerContact(corner, surface, normal, escape);
        }
        else
        {
            const math::Vec3 delta           = local - closest;
            const double     distanceSquared = math::lengthSquared(delta);
            if (distanceSquared >= margin * margin)
            {
                continue;
            }
            const double     distance = std::sqrt(distanceSquared);
            const math::Vec3 normal
                = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
            const math::Vec3 worldClosest = transformB.position + math::rotate(transformB.rotation, closest);
            offerContact(corner, worldClosest, normal, distance);
        }
    }

    // Cylinder surface samples against the box solid.
    math::Vec3 samples[10];
    appendCylinderSurfaceSamples(transformB, radiusB, halfHeightB, samples);
    const math::Mat3 rotationA = math::makeRotationMatrix(transformA.rotation);
    for (std::uint32_t sampleIndex = 0; sampleIndex < 10; ++sampleIndex)
    {
        const math::Vec3 local = math::inverseRotate(transformA.rotation, samples[sampleIndex] - transformA.position);
        const math::Vec3 clamped{
            std::clamp(local.x, -halfExtents[0], halfExtents[0]),
            std::clamp(local.y, -halfExtents[1], halfExtents[1]),
            std::clamp(local.z, -halfExtents[2], halfExtents[2])};
        math::Vec3 localNormal;
        math::Vec3 closestLocal;
        double     depth = 0.0;
        if (math::distanceSquared(local, clamped) <= kEpsilon * kEpsilon)
        {
            // Sample inside the box: push out through the nearest face.
            const double faceDistances[3] = {halfExtents[0] - std::abs(local.x),
                halfExtents[1] - std::abs(local.y),
                halfExtents[2] - std::abs(local.z)};
            std::uint32_t nearestAxis = 0;
            for (std::uint32_t axis = 1; axis < 3; ++axis)
            {
                if (faceDistances[axis] < faceDistances[nearestAxis])
                {
                    nearestAxis = axis;
                }
            }
            localNormal                           = math::Vec3{};
            math::component(localNormal, nearestAxis) = math::component(local, nearestAxis) >= 0.0 ? 1.0 : -1.0;
            depth                                 = faceDistances[nearestAxis];
            closestLocal                          = localNormal * halfExtents[nearestAxis];
        }
        else
        {
            const math::Vec3 delta           = local - clamped;
            const double     distanceSquared = math::lengthSquared(delta);
            if (distanceSquared >= margin * margin)
            {
                continue;
            }
            const double distance = std::sqrt(distanceSquared);
            localNormal           = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
            depth                 = distance;
            closestLocal          = clamped;
        }
        const math::Vec3 normal      = math::rotate(transformA.rotation, localNormal) * -1.0;
        const math::Vec3 pointOnBox  = transformA.position + math::multiplyMatrixVector(rotationA, closestLocal);
        offerContact(pointOnBox, samples[sampleIndex], normal, depth);
    }

    if (!touching)
    {
        return false;
    }
    trimManifold(manifold, 4);
    return true;
}

// Capsule against solid cylinder: the capsule's two interior spheres are
// tested against the cylinder (the sampled-sphere idiom of boxCapsule).
bool capsuleCylinder(const RigidTransform &transformA, double radiusA, double halfLengthA,
    const RigidTransform &transformB, double radiusB, double halfHeightB, double margin, ContactManifold &manifold)
{
    manifold.pointCount       = 0;
    manifold.normal           = math::Vec3{0.0, 1.0, 0.0};
    bool             touching = false;
    const math::Vec3 halfAxis = math::rotate(transformA.rotation, math::Vec3{0.0, halfLengthA * 0.5, 0.0});
    for (std::uint32_t sample = 0; sample < 2; ++sample)
    {
        RigidTransform   sphereTransform{};
        sphereTransform.position = transformA.position + (sample == 0 ? halfAxis * -1.0 : halfAxis);
        ContactManifold sampleManifold;
        if (sphereCylinder(sphereTransform, radiusA, transformB, radiusB, halfHeightB, margin, sampleManifold))
        {
            touching        = true;
            manifold.normal = sampleManifold.normal;
            for (std::uint32_t point = 0; point < sampleManifold.pointCount; ++point)
            {
                appendContact(manifold, sampleManifold.points[point].pointA, sampleManifold.points[point].pointB);
            }
        }
    }
    if (!touching)
    {
        return false;
    }
    trimManifold(manifold, 2);
    return true;
}

// Solid cylinder against solid cylinder: each cylinder's ten surface samples
// are projected onto the other solid; the deepest contact's normal becomes
// the manifold normal. The fixed sample sets keep the result deterministic.
bool cylinderCylinder(const RigidTransform &transformA, double radiusA, double halfHeightA,
    const RigidTransform &transformB, double radiusB, double halfHeightB, double margin, ContactManifold &manifold)
{
    manifold.pointCount       = 0;
    manifold.normal           = math::Vec3{0.0, 1.0, 0.0};
    bool             touching = false;
    double           bestDepth = 0.0;

    const auto projectSamples = [&](const RigidTransform &fromTransform, double fromRadius, double fromHalfHeight,
                                const RigidTransform &toTransform, double toRadius, double toHalfHeight,
                                bool flip)
    {
        math::Vec3 samples[10];
        appendCylinderSurfaceSamples(fromTransform, fromRadius, fromHalfHeight, samples);
        for (std::uint32_t sampleIndex = 0; sampleIndex < 10; ++sampleIndex)
        {
            const math::Vec3 local
                = math::inverseRotate(toTransform.rotation, samples[sampleIndex] - toTransform.position);
            bool        inside  = false;
            const math::Vec3 closest = closestPointOnSolidCylinderLocal(local, toRadius, toHalfHeight, inside);
            math::Vec3 normal;
            double     depth      = 0.0;
            math::Vec3 pointOnTo;
            if (inside)
            {
                math::Vec3 localNormal;
                depth          = insideSolidCylinderEscape(local, toRadius, toHalfHeight, localNormal);
                normal         = math::rotate(toTransform.rotation, localNormal);
                pointOnTo      = toTransform.position + math::rotate(toTransform.rotation, local + localNormal * depth);
            }
            else
            {
                const math::Vec3 delta           = local - closest;
                const double     distanceSquared = math::lengthSquared(delta);
                if (distanceSquared >= margin * margin)
                {
                    continue;
                }
                const double distance = std::sqrt(distanceSquared);
                normal                = distance > kEpsilon
                        ? delta * (1.0 / distance)
                        : math::Vec3{0.0, 1.0, 0.0};
                depth                 = distance;
                pointOnTo             = toTransform.position + math::rotate(toTransform.rotation, closest);
            }
            if (flip)
            {
                normal = normal * -1.0;
            }
            if (!touching || depth > bestDepth)
            {
                bestDepth       = depth;
                manifold.normal = normal;
            }
            touching = true;
            if (flip)
            {
                appendContact(manifold, pointOnTo, samples[sampleIndex]);
            }
            else
            {
                appendContact(manifold, samples[sampleIndex], pointOnTo);
            }
        }
    };

    projectSamples(transformA, radiusA, halfHeightA, transformB, radiusB, halfHeightB, false);
    projectSamples(transformB, radiusB, halfHeightB, transformA, radiusA, halfHeightA, true);

    if (!touching)
    {
        return false;
    }
    trimManifold(manifold, 4);
    return true;
}

// Solid cylinder against a plane: cap centers and both cap rims are sampled
// at fixed quarter angles, so an upright cylinder rests on four rim contacts
// the way a box rests on four corner contacts.
bool cylinderPlane(const RigidTransform &transformA, double radiusA, double halfHeightA,
    const math::Vec3 &planeNormal, double planeOffset, double margin, ContactManifold &manifold)
{
    manifold.normal     = planeNormal;
    manifold.pointCount = 0;
    math::Vec3 samples[10];
    appendCylinderSurfaceSamples(transformA, radiusA, halfHeightA, samples);
    for (std::uint32_t sampleIndex = 0; sampleIndex < 10; ++sampleIndex)
    {
        const math::Vec3 point          = samples[sampleIndex];
        const double     signedDistance = math::dot(point, planeNormal) - planeOffset;
        if (signedDistance < margin)
        {
            appendContact(manifold, point, point - planeNormal * std::min(signedDistance, 0.0));
        }
    }
    if (manifold.pointCount == 0)
    {
        return false;
    }
    trimManifold(manifold, 4);
    return true;
}

} // namespace

bool makeShape(const ShapeDescription &description, Shape &outShape)
{
    if (!validateShape(description))
    {
        return false;
    }
    Shape shape;
    shape.kind              = description.kind;
    shape.sphereRadius      = description.sphereRadius;
    shape.boxHalfExtents[0] = description.boxHalfExtents[0];
    shape.boxHalfExtents[1] = description.boxHalfExtents[1];
    shape.boxHalfExtents[2] = description.boxHalfExtents[2];
    shape.capsuleRadius     = description.capsuleRadius;
    shape.capsuleHalfLength = description.capsuleHalfLength;
    shape.cylinderRadius     = description.cylinderRadius;
    shape.cylinderHalfHeight = description.cylinderHalfHeight;
    shape.planeNormal
        = math::normalized(Vec3{description.planeNormal[0], description.planeNormal[1], description.planeNormal[2]});
    shape.planeOffset = description.planeOffset;
    outShape          = shape;
    return true;
}

double computeShapeVolume(const Shape &shape)
{
    switch (shape.kind)
    {
    case ShapeKind::Sphere:
        return (4.0 / 3.0) * kPi * shape.sphereRadius * shape.sphereRadius * shape.sphereRadius;
    case ShapeKind::Box:
        return 8.0 * shape.boxHalfExtents[0] * shape.boxHalfExtents[1] * shape.boxHalfExtents[2];
    case ShapeKind::Capsule: {
        const double cylinderVolume = 2.0 * shape.capsuleHalfLength * kPi * shape.capsuleRadius * shape.capsuleRadius;
        const double sphereVolume = (4.0 / 3.0) * kPi * shape.capsuleRadius * shape.capsuleRadius * shape.capsuleRadius;
        return cylinderVolume + sphereVolume;
    }
    case ShapeKind::Cylinder:
        return 2.0 * shape.cylinderHalfHeight * kPi * shape.cylinderRadius * shape.cylinderRadius;
    case ShapeKind::Plane:
    default:
        return 0.0;
    }
}

bool computeMassProperties(
    const Shape &shape, double density, double explicitMass, double &outMass, math::Vec3 &outInertiaDiagonal)
{
    const double mass = density > 0.0 ? computeShapeVolume(shape) * density : explicitMass;
    if (!(mass > 0.0) || !std::isfinite(mass))
    {
        return false;
    }
    math::Vec3 inertia{};
    switch (shape.kind)
    {
    case ShapeKind::Sphere: {
        const double radiusSquared = shape.sphereRadius * shape.sphereRadius;
        inertia = math::Vec3{0.4 * mass * radiusSquared, 0.4 * mass * radiusSquared, 0.4 * mass * radiusSquared};
        break;
    }
    case ShapeKind::Box: {
        const double width  = shape.boxHalfExtents[0] * 2.0;
        const double height = shape.boxHalfExtents[1] * 2.0;
        const double depth  = shape.boxHalfExtents[2] * 2.0;
        inertia             = math::Vec3{
            mass * (height * height + depth * depth) / 12.0,
            mass * (width * width + depth * depth) / 12.0,
            mass * (width * width + height * height) / 12.0};
        break;
    }
    case ShapeKind::Capsule: {
        // Exact solid capsule: cylinder (height 2 * halfLength) plus two
        // hemispheres whose centers of mass sit 3r/8 beyond their flat
        // faces; the parallel-axis theorem lifts each hemisphere.
        const double radius           = shape.capsuleRadius;
        const double radiusSquared    = radius * radius;
        const double cylinderVolume   = 2.0 * shape.capsuleHalfLength * kPi * radiusSquared;
        const double hemisphereVolume = (2.0 / 3.0) * kPi * radiusSquared * radius;
        const double totalVolume      = cylinderVolume + 2.0 * hemisphereVolume;
        const double cylinderMass     = mass * cylinderVolume / totalVolume;
        const double hemisphereMass   = mass * hemisphereVolume / totalVolume;
        const double hemisphereOffset = shape.capsuleHalfLength + 0.375 * radius;
        const double offsetSquared    = hemisphereOffset * hemisphereOffset;
        const double yAxis            = 0.5 * cylinderMass * radiusSquared
                                      + 2.0 * (0.4 * hemisphereMass * radiusSquared + hemisphereMass * offsetSquared);
        const double transverse
            = (1.0 / 12.0) * cylinderMass
                * (4.0 * shape.capsuleHalfLength * shape.capsuleHalfLength + 3.0 * radiusSquared)
            + 2.0 * ((83.0 / 320.0) * hemisphereMass * radiusSquared + hemisphereMass * offsetSquared);
        inertia = math::Vec3{transverse, yAxis, transverse};
        break;
    }
    case ShapeKind::Cylinder: {
        // Solid cylinder about its local +Y axis: the axial moment is the
        // disc moment; the transverse moments follow from the parallel-axis
        // form for a solid cylinder of half height `h`.
        const double radius        = shape.cylinderRadius;
        const double heightSquared = 4.0 * shape.cylinderHalfHeight * shape.cylinderHalfHeight;
        inertia                    = math::Vec3{
            mass * (3.0 * radius * radius + heightSquared) / 12.0,
            0.5 * mass * radius * radius,
            mass * (3.0 * radius * radius + heightSquared) / 12.0};
        break;
    }
    case ShapeKind::Plane:
    default:
        return false;
    }
    outMass            = mass;
    outInertiaDiagonal = inertia;
    return true;
}

void computeCapsuleSegment(
    const Shape &shape, const RigidTransform &transform, math::Vec3 &outEndpointA, math::Vec3 &outEndpointB)
{
    const math::Vec3 axis = math::rotate(transform.rotation, math::Vec3{0.0, shape.capsuleHalfLength, 0.0});
    outEndpointA          = transform.position - axis;
    outEndpointB          = transform.position + axis;
}

void computeShapeAabb(
    const Shape &shape, const RigidTransform &transform, math::Vec3 &outMinimum, math::Vec3 &outMaximum)
{
    switch (shape.kind)
    {
    case ShapeKind::Sphere: {
        const math::Vec3 padding{shape.sphereRadius, shape.sphereRadius, shape.sphereRadius};
        outMinimum = transform.position - padding;
        outMaximum = transform.position + padding;
        break;
    }
    case ShapeKind::Box: {
        const math::Mat3 rotation = math::makeRotationMatrix(transform.rotation);
        const math::Vec3 extentColumn{
            std::abs(rotation.elements[0]) * shape.boxHalfExtents[0]
                + std::abs(rotation.elements[3]) * shape.boxHalfExtents[1]
                + std::abs(rotation.elements[6]) * shape.boxHalfExtents[2],
            std::abs(rotation.elements[1]) * shape.boxHalfExtents[0]
                + std::abs(rotation.elements[4]) * shape.boxHalfExtents[1]
                + std::abs(rotation.elements[7]) * shape.boxHalfExtents[2],
            std::abs(rotation.elements[2]) * shape.boxHalfExtents[0]
                + std::abs(rotation.elements[5]) * shape.boxHalfExtents[1]
                + std::abs(rotation.elements[8]) * shape.boxHalfExtents[2]};
        outMinimum = transform.position - extentColumn;
        outMaximum = transform.position + extentColumn;
        break;
    }
    case ShapeKind::Capsule: {
        math::Vec3 endpointA;
        math::Vec3 endpointB;
        computeCapsuleSegment(shape, transform, endpointA, endpointB);
        const math::Vec3 padding{shape.capsuleRadius, shape.capsuleRadius, shape.capsuleRadius};
        outMinimum = math::Vec3{
            std::min(endpointA.x, endpointB.x) - padding.x,
            std::min(endpointA.y, endpointB.y) - padding.y,
            std::min(endpointA.z, endpointB.z) - padding.z};
        outMaximum = math::Vec3{
            std::max(endpointA.x, endpointB.x) + padding.x,
            std::max(endpointA.y, endpointB.y) + padding.y,
            std::max(endpointA.z, endpointB.z) + padding.z};
        break;
    }
    case ShapeKind::Cylinder: {
        // Support-function AABB: along each world axis the extent is the
        // half-height projected on the axis plus the cap-disc radius in the
        // perpendicular plane, which is exact for a rotated cylinder.
        const math::Vec3 axis = math::rotate(transform.rotation, math::Vec3{0.0, shape.cylinderHalfHeight, 0.0});
        math::Vec3       extent;
        const double     components[3] = {axis.x, axis.y, axis.z};
        for (std::uint32_t axisIndex = 0; axisIndex < 3; ++axisIndex)
        {
            const double axial       = components[axisIndex];
            math::component(extent, axisIndex) = shape.cylinderHalfHeight * std::abs(axial)
                + shape.cylinderRadius * std::sqrt(std::max(0.0, 1.0 - axial * axial));
        }
        outMinimum = transform.position - extent;
        outMaximum = transform.position + extent;
        break;
    }
    case ShapeKind::Plane:
    default:
        // Planes are unbounded; callers never hash them.
        outMinimum = math::Vec3{1.0, 1.0, 1.0};
        outMaximum = math::Vec3{-1.0, -1.0, -1.0};
        break;
    }
}

bool collideShapes(const Shape &shapeA, const RigidTransform &transformA, const Shape &shapeB,
    const RigidTransform &transformB, double margin, ContactManifold &outManifold)
{
    ContactManifold manifold;
    bool            touching = false;
    const ShapeKind kindA    = shapeA.kind;
    const ShapeKind kindB    = shapeB.kind;
    if (kindA == ShapeKind::Sphere && kindB == ShapeKind::Sphere)
    {
        touching = sphereSphere(transformA, shapeA.sphereRadius, transformB, shapeB.sphereRadius, margin, manifold);
    }
    else if (kindA == ShapeKind::Sphere && kindB == ShapeKind::Box)
    {
        touching = sphereBox(transformA, shapeA.sphereRadius, transformB, shapeB.boxHalfExtents, margin, manifold);
    }
    else if (kindA == ShapeKind::Box && kindB == ShapeKind::Sphere)
    {
        touching = sphereBox(transformB, shapeB.sphereRadius, transformA, shapeA.boxHalfExtents, margin, manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Sphere && kindB == ShapeKind::Capsule)
    {
        touching = sphereCapsule(transformA,
            shapeA.sphereRadius,
            transformB,
            shapeB.capsuleRadius,
            shapeB.capsuleHalfLength,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Capsule && kindB == ShapeKind::Sphere)
    {
        touching = sphereCapsule(transformB,
            shapeB.sphereRadius,
            transformA,
            shapeA.capsuleRadius,
            shapeA.capsuleHalfLength,
            margin,
            manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Box && kindB == ShapeKind::Box)
    {
        touching = boxBox(transformA, shapeA.boxHalfExtents, transformB, shapeB.boxHalfExtents, margin, manifold);
    }
    else if (kindA == ShapeKind::Box && kindB == ShapeKind::Capsule)
    {
        touching = boxCapsule(transformA,
            shapeA.boxHalfExtents,
            transformB,
            shapeB.capsuleRadius,
            shapeB.capsuleHalfLength,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Capsule && kindB == ShapeKind::Box)
    {
        touching = boxCapsule(transformB,
            shapeB.boxHalfExtents,
            transformA,
            shapeA.capsuleRadius,
            shapeA.capsuleHalfLength,
            margin,
            manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Capsule && kindB == ShapeKind::Capsule)
    {
        touching = capsuleCapsule(transformA,
            shapeA.capsuleRadius,
            shapeA.capsuleHalfLength,
            transformB,
            shapeB.capsuleRadius,
            shapeB.capsuleHalfLength,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Sphere && kindB == ShapeKind::Cylinder)
    {
        touching = sphereCylinder(transformA,
            shapeA.sphereRadius,
            transformB,
            shapeB.cylinderRadius,
            shapeB.cylinderHalfHeight,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Cylinder && kindB == ShapeKind::Sphere)
    {
        touching = sphereCylinder(transformB,
            shapeB.sphereRadius,
            transformA,
            shapeA.cylinderRadius,
            shapeA.cylinderHalfHeight,
            margin,
            manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Box && kindB == ShapeKind::Cylinder)
    {
        touching = boxCylinder(transformA,
            shapeA.boxHalfExtents,
            transformB,
            shapeB.cylinderRadius,
            shapeB.cylinderHalfHeight,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Cylinder && kindB == ShapeKind::Box)
    {
        touching = boxCylinder(transformB,
            shapeB.boxHalfExtents,
            transformA,
            shapeA.cylinderRadius,
            shapeA.cylinderHalfHeight,
            margin,
            manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Capsule && kindB == ShapeKind::Cylinder)
    {
        touching = capsuleCylinder(transformA,
            shapeA.capsuleRadius,
            shapeA.capsuleHalfLength,
            transformB,
            shapeB.cylinderRadius,
            shapeB.cylinderHalfHeight,
            margin,
            manifold);
    }
    else if (kindA == ShapeKind::Cylinder && kindB == ShapeKind::Capsule)
    {
        touching = capsuleCylinder(transformB,
            shapeB.capsuleRadius,
            shapeB.capsuleHalfLength,
            transformA,
            shapeA.cylinderRadius,
            shapeA.cylinderHalfHeight,
            margin,
            manifold);
        if (touching)
        {
            flipManifold(manifold);
        }
    }
    else if (kindA == ShapeKind::Cylinder && kindB == ShapeKind::Cylinder)
    {
        touching = cylinderCylinder(transformA,
            shapeA.cylinderRadius,
            shapeA.cylinderHalfHeight,
            transformB,
            shapeB.cylinderRadius,
            shapeB.cylinderHalfHeight,
            margin,
            manifold);
    }
    if (!touching)
    {
        return false;
    }
    outManifold = manifold;
    return true;
}

bool collidePlaneSolid(const math::Vec3 &planeNormal, double planeOffset, const Shape &solidShape,
    const RigidTransform &solidTransform, double margin, ContactManifold &outManifold)
{
    ContactManifold manifold;
    bool            touching = false;
    switch (solidShape.kind)
    {
    case ShapeKind::Sphere:
        touching = spherePlane(solidTransform, solidShape.sphereRadius, planeNormal, planeOffset, margin, manifold);
        break;
    case ShapeKind::Box:
        touching = boxPlane(solidTransform, solidShape.boxHalfExtents, planeNormal, planeOffset, margin, manifold);
        break;
    case ShapeKind::Capsule:
        touching = capsulePlane(solidTransform,
            solidShape.capsuleRadius,
            solidShape.capsuleHalfLength,
            planeNormal,
            planeOffset,
            margin,
            manifold);
        break;
    case ShapeKind::Cylinder:
        touching = cylinderPlane(solidTransform,
            solidShape.cylinderRadius,
            solidShape.cylinderHalfHeight,
            planeNormal,
            planeOffset,
            margin,
            manifold);
        break;
    case ShapeKind::Plane:
    default:
        return false;
    }
    if (!touching)
    {
        return false;
    }
    outManifold = manifold;
    return true;
}

bool rayCastSphere(const RigidTransform &transform, double radius, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance)
{
    const math::Vec3 offset = origin - transform.position;
    const double     a      = math::dot(direction, direction);
    if (a <= kEpsilon)
    {
        return false;
    }
    const double b            = 2.0 * math::dot(offset, direction);
    const double c            = math::dot(offset, offset) - radius * radius;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0)
    {
        return false;
    }
    const double root = std::sqrt(discriminant);
    const double t1   = (-b - root) / (2.0 * a);
    const double t2   = (-b + root) / (2.0 * a);
    const double hit  = t1 > 0.0 ? t1 : t2;
    if (hit <= 0.0 || hit > maximumDistance)
    {
        return false;
    }
    outDistance = hit;
    return true;
}

bool rayCastBox(const RigidTransform &transform, const double halfExtents[3], const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance)
{
    const math::Vec3 localOrigin    = math::inverseRotate(transform.rotation, origin - transform.position);
    const math::Vec3 localDirection = math::inverseRotate(transform.rotation, direction);
    double           entry          = 0.0;
    double           exit           = maximumDistance;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        const double axisOrigin    = math::component(localOrigin, axis);
        const double axisDirection = math::component(localDirection, axis);
        const double extent        = halfExtents[axis];
        if (std::abs(axisDirection) < kEpsilon)
        {
            if (axisOrigin < -extent || axisOrigin > extent)
            {
                return false;
            }
            continue;
        }
        double near = (-extent - axisOrigin) / axisDirection;
        double far  = (extent - axisOrigin) / axisDirection;
        if (near > far)
        {
            std::swap(near, far);
        }
        entry = std::max(entry, near);
        exit  = std::min(exit, far);
        if (entry > exit)
        {
            return false;
        }
    }
    if (exit <= 0.0 || entry > maximumDistance)
    {
        return false;
    }
    outDistance = entry > 0.0 ? entry : exit;
    return outDistance > 0.0 && outDistance <= maximumDistance;
}

bool rayCastCapsule(const RigidTransform &transform, double radius, double halfLength, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance)
{
    // Minimum over the two spherical caps and the cylinder side surface.
    double best     = -1.0;
    auto   consider = [&best, maximumDistance](bool hit, double distance)
    {
        if (hit && distance > 0.0 && distance <= maximumDistance && (best < 0.0 || distance < best))
        {
            best = distance;
        }
    };
    RigidTransform capTransform{};
    capTransform.position = transform.position;
    double centerDistance = -1.0;
    if (rayCastSphere(capTransform, radius, origin, direction, maximumDistance, centerDistance))
    {
        consider(true, centerDistance);
    }
    const math::Vec3 axis = math::rotate(transform.rotation, math::Vec3{0.0, halfLength, 0.0});
    for (const double sign : {-1.0, 1.0})
    {
        RigidTransform endTransform{};
        endTransform.position = transform.position + axis * sign;
        double endDistance    = -1.0;
        if (rayCastSphere(endTransform, radius, origin, direction, maximumDistance, endDistance))
        {
            consider(true, endDistance);
        }
    }
    const math::Vec3 localOrigin            = math::inverseRotate(transform.rotation, origin - transform.position);
    const math::Vec3 localDirection         = math::inverseRotate(transform.rotation, direction);
    const double     radialDirectionSquared = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
    if (radialDirectionSquared > kEpsilon)
    {
        const double b            = 2.0 * (localOrigin.x * localDirection.x + localOrigin.z * localDirection.z);
        const double c            = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - radius * radius;
        const double discriminant = b * b - 4.0 * radialDirectionSquared * c;
        if (discriminant >= 0.0)
        {
            const double root = std::sqrt(discriminant);
            for (const double t :
                {(-b - root) / (2.0 * radialDirectionSquared), (-b + root) / (2.0 * radialDirectionSquared)})
            {
                if (t <= 0.0 || t > maximumDistance)
                {
                    continue;
                }
                const double y = localOrigin.y + localDirection.y * t;
                if (std::abs(y) <= halfLength)
                {
                    consider(true, t);
                    break;
                }
            }
        }
    }
    else if (localDirection.y != 0.0)
    {
        for (const double face : {-halfLength, halfLength})
        {
            const double t = (face - localOrigin.y) / localDirection.y;
            if (t > 0.0 && t <= maximumDistance
                && localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z <= radius * radius)
            {
                consider(true, t);
            }
        }
    }
    if (best <= 0.0)
    {
        return false;
    }
    outDistance = best;
    return true;
}

// Ray against a solid cylinder: the side surface (infinite cylinder of the
// given radius clipped to the half-height) and the two flat cap discs, all in
// the cylinder's local frame; the nearest valid entry wins.
bool rayCastCylinder(const RigidTransform &transform, double radius, double halfHeight, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance)
{
    const math::Vec3 localOrigin    = math::inverseRotate(transform.rotation, origin - transform.position);
    const math::Vec3 localDirection = math::inverseRotate(transform.rotation, direction);
    double           bestDistance   = -1.0;

    const double radialSquared = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
    if (radialSquared > kEpsilon)
    {
        const double a            = radialSquared;
        const double b            = 2.0 * (localOrigin.x * localDirection.x + localOrigin.z * localDirection.z);
        const double c            = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - radius * radius;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0)
        {
            const double root = std::sqrt(discriminant);
            for (const double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)})
            {
                if (t <= 0.0 || t > maximumDistance || (bestDistance >= 0.0 && t >= bestDistance))
                {
                    continue;
                }
                const double y = localOrigin.y + localDirection.y * t;
                if (std::abs(y) <= halfHeight)
                {
                    bestDistance = t;
                }
            }
        }
    }

    for (const double cap : {-halfHeight, halfHeight})
    {
        if (std::abs(localDirection.y) > kEpsilon)
        {
            const double t = (cap - localOrigin.y) / localDirection.y;
            if (t > 0.0 && t <= maximumDistance && (bestDistance < 0.0 || t < bestDistance))
            {
                const double x = localOrigin.x + localDirection.x * t;
                const double z = localOrigin.z + localDirection.z * t;
                if (x * x + z * z <= radius * radius)
                {
                    bestDistance = t;
                }
            }
        }
    }

    if (bestDistance <= 0.0)
    {
        return false;
    }
    outDistance = bestDistance;
    return true;
}

bool rayCastPlane(const math::Vec3 &planeNormal, double planeOffset, const math::Vec3 &origin,
    const math::Vec3 &direction, double maximumDistance, double &outDistance)
{
    const double denominator = math::dot(direction, planeNormal);
    if (std::abs(denominator) < kEpsilon)
    {
        return false;
    }
    const double t = (planeOffset - math::dot(origin, planeNormal)) / denominator;
    if (t <= 0.0 || t > maximumDistance)
    {
        return false;
    }
    outDistance = t;
    return true;
}

bool collideSphereSolid(const math::Vec3 &center, double radius, const Shape &solidShape,
    const RigidTransform &solidTransform, math::Vec3 &outNormal, double &outDepth)
{
    switch (solidShape.kind)
    {
    case ShapeKind::Sphere: {
        const math::Vec3 delta           = center - solidTransform.position;
        const double     combined        = radius + solidShape.sphereRadius;
        const double     distanceSquared = math::lengthSquared(delta);
        if (distanceSquared >= combined * combined)
        {
            return false;
        }
        const double distance = std::sqrt(distanceSquared);
        outNormal             = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
        outDepth              = combined - distance;
        return true;
    }
    case ShapeKind::Box: {
        ContactManifold manifold;
        RigidTransform  sphereTransform{};
        sphereTransform.position = center;
        if (!sphereBox(sphereTransform, radius, solidTransform, solidShape.boxHalfExtents, 0.0, manifold))
        {
            return false;
        }
        // The manifold normal already runs box-to-sphere: the push-out
        // direction for the query sphere.
        outNormal = manifold.normal;
        outDepth  = math::dot(manifold.points[0].pointA - manifold.points[0].pointB, manifold.normal);
        return outDepth > 0.0;
    }
    case ShapeKind::Capsule: {
        const math::Vec3 axis
            = math::rotate(solidTransform.rotation, math::Vec3{0.0, solidShape.capsuleHalfLength, 0.0});
        const math::Vec3 start     = solidTransform.position - axis;
        const math::Vec3 direction = axis * 2.0;
        const double t = std::clamp(math::dot(center - start, direction) / math::dot(direction, direction), 0.0, 1.0);
        const math::Vec3 segmentPoint    = start + direction * t;
        const math::Vec3 delta           = center - segmentPoint;
        const double     combined        = radius + solidShape.capsuleRadius;
        const double     distanceSquared = math::lengthSquared(delta);
        if (distanceSquared >= combined * combined)
        {
            return false;
        }
        const double distance = std::sqrt(distanceSquared);
        outNormal             = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
        outDepth              = combined - distance;
        return true;
    }
    case ShapeKind::Cylinder: {
        const math::Vec3 local    = math::inverseRotate(solidTransform.rotation, center - solidTransform.position);
        bool             inside   = false;
        const math::Vec3 closest  = closestPointOnSolidCylinderLocal(
            local, solidShape.cylinderRadius, solidShape.cylinderHalfHeight, inside);
        math::Vec3 localNormal;
        double     depth = 0.0;
        if (inside)
        {
            depth = radius + insideSolidCylinderEscape(local, solidShape.cylinderRadius, solidShape.cylinderHalfHeight, localNormal);
        }
        else
        {
            const math::Vec3 delta           = local - closest;
            const double     distanceSquared = math::lengthSquared(delta);
            if (distanceSquared >= radius * radius)
            {
                return false;
            }
            const double distance = std::sqrt(distanceSquared);
            localNormal           = distance > kEpsilon ? delta * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
            depth                 = radius - distance;
        }
        outNormal = math::rotate(solidTransform.rotation, localNormal);
        outDepth  = depth;
        return true;
    }
    case ShapeKind::Plane:
    default:
        return false;
    }
}

bool collideSpherePlane(const math::Vec3 &center, double radius, const math::Vec3 &planeNormal, double planeOffset,
    math::Vec3 &outNormal, double &outDepth)
{
    const double signedDistance = math::dot(center, planeNormal) - planeOffset;
    if (signedDistance >= radius)
    {
        return false;
    }
    outNormal = planeNormal;
    outDepth  = radius - signedDistance;
    return true;
}

double computeShapeBoundingRadius(const Shape &shape)
{
    switch (shape.kind)
    {
    case ShapeKind::Sphere:
        return shape.sphereRadius;
    case ShapeKind::Box:
        return math::length(math::Vec3{shape.boxHalfExtents[0], shape.boxHalfExtents[1], shape.boxHalfExtents[2]});
    case ShapeKind::Capsule:
        return shape.capsuleHalfLength + shape.capsuleRadius;
    case ShapeKind::Cylinder:
        return std::sqrt(shape.cylinderHalfHeight * shape.cylinderHalfHeight + shape.cylinderRadius * shape.cylinderRadius);
    case ShapeKind::Plane:
    default:
        return 0.0;
    }
}

bool sweepSphereSolid(const math::Vec3 &start, double radius, const math::Vec3 &delta, const Shape &solidShape,
    const RigidTransform &solidTransform, double &outDistance, math::Vec3 &outNormal)
{
    const double lengthSquared = math::lengthSquared(delta);
    if (!(radius > 0.0) || lengthSquared <= kEpsilon)
    {
        return false;
    }
    const double length           = std::sqrt(lengthSquared);
    const math::Vec3 direction    = delta * (1.0 / length);
    double           bestDistance = -1.0;
    math::Vec3       bestNormal;

    switch (solidShape.kind)
    {
    case ShapeKind::Sphere: {
        // Ray against the target sphere inflated by the query radius.
        double distance = 0.0;
        RigidTransform sphereTransform{};
        sphereTransform.position = solidTransform.position;
        if (rayCastSphere(sphereTransform, solidShape.sphereRadius + radius, start, direction, length, distance))
        {
            bestDistance = distance;
            bestNormal   = math::normalized(start + direction * distance - solidTransform.position);
        }
        break;
    }
    case ShapeKind::Box: {
        // Slab test against the box inflated by the query radius, in the
        // box's local frame (conservative rounded-corner approximation).
        const math::Vec3 localOrigin    = math::inverseRotate(solidTransform.rotation, start - solidTransform.position);
        const math::Vec3 localDirection = math::inverseRotate(solidTransform.rotation, direction);
        double           entry          = 0.0;
        double           exit           = length;
        std::uint32_t    entryAxis      = 0;
        double           entrySign      = 0.0;
        bool             hit            = true;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double axisOrigin    = math::component(localOrigin, axis);
            const double axisDirection = math::component(localDirection, axis);
            const double extent        = solidShape.boxHalfExtents[axis] + radius;
            if (std::abs(axisDirection) < kEpsilon)
            {
                if (axisOrigin < -extent || axisOrigin > extent)
                {
                    hit = false;
                    break;
                }
                continue;
            }
            double near = (-extent - axisOrigin) / axisDirection;
            double far  = (extent - axisOrigin) / axisDirection;
            double sign = -1.0;
            if (near > far)
            {
                std::swap(near, far);
                sign = 1.0;
            }
            if (near > entry)
            {
                entry     = near;
                entryAxis = axis;
                entrySign = sign;
            }
            exit = std::min(exit, far);
            if (entry > exit)
            {
                hit = false;
                break;
            }
        }
        if (hit && entry > 0.0 && entry <= length)
        {
            math::Vec3 localNormal{};
            math::component(localNormal, entryAxis) = entrySign;
            bestDistance                = entry;
            bestNormal                  = math::rotate(solidTransform.rotation, localNormal);
        }
        break;
    }
    case ShapeKind::Capsule: {
        // Ray against the capsule inflated by the query radius: the two cap
        // spheres and the (clipped) cylinder around the segment axis.
        const math::Vec3 axis = math::rotate(solidTransform.rotation, math::Vec3{0.0, solidShape.capsuleHalfLength, 0.0});
        double distance = 0.0;
        RigidTransform capTransform{};
        capTransform.position = solidTransform.position;
        if (rayCastSphere(capTransform, solidShape.capsuleRadius + radius, start, direction, length, distance))
        {
            bestDistance = distance;
            bestNormal   = math::normalized(start + direction * distance - solidTransform.position);
        }
        for (const double sign : {-1.0, 1.0})
        {
            RigidTransform endTransform{};
            endTransform.position = solidTransform.position + axis * sign;
            double endDistance    = 0.0;
            if (rayCastSphere(endTransform, solidShape.capsuleRadius + radius, start, direction, length, endDistance)
                && (bestDistance < 0.0 || endDistance < bestDistance))
            {
                bestDistance = endDistance;
                bestNormal   = math::normalized(start + direction * endDistance - endTransform.position);
            }
        }
        const math::Vec3 localOrigin    = math::inverseRotate(solidTransform.rotation, start - solidTransform.position);
        const math::Vec3 localDirection = math::inverseRotate(solidTransform.rotation, direction);
        const double     radialSquared  = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
        const double     inflated       = solidShape.capsuleRadius + radius;
        if (radialSquared > kEpsilon)
        {
            const double b            = 2.0 * (localOrigin.x * localDirection.x + localOrigin.z * localDirection.z);
            const double c            = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - inflated * inflated;
            const double discriminant = b * b - 4.0 * radialSquared * c;
            if (discriminant >= 0.0)
            {
                const double root = std::sqrt(discriminant);
                for (const double t : {(-b - root) / (2.0 * radialSquared), (-b + root) / (2.0 * radialSquared)})
                {
                    if (t <= 0.0 || t > length || (bestDistance >= 0.0 && t >= bestDistance))
                    {
                        continue;
                    }
                    const double y = localOrigin.y + localDirection.y * t;
                    if (std::abs(y) <= solidShape.capsuleHalfLength)
                    {
                        const math::Vec3 hitLocal{localOrigin.x + localDirection.x * t,
                            y,
                            localOrigin.z + localDirection.z * t};
                        bestDistance = t;
                        bestNormal   = math::rotate(solidTransform.rotation, math::normalized(hitLocal));
                        break;
                    }
                }
            }
        }
        break;
    }
    case ShapeKind::Cylinder: {
        // Ray against the cylinder inflated by the query radius: the side
        // band, the flat cap discs, and the rounded rim spheres at the cap
        // centers. The union is conservative (a sweep may stop slightly
        // early near the rim rounding) and never tunnels.
        const math::Vec3 axis
            = math::rotate(solidTransform.rotation, math::Vec3{0.0, solidShape.cylinderHalfHeight, 0.0});
        double distance = 0.0;
        RigidTransform capTransform{};
        capTransform.position = solidTransform.position;
        if (rayCastSphere(capTransform, solidShape.cylinderRadius + radius, start, direction, length, distance))
        {
            bestDistance = distance;
            bestNormal   = math::normalized(start + direction * distance - solidTransform.position);
        }
        for (const double sign : {-1.0, 1.0})
        {
            RigidTransform endTransform{};
            endTransform.position = solidTransform.position + axis * sign;
            double endDistance    = 0.0;
            if (rayCastSphere(endTransform, solidShape.cylinderRadius + radius, start, direction, length, endDistance)
                && (bestDistance < 0.0 || endDistance < bestDistance))
            {
                bestDistance = endDistance;
                bestNormal   = math::normalized(start + direction * endDistance - endTransform.position);
            }
        }
        const math::Vec3 localOrigin    = math::inverseRotate(solidTransform.rotation, start - solidTransform.position);
        const math::Vec3 localDirection = math::inverseRotate(solidTransform.rotation, direction);
        const double     radialSquared  = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
        const double     inflated       = solidShape.cylinderRadius + radius;
        if (radialSquared > kEpsilon)
        {
            const double b            = 2.0 * (localOrigin.x * localDirection.x + localOrigin.z * localDirection.z);
            const double c            = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - inflated * inflated;
            const double discriminant = b * b - 4.0 * radialSquared * c;
            if (discriminant >= 0.0)
            {
                const double root = std::sqrt(discriminant);
                for (const double t : {(-b - root) / (2.0 * radialSquared), (-b + root) / (2.0 * radialSquared)})
                {
                    if (t <= 0.0 || t > length || (bestDistance >= 0.0 && t >= bestDistance))
                    {
                        continue;
                    }
                    const double y = localOrigin.y + localDirection.y * t;
                    if (std::abs(y) <= solidShape.cylinderHalfHeight)
                    {
                        const math::Vec3 hitLocal{localOrigin.x + localDirection.x * t,
                            y,
                            localOrigin.z + localDirection.z * t};
                        bestDistance = t;
                        bestNormal   = math::rotate(solidTransform.rotation, math::normalized(hitLocal));
                        break;
                    }
                }
            }
        }
        for (const double cap : {-1.0, 1.0})
        {
            if (std::abs(localDirection.y) > kEpsilon)
            {
                const double capPlane = solidShape.cylinderHalfHeight + cap * radius;
                const double t        = (capPlane - localOrigin.y) / localDirection.y;
                if (t > 0.0 && t <= length && (bestDistance < 0.0 || t < bestDistance))
                {
                    const double x = localOrigin.x + localDirection.x * t;
                    const double z = localOrigin.z + localDirection.z * t;
                    if (x * x + z * z <= solidShape.cylinderRadius * solidShape.cylinderRadius)
                    {
                        math::Vec3 localNormal{};
                        localNormal.y = cap;
                        bestDistance  = t;
                        bestNormal    = math::rotate(solidTransform.rotation, localNormal);
                    }
                }
            }
        }
        break;
    }
    case ShapeKind::Plane:
    default:
        return false;
    }

    if (bestDistance <= 0.0)
    {
        return false;
    }
    // The normal faces back against the motion (the direction a body stops
    // with), matching the ray-cast normal convention.
    if (math::dot(bestNormal, direction) > 0.0)
    {
        bestNormal = bestNormal * -1.0;
    }
    outDistance = bestDistance;
    outNormal   = bestNormal;
    return true;
}

bool sweepSpherePlane(const math::Vec3 &start, double radius, const math::Vec3 &delta, const math::Vec3 &planeNormal,
    double planeOffset, double &outDistance, math::Vec3 &outNormal)
{
    if (!(radius > 0.0))
    {
        return false;
    }
    const double length = math::length(delta);
    if (length <= kEpsilon)
    {
        return false;
    }
    const double travel = math::dot(delta, planeNormal);
    const double offset = math::dot(start, planeNormal) - planeOffset;
    // First contact while moving: the sphere touches the half-space from the
    // open side at signedDistance = radius, from the far side at -radius.
    double     distance = -1.0;
    math::Vec3 normal{};
    if (travel < 0.0)
    {
        const double openTravel = (radius - offset) / travel;
        if (openTravel > 0.0 && openTravel <= 1.0)
        {
            distance = openTravel * length;
            normal   = planeNormal;
        }
    }
    else if (travel > 0.0)
    {
        const double farTravel = (-radius - offset) / travel;
        if (farTravel > 0.0 && farTravel <= 1.0)
        {
            distance = farTravel * length;
            normal   = planeNormal * -1.0;
        }
    }
    if (distance <= 0.0)
    {
        return false;
    }
    outDistance = distance;
    outNormal   = normal;
    return true;
}

} // namespace engine::physics_rigid
