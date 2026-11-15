// Kinematic surface walking implementation (see walker.hpp).

#include <cstdint>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/planet/walker.hpp>

#include <algorithm>
#include <cmath>

namespace engine
{
namespace planet
{
namespace
{

// A host hiccup must never integrate into a terrain tunnel: frames longer
// than this advance the walk in slices of at most this length.
constexpr double kMaxDeltaSeconds = 0.1;

// Bisection depth for every ground search. 24 halvings resolve any bracket
// in the shell band well below a millimeter - far under the mesh resolution,
// so the feet land exactly on the rendered surface.
constexpr std::uint32_t kGroundBisectSteps = 24;

// Margin the ground logic keeps around the found surface so standing
// comparisons do not flap on floating-point noise.
constexpr double kGroundTouchMeters = 0.02;

} // namespace

SurfaceWalker::SurfaceWalker(const PlanetGenerationParams &params, const DensityField &field)
    : params_(params)
    , field_(field)
{
}

double SurfaceWalker::topSurfaceRadius(const Vec3 &unitDirection) const
{
    // The shell band bounds every crossing the field can hold; the scan
    // walks it outward and keeps the LAST solid-to-air bracket (the terrain
    // top a descending probe from open sky reaches first). Overhang
    // undersides and cave mouths sit below that bracket and are ignored -
    // placing characters or cameras on them would bury them in the rock
    // above.
    const double band       = params_.derivedMaxReliefMeters * 2.0 + params_.overhangAmplitudeMeters * 2.0
        + params_.caveCarveMeters;
    const double lowRadius  = std::max(1.0, params_.radiusMeters - band);
    const double highRadius = params_.radiusMeters + band;
    constexpr std::uint32_t kMarchSteps = 24;
    const double stepMeters = (highRadius - lowRadius) / static_cast<double>(kMarchSteps);

    const auto densityAtRadius = [this, &unitDirection](double radius)
    {
        return field_.densityAt(Vec3{unitDirection.x * radius, unitDirection.y * radius, unitDirection.z * radius});
    };

    bool   haveBracket     = false;
    double below           = lowRadius;
    double above           = highRadius;
    double previousRadius  = lowRadius;
    double previousDensity = densityAtRadius(lowRadius);
    for (std::uint32_t step = 1; step <= kMarchSteps; ++step)
    {
        const double radius  = lowRadius + stepMeters * static_cast<double>(step);
        const double density = densityAtRadius(radius);
        if (previousDensity > 0.0 && density <= 0.0)
        {
            below       = previousRadius;
            above       = radius;
            haveBracket = true;
        }
        previousRadius  = radius;
        previousDensity = density;
    }
    if (!haveBracket)
    {
        // No crossing anywhere in the band (degenerate parameters): the
        // datum is the least surprising answer for consumers.
        return params_.radiusMeters;
    }
    for (std::uint32_t iteration = 0; iteration < kGroundBisectSteps; ++iteration)
    {
        const double mid = 0.5 * (below + above);
        if (densityAtRadius(mid) > 0.0)
        {
            below = mid;
        }
        else
        {
            above = mid;
        }
    }
    return 0.5 * (below + above);
}

bool SurfaceWalker::groundRadiusWithin(const Vec3 &position, double maxDepthMeters, double &outRadius) const
{
    const double radius = std::max(1.0e-9, length(position));
    const Vec3   up     = position * (1.0 / radius);
    const double depth  = std::max(kGroundTouchMeters, maxDepthMeters);

    const auto densityAtOffset = [this, &position, &up](double offset)
    {
        return field_.densityAt(position + up * offset);
    };
    const auto bisect = [&densityAtOffset](double solidOffset, double airOffset)
    {
        for (std::uint32_t iteration = 0; iteration < kGroundBisectSteps; ++iteration)
        {
            const double mid = 0.5 * (solidOffset + airOffset);
            if (densityAtOffset(mid) > 0.0)
            {
                solidOffset = mid;
            }
            else
            {
                airOffset = mid;
            }
        }
        return 0.5 * (solidOffset + airOffset);
    };

    if (field_.densityAt(position) > 0.0)
    {
        // The feet are inside solid rock (the terrain rose into them, or a
        // teleport landed inside a hill): search OUTWARD for the first
        // surface above and pop out there.
        constexpr std::uint32_t kMarchSteps = 12;
        const double stepMeters = depth / static_cast<double>(kMarchSteps);
        double previousOffset   = 0.0;
        for (std::uint32_t step = 1; step <= kMarchSteps; ++step)
        {
            const double offset  = stepMeters * static_cast<double>(step);
            if (densityAtOffset(offset) <= 0.0)
            {
                outRadius = length(position + up * bisect(previousOffset, offset));
                return true;
            }
            previousOffset = offset;
        }
        return false; // still buried at the probe top: the caller keeps the last state
    }

    // In air: march inward until the density turns positive, then bisect the
    // bracket. The probe follows the local radial direction, so the ground it
    // finds is the surface a point-feet character stands on - including
    // overhang undersides and cave floors directly beneath the feet.
    constexpr std::uint32_t kMarchSteps = 12;
    const double stepMeters = depth / static_cast<double>(kMarchSteps);
    double previousOffset   = 0.0;
    for (std::uint32_t step = 1; step <= kMarchSteps; ++step)
    {
        const double offset = stepMeters * static_cast<double>(step);
        if (densityAtOffset(-offset) > 0.0)
        {
            outRadius = length(position + up * bisect(-offset, -previousOffset));
            return true;
        }
        previousOffset = offset;
    }
    return false;
}

void SurfaceWalker::placeOnSurface(const Vec3 &unitDirection)
{
    const double directionLength = length(unitDirection);
    if (!std::isfinite(directionLength) || directionLength < 1.0e-12)
    {
        return;
    }
    const Vec3  direction = unitDirection * (1.0 / directionLength);
    const double radius  = topSurfaceRadius(direction);
    state_.positionMeters = direction * radius;
    state_.groundRadius   = radius;
    state_.grounded       = true;
    state_.swimming       = false;
    verticalSpeed_        = 0.0;
    state_.up             = direction;
    state_.eyePosition    = state_.positionMeters + direction * params_.eyeHeightMeters;
    state_.speed          = 0.0;
}

void SurfaceWalker::teleport(const Vec3 &position)
{
    state_.positionMeters = position;
    verticalSpeed_        = 0.0;
    state_.grounded       = false;
    state_.swimming       = false;
    state_.groundRadius   = 0.0;
    state_.up             = normalized(position);
    state_.eyePosition    = position + state_.up * params_.eyeHeightMeters;
    state_.speed          = 0.0;
}

void SurfaceWalker::update(double deltaSeconds, const WalkIntent &intent)
{
    // Long frames advance in slices of at most kMaxDeltaSeconds so a stalled
    // host cannot step the character through terrain; every slice runs the
    // identical integration, which keeps the walk deterministic for a given
    // sequence of (deltaSeconds, intent) inputs.
    double remaining = std::clamp(deltaSeconds, 0.0, 1.0);
    while (remaining > 0.0)
    {
        const double slice = std::min(remaining, kMaxDeltaSeconds);
        remaining -= slice;
        step(slice, intent);
    }
}

void SurfaceWalker::step(double deltaSeconds, const WalkIntent &intent)
{
    const Vec3  feet   = state_.positionMeters;
    const double radius = std::max(1.0e-9, length(feet));
    const Vec3  up     = feet * (1.0 / radius);

    // ---- heading: project the view direction onto the tangent plane --------
    Vec3 forwardDir{0.0, 0.0, 0.0};
    Vec3 rightDir{0.0, 0.0, 0.0};
    const double headingLength = length(intent.heading);
    if (headingLength > 1.0e-9)
    {
        const Vec3  projected      = intent.heading * (1.0 / headingLength) - up * (dot(intent.heading, up) / headingLength);
        const double projectedLength = length(projected);
        if (projectedLength > 1.0e-6)
        {
            forwardDir = projected * (1.0 / projectedLength);
            rightDir   = cross(forwardDir, up); // camera convention: right of the view
        }
    }
    Vec3 moveDirection = forwardDir * intent.forward + rightDir * intent.right;
    const double moveLength = length(moveDirection);
    if (moveLength > 1.0)
    {
        moveDirection = moveDirection * (1.0 / moveLength); // no diagonal speed bonus
    }
    const bool wantsMove = moveLength > 1.0e-9;

    // ---- ground under the feet ------------------------------------------------
    // The probe covers everything this frame can legally cross: the step-up
    // limit while walking, the snap window following ground down, and the
    // fall distance while airborne.
    const double fallThisFrame = std::abs(verticalSpeed_) * deltaSeconds + 4.0 * kGroundTouchMeters;
    const double probeDepth
        = std::max(params_.walkMaxStepUpMeters + params_.walkGroundSnapMeters + 1.0, fallThisFrame);
    double     groundRadius = state_.groundRadius;
    const bool haveGround   = groundRadiusWithin(feet, probeDepth, groundRadius);

    // ---- swimming ----------------------------------------------------------------
    // The sea surface is a floor where the water is deeper than the wade
    // band: the feet ride the waterline, movement slows to the swim speed,
    // and gravity is replaced by the surface clamp. Shallow shelf water
    // stays walkable - the terrain floor carries the feet. The float also
    // requires an OPEN WATER COLUMN between the feet and the waterline: a
    // march along the local up must read air (or water) at every step.
    // Cave pockets carved into the sea floor sit below the sea radius with
    // rock above them, and without the column check the float would carry
    // the feet straight up through the cave ceiling.
    const double seaRadius = params_.derivedSeaRadiusMeters;
    bool overDeepWater = params_.waterEnabled && params_.swimEnabled
        && (!haveGround || seaRadius - groundRadius > params_.swimWadeDepthMeters)
        && radius < seaRadius + kGroundTouchMeters;
    if (overDeepWater)
    {
        // Eight probes resolve a ceiling anywhere in the column: thinner
        // rock shelves than one step of this march cannot seal a pocket a
        // walker fits into, and thicker ones always catch a sample.
        constexpr std::uint32_t kColumnProbes = 8;
        const double columnRise = seaRadius - radius;
        for (std::uint32_t probe = 1; probe <= kColumnProbes && overDeepWater; ++probe)
        {
            const Vec3 testPoint = feet + up * (columnRise * (static_cast<double>(probe) / static_cast<double>(kColumnProbes)));
            if (field_.densityAt(testPoint) > 0.0)
            {
                overDeepWater = false; // rock overhead: not open water, keep the ground logic
            }
        }
    }
    if (overDeepWater && radius < seaRadius + kGroundTouchMeters)
    {
        state_.positionMeters = feet + up * (seaRadius - radius);
        state_.up             = Vec3{up.x, up.y, up.z};
        state_.swimming       = true;
        state_.grounded       = false;
        verticalSpeed_        = 0.0;
        state_.groundRadius   = seaRadius;
        if (wantsMove)
        {
            state_.positionMeters = state_.positionMeters + moveDirection * (params_.swimSpeedMetersPerSecond * deltaSeconds);
            state_.speed = params_.swimSpeedMetersPerSecond;
        }
        else
        {
            state_.speed = 0.0;
        }
        state_.up          = normalized(state_.positionMeters);
        state_.eyePosition = state_.positionMeters + state_.up * params_.eyeHeightMeters;
        return;
    }
    state_.swimming = false;

    // ---- horizontal movement with step/grade limits -------------------------------
    double speed = params_.walkSpeedMetersPerSecond * (intent.sprint ? params_.walkSprintMultiplier : 1.0);
    const double horizontalDistance = speed * deltaSeconds;

    if (wantsMove && horizontalDistance > 0.0)
    {
        // Attempt the full move first; on a blocked axis, retry each
        // component alone so the character slides along steep ground instead
        // of sticking to it.
        const Vec3 fullDelta    = moveDirection * horizontalDistance;
        const Vec3 forwardDelta = forwardDir * (intent.forward * horizontalDistance);
        const Vec3 rightDelta   = rightDir * (intent.right * horizontalDistance);
        const Vec3 attempts[3]  = {fullDelta, forwardDelta, rightDelta};

        // One move attempt: accept when the ground ahead is climbable (a
        // step within the step-up limit on a slope inside the grade limit);
        // a drop beyond the snap window is accepted as a walk off the edge
        // (the airborne branch below carries the fall); open sky below is
        // accepted the same way.
        const auto tryMove = [&](const Vec3 &delta)
        {
            const double attemptLength = length(delta);
            if (attemptLength < 1.0e-12)
            {
                return true;
            }
            const Vec3 candidate   = state_.positionMeters + delta;
            double     candidateGround = 0.0;
            const double candidateProbe = params_.walkMaxStepUpMeters + params_.walkGroundSnapMeters + 1.0;
            if (!groundRadiusWithin(candidate, candidateProbe, candidateGround))
            {
                state_.positionMeters = candidate;
                state_.groundRadius   = 0.0;
                state_.grounded       = false; // left the ground this frame
                return true;
            }
            const double rise  = candidateGround - radius;
            const double grade = rise / attemptLength;
            if (rise > params_.walkMaxStepUpMeters || grade > params_.walkMaxClimbGrade)
            {
                return false; // a wall: blocked, the caller slides along it
            }
            state_.positionMeters = candidate;
            if (-rise > params_.walkGroundSnapMeters)
            {
                state_.groundRadius = 0.0;
                state_.grounded     = false; // a drop too deep to snap: fall
            }
            else
            {
                state_.groundRadius = candidateGround;
            }
            return true;
        };

        for (const Vec3 &attempt : attempts)
        {
            if (tryMove(attempt))
            {
                break;
            }
        }
    }

    // ---- vertical integration ------------------------------------------------------
    if (state_.grounded)
    {
        if (intent.jump && params_.jumpSpeedMetersPerSecond > 0.0)
        {
            verticalSpeed_  = params_.jumpSpeedMetersPerSecond;
            state_.grounded = false;
        }
        else
        {
            // Ride the surface (state_.groundRadius was refreshed by the
            // accepted move attempt; standing still keeps the last ground).
            const double currentRadius = length(state_.positionMeters);
            if (state_.groundRadius > 0.0 && std::abs(state_.groundRadius - currentRadius) < params_.walkGroundSnapMeters + params_.walkMaxStepUpMeters + 1.0)
            {
                state_.positionMeters = state_.positionMeters * (state_.groundRadius / currentRadius);
            }
            verticalSpeed_ = 0.0;
        }
    }
    if (!state_.grounded)
    {
        verticalSpeed_ -= params_.surfaceGravityMetersPerSecond2 * deltaSeconds;
        const Vec3 verticalStep = up * (verticalSpeed_ * deltaSeconds);
        Vec3       moved        = state_.positionMeters + verticalStep;
        if (verticalSpeed_ > 0.0 && field_.densityAt(moved) > 0.0)
        {
            // Rising into rock (an overhang ceiling): cancel the climb.
            verticalSpeed_ = 0.0;
            moved          = state_.positionMeters;
        }
        state_.positionMeters = moved;

        // Landing: a descending probe below the new position ends the fall
        // exactly on the surface (the buried branch of the search reports
        // the tunnelled-into ground when the step crossed it).
        double landingRadius = 0.0;
        const double landingProbe = std::max(params_.walkGroundSnapMeters,
            std::abs(verticalSpeed_) * deltaSeconds + 4.0 * kGroundTouchMeters);
        if (verticalSpeed_ <= 0.0 && groundRadiusWithin(state_.positionMeters, landingProbe, landingRadius))
        {
            const double landingHeight = length(state_.positionMeters);
            if (landingHeight <= landingRadius + kGroundTouchMeters)
            {
                state_.positionMeters = state_.positionMeters * (landingRadius / landingHeight);
                state_.groundRadius   = landingRadius;
                state_.grounded       = true;
                verticalSpeed_        = 0.0;
            }
        }
    }

    // ---- refresh the reported state --------------------------------------------------
    state_.up          = normalized(state_.positionMeters);
    state_.eyePosition = state_.positionMeters + state_.up * params_.eyeHeightMeters;
    state_.speed       = wantsMove ? speed : 0.0;
}

} // namespace planet
} // namespace engine
