// Analytic eroded-mountain relief: the Advanced Terrain Erosion filter (see
// erosion.hpp for the technique and attribution). Everything below is a pure
// function of (direction, cutoff, params, seed) unless it is an explicitly
// marked thread-local cache; caches only ever hold recomputable values.

#include <engine/planet/erosion.hpp>

#include <engine/planet/fbm.hpp>
#include <engine/planet/flat_key_map.hpp>

#include <atomic>
#include <bit>
#include <cmath>

namespace engine
{
namespace planet
{
namespace
{

constexpr double kTau = 6.28318530717958647692;

struct Vec2
{
    double x = 0.0;
    double y = 0.0;
};

// Engine lattice-hash convention (see noise.cpp): full-width multiply / add
// chains over wrapped int64 values. The original filter hashes its cell
// pivots and noise corners with a shader-style irrational hash (fractions of
// products of 1/pi and e^-1); that hash keeps its fraction only while the
// products stay inside double precision's 2^53 integer headroom, which the
// module's 64-bit per-stream seeds leave immediately. The substitution
// changes which random values the cells draw, nothing about the filter's
// mathematics.
inline void hashLattice2(std::int64_t seed, std::int64_t ix, std::int64_t iy, double &outX, double &outY)
{
    std::uint64_t state = static_cast<std::uint64_t>(seed);
    state = hashCombine64(state, static_cast<std::uint64_t>(ix));
    state = hashCombine64(state, static_cast<std::uint64_t>(iy));
    outX = -1.0 + 2.0 * static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
    outY = -1.0 + 2.0 * static_cast<double>(hashCombine64(state, 0x51A7E5C3D2F1B09Aull) >> 11)
        * (1.0 / 9007199254740992.0);
}

// The easing helpers of the published filter, kept shape-identical.
inline double clamp01Local(double value)
{
    return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

inline double easeOut(double t)
{
    const double v = 1.0 - clamp01Local(t);
    return 1.0 - v * v;
}

inline double smoothStart(double t, double smoothing)
{
    if (smoothing <= 0.0 || t >= smoothing)
    {
        return t - 0.5 * smoothing;
    }
    return 0.5 * t * t / smoothing;
}

inline double powInv(double t, double power)
{
    return 1.0 - std::pow(1.0 - clamp01Local(t), power);
}

// Gradient value noise with analytic derivatives (the filter's terrain
// function; the original credits https://www.shadertoy.com/view/XdXBRH).
// Corner gradients come from the engine lattice hash above.
struct NoisedResult
{
    double value = 0.0;
    double dx    = 0.0;
    double dy    = 0.0;
};

NoisedResult noised(std::int64_t seed, double x, double y)
{
    const double floorX = std::floor(x);
    const double floorY = std::floor(y);
    const std::int64_t ix = static_cast<std::int64_t>(floorX);
    const std::int64_t iy = static_cast<std::int64_t>(floorY);
    const double fx = x - floorX;
    const double fy = y - floorY;

    const double ux = fx * fx * fx * (fx * (fx * 6.0 - 15.0) + 10.0);
    const double uy = fy * fy * fy * (fy * (fy * 6.0 - 15.0) + 10.0);
    const double dux = 30.0 * fx * fx * (fx * (fx - 2.0) + 1.0);
    const double duy = 30.0 * fy * fy * (fy * (fy - 2.0) + 1.0);

    double gax = 0.0, gay = 0.0;
    double gbx = 0.0, gby = 0.0;
    double gcx = 0.0, gcy = 0.0;
    double gdx = 0.0, gdy = 0.0;
    hashLattice2(seed, ix, iy, gax, gay);
    hashLattice2(seed, ix + 1, iy, gbx, gby);
    hashLattice2(seed, ix, iy + 1, gcx, gcy);
    hashLattice2(seed, ix + 1, iy + 1, gdx, gdy);

    const double va = gax * fx + gay * fy;
    const double vb = gbx * (fx - 1.0) + gby * fy;
    const double vc = gcx * fx + gcy * (fy - 1.0);
    const double vd = gdx * (fx - 1.0) + gdy * (fy - 1.0);

    NoisedResult result;
    result.value = va + ux * (vb - va) + uy * (vc - va) + ux * uy * (va - vb - vc + vd);
    result.dx = gax + ux * (gbx - gax) + uy * (gcx - gax) + ux * uy * (gax - gbx - gcx + gdx)
        + dux * (uy * (va - vb - vc + vd) + vb - va);
    result.dy = gay + ux * (gby - gay) + uy * (gcy - gay) + ux * uy * (gay - gby - gcy + gdy)
        + duy * (ux * (va - vb - vc + vd) + vc - va);
    return result;
}

// Phacelle noise: a stripe pattern aligned with the input direction, built
// by interpolating the cosine / sine circle points of per-cell wave stripes
// (each cell pivots its stripes around a hashed point, like Worley noise)
// and partially re-normalizing the interpolated circle point so stripes keep
// a consistent magnitude without the loops full normalization produces.
struct PhacelleResult
{
    double cosValue = 0.0; // stripe height (cosine wave)
    double sinValue = 0.0; // stripe slope (sine wave)
    double dirX     = 0.0; // stripe direction vector (sideDir); multiply the
    double dirY     = 0.0; // sine by this for the height gradient
};

PhacelleResult phacelleNoise(
    std::int64_t seed, double px, double py, double dirX, double dirY, double freq, double offset, double normalization)
{
    // A vector orthogonal to the stripe direction, scaled so one unit of
    // travel along it advances the wave phase by a full turn times freq.
    const double sideX = dirY * -1.0 * freq * kTau;
    const double sideY = dirX * 1.0 * freq * kTau;
    const double phaseOffset = offset * kTau;

    const double floorX = std::floor(px);
    const double floorY = std::floor(py);
    const std::int64_t cellX = static_cast<std::int64_t>(floorX);
    const std::int64_t cellY = static_cast<std::int64_t>(floorY);
    const double fracX = px - floorX;
    const double fracY = py - floorY;

    double cosSum = 0.0;
    double sinSum = 0.0;
    double weightSum = 0.0;
    for (int i = -1; i <= 2; ++i)
    {
        for (int j = -1; j <= 2; ++j)
        {
            double hashX = 0.0;
            double hashY = 0.0;
            hashLattice2(seed, cellX + i, cellY + j, hashX, hashY);
            const double offsetX = hashX * 0.5;
            const double offsetY = hashY * 0.5;

            const double vx = fracX - static_cast<double>(i) - offsetX;
            const double vy = fracY - static_cast<double>(j) - offsetY;

            // Bell-shaped weight: 1 at the cell point, zero past 1.5 units
            // (the closest any out-of-loop cell point can sit). The subtract
            // lands the weight at exactly 0 there, hiding subtle grid lines.
            double weight = std::exp(-2.0 * (vx * vx + vy * vy));
            weight -= 0.01111;
            if (weight <= 0.0)
            {
                continue;
            }

            weightSum += weight;
            const double waveInput = vx * sideX + vy * sideY + phaseOffset;
            cosSum += std::cos(waveInput) * weight;
            sinSum += std::sin(waveInput) * weight;
        }
    }

    const double scale = 1.0 / std::max(weightSum, 1.0e-12);
    const double interpolatedX = cosSum * scale;
    const double interpolatedY = sinSum * scale;

    // Partial normalization: only lengths above (1 - normalization) compress
    // back to the unit circle, which keeps stripe magnitudes consistent
    // without the spiky loops full normalization produces where interpolated
    // waves cancel out.
    const double magnitude = std::sqrt(interpolatedX * interpolatedX + interpolatedY * interpolatedY);
    const double floorMagnitude = std::max(1.0 - normalization, magnitude);
    const double normalize = 1.0 / std::max(floorMagnitude, 1.0e-12);

    PhacelleResult result;
    result.cosValue = interpolatedX * normalize;
    result.sinValue = interpolatedY * normalize;
    result.dirX = sideX;
    result.dirY = sideY;
    return result;
}

} // namespace

namespace
{

// Packed lattice cell index: three signed 20-bit axes (the stride floor in
// the constructor keeps every supported radius inside the range, the same
// packing the mesher's macro-climate grid uses).
struct ErosionCellKeyPacker
{
    static constexpr std::int64_t kOffset = static_cast<std::int64_t>(1) << 19;

    [[nodiscard]] static std::uint64_t pack(std::int64_t ix, std::int64_t iy, std::int64_t iz)
    {
        const std::uint64_t x = static_cast<std::uint64_t>(ix + kOffset);
        const std::uint64_t y = static_cast<std::uint64_t>(iy + kOffset);
        const std::uint64_t z = static_cast<std::uint64_t>(iz + kOffset);
        return (x << 40u) | (y << 20u) | z;
    }
};

struct ErosionCornerEntry
{
    std::uint64_t fieldId = 0;      // which field computed the value
    std::uint64_t cutoffBits = 0;   // at which band limit
    MountainErosionSample sample;
};

std::atomic<std::uint64_t> nextErosionFieldId{1};

} // namespace

MountainErosionField::MountainErosionField(const PlanetGenerationParams &params, std::int64_t seed)
    : params_(&params)
    , seed_(seed)
    , fieldId_(nextErosionFieldId.fetch_add(1, std::memory_order_relaxed))
    , latticeStrideFloorMeters_(std::max(1.0, params.radiusMeters / 600000.0))
    // The filter's length unit is METRIC by default (see mountainErosionUnit
    // - meters in params.hpp): the gully/valley scale is an absolute real-
    // world size, independent of the belt-placement wavelength the mountain
    // term rides. A unit of 0 derives it from the wavelength (the small-
    // planet convenience path where the two scales coincide).
    , metersPerUnit_(params.mountainErosionUnitMeters > 0.0
            ? params.mountainErosionUnitMeters
            : std::max(1.0, params.mountainWavelengthMeters))
{
}

MountainErosionSample MountainErosionField::sampleFaceExact(const Vec3 &unitDirection, double lowPassCutoffMeters) const
{
    const PlanetGenerationParams &p = *params_;

    // ---- cubemap projection -------------------------------------------------
    // The dominant-axis faces carry the 2D filter around the sphere; near
    // face edges the two (at corners three) overlapping faces blend with a
    // smoothstep window, so stripes continue across seams (blending two
    // unaligned sine stripes yields a smaller sine - the property the whole
    // technique leans on). The gnomonic face map stretches metric scale by
    // up to ~3x toward the corners; the filter's wavelengths are calibrated
    // at the face centers, which keeps the belt-scale morphology readable
    // everywhere (mountain gullies tolerate far more scale variance than
    // the gridded textures this projection would bother).
    const double absX = std::abs(unitDirection.x);
    const double absY = std::abs(unitDirection.y);
    const double absZ = std::abs(unitDirection.z);
    const double maxAbs = std::max(absX, std::max(absY, absZ));
    const double blendWidth = std::max(p.mountainErosionFaceBlendWidth, 1.0e-10);
    const double blendStart = maxAbs - blendWidth;

    // p spans [0, faceFrequency] over a face; one p unit = one mountain
    // wavelength of arc at the face center. The gnomonic remap folds a face
    // into uv [0, 1], so one uv unit is half a radian of arc there:
    // faceFrequency = (radius / 2) / metersPerUnit_.
    const double faceFrequency = 0.5 * p.radiusMeters / metersPerUnit_;

    double heightSum = 0.0;
    double ridgeSum = 0.0;
    double weightSum = 0.0;
    const auto accumulateFace = [&](double u, double v)
    {
        const double px = (u * 0.5 + 0.5) * faceFrequency;
        const double py = (v * 0.5 + 0.5) * faceFrequency;

        // ---- terrain height with derivatives (faded per octave like every
        // fbm stream of the module; the fade multiplies both the value and
        // the derivative contributions so the band-limited field and its
        // gradient stay consistent).
        double terrainValue = 0.0;
        double terrainDx = 0.0;
        double terrainDy = 0.0;
        double amplitude = 1.0;
        double frequency = std::max(0.25, p.mountainErosionTerrainFrequency);
        for (std::uint32_t octave = 0; octave < p.mountainErosionTerrainOctaves; ++octave)
        {
            const double wavelengthMeters = metersPerUnit_ / frequency;
            const double fade = lowPassFade(lowPassCutoffMeters, wavelengthMeters);
            if (fade <= 0.0)
            {
                break; // octaves shorten monotonically: later fades are zero too
            }
            const NoisedResult n = noised(seed_, px * frequency, py * frequency);
            terrainValue += amplitude * fade * n.value;
            terrainDx += amplitude * fade * frequency * n.dx;
            terrainDy += amplitude * fade * frequency * n.dy;
            amplitude *= p.mountainErosionTerrainGain;
            frequency *= p.mountainErosionTerrainLacunarity;
        }
        terrainValue *= p.mountainErosionTerrainAmplitude;
        terrainDx *= p.mountainErosionTerrainAmplitude;
        terrainDy *= p.mountainErosionTerrainAmplitude;

        // Fade target: -1 at the input terrain's valleys, +1 at its peaks;
        // the gully octaves fade toward it wherever the slope is too small
        // to carry gullies (this is what keeps peaks crisp and valley floors
        // from bulging). Overshooting is fine - it is clamped.
        const double inputFadeTarget = (terrainValue / std::max(1.0e-9, p.mountainErosionTerrainAmplitude * 0.6));

        // The filter tracks heights in the [0, 1] band around 0.5.
        double height = 0.5 + 0.5 * terrainValue;
        double heightDx = 0.5 * terrainDx;
        double heightDy = 0.5 * terrainDy;

        // ---- erosion filter --------------------------------------------------
        double strength = p.mountainErosionStrength * p.mountainErosionScale;
        double fadeTarget = std::min(1.0, std::max(-1.0, inputFadeTarget));
        double stripeFrequency = 1.0 / std::max(1.0e-6, p.mountainErosionScale * p.mountainErosionCellScale);
        double slopeLength = std::max(std::sqrt(heightDx * heightDx + heightDy * heightDy), 1.0e-10);
        double magnitude = 0.0;
        double roundingMultiplier = 1.0;

        const double roundingForInput = mix(p.mountainErosionRoundingCrease, p.mountainErosionRoundingRidge,
            clamp01(fadeTarget + 0.5))
            * p.mountainErosionRoundingGlobal;
        double combiMask = easeOut(smoothStart(slopeLength * p.mountainErosionOnsetInput,
            roundingForInput * p.mountainErosionOnsetInput));

        double ridgeMapCombiMask = easeOut(slopeLength * p.mountainErosionOnsetRidgeMap);
        double ridgeMapFadeTarget = fadeTarget;

        // Gully directions use a fixed assumed slope magnitude blended over
        // the input terrain's own slope, so gullies stay aligned through
        // rounded peaks and valleys where the real slope vanishes.
        const double assumedX = heightDx / slopeLength * p.mountainErosionAssumedSlope;
        const double assumedY = heightDy / slopeLength * p.mountainErosionAssumedSlope;
        double gullySlopeX = mix(heightDx, assumedX, p.mountainErosionAssumedSlopeBlend);
        double gullySlopeY = mix(heightDy, assumedY, p.mountainErosionAssumedSlopeBlend);

        for (std::uint32_t octave = 0; octave < p.mountainErosionOctaves; ++octave)
        {
            // Band limit: a gully octave finer than the consuming chunk's
            // sampling cutoff drops out (scaled through the octave's
            // strength, like every faded fbm octave of the module).
            const double stripeWavelengthMeters =
                p.mountainErosionScale * p.mountainErosionCellScale * metersPerUnit_
                / std::pow(p.mountainErosionLacunarity, static_cast<double>(octave));
            const double fade = lowPassFade(lowPassCutoffMeters, stripeWavelengthMeters);
            if (fade <= 0.0)
            {
                break; // stripe octaves shorten monotonically
            }
            const double octaveStrength = strength * fade;

            const double gullyLength = std::max(std::sqrt(gullySlopeX * gullySlopeX + gullySlopeY * gullySlopeY), 1.0e-10);
            const PhacelleResult phacelle = phacelleNoise(seed_, px * stripeFrequency, py * stripeFrequency,
                gullySlopeX / gullyLength, gullySlopeY / gullyLength, p.mountainErosionCellScale, 0.25,
                p.mountainErosionNormalization);

            // Undo the pre-scaled input coordinates and flip the sign: the
            // slopes point downhill, the wave phase advances uphill.
            const double dirX = -phacelle.dirX * stripeFrequency;
            const double dirY = -phacelle.dirY * stripeFrequency;

            const double sloping = std::abs(phacelle.sinValue);

            // Straight gullies: feed the following octaves the SIGN of the
            // stripe slope (as if the stripes were triangle waves), so
            // smaller gullies branch off larger ones at clean angles instead
            // of curling along them.
            const double slopeSign = phacelle.sinValue < 0.0 ? -1.0 : 1.0;
            gullySlopeX += slopeSign * dirX * octaveStrength * p.mountainErosionGullyWeight;
            gullySlopeY += slopeSign * dirY * octaveStrength * p.mountainErosionGullyWeight;

            // Gully height and derivative; the derivative fades toward a
            // slope of zero (the fade target is a height, only relevant to
            // the height channel).
            const double gullyHeight = phacelle.cosValue;
            const double gullyDx = phacelle.sinValue * dirX;
            const double gullyDy = phacelle.sinValue * dirY;

            const double fadedHeight =
                mix(fadeTarget, gullyHeight * p.mountainErosionGullyWeight, combiMask);
            const double fadedDx = mix(0.0, gullyDx * p.mountainErosionGullyWeight, combiMask);
            const double fadedDy = mix(0.0, gullyDy * p.mountainErosionGullyWeight, combiMask);

            height += fadedHeight * octaveStrength;
            heightDx += fadedDx * octaveStrength;
            heightDy += fadedDy * octaveStrength;
            magnitude += octaveStrength;

            // Stacked fading: the faded gullies become the next octave's
            // fade target, and the combi mask tightens over the stripes'
            // own ridges and creases, so finer gullies leave the crest lines
            // of coarser ones intact.
            fadeTarget = fadedHeight;

            const double roundingForOctave = mix(p.mountainErosionRoundingCrease, p.mountainErosionRoundingRidge,
                clamp01(phacelle.cosValue + 0.5))
                * roundingMultiplier;
            const double newMask = easeOut(smoothStart(sloping * p.mountainErosionOnsetOctave,
                roundingForOctave * p.mountainErosionOnsetOctave));
            combiMask = powInv(combiMask, p.mountainErosionDetail) * newMask;

            // The ridge map runs its own fade / mask chain (its mask is not
            // detail-throttled; it reports where ridges and creases are).
            ridgeMapFadeTarget = mix(ridgeMapFadeTarget, gullyHeight, ridgeMapCombiMask);
            ridgeMapCombiMask *= easeOut(sloping * p.mountainErosionOnsetRidgeMask);

            strength *= p.mountainErosionGain;
            stripeFrequency *= p.mountainErosionLacunarity;
            roundingMultiplier *= p.mountainErosionRoundingGain;
        }

        const double ridgeMap = ridgeMapFadeTarget * (1.0 - ridgeMapCombiMask);

        // Height offset: a constant (optionally fade-target-blended) shift
        // scaled by the accumulated erosion magnitude, which compensates the
        // ridge-up bias of the gully stack and keeps the belt's mean height
        // stable (the recommended -0.65 lands the eroded profile back on the
        // input terrain's average).
        const double offset = mix(p.mountainErosionHeightOffsetLow, -fadeTarget, p.mountainErosionHeightOffsetBlend)
            * magnitude;

        MountainErosionSample sample;
        sample.height = height + offset;
        sample.ridgeMap = ridgeMap;
        return sample;
    };

    // Face uv (gnomonic) per dominant axis, faithful to the published
    // mapping (each face's u/v run counter to the shared edge's neighbor so
    // the blend regions overlap without mirroring).
    if (absX > 0.0)
    {
        const double weight = smoothstep(blendStart, maxAbs, absX);
        if (weight > 0.0)
        {
            const double u = (unitDirection.x >= 0.0 ? -unitDirection.z : unitDirection.z)
                / std::max(absX, 1.0e-10);
            const MountainErosionSample s = accumulateFace(u, unitDirection.y / std::max(absX, 1.0e-10));
            heightSum += s.height * weight;
            ridgeSum += s.ridgeMap * weight;
            weightSum += weight;
        }
    }
    if (absY > 0.0)
    {
        const double weight = smoothstep(blendStart, maxAbs, absY);
        if (weight > 0.0)
        {
            const double v = (unitDirection.y >= 0.0 ? -unitDirection.z : unitDirection.z)
                / std::max(absY, 1.0e-10);
            const MountainErosionSample s = accumulateFace(unitDirection.x / std::max(absY, 1.0e-10), v);
            heightSum += s.height * weight;
            ridgeSum += s.ridgeMap * weight;
            weightSum += weight;
        }
    }
    if (absZ > 0.0)
    {
        const double weight = smoothstep(blendStart, maxAbs, absZ);
        if (weight > 0.0)
        {
            const double u = (unitDirection.z >= 0.0 ? unitDirection.x : -unitDirection.x)
                / std::max(absZ, 1.0e-10);
            const MountainErosionSample s = accumulateFace(u, unitDirection.y / std::max(absZ, 1.0e-10));
            heightSum += s.height * weight;
            ridgeSum += s.ridgeMap * weight;
            weightSum += weight;
        }
    }

    if (weightSum <= 1.0e-10)
    {
        return MountainErosionSample{};
    }
    const double scale = 1.0 / weightSum;
    MountainErosionSample out;
    out.height = heightSum * scale;
    out.ridgeMap = ridgeSum * scale;
    return out;
}

MountainErosionSample MountainErosionField::sampleExact(const Vec3 &unitDirection, double lowPassCutoffMeters) const
{
    const double radius = length(unitDirection);
    if (radius < 1.0e-9)
    {
        return MountainErosionSample{};
    }
    return sampleFaceExact(Vec3{unitDirection.x / radius, unitDirection.y / radius, unitDirection.z / radius},
        lowPassCutoffMeters);
}

MountainErosionSample MountainErosionField::sample(
    const Vec3 &planetLocalPoint, double lowPassCutoffMeters, bool denseSampling) const
{
    const PlanetGenerationParams &p = *params_;

    // Finest stripe wavelength the configuration can produce.
    const double finestStripeMeters = std::max(1.0,
        p.mountainErosionScale * p.mountainErosionCellScale * metersPerUnit_
            / std::pow(p.mountainErosionLacunarity, static_cast<double>(p.mountainErosionOctaves - 1)));

    // The exact filter per point; see the header for when each path runs.
    const auto sampleDirect = [&]()
    {
        const double radius = length(planetLocalPoint);
        if (radius < 1.0e-9)
        {
            return MountainErosionSample{};
        }
        return sampleFaceExact(
            Vec3{planetLocalPoint.x / radius, planetLocalPoint.y / radius, planetLocalPoint.z / radius},
            lowPassCutoffMeters);
    };
    if (!denseSampling)
    {
        return sampleDirect();
    }

    // Lattice stride: a third of the finest wavelength that SURVIVES the
    // caller's band limit (features fade out below the cutoff and reach
    // full amplitude at twice it, so 2 x cutoff is the finest fully-alive
    // wavelength; three samples per period keep trilinear interpolation
    // error well under the surviving octaves' amplitude), rounded DOWN to
    // a power of two so neighboring cutoffs inside one octave of each
    // other share corner values exactly, and never below the radius-derived
    // packing floor. A caller whose sampling spacing already meets or
    // exceeds the stride cannot share cells between samples, so it takes
    // the direct path too (same values either side of the boundary - the
    // lattice resolves everything the caller's cutoff keeps).
    const double finestNeeded = std::max(finestStripeMeters, 2.0 * lowPassCutoffMeters);
    double stride = finestNeeded / 3.0;
    stride = std::exp2(std::floor(std::log2(std::max(stride, 1.0e-6))));
    stride = std::max(stride, latticeStrideFloorMeters_);
    if (lowPassCutoffMeters >= stride)
    {
        return sampleDirect();
    }

    const double fx = planetLocalPoint.x / stride;
    const double fy = planetLocalPoint.y / stride;
    const double fz = planetLocalPoint.z / stride;
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(fx));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(fy));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(fz));

    // Corner cache: values depend only on (cell, band limit, field), and the
    // same corners serve every caller of a cell - neighboring chunks, probe
    // rays marching along one direction, octree probes. The table is
    // thread-local scratch (like the mesher's macro-climate corner cache);
    // entries carry their field and cutoff identity so interleaved planets
    // or LOD depths never read a stale value, and the table clears itself
    // once streaming has filled it (recomputing a corner is cheap).
    thread_local FlatKeyMap<ErosionCornerEntry> cornerCache(256);
    if (cornerCache.size() > 4096)
    {
        cornerCache.clear();
    }

    // Active-cell memo: consecutive samples overwhelmingly land in the same
    // cell (a chunk build walks its grid, a probe ray holds its direction),
    // so the eight corners and their weights live in locals until the cell
    // changes.
    struct CellMemo
    {
        bool valid = false;
        std::uint64_t fieldId = 0;
        std::uint64_t cutoffBits = 0;
        std::int64_t cell[3] = {0, 0, 0};
        MountainErosionSample corners[8];
    };
    thread_local CellMemo memo;

    const std::uint64_t fieldId = fieldId_;
    const std::uint64_t cutoffBits = std::bit_cast<std::uint64_t>(lowPassCutoffMeters);
    if (!memo.valid || memo.fieldId != fieldId || memo.cutoffBits != cutoffBits || memo.cell[0] != ix
        || memo.cell[1] != iy || memo.cell[2] != iz)
    {
        const auto cornerAt = [&](std::int64_t cx, std::int64_t cy, std::int64_t cz)
        {
            const std::uint64_t key = ErosionCellKeyPacker::pack(cx, cy, cz);
            if (const ErosionCornerEntry *cached = cornerCache.find(key))
            {
                if (cached->fieldId == fieldId && cached->cutoffBits == cutoffBits)
                {
                    return cached->sample;
                }
            }
            const Vec3 cornerPosition{
                static_cast<double>(cx) * stride, static_cast<double>(cy) * stride, static_cast<double>(cz) * stride};
            const MountainErosionSample evaluated = sampleExact(cornerPosition, lowPassCutoffMeters);
            ErosionCornerEntry &entry = cornerCache.findOrInsert(key);
            entry.fieldId = fieldId;
            entry.cutoffBits = cutoffBits;
            entry.sample = evaluated;
            return evaluated;
        };

        memo.corners[0] = cornerAt(ix, iy, iz);
        memo.corners[1] = cornerAt(ix + 1, iy, iz);
        memo.corners[2] = cornerAt(ix, iy + 1, iz);
        memo.corners[3] = cornerAt(ix + 1, iy + 1, iz);
        memo.corners[4] = cornerAt(ix, iy, iz + 1);
        memo.corners[5] = cornerAt(ix + 1, iy, iz + 1);
        memo.corners[6] = cornerAt(ix, iy + 1, iz + 1);
        memo.corners[7] = cornerAt(ix + 1, iy + 1, iz + 1);
        memo.valid = true;
        memo.fieldId = fieldId;
        memo.cutoffBits = cutoffBits;
        memo.cell[0] = ix;
        memo.cell[1] = iy;
        memo.cell[2] = iz;
    }

    // Trilinear interpolation. The fractional weights are exact doubles of
    // the sample position, so every caller of a point reads bit-identical
    // values (the shared-lattice guarantee the mesher's grids give).
    const double tx = fx - static_cast<double>(ix);
    const double ty = fy - static_cast<double>(iy);
    const double tz = fz - static_cast<double>(iz);

    const auto lerp2 = [](const MountainErosionSample &a, const MountainErosionSample &b, double t)
    {
        MountainErosionSample out;
        out.height = a.height + (b.height - a.height) * t;
        out.ridgeMap = a.ridgeMap + (b.ridgeMap - a.ridgeMap) * t;
        return out;
    };
    const MountainErosionSample x00 = lerp2(memo.corners[0], memo.corners[1], tx);
    const MountainErosionSample x10 = lerp2(memo.corners[2], memo.corners[3], tx);
    const MountainErosionSample x01 = lerp2(memo.corners[4], memo.corners[5], tx);
    const MountainErosionSample x11 = lerp2(memo.corners[6], memo.corners[7], tx);
    const MountainErosionSample y0 = lerp2(x00, x10, ty);
    const MountainErosionSample y1 = lerp2(x01, x11, ty);
    return lerp2(y0, y1, tz);
}

} // namespace planet
} // namespace engine
