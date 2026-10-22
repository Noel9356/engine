// Shared scene math for the renderer and lighting shaders.
//
// The SDF here must stay EXACTLY consistent with the rasterized geometry:
// both are built from the same GpuScenePrimitive list uploaded by the renderer
// (scene_description.hpp). Exact analytic distance functions are the first
// pillar of the no-light-leak guarantee - no voxelization, no distance-field
// texture, no coarseness artifacts (docs/robustness.md, problem 24).
//
// Consumers compile with: #extension GL_GOOGLE_include_directive : require

#ifndef ENGINE_SCENE_COMMON_GLSL
#define ENGINE_SCENE_COMMON_GLSL

const uint primitiveTypeBox = 0u;
const uint primitiveTypeSphere = 1u;
const uint maxPointLights = 16u;

struct GpuPrimitive
{
    vec4 positionType;             // xyz position, w type (0 box, 1 sphere)
    vec4 sizeRotation;             // xyz half extents / radius, w rotationY radians
    vec4 albedoRoughness;          // rgb albedo, a roughness
    vec4 metallicEmissiveStrength; // x metallic, y emissive strength
};

layout(std430, set = 0, binding = 1) readonly buffer PrimitiveBuffer
{
    GpuPrimitive primitives[];
};

layout(std140, set = 0, binding = 0) uniform CameraUniform
{
    mat4 viewProjection;
    mat4 inverseViewProjection;
    vec4 cameraPosition;
};

// World -> primitive-local, the inverse of the model matrix the geometry pass
// rasterizes with. modelMatrix() places a unit primitive with
// world = translate * rotateY(+radians) * scale, so recovering the local point
// a world position lies on requires the OPPOSITE rotation: the earlier call
// sites passed +radians here, which rotated the SDF box by TWICE the angle in
// the mirrored direction - every rotated object was traced as a ghost box
// sitting at -rotationY around its centre while the G-buffer showed it at
// +rotationY: shadows cast from the wrong silhouette, GI occluded by the wrong
// shape (blocky dark patches and leaks around the object), reflection rays
// hitting empty air where the object visibly stands, and material UVs sampled
// across a 2x rotation seam on every GI bounce and reflection hit. With
// rotation_y = 0 (the sealed leak-test room) both directions agree, which is
// why the automated leak check never caught it.
//
// The rotation's sin/cos are NOT recomputed here: the primitive record carries
// them baked at upload (metallicEmissiveStrength.zw = (sinY, cosY) of
// sizeRotation.w, see scene_description.hpp). This transform runs for every
// primitive on every sphere-tracing step - the march alone evaluates the field
// up to (extent/stride)^2 * rays * steps times per frame - and two
// transcendental re-evaluations inside that loop were the single largest
// per-frame cost the tracing path carried. Mathematically identical to
// rotateY(delta, -radians); only the evaluation site moved to the CPU.
vec3 worldToLocalY(vec3 worldDelta, vec2 rotationTrig)
{
    // rotateY(delta, -r): s = -sinY, c = cosY packed from the primitive.
    return vec3(rotationTrig.y * worldDelta.x - rotationTrig.x * worldDelta.z,
                worldDelta.y,
                rotationTrig.x * worldDelta.x + rotationTrig.y * worldDelta.z);
}

// ---- Surface parameterization ----------------------------------------------------
//
// UVs and tangent frames are pure functions of the primitive's LOCAL space so
// the rasterized geometry pass and the SDF relight compute IDENTICAL surface
// data for the same world point: the G-buffer pass feeds the interpolated
// vertex local position, the tracing passes transform their hit point with
// rotateY. Sharing the functions is what keeps texture tiling seamless
// between direct light, GI bounces and reflections.

// Tiling scale for one material: uv_scale is WORLD METERS PER TEXTURE TILE
// (uv = surface meters / uvScale). Box faces use the face plane's two axes,
// normalized by the face size so a texture tile covers uvScale world meters
// along both axes.
vec2 boxSurfaceUv(vec3 local, vec3 halfExtents, float uvScale)
{
    const vec3 relative = abs(local) / max(halfExtents, vec3(1e-5));
    vec2 uv;
    if (relative.x >= relative.y && relative.x >= relative.z)
    {
        uv = vec2(local.z, -local.y);
    }
    else if (relative.y >= relative.z)
    {
        uv = vec2(local.x, local.z);
    }
    else
    {
        uv = vec2(local.x, -local.y);
    }
    return uv / uvScale;
}

// Sphere UV from arc length: azimuth * radius and polar * radius, tiled by
// uvScale (world meters per tile). The seam wraps through the repeat sampler
// like any tiled face.
vec2 sphereSurfaceUv(vec3 local, float radius, float uvScale)
{
    const float azimuth = atan(local.z, local.x);
    const float polar = acos(clamp(local.y / max(radius, 1e-5), -1.0, 1.0));
    return vec2(azimuth, polar) * max(radius, 1e-5) / uvScale;
}

// World-space tangent frame for tangent-space normal maps. The tangent axes
// are derived from the SAME face mapping boxSurfaceUv uses, so a normal map's
// +X/+Y (OpenGL convention, ambientCG NormalGL) align with +U/+V on every
// box face; the sphere uses east/north at the surface point (poles degenerate
// to an arbitrary tangent - a known, acceptable limit).
void surfaceTbn(vec3 worldNormal, vec3 localNormal, mat3 modelRotation, uint primitiveType,
                out vec3 tangent, out vec3 bitangent)
{
    vec3 tangentLocal;
    vec3 bitangentLocal;
    if (primitiveType == primitiveTypeSphere)
    {
        tangentLocal = cross(localNormal, vec3(0.0, 1.0, 0.0));
        tangentLocal = length(tangentLocal) > 1e-5 ? normalize(tangentLocal) : vec3(1.0, 0.0, 0.0);
        bitangentLocal = cross(localNormal, tangentLocal);
    }
    else
    {
        const vec3 absolute = abs(localNormal);
        if (absolute.x >= absolute.y && absolute.x >= absolute.z)
        {
            tangentLocal = vec3(0.0, 0.0, 1.0);   // u = local.z
            bitangentLocal = vec3(0.0, -1.0, 0.0); // v = -local.y
        }
        else if (absolute.y >= absolute.z)
        {
            tangentLocal = vec3(1.0, 0.0, 0.0);   // u = local.x
            bitangentLocal = vec3(0.0, 0.0, 1.0); // v = local.z
        }
        else
        {
            tangentLocal = vec3(1.0, 0.0, 0.0);   // u = local.x
            bitangentLocal = vec3(0.0, -1.0, 0.0); // v = -local.y
        }
    }
    tangent = modelRotation * tangentLocal;
    tangent -= worldNormal * dot(tangent, worldNormal);
    tangent = normalize(tangent);
    bitangent = modelRotation * bitangentLocal;
    bitangent -= worldNormal * dot(bitangent, worldNormal);
    bitangent -= tangent * dot(bitangent, tangent);
    bitangent = normalize(bitangent);
}

float signedDistanceBox(vec3 point, vec3 halfExtents)
{
    const vec3 distance = abs(point) - halfExtents;
    const vec3 clamped = max(distance, 0.0);
    return length(clamped) + min(max(distance.x, max(distance.y, distance.z)), 0.0);
}

float signedDistanceSphere(vec3 point, float radius)
{
    return length(point) - radius;
}

// Exact scene SDF: minimum over all configured primitives. Cost is linear in
// the object count; the configuration caps it (max 32 objects).
//
// sceneSDFOwned is the ONE loop definition: it additionally tracks which
// primitive owns the minimum (first index on ties, the same rule
// primitiveOwnerAt applies). sceneSDF discards the owner - the compiler
// eliminates the dead tracking - so every existing consumer keeps its
// instruction mix while traced hits get the owner for free (see
// traceIntervalOwned below).
float sceneSDFOwned(vec3 point, uint primitiveCount, out uint owner)
{
    float minimum = 1e30;
    uint nearest = 0u;
    for (uint index = 0u; index < primitiveCount; ++index)
    {
        const GpuPrimitive primitive = primitives[index];
        const vec3 local = worldToLocalY(point - primitive.positionType.xyz, primitive.metallicEmissiveStrength.zw);
        float distance;
        if (primitive.positionType.w < 0.5)
        {
            distance = signedDistanceBox(local, primitive.sizeRotation.xyz);
        }
        else
        {
            distance = signedDistanceSphere(local, primitive.sizeRotation.x);
        }
        if (distance < minimum)
        {
            minimum = distance;
            nearest = index;
        }
    }
    owner = nearest;
    return minimum;
}

float sceneSDF(vec3 point, uint primitiveCount)
{
    uint unusedOwner;
    return sceneSDFOwned(point, primitiveCount, unusedOwner);
}

// Sphere tracing inside [tStart, tEnd] (the cascade interval). Returns the hit
// distance, or -1.0 when the ray leaves the interval unoccluded. The epsilon
// grows with distance so grazing rays stay conservative; a fixed epsilon is a
// classic light-leak source at thin walls.
//
// traceIntervalOwned additionally reports the owning primitive of the hit:
// the hit step already evaluated the field at the hit point, so the arg-min
// index is known there and the consumer's post-hit primitiveOwnerAt scan (one
// full linear pass over the primitives per hit) becomes unnecessary. The owner
// is only meaningful when the returned distance is >= 0. The plain
// traceInterval keeps its signature for callers that never use the owner.
float traceIntervalOwned(vec3 origin, vec3 direction, float tStart, float tEnd, uint primitiveCount,
                         uint maxSteps, out uint owner)
{
    float t = tStart;
    owner = 0u;
    for (uint step = 0u; step < maxSteps; ++step)
    {
        const vec3 point = origin + direction * t;
        uint stepOwner;
        const float distance = sceneSDFOwned(point, primitiveCount, stepOwner);
        const float epsilon = max(0.0004, 0.0008 * t);
        if (distance < epsilon)
        {
            owner = stepOwner;
            return t;
        }
        t += distance;
        if (t > tEnd)
        {
            return -1.0;
        }
    }
    return -1.0;
}

float traceInterval(vec3 origin, vec3 direction, float tStart, float tEnd, uint primitiveCount, uint maxSteps)
{
    uint unusedOwner;
    return traceIntervalOwned(origin, direction, tStart, tEnd, primitiveCount, maxSteps, unusedOwner);
}

// Visibility query between two world points used by the probe gather: 1.0
// when the straight line is clear, 0.0 when occluded well before the target,
// and a smooth blend when an occluder sits just barely inside the traced
// interval (a silhouette grazing the line right before the probe). Both
// endpoints are kept clear by a buffer: probes anchor ON surfaces, so a line
// between two points of the same floor would graze the floor itself and a
// naive march reports the floor as an occluder - which dropped every probe
// and made the whole GI black (found by the leak test). Grazing geometry in
// between still blocks.
//
// The blend replaces the earlier strictly binary 0/1 result: binary weights
// flip per probe between neighbouring pixels along every silhouette, which
// shows as blocky bands inside shadowed regions (the exact "shadow is
// blocky" artifact). The transition band scales with the ray length so the
// angular width of the blend stays roughly constant. The no-leak property is
// unchanged where it matters: an occluder that separates the endpoints by
// more than the band (any wall thicker than centimetres, hit early in the
// march) still yields exactly 0, and the automated leak check
// (engine_leak_check) enforces that on a sealed room.
float lineOfSight(vec3 from, vec3 to, uint primitiveCount, uint maxSteps)
{
    const vec3 delta = to - from;
    const float length = length(delta);
    if (length < 0.12)
    {
        return 1.0;
    }
    const vec3 direction = delta / length;
    const float hit = traceInterval(from, direction, 0.02, length - 0.06, primitiveCount, maxSteps);
    if (hit < 0.0)
    {
        return 1.0;
    }
    // Depth of the occluder before the cleared endpoint: 0 means it sat
    // right at the clearance boundary (the probe is marginally visible
    // around the silhouette), large means deep occlusion.
    const float remaining = (length - 0.06) - hit;
    const float softness = max(0.05, 0.05 * length);
    return clamp(1.0 - remaining / softness, 0.0, 1.0);
}

vec3 cameraRayDirection(mat4 inverseViewProjection, vec3 cameraPosition, vec2 normalizedCoordinates)
{
    const vec4 target = inverseViewProjection * vec4(normalizedCoordinates, 1.0, 1.0);
    return normalize(target.xyz / target.w - cameraPosition);
}

vec3 worldPositionFromDepth(mat4 inverseViewProjection, vec2 normalizedCoordinates, float depth)
{
    const vec4 target = inverseViewProjection * vec4(normalizedCoordinates, depth, 1.0);
    return target.xyz / target.w;
}

// Near/far view depth inside one screen cell, sampled on a 4x4 tap grid.
// Shared by the march and the gather so both agree on where a cell's probe is
// anchored. Sky taps (depth 1, no surface) are excluded from both bounds; an
// all-sky cell reports farDepth 1.0, which callers treat as "no surface".
// cellCenterUv is the cell centre in UV coordinates, cellSizeUv the cell's
// extent in the same units (stride / extent).
void cellDepthRange(sampler2D depthTexture, vec2 cellCenterUv, vec2 cellSizeUv,
                    out float nearDepth, out float farDepth)
{
    nearDepth = 1.0;
    farDepth = 1.0;
    for (int tapY = 0; tapY < 4; ++tapY)
    {
        for (int tapX = 0; tapX < 4; ++tapX)
        {
            const vec2 tapUv = cellCenterUv + (vec2(float(tapX), float(tapY)) / 3.0 - vec2(0.5)) * cellSizeUv;
            const float tapDepth = texture(depthTexture, clamp(tapUv, vec2(0.0), vec2(1.0))).r;
            if (tapDepth >= 1.0)
            {
                continue;
            }
            const vec3 tapPoint = worldPositionFromDepth(inverseViewProjection, tapUv * 2.0 - vec2(1.0), tapDepth);
            const float tapViewDepth = length(tapPoint - cameraPosition.xyz);
            nearDepth = min(nearDepth, tapViewDepth);
            farDepth = min(1.0, max(farDepth, tapViewDepth));
        }
    }
}

// Continuous view depth at one UV: the surface seen exactly there, or 1.0 for
// sky. The tap grid of cellDepthRange quantizes the near bound to the cell's
// tap lattice, which anchored the probes in visible steps along iso-depth
// contours (diagonal staircase bands on every surface seen at an angle);
// callers that position a probe use this smooth measure instead and keep the
// tap grid only for the mixed/sky CLASSIFICATION.
float viewDepthAt(sampler2D depthTexture, vec2 uv)
{
    const float depth = texture(depthTexture, uv).r;
    if (depth >= 1.0)
    {
        return 1.0;
    }
    const vec3 point = worldPositionFromDepth(inverseViewProjection, uv * 2.0 - vec2(1.0), depth);
    return length(point - cameraPosition.xyz);
}

#endif // ENGINE_SCENE_COMMON_GLSL
