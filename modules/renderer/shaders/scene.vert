#version 450
#extension GL_GOOGLE_include_directive : require
// Geometry pass vertex shader: unit primitives (unit box, corners at +-1;
// unit radius sphere) are placed by per-object push constants. The model
// scales by the primitive's half extents, so a scaled unit box spans exactly
// the box the analytic SDF describes - rasterized and traced geometry agree. Local position/normal go to the fragment shader,
// which derives UVs and the tangent frame from them with the same functions
// the SDF tracing passes use.


layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

// std140 mirror of render_interface.hpp CameraData.
layout(std140, set = 0, binding = 0) uniform CameraUniform
{
    mat4 viewProjection;
    mat4 inverseViewProjection;
    vec4 cameraPosition;
};

layout(push_constant) uniform ObjectPush
{
    layout(offset = 0) mat4 model;
    layout(offset = 64) vec4 albedoRoughness;          // rgb albedo, a roughness
    layout(offset = 80) vec4 metallicEmissiveStrength; // x metallic, y emissive strength
    layout(offset = 96) vec4 materialLayers;           // x albedo layer, y data layer (-1 none), z uv scale, w primitive type
} object;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec3 outLocalPosition;
layout(location = 3) out vec3 outLocalNormal;

void main()
{
    const vec4 world = object.model * vec4(inPosition, 1.0);
    outWorldPosition = world.xyz;
    // Axis-aligned scaling and Y rotation preserve direction after
    // normalization; spheres use a uniform radius, so the normal stays exact.
    outWorldNormal = normalize(mat3(object.model) * inNormal);
    // Local position in METERS: the model's column lengths are the axis
    // scales (position + rotY + scale model), so unitLocal * size equals the
    // rotated-back local point the SDF tracing passes reconstruct from world
    // positions - both sides of the material system measure UVs the same way.
    outLocalPosition = inPosition * vec3(length(object.model[0].xyz), length(object.model[1].xyz),
                                         length(object.model[2].xyz));
    outLocalNormal = inNormal;
    gl_Position = viewProjection * world;
}
