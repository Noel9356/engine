#version 450
// Vertex stage of the render module's Vulkan backend.
//
// Inputs are the render capability's documented mesh layout (ABI 3 added the
// uv attribute; ABI 4 changed only the material, not the mesh):
// position, normal, color, uv (3+3+3+2 floats, stride 44 bytes).
//
// Push constants carry the per-draw state (112 bytes, within the 128-byte
// minimum guarantee - mobile GPUs included):
//   uModel             mat4   - object-to-world transform (column-major)
//   uBaseColorFactor   vec4   - material color multiplier (rgb used)
//   uMetallicRoughness vec4   - (metallicFactor, roughnessFactor,
//                                normalScale, occlusionStrength)
//   uEmissive          vec4   - (emissiveColor.rgb, unlit flag)
// The per-frame state comes from the uniform buffer bound at set 0, binding
// 0 (view-projection with the Vulkan clip-space fixup pre-applied, camera
// position, and the pbr module's lighting environment - see mesh.frag), so
// consumers keep supplying OpenGL-convention matrices through the agnostic
// capability. The material's five maps are sampled at set 1, bindings 0-4
// (fragment stage only).

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec2 aUv;
layout(location = 4) in vec4 aInstanceModel0;
layout(location = 5) in vec4 aInstanceModel1;
layout(location = 6) in vec4 aInstanceModel2;
layout(location = 7) in vec4 aInstanceModel3;

layout(push_constant) uniform PushData
{
    mat4 uModel;
    vec4 uBaseColorFactor;
    vec4 uMetallicRoughness;
    vec4 uEmissive;
} pushData;

layout(std140, set = 0, binding = 0) uniform FrameData
{
    mat4 uViewProj;
    vec4 uCameraPos;
    vec4 uSunDirection;
    vec4 uSunColor;
    vec4 uAmbientSky;
    vec4 uAmbientGround;
    vec4 uToneMapping;
} frameData;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec3 vColor;
layout(location = 2) out vec2 vUv;
layout(location = 3) out vec3 vWorldPosition;

void main()
{
    const mat4 instanceModel = mat4(
        aInstanceModel0, aInstanceModel1, aInstanceModel2, aInstanceModel3);
    const mat4 model = pushData.uModel * instanceModel;
    gl_Position = frameData.uViewProj * model * vec4(aPosition, 1.0);
    // SAFETY: inverse-transpose normal transform. The previous mat3(model)
    // was wrong for non-uniformly scaled models (sheared normals produce
    // incorrect lighting). The physically correct transform is
    // transpose(inverse(mat3(model))). For uniformly scaled models (the
    // common case) the result is identical up to a scalar; the fragment
    // stage renormalizes so only direction matters. Zero regression risk —
    // identical output for uniform scale, correct output for non-uniform.
    const mat3 inverseTransposeModel = transpose(inverse(mat3(model)));
    vNormal        = inverseTransposeModel * aNormal;
    vColor         = aColor;
    vUv            = aUv;
    vWorldPosition = (model * vec4(aPosition, 1.0)).xyz;
}
