#version 450
// Vertex stage of the renderer module's shell-fur pipeline.
//
// Technique (Lengyel et al. 2001, "Real-Time Fur over Arbitrary Surfaces",
// the shells half; silhouette fins are replaced by the soft undercoat ramp
// documented in fur.frag): the caller submits the SAME mesh once per fur
// shell layer; this stage displaces every vertex outward along its normal by
// (layerIndex / layerCount) * furHeight, so the stack of shell layers carries
// the strand texture that fur.frag crops out per layer.
//
// Inputs are the renderer capability's documented mesh layout, identical to
// shaders/mesh.vert (position, normal, color, uv + per-instance mat4).
//
// Push constants carry the per-draw state (128 bytes - exactly the Vulkan
// minimum guarantee, mobile GPUs included). The first 112 bytes are the mesh
// pipeline's PushData block unchanged; the extra vec4 is the fur state:
//   uFur = (layerIndex, layerCount, furHeight, gravityStrength)
// The material's uMetallicRoughness.z / .w are REINTERPRETED on this pipeline:
// they carry (densityBoost, outerLayerFade) because fur shading never reads
// the normal-map/occlusion factors (see fur.frag). uEmissive keeps its mesh
// semantics (rgb emissive color, w unlit flag).
//
// Per-frame state comes from the same set-0 uniform buffer as mesh.vert and
// the material's five maps are sampled at set 1 (fur.frag uses only the
// base-color map; the fur strand field rides its ALPHA channel).
//
// Lighting-model parity: fur.frag mirrors the CPU path in
// source/core/render_shared.hpp (furShade) the same way mesh.frag mirrors
// pbrShade - changing either side requires updating the other.

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
    vec4 uMetallicRoughness; // on the fur pipeline: (metallic, roughness, densityBoost, outerLayerFade)
    vec4 uEmissive;          // rgb emissiveColor, w unlit flag
    vec4 uFur;               // (layerIndex, layerCount, furHeight, gravityStrength)
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
// Shell layer ratio in 0..1 (0 = base surface, 1 = strand tips). fur.frag
// compares it against the strand height field in the fur mask's alpha.
layout(location = 4) out float vShellRatio;

// Per-vertex strand variation, hashed from the object-space position. The
// classic shell-fur implementations bake per-strand length jitter into the
// fur texture; this hash adds per-VERTEX jitter on top (shells of the same
// texel keep a consistent field because the hash is position-derived), which
// breaks up the perfect concentric layering without new vertex attributes.
float strandJitter(vec3 position)
{
    return fract(sin(dot(position, vec3(12.9898, 78.233, 37.719))) * 43758.5453);
}

void main()
{
    const float layerIndex = max(pushData.uFur.x, 0.0);
    const float layerCount = max(pushData.uFur.y, 1.0);
    const float t          = clamp(layerIndex / layerCount, 0.0, 1.0);

    const float jitter = strandJitter(aPosition);
    // Height grows linearly with the layer; the jitter term (0.7..1.3) makes
    // individual strand roots/tips uneven so consecutive layers do not read
    // as evenly spaced sheets.
    const float height = pushData.uFur.z * t * (0.7 + 0.6 * jitter);
    const vec3  displaced = aPosition + aNormal * height;

    const mat4 instanceModel = mat4(
        aInstanceModel0, aInstanceModel1, aInstanceModel2, aInstanceModel3);
    const mat4 model = pushData.uModel * instanceModel;
    vec4 worldPosition = model * vec4(displaced, 1.0);
    // Gravity droop: tips sink toward -Y in world space, quadratically in the
    // strand height, so roots stay anchored while long strands sag. Applied
    // after the model transform so it is world-down regardless of orientation.
    worldPosition.y -= pushData.uFur.w * height * height;

    gl_Position = frameData.uViewProj * worldPosition;
    // Same inverse-transpose normal transform as mesh.vert (uniform scale is
    // the common case; fur.frag renormalizes).
    const mat3 inverseTransposeModel = transpose(inverse(mat3(model)));
    vNormal        = inverseTransposeModel * aNormal;
    vColor         = aColor;
    vUv            = aUv;
    vWorldPosition = worldPosition.xyz;
    vShellRatio    = t;
}
