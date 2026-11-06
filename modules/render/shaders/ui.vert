#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec2 inUv;
layout(location = 2) in vec4 inColor;

layout(push_constant) uniform UiTransform
{
    vec2 scale;
    vec2 translate;
} transform;

layout(location = 0) out vec2 outUv;
layout(location = 1) out vec4 outColor;

void main()
{
    gl_Position = vec4(inPosition * transform.scale + transform.translate, 0.0, 1.0);
    outUv = inUv;
    outColor = inColor;
}
