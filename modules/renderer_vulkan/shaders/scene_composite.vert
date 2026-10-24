// Composite pass vertex shader: one oversized triangle covers the viewport,
// no vertex buffer needed.

#version 450

layout(location = 0) out vec2 outUv;

void main()
{
    const vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    outUv = positions[gl_VertexIndex] * 0.5 + 0.5;
}
