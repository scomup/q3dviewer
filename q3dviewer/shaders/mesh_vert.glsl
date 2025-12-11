#version 430 core

/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

// Face attributes (per-instance) - vertex positions embedded in each face
layout(location = 1) in vec3 v0;
layout(location = 2) in vec3 v1;
layout(location = 3) in vec3 v2;
layout(location = 4) in vec3 v3;
layout(location = 5) in float good;

// Uniforms
uniform mat4 view;
uniform mat4 projection;

// Outputs to fragment shader (via geometry shader)
out VS_OUT {
    vec3 v0, v1, v2, v3;
    float good;
} vs_out;

void main()
{
    // Pass vertex positions directly (no SSBO lookup needed)
    vs_out.v0 = v0;
    vs_out.v1 = v1;
    vs_out.v2 = v2;
    vs_out.v3 = v3;
    vs_out.good = good;
    
    // Output dummy point (geometry shader will generate triangles)
    // Note: This position is not used; geometry shader generates actual triangles
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
}
