#version 430 core

/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

// Triangle face attributes (per-instance) - 3 vertex positions
layout(location = 1) in vec3 v0;
layout(location = 2) in vec3 v1;
layout(location = 3) in vec3 v2;

// Uniforms
uniform mat4 view;
uniform mat4 projection;

// Outputs to fragment shader (via geometry shader)
out VS_OUT {
    vec3 v0, v1, v2;
} vs_out;

void main()
{
    // Pass vertex positions directly (no SSBO lookup needed)
    vs_out.v0 = v0;
    vs_out.v1 = v1;
    vs_out.v2 = v2;
    
    // Output dummy point (geometry shader will generate triangles)
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
}
