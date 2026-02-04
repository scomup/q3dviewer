#version 430 core

/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

layout(points) in;
layout(triangle_strip, max_vertices = 3) out;

// Input from vertex shader
in VS_OUT {
    vec3 v0, v1, v2;
} gs_in[];

// Uniforms
uniform mat4 view;
uniform mat4 projection;

uniform int flat_rgb;

// Output to fragment shader
out vec3 FragPos;
out vec3 Normal;
out vec3 objectColor;

// Calculate normal from three vertices
vec3 calculateNormal(vec3 a, vec3 b, vec3 c) {
    vec3 edge1 = b - a;
    vec3 edge2 = c - a;
    return normalize(cross(edge1, edge2));
}

void emitVertex(vec3 pos, vec3 normal, vec3 color) {
    FragPos = pos;
    Normal = normal;
    objectColor = color;
    gl_Position = projection * view * vec4(pos, 1.0);
    EmitVertex();
}

void main()
{
    vec3 v0 = gs_in[0].v0;
    vec3 v1 = gs_in[0].v1;
    vec3 v2 = gs_in[0].v2;
    
    float eps = 0.0001;
    
    // Use color from uniform
    vec3 color = vec3(
        float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.,
        float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.,
        float( uint(flat_rgb) & uint(0x000000FF))/255.
    );
    
    // Triangle: (v0, v1, v2)
    vec3 normal = calculateNormal(v0, v1, v2);
    // Skip degenerate triangles
    if (length(normal) > eps) {
        emitVertex(v0, normal, color);
        emitVertex(v1, normal, color);
        emitVertex(v2, normal, color);
        EndPrimitive();
    }
}
