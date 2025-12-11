#version 430 core

/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

// Input from vertex shader
in VS_OUT {
    vec3 v0, v1, v2, v3;
    float good;
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
    // Discard if face is not good
    if (gs_in[0].good != 1.0) {
        return;
    }
    
    vec3 v0 = gs_in[0].v0;
    vec3 v1 = gs_in[0].v1;
    vec3 v2 = gs_in[0].v2;
    vec3 v3 = gs_in[0].v3;
    
    float eps = 0.0001;
    
    // Use default light blue color
    vec3 color = vec3(
        float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.,
        float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.,
        float( uint(flat_rgb) & uint(0x000000FF))/255.
    );
    // Triangle 1: (v0, v1, v2)
    vec3 normal1 = calculateNormal(v0, v1, v2);
    // Skip degenerate triangles
    if (length(normal1) > eps) {
        emitVertex(v0, normal1, color);
        emitVertex(v1, normal1, color);
        emitVertex(v2, normal1, color);
        EndPrimitive();
    }
    
    // Triangle 2: (v0, v1, v3)
    vec3 normal2 = calculateNormal(v0, v1, v3);
    // Skip degenerate triangles
    if (length(normal2) > eps) {
        emitVertex(v0, normal2, color);
        emitVertex(v1, normal2, color);
        emitVertex(v3, normal2, color);
        EndPrimitive();
    }
}
