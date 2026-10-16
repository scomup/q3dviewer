#version 430 core

layout(local_size_x = 256) in;

layout(std430, binding = 0) readonly buffer Points {
    vec4 points[];
};

layout(std430, binding = 1) buffer SelectedIndices {
    uint indices[];
};

layout(std430, binding = 2) buffer SelectedCount {
    uint count;
};

uniform mat4 projection_view;
uniform vec4 rectangle_ndc;
uniform uint point_count;

void main() {
    uint point_index = gl_GlobalInvocationID.x;
    if (point_index >= point_count) {
        return;
    }

    vec4 clip = projection_view * vec4(points[point_index].xyz, 1.0);
    if (clip.w <= 0.0) {
        return;
    }

    vec2 ndc = clip.xy / clip.w;
    if (ndc.x < rectangle_ndc.x || ndc.x > rectangle_ndc.y
            || ndc.y < rectangle_ndc.z || ndc.y > rectangle_ndc.w) {
        return;
    }

    uint output_index = atomicAdd(count, 1u);
    indices[output_index] = point_index;
}
