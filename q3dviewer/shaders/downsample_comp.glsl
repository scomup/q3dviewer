#version 430 core
/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.

GPU-only in-place point cloud downsampling compute shader.
Keeps every other point: buf[i] = buf[i * 2]

Safety: dst index i <= src index i*2 always holds,
so no write ever overtakes its own read. Zero extra GPU memory.
*/

layout(local_size_x = 256) in;

// Single buffer — dual role as src (i*2) and dst (i).
// Each point is exactly 16 bytes (float x,y,z + uint irgb).
// vec4 (std430: 16-byte aligned/sized) maps 1:1 to the point stride.
layout(std430, binding = 0) buffer Points {
    vec4 buf[];
};

// Number of output points = valid_buff_top / 2
uniform uint num_dst_points;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= num_dst_points) return;
    // i <= i*2 always  =>  write never reaches an unread slot.
    buf[i] = buf[i * 2u];
}
