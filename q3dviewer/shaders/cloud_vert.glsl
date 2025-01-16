/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in uint value;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float alpha = 1;
uniform int color_mode = 0;
uniform int flat_rgb = 0;
uniform float vmin = 0;
uniform float vmax = 255;
uniform float focal = 1000;
uniform int point_type = 0; // 0 pixel, 1 flat square, 2 sphere
uniform float point_size = 0.01;  // World size for each point (meter)
out vec4 color;

vec3 getRainbowColor(uint value_raw) {
    float range = vmax - vmin;
    float value = 1.0 - (float(value_raw) - vmin) / range;
    value = clamp(value, 0.0, 1.0);
    float hue = value * 5.0 + 1.0;
    int i = int(floor(hue));
    float f = hue - float(i);
    if (mod(i, 2) == 0) f = 1.0 - f;
    float n = 1.0 - f;

    vec3 color;
    if (i <= 1) color = vec3(n, 0.0, 1.0);
    else if (i == 2) color = vec3(0.0, n, 1.0);
    else if (i == 3) color = vec3(0.0, 1.0, n);
    else if (i == 4) color = vec3(n, 1.0, 0.0);
    else color = vec3(1.0, n, 0.0);
    return color;
}

void main()
{
    vec4 pw = vec4(position, 1.0);
    vec4 pc = view_matrix * pw;
    gl_Position = projection_matrix * pc;

    // Calculate point size in pixels based on distance
    if (point_type == 0)
        gl_PointSize = int(point_size);
    else
        gl_PointSize = point_size / gl_Position.w * focal;
    vec3 c = vec3(1.0, 1.0, 1.0);
    if (color_mode == 1)
    {
        uint intensity = value >> 24;
        c = getRainbowColor(intensity);
    }
    else if(color_mode == 2)
    {
        c.z = float(value & uint(0x000000FF))/255.;
        c.y = float((value & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((value & uint(0x00FF0000)) >> 16)/255.;
    }
    else
    {
        c.z = float( uint(flat_rgb) & uint(0x000000FF))/255.;
        c.y = float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.;
    }
    color = vec4(c, alpha);
}
