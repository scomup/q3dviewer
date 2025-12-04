#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in uint aColor;

out vec3 FragPos;
out vec3 Normal;
out vec3 objectColor;

uniform mat4 view;
uniform mat4 projection;
uniform int flat_rgb;
uniform int color_mode;  // 0: FLAT, 1: Intensity, 2: RGB
uniform float vmin;
uniform float vmax;

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
    FragPos = aPos;
    Normal = aNormal;
    
    vec3 c = vec3(1.0, 1.0, 1.0);
    
    if (color_mode == 0) {
        // FLAT: use uniform flat color
        c.z = float( uint(flat_rgb) & uint(0x000000FF))/255.;
        c.y = float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.;
    }
    else if (color_mode == 1) {
        // Intensity: use intensity channel (bits 24-31) for rainbow color
        uint intensity = aColor >> 24;
        c = getRainbowColor(intensity);
    }
    else if (color_mode == 2) {
        // RGB: use RGB channels (bits 0-23)
        c.z = float(aColor & uint(0x000000FF))/255.;
        c.y = float((aColor & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((aColor & uint(0x00FF0000)) >> 16)/255.;
    }
    
    objectColor = c;
    
    // Final vertex position (apply view/projection)
    gl_Position = projection * view * vec4(aPos, 1.0);
}