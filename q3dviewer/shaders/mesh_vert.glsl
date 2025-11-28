#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in float aColor;

out vec3 FragPos;
out vec3 Normal;
out vec3 objectColor;

uniform mat4 view;
uniform mat4 projection;
uniform int flat_rgb;

void main()
{
    FragPos = aPos;
    
    Normal = aNormal;
    
    int color_int = int(aColor);
    objectColor = vec3(
        float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.,
        float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.,
        float( uint(flat_rgb) & uint(0x000000FF))/255.
    ) ;
    
    // Final vertex position (apply view/projection)
    gl_Position = projection * view * vec4(aPos, 1.0);
}