/*
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
*/

#version 330 core

uniform int point_type;

in vec4 color;

out vec4 finalColor;

void main()
{
    // only do this when point_type is sphere
    if (point_type == 2)
    {
        vec2 coord = gl_PointCoord * 2.0 - vec2(1.0); // Map [0,1] to [-1,1]
        float distance = dot(coord, coord); // Squared distance

        // Discard fragments outside the circle (radius = 1.0)
        if (distance > 1.0)
            discard;
    }

    finalColor = color;
}
