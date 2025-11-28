#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;  
in vec3 objectColor;

const vec3 LIGHT_DIR = normalize(vec3(1.0, 1.0, 1.0));
// Lighting uniforms
uniform bool if_light;
uniform bool two_sided;
// directional light - use a global constant direction (world space)
uniform vec3 light_color;
uniform vec3 view_pos;

// Material properties
uniform float ambient_strength;
uniform float diffuse_strength;
uniform float specular_strength;
uniform float shininess;
// Opacity
uniform float alpha;

void main()
{
    if (if_light)
    {
        // Ambient lighting
        vec3 ambient = ambient_strength * light_color;
        
        // Diffuse lighting with optional two-sided support
        vec3 norm = normalize(Normal);
        vec3 lightDir = LIGHT_DIR; // global directional light
        float diff;
        if (two_sided) {
            // Two-sided lighting - both front and back faces receive light
            diff = max(abs(dot(norm, lightDir)), 0.0);
        } else {
            // Standard one-sided lighting
            diff = max(dot(norm, lightDir), 0.0);
        }
        vec3 diffuse = diffuse_strength * diff * light_color;
        
        // Specular lighting (Phong reflection model) with optional two-sided support
    vec3 viewDir = normalize(view_pos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
        float spec;
        if (two_sided) {
            // Two-sided specular
            spec = pow(max(abs(dot(viewDir, reflectDir)), 0.0), shininess);
        } else {
            // Standard one-sided specular
            spec = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
        }
        vec3 specular = specular_strength * spec * light_color;
        
        // Combine all lighting components
        vec3 result = (ambient + diffuse + specular) * objectColor;
        FragColor = vec4(result, alpha);
    }
    else
    {
        // No lighting - just object color
        FragColor = vec4(objectColor, alpha);
    }
}