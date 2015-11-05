#version 410

in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;

uniform sampler2D Tex1;

uniform vec4 LightPosition;
uniform vec3 LightIntensity;

uniform vec3 Kd;            // Diffuse reflectivity
uniform vec3 Ka;            // Ambient reflectivity
uniform vec3 Ks;            // Specular reflectivity
uniform float Shininess;    // Specular shininess factor
uniform bool IsTexture;

layout( location = 0 ) out vec4 FragColor;

void main() {
    vec3 e = vec3(0,0,0);
    vec3 l = vec3(LightPosition);
    vec3 p = Position;
    vec3 Li = LightIntensity;

    vec3 n = normalize(Normal);
    vec3 s = normalize(l - p);
    vec3 v = normalize(e - p);   

    vec3 r = -s + 2 * dot(s, n) * n;

    vec3 Kd_tex;
    if(IsTexture)
        Kd_tex = vec3(texture2D( Tex1, TexCoord ));
    else
        Kd_tex = vec3(1, 1, 1);
    vec3 Ka_tex = Kd_tex;

    vec3 L = Li * (Ka * Ka_tex +                              // ambient
                   Kd * Kd_tex * max(0, dot(n, s)) +          // diffuse
                   Ks * pow( max(0, dot(r,v)), Shininess) );   // specular

    FragColor = vec4(L, 1.0); // no transparancy
}

