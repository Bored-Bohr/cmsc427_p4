#version 330

in vec3 Position;
in vec3 Normal;

uniform vec4 LightPosition;
uniform vec3 LightIntensity;

in vec3 Kd;            // Diffuse reflectivity
in vec3 Ka;            // Ambient reflectivity
in vec3 Ks;            // Specular reflectivity
in float Shininess;    // Specular shininess factor

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
    vec3 L = Li * ( Ka + Kd * max(0, dot(n, s)) + Ks * pow( max(0, dot(r,v)), Shininess) );   
    // -----------------------
    FragColor = vec4(L, 1.0); // no transparancy
}
