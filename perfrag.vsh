#version 330

layout (location = 0) in vec3 VertexPosition;
layout (location = 1) in vec3 VertexNormal;
in vec3 KdIn;
in vec3 KaIn;
in vec3 KsIn;
in float ShininessIn;

out vec3 Kd;
out vec3 Ks;
out vec3 Ka;
out float Shininess;

out vec3 Position;
out vec3 Normal;

uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;
uniform mat4 MVP;

void main()
{
    // ZK: Will need to make modifications here.
    Normal = normalize( NormalMatrix * VertexNormal);
    Position = vec3( ModelViewMatrix * vec4(VertexPosition,1.0) );
    gl_Position = MVP * vec4(VertexPosition,1.0);
    Kd = KdIn;
    Ks = KsIn;
    Ka = KaIn;
    Shininess = ShininessIn;
}
