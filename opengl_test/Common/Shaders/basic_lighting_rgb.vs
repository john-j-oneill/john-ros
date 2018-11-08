#version 330    
    
layout (location = 0) in vec3 Position;     
layout (location = 1) in vec3 Normal;     
layout (location = 2) in vec3 v3ColorIn;    
    
uniform mat4 gWVP;    
uniform mat4 gWorld;    
    
out vec4 v4Color;     
out vec3 Normal0;     
out vec3 WorldPos0;     
    
void main()     
{     
    gl_Position = gWVP * vec4(Position, 1.0);     
    v4Color = vec4(v3ColorIn, 1.0);
    Normal0     = (gWorld * vec4(Normal, 0.0)).xyz;     
    WorldPos0     = (gWorld * vec4(Position, 1.0)).xyz;     
}
