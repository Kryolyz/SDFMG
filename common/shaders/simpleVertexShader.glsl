#version 330 core


layout(location = 0) in vec4 vertexPosition_modelspace;
layout(location = 1) in vec4 vertexColor;

out vec4 fragmentColor;

uniform mat4 MVP;

void main()
{
	gl_Position = MVP * vertexPosition_modelspace;
	vec4 color = vec4(vertexColor.xyz, 0.5f);
	fragmentColor = color;
}