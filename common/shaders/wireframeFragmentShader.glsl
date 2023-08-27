#version 330 core

uniform vec3 WIRE_COL;
uniform vec3 FILL_COL;
in vec3 dist;
out vec4 FragColor;
void main()
{
	// Wireframe rendering is better like this:
	vec3 dist_vec = dist;

	// Compute the shortest distance to the edge
	float d = min(dist_vec[0], min(dist_vec[1], dist_vec[2]));

	// Compute line intensity and then fragment color
	float I = exp2(-2.0 * d * d);
	float If = exp2(-3.0 * d * d);

	FragColor.rgb = I * WIRE_COL + If * FILL_COL;
	FragColor.a = I;
}
