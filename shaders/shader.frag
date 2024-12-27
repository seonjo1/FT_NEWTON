#version 450

layout(binding = 1) uniform sampler2D diffuseSampler;
layout(binding = 2) uniform sampler2D specularSampler;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) flat in int outIsCollide;

layout(location = 0) out vec4 outColor;

void main()
{
	if (outIsCollide == 0)
	{
		vec3 texColor = texture(diffuseSampler, fragTexCoord).xyz;
    	outColor = vec4(texColor, 1.0);
	}
	else
	{
		vec3 texColor = texture(specularSampler, fragTexCoord).xyz;
    	outColor = vec4(texColor, 1.0);
	}
}