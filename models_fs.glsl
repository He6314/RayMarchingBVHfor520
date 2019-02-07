#version 400

uniform sampler2D texture;

out vec4 fragcolor;           
in vec2 tex_coord;
in vec3 normal;

void main(void)
{
   fragcolor = texture2D(texture, tex_coord);
}




















