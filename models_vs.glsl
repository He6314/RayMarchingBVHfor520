#version 400            
uniform mat4 PVM;
uniform float time;

in vec3 pos_attrib;
in vec2 tex_coord_attrib;
in vec3 normal_attrib;

out vec2 tex_coord;  
out vec3 normal; 

void main(void)
{
    float movX = 0.f;
	float movY = 0.2 *amp*sin(10.f*pos_attrib.x+spd*time+0.5);;
	float movZ = amp*sin(10.f*pos_attrib.x+spd*time);
	//+ vec3(movX,movY,movZ) + swl*0.015*normal_attrib

   gl_Position = PVM*vec4(pos_attrib, 1.0);
   tex_coord = tex_coord_attrib + vec2(3*movY,2*movY);
   normal = normal_attrib;
}