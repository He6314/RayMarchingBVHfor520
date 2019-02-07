#version 430

layout(location = 0) uniform mat4 PV;
layout(location = 4) uniform mat4 M;
layout(location = 5) uniform mat4 T;

layout(location = 1) uniform int pass;
layout(location = 2) uniform sampler2D backfaces_tex;

layout(location = 6) uniform float time;
layout(location = 7) uniform vec4 slider;
layout(location = 8) uniform int scene = 0;
layout(location = 9) uniform int inside_out = 0;
layout(location = 10) uniform vec2 sizeS; 

layout(location = 34) uniform float clientJointSize = 1.0f;
layout(location = 35) uniform float clientBoneSize = 0.5f;
layout(location = 36) uniform int trackFlag=0;
layout(location = 37) uniform int vel2stringFlag=0;
layout(location = 38) uniform int vel2colorFlag=0;
layout(location = 39) uniform float scale_factor = 80.f;
layout(location = 40) uniform vec3 offset = vec3(0.f,0.f,0.3f);
layout(location = 46) uniform float groundLevel = -1.0f;
layout(location = 47) uniform int jointsFlag = 1;
layout(location = 48) uniform int stickFlag = 1;
layout(location = 49) uniform int node_size = 50;
layout(location = 50) uniform vec3 nd_pos[50];
layout(location = 250) uniform vec3 nd_direction[50];
layout(location = 450) uniform float nd_length[50];
layout(location = 550) uniform float nd_vel[50];
layout(location = 650) uniform float bound[4];
layout(location = 654) uniform float ringPos[4];
layout(location = 700) uniform vec3 nd_veloVec[50];
layout(location = 900) uniform int parentID[50];
layout(location = 950) uniform int nodeIden[50];
layout(location = 1000) uniform int hnfIdx[4];


layout(location = 1050) uniform vec3 past_pos1[50];
layout(location = 1200) uniform vec3 past_pos2[50];
layout(location = 1350) uniform vec3 past_pos3[50];
layout(location = 1500) uniform vec3 past_pos4[50];
layout(location = 1650) uniform vec3 past_pos5[50];
layout(location = 1800) uniform vec3 past_pos6[50];

in vec3 vpos;  
layout(location = 0) out vec4 fragcolor;

vec4 ambient_color = vec4(0.071, 0.188, 0.294, 1.0);

//forward function declarations
vec4 raySky(vec3 rayStart, vec3 rayStop);
vec4 raytracedcolor(vec3 rayStart, vec3 rayStop);
vec4 lighting(vec3 pos, vec3 rayDir, vec4 jointColor);
float distToShape(vec3 pos);
vec4 isJoint(vec3 pos);
vec3 normal(vec3 pos);

void main(void)
{   
	if(pass == 1)
	{
		fragcolor = vec4((vpos), 1.0); //write cube positions to texture
	}
	else if(pass == 2) 
	{
		vec3 rayStart = vpos.xyz;
		vec3 rayStop = texelFetch(backfaces_tex, ivec2(gl_FragCoord.xy), 0).xyz;
		fragcolor = raytracedcolor(rayStart, rayStop);
		if(fragcolor.a==0.0) fragcolor = raySky(rayStart, rayStop);
	}
}

vec4 raySky(vec3 rayStart, vec3 rayStop)
{
	vec4 color = vec4(0.0, 0.0, 0.0, 0.0);

	vec3 rayDir = normalize(rayStop-rayStart);
	const vec3 light = vec3(0.577, 0.577, 0.577); //light direction

	vec4 sky_color = vec4(0.2, 0.35, 0.802, 1.0);

if(inside_out==0)
{
	const vec4 arm1_color = vec4(-0.010, 0.098, 0.140, 0.2);
	vec3 diff_arm1 =  (nd_pos[hnfIdx[0]]/scale_factor-offset) - rayStop;
	vec4 arm1_component = step(0.0f, diff_arm1.z+0.3*sin(diff_arm1.x*2)+0.3*sin(diff_arm1.y*2)) * arm1_color;

	const vec4 arm2_color = vec4(-0.010, 0.098, 0.140, 0.2);
	vec3 diff_arm2 = rayStop - (nd_pos[hnfIdx[1]]/scale_factor-offset);
	vec4 arm2_component = (1.0f-step(0.0f, diff_arm2.z-0.3-0.3*sin(diff_arm2.x*2)-0.3*sin(diff_arm2.y*2))) * arm2_color;

	const vec4 leg1_color = vec4(0.173, 0.145, 0.024, 0.2);
	vec3 diff_leg1 = (nd_pos[hnfIdx[2]]/scale_factor-offset) - rayStop;
	vec4 leg1_component = step(0.0f, diff_leg1.z+0.3*sin(diff_leg1.x*2)+0.3*sin(diff_leg1.y*2)) * leg1_color;

	const vec4 leg2_color = vec4(0.173, 0.145, 0.024, 0.2);
	vec3 diff_leg2 = rayStop - (nd_pos[hnfIdx[3]]/scale_factor-offset);
	vec4 leg2_component = (1.0f-step(0.0f, diff_leg2.z-0.3-0.3*sin(diff_leg2.x*2)-0.3*sin(diff_leg2.y*2))) * leg2_color;

	sky_color = vec4(0.105, 0.137, 0.129, 0.2) + arm1_component + arm2_component + leg1_component + leg2_component;
}
	const vec4 source_color = vec4(0.9, 0.9, 0.1, 1.0);

	const vec4 diffuse_color = vec4(1.0, 1.0, 1.0, 1.0);
	
	vec3 nLight = normalize(light);
	vec3 n = normal(rayDir);
	vec3 v = -rayDir;
	vec3 r = reflect(-light, n);

	return  sky_color  + source_color * max(0.0, -dot(light,v))  ;//+ diffuse_color*rand()/RAND_MAX;//max(0.0, dot(n, light));

	//vec3 pos = rayStart;	
	//return color;
}

// trace rays until they intersect the surface
vec4 raytracedcolor(vec3 rayStart, vec3 rayStop)
{
	vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
	const int MaxSamples = 1000;

	vec3 rayDir = normalize(rayStop-rayStart);
	float travel = distance(rayStop, rayStart);
	float stepSize = travel/MaxSamples;
	vec3 pos = rayStart;
	vec3 step = rayDir*stepSize;
	
	for (int i=0; i < MaxSamples && travel > 0.0; ++i, pos += step, travel -= stepSize)
	{
		float dist = distToShape(pos);

		stepSize = dist;
		step = rayDir*stepSize;
		
		if(dist<=0.001){
			vec4 jointColor = vec4(0.0f);
			if(vel2colorFlag==1)
				 jointColor = isJoint(pos);
			color = lighting(pos, rayDir, jointColor);
			return color;
		}	
	}
	return color;
}

//Compute lighting on the raycast surface using Phong lighting model
float softshadow(vec3 ro, vec3 rd, float mint, float maxt, float k);
vec4 lighting(vec3 pos, vec3 rayDir, vec4 jointColor)
{
	const vec3 light = vec3(0.577, 0.577, 0.577);
	if(jointColor != vec4(0.0f))
	{
		ambient_color = jointColor;
	}
	const vec4 diffuse_color = vec4(0.722, 0.627, 0.545, 1.0);
	const vec4 spec_color = vec4(0.120, 0.120, 0.060, 1.0);

	vec3 n = normal(pos);
	vec3 v = -rayDir;
	vec3 r = reflect(-light, n);

	float shadow = 1.f;

    shadow = softshadow(pos, light, 0.015, 1, 5.);

	return ambient_color + diffuse_color*max(0.0, dot(n, light)) * shadow + spec_color*pow(max(0.0, dot(r, v)), 15.0);	
}

//Abandoned?
vec4 CTlighting(vec3 pos, vec3 rayDir)
{
	const vec3 light = vec3(0.577, 0.577, 0.577);
	const vec4 ambient_color = vec4(1.0);
	const vec4 diffuse_color = vec4(1.0);
	const vec4 spec_color = vec4(1.0);

	vec3 n = normal(pos);
	vec3 v = -rayDir;
	vec3 r = reflect(-light, n);

	float shadow = 1.f;

	if(scene!=0)
		shadow = softshadow(pos, light, 0.015, 1, 4.);

	return ambient_color + diffuse_color*max(0.0, dot(n, light)) * shadow + spec_color*pow(max(0.0, dot(r, v)), 15.0);	
}


//shape function declarations
float sdSphere( vec3 p, vec3 c, float s);

float cylinder(vec3 p, float h, float r);
float trunk(vec3 p, vec3 c, vec3 rotation, float h, float r);
float trunk2(vec3 p, vec3 c1, vec3 c2, float r);
float sdTorus( vec3 p, vec3 c, vec2 t );
float pipe_track(vec3 pos, vec3 offset, float scale_factor, float boneSize, float polyK, int i);

float polysmin( float a, float b, float k )
{
    float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
    return mix( b, a, h ) - k*h*(1.0-h);
}

//distance to the shape we are drawing
float distToShape(vec3 pos)
{
    float obj = 0;
	if(scene == 0)
	{
		const float pi = 3.141592654;
		obj = 100.f;

		vec3 boundCenter = vec3(bound[0],bound[1],bound[2]);	
		//boundCenter = vec3(T*M*vec4(boundCenter, 1.0));
		boundCenter = (M*vec4(boundCenter, 1.0)).xyz;
		boundCenter = boundCenter/scale_factor - offset;
		float boundRadius = bound[3]/scale_factor;

		//if((length(pos-boundCenter)-boundRadius)<0.001)
		{
			for(int i = 0;i<node_size;i++)
			{
				vec3 node = nd_pos[i]/scale_factor - offset;
				vec3 par_node = nd_pos[parentID[i]]/scale_factor - offset;
				vec3 dir = nd_direction[i];
		
				vec3 veloVec = nd_veloVec[i];
				vec3 par_veloVec = nd_veloVec[parentID[i]];

				float speed = nd_vel[i]/scale_factor*3.0f;
				float boneLength = nd_length[i]/scale_factor/2;

				float nodeSize = clientJointSize/scale_factor;
				float boneSize = clientBoneSize/scale_factor;
				if(nodeIden[i]==1)
					nodeSize = nodeSize+0.02;
				
				if(nodeIden[i]!=9)
				{
					if(stickFlag==1 && i>0){
						if(vel2colorFlag==0){
							float polyK = nodeSize/4.0f;
							obj = polysmin(obj, trunk(pos, node, dir, boneLength, boneSize), polyK);
						}
						else
							obj = min(obj, trunk(pos, node, dir, boneLength, boneSize));
						//obj = min(obj, trunk2(pos, node, pre, boneSize));
						if(vel2stringFlag==1)
						for (int k=0;k<6;k++)
						{
							float kf = k;
							vec3 root = node+kf*(par_node - node)/6.0;
							vec3 velDir = veloVec + kf*(par_veloVec - veloVec)/6.0;
							obj = min(obj, trunk(pos, root, velDir, speed, boneSize/1.5f));
						}					
					}	
				
					if(jointsFlag==1){
						if(vel2colorFlag==0)
						obj = polysmin(obj, sdSphere(pos,node,nodeSize),0.05);
						else
						obj = min(obj, sdSphere(pos,node,nodeSize));

						if(trackFlag==1)
							if(nodeIden[i]>=2&&nodeIden[i]<=5){
								obj = polysmin(obj, pipe_track(pos,offset,scale_factor,boneSize,0.005,i),0.005);
							}	
					}
				}
			}
		}

		//obj = min(obj, sdSphere(pos,boundCenter,boundRadius));
		//vec4 n = vec4(0.f,0.f,.f,.f);
		vec2 paraT = vec2(0.1,0.01);
		vec3 ringCenter1 = vec3(ringPos[0],ringPos[1],groundLevel);
		vec3 ringCenter2 = vec3(ringPos[2],ringPos[3],groundLevel);
		ringCenter1 = ringCenter1/scale_factor-offset;
		ringCenter2 = ringCenter2/scale_factor-offset;
		obj = min(obj,sdTorus(pos,  ringCenter1,  paraT));	
		obj = min(obj,sdTorus(pos,  ringCenter2,  paraT));

		obj = min(obj,pos.z-(groundLevel/scale_factor-offset.z));
	}

	return obj;
}

float pipe_track(vec3 pos, vec3 offset, float scale_factor, float boneSize, float polyK, int i)
{
	vec3 node = nd_pos[i]/scale_factor - offset;
	vec3 node1 = past_pos1[i]/scale_factor - offset;
	vec3 node2 = past_pos2[i]/scale_factor - offset;
	vec3 node3 = past_pos3[i]/scale_factor - offset;
	vec3 node4 = past_pos4[i]/scale_factor - offset;
	vec3 node5 = past_pos5[i]/scale_factor - offset;
	vec3 node6 = past_pos6[i]/scale_factor - offset;
	float obj = trunk2(pos, node, node1, boneSize);
	obj = polysmin(obj, trunk2(pos, node1, node2, 0.01),0.005);
	obj = polysmin(obj, trunk2(pos, node2, node3, 0.01),0.005);
	obj = polysmin(obj, trunk2(pos, node3, node4, 0.01),0.005);
	obj = polysmin(obj, trunk2(pos, node4, node5, 0.01),0.005);
	obj = polysmin(obj, trunk2(pos, node5, node6, 0.01),0.005);

	return obj;
}

vec4 isJoint(vec3 pos)
{
		const float pi = 3.141592654;
		for(int i = 0;i<node_size;i++)
		{
			vec3 node = nd_pos[i]/scale_factor - offset;
			vec3 dir = nd_direction[i];
			float boneLength = nd_length[i]/scale_factor/2;
			float nodeSize = clientJointSize/scale_factor;
			float boneSize = clientBoneSize/scale_factor;
			if(nodeIden[i]==1)
					nodeSize = nodeSize+0.02;

			vec3 veloVec = nd_veloVec[i];
			float speed = nd_vel[i]/scale_factor*3.0f;

			if(trunk(pos, node, dir, boneLength, boneSize)<0.001 || sdSphere(pos,node,nodeSize)<0.001 ){
				float v = 0.08*nd_vel[i];
				//return vec4(0.05+v,0.05f+0.75*v,0.05f+0.6*v,1.0f);
				return vec4(0.05+v,0.05f,0.05f,1.0f);
			}

			if(trunk(pos, node, veloVec, speed, boneSize/1.5f)<0.001)	{
				//float v = 0.1*nd_vel[i];
				return vec4(0.75,0.0,0.0f,1.0f);
			}

			if(pipe_track(pos,offset,scale_factor,boneSize,0.005,i)<0.001) {
				return vec4(0.55,0.0,0.0f,1.0f);
			}

			vec2 paraT = vec2(0.1,0.2);
			paraT = vec2(0.1,0.01);
			vec3 ringCenter1 = vec3(ringPos[0],ringPos[1],groundLevel);
			ringCenter1 = ringCenter1/scale_factor-offset;
			vec3 ringCenter2 = vec3(ringPos[2],ringPos[3],groundLevel);
			ringCenter2 = ringCenter2/scale_factor-offset;
			if(sdTorus(pos,  ringCenter1,  paraT)<0.001 || sdTorus(pos,  ringCenter2,  paraT)<0.001)
			{
				return vec4(1.0f,0.788f,0.055f,1.0f);
			}
			// if(sdSphere(pos,node,nodeSize)<0.001)
			// {
				
			// 	return vec3(0.15f,0.1f,0.1f);
			// }
		}
	return vec4(0.0f);
}
//=======================================================================


mat3 rotateX(float angle)
{
	angle *= 3.1415927/180.;
	return mat3(1.,0.,0.,0.,cos(angle),-1*sin(angle),0.,sin(angle),cos(angle));
}
mat3 rotateY(float angle)
{
	angle *= 3.1415927/180.;
	return mat3(cos(angle),0.,sin(angle),0.,1.,0.,-1*sin(angle),0.,cos(angle));
}
mat3 rotateZ(float angle)
{
	angle *= 3.1415927/180.;
	return mat3(cos(angle),-1*sin(angle),0.,sin(angle),cos(angle),0.,0.,0.,1.);
}

////////////////////////////////////////////////////////////////////////////////
// shape function definitions
//////////////////////////////////////////////////////////////////////////////////

float sdTorus( vec3 p, vec3 c, vec2 t )
{
 	vec3 pM = p-c;
  vec2 q = vec2(length(pM.xy)-t.x,pM.z);
  return length(q)-t.y;
}

// float sdTorus82( vec3 p, vec2 t )
// {
//   vec2 q = vec2(length2(p.xz)-t.x,p.y);
//   return length8(q)-t.y;
// }

//draw a cylinder with one point, the direction and the height given
float trunk(vec3 p, vec3 c, vec3 rotation, float h, float r)
{
	vec3 q = p-c;
	mat3 rX = rotateX(rotation.x);
	mat3 rY = rotateY(rotation.y);
	mat3 rZ = rotateZ(rotation.z);
	mat3 t = rZ*rX*rY;
	return cylinder( inverse(t)*q, h, r);
}

//draw a cylinder with two points given
float trunk2(vec3 p, vec3 c1, vec3 c2, float r)
{
	vec3 q = p-c1;

	vec3 seg = c2-c1;
	float h = length(seg);

	vec3 rotation = vec3(0.f,0.f,0.f);

	if (seg.x == 0){
		if(seg.y == 0){
			if (seg.z >= 0){// Z-axis
				rotation = vec3(0.f, 0.f, 0.f);}
			else{
				rotation = vec3(0.f, 180.f, 0.f);}}
		else{
			if(seg.z == 0){ // Y-axis
				if(seg.y>=0){
					rotation = vec3(0.0f,-90.f,-90.f);}
				else{
					rotation = vec3(0.0f,-90.f,90.f);}}
			else{
				double theta1 = acos(seg.z  / h) / 3.1415926535897932384626 * 180 * -1;
				double theta2 = acos(seg.x / length(vec2(seg.x, seg.y))) / 3.1415926535897932384626 * 180 * -1 * (abs(seg.y)/seg.y);
				rotation = vec3(0, theta1, theta2);}}}

	else {
		if(seg.y == 0){
			if(seg.z == 0){
				if(seg.x>=0){
					rotation = vec3(0.0f,-90.f,0.f);}
				else{
					rotation = vec3(0.0f,90.f,0.f);}}
			else{
				double theta = acos( seg.z  /  length(vec2(seg.x, seg.z))) / 3.1415926535897932384626 * 180 * -1;
				rotation = vec3(0, theta, 0.f);}}
		else{
			double theta1 = acos(seg.z  / h ) / 3.1415926535897932384626 * 180 * -1;
			double theta2 = acos(seg.x / length(vec2(seg.x, seg.y))) / 3.1415926535897932384626 * 180 * -1 * (abs(seg.y)/seg.y);
			rotation = vec3(0, theta1, theta2);}}

	// float stepXY = (step(-1e-35,seg.x)-step(1e-35,seg.x))*(step(-1e-35,seg.y)-step(1e-35,seg.y))*(1-step(1e-35,seg.z));
	// float stepXZ = (step(-1e-35,seg.x)-step(1e-35,seg.x))*(step(-1e-35,seg.z)-step(1e-35,seg.z));
	// float stepYZ = (step(-1e-35,seg.y)-step(1e-35,seg.y))*(step(-1e-35,seg.z)-step(1e-35,seg.z));
	// float stepY = (step(-1e-35,seg.y)-step(1e-35,seg.y));
	// float stepElse =step(1, ((step(1e-35,seg.x) + 1 -step(-1e-35,seg.x))*(step(1e-35,seg.z) + 1 -step(-1e-35,seg.z))) + (step(1e-35,seg.y) + 1 -step(-1e-35,seg.y)));

	// float theta1YZ = acos(seg.z  / h ) / 3.1415926535897932384626 * 180 * -1;
	// float theta2YZ = acos(seg.x / length(vec2(seg.x, seg.y))) / 3.1415926535897932384626 * 180 * -1 * (abs(seg.y)/seg.y);
	// float theta1XZ = acos( seg.z  /  length(vec2(seg.x, seg.z))) / 3.1415926535897932384626 * 180 * -1;

	// vec3 theta1v1 = vec3(-90.0f, 90.0f*(-1*abs(seg.x)/seg.x), theta1XZ);
	// vec3 step1v1 = vec3(stepXZ, stepYZ, stepY);
	// vec2 theta1v2 = vec2(180.0f, theta1YZ);
	// vec2 step1v2 = vec2(stepXY, stepElse);

	// vec2 theta2v = vec2(90.0f*(-1*abs(seg.y)/seg.y),theta2YZ);
	// vec2 step2v = vec2(stepXZ,stepElse);

	// rotation.y += dot(theta1v1,step1v1)+dot(theta1v2,step1v2);
	// rotation.z += dot(theta2v,step2v);

	mat3 rX = rotateX(rotation.x);
	mat3 rY = rotateY(rotation.y);
	mat3 rZ = rotateZ(rotation.z);
	mat3 t = rZ*rY*rX;
	return cylinder( inverse(t)*q, h/2, r);
}

float cylinder(vec3 p, float h, float r)
{
    p.z -= h;
    vec2 d = abs(vec2(length(p.xy),p.z)) - vec2(r,h);
	return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}
float Union(float d1, float d2)
{
    if(d1<d2)
	    return d1;
	else
	    return d2;
}
   
float sdSphere( vec3 p, vec3 c, float s)
{
	return length(p-c)-s;
}

//normal vector of the shape we are drawing.
//Estimated as the gradient of the signed distance function.
vec3 normal(vec3 pos)
{
	const float h = 0.001;
	const vec3 Xh = vec3(h, 0.0, 0.0);	
	const vec3 Yh = vec3(0.0, h, 0.0);	
	const vec3 Zh = vec3(0.0, 0.0, h);	

	return normalize(vec3(distToShape(pos+Xh)-distToShape(pos-Xh), distToShape(pos+Yh)-distToShape(pos-Yh), distToShape(pos+Zh)-distToShape(pos-Zh)));
}

float softshadow(vec3 ro, vec3 rd, float mint, float maxt, float k )
{
	float res = 1.0;
    float t = mint;
    for( int i=0; i<16; i++ )
    {
		float h = distToShape( ro + rd*t );
        res = min( res, k*h/t );
        t += clamp( h, 0.02, 0.10 );
        if( h<0.001 || t>maxt ) break;
    }
    return clamp( res, 0.0, 1.0 );
}
