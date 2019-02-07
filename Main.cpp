#include <windows.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include <iostream>
#include <algorithm>

#include "InitShader.h"
#include "imgui_impl_glut.h"
#include "VideoMux.h"
#include "DebugCallback.h"
#include "Cube.h"
#include "time.h"

#include "BVHhqIO.h"

//names of the shader files to load
static const std::string vertex_shader("raycast_vs.glsl");
static const std::string fragment_shader("raycast_fs.glsl");
GLuint shader_program = -1;

//Full screen quad in a VAO
GLuint cube_vao = -1;

//Texture which we will render into
GLuint fbo_texture = -1;
GLuint fbo_texture_width = 1920;
GLuint fbo_texture_height = 1080;

//The frame buffer object provides render-to-texture functionality 
GLuint fbo;

float time_sec = 0.0f;
float angle = 0.0f;
float angleV = 0.0f;
float angleP = 0.0f;
bool recording = false;

void display_inside_out();
void display_outside_in();

void display_models();

int frameShift = 0;
float speed = 1.f;

float jointSize = 1.f;
float boneSize = .5f;

bool psFlag = false;
bool cylFlag = true;
bool sphFlag = true;

bool vel2col = false;
bool vel2str = false;
bool trackFlag = false;

float skeletonScale = 50.f;//20.f
glm::vec3 skeletonShift = glm::vec3(0.f, 0.f, -0.3f);

int frameNumber = 0;
glm::vec3** joints;
glm::vec3** bones;
int* parentIDs;
int* nodeIden;
int hnfPos[4];

glm::vec3** veloVec;
float** boneLength;
float** jointVelocity;

ReadingBVH obj1;
static unsigned int vao;
double* frame;
#define BUFFER_OFFSET(i) ((char *)NULL + (i)) 

glm::mat3 rotateX(float angle)
{
	float radian = angle*3.1415927f / 180.f;
	glm::mat3 rotateMatrix = glm::mat3(0.f);

	rotateMatrix[0][0] = 1.f;
	rotateMatrix[1][1] = cos(radian);
	rotateMatrix[1][2] = -1 * sin(radian);
	rotateMatrix[2][1] = sin(radian);
	rotateMatrix[2][2] = cos(radian);

	return rotateMatrix;
}
glm::mat3 rotateY(float angle)
{
	float radian = angle*3.1415927f / 180.f;
	glm::mat3 rotateMatrix = glm::mat3(0.f);

	rotateMatrix[1][1] = 1.f;
	rotateMatrix[0][0] = cos(radian);
	rotateMatrix[0][2] = sin(radian);
	rotateMatrix[2][0] = -1 * sin(radian);
	rotateMatrix[2][2] = cos(radian);

	return rotateMatrix;
}
glm::mat3 rotateZ(float angle)
{
	float radian = angle*3.1415927f / 180.f;
	glm::mat3 rotateMatrix = glm::mat3(0.f);

	rotateMatrix[2][2] = 1.f;
	rotateMatrix[0][0] = cos(radian);
	rotateMatrix[0][1] = -1 * sin(radian);
	rotateMatrix[1][0] = sin(radian);
	rotateMatrix[1][1] = cos(radian);

	return rotateMatrix;
}

glm::vec3 distance2rotation(glm::vec3 dist, bool axisTran = true)
{
	glm::vec3 distance = dist;
	if (axisTran)
		distance = glm::vec3(dist.x, dist.z, dist.y);
		
	float h = glm::length(distance);
	glm::vec3 rotation(0.f, 0.f, 0.f);

	if (distance.x == 0) {
		if (distance.y == 0) {
			if (distance.z >= 0) {// Z-axis
				rotation = glm::vec3(0.f, 0.f, 0.f);
			}
			else {
				rotation = glm::vec3(0.f, 180.f, 0.f);
			}
		}
		else {
			if (distance.z == 0) { // Y-axis
				if (distance.y >= 0) {
					rotation = glm::vec3(0.0f, -90.f, -90.f);
				}
				else {
					rotation = glm::vec3(0.0f, -90.f, 90.f);
				}
			}
			else {
				double theta1 = acos(distance.z / h) / 3.1415926535897932384626 * 180 * -1;
				double theta2 = acos(distance.x / glm::length(glm::vec2(distance.x, distance.y))) / 3.1415926535897932384626 * 180 * -1 * (abs(distance.y) / distance.y);
				rotation = glm::vec3(0, theta1, theta2);
			}
		}
	}

	else {
		if (distance.y == 0) {
			if (distance.z == 0) {
				if (distance.x >= 0) {
					rotation = glm::vec3(0.0f, -90.f, 0.f);
				}
				else {
					rotation = glm::vec3(0.0f, 90.f, 0.f);
				}
			}
			else {
				double theta = acos(distance.z / glm::length(glm::vec2(distance.x, distance.z))) / 3.1415926535897932384626 * 180 * -1;
				rotation = glm::vec3(0, theta, 0.f);
			}
		}
		else {
			double theta1 = acos(distance.z / h) / 3.1415926535897932384626 * 180 * -1;
			double theta2 = acos(distance.x / glm::length(glm::vec2(distance.x, distance.y))) / 3.1415926535897932384626 * 180 * -1 * (abs(distance.y) / distance.y);
			rotation = glm::vec3(0, theta1, theta2);
		}
	}
	return rotation;
}

void serial2attribs(int frameNb, double** serial, BVHSkeleton skeleton)
{
	clock_t very_begin, frame_begin,skeleton_begin,node_begin;
	very_begin = clock();

	joints = new glm::vec3*[frameNb];
	bones = new glm::vec3*[frameNb];
	veloVec = new glm::vec3*[frameNb];
	boneLength = new float*[frameNb];
	jointVelocity = new float*[frameNb];
	int skSize = skeleton.Bones().size();

	for (int n = 0; n < frameNb; n++)
	{
		//if(n==1)
		//	cout<<"time per frame: " << clock()-frame_begin<<endl;
		frame_begin = clock();

		joints[n] = new glm::vec3[skSize];
		bones[n] = new glm::vec3[skSize];
		veloVec[n] = new glm::vec3[skSize];

		parentIDs = new int[skSize];
		nodeIden = new int[skSize];
		boneLength[n] = new float[skSize];
		jointVelocity[n] = new float[skSize];

		glm::vec3 positionS[50];
		glm::vec3 directionS[50];
		float length[50];

		vector<int> levelS;

		glm::vec3 curPos = glm::vec3(serial[n][0], serial[n][1] - 25.f, serial[n][2]);
		glm::vec3 curDir;// = glm::vec3(serial[n][4], serial[n][3], serial[n][5]);
		positionS[0] = curPos;
		directionS[0] = curDir;
		parentIDs[0] = 0;
		length[0] = 0;

		vector<glm::vec3> dirS;

		levelS.push_back(0);

		//dirS.push_back(glm::vec3(serial[n][4], serial[n][5], serial[n][3]));
		dirS.push_back(glm::vec3(serial[n][5], serial[n][4], serial[n][3]));

		unsigned int nrEndset = 0;
		for (int i = 1; i < skSize; i++)
		{
			/*
			//int offset = 3 + (i - nrEndset) * 3;
			if (n == 1)
				if (i > 0)
				{
					cout << "time per node: " << clock() - skeleton_begin << endl;
					if (clock() - skeleton_begin > 10)
						cout << "罪魁祸首：" << skeleton.Bones()[i-1].Name() << endl;
					cout << "=============================================" << endl;
				}
			skeleton_begin = clock();
			*/
			for (int j = 0; j < skeleton.Bones()[i - 1].Depth() - skeleton.Bones()[i].Depth() + 1; j++)
			{
				levelS.pop_back();
				dirS.pop_back();
			}

			glm::vec3 transOffset = glm::vec3(skeleton.Bones()[i].translOffset[0], skeleton.Bones()[i].translOffset[1], skeleton.Bones()[i].translOffset[2]);

			if (skeleton.Bones()[i].Name().find("End") == -1)
			{
				 glm::mat4 rotation = glm::mat4();

				for (vector<glm::vec3>::reverse_iterator it = dirS.rbegin(); it != dirS.rend(); it++)
				{/*
					if (n == 1)
					if (it!=dirS.rbegin())
						cout << "time per angle: " << clock() - node_begin << endl;
					node_begin = clock();
					*/
					curDir = - (*it) / 180.f * 3.1415927f;
	   /*y x z*/	//rotation = rotation * glm::rotate(curDir[1], glm::vec3(0.0f, 1.0f, 0.0f)) * glm::rotate(curDir[0], glm::vec3(1.0f, 0.0f, 0.0f)) * glm::rotate(curDir[2], glm::vec3(0.0f, 0.0f, 1.0f));
	   /*x y z*/	rotation = rotation * glm::rotate(curDir[0], glm::vec3(1.0f, 0.0f, 0.0f)) * glm::rotate(curDir[1], glm::vec3(0.0f, 1.0f, 0.0f)) * glm::rotate(curDir[2], glm::vec3(0.0f, 0.0f, 1.0f));
					
				}

				
				glm::vec4 translate4 = glm::vec4(transOffset, 1.0f) * rotation;

				//transOffset;//
				glm::vec3 translate = glm::vec3(translate4.x / translate4.w, translate4.y / translate4.w, translate4.z / translate4.w);
				curPos = positionS[levelS.back()] + translate;

				int offset = 3 + (i - nrEndset) * 3;
				//dirS.push_back(glm::vec3(serial[n][offset + 1], serial[n][offset + 2], serial[n][offset + 0]));
				dirS.push_back(glm::vec3(serial[n][offset + 2], serial[n][offset + 1], serial[n][offset + 0]));
				//dirS.push_back(glm::vec3(0.f));
			}
			else
			{
				curDir = glm::vec3(0.f);
				curPos = positionS[levelS.back()] + transOffset;
				nrEndset++;
				dirS.push_back(glm::vec3(0.f));
			}

			positionS[i] = curPos;
			directionS[i] = distance2rotation(positionS[levelS.back()] - positionS[i]);
			length[i] = glm::length(positionS[levelS.back()] - positionS[i]);

			parentIDs[i] = levelS.back();
			string boneName = skeleton.Bones()[i].Name();
			if (boneName.find("Head") != -1)
				nodeIden[i] = 1;
			else if (boneName.find("Wrist") != -1){
				nodeIden[i] = 2;
				if (boneName.find("Left") != -1)
					hnfPos[0] = i;
				else if (boneName.find("Right") != -1)
					hnfPos[1] = i;
			}
			else if (boneName.find("Ankle") != -1) {
				nodeIden[i] = 3;
				if (boneName.find("Left") != -1)
					hnfPos[2] = i;
				else if (boneName.find("Right") != -1)
					hnfPos[3] = i;
			}
			else if (boneName.find("Elbow") != -1)
				nodeIden[i] = 4;
			else if (boneName.find("Knee") != -1)
				nodeIden[i] = 5;
			else if (boneName.find("End") != -1)
				nodeIden[i] = 9;
			else
				nodeIden[i] = 0;

			levelS.push_back(i);
		}

		clock_t hehe = clock();
		for (int i = 0; i < skSize; i++)
		{
			//if(obj1.Skeleton.Channels)
			//joints[n][i] = glm::vec3(positionS[i].x, positionS[i].z, positionS[i].y);
			joints[n][i] = glm::vec3(positionS[i].x, positionS[i].z, positionS[i].y);
			bones[n][i] = directionS[i];
			boneLength[n][i] = length[i];
			veloVec[n][i] = glm::vec3(0.0f);
		}
	}

	for (int n = 0; n < frameNb; n++)
		for (int i = 0; i < skSize; i++){{
				if (n == 0) {
					jointVelocity[n][i] = glm::length(joints[n + 1][i] - joints[n][i]);
					veloVec[n][i] = distance2rotation(joints[n][i] - joints[n + 1][i], false);
				}
				else {
					jointVelocity[n][i] = glm::length(joints[n][i] - joints[n - 1][i]);
					veloVec[n][i] = distance2rotation(joints[n - 1][i] - joints[n][i], false);
				}
			}
		}
}

float* calculBoundaryBox(glm::vec3* frame, int nrJoints)
{
	static float boundary[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float xMax = -1000.f;
	float yMax = -1000.f;
	float zMax = -1000.f;
	float xMin = 1000.f;
	float yMin = 1000.f;
	float zMin = 1000.f;
	
	for (int i = 0; i < nrJoints; i++)
	{
		if (frame[i].x > xMax) xMax = frame[i].x;
		if (frame[i].x < xMin) xMin = frame[i].x;
		if (frame[i].y > yMax) yMax = frame[i].y;
		if (frame[i].y < yMin) yMin = frame[i].y;
		if (frame[i].z > zMax) zMax = frame[i].z;
		if (frame[i].z < zMin) zMin = frame[i].z;
	}

	boundary[0] = xMax + 5.f;
	boundary[1] = yMax + 5.f;
	boundary[2] = zMax + 5.f;
	boundary[3] = xMin - 5.f;
	boundary[4] = yMin - 5.f;
	boundary[5] = zMin - 5.f;

	return boundary;
}

float* calculPosRing(glm::vec3* frame, int nrJoints)
{
	static float groundPos[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	float zMin = 1000.f;
	float zSecMin = 1000.f;
	for (int i = 0; i < nrJoints; i++)
	{
		if (frame[i].z < zMin)
		{
			zSecMin = zMin;
			groundPos[2] = groundPos[0];
			groundPos[3] = groundPos[1];
			zMin = frame[i].z;
			groundPos[0] = frame[i].x;
			groundPos[1] = frame[i].y;

		}
	}
	return groundPos;
}

float* calculBoundary(glm::vec3* frame, int nrJoints)
{
	static float boundary[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	float xMax = -1000.f;
	float yMax = -1000.f;
	float zMax = -1000.f;
	float xMin = 1000.f;
	float yMin = 1000.f;
	float zMin = 1000.f;

	for (int i = 0; i < nrJoints; i++)
	{
		if (frame[i].x > xMax) xMax = frame[i].x;
		if (frame[i].x < xMin) xMin = frame[i].x;
		if (frame[i].y > yMax) yMax = frame[i].y;
		if (frame[i].y < yMin) yMin = frame[i].y;
		if (frame[i].z > zMax) zMax = frame[i].z;
		if (frame[i].z < zMin) zMin = frame[i].z;
	}

	boundary[0] = (xMax + xMin) / 2.f;
	boundary[1] = (yMax + yMin) / 2.f;
	boundary[2] = (zMax + zMin) / 2.f;

	boundary[3] = 5.f + sqrt((xMax - boundary[0]) *(xMax - boundary[0]) + (yMax - boundary[1]) *(yMax - boundary[1]) + (zMax - boundary[2]) *(zMax - boundary[2]));

	return boundary;
}

void draw_gui()
{
   ImGui_ImplGlut_NewFrame();

   ImGui::SetWindowPos(ImVec2(0, 0));
   ImGui::SetWindowSize(ImVec2(600, 400));
   
   const int filename_len = 64;
   static char video_filename[filename_len] = "CG5520finaldemo.mp4";

   ImGui::InputText("Video filename", video_filename, filename_len);
   ImGui::SameLine();
   if (recording == false)
   {
      if (ImGui::Button("Start Recording"))
      {
         const int w = glutGet(GLUT_WINDOW_WIDTH);
         const int h = glutGet(GLUT_WINDOW_HEIGHT);
         recording = true;
         start_encoding(video_filename, w, h);
      }
   }
   else
   {
      if (ImGui::Button("Stop Recording"))
      {
         recording = false;
         finish_encoding(); 
      }
   }

	  static bool model_display = false;
      static bool outside_in = true;

	  ImGui::Checkbox("Obj model", &model_display);
	  ImGui::SameLine();
	  ImGui::Checkbox("Outside in", &outside_in);
	  ImGui::NextColumn();
	  ImGui::Checkbox("Color", &vel2col);
	  ImGui::SameLine();
	  ImGui::Checkbox("Velocity Strings", &vel2str);
	  ImGui::SameLine();
	  ImGui::Checkbox("Tracks", &trackFlag);

	  if (model_display == false)
	  {

		  if (outside_in == true)
		  {
			  glutDisplayFunc(display_outside_in);
		  }
		  else
		  {
			  glutDisplayFunc(display_inside_out);
		  }
	  }
	  else
	  {
		  glutDisplayFunc(display_models);
	  }

      ImGui::NextColumn();

      static int scene = 0;
      const int scene_loc = 8;
      glUniform1i(scene_loc, scene);

   ImGui::SliderFloat("View angle Yaw", &angle, -3.141592f, +3.141592f);
   ImGui::SliderFloat("View angle Roll", &angleV, -3.141592f, +3.141592f);
   ImGui::SliderFloat("View angle Pitch", &angleP, -3.141592f, +3.141592f);

   ImGui::SliderFloat("Skeleton Scaling", &skeletonScale, 100.f, 20.f);
   ImGui::SliderFloat3("Origin Position", glm::value_ptr(skeletonShift), -0.5f, 0.5f);
   static glm::vec4 slider(0.0f);
   static glm::vec2 sliderCubeSphere(.4f,.45f);

   ImGui::SliderFloat("Speed", &speed, 0.5f, 10.f);
   ImGui::Checkbox("Pause", &psFlag);
   ImGui::SameLine();
   ImGui::Checkbox("Show joints", &sphFlag);
   ImGui::SameLine();
   ImGui::Checkbox("Show bones", &cylFlag);

   ImGui::NextColumn();
   ImGui::SliderFloat("Joint size", &jointSize, 0.01f, 2.f);
   ImGui::SliderFloat("Radius of bones", &boneSize, 0.01f, 2.f);

   ImGui::Render();
 }

 void display_models()
 {
	 glUseProgram(shader_program);

	 const int w = glutGet(GLUT_WINDOW_WIDTH);
	 const int h = glutGet(GLUT_WINDOW_HEIGHT);
	 const float aspect_ratio = float(w) / float(h);

	 glm::mat4 M = glm::scale(glm::vec3(1.0f));
	 glm::mat4 V = glm::lookAt(glm::vec3(0.0f, 4.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f))*glm::rotate(angle, glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(angleV, glm::vec3(0.0f, 1.0f, 0.0f))  * glm::rotate(angleP, glm::vec3(1.0f, 0.0f, 0.0f));
	 glm::mat4 P = glm::perspective(3.141592f / 4.0f, aspect_ratio, 0.1f, 100.0f);
	 glm::mat4 T(1.0f);

	 const int PV_loc = 0;
	 const int pass_loc = 1;
	 const int tex_loc = 2;
	 const int M_loc = 4;
	 const int T_loc = 5;

	 glm::mat4 PV = P*V;
	 glUniformMatrix4fv(PV_loc, 1, false, glm::value_ptr(PV));
	 glUniformMatrix4fv(M_loc, 1, false, glm::value_ptr(M));
	 glUniformMatrix4fv(T_loc, 1, false, glm::value_ptr(T));

	 glUniform1i(pass_loc, 1);

	 glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	 glDrawBuffer(GL_COLOR_ATTACHMENT0);
	 glClear(GL_COLOR_BUFFER_BIT);

	 glCullFace(GL_FRONT);
	 draw_cube_vao(cube_vao);

	 glUniform1i(pass_loc, 2);

	 glBindFramebuffer(GL_FRAMEBUFFER, 0); 
	 glDrawBuffer(GL_BACK); 
	 glActiveTexture(GL_TEXTURE0);
	 glBindTexture(GL_TEXTURE_2D, fbo_texture);
	 glUniform1i(tex_loc, 0);

	 if (sphFlag)
		 glUniform1i(47, 1);
	 else
		 glUniform1i(47, 0);

	 if (cylFlag)
		 glUniform1i(48, 1);
	 else
		 glUniform1i(48, 0);

	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	 glCullFace(GL_BACK);
	 draw_cube_vao(cube_vao);

	 draw_gui();

	 if (recording == true)
	 {
		 glFinish();

		 glReadBuffer(GL_BACK);
		 read_frame_to_encode(&rgb, &pixels, w, h);
		 encode_frame(rgb);
	 }

	 glutSwapBuffers();
 }

// 2 glut display callback functions.
void display_outside_in()
{
   glUseProgram(shader_program);

   const int w = glutGet(GLUT_WINDOW_WIDTH);
   const int h = glutGet(GLUT_WINDOW_HEIGHT);
   const float aspect_ratio = float(w) / float(h);

   glm::mat4 M = glm::scale(glm::vec3(1.0f));
   glm::mat4 V = glm::lookAt(glm::vec3(0.0f, 4.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f))*glm::rotate(angle, glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(angleV, glm::vec3(0.0f, 1.0f, 0.0f))  * glm::rotate(angleP, glm::vec3(1.0f, 0.0f, 0.0f));
   glm::mat4 P = glm::perspective(3.141592f / 4.0f, aspect_ratio, 0.1f, 100.0f);
   glm::mat4 T(1.0f);

   const int PV_loc = 0;
   const int pass_loc = 1;
   const int tex_loc = 2;
   const int M_loc = 4;
   const int T_loc = 5;

   glm::mat4 PV = P*V;
   glUniformMatrix4fv(PV_loc, 1, false, glm::value_ptr(PV));
   glUniformMatrix4fv(M_loc, 1, false, glm::value_ptr(M));
   glUniformMatrix4fv(T_loc, 1, false, glm::value_ptr(T));
   glUniform1i(9, 0);
   ///////////////////////////////////////////////////
   //pass 1
   ///////////////////////////////////////////////////
   glUniform1i(pass_loc, 1);

   glBindFramebuffer(GL_FRAMEBUFFER, fbo); 
   glDrawBuffer(GL_COLOR_ATTACHMENT0); 

   glClear(GL_COLOR_BUFFER_BIT);

   glCullFace(GL_FRONT);
   draw_cube_vao(cube_vao);

   ///////////////////////////////////////////////////
   // pass 2
   ///////////////////////////////////////////////////
   glUniform1i(pass_loc, 2);

   glBindFramebuffer(GL_FRAMEBUFFER, 0);
   glDrawBuffer(GL_BACK);

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, fbo_texture);
   glUniform1i(tex_loc, 0);

   if (sphFlag)
	   glUniform1i(47, 1);
   else
	   glUniform1i(47, 0);

   if (cylFlag)
	   glUniform1i(48, 1);
   else
	   glUniform1i(48, 0);

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glCullFace(GL_BACK); 
   draw_cube_vao(cube_vao); 
         
   draw_gui();

   if (recording == true)
   {
      glFinish();

      glReadBuffer(GL_BACK);
      read_frame_to_encode(&rgb, &pixels, w, h);
      encode_frame(rgb);
   }

   glutSwapBuffers();
}

void display_inside_out()
{
   glUseProgram(shader_program);

   const int w = glutGet(GLUT_WINDOW_WIDTH);
   const int h = glutGet(GLUT_WINDOW_HEIGHT);
   const float aspect_ratio = float(w) / float(h);

   glm::mat4 M1 = glm::scale(glm::vec3(1.0f));
   glm::mat4 M2 = glm::scale(glm::vec3(10.0f));
   glm::mat4 V = glm::lookAt(glm::vec3(0.0f, 4.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(angle, glm::vec3(0.0f, 0.0f, 1.0f));
   V[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
   glm::mat4 P = glm::perspective(3.141592f / 4.0f, aspect_ratio, 0.1f, 100.0f);
   glm::mat4 T = glm::translate(glm::vec3(0.0f, 4.0f, 2.0f));

   const int PV_loc = 0;
   const int pass_loc = 1;
   const int tex_loc = 2;
   const int M_loc = 4;
   const int T_loc = 5;

   glUniformMatrix4fv(PV_loc, 1, false, glm::value_ptr(P*V));
   glUniformMatrix4fv(T_loc, 1, false, glm::value_ptr(T));
   glUniform1i(9, 1);

   ///////////////////////////////////////////////////
   //pass 1
   ///////////////////////////////////////////////////
   
   glUniformMatrix4fv(M_loc, 1, false, glm::value_ptr(M2));
   glUniform1i(pass_loc, 1);

   glBindFramebuffer(GL_FRAMEBUFFER, fbo);
   glDrawBuffer(GL_COLOR_ATTACHMENT0);

   glClear(GL_COLOR_BUFFER_BIT);
   glCullFace(GL_FRONT); 

   draw_cube_vao(cube_vao); 

   //////////////////////////////////////////////////////////
   //pass 2
   //////////////////////////////////////////////////////////
   glUniformMatrix4fv(M_loc, 1, false, glm::value_ptr(M1));
   glUniform1i(pass_loc, 2);

   glBindFramebuffer(GL_FRAMEBUFFER, 0); 
   glDrawBuffer(GL_BACK); 

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, fbo_texture);
   glUniform1i(tex_loc, 0); 
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   draw_cube_vao(cube_vao);

   glCullFace(GL_BACK);

   draw_gui();

   if (recording == true)
   {
      glFinish();

      glReadBuffer(GL_BACK);
      read_frame_to_encode(&rgb, &pixels, w, h);
      encode_frame(rgb);
   }
   glutSwapBuffers();
}


// glut idle callback.
//This function gets called between frames
void idle()
{
	glutPostRedisplay();

	const int time_loc = 6;
	static int time_ps;

	if (!psFlag)
	{
		const int time_ms = glutGet(GLUT_ELAPSED_TIME);
		time_sec = 0.001f*time_ms;
		time_ps = time_ms;
	}


	int frameCount =  frameShift + (time_ps % (int(obj1.TimePerFrame() * 1000 * speed)*obj1.NumOfFrames())) / int(obj1.TimePerFrame() * 1000 * speed);
	if (frameCount < 0)
		frameCount = frameCount + obj1.NumOfFrames();
	float* boundarySphere = calculBoundary(joints[frameCount], obj1.Skeleton.NrBones());
	float* ringPos = calculPosRing(joints[frameCount], obj1.Skeleton.NrBones());
	//MARK
	const int pos_loc = 50;
	const int dir_loc = 250;
	const int bLen_loc = 450;
	const int vel_loc = 550;
	const int velvec_loc = 700;

	glUniform3fv(pos_loc, 50, value_ptr(joints[frameCount][0])); //exception
	glUniform3fv(dir_loc, 50, value_ptr(bones[frameCount][0]));
	glUniform1fv(bLen_loc, 50, boneLength[frameCount]);
	glUniform1fv(vel_loc, 50, jointVelocity[frameCount]);
	glUniform3fv(velvec_loc, 50, value_ptr(veloVec[frameCount][0]));

	glUniform1f(34, jointSize);
	glUniform1f(35, boneSize);

	const int box_loc = 650;
	glUniform1fv(box_loc, 4, boundarySphere);
	const int ring_loc = 654;
	glUniform1fv(ring_loc, 4, ringPos);

	const int scale_loc = 39;
	glUniform1f(scale_loc, skeletonScale);

	const int ori_loc = 40;
	glUniform3fv(ori_loc, 1, value_ptr(skeletonShift));

	glUniform1iv(900, 50, parentIDs);
	glUniform1iv(950, 50, nodeIden);
	glUniform1iv(1000, 4, hnfPos);

	if (vel2col) glUniform1i(38, 1);
	else glUniform1i(38, 0);

	if(vel2str)	glUniform1i(37, 1);
	else glUniform1i(37, 0);

	if (trackFlag)	glUniform1i(36, 1);
	else glUniform1i(36, 0);

	{
		int past1 = max(frameCount - 1, 0); 
		int past2 = max(frameCount - 3, 0);
		int past3 = max(frameCount - 5, 0);
		int past4 = max(frameCount - 7, 0);
		int past5 = max(frameCount - 9, 0);
		int past6 = max(frameCount - 11, 0);
		glUniform3fv(1050, 50, value_ptr(joints[past1][0]));
		glUniform3fv(1200, 50, value_ptr(joints[past2][0]));
		glUniform3fv(1350, 50, value_ptr(joints[past3][0]));
		glUniform3fv(1500, 50, value_ptr(joints[past4][0]));
		glUniform3fv(1650, 50, value_ptr(joints[past5][0]));
		glUniform3fv(1800, 50, value_ptr(joints[past6][0]));
	}
}

void reload_shader()
{
   GLuint new_shader = InitShader(vertex_shader.c_str(), fragment_shader.c_str());

   if(new_shader == -1)
   {
      glClearColor(1.0f, 0.0f, 1.0f, 0.0f);
   }
   else
   {
      glClearColor(0.15f, 0.15f, 0.15f, 0.0f);

      if(shader_program != -1)
      {
         glDeleteProgram(shader_program);
      }
      shader_program = new_shader;
   }

   const int size_loc = 49;
   glUniform1i(size_loc, obj1.Skeleton.Bones().size());

   serial2attribs(obj1.NumOfFrames(), obj1.MotionData, obj1.Skeleton);

   float minZ = 500.f;
   for (int n = 0; n < obj1.NumOfFrames(); n++)
	   for (int i = 0; i < obj1.Skeleton.Bones().size(); i++)
	   {
		   if (joints[n][i].z < minZ)
			   minZ = joints[n][i].z;
	   }
   const int ground_loc = 46;
   glUniform1f(ground_loc, minZ);
}

void printGlInfo()
{
   std::cout << "Vendor: "       << glGetString(GL_VENDOR)                    << std::endl;
   std::cout << "Renderer: "     << glGetString(GL_RENDERER)                  << std::endl;
   std::cout << "Version: "      << glGetString(GL_VERSION)                   << std::endl;
   std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION)  << std::endl;
}

void initOpenGl()
{
   glewInit();

   RegisterCallback();

   glEnable(GL_DEPTH_TEST);
   glEnable(GL_CULL_FACE);

   reload_shader();

   cube_vao = create_cube_vao();

   glGenTextures(1, &fbo_texture);
   glBindTexture(GL_TEXTURE_2D, fbo_texture);
   glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, fbo_texture_width, fbo_texture_height, 0, GL_RGBA, GL_FLOAT, 0);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glBindTexture(GL_TEXTURE_2D, 0);

   glGenFramebuffers(1, &fbo);
   glBindFramebuffer(GL_FRAMEBUFFER, fbo);
   glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture, 0);

   glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// glut callbacks need to send keyboard and mouse events to imgui
void keyboard(unsigned char key, int x, int y)
{
   ImGui_ImplGlut_KeyCallback(key);
   std::cout << "key : " << key << ", x: " << x << ", y: " << y << std::endl;

   std::cout << "frame per second:" << 1.0f/(obj1.TimePerFrame() * speed) << std::endl;

   switch(key)
   {
      case 'r':
      case 'R':
         reload_shader(); 
	  break;

	  case ' ':
		  psFlag = !psFlag;
      break;

	  case '+': 
		  frameShift++; 
		  if (frameShift == obj1.NumOfFrames())
			  frameShift = 0;
	  break;

	  case '-':
		  frameShift--;
		  if (frameShift == -obj1.NumOfFrames())
			  frameShift = 0;
		  break;
   }
}

void keyboard_up(unsigned char key, int x, int y)
{
   ImGui_ImplGlut_KeyUpCallback(key);
}

void special_up(int key, int x, int y)
{
   ImGui_ImplGlut_SpecialUpCallback(key);
}

void passive(int x, int y)
{
   ImGui_ImplGlut_PassiveMouseMotionCallback(x,y);
}

void special(int key, int x, int y)
{
   ImGui_ImplGlut_SpecialCallback(key);
}

void motion(int x, int y)
{
   ImGui_ImplGlut_MouseMotionCallback(x, y);
}

void mouse(int button, int state, int x, int y)
{
   ImGui_ImplGlut_MouseButtonCallback(button, state);
}

int main (int argc, char **argv)
{
   //Configure initial window state using freeglut

#if _DEBUG
   glutInitContextFlags(GLUT_DEBUG);
#endif
   glutInitContextVersion(4, 3);

   glutInit(&argc, argv); 
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
   glutInitWindowPosition (5, 5);
   glutInitWindowSize (fbo_texture_width, fbo_texture_height);
   int win = glutCreateWindow ("CGT520 Final Project-Qin He");

   printGlInfo();

   //obj1.ReadFromFile(string("D:\\cmu_3ds\\cmuconvert-max-01-09\\05\\05_04.bvh"));
   //obj1.ReadFromFile(string("D:\\cmu_3ds\\cmuconvert-max-86-94\\94\\94_01.bvh"));
   //obj1.ReadFromFile(string("D:\\[www.cgmxw.com]BVH动作文件\\BVH动作文件\\55046437\\0000093b.bvh"));
   //obj1.ReadFromFile(string("D:\\CGT520\\bioviewer-0.0.21\\83c.bvh"));
   //obj1.ReadFromFile(string("D:\\[www.cgmxw.com]BVH动作文件\\BVH动作文件\\59388066\\0000098a.bvh"));
   //obj1.ReadFromFile(string("93b.bvh"));
   obj1.ReadFromFile(string("05_04.bvh"));

   //Register callback functions with glut. 
   glutDisplayFunc(display_models);
   glutKeyboardFunc(keyboard);
   glutSpecialFunc(special);
   glutKeyboardUpFunc(keyboard_up);
   glutSpecialUpFunc(special_up);
   glutMouseFunc(mouse);
   glutMotionFunc(motion);
   glutPassiveMotionFunc(motion);

   glutIdleFunc(idle);

   initOpenGl();
   ImGui_ImplGlut_Init(); 

   glutMainLoop();
   glutDestroyWindow(win);
   return 0;	
}


