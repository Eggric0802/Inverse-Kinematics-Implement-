#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stack>


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtc/type_ptr.hpp>	

using namespace std;
using namespace glm;

class Node;
class skeleton;

class Node { //linked list
public:
	vec3 scale_vector;
	skeleton* next;
	mat4* obj;
	vec4 color;
	vec3 basePoint;
	vec3 topPoint;

	Node(vec3 & s) :scale_vector(s) {
		obj = new mat4();
		vec4 v = vec4(scale_vector, 1.0);
		*obj = diagonal4x4(v);
	};
	~Node() { delete obj; };
};

class skeleton {
public:
	skeleton() {};
	~skeleton() {};

	mat4 Trans = mat4();
	mat4 bones_center = mat4();
	mat4 RAngle = mat4();

	int theta_X = 0; int theta_Y = 0; int theta_Z = 0;
	Node* n;
};

class Bone_Animation
{
public:
	Bone_Animation();
	~Bone_Animation();

	void init();
	void update(float delta_time);
	void reset();
	void Traverse(skeleton* head, mat4 mat);

	skeleton** bones;
	Node** nodes;

public:

	// Here the head of each vector is the root bone
	std::vector<glm::vec3> scale_vector;
	std::vector<glm::vec3> rotation_degree_vector;
	std::vector<glm::vec4> colors;

	glm::vec3 root_position;
	//glm::vec3 First_position;
	//glm::vec3 Second_position;
	//glm::vec3 Third_position;

	bool Run = false;

	stack<mat4> Stack;
	vector<mat4> boneMat;
	vector<mat4> pivot;
	vector<vec3> end;
	vec3 goal = vec3(3, 8, 3);
	vec3 efr;//effector
};

