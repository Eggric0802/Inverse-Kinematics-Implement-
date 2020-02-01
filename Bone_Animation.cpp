#include "Bone_Animation.h"
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace glm;



Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	root_position = { 2.0f,0.5f,2.0f };
	//First_position = { 2.0f,3.5f,2.0f };
	//Second_position = { 2.0f,7.0f,2.0f };
	//Third_position = { 2.0f,9.5f,2.0f };


	scale_vector =
	{
		{1.0f,1.0f,1.0f},
		{0.5f,4.0f,0.5f},
		{0.5f,3.0f,0.5f},
		{0.5f,2.0f,0.5f}
	};

	rotation_degree_vector = 
	{
		{0.0f,0.0f,0.0f},
		{0.0f,30.0f,0.0f},
		{0.0f,30.0f,0.0f},
		{0.0f,30.0f,0.0f}
	};

	colors = 
	{
		{0.7f,0.0f,0.0f,1.0f},
		{0.7f,0.7f,0.0f,1.0f},
		{0.7f,0.0f,0.7f,1.0f},
		{0.0f,0.7f,0.7f,1.0f}
	};

	bones = new skeleton*[4];
	nodes = new Node*[4];
	for (int i = 0; i < 4; i++)
	{
		bones[i] = new skeleton();
		nodes[i] = new Node(scale_vector[i]);
	}
}

void Bone_Animation::update(float delta_time)
{
	boneMat.clear();
	pivot.clear();
	end.clear();

	for (int i = 0; i < 4; i++) {
		bones[i]->theta_X = rotation_degree_vector[i][2];
		bones[i]->theta_Y = rotation_degree_vector[i][0];
		bones[i]->theta_Z = rotation_degree_vector[i][1];

		bones[i]->Trans = translate(mat4(1.0f), vec3(0, scale_vector[i][1] / 2.0f, 0.0f));

		if (i == 0) bones[i]->bones_center = translate(mat4(1.0f), { root_position[0], (root_position[1] - scale_vector[i][1] / 2.0f),root_position[2] });

		//if (i == 0) bones[i]->bones_center = translate(mat4(1.0f), root_position);
		//else bones[i]->bones_center = translate(mat4(1.0f), vec3(0, scale_vector[i - 1][1] / 2.0f, 0.0f));

		else bones[i]->bones_center = translate(mat4(1.0f), vec3(0.0f, scale_vector[i-1][1] / 2.0f, 0.0f));

		if (i == 0) bones[i]->RAngle = mat4(1.0f);
		else {
			double x = radians((float)bones[i]->theta_X);
			double y = radians((float)bones[i]->theta_Y);
			double z = radians((float)bones[i]->theta_Z);

			glm::mat4 trans_X = glm::eulerAngleX(x);
			glm::mat4 trans_Y = glm::eulerAngleY(y);
			glm::mat4 trans_Z = glm::eulerAngleZ(z);

			bones[i]->RAngle = trans_X * trans_Z * trans_Y;
		}
	}


	for (int i = 0; i < 4; i++) {
		bones[i]->n = nodes[i];
		if (i < 3) nodes[i]->next = bones[i + 1];
		else nodes[i]->next = nullptr;
	}
	Traverse(bones[0], mat4(1));

	for (int i = 0; i < 4; i++) {
		end.push_back(pivot[i] * vec4(0, scale_vector[i][1] / 2.0f, 0.0f, 1.0f));
	}
	efr = end[3];
	vec3 distance = goal - efr;
	double dist = (dot(distance, distance));

	if (Run)
	{
		//vec3 distance = goal - efr;

		//mat3 jacoMat; // Jacobian Matrix
		//for (int i = 0; i < 3; i++)
		//{
		//	if(dot(distance, distance) > 1e-6)
		//	{
		//		jacoMat[i] = vec3(cross(normalize(cross(efr - end[i], goal - end[i])), efr - end[i]));
		//	}
		//}
		//mat3 jacoTMat = transpose(jacoMat); // Transpose of J

		mat3 jacoMat; // Jacobian Matrix
		for (int i = 0; i < 3; i++)
		{
				vec3 Axis = normalize(cross(efr - end[i], goal - end[i]));
				jacoMat[i] = vec3(cross(Axis, efr - end[i]));
		}
		mat3 jacoTMat = transpose(jacoMat); // Transpose of J

		float alpha = ((dot(jacoTMat*distance, (jacoTMat*distance))) / (dot(jacoMat*jacoTMat*distance, jacoMat*jacoTMat*distance))); //step size
		vec3 dtheta = alpha * jacoTMat*distance; //Update 9 DOF bone values using the transpose of Jacobian Matrix and step size 

		for (int i = 0; i < 3; i++)
		{
			if (dist > 1e-6)
			{
				quat q = angleAxis(dtheta[i], normalize(cross(efr - end[i], goal - end[i])));
				vec3 Agl = eulerAngles(q);
				rotation_degree_vector[i + 1] = rotation_degree_vector[i + 1] + vec3(Agl.y, Agl.z, Agl.x);
			}
		}

	}
}

void Bone_Animation::reset()
{
	rotation_degree_vector = { vec3(0),
		{ 0.0f,30.0f,0.0f },
		{ 0.0f,30.0f,0.0f },
		{ 0.0f,30.0f,0.0f } };
}

void Bone_Animation::Traverse(skeleton* head, mat4 mat) {

	if (head == nullptr) return;

	mat = mat * head->bones_center;
	mat = mat * head->RAngle;
	mat = mat * head->Trans;

	pivot.push_back(mat);
	Node* node = head->n;
	Stack.push(mat);
	mat = mat * (*(node->obj));
	boneMat.push_back(mat);
	mat = Stack.top();
	Stack.pop();
	Traverse(node->next, mat);
}

