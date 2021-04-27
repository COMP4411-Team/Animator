#include "ModelHelper.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <FL/gl.h>
#include <gl/GLU.h>

#include "bitmap.h"
#include "bitmap.h"
#include "camera.h"
#include "IKSolver.h"
#include "LSystem.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "modelerglobals.h"
#include "modelerui.h"
#include "modelerview.h"

using namespace Assimp;
using namespace std;

extern ModelHelper helper;

void ModelHelper::loadModel(const string& model, const string& bone)
{
	const auto* new_scene = importer.ReadFile(model, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
	                                    aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
	if (new_scene == nullptr)
		throw runtime_error("failed to parse model file");
	
	delete scene;
	scene = new_scene;
	preprocess();
	if (!bone.empty())
	{
		for (auto& mesh : meshes)
			parseBoneInfo(mesh, bone);
	}
	calBoneTransformation(aiQuaternion(), scene->mRootNode);
}

void ModelHelper::preprocess()
{
	meshes.resize(scene->mNumMeshes);
	for (int i = 0; i < scene->mNumMeshes; ++i)
	{
		auto* mesh = scene->mMeshes[i];
		meshes[i].data = mesh;
		meshes[i].name = mesh->mName.data;
		auto& vertices = meshes[i].vertices;
		auto& bones = meshes[i].bones;
		auto& bone_map = meshes[i].bone_map;

		vertices.resize(mesh->mNumVertices);
		for (int j = 0; j < mesh->mNumVertices; ++j)
			vertices[j].original_pos = (mesh->mVertices[j]);

		if (mesh->HasTextureCoords(0))
		{
			for (int j = 0; j < mesh->mNumVertices; ++j)
				vertices[j].tex_coords = (mesh->mTextureCoords[0][j]);
		}

		bones.resize(mesh->mNumBones);
		for (int j = 0; j < mesh->mNumBones; ++j)
		{
			auto* bone = mesh->mBones[j];
			bone_map[Mesh::processBoneName(string(bone->mName.data))] = j;
			bones[j].offset = bone->mOffsetMatrix;

			for (int k = 0; k < bone->mNumWeights; ++k)
			{
				int vertex_id = bone->mWeights[k].mVertexId;
				vertices[vertex_id].bone_index.push_back(j);
				vertices[vertex_id].bone_weight.push_back(bone->mWeights[k].mWeight);
			}
		}
	}
}

// transformation can transform bones from world space to parent space
void ModelHelper::calBoneTransformation(const aiQuaternion& global_rotation, const aiNode* cur)
{
	string name = Mesh::processBoneName(cur->mName.data);
	aiQuaternion new_global_rotation = global_rotation;
	
	for (auto& mesh : meshes)
	{
		if (mesh.bone_map.find(name) == mesh.bone_map.end())
			continue;

		auto& bone = mesh.bones[mesh.bone_map[name]];
		bone.node = cur;
		bone.name = name;

		auto trafo = cur->mTransformation;
		aiQuaternion rotation;
		aiVector3D position;
		trafo.DecomposeNoScaling(rotation, position);
		
		bone.rotation = rotation;
		bone.global_rotation = global_rotation * rotation;
		new_global_rotation = bone.global_rotation;
		bone.length = (bone.end - bone.start).Length();
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		calBoneTransformation(new_global_rotation, cur->mChildren[i]);
}

void ModelHelper::parseBoneInfo(Mesh& mesh, const string& filename)
{
	ifstream fs(filename);
	if (!fs.is_open())
		throw runtime_error("failed to open bone info");

	string bone_name;
	while (fs >> bone_name)
	{
		if (mesh.bone_map.find(bone_name) != mesh.bone_map.end())
		{
			Bone& bone = mesh.getBone(bone_name);
			float x, y, z;
			
			fs >> x >> y >> z;
			bone.start = {x, y, z};

			fs >> x >> y >> z;
			bone.end = {x, y, z};	
		}
	}
}

void Mesh::loadTexture(const std::string& filename)
{
	int height, width;
	auto* new_tex = readBMP(const_cast<char*>(filename.c_str()), width, height);
	if (new_tex == nullptr)
		throw runtime_error("failed to load texture");
	delete tex;
	tex = new_tex;
	tex_height = height;
	tex_width = width;
	tex_loaded = false;		// not loaded or updated in openGL
}

void Mesh::bindTexture()
{
	if (tex == nullptr)
	{
		glDisable(GL_TEXTURE_2D);
		return;
	}

	glEnable(GL_TEXTURE_2D);
	if (!tex_loaded)
	{
		unsigned int texture_id;
		glGenTextures(1, &texture_id);
		glBindTexture(GL_TEXTURE_2D, texture_id);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_width, tex_height,
		0, GL_RGB, GL_UNSIGNED_BYTE, tex);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		tex_loaded = true;
		tex_id = texture_id;
	}
	else
		glBindTexture(GL_TEXTURE_2D, tex_id);
}

void ModelHelper::printMeshInfo(bool showBoneHierarchy)
{
	for (int i = 0; i < meshes.size(); ++i)
	{
		std::cout << "mesh " << i << ": " << meshes[i].name << std::endl
			<< "mNumVertices: " << scene->mMeshes[i]->mNumVertices << std::endl
			<< "mNumFaces: " << scene->mMeshes[i]->mNumFaces << std::endl
			<< "mNumBones: " << scene->mMeshes[i]->mNumBones << std::endl;
	
		if (showBoneHierarchy)
		{
			meshes[i].printBoneHierarchy(scene->mRootNode, 0);
			cout << endl;
		}
	}
}

aiVector3D ModelHelper::calSphericalCoords(const aiVector3D& vec)
{
	float rho = vec.Length();
	float theta = atan2(vec.y, vec.x);
	float phi = acos(vec.z / rho);
	return aiVector3D(theta, phi, rho);
}

aiMatrix4x4t<float> ModelHelper::calTrafoMatrix(const aiVector3D& vec)
{
	float theta = -atan2(vec.y, vec.x);		// the inverse
	float phi = -acos(vec.z / vec.Length());
	aiMatrix4x4t<float> out1, out2;
	aiMatrix4x4t<float>::RotationZ(theta, out1);
	aiMatrix4x4t<float>::RotationY(phi, out2);
	return out2 * out1;		// the inverse
}

aiMatrix4x4t<float> ModelHelper::calViewingTransformation(Vec3f& eye, Vec3f& at, Vec3f& up)
{
	
	Vec3f viewDir = eye - at, upDir = up - eye;
	viewDir.normalize();
	upDir.normalize();

	// up cross view, somehow the vec header does not implement cross product
	float x = upDir[1] * viewDir[2] - upDir[2] * viewDir[1];
	float y = -(upDir[0] * viewDir[2] - upDir[2] * viewDir[0]);
	float z = upDir[0] * viewDir[1] - upDir[1] * viewDir[0];
	Vec3f leftNormal(x, y, z);
	leftNormal.normalize();

	// recalculate up by viewDir cross leftNormal
	x = viewDir[1] * leftNormal[2] - viewDir[2] * leftNormal[1];
	y = -(viewDir[0] * leftNormal[2] - viewDir[2] * leftNormal[0]);
	z = viewDir[0] * leftNormal[1] - viewDir[1] * leftNormal[0];

	upDir = {x, y, z};
	aiMatrix4x4t<float> mat;

	mat.a1 = leftNormal[0];
	mat.b1 = leftNormal[1];
	mat.c1 = leftNormal[2];
	
	mat.a2 = upDir[0];
	mat.b2 = upDir[1];
	mat.c2 = upDir[2];

	mat.a3 = viewDir[0];
	mat.b3 = viewDir[1];
	mat.c3 = viewDir[2];
	
	mat.a4 = -(leftNormal * Vec3f(eye[0], eye[1], eye[2]));
	mat.b4 = -(upDir * Vec3f(eye[0], eye[1], eye[2]));
	mat.c4 = -(viewDir * Vec3f(eye[0], eye[1], eye[2]));
	mat.d4 = 1.f;
	return mat;
}

bool Mesh::applyTranslate(const std::string& bone_name, const aiVector3D& translation)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Translation(translation, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationX(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationX(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationY(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationY(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationZ(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationZ(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyScaling(const std::string& bone_name, const aiVector3D& scale)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Scaling(scale, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::restoreIdentity(const std::string& bone_name)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = aiMatrix4x4t<float>();
	return true;
}

bool Mesh::applyMatrix(const std::string& bone_name, const aiMatrix4x4t<float>& mat)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = mat * bones[index].local_transformation;
	return true;
}

void Mesh::printBoneHierarchy(const aiNode* cur, int depth)
{
	string name = processBoneName(cur->mName.data);
	if (bone_map.find(name) != bone_map.end())
	{
		string prefix(depth, '\t');
		cout << prefix << name << endl;
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		printBoneHierarchy(cur->mChildren[i], depth + 1);
}

Bone& Mesh::getBone(const std::string& name)
{
	if (bone_map.find(name) != bone_map.end())
		return bones[bone_map[name]];

	throw invalid_argument("bone does not exist");
}

std::string Mesh::processBoneName(const std::string& name)
{
	int pos = name.find_last_of('_');
	if (pos != name.npos)
		return name.substr(pos + 1);
	return name;
}


// Apply all the user controls to meshes in one place
void applyMeshControls()
{
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationX("main", VAL(ROTATE_ALL));

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
		mesh.restoreIdentity("neck");


		mesh.applyRotationZ("neck", VAL(NECK_PITCH));
		mesh.applyRotationX("neck", VAL(NECK_YAW));
		mesh.applyRotationY("neck", VAL(NECK_ROLL));

		mesh.restoreIdentity("head");
		mesh.applyRotationZ("head", VAL(HEAD_PITCH));
		mesh.applyRotationX("head", VAL(HEAD_YAW));
		mesh.applyRotationY("head", VAL(HEAD_ROLL));

		//============================================================

		mesh.restoreIdentity("foreLimpLeft1");
		mesh.applyRotationZ("foreLimpLeft1", VAL(LEFT_FORELIMP_1));
		mesh.applyRotationX("foreLimpLeft1", VAL(LEFT_FORELIMP_1_YAW));

		mesh.restoreIdentity("foreLimpRight1");
		mesh.applyRotationZ("foreLimpRight1", VAL(RIGHT_FORELIMP_1));
		mesh.applyRotationX("foreLimpRight1", VAL(RIGHT_FORELIMP_1_YAW));

		mesh.restoreIdentity("rearLimpLeft1");
		mesh.applyRotationZ("rearLimpLeft1", VAL(LEFT_REARLIMP_1));
		mesh.applyRotationX("rearLimpLeft1", VAL(LEFT_REARLIMP_1_YAW));

		mesh.restoreIdentity("rearLimpRight1");
		mesh.applyRotationZ("rearLimpRight1", VAL(RIGHT_REARLIMP_1));
		mesh.applyRotationX("rearLimpRight1", VAL(RIGHT_REARLIMP_1_YAW));

		//============================================================

		mesh.restoreIdentity("foreLimpLeft2");
		mesh.applyRotationZ("foreLimpLeft2", VAL(LEFT_FORELIMP_2));

		mesh.restoreIdentity("foreLimpRight2");
		mesh.applyRotationZ("foreLimpRight2", VAL(RIGHT_FORELIMP_2));

		mesh.restoreIdentity("rearLimpLeft2");
		mesh.applyRotationZ("rearLimpLeft2", VAL(LEFT_REARLIMP_2));

		mesh.restoreIdentity("rearLimpRight2");
		mesh.applyRotationZ("rearLimpRight2", VAL(RIGHT_REARLIMP_2));

		//=============================================================

		mesh.applyRotationX("foreLimpLeft2", VAL(LEFT_FORELIMP_2_YAW));

		mesh.applyRotationX("foreLimpRight2", VAL(RIGHT_FORELIMP_2_YAW));

		mesh.applyRotationX("rearLimpLeft2", VAL(LEFT_REARLIMP_2_YAW));

		mesh.applyRotationX("rearLimpRight2", VAL(RIGHT_REARLIMP_2_YAW));

		//=============================================================

		mesh.restoreIdentity("foreLimpLeft3");
		mesh.applyRotationZ("foreLimpLeft3", VAL(LEFT_FORELIMP_3));
		mesh.applyRotationX("foreLimpLeft3", VAL(LEFT_FORELIMP_3_YAW));

		mesh.restoreIdentity("foreLimpRight3");
		mesh.applyRotationZ("foreLimpRight3", VAL(RIGHT_FORELIMP_3));
		mesh.applyRotationX("foreLimpRight3", VAL(RIGHT_FORELIMP_3_YAW));

		mesh.restoreIdentity("rearLimpLeft3");
		mesh.applyRotationZ("rearLimpLeft3", VAL(LEFT_REARLIMP_3));
		mesh.applyRotationX("rearLimpLeft3", VAL(LEFT_REARLIMP_3_YAW));

		mesh.restoreIdentity("rearLimpRight3");
		mesh.applyRotationZ("rearLimpRight3", VAL(RIGHT_REARLIMP_3));
		mesh.applyRotationX("rearLimpRight3", VAL(RIGHT_REARLIMP_3_YAW));

		//==============================================================

		mesh.restoreIdentity("tail");
		mesh.applyRotationZ("tail", VAL(TAIL_PITCH));
		mesh.applyRotationX("tail", VAL(TAIL_YAW));


	// For basic requirements: one slider control multiple bones
	float angle = VAL(LIMP_FOLDING);
	mesh.applyRotationZ("foreLimpLeft2", -angle);
	mesh.applyRotationZ("foreLimpLeft3", angle * 3);
	
	mesh.applyRotationZ("foreLimpRight2", -angle);
	mesh.applyRotationZ("foreLimpRight3", angle * 3);
	
	mesh.applyRotationZ("rearLimpLeft2", -angle);
	mesh.applyRotationZ("rearLimpLeft3", angle * 3);

	mesh.applyRotationZ("rearLimpRight2", -angle);
	mesh.applyRotationZ("rearLimpRight3", angle * 3);
}


void applyPeaceMood() {
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationX("main", VAL(ROTATE_ALL));

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
	mesh.restoreIdentity("neck");


	mesh.applyRotationZ("neck",5);
	mesh.applyRotationX("neck", 30);
	mesh.applyRotationY("neck", 50);
	mesh.applyRotationZ("neck", VAL(NECK_PITCH) / 3);
	mesh.applyRotationX("neck", VAL(NECK_YAW) / 3);
	mesh.applyRotationY("neck", VAL(NECK_ROLL) / 3);


	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", 25);
	mesh.applyRotationX("head", 27);
	mesh.applyRotationY("head", -19);
	mesh.applyRotationZ("head", VAL(HEAD_PITCH) / 3);
	mesh.applyRotationX("head", VAL(HEAD_YAW) / 3);
	mesh.applyRotationY("head", VAL(HEAD_ROLL) / 3);

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationZ("foreLimpLeft1",40);
	mesh.applyRotationX("foreLimpLeft1",8);

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationZ("foreLimpRight1", 40);
	mesh.applyRotationX("foreLimpRight1", -8);

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", -60);
	mesh.applyRotationX("rearLimpLeft1", 17);

	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1",-60);
	mesh.applyRotationX("rearLimpRight1", -17);

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationZ("foreLimpLeft2", -119);

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationZ("foreLimpRight2", -119);

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2", 93);

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", 93);

	//=============================================================

	mesh.applyRotationX("foreLimpLeft2", 0);

	mesh.applyRotationX("foreLimpRight2", 0);

	mesh.applyRotationX("rearLimpLeft2",-13);

	mesh.applyRotationX("rearLimpRight2",-13);

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationZ("foreLimpLeft3",180);
	mesh.applyRotationX("foreLimpLeft3", -15);

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationZ("foreLimpRight3",180);
	mesh.applyRotationX("foreLimpRight3",15);

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3", -127);
	mesh.applyRotationX("rearLimpLeft3", -44);

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3",-127);
	mesh.applyRotationX("rearLimpRight3", 44);

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationZ("tail",-45);
	mesh.applyRotationX("tail", 30);
	mesh.applyRotationZ("tail", VAL(TAIL_PITCH) / 3);
	mesh.applyRotationX("tail", VAL(TAIL_YAW) / 3);
}


void applyWatchMood() {
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationX("main", VAL(ROTATE_ALL));

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
	mesh.restoreIdentity("neck");


	mesh.applyRotationZ("neck", -27);
	mesh.applyRotationX("neck", 0);
	mesh.applyRotationY("neck", 0);
	mesh.applyRotationZ("neck", max(VAL(NECK_PITCH), 0));
	mesh.applyRotationX("neck", VAL(NECK_YAW));
	mesh.applyRotationY("neck", VAL(NECK_ROLL));

	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", -25);
	mesh.applyRotationX("head", 0);
	mesh.applyRotationY("head",0);
	mesh.applyRotationZ("head", min(0, -VAL(NECK_PITCH)));
	mesh.applyRotationX("head", -VAL(NECK_YAW));
	mesh.applyRotationY("head", VAL(HEAD_ROLL) / 3);

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationZ("foreLimpLeft1", -17);
	mesh.applyRotationX("foreLimpLeft1",0);
	mesh.applyRotationZ("foreLimpLeft1", VAL(LEFT_FORELIMP_1));
	mesh.applyRotationX("foreLimpLeft1", VAL(LEFT_FORELIMP_1_YAW));

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationZ("foreLimpRight1",5);
	mesh.applyRotationX("foreLimpRight1", 0);
	mesh.applyRotationZ("foreLimpRight1", VAL(RIGHT_FORELIMP_1));
	mesh.applyRotationX("foreLimpRight1", VAL(RIGHT_FORELIMP_1_YAW));

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", -14);
	mesh.applyRotationX("rearLimpLeft1", 0);
	mesh.applyRotationZ("rearLimpLeft1", VAL(LEFT_REARLIMP_1));
	mesh.applyRotationX("rearLimpLeft1", VAL(LEFT_REARLIMP_1_YAW));


	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1", 8);
	mesh.applyRotationX("rearLimpRight1", 0);
	mesh.applyRotationZ("rearLimpRight1", VAL(RIGHT_REARLIMP_1));
	mesh.applyRotationX("rearLimpRight1", VAL(RIGHT_REARLIMP_1_YAW));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationZ("foreLimpLeft2", -5);
	mesh.applyRotationZ("foreLimpLeft2", VAL(LEFT_FORELIMP_2));

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationZ("foreLimpRight2", 0);
	mesh.applyRotationZ("foreLimpRight2", VAL(RIGHT_FORELIMP_2));

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2",9);
	mesh.applyRotationZ("rearLimpLeft2", VAL(LEFT_REARLIMP_2));

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", 0);
	mesh.applyRotationZ("rearLimpRight2", VAL(RIGHT_REARLIMP_2));

	//=============================================================

	mesh.applyRotationX("foreLimpLeft2", 0);

	mesh.applyRotationX("foreLimpRight2", 0);

	mesh.applyRotationX("rearLimpLeft2", 0);

	mesh.applyRotationX("rearLimpRight2",0);

	mesh.applyRotationX("foreLimpLeft2", VAL(LEFT_FORELIMP_2_YAW));

	mesh.applyRotationX("foreLimpRight2", VAL(RIGHT_FORELIMP_2_YAW));

	mesh.applyRotationX("rearLimpLeft2", VAL(LEFT_REARLIMP_2_YAW));

	mesh.applyRotationX("rearLimpRight2", VAL(RIGHT_REARLIMP_2_YAW));

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationZ("foreLimpLeft3", 27);
	mesh.applyRotationX("foreLimpLeft3",0);
	mesh.applyRotationZ("foreLimpLeft3", VAL(LEFT_FORELIMP_3));
	mesh.applyRotationX("foreLimpLeft3", VAL(LEFT_FORELIMP_3_YAW));

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationZ("foreLimpRight3", 0);
	mesh.applyRotationX("foreLimpRight3", 0);
	mesh.applyRotationZ("foreLimpRight3", VAL(RIGHT_FORELIMP_3));
	mesh.applyRotationX("foreLimpRight3", VAL(RIGHT_FORELIMP_3_YAW));

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3",-9);
	mesh.applyRotationX("rearLimpLeft3", 0);
	mesh.applyRotationZ("rearLimpLeft3", VAL(LEFT_REARLIMP_3));
	mesh.applyRotationX("rearLimpLeft3", VAL(LEFT_REARLIMP_3_YAW));

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3", 0);
	mesh.applyRotationX("rearLimpRight3",0);
	mesh.applyRotationZ("rearLimpRight3", VAL(RIGHT_REARLIMP_3));
	mesh.applyRotationX("rearLimpRight3", VAL(RIGHT_REARLIMP_3_YAW));

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationZ("tail", 8);
	mesh.applyRotationX("tail", 0);
	mesh.applyRotationZ("tail", VAL(TAIL_PITCH));
	mesh.applyRotationX("tail", VAL(TAIL_YAW));
}


void applyPreJumpMood() {
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationX("main", VAL(ROTATE_ALL));
	mesh.applyRotationZ("main", -17);

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
	mesh.restoreIdentity("neck");


	mesh.applyRotationZ("neck", -18);
	mesh.applyRotationX("neck", 0);
	mesh.applyRotationY("neck", 0);
	mesh.applyRotationZ("neck", max(VAL(NECK_PITCH), 0));
	mesh.applyRotationX("neck", VAL(NECK_YAW));
	mesh.applyRotationY("neck", VAL(NECK_ROLL));

	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", -12);
	mesh.applyRotationX("head", 0);
	mesh.applyRotationY("head", 0);
	mesh.applyRotationZ("head", min(0, -VAL(NECK_PITCH)));
	mesh.applyRotationX("neck", -VAL(NECK_YAW));
	mesh.applyRotationY("head", VAL(HEAD_ROLL) / 2);

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationZ("foreLimpLeft1", -8);
	mesh.applyRotationX("foreLimpLeft1", 17);
	mesh.applyRotationZ("foreLimpLeft1", -abs(VAL(LEFT_FORELIMP_1)));
	mesh.applyRotationX("foreLimpLeft1", VAL(LEFT_FORELIMP_1_YAW));

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationZ("foreLimpRight1", -4);
	mesh.applyRotationX("foreLimpRight1", -10);
	mesh.applyRotationZ("foreLimpRight1", -abs(VAL(RIGHT_FORELIMP_1)));
	mesh.applyRotationX("foreLimpRight1", VAL(RIGHT_FORELIMP_1_YAW));

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", -14);
	mesh.applyRotationX("rearLimpLeft1", 10);
	mesh.applyRotationZ("rearLimpLeft1", -abs(VAL(LEFT_REARLIMP_1)));
	mesh.applyRotationX("rearLimpLeft1", VAL(LEFT_REARLIMP_1_YAW));

	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1", -5);
	mesh.applyRotationX("rearLimpRight1", -16);
	mesh.applyRotationZ("rearLimpRight1", -abs(VAL(RIGHT_REARLIMP_1)));
	mesh.applyRotationX("rearLimpRight1", VAL(RIGHT_REARLIMP_1_YAW));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationZ("foreLimpLeft2", 13);
	mesh.applyRotationZ("foreLimpLeft2", -abs(VAL(LEFT_FORELIMP_2)));

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationZ("foreLimpRight2", 9);
	mesh.applyRotationZ("foreLimpRight2", -abs(VAL(RIGHT_FORELIMP_2)));

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2", 27);
	mesh.applyRotationZ("rearLimpLeft2", abs(VAL(LEFT_REARLIMP_2)));

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", 25);
	mesh.applyRotationZ("rearLimpRight2", abs(VAL(RIGHT_REARLIMP_2)));

	//=============================================================

	mesh.applyRotationX("foreLimpLeft2", -8);
	mesh.applyRotationX("foreLimpLeft2", VAL(LEFT_FORELIMP_2_YAW));

	mesh.applyRotationX("foreLimpRight2", 6);
	mesh.applyRotationX("foreLimpRight2", VAL(RIGHT_FORELIMP_2_YAW));

	mesh.applyRotationX("rearLimpLeft2", 0);
	mesh.applyRotationX("rearLimpLeft2", VAL(LEFT_REARLIMP_2_YAW));

	mesh.applyRotationX("rearLimpRight2", 0);
	mesh.applyRotationX("rearLimpRight2", VAL(RIGHT_REARLIMP_2_YAW));

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationZ("foreLimpLeft3", 25);
	mesh.applyRotationX("foreLimpLeft3", -8);
	mesh.applyRotationZ("foreLimpLeft3", abs(VAL(LEFT_FORELIMP_3)));
	mesh.applyRotationX("foreLimpLeft3", VAL(LEFT_FORELIMP_3_YAW));

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationZ("foreLimpRight3", 25);
	mesh.applyRotationX("foreLimpRight3", 6);
	mesh.applyRotationZ("foreLimpRight3", abs(VAL(RIGHT_FORELIMP_3)));
	mesh.applyRotationX("foreLimpRight3", VAL(RIGHT_FORELIMP_3_YAW));

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3", -34);
	mesh.applyRotationX("rearLimpLeft3", -6);
	mesh.applyRotationZ("rearLimpLeft3", -abs(VAL(LEFT_REARLIMP_3)));
	mesh.applyRotationX("rearLimpLeft3", VAL(LEFT_REARLIMP_3_YAW));

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3", -25);
	mesh.applyRotationX("rearLimpRight3", 9);
	mesh.applyRotationZ("rearLimpRight3", -abs(VAL(RIGHT_REARLIMP_3)));
	mesh.applyRotationX("rearLimpRight3", VAL(RIGHT_REARLIMP_3_YAW));

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationZ("tail", -14);
	mesh.applyRotationX("tail", 0);
	mesh.applyRotationZ("tail", VAL(TAIL_PITCH) * 2);
	mesh.applyRotationX("tail", VAL(TAIL_YAW) * 2);
}


void applyJumpMood() {
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationX("main", VAL(ROTATE_ALL));
	mesh.applyRotationZ("main", -25);

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
	mesh.restoreIdentity("neck");


	mesh.applyRotationZ("neck", -9);
	mesh.applyRotationX("neck", 0);
	mesh.applyRotationY("neck", 0);
	mesh.applyRotationZ("neck", VAL(NECK_PITCH));
	mesh.applyRotationX("neck", VAL(NECK_YAW));
	mesh.applyRotationY("neck", VAL(NECK_ROLL));

	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", 4);
	mesh.applyRotationX("head", 24);
	mesh.applyRotationY("head", -30);
	mesh.applyRotationZ("head", VAL(HEAD_PITCH));
	mesh.applyRotationX("head", VAL(HEAD_YAW));
	mesh.applyRotationY("head", VAL(HEAD_ROLL));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationZ("foreLimpLeft1", -48);
	mesh.applyRotationX("foreLimpLeft1", 11);

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationZ("foreLimpRight1", -51);
	mesh.applyRotationX("foreLimpRight1", -17);

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", 44);
	mesh.applyRotationX("rearLimpLeft1", 23);

	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1", 41);
	mesh.applyRotationX("rearLimpRight1", -15);

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationZ("foreLimpLeft2", -13);

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationZ("foreLimpRight2", -19);

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2", 17);

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", 7);

	//=============================================================

	mesh.applyRotationX("foreLimpLeft2", 1);

	mesh.applyRotationX("foreLimpRight2", 6);

	mesh.applyRotationX("rearLimpLeft2", 0);

	mesh.applyRotationX("rearLimpRight2", 0);

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationZ("foreLimpLeft3", 135);
	mesh.applyRotationX("foreLimpLeft3", 40);

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationZ("foreLimpRight3", 145);
	mesh.applyRotationX("foreLimpRight3", -31);

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3", 15);
	mesh.applyRotationX("rearLimpLeft3", 0);

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3", 21);
	mesh.applyRotationX("rearLimpRight3", 0);

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationZ("tail", 14);
	mesh.applyRotationX("tail", 0);
	mesh.applyRotationX("tail", VAL(TAIL_YAW) * 2);
}


void applyJumpDoneMood() {
	auto& mesh = helper.meshes[helper.active_index];

	mesh.restoreIdentity("main");
	mesh.applyRotationZ("main", 32);
	mesh.applyRotationX("main", VAL(ROTATE_ALL));

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================
	mesh.restoreIdentity("neck");


	mesh.applyRotationZ("neck", -38);
	mesh.applyRotationX("neck", 41);
	mesh.applyRotationY("neck", 44);
	mesh.applyRotationZ("neck", VAL(NECK_PITCH));
	mesh.applyRotationX("neck", VAL(NECK_YAW));
	mesh.applyRotationY("neck", VAL(NECK_ROLL));

	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", 45);
	mesh.applyRotationX("head", 30);
	mesh.applyRotationY("head", 19);
	mesh.applyRotationZ("head", VAL(HEAD_PITCH));
	mesh.applyRotationX("head", VAL(HEAD_YAW));
	mesh.applyRotationY("head", VAL(HEAD_ROLL));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationZ("foreLimpLeft1", -60);
	mesh.applyRotationX("foreLimpLeft1", 18);
	mesh.applyRotationZ("foreLimpLeft1", abs(VAL(LEFT_FORELIMP_1)));
	mesh.applyRotationX("foreLimpLeft1", VAL(LEFT_FORELIMP_1_YAW));

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationZ("foreLimpRight1", -60);
	mesh.applyRotationX("foreLimpRight1", -15);
	mesh.applyRotationZ("foreLimpRight1", abs(VAL(RIGHT_FORELIMP_1)));
	mesh.applyRotationX("foreLimpRight1", VAL(RIGHT_FORELIMP_1_YAW));

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", -18);
	mesh.applyRotationX("rearLimpLeft1", 0);
	mesh.applyRotationZ("rearLimpLeft1", abs(VAL(LEFT_REARLIMP_1)));
	mesh.applyRotationX("rearLimpLeft1", VAL(LEFT_REARLIMP_1_YAW));


	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1", -13);
	mesh.applyRotationX("rearLimpRight1", 0);
	mesh.applyRotationZ("rearLimpRight1", abs(VAL(RIGHT_REARLIMP_1)));
	mesh.applyRotationX("rearLimpRight1", VAL(RIGHT_REARLIMP_1_YAW));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationZ("foreLimpLeft2", 7);
	mesh.applyRotationZ("foreLimpLeft2", abs(VAL(LEFT_FORELIMP_2)));

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationZ("foreLimpRight2", -32);
	mesh.applyRotationZ("foreLimpRight2", abs(VAL(RIGHT_FORELIMP_2)));

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2", 50);
	mesh.applyRotationZ("rearLimpLeft2", -abs(VAL(LEFT_REARLIMP_2)));

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", 50);
	mesh.applyRotationZ("rearLimpRight2", -abs(VAL(RIGHT_REARLIMP_2)));

	//=============================================================

	mesh.applyRotationX("foreLimpLeft2", -4);
	mesh.applyRotationX("foreLimpLeft2", VAL(LEFT_FORELIMP_2_YAW));

	mesh.applyRotationX("foreLimpRight2", -21);
	mesh.applyRotationX("foreLimpRight2", VAL(RIGHT_FORELIMP_2_YAW));

	mesh.applyRotationX("rearLimpLeft2", 0);
	mesh.applyRotationX("rearLimpLeft2", VAL(LEFT_REARLIMP_2_YAW));

	mesh.applyRotationX("rearLimpRight2", 0);
	mesh.applyRotationX("rearLimpRight2", VAL(RIGHT_REARLIMP_2_YAW));

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationZ("foreLimpLeft3", 5);
	mesh.applyRotationX("foreLimpLeft3", -18);
	mesh.applyRotationZ("foreLimpLeft3", -abs(VAL(LEFT_FORELIMP_3)));
	mesh.applyRotationX("foreLimpLeft3", VAL(LEFT_FORELIMP_3_YAW));

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationZ("foreLimpRight3", 109);
	mesh.applyRotationX("foreLimpRight3", -90);
	mesh.applyRotationZ("foreLimpRight3", -abs(VAL(RIGHT_FORELIMP_3)));
	mesh.applyRotationX("foreLimpRight3", VAL(RIGHT_FORELIMP_3_YAW));

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3", -54);
	mesh.applyRotationX("rearLimpLeft3", 0);
	mesh.applyRotationZ("rearLimpLeft3", abs(VAL(LEFT_REARLIMP_3)));
	mesh.applyRotationX("rearLimpLeft3", VAL(LEFT_REARLIMP_3_YAW));

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3", -78);
	mesh.applyRotationX("rearLimpRight3", 0);
	mesh.applyRotationZ("rearLimpRight3", abs(VAL(RIGHT_REARLIMP_3)));
	mesh.applyRotationX("rearLimpRight3", VAL(RIGHT_REARLIMP_3_YAW));

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationZ("tail", 23);
	mesh.applyRotationX("tail", 0);
	mesh.applyRotationZ("tail", VAL(TAIL_PITCH));
	mesh.applyRotationX("tail", VAL(TAIL_YAW));
}


void nurbsDemo()
{
	glPushMatrix();
	glDisable(GL_TEXTURE_2D);

	GLfloat ambient[] = {0.4, 0.6, 0.2, 1.0};
	GLfloat position[] = {5.0, 5.0, 5.0, 1.0};

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	//glLightfv(GL_LIGHT0, GL_POSITION, position);

	GLfloat mat_ambient[] = {0.247250, 0.199500, 0.074500, 1.000000};
	GLfloat mat_diffuse[] = {0.751640, 0.606480, 0.226480, 1.000000};
	GLfloat mat_specular[] = {0.628281, 0.555802, 0.366065, 1.000000};
	GLfloat mat_shininess[] = {51.200001};

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
	
	glScaled(0.5, 0.5, 0.5);
	glRotated(135, 1, 0, 0);
	glTranslated(-10, -10, 5);
	
	constexpr int n = 20;
	float control_points[n * n * 3];
	float t = 0;

	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			int index = (i + j * n) * 3;
			control_points[index] = i;
			control_points[index + 1] = j;
			control_points[index + 2] = cos(i) * 2 + sin(j) * 2;
		}

	drawNurbs(control_points, n, n);

	glDisable(GL_AUTO_NORMAL);
    glDisable(GL_NORMALIZE);
	glPopMatrix();
}
