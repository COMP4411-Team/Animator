#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <FL/gl.h>
#include <gl/GLU.h>

#include "bitmap.h"
#include "camera.h"
#include "IKSolver.h"
#include "LSystem.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "modelerglobals.h"
#include "modelerui.h"
#include "modelerview.h"
#include "ModelHelper.h"
#include "Torus.h"

using namespace std;
using namespace Assimp;
using Matrix4f = aiMatrix4x4t<float>;

ModelHelper helper;		// simply use global variable for now
Matrix4f global_inverse;
float tick = 0.f;
float cur_fov = 30.f;
float cur_zfar = 100.f;
LSystem l_system;
IKSolver solver;
Torus* torus; 

// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView 
{
public:
    SampleModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) { }

    virtual void draw();
	//void ChangeLight0();
	//void ChangeLight1();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{ 
    return new SampleModel(x,y,w,h,label); 
}


inline float degree2Radian(float degree)
{
	return degree / 180.f * AI_MATH_PI_F;
}

inline float radian2Degree(float radian)
{
	return radian / AI_MATH_PI_F * 180.f;
}


void drawTriangle(const aiVector3D& v1, const aiVector3D& v2, const aiVector3D& v3)
{
	drawTriangle(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
}

void drawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
{
	drawTriangle(v1.world_pos, v2.world_pos, v3.world_pos);
}

void applyAiMatrix(const aiMatrix4x4t<float>& mat)
{
	float m[16];
	m[0] = mat.a1; m[1] = mat.b1; m[2] = mat.c1; m[3] = mat.d1;
	m[4] = mat.a2; m[5] = mat.b2; m[6] = mat.c2; m[7] = mat.d2;
	m[8] = mat.a3; m[9] = mat.b3; m[10] = mat.c3; m[11] = mat.d3;
	m[12] = mat.a4; m[13] = mat.b4; m[14] = mat.c4; m[15] = mat.d4;
	glMultMatrixf(m);
}

//control of light 0 and 1
//void SampleModel::ChangeLight0() {
	
//}

//void SampleModel::ChangeLight1() {
//	ModelerView::moveLight1(VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1));
//}

// column to row order
aiMatrix4x4t<float> array2Mat(float* a)
{
	aiMatrix4x4t<float> mat;
	mat.a1 = a[0]; mat.b1 = a[1]; mat.c1 = a[2]; mat.d1 = a[3];
	mat.a2 = a[4]; mat.b2 = a[5]; mat.c2 = a[6]; mat.d2 = a[7];
	mat.a3 = a[8]; mat.b3 = a[9]; mat.c3 = a[10]; mat.d3 = a[11];
	mat.a4 = a[12]; mat.b4 = a[13]; mat.c4 = a[14]; mat.d4 = a[15];
	return mat;
}

// Get the current model view matrix
aiMatrix4x4t<float> getModelViewMatrix()
{
    GLfloat m[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, m);
	return array2Mat(m);
}

// Animation
void animate()
{
	auto& mesh = helper.meshes[helper.active_index];
	float left1 = -cos(tick) * 20;
	float right1 = -sin(tick) * 20;
	float left2 = -cos(tick) * 45;
	float right2 = -sin(tick) * 45;
	float left3 = max(cos(tick) * 110, 0);
	float right3 = max(sin(tick) * 110, 0);
	float head = sin(tick) * 2;
	float neck = cos(tick) * 2;
	float tail = cos(tick) * 4;
	float main = sin(tick) * 0.1f;
	float fore_body = sin(tick) * 1.5f;
	float rear = cos(tick) * 1.f;

	mesh.applyTranslate("main", aiVector3D(0, 0, main));
	mesh.applyRotationZ("neck", neck);
	mesh.applyRotationZ("head", head);
	mesh.applyRotationZ("tail", tail);
	mesh.applyRotationZ("foreBody", fore_body);
	mesh.applyRotationZ("rear", rear);
	
	mesh.applyRotationZ("foreLimpLeft1", left1);
	mesh.applyRotationZ("foreLimpRight1", right1);
	mesh.applyRotationZ("rearLimpLeft1", left1);
	mesh.applyRotationZ("rearLimpRight1", right1);

	mesh.applyRotationZ("foreLimpLeft2", left2);
	mesh.applyRotationZ("foreLimpRight2", right2);
	mesh.applyRotationZ("rearLimpLeft2", left2);
	mesh.applyRotationZ("rearLimpRight2", right2);

	mesh.applyRotationZ("foreLimpLeft3", left3);
	mesh.applyRotationZ("foreLimpRight3", right3);
	mesh.applyRotationZ("rearLimpLeft3", left3 * 0.5);
	mesh.applyRotationZ("rearLimpRight3", right3 * 0.5);

	tick += 0.5f;
	if (tick > 1e4f * AI_MATH_PI_F)
		tick = 0.f;
}


// aiNode* is a node in the bone hierarchy, it contains the name, its own transformation, and
// pointers to its parent and children
void traverseBoneHierarchy(Mesh& mesh, const aiNode* cur, const Matrix4f& parent_transformation)
{
	if (cur == nullptr) return;
	string bone_name(Mesh::processBoneName(cur->mName.data));

	// cur->mTransformation transforms the node from its local space to its parent's space
	Matrix4f cur_transformation = cur->mTransformation;
	Matrix4f global_transformation = parent_transformation * cur_transformation;

	// In case some node doesn't represent a bone, we check whether we can find the bone_name in the map
	if (mesh.bone_map.find(bone_name) != mesh.bone_map.end())
	{
		int bone_index = mesh.bone_map[bone_name];

		global_transformation = global_transformation * mesh.bones[bone_index].local_transformation; 

		// final_transformation is used to transform the vertices from local space to world space
		// any other transformation should be right-multiplied to global_transformation
		mesh.bones[bone_index].final_transformation = 
			global_inverse * global_transformation * mesh.bones[bone_index].offset;
	}
	else if (mesh.parent != nullptr)
	{
		Mesh& p = *(mesh.parent);
		if (p.bone_map.find(bone_name) != p.bone_map.end())
			global_transformation = global_transformation * p.bones[p.bone_map[bone_name]].local_transformation;
	}

	// Recursively visit its children
	for (int i = 0; i < cur->mNumChildren; ++i)
	{
		traverseBoneHierarchy(mesh, cur->mChildren[i], global_transformation);
	}
}


// Update the position in world space for each vertex
void processVertices(Mesh& mesh)
{
	for (auto& vertex : mesh.vertices)
	{
		auto& bone = mesh.bones[vertex.bone_index[0]];
		Matrix4f transformation = bone.final_transformation * vertex.bone_weight[0];

		for (int i = 1; i < vertex.bone_index.size(); ++i)
		{
			auto& bone = mesh.bones[vertex.bone_index[i]];
			transformation = transformation + bone.final_transformation * vertex.bone_weight[i];
		}
		vertex.world_pos = transformation * vertex.original_pos;
	}
}


void renderMesh(Mesh& mesh)
{
	auto* ai_mesh = mesh.data;
	for (int i = 0; i < ai_mesh->mNumFaces; ++i)
	{
		const aiFace& face = ai_mesh->mFaces[i];
		//auto& vertices = mesh.vertices;
		//int v1 = face.mIndices[0];
		//int v2 = face.mIndices[1];
		//int v3 = face.mIndices[2];
		
		// drawTriangle(vertices[v1], vertices[v2], vertices[v3]);
		drawTriangle(mesh, face);
	}

	// Recalculate aabb for the mesh
	mesh.aabb_min = mesh.aabb_max = mesh.vertices[0].world_pos;
	for (auto& vt : mesh.vertices)
	{
		aiVector3D& pos = vt.world_pos;
		mesh.aabb_min.x = min(mesh.aabb_min.x, pos.x);
		mesh.aabb_min.y = min(mesh.aabb_min.y, pos.y);
		mesh.aabb_min.z = min(mesh.aabb_min.z, pos.z);

		mesh.aabb_max.x = max(mesh.aabb_max.x, pos.x);
		mesh.aabb_max.y = max(mesh.aabb_max.y, pos.y);
		mesh.aabb_max.z = max(mesh.aabb_max.z, pos.z);
	}
}


void renderBones(Mesh& mesh, const aiNode* cur)
{
	// z -> y, y -> x, x -> z
	Matrix4f permutation{0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	Matrix4f inverse_permutation = permutation;
	inverse_permutation.Inverse();
	
	glPushMatrix();

	string name = Mesh::processBoneName(cur->mName.data);
	if (mesh.bone_map.find(name) != mesh.bone_map.end())
	{
		Bone& bone = mesh.getBone(name);
		string p_name = Mesh::processBoneName(cur->mParent->mName.data);

		try
		{
			Bone& p = mesh.getBone(p_name);
			if (p.end == bone.start)
				glTranslatef(0, 0, p.length);
			
		} catch (...) { }

		// Use quaternion calculated from the mTransformation
		aiQuaternion rotation = bone.rotation;
		float theta = acos(rotation.w) * 2;
		float factor = sin(theta / 2);
		aiVector3D axis{rotation.x, rotation.y, rotation.z};
		axis = (axis / factor).Normalize();		// convert to angle-axis form

		// Note that the bone in the model is on y axis and here we draw on z axis
		// so we need to rotate the order of xyz
		glRotatef(theta * 180.f / AI_MATH_PI_F, axis.z, axis.x, axis.y);

		// Apply user controls, after change of coordinates
		applyAiMatrix(inverse_permutation * bone.local_transformation * permutation);

		// In the animator project, don't render the bone
		// drawCylinder(bone.length, 0.3, 0.01);	// cylinder for now

		// This is for recording the transformation of the end of the bone for particle emitter
		glPushMatrix();
		glTranslatef(0, 0, bone.length);
		bone.modelViewMatrix = getModelViewMatrix();
		glPopMatrix();
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		renderBones(mesh, cur->mChildren[i]);

	glPopMatrix();
}

void render(int mesh_id, void(* applyMethod)())
{
	helper.active_index = mesh_id;
	helper.meshes[mesh_id].bindTexture();
	applyMeshControls();
	applyMethod();
	traverseBoneHierarchy(helper.meshes[mesh_id], helper.scene->mRootNode, Matrix4f());
	processVertices(helper.meshes[mesh_id]);
	renderMesh(helper.meshes[mesh_id]);
}

//void adjustLight

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{
	// Change LOD
	int lod = VAL(LOD);
	switch (lod)
	{
	case 0:
		helper.active_index = 7;
		break;
	case 1:
		helper.active_index = 6;
		break;
	case 2:
		helper.active_index = 0;
		break;
	case 3:
		helper.active_index = 5;
		break;
	}
	
	// Switch between instances
	int instance = VAL(INSTANCES);
	switch (instance)
	{
	case 1: case 3:
		// helper.active_index = 0;
		break;
	case 2:
		helper.active_index = 4;
		break;
	}
    
	//ModelerView::openLight0(VAL(LIGHT0_ENABLE));
	//ModelerView::openLight1(VAL(LIGHT1_ENABLE));
	//ModelerView::moveLight0(VAL(LIGHTX_0), VAL(LIGHTY_0), VAL(LIGHTZ_0));
	//ModelerView::moveLight1(VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1));

	//if (enableFrame) {
	//	frameAll();
	//	enableFrame = FALSE;
	//}
	
	// This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();

	// Light settings
	GLfloat LightDiffuse[] = { VAL(LIGHT_INTENSITY), VAL(LIGHT_INTENSITY), VAL(LIGHT_INTENSITY) };
	GLfloat LightAmbient[] = { 0.0, 0.0, 0.0 };
	switch (int(VAL(LIGHT_RGB))) {
	case 1:
		LightDiffuse[0] *= 0.5; break;
	case 2:
		LightDiffuse[1] *= 0.5; break;
	case 3:
		LightDiffuse[2] *= 0.5; LightDiffuse[1] *= 0.7; break;
	}
	if (VAL(LIGHT0_ENABLE)) {
		glEnable(GL_LIGHT0);
		GLfloat changedLightPosition0[] = { VAL(LIGHTX_0), VAL(LIGHTY_0), VAL(LIGHTZ_0),0 };
		glLightfv(GL_LIGHT0, GL_POSITION, changedLightPosition0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
		glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	}
	else {
		glDisable(GL_LIGHT0);
	}

	if (VAL(LIGHT1_ENABLE)) {
		glEnable(GL_LIGHT1);
		GLfloat changedLightPosition1[] = { VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1),0 };
		//GLfloat LightAmbatient[] = { 0.5, 0, 0 };
		glLightfv(GL_LIGHT1, GL_POSITION, changedLightPosition1);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
		glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	}
	else {
		glDisable(GL_LIGHT1);
	}


	//const auto* glVersion = glGetString(GL_VERSION);
	//const auto* glRenderer = glGetString(GL_RENDERER);
	//const auto* glVendor = glGetString(GL_VENDOR);
	//const auto* gluVersion = glGetString(GLU_VERSION);
	//printf("version %s\n", glVersion);
	//printf("renderer %s\n", glRenderer);
	//printf("vendor %s\n", glVendor);
	//printf("glu version %s\n", gluVersion);

	// Render L-system
	if (VAL(L_SYSTEM_ENABLE))
	{
		glPushMatrix();
		glRotated(-90, 1, 0, 0);
		glTranslated(0, -5, 0);
		if (l_system.pitch_angle != VAL(L_SYSTEM_ANGLE) || l_system.forward_dist != VAL(L_SYSTEM_BRANCH_LENGTH))
		{
			l_system.pitch_angle = l_system.yaw_angle = l_system.roll_angle = VAL(L_SYSTEM_ANGLE);
			l_system.forward_dist = VAL(L_SYSTEM_BRANCH_LENGTH);
			l_system.need_regenerate = true;
		}
		l_system.draw();
		glPopMatrix();
	}

	if (VAL(POLYGON_TORUS)) {
		torus = new Torus(VAL(TORUS_TUBE_LR), VAL(TORUS_TUBE_SR), VAL(TORUS_RING_LR), VAL(TORUS_RING_SR), VAL(TORUS_PX),
			VAL(TORUS_PY), VAL(TORUS_PZ), VAL(TORUS_RX), VAL(TORUS_RY), VAL(TORUS_RZ), VAL(TORUS_FLOWER), VAL(TORUS_PETAL));
		glPushMatrix();
		torus->draw();
		glPopMatrix();
		delete torus;
	}

	if (VAL(PRIMITIVE_TORUS)) {
		glPushMatrix();
		//glScaled(VAL(TORUS_PX), VAL(TORUS_PY), VAL(TORUS_PZ));
		// glRotatef(VAL(TORUS_RX), 1.0, 0.0, 0.0);
		//glRotatef(VAL(TORUS_RY), 0.0, 1.0, 0.0);
		//glRotatef(VAL(TORUS_RZ), 0.0, 0.0, 1.0);
		drawTorus(VAL(TORUS_RING_LR),VAL(TORUS_RING_SR), VAL(TORUS_TUBE_LR),VAL(TORUS_TUBE_SR), 
			VAL(TORUS_PX), VAL(TORUS_PY), VAL(TORUS_PZ), VAL(TORUS_RX), VAL(TORUS_RY), VAL(TORUS_RZ), VAL(TORUS_FLOWER), VAL(TORUS_PETAL));
		glPopMatrix();
	}

	if (VAL(CURVE_ENABLE)) {
		glPushMatrix();
		drawCurve(VAL(POINT_X1), VAL(POINT_Y1), VAL(POINT_Z1), VAL(POINT_X2), VAL(POINT_Y2), VAL(POINT_Z2), VAL(POINT_X3), VAL(POINT_Y3), VAL(POINT_Z3), VAL(POINT_X4), VAL(POINT_Y4), VAL(POINT_Z4));
		glPopMatrix();
	}

	if (VAL(CURVE_ROTATION)) {
		glPushMatrix();
		glRotatef(30, 0.0, 1.0, 0.0);
		drawRotation(VAL(POINT_X1), VAL(POINT_Y1), VAL(POINT_Z1), VAL(POINT_X2), VAL(POINT_Y2), VAL(POINT_Z2), VAL(POINT_X3), VAL(POINT_Y3), VAL(POINT_Z3), VAL(POINT_X4), VAL(POINT_Y4), VAL(POINT_Z4));
		glPopMatrix();
	}

	if (VAL(DRAW_NURBS))
		nurbsDemo();
	
	// drawSphere(0.1);
	// drawCylinder(1, 0.1, 0.01);

	// Render the model
	if (!VAL(POLYGON_TORUS) && !VAL(PRIMITIVE_TORUS) && !VAL(DRAW_NURBS) && !VAL(CURVE_ENABLE) && !VAL(CURVE_ROTATION)) 
	{	
		// Setup environment and pose
		setAmbientColor(0.75f, 0.75f, 0.75f);
		setDiffuseColor(0.75f, 0.75f, 0.75f);
		glScaled(0.5, 0.5, 0.5);
		glRotated(105, 1, 0, 0);
		glRotated(-45, 0, 0, 1);
		glTranslated(0, 0, 5);

		// Initialization
		auto& mesh = helper.meshes[helper.active_index];
		auto* scene = helper.scene;
		global_inverse = scene->mRootNode->mTransformation.Inverse();

		mesh.bindTexture();

		// Function pointer for mesh controls
		auto applyMethod = applyMeshControls;

		// Switch between different controls according to moods
		switch (int(VAL(MOODS)))
		{
		case 1:
			applyMethod = applyPeaceMood;
			break;
		case 2:
			applyMethod = applyWatchMood;
			break;
		case 3:
			applyMethod = applyPreJumpMood;
			break;
		case 4:
			applyMethod = applyJumpMood;
			break;
		case 5:
			applyMethod = applyJumpDoneMood;
			break;
		default: 
			break;
		}

		// Apply controls to meshes
		applyMethod();

		// Animation
		//if (ModelerApplication::Instance()->m_animating && !solver.show_ik_result && int(VAL(MOODS))==0)
		//	animate();

		// Apply the solution of IKSolver
		if (solver.show_ik_result)
			solver.applyRotation(mesh);

		// Apply controls to bones and render them
		glPushMatrix();

		glLoadIdentity();
		glScaled(0.5, 0.5, 0.5);
		glRotated(105, 1, 0, 0);
		glRotated(-45, 0, 0, 1);
		glTranslated(0, 0, 5);
		
		glRotated(90, 1, 0, 0);
		glRotated(-90, 0, 0, 1);
		glTranslated(6.3, -3.3, 0);
		renderBones(mesh, scene->mRootNode);		// to record the transformation for particle emitter
		glPopMatrix();


		// Avoid overlapping bones and meshes
		//glTranslated(0, 5, 0);
		//glRotated(180, 1, 0, 0);

		// Apply the solution of IKSolver
		if (solver.show_ik_result)
			solver.applyRotation(mesh);

		// Render the meshes
		traverseBoneHierarchy(mesh, scene->mRootNode, Matrix4f());
		processVertices(mesh);
		renderMesh(mesh);

		GLfloat mat_ambient[] = {0.247250, 0.199500, 0.074500, 1.000000};
		GLfloat mat_diffuse[] = {0.751640, 0.606480, 0.226480, 1.000000};
		GLfloat mat_specular[] = {0.628281, 0.555802, 0.366065, 1.000000};
		GLfloat mat_shininess[] = {51.200001};

		switch (instance)
		{
		case 3:		// wreath
			for (int i = 1; i <= 3; ++i)
				render(i, applyMethod);
			break;
		case 4:		// bells
			glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
			render(8, applyMethod);
			break;
		case 5:		// jet pack
			glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
			render(9, applyMethod);
			render(10, applyMethod);
			break;
		}
	}

}

int main()
{
	// Load the model and textures and init IK solver
	helper.loadModel("./models/lowpolydeer_1.3.dae", "./models/lowpolydeer_bone_1.1.txt");

	helper.meshes[0].loadTexture("./models/wood_texture.bmp");

	for (int i = 4; i <= 7; ++i)
		helper.meshes[i].loadTexture("./models/wood_texture.bmp");
	
	helper.meshes[1].loadTexture("./models/wreath_cones_diffuse.bmp");
	helper.meshes[3].loadTexture("./models/wreath_cones_diffuse.bmp");
	helper.meshes[2].loadTexture("./models/wreath_diffuse.bmp");
	
	auto* scene = helper.scene;
	std::cout << "Import done, mNumMeshes: " << scene->mNumMeshes << std::endl;
	helper.printMeshInfo();

	for (int i = 1; i <= 3; ++i)
		helper.meshes[i].parent = &helper.meshes[0];

	for (int i = 8; i <= 10; ++i)
		helper.meshes[i].parent = &helper.meshes[0];

	Mesh& mesh = helper.meshes[helper.active_index];
	solver.scene = scene;
	solver.mesh = &mesh;

	// Set the particle system
	ParticleSystem* particle = new ParticleSystem();
	ParticleEmitter emitter;
	auto& emitters = particle->getEmitters();
	
	emitter.registerBone(&helper.meshes[0].getBone("foreLimpLeft3"));
	emitters.push_back(emitter);
	emitter.registerBone(&helper.meshes[0].getBone("rearLimpLeft3"));
	emitters.push_back(emitter);
	emitter.registerBone(&helper.meshes[0].getBone("foreLimpRight3"));
	emitters.push_back(emitter);
	emitter.registerBone(&helper.meshes[0].getBone("rearLimpRight3"));
	emitters.push_back(emitter);
	
	ModelerApplication::Instance()->SetParticleSystem(particle);
	
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
    controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
    controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
    controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);

	controls[LIGHT0_ENABLE] = ModelerControl("Open Light source 0?", 0, 1, 1, 1);
	controls[LIGHTX_0] = ModelerControl("Light0 X Position", -10, 10, 0.1f, 4);
	controls[LIGHTY_0] = ModelerControl("Light0 Y Position", -10, 10, 0.1f, 2);
	controls[LIGHTZ_0] = ModelerControl("Light0 Z Position", -10, 10, 0.1f,-4);

	controls[LIGHT1_ENABLE] = ModelerControl("Open Light source 1?", 0, 1, 1, 1);
	controls[LIGHTX_1] = ModelerControl("Light1 X Position", -10, 10, 0.1f, 1);
	controls[LIGHTY_1] = ModelerControl("Light1 Y Position", -10, 10, 0.1f, 1);
	controls[LIGHTZ_1] = ModelerControl("Light1 Z Position", -10, 10, 0.1f, 5);

	controls[LIGHT_INTENSITY] = ModelerControl("Lights'Intensity", 0, 2, 0.1, 1);
	controls[LIGHT_RGB] = ModelerControl("Lights'color", 0, 3, 1, 0);
	
	controls[LOD] = ModelerControl("Level Of Details", 0, 3, 1, 2);
	controls[INSTANCES] = ModelerControl("Different Instances", 1, 5, 1, 1);
	controls[MOODS] = ModelerControl("Different Moods", 0, 5, 1, 0);

	controls[ROTATE_ALL] = ModelerControl("Rotate All", -180, 180, 1, 0);

	controls[NECK_PITCH] = ModelerControl("Neck Pitch", -90, 90, 1, 0);
	controls[NECK_YAW] = ModelerControl("Neck Yaw", -90, 90, 1, 0);
	controls[NECK_ROLL] = ModelerControl("Neck Roll", -90, 90, 1, 0);

	controls[HEAD_PITCH] = ModelerControl("Head Pitch", -45, 45, 1, 0);
	controls[HEAD_YAW] = ModelerControl("Head Yaw", -30, 30, 1, 0);
	controls[HEAD_ROLL] = ModelerControl("Head Roll", -30, 30, 1, 0);

	controls[LEFT_FORELIMP_1] = ModelerControl("Left Fore Thigh Pitch", -60, 60, 1, 0);
	controls[RIGHT_FORELIMP_1] = ModelerControl("Right Fore Thigh Pitch", -60, 60, 1, 0);
	controls[LEFT_REARLIMP_1] = ModelerControl("Left Rear Thigh Pitch", -60, 60, 1, 0);
	controls[RIGHT_REARLIMP_1] = ModelerControl("Right Rear Thigh Pitch", -60, 60, 1, 0);

	controls[LEFT_FORELIMP_1_YAW] = ModelerControl("Left Fore Thigh Yaw", -60, 60, 1, 0);
	controls[RIGHT_FORELIMP_1_YAW] = ModelerControl("Right Fore Thigh Yaw", -60, 60, 1, 0);
	controls[LEFT_REARLIMP_1_YAW] = ModelerControl("Left Rear Thigh Yaw", -60, 60, 1, 0);
	controls[RIGHT_REARLIMP_1_YAW] = ModelerControl("Right Rear Thigh Yaw", -60, 60, 1, 0);

	controls[LEFT_FORELIMP_2] = ModelerControl("Left Fore Limp1 Pitch", -180, 180, 1, 0);
	controls[RIGHT_FORELIMP_2] = ModelerControl("Right Fore Limp1 Pitch", -180, 180, 1, 0);
	controls[LEFT_REARLIMP_2] = ModelerControl("Left Rear Limp1 Pitch", -180, 180, 1, 0);
	controls[RIGHT_REARLIMP_2] = ModelerControl("Right Rear Limp1 Pitch", -180, 180, 1, 0);

	controls[LEFT_FORELIMP_2_YAW] = ModelerControl("Left Fore Limp1 Yaw", -60, 60, 1, 0);
	controls[RIGHT_FORELIMP_2_YAW] = ModelerControl("Right Fore Limp1 Yaw", -60, 60, 1, 0);
	controls[LEFT_REARLIMP_2_YAW] = ModelerControl("Left Rear Limp1 Yaw", -60, 60, 1, 0);
	controls[RIGHT_REARLIMP_2_YAW] = ModelerControl("Right Rear Limp1 Yaw", -60, 60, 1, 0);

	controls[LEFT_FORELIMP_3] = ModelerControl("Left Fore Limp2 Pitch", -180, 180, 1, 0);
	controls[RIGHT_FORELIMP_3] = ModelerControl("Right Fore Limp2 Pitch", -180, 180, 1, 0);
	controls[LEFT_REARLIMP_3] = ModelerControl("Left Rear Limp2 Pitch", -180, 180, 1, 0);
	controls[RIGHT_REARLIMP_3] = ModelerControl("Right Rear Limp2 Pitch", -180, 180, 1, 0);

	controls[LEFT_FORELIMP_3_YAW] = ModelerControl("Left Fore Limp2 Yaw", -60, 60, 1, 0);
	controls[RIGHT_FORELIMP_3_YAW] = ModelerControl("Right Fore Limp2 Yaw", -60, 60, 1, 0);
	controls[LEFT_REARLIMP_3_YAW] = ModelerControl("Left Rear Limp2 Yaw", -60, 60, 1, 0);
	controls[RIGHT_REARLIMP_3_YAW] = ModelerControl("Right Rear Limp2 Yaw", -60, 60, 1, 0);

	controls[TAIL_PITCH] = ModelerControl("Tail Pitch", -45, 45, 1, 0);
	controls[TAIL_YAW] = ModelerControl("Tail Yaw", -30, 30, 1, 0);

	controls[LIMP_FOLDING] = ModelerControl("Limp Folding", -45, 45, 1, 0);

	controls[L_SYSTEM_ENABLE] = ModelerControl("L-system Enable", 0, 1, 1, 0);
	controls[L_SYSTEM_ANGLE] = ModelerControl("L-system Angle", 0, 60, 1, 22.5);
	controls[L_SYSTEM_BRANCH_LENGTH] = ModelerControl("L-system Branch Length", 0, 1, 0.001, 0.1);

	controls[CURVE_ENABLE] = ModelerControl("Draw curve", 0, 1, 1, 0);
	controls[CURVE_ROTATION] = ModelerControl("Rotate the curve", 0, 1, 1, 0);
	controls[POINT_X1] = ModelerControl("x of 1st point on the curve", -5, 5, 0.1, -3);
	controls[POINT_Y1] = ModelerControl("y of 1st point on the curve", -5, 5, 0.1, -3);
	controls[POINT_Z1] = ModelerControl("z of 1st point on the curve", -5, 5, 0.1, -3);
	controls[POINT_X2] = ModelerControl("x of 2nd point on the curve", -5, 5, 0.1, -1);
	controls[POINT_Y2] = ModelerControl("y of 2nd point on the curve", -5, 5, 0.1, -2);
	controls[POINT_Z2] = ModelerControl("z of 2nd point on the curve", -5, 5, 0.1, -1.5);
	controls[POINT_X3] = ModelerControl("x of 3rd point on the curve", -5, 5, 0.1, 1.5);
	controls[POINT_Y3] = ModelerControl("y of 3rd point on the curve", -5, 5, 0.1, 3);
	controls[POINT_Z3] = ModelerControl("z of 3rd point on the curve", -5, 5, 0.1, 0.5);
	controls[POINT_X4] = ModelerControl("x of 4th point on the curve", -5, 5, 0.1, 2);
	controls[POINT_Y4] = ModelerControl("y of 4th point on the curve", -5, 5, 0.1, 1.5);
	controls[POINT_Z4] = ModelerControl("z of 4th point on the curve", -5, 5, 0.1, 3);

	controls[POLYGON_TORUS] = ModelerControl("Polygon Torus Enable", 0, 1, 1, 0);
	controls[PRIMITIVE_TORUS] = ModelerControl("Primitive Torus Enable", 0, 1, 1, 0);
	controls[TORUS_RING_LR] = ModelerControl("Longer Radius of Torus Ring", 2, 5, 0.1, 4);
	controls[TORUS_RING_SR] = ModelerControl("Shorter Radius of Torus Ring", 0.5, 5, 0.1, 3);
	controls[TORUS_TUBE_LR] = ModelerControl("Longer Radius of Torus Tube", 0.01, 1, 0.01, 0.5);
	controls[TORUS_TUBE_SR] = ModelerControl("Shorter Radius of Torus Tube", 0.01, 1, 0.01, 0.3);
	controls[TORUS_PX] = ModelerControl("x Position of Torus", -5, 5, 0.1, 0);
	controls[TORUS_PY] = ModelerControl("y Position of Torus", -5, 5, 0.1, 0);
	controls[TORUS_PZ] = ModelerControl("z Position of Torus", -5, 5, 0.1, 0);
	controls[TORUS_RX] = ModelerControl("x Rotation of Torus", -90, 90, 1, 0);
	controls[TORUS_RY] = ModelerControl("y Rotation of Torus", -90, 90, 1, 0);
	controls[TORUS_RZ] = ModelerControl("z Rotation of Torus", -90, 90, 1, 0);
	controls[TORUS_FLOWER] = ModelerControl("Use Flower Shape Torus?" ,0, 1, 1, 0);
	controls[TORUS_PETAL] = ModelerControl("Number of Flower Petals" ,3, 8, 1, 3);

	controls[DRAW_NURBS] = ModelerControl("Extruded Surface", 0, 1, 1, 0);

    ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
