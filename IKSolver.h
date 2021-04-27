#pragma once

#include <assimp/scene.h>
#include "ModelHelper.h"

class IKSolver
{
public:
	enum class EndEffector
	{
		HEAD, LEFT_FORE_FOOT, RIGHT_FORE_FOOT, LEFT_REAR_FOOT, RIGHT_REAR_FOOT
	};

	// Pitch is around Z axis, yaw is around X axis, roll is around Y axis
	class Constraints
	{
	public:
		bool enable_yaw{true}, enable_pitch{true}, enable_roll{true};
		float max_yaw_angle{180.f}, min_yaw_angle{-180.f};
		float max_pitch_angle{180.f}, min_pitch_angle{-180.f};
		float max_roll_angle{180.f}, min_roll_angle{-180.f};
	};

	IKSolver();
	
	void setBoneChain(EndEffector end);
	void setContext();
	bool traverseBones(const aiNode* cur);
	void solve();
	void ccdSolve(Bone& cur, Bone& end, int index, int iter);
	void updateBonePos(int index);
	void applyRotation(Mesh& mesh);

	static aiVector3D quaternion2Euler(const aiQuaternion& quaternion);

	const aiScene* scene;
	Mesh* mesh;
	std::string start, end;
	std::vector<Bone> bones;
	std::vector<Constraints> constraints;
	aiVector3D offset;
	aiVector3D target;
	float angle_limit{AI_MATH_PI_F / 6.f};
	float epsilon{1e-3f};
	int max_iter{20};
	bool show_ik_result{false};
	bool enable_constraints{false};

private:
	bool isApproximatelyEqual(const aiVector3D& a, const aiVector3D& b);
	float clamp(float value, float lower_bound, float upper_bound);
	static aiVector3D degree2Radian(const aiVector3D& angles);
	static aiVector3D radian2Degree(const aiVector3D& angles);
};

