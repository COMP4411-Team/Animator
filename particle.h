 // SAMPLE_SOLUTION

#pragma once

#include <assimp/vector3.h>

class Particle
{
public:
	Particle(float life, const aiVector3D& pos, const aiVector3D& vel);
	float life;
	aiVector3D pos, vel;
};
