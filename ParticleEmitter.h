#pragma once
#include <vector>
#include <assimp/vector3.h>

#include "particle.h"
#include "vec.h"

class Bone;

class ParticleEmitter
{
	friend class ParticleSystem;

public:
	ParticleEmitter() = default;
	ParticleEmitter(const aiVector3D& localPos, const aiVector3D& dir);
	virtual void emit(float t, std::vector<Particle>& newParticles);
	virtual void registerBone(Bone* bone);

protected:
	Bone* parentBone{nullptr};
	aiVector3D localPos;
	aiVector3D initDir{-1, 0, 1};

	int emissionRate{10};
	float randomness{0.1f};
	float emissionStartTime{0.0f};
	float emissionEndTime{3600.0f};
	float initialSpeed{1.0f};
};

