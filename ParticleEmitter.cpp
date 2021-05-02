#include "ParticleEmitter.h"

#include <assimp/vector2.h>

#include "ModelHelper.h"

ParticleEmitter::ParticleEmitter(const aiVector3D& localPos, const aiVector3D& dir)
{
	this->localPos = localPos;
	initDir = dir;
}

void ParticleEmitter::emit(float t, std::vector<Particle>& newParticles)
{
	if (t < emissionStartTime || t > emissionEndTime)
		return;

	aiVector3D worldPos = localPos;
	if (parentBone != nullptr)
		worldPos = parentBone->modelViewMatrix * worldPos;
	
	for (int i = 0; i < emissionRate; ++i)
	{
		aiVector2D offest = concentricSampleDisk();
		offest *= randomness;
		aiVector3D dir = aiVector3D(offest.x, offest.y, 1.0);
		newParticles.emplace_back(0.0f, worldPos, localToWorld(dir, initDir).Normalize() * initialSpeed);
	}
}

void ParticleEmitter::registerBone(Bone* bone)
{
	parentBone = bone;
}
