#pragma warning(disable : 4786)

#include "particleSystem.h"


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>

#include "modelerdraw.h"
#include "particle.h"
#include <assimp/vector3.inl>

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
	bake_fps = 30.f;
}





/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO

}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
    
	// TODO
	prevT = t;
	bake_start_time = t;
	clearBaked();
	particles.clear();

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
    
	// TODO
	bake_end_time = t;

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
    
	// TODO
	clearBaked();
	particles.clear();

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{

	// TODO
	if (!simulate)
		return;
	
	while (!particles.empty() && particles.front().life > lifespan)
	{
		particles.pop_front();
	}
	
	if (particles.size() < maxNumParticle)
	{
		for (ParticleEmitter& emitter : emitters)
		{
			vector<Particle> newParticles;
			emitter.emit(t, newParticles);
			particles.insert(particles.end(), std::make_move_iterator(newParticles.begin()),
				std::make_move_iterator(newParticles.end()));
		}
	}

	aiVector3D wind = emitters[0].initDir ^ aiVector3D(0, 1, 0);
	wind *= 100 * sin(t * 100);
	
	aiVector3D accel = (10.0f * emitters[0].initDir + gravity + wind) / mass;
	float deltaT = t - prevT;
	for (Particle& particle : particles)
	{
		particle.life += deltaT;
		particle.vel += accel * deltaT;
		particle.pos += particle.vel * deltaT;
	}

	bakeParticles(t);

	prevT = t;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{

	// TODO
	// Still simulating
	if (bake_end_time < 0)
	{
		for (Particle& particle : particles)
			drawSphere(renderingRadius, particle.pos);
		return;
	}

	// Use the cache
	int index = static_cast<int>((t - bake_start_time) * bake_fps);
	if (index < 0)
		return;
	if (index < cache.size())
	{
		for (Particle& particle : cache[index])
			drawSphere(renderingRadius, particle.pos);
	}
}





/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{

	// TODO
	int index = static_cast<int>((t - bake_start_time) * bake_fps);
	index = max(0, index);
	if (index + 1 > cache.size())
		cache.resize((cache.size() + 1) * 2);
	cache[index] = particles;
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{

	// TODO
	cache.clear();
}





