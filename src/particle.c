/*
* particle.cpp
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include <stdlib.h>


#include "particle.h"
#include "resampling.h"

///// for initializationg of particle pool /////
YMCL_EXPORT void particle_pool_init(particle_pool_t* particle_pool, const map_t* map, const uint32_t max_particles)
{
	particle_pool->particles = (particle_t*)malloc(sizeof(particle_t)*max_particles);
	particle_pool->old_particles = (particle_t*)malloc(sizeof(particle_t)*max_particles);
	particle_pool->selected_index_sequence = (int32_t*)malloc(sizeof(int32_t)*max_particles);

	particle_pool->num_particles = 0;
	particle_pool->max_particles = max_particles;
	particle_pool->mean_weight_fast = -1;
	particle_pool->mean_weight_slow = -1;
}

YMCL_EXPORT void particle_pool_fini(particle_pool_t* particle_pool)
{
	free(particle_pool->old_particles);
	free(particle_pool->particles);
	free(particle_pool->selected_index_sequence);
}

