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
void particle_pool_init(particle_pool_t* particle_pool, const map_t* map, const sampling_param_t* param)
{
	uint32_t max_particles = param->kld_max_particles > param->initial_sample_size ? param->kld_max_particles : param->initial_sample_size;
	//particle_pool->particles = new particle_t[max_particles];
	//particle_pool->old_particles = new particle_t[max_particles];
	//particle_pool->selected_index_sequence = new int32_t[max_particles];
	particle_pool->particles = (particle_t*)malloc(sizeof(particle_t)*max_particles);
	particle_pool->old_particles = (particle_t*)malloc(sizeof(particle_t)*max_particles);
	particle_pool->selected_index_sequence = (int32_t*)malloc(sizeof(int32_t)*max_particles);

	particle_pool->num_particles = 0;// num_particle;
	particle_pool->max_particles = max_particles;
	sample_initial_particle(particle_pool, map, param);
}

void particle_pool_fini(particle_pool_t* particle_pool)
{
	free(particle_pool->old_particles);
	free(particle_pool->particles);
	free(particle_pool->selected_index_sequence);
	//delete particle_pool->old_particles;
	//delete particle_pool->particles;
}

