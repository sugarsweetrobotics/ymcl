#pragma once
/*
* particle.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"

#include "pose.h"

	typedef struct particle_struct_ {
		pose_t pose;
		real_t weight;
	} particle_t;

	typedef struct particle_pool_ {
		particle_t* particles;
		particle_t* old_particles;
		int32_t* selected_index_sequence;
		int32_t sampler_counter;

		real_t min_weight;
		int32_t min_index;
		real_t max_weight;
		int32_t max_index;

		real_t sum_weight;
		int32_t num_particles;
		int32_t max_particles;

		pose_t mean_pose;
		real_t ess;
		real_t mean_weight_fast;
		real_t mean_weight_slow;
	} particle_pool_t;


#include "map.h"
#include "sampling_param.h"
#include "particle.h"

	YMCL_EXPORT void particle_pool_init(particle_pool_t* particle_pool, const map_t* map, const uint32_t max_particles);
	YMCL_EXPORT void particle_pool_fini(particle_pool_t* particle_pool);

#ifdef __cplusplus
}
#endif 
