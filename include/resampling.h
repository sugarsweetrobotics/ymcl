#pragma once
/*
* resampling.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/


#include "util.h"
#include "pose.h"
#ifdef __cplusplus
extern "C" {
#endif

	typedef enum {
		SAMPLING_ROULETTE,
		SAMPLING_SYSTEMATIC,
	} sampling_method_t;

	typedef enum {
		RANDOM_SAMPLING_NONE,
		RANDOM_SAMPLING_GAUSIAN,
		RANDOM_SAMPLING_UNIFORM,
	} random_sample_mode_t;

	typedef struct sampling_param_struct_ {
		int32_t sampling_method;

		bool_t kld_sampling;
		real_t kld_bin_size_xy;
		real_t kld_bin_size_phi;
		real_t kld_delta;
		real_t kld_epsilon;
		int32_t kld_max_particles;
		int32_t kld_min_particles;

		real_t random_alpha_fast;
		real_t random_alpha_slow;
		random_sample_mode_t random_mode;
		real_t random_sample_std_xy;
		real_t random_sample_std_phi;
		real_t random_sample_x_min;
		real_t random_sample_x_max;
		real_t random_sample_y_min;
		real_t random_sample_y_max;
		real_t random_sample_phi_min;
		real_t random_sample_phi_max;

		uint32_t resample_update_count;
		real_t resample_distance;
		real_t resample_heading;
	} sampler_param_t;

	typedef struct sampler_struct_ {
		uint32_t kld_numBinWidth;
		uint32_t kld_numBinHeight;
		uint32_t kld_numBinDirs;
		uint32_t kld_numBins;
		uint32_t*  kld_bin;
		uint32_t update_counter;
		pose_t last_resampled_pose;
	} sampler_t;

#include "util.h"
#include "map.h"
#include "ranger.h"
#include "particle.h"
#include "sampling_param.h"
#include "laser.h"

	YMCL_EXPORT void sampler_init(sampler_t* sampler, const map_t* map, const sampler_param_t* param);

	YMCL_EXPORT void sampler_fini(sampler_t* sampler);

	YMCL_EXPORT void sample_initial_particle(particle_pool_t* particle_pool, const map_t* map, const initial_param_t* param);

	YMCL_EXPORT void resample_particle(particle_pool_t* particle_pool, sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param);

	YMCL_EXPORT void calc_all_paritcles_weight(particle_pool_t* particle_pool, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* param, const sampler_param_t* sampler_param);


#ifdef __cplusplus
}
#endif
