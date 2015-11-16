#pragma once

#include "util.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct sampling_param_struct_ {
	real_t initial_sample_x_min;
	real_t initial_sample_y_min;
	real_t initial_sample_phi_min;
	real_t initial_sample_x_max;
	real_t initial_sample_y_max;
	real_t initial_sample_phi_max;
	int32_t initial_sample_size;
	real_t initial_sample_pose_x;
	real_t initial_sample_pose_y;
	real_t initial_sample_pose_phi;

	real_t motion_alpha1;
	real_t motion_alpha2;
	real_t motion_alpha3;
	real_t motion_alpha4;

	real_t zeta_max;
	real_t sigma_max;
	real_t zeta_short;
	real_t lamda_short;
	real_t zeta_hit;
	real_t sigma_hit;
	real_t zeta_rand;
	real_t ranger_max_distance;
	uint32_t scan_num;

	real_t kld_bin_size_xy;
	real_t kld_bin_size_phi;
	real_t kld_delta;
	real_t kld_epsilon;
	int32_t kld_max_particles;
	int32_t kld_min_particles;

} sampling_param_t;


#ifdef __cplusplus
}
#endif
