/*
* ymcl.c
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "ymcl.h"
#include <math.h>


YMCL_EXPORT bool_t ymcl_param_init(ymcl_param_t *param) {


	param->map.pixel_width = 0;
	param->map.pixel_height = 0;
	param->map.resolution = 1.0;
	param->map.topleft_x = 0;
	param->map.topleft_y = 0;

	param->initial.sample_mode = INITIAL_SAMPLE_GAUSIAN;
	param->initial.sample_size = 1000;

	param->initial.pose_x = 0.0;
	param->initial.pose_y = 0.0;
	param->initial.pose_phi = 0.0;

	if (param->initial.sample_mode == INITIAL_SAMPLE_GAUSIAN) {
		param->initial.std_xy = 0.3;
		param->initial.std_phi = 0.1;
	}
	else {
		param->initial.sample_x_min = -0.3;
		param->initial.sample_x_max = 0.3;
		param->initial.sample_y_min = -0.3;
		param->initial.sample_y_max = 0.3;
		param->initial.sample_phi_min = -0.1;
		param->initial.sample_phi_max = 0.1;
	}

	param->motion.alpha1 = 0.2;
	param->motion.alpha2 = 0.2;
	param->motion.alpha3 = 0.2;
	param->motion.alpha4 = 0.2;
	param->motion.update_distance = -1;
	param->motion.update_heading = -1;


	param->laser.model = LASER_LIKELIHOOD;
	if (param->laser.model == LASER_LIKELIHOOD) {
		param->laser.likelihood.zeta_hit = 0.95;
		param->laser.likelihood.zeta_rand = 0.05;
		param->laser.likelihood.sigma_hit = 0.1;
		param->laser.likelihood.scan_num = 20;
		param->laser.likelihood.ranger_max_distance = 10;
	}
	else {
		param->laser.beam.scan_num = 20;
		param->laser.beam.zeta_hit = 0.95;
		param->laser.beam.zeta_rand = 0.01;
		param->laser.beam.zeta_max = 0.05;
		param->laser.beam.zeta_short = 0.04;

		param->laser.beam.sigma_max = 0.01;
		param->laser.beam.sigma_hit = 0.1;
		param->laser.beam.lambda_short = 0.01;
	}

	param->sampler.kld_bin_size_xy = 0.2;
	param->sampler.kld_bin_size_phi = 0.2;
	param->sampler.kld_epsilon = 0.05;
	param->sampler.kld_delta = 0.02;
	param->sampler.kld_max_particles = 10000;
	param->sampler.kld_min_particles = 1000;
	param->sampler.random_alpha_fast = 0.8;
	param->sampler.random_alpha_slow = 0.2;
	param->sampler.random_mode = RANDOM_SAMPLING_GAUSIAN;
	if (param->sampler.random_mode == RANDOM_SAMPLING_GAUSIAN) {
		param->sampler.random_sample_std_xy = 0.5;
		param->sampler.random_sample_std_phi = 0.2;
	}
	else if (param->sampler.random_mode == RANDOM_SAMPLING_UNIFORM){
		param->sampler.random_sample_x_min = -0.5;
		param->sampler.random_sample_x_max = 0.5;
		param->sampler.random_sample_y_min = -0.5;
		param->sampler.random_sample_y_max = 0.5;
		param->sampler.random_sample_phi_min = -0.2;
		param->sampler.random_sample_phi_max = 0.2;
	}
	param->sampler.sampling_method = SAMPLING_SYSTEMATIC;

	param->ranger.min_angle = 0;
	param->ranger.max_angle = 0;
	param->ranger.min_distance = 0.0;
	param->ranger.max_distance = 80;
	param->ranger.resolution = 1.0;
	param->ranger.num_range = 0;
	return TRUE;
}
YMCL_EXPORT bool_t ymcl_init(ymcl_t* ymcl) {

	random_init();

	map_init(&ymcl->map, &ymcl->param.map);
	ranger_data_init(&ymcl->ranger, &ymcl->param.ranger);

	return TRUE;
}

YMCL_EXPORT bool_t ymcl_cleanup(ymcl_t* ymcl) {
	particle_pool_fini(&ymcl->particle_pool);
	map_fini(&ymcl->map);
	ranger_data_fini(&ymcl->ranger);
	sampler_fini(&ymcl->sampler);
	likelihood_field_fini(&ymcl->likelihood_field);
	return TRUE;
}

YMCL_EXPORT bool_t ymcl_set_map_pixel(ymcl_t* ymcl, const int x, const int y, const map_cell_t value) {
	if (!map_validate_index(&ymcl->map, x, y)) {
		return FALSE;
	}
	ymcl->map.cell[y * ymcl->map.param.pixel_width + x] = value;
	return TRUE;
}

YMCL_EXPORT bool_t ymcl_set_initial_pose(ymcl_t* ymcl, const pose_t* pose) {
	ymcl->motion.pose0 = *pose;
	ymcl->sampler.update_counter = 0;
	ymcl->sampler.last_resampled_pose = *pose;
	return TRUE;
}

YMCL_EXPORT bool_t ymcl_reset(ymcl_t* ymcl) {
	if (ymcl->param.laser.model == LASER_LIKELIHOOD) {
		map_calc_distance_map(&ymcl->map, &ymcl->param.laser.likelihood, &ymcl->likelihood_field);
	}

	sampler_init(&ymcl->sampler, &ymcl->map, &ymcl->param.sampler);

	uint32_t max_particles = ymcl->param.sampler.kld_max_particles > ymcl->param.initial.sample_size ? ymcl->param.sampler.kld_max_particles : ymcl->param.initial.sample_size;

	particle_pool_init(&ymcl->particle_pool, &ymcl->map, max_particles);

	sample_initial_particle(&ymcl->particle_pool, &ymcl->map, &ymcl->param.initial);

	return TRUE;
}

YMCL_EXPORT bool_t ymcl_force_update_all_particles_weight(ymcl_t* ymcl) {
	calc_all_paritcles_weight(&ymcl->particle_pool, &ymcl->map, &ymcl->likelihood_field, &ymcl->ranger, &ymcl->param.laser, &ymcl->param.sampler);
	return TRUE;
}

YMCL_EXPORT bool_t ymcl_force_resample(ymcl_t* ymcl) {
	resample_particle(&ymcl->particle_pool, &ymcl->sampler, &ymcl->map, &ymcl->likelihood_field, &ymcl->ranger, &ymcl->param.laser, &ymcl->param.sampler);
	return TRUE;
}

YMCL_EXPORT bool_t ymcl_push_odometry(ymcl_t* ymcl, const pose_t* pose) {
	ymcl->motion.pose1 = *pose;
	if (sample_motion_particles_thrun(&ymcl->particle_pool, &ymcl->motion, &ymcl->param.motion)) {
		// Updated
		ymcl->sampler.update_counter++;
		double dx = ymcl->sampler.last_resampled_pose.x - pose->x;
		double dy = ymcl->sampler.last_resampled_pose.y - pose->y;
		double dth = normalize_angle(ymcl->sampler.last_resampled_pose.th - pose->th);
		double distance = sqrt(dx*dx + dy*dy);
		if ((ymcl->sampler.update_counter > ymcl->param.sampler.resample_update_count && ymcl->param.sampler.resample_update_count >= 0) || 
			(distance > ymcl->param.sampler.resample_distance || fabs(dth) > ymcl->param.sampler.resample_heading)) {
			resample_particle(&ymcl->particle_pool, &ymcl->sampler, &ymcl->map, &ymcl->likelihood_field, &ymcl->ranger, &ymcl->param.laser, &ymcl->param.sampler);
			ymcl->sampler.update_counter = 0;
			ymcl->sampler.last_resampled_pose = *pose;
		}
		ymcl->motion.pose0 = *pose;
		return TRUE;
	}
	return FALSE;
}

YMCL_EXPORT bool_t ymcl_get_mean_pose(ymcl_t* ymcl, pose_t* pose) {
	*pose = ymcl->particle_pool.mean_pose;
	return TRUE;
}
