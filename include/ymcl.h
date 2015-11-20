#pragma once
/*
 * pf.h
 * author: Yuki Suga
 * copyright: Yuki Suga, 2015
 * license: GPLv3
 */

#include "util.h"


#include "random.h"
#include "particle.h"
#include "map.h"
#include "resampling.h"
#include "motion_model.h"
#include "laser.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct _ymcl_param_struct {
		map_param_t map;
		laser_param_t laser;
		sampler_param_t sampler;
		initial_param_t initial;
		ranger_param_t ranger;
		motion_param_t motion;
	} ymcl_param_t;

	typedef struct _ymcl_struct {
		map_t map;
		ymcl_param_t param;
		likelihood_field_t likelihood_field;
		particle_pool_t particle_pool;
		pose_t initial_pose;
		ranger_data_t ranger;
		motion_data_t motion;
		sampler_t sampler;
	} ymcl_t;


	YMCL_EXPORT bool_t ymcl_param_init(ymcl_param_t *param);

	YMCL_EXPORT bool_t ymcl_init(ymcl_t* ymcl);

	YMCL_EXPORT bool_t ymcl_cleanup(ymcl_t* ymcl);

	YMCL_EXPORT bool_t ymcl_set_map_pixel(ymcl_t* ymcl, const int x, const int y, const map_cell_t value);

	YMCL_EXPORT bool_t ymcl_set_initial_pose(ymcl_t* ymcl, const pose_t *pose);

	YMCL_EXPORT bool_t ymcl_reset(ymcl_t* ymcl);

	YMCL_EXPORT bool_t ymcl_force_update_all_particles_weight(ymcl_t* ymcl);

	YMCL_EXPORT bool_t ymcl_force_resample(ymcl_t* ymcl);

	YMCL_EXPORT bool_t ymcl_push_odometry(ymcl_t* ymcl, const pose_t* pose);

	YMCL_EXPORT bool_t ymcl_get_mean_pose(ymcl_t* ymcl, pose_t* pose);
#ifdef __cplusplus
}
#endif
