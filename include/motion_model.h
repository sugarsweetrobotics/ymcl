#pragma once
/*
* pose.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"
#include "pose.h"
#include "particle.h"
#include "sampling_param.h"

#ifndef _PI
#define _PI 3.141592653589793238
#endif

#ifdef __cplusplus
extern "C" {
#endif


	typedef enum {
		MOTION_MODEL_THRUN,
	} motion_model_t;

	typedef struct motion_param_struct_ {
		motion_model_t model;
		real_t alpha1;
		real_t alpha2;
		real_t alpha3;
		real_t alpha4;
		real_t update_distance;
		real_t update_heading;
	} motion_param_t;

	typedef struct motion_data_struct_ {
		pose_t pose0;
		pose_t pose1;
		real_t delta_rot1;
		real_t delta_trans;
		real_t delta_rot2;
	} motion_data_t;

	YMCL_EXPORT bool_t sample_motion_particles_thrun(particle_pool_t* particle_pool, motion_data_t* motion, const motion_param_t *param);

	static real_t normalize_angle(real_t angle) {
		while (angle > _PI) {
			angle -= 2 * _PI;
		}
		while (angle < -_PI) {
			angle += 2 * _PI;
		}

		return(angle);
	}
	


#ifdef __cplusplus
}
#endif