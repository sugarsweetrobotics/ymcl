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
#include "resampling.h"

typedef struct motion_data_struct_ {
	pose_t pose0;
	pose_t pose1;
	real_t delta_rot1;
	real_t delta_trans;
	real_t delta_rot2;
} motion_data_t;

void sample_motion_particles_thrun(particle_pool_t* particle_pool, motion_data_t* motion, const sampling_param_t *param);
