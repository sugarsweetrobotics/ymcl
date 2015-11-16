#pragma once
/*
* motion_model.cpp
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "random.h"
#include "motion_model.h"

#define _USE_MATH_DEFINES
#include <math.h>


static void sample_motion_data_thrun(particle_t* particle, const motion_data_t* motion, const sampling_param_t *param) {
	real_t rot12 = motion->delta_rot1*motion->delta_rot1;
	real_t rot22 = motion->delta_rot2*motion->delta_rot2;
	real_t trans2 = motion->delta_trans * motion->delta_trans;
	real_t delta_rot1 = motion->delta_rot1 + random_gaussian(param->motion_alpha1*rot12 + param->motion_alpha2*trans2);
	real_t delta_trans = motion->delta_trans + random_gaussian(param->motion_alpha3*trans2 + param->motion_alpha4*rot12 + param->motion_alpha4*rot22);
	real_t delta_rot2 = motion->delta_rot2 + random_gaussian(param->motion_alpha1*rot22 + param->motion_alpha2*trans2);

	particle->pose.x += delta_trans * cos(particle->pose.th + delta_rot1);
	particle->pose.y += delta_trans * sin(particle->pose.th + delta_rot1);
	particle->pose.th += delta_rot1 + delta_rot2;
}

static real_t normalize_angle(real_t angle) {
	return angle > M_PI ? angle - 2 * M_PI : (angle < -M_PI ? angle + 2 * M_PI : angle);
}

void sample_motion_particles_thrun(particle_pool_t* particle_pool, motion_data_t* motion, const sampling_param_t *param) {
	int i;
	real_t dx = motion->pose1.x - motion->pose0.x;
	real_t dy = motion->pose1.y - motion->pose0.y;
	motion->delta_rot1 = normalize_angle(atan2(dy, dx) - motion->pose0.th);
	if (motion->delta_rot1 < -M_PI / 2 || motion->delta_rot1 > M_PI / 2) {
		motion->delta_trans = -sqrt(dx*dx + dy*dy);
		motion->delta_rot1 = normalize_angle(motion->delta_rot1 + M_PI);
	}
	else {
		motion->delta_trans = sqrt(dx*dx + dy*dy);
	}

	motion->delta_rot2 = normalize_angle(motion->pose1.th - motion->pose0.th - motion->delta_rot1);

	for (i = 0; i < particle_pool->num_particles; i++) {
		sample_motion_data_thrun(particle_pool->particles + i, motion, param);
	}
}
