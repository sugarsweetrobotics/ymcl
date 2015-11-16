#pragma once
/*
* laser_raytracing.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"
#include "map.h"
#include "resampling.h"
#include "ranger.h"
#include "particle.h"


void calc_particles_weight_with_raycasting(const map_t* map, const ranger_data_t* ranger, particle_pool_t* particle_pool, const sampling_param_t* param);

double ray_casting(const pose_t* robot_pose, const map_t* map, const ranger_data_t *ranger, const real_t direction
	//);
	, int *xindex, int * yindex);





