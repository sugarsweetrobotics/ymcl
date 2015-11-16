#pragma once
/*
* laser_likelihood.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"
#include "map.h"
#include "resampling.h"
#include "ranger.h"
#include "particle.h"

YMCL_EXPORT void calc_particles_weight_with_likelihood_field(const map_t* map, const ranger_data_t* ranger, particle_pool_t* particle_pool, const sampling_param_t* param);
YMCL_EXPORT void map_calc_distance_map(map_t* map, const sampling_param_t* param);
