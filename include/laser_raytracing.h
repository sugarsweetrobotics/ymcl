#pragma once
/*
* laser_raytracing.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif


	typedef struct laser_beam_param_struct_ {
		uint32_t scan_num;
		real_t zeta_hit;
		real_t zeta_short;
		real_t zeta_max;
		real_t zeta_rand;

		real_t sigma_hit;
		real_t lambda_short;
		real_t sigma_max;
	} laser_beam_param_t;



#include "particle.h"
#include "map.h"
#include "resampling.h"
#include "ranger.h"


	YMCL_EXPORT void calc_particles_weight_with_raycasting(particle_pool_t* particle_pool, const map_t* map, const ranger_data_t* ranger, const laser_beam_param_t* param);

	YMCL_EXPORT void calc_particle_weight_with_raycasting(particle_t* particle, const map_t* map, const ranger_data_t* ranger, const laser_beam_param_t* param, const int32_t scan_step);

	YMCL_EXPORT double ray_casting(const pose_t* robot_pose, const map_t* map, const ranger_data_t *ranger, const real_t direction
	//);
	, int *xindex, int * yindex);





#ifdef __cplusplus
}
#endif
