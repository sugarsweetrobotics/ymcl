#pragma once
/*
* laser_likelihood.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "util.h"
#include "map.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct laser_likelihood_param_struct_ {
		uint32_t scan_num;
		real_t zeta_hit;
		real_t sigma_hit;
		real_t zeta_rand;
		real_t ranger_max_distance;
	} laser_likelihood_param_t;


	typedef struct _likelihood_field_struct {
		real_t* distance_cell;
		real_t max_distance;
		map_param_t param;
		uint32_t origin_index_x;
		uint32_t origin_index_y;
	} likelihood_field_t;

#include "ranger.h"
#include "particle.h"


	static bool_t lf_validate_index(const likelihood_field_t* lf, const int x, const int y) {
		return (x >= 0 && y >= 0 && (uint32_t)x < lf->param.pixel_width && (uint32_t)y < lf->param.pixel_height);
	}

	YMCL_EXPORT void map_calc_distance_map(const map_t* map, const laser_likelihood_param_t* param, likelihood_field_t *lf);
	YMCL_EXPORT void calc_particles_weight_with_likelihood_field(particle_pool_t* particle_pool, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_likelihood_param_t* param);
	YMCL_EXPORT void calc_particle_weight_with_likelihood_field(particle_t* particle, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_likelihood_param_t* param, const int scan_step);

	YMCL_EXPORT void likelihood_field_fini(likelihood_field_t* lf);

#ifdef __cplusplus
}
#endif
