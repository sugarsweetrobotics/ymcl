#pragma once
/*
* laser_likelihood.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include <stdio.h>
#include <stdlib.h>

#include "laser_likelihood.h"
#include "random.h"

#define _USE_MATH_DEFINES
#include <math.h>



static void pose_to_cell_lf(const pose_t* pose, const likelihood_field_t* lf, int32_t *x, int32_t* y) {
	*x = (int32_t)(lf->origin_index_x + llround(pose->x / lf->param.resolution));
	*y = (int32_t)(lf->origin_index_y - llround(pose->y / lf->param.resolution));
}

static real_t get_distance_to_nearest_cell(const map_t* map, const int x, const int y) {
	int i;

	real_t max_distance = 5.0;

	if (map_occupied(map, x, y)) {
		return 0;
	}

	for (i = 1; i < 5.0 / map->param.resolution; i++) {
		int j;
		for (j = -i; j <= i; j++) {
			int target_y = y + j;
			int target_x_d = (int)sqrt((double)(i*i - j*j));
			int target_x = target_x_d + x;

			if (target_y >= 0 && (uint32_t)target_y < map->param.pixel_height && target_x >= 0 && (uint32_t)target_x < map->param.pixel_height) {
				if (map_occupied(map, x + target_x_d, target_y)) {
					return i*map->param.resolution; // sqrt((double)(target_x_d*target_x_d + j*j));
				}
			}
			target_x = x - target_x_d;

			if (target_y >= 0 && (uint32_t)target_y < map->param.pixel_height && target_x >= 0 && (uint32_t)target_x < map->param.pixel_height) {
				if (map_occupied(map, x - target_x_d, target_y)) {
					return i*map->param.resolution;// ((double)(target_x_d*target_x_d + j*j));
				}
			}
		}
	}

	return max_distance;
}



YMCL_EXPORT void map_calc_distance_map(const map_t* map, const laser_likelihood_param_t *param, likelihood_field_t *lf) {
	//map->distance_cell = new real_t[map->pixel_width * map->pixel_height];
	lf->param = map->param;
	lf->origin_index_x = map->origin_index_x;
	lf->origin_index_y = map->origin_index_y;


	lf->distance_cell = (real_t*)malloc(lf->param.pixel_width * lf->param.pixel_height * sizeof(real_t));
	lf->max_distance = -1;

	real_t max_distance = param->ranger_max_distance;
	uint32_t i;
	/*
	for (i = 0; i < map->pixel_width; i++) {
	int j;
	int flag = 0;
	for (j = 0; j < map->pixel_height; j++) {
	if (!map_unknown(map, i, j)) {
	flag = 1;
	}

	if (flag) {
	map->distance_cell[j * map->pixel_width + i] = 0;
	}
	else {
	map->distance_cell[j * map->pixel_width + i] = param->zeta_rand / param->ranger_max_distance;
	}
	}


	flag = 0;
	for (j = map->pixel_height-1; j >=0; j--) {
	if (!map_unknown(map, i, j)) {
	flag = 1;
	}

	if (flag) {
	//				map->distance_cell[j * map->pixel_height * i] = 0;
	}
	else {
	map->distance_cell[j * map->pixel_width + i] = param->zeta_rand / param->ranger_max_distance;
	}
	}
	}

	for (i = 0; i < map->pixel_height; i++) {
	int j;
	int flag = 0;
	for (j = 0; j < map->pixel_width; j++) {
	if (!map_unknown(map, j, i)) {
	flag = 1;
	}

	if (flag) {
	//				map->distance_cell[j * map->pixel_width + i] = 0;
	}
	else {
	map->distance_cell[i * map->pixel_width + j] = param->zeta_rand / param->ranger_max_distance;
	}
	}


	flag = 0;
	for (j = map->pixel_width - 1; j >= 0; j--) {
	if (!map_unknown(map, j, i)) {
	flag = 1;
	}

	if (flag) {
	//				map->distance_cell[j * map->pixel_height * i] = 0;
	}
	else {
	map->distance_cell[i * map->pixel_width + j] = param->zeta_rand / param->ranger_max_distance;
	}
	}
	}
	*/


	//param->ranger_max_distance = max_distance;

	double max_likelihood = -1;
	for (i = 0; i < lf->param.pixel_width; i++) {
		uint32_t j;
		for (j = 0; j < lf->param.pixel_height; j++) {
			if (map_unknown(map, i, j)) {
				lf->distance_cell[j*lf->param.pixel_width + i] = 1 / param->ranger_max_distance;
			}
			else if (map_occupied(map, i, j)) {
				lf->distance_cell[j*lf->param.pixel_width + i] = param->zeta_hit * prob_normal_distribution(0, param->sigma_hit) + param->zeta_rand / param->ranger_max_distance;
			}
			else {
				real_t distance = get_distance_to_nearest_cell(map, i, j);
				//real_t distance = map->cell[j*map->pixel_width + i];
				//map->distance_cell[j*map->pixel_width + i] = distance;
				real_t likelihood = param->zeta_hit * prob_normal_distribution(distance, param->sigma_hit) + param->zeta_rand / max_distance;
				lf->distance_cell[j*lf->param.pixel_width + i] = likelihood;
				if (likelihood > max_likelihood) {
					max_likelihood = likelihood;
				}
				//				if (distance > map->max_distance) {
				//					map->max_distance = distance;
				//				}
			}
		}
	}
	real_t* distance_cell = (real_t*)malloc(lf->param.pixel_width * lf->param.pixel_height * sizeof(real_t));

	max_likelihood = -1;
	for (i = 0; i < lf->param.pixel_width; i++) {
		uint32_t j = 0;
		for (j = 0; j < lf->param.pixel_height; j++) {
			double sum = 0;
			int count = 0;

			int32_t k;
			int d = 1;
			for (k = i - d; k <= (int32_t)i + d; k++) {
				uint32_t m = j-d < 0 ? 0 : j-d;
				for (; m <= j + d; m++) {
					if (map_validate_index(map, k, m)) {
						sum += lf->distance_cell[m*lf->param.pixel_width + k];
						count++;
					}
				}
			}

			distance_cell[j*lf->param.pixel_width + i] = sum / count;
			if (sum / count > max_likelihood) {
				max_likelihood = sum / count;
			}
		}
	}

	//delete map->distance_cell;
	free(lf->distance_cell);
	lf->distance_cell = distance_cell;

	for (i = 0; i < lf->param.pixel_width; i++) {
		uint32_t j;
		for (j = 0; j < lf->param.pixel_height; j++) {
			lf->distance_cell[j*lf->param.pixel_width + i] /= max_likelihood;
		}
	}

}


YMCL_EXPORT void likelihood_field_fini(likelihood_field_t* lf) {
	free(lf->distance_cell);
	lf->distance_cell = NULL;
}


double likelihood(const pose_t* robot_pose, const likelihood_field_t* lf, const ranger_data_t* ranger, const int index) {
	pose_t z;
	int x, y;
	double th = robot_pose->th + ranger->param.offset.yaw + ranger->param.min_angle + ranger->param.resolution * index;
	double sinth = sin(th);
	double costh = cos(th);
	if (ranger->ranges[index] >= ranger->param.max_distance) {
		return -1; // skip
	}

	z.x = robot_pose->x + ranger->ranges[index] * cos(th);
	z.y = robot_pose->y + ranger->ranges[index] * sin(th);
	pose_to_cell_lf(&z, lf, &x, &y);
	if (lf_validate_index(lf, x, y)) {
		return lf->distance_cell[y * lf->param.pixel_width + x];
	}
	else {
		return -1;
	}
}

void calc_particle_weight_with_likelihood_field(particle_t* particle, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_likelihood_param_t* param, const int scan_step) {
	uint32_t j;
	particle->weight = 1;
	for (j = 0; j < ranger->param.num_range; j += scan_step) {
		double like = likelihood(&particle->pose, lf, ranger, j);
		if (like == 0) {
			like = 0.00001;
		}
		else if (like < 0) {
			like = 1;
		}
		else {
			particle->weight *= like;
		}
	}
}

void calc_particles_weight_with_likelihood_field(particle_pool_t* particle_pool, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_likelihood_param_t* param) {
	int i;
	particle_pool->max_weight = -1;
	particle_pool->sum_weight = 0;
	int scan_step = ranger->param.num_range / param->scan_num;

	for (i = 0; i < particle_pool->num_particles; i++) {
		calc_particle_weight_with_likelihood_field(particle_pool->particles + i, lf, ranger, param, scan_step);
		if (particle_pool->particles[i].weight > particle_pool->max_weight) {
			particle_pool->max_weight = particle_pool->particles[i].weight;
			particle_pool->max_index = i;
		}
		particle_pool->sum_weight += particle_pool->particles[i].weight;
	}
}