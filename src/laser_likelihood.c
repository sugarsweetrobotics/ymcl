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



static real_t get_distance_to_nearest_cell(const map_t* map, const int x, const int y) {
	int i;

	real_t max_distance = 5.0;

	if (map_occupied(map, x, y)) {
		return 0;
	}

	for (i = 1; i < 5.0 / map->resolution; i++) {
		int j;
		for (j = -i; j <= i; j++) {
			int target_y = y + j;
			int target_x_d = (int)sqrt((double)(i*i - j*j));
			int target_x = target_x_d + x;

			if (target_y >= 0 && (uint32_t)target_y < map->pixel_height && target_x >= 0 && (uint32_t)target_x < map->pixel_height) {
				if (map_occupied(map, x + target_x_d, target_y)) {
					return i*map->resolution; // sqrt((double)(target_x_d*target_x_d + j*j));
				}
			}
			target_x = x - target_x_d;

			if (target_y >= 0 && (uint32_t)target_y < map->pixel_height && target_x >= 0 && (uint32_t)target_x < map->pixel_height) {
				if (map_occupied(map, x - target_x_d, target_y)) {
					return i*map->resolution;// ((double)(target_x_d*target_x_d + j*j));
				}
			}
		}
	}

	return max_distance;
}



void map_calc_distance_map(map_t* map, const sampling_param_t *param) {
	//map->distance_cell = new real_t[map->pixel_width * map->pixel_height];
	map->distance_cell = (real_t*)malloc(map->pixel_width * map->pixel_height * sizeof(real_t));
	map->max_distance = -1;

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


	map->max_distance = max_distance;

	double max_likelihood = -1;
	for (i = 0; i < map->pixel_width; i++) {
		uint32_t j;
		for (j = 0; j < map->pixel_height; j++) {
			if (map_unknown(map, i, j)) {
				map->distance_cell[j*map->pixel_width + i] = 1 / param->ranger_max_distance;
			}
			else if (map_occupied(map, i, j)) {
				map->distance_cell[j*map->pixel_width + i] = param->zeta_hit * 1 + param->zeta_rand / param->ranger_max_distance;
			}
			else {
				real_t distance = get_distance_to_nearest_cell(map, i, j);
				//real_t distance = map->cell[j*map->pixel_width + i];
				//map->distance_cell[j*map->pixel_width + i] = distance;
				real_t likelihood = param->zeta_hit * prob_normal_distribution(distance, param->sigma_hit) + param->zeta_rand / max_distance;
				map->distance_cell[j*map->pixel_width + i] = likelihood;
				if (likelihood > max_likelihood) {
					max_likelihood = likelihood;
				}
				//				if (distance > map->max_distance) {
				//					map->max_distance = distance;
				//				}
			}
		}
	}


	//real_t* distance_cell = new real_t[map->pixel_height* map->pixel_width];
	real_t* distance_cell = (real_t*)malloc(map->pixel_width * map->pixel_height * sizeof(real_t));

	max_likelihood = -1;
	for (i = 0; i < map->pixel_width; i++) {
		uint32_t j = 0;
		for (j = 0; j < map->pixel_height; j++) {
			double sum = 0;
			int count = 0;

			int32_t k;
			int d = 1;
			for (k = i - d; k <= (int32_t)i + d; k++) {
				int m;
				for (m = j - d; m <= j + d; m++) {
					if (map_validate_index(map, k, m)) {
						sum += map->distance_cell[m*map->pixel_width + k];
						count++;
					}
				}
			}

			distance_cell[j*map->pixel_width + i] = sum / count;
			if (sum / count > max_likelihood) {
				max_likelihood = sum / count;
			}
		}
	}

	//delete map->distance_cell;
	free(map->distance_cell);
	map->distance_cell = distance_cell;

	for (i = 0; i < map->pixel_width; i++) {
		uint32_t j;
		for (j = 0; j < map->pixel_height; j++) {
			map->distance_cell[j*map->pixel_width + i] /= max_likelihood;
		}
	}

}




double likelihood(const pose_t* robot_pose, const map_t* map, const ranger_data_t* ranger, const int index) {
	double th = robot_pose->th + ranger->offset.yaw + ranger->min_angle + ranger->resolution * index;
	double sinth = sin(th);
	double costh = cos(th);
	//double a = sinth / costh;
	//int robot_cell_x = llroundf(robot_pose->x / map->resolution) + map->origin_index_x;
	//int robot_cell_y = -llroundf(robot_pose->y / map->resolution) + map->origin_index_y;

	if (ranger->ranges[index] >= ranger->max_distance) {
		return -1; // skip
	}

	pose_t z;
	int x, y;
	z.x = robot_pose->x + ranger->ranges[index] * cos(th);
	z.y = robot_pose->y + ranger->ranges[index] * sin(th);
	pose_to_cell(&z, map, &x, &y);
	if (map_validate_index(map, x, y)) {
		return map->distance_cell[y * map->pixel_width + x];
	}
	else {
		return -1;
	}
}





void calc_particle_weight_with_likelihood_field(particle_t* particle, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param, const int scan_step) {
	int j;
	particle->weight = 1;
	for (j = 0; j < ranger->num_range; j += scan_step) {
///		int x, y;
		double like = likelihood(&particle->pose, map, ranger, j);
		if (like == 0) {
			like = 0.00001;
		}
		else if (like < 0) {
			//particle->weight *= 1 / param->ranger_max_distance;
		}
		else {
			particle->weight *= like;
		}
		//printf("%f, ", like);
	}
	//printf("]\n");
}

void calc_particles_weight_with_likelihood_field(const map_t* map, const ranger_data_t* ranger, particle_pool_t* particle_pool, const sampling_param_t* param) {
	int i;/// , j;
	particle_pool->max_weight = -1;
	particle_pool->sum_weight = 0;
	int scan_step = ranger->num_range / param->scan_num;

	for (i = 0; i < particle_pool->num_particles; i++) {
		calc_particle_weight_with_likelihood_field(particle_pool->particles + i, map, ranger, param, scan_step);
		if (particle_pool->particles[i].weight > particle_pool->max_weight) {
			particle_pool->max_weight = particle_pool->particles[i].weight;
			particle_pool->max_index = i;
		}
		particle_pool->sum_weight += particle_pool->particles[i].weight;
		//printf("weight = %f\n", particle_pool->particles[i].weight);
		//show_range(&particle_pool->particles[i].pose, map, ranger);
	}
}