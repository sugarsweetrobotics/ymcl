#pragma once
/*
* pf.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/


#include "laser_raytracing.h"

#include <math.h>

real_t random_gaussian(real_t sigma);

real_t random_gaussian(real_t sigma);
double prob_normal_distribution(const double a, const double sigma);


#define INVALID_DISTANCE 0.0
double ray_casting(const pose_t* robot_pose, const map_t* map, const ranger_data_t *ranger, const real_t direction
	//	)
	, int* ix, int* iy)
{
	double th = robot_pose->th + ranger->param.offset.yaw + direction;
	double sinth = sin(th);
	double costh = cos(th);
	double a = sinth / costh;
	int robot_cell_x = (int)llroundf((float)(robot_pose->x / map->param.resolution)) + map->origin_index_x;
	int robot_cell_y = -(int)llroundf((float)(robot_pose->y / map->param.resolution)) + map->origin_index_y;
	int x = robot_cell_x;
	int y = robot_cell_y;
	int x_step = 1;
	int y_step = -1;
	int max_x = +(int)(ranger->param.max_distance * costh / map->param.resolution);
	int max_y = -(int)(ranger->param.max_distance * sinth / map->param.resolution);
	double y_real_step = -map->param.resolution;
	double delta_y = map->param.resolution / 2;
	double delta_x = map->param.resolution / 2;
	if (costh >= 0) {
		if (sinth >= 0) { // #1
			x_step = +1;
			y_step = -1;
		}
		else { // #4
			x_step = +1;
			y_step = +1;
			a *= -1;
		}
	}
	else {
		if (sinth >= 0) { // #2
			x_step = -1;
			y_step = -1;
			a *= -1;
		}
		else { // #3
			x_step = -1;
			y_step = +1;
		}
	}

	if (sinth*sinth > costh*costh) { // Steep
		while (TRUE) {
			y += y_step;
			delta_x += map->param.resolution / a;
			if (delta_x > -y_real_step) {
				delta_x += y_real_step;
				x += x_step;
				//if (map_occupied(map, x, y - y_step)) {
				//	break;
				//}
			}
			if ((x < 0 || (uint32_t)x >= map->param.pixel_width || y < 0 || (uint32_t)y >= map->param.pixel_height)) {
				return INVALID_DISTANCE;
			}
			if (map_occupied(map, x, y)) {
				break;
			}
		}
	}
	else {
		while (TRUE) {
			x += x_step;
			delta_y += a * map->param.resolution;
			if (delta_y > -y_real_step) {
				delta_y += y_real_step;
				y += y_step;
				//if (map_occupied(map, x - x_step, y)) {
				//	break;
				//}
			}
			if ((x < 0 || (uint32_t)x >= map->param.pixel_width || y < 0 || (uint32_t)y >= map->param.pixel_height)) {
				return INVALID_DISTANCE;
			}

			if (map_occupied(map, x, y)) {
				break;
			}
		}
	}
	int dx = (x - robot_cell_x);
	int dy = (y - robot_cell_y);
	*ix = x;
	*iy = y;
	double distance = sqrt((double)(dx*dx + dy*dy)) * map->param.resolution;
	if (distance > ranger->param.max_distance) return ranger->param.max_distance;
	else if (distance < ranger->param.min_distance) return ranger->param.min_distance;
	return distance;
}


static real_t prob_hit(const real_t distance, const real_t simulated_distance, const real_t sigma) {
	return prob_normal_distribution((simulated_distance - distance), sigma);
}

static real_t prob_short(const real_t distance, const real_t simulated_distance, const real_t lambda) {
	return distance < simulated_distance ? lambda * exp(-lambda * distance) : 0;
}

static real_t prob_max(const real_t distance, const real_t simulated_distance, const real_t sigma) {
	return distance < simulated_distance + sigma && distance > simulated_distance - sigma ? 1 : 0;
}

static real_t prob_rand(const real_t simulated_distance) {
	return 1 / simulated_distance;
}

static void calc_particle_weight_from_distance(real_t distance, const real_t simulated_distance, const real_t min_distance, const real_t max_distance, const laser_beam_param_t* param, particle_t* particle) {
	if (simulated_distance == INVALID_DISTANCE) {
		return;
	}
	if (distance > max_distance) { distance = max_distance; }
	else if (distance < min_distance) { distance = min_distance; }
	particle->weight +=
		param->zeta_hit * prob_hit(distance, simulated_distance, param->sigma_hit)
		+ param->zeta_short * prob_short(distance, simulated_distance, param->lambda_short)
		+ param->zeta_max   * prob_max(distance, simulated_distance, param->sigma_max)
		+ param->zeta_rand  * prob_rand(simulated_distance);
}

YMCL_EXPORT void calc_particle_weight_with_raycasting(particle_t* particle, const map_t* map, const ranger_data_t* ranger, const laser_beam_param_t* param, const int32_t scan_step) {
	uint32_t i;
	particle->weight = 0;
	for (i = 0; i < ranger->param.num_range; i += scan_step) {
		int x, y;
		calc_particle_weight_from_distance(ranger->ranges[i],
			ray_casting(&particle->pose, map, ranger, ranger->param.max_angle - ranger->param.resolution * i, &x, &y),
			ranger->param.min_distance, ranger->param.max_distance, param, particle);
	}
}


YMCL_EXPORT void calc_particles_weight_with_raycasting(particle_pool_t* particle_pool, const map_t* map, const ranger_data_t* ranger, const laser_beam_param_t* param) {
	int i;
	particle_pool->max_weight = -1;
	particle_pool->sum_weight = 0;
	int32_t scan_step = ranger->param.num_range / param->scan_num;
	for (i = 0; i < particle_pool->num_particles; i++) {
		calc_particle_weight_with_raycasting(particle_pool->particles + i, map, ranger, param, scan_step);
		if (particle_pool->particles[i].weight > particle_pool->max_weight) {
			particle_pool->max_weight = particle_pool->particles[i].weight;
			particle_pool->max_index = i;
		}
		particle_pool->sum_weight += particle_pool->particles[i].weight;
	}
}