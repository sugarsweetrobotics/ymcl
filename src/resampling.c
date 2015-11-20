#pragma once
/*
* resampling.c
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include <stdio.h>


#include "resampling.h"
#include "laser.h"
#include "random.h"
#include "motion_model.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>


static void sample_particle_uniform(const map_t* map, particle_t *particle, 
	const real_t pose_x, const real_t pose_y, const real_t pose_phi,
	const real_t x_min, const real_t x_max,
	const real_t y_min, const real_t y_max,
	const real_t phi_min, const real_t phi_max) {
	while (TRUE) {
		int x, y;
		particle->pose.x = random_uniform() * (x_max - x_min) + x_min + pose_x;
		particle->pose.y = random_uniform() * (y_max - y_min) + y_min + pose_y;
		particle->pose.th = random_uniform() * (phi_max - phi_min) + phi_min + pose_phi;
		pose_to_cell(&particle->pose, map, &x, &y);
		if (map_validate_index(map, x, y)) {
			if (map_empty(map, x, y)) {
				break;
			}
		}
	}
}

static void sample_particle_gausian(const map_t* map, particle_t *particle, 
	const real_t initial_sample_pose_x, const real_t initial_sample_pose_y, const real_t initial_sample_pose_phi,
	const real_t initial_sample_std_xy, const real_t initial_sample_std_phi) {
	while (TRUE) {
		int x, y;
		particle->pose.x = random_gaussian(initial_sample_std_xy) + initial_sample_pose_x;
		particle->pose.y = random_gaussian(initial_sample_std_xy) + initial_sample_pose_y;
		particle->pose.th = random_gaussian(initial_sample_std_phi) + initial_sample_pose_phi;
		pose_to_cell(&particle->pose, map, &x, &y);
		if (map_validate_index(map, x, y)) {
			if (map_empty(map, x, y)) {
				break;
			}
		}
	}
}

void sample_initial_particle(particle_pool_t* particle_pool, const map_t* map, const initial_param_t* param) {
	int i = 0;
	if (param->sample_mode == INITIAL_SAMPLE_GAUSIAN) {
		for (; i < param->sample_size; i++) {
			sample_particle_gausian(map, particle_pool->particles + i, param->pose_x, param->pose_y, param->pose_phi,
				param->std_xy, param->std_phi);
		}
	}
	else {
		for (; i < param->sample_size; i++) {
			sample_particle_uniform(map, particle_pool->particles + i, param->pose_x, param->pose_y, param->pose_phi,
				param->sample_x_min, param->sample_x_max,
				param->sample_y_min, param->sample_y_max,
				param->sample_phi_min, param->sample_phi_max);
		}
	}
	particle_pool->num_particles = param->sample_size;
}

void sampler_init(sampler_t* sampler, const map_t* map, const sampler_param_t* param) {
	sampler->kld_numBinWidth = (uint32_t)ceil(map->param.pixel_width * map->param.resolution / param->kld_bin_size_xy);
	sampler->kld_numBinHeight = (uint32_t)ceil(map->param.pixel_height * map->param.resolution / param->kld_bin_size_xy);
	sampler->kld_numBinDirs = (uint32_t)ceil(M_PI / param->kld_bin_size_phi);
	sampler->kld_numBins = sampler->kld_numBinWidth * sampler->kld_numBinHeight * sampler->kld_numBinDirs;
	sampler->kld_bin = (uint32_t*)malloc(sizeof(uint32_t)*sampler->kld_numBins);
}

void sampler_fini(sampler_t* sampler) {
	free(sampler->kld_bin);
}


static void normalize_particle_weight(particle_pool_t* particle_pool) {
	int i;
///	particle_pool->sum_weight = 0;
	particle_pool->max_weight = -1;
	for (i = 0; i < particle_pool->num_particles; i++) {
		particle_pool->particles[i].weight /= particle_pool->sum_weight;
		if (particle_pool->particles[i].weight > particle_pool->max_weight) {
			particle_pool->max_weight = particle_pool->particles[i].weight;
			particle_pool->max_index = i;
		}
//		particle_pool->sum_weight += particle_pool->particles[i].weight;
	}
	particle_pool->sum_weight = 1.0;
}

int roulette_selection(particle_pool_t* particle_pool) {
	int i;
	real_t threshold = random_uniform() * particle_pool->sum_weight;
	real_t sum = 0;
	for (i = 0; i < particle_pool->num_particles; i++) {
		sum += particle_pool->particles[i].weight;
		if (sum >= threshold) {
			return i;
		}
	}
	return i;
}

void systemaitc_selection_init(particle_pool_t* particle_pool) {
	real_t threshold = random_uniform() * particle_pool->sum_weight;
	real_t sum = 0;
	int i;
	real_t stroke = particle_pool->sum_weight / particle_pool->max_particles;
	particle_pool->sampler_counter = 0;

	for (i = 0; particle_pool->sampler_counter < particle_pool->max_particles; i++) {
		if (i >= particle_pool->num_particles) i = 0;
		sum += particle_pool->particles[i].weight;
		while (sum >= threshold) {
			particle_pool->selected_index_sequence[particle_pool->sampler_counter] = i;
			threshold += stroke;
			particle_pool->sampler_counter++;
			if (particle_pool->sampler_counter >= particle_pool->max_particles) {
				break;
			}
		}
	}
	particle_pool->sampler_counter = 0;
}

int systematic_selection(particle_pool_t* particle_pool) {
	int selected_index = particle_pool->selected_index_sequence[particle_pool->sampler_counter];
	particle_pool->sampler_counter++;
	if (particle_pool->sampler_counter >= particle_pool->max_particles) {
		particle_pool->sampler_counter = 0;
	}
	printf("s = %d\n", selected_index);
	return selected_index;
}

static void update_mean_weight(particle_pool_t* particle_pool, const sampler_param_t* param) {
	real_t mean_weight = particle_pool->sum_weight / particle_pool->num_particles;
	if (particle_pool->mean_weight_fast < 0) {
		particle_pool->mean_weight_fast = mean_weight;
	}
	else {
		particle_pool->mean_weight_fast = (1-param->random_alpha_fast) * particle_pool->mean_weight_fast + param->random_alpha_fast * mean_weight;
	}

	if (particle_pool->mean_weight_slow < 0) {
		particle_pool->mean_weight_slow = mean_weight;
	}
	else {
		particle_pool->mean_weight_slow = (1 - param->random_alpha_slow) * particle_pool->mean_weight_slow + param->random_alpha_slow * mean_weight;
	}

}
static void kld_resample_particle(particle_pool_t* particle_pool, sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param);
static void resample_particle_fix_size(particle_pool_t* particle_pool, sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param);
static bool_t is_fall_into_empty_bin(sampler_t* sampler, const map_t* map, const particle_t* particle, const sampler_param_t* param);
static double calc_Mx(double epsilon, double delta, double k);


static bool_t is_put_random_particle(const particle_pool_t* particle_pool, const sampler_param_t* param) {
	if (param->random_mode == RANDOM_SAMPLING_NONE) { return FALSE; }
	double random = random_uniform();
	double threshold = (1.0 - particle_pool->mean_weight_fast / particle_pool->mean_weight_slow);
	threshold = threshold > 0 ? threshold : 0;
	if (random > threshold) {
		return TRUE;
	}
	return FALSE;
}

static bool_t draw_particle_with_random(particle_pool_t* particle_pool, const map_t* map, const sampler_param_t* param, particle_t* new_particle) {
//	particle_t new_particle;
	int selected;
	if ((is_put_random_particle(particle_pool, param))) {
		if (param->random_mode == RANDOM_SAMPLING_GAUSIAN) {
			sample_particle_gausian(map, new_particle, particle_pool->mean_pose.x, particle_pool->mean_pose.y, particle_pool->mean_pose.th,
				param->random_sample_std_xy, param->random_sample_std_phi);
		}
		else if(param->random_mode == RANDOM_SAMPLING_UNIFORM) {
			sample_particle_uniform(map, new_particle, particle_pool->mean_pose.x, particle_pool->mean_pose.y, particle_pool->mean_pose.th,
				param->random_sample_x_min, param->random_sample_x_max,
				param->random_sample_y_min, param->random_sample_y_max,
				param->random_sample_phi_min, param->random_sample_phi_max);
		}
		return FALSE;
	}
	else {
		if (param->sampling_method == SAMPLING_SYSTEMATIC) {
			selected = systematic_selection(particle_pool);
		}
		else {
			selected = roulette_selection(particle_pool);
		}
		*new_particle = particle_pool->particles[selected];
		return TRUE;
	}

//	return new_particle;
}

void calc_all_paritcles_weight(particle_pool_t* particle_pool, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* param, const sampler_param_t* sampler_param) {
	if (param->model == LASER_LIKELIHOOD) {
		calc_particles_weight_with_likelihood_field(particle_pool, lf, ranger, &param->likelihood);
	}
	else {
		calc_particles_weight_with_raycasting(particle_pool, map, ranger, &param->beam);
	}
	normalize_particle_weight(particle_pool);
	update_mean_weight(particle_pool, sampler_param);
}

void resample_particle(particle_pool_t* particle_pool, sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param) {
	if (param->kld_sampling) {
		kld_resample_particle(particle_pool, sampler, map, lf,  ranger, laser_param, param);
	}
	else {
		resample_particle_fix_size(particle_pool, sampler, map, lf, ranger, laser_param, param);
	}
}

static void resample_particle_fix_size(particle_pool_t* particle_pool,sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param) {
	int i;
	int max_index = 0;
	real_t max_weight = -1;
	real_t sum_weight = 0;
	particle_t* new_particles = particle_pool->old_particles;// new particle_t[particle_pool->max_particles];
	pose_t sum_pose;
	sum_pose.x = sum_pose.y = sum_pose.th = 0;
	
	calc_all_paritcles_weight(particle_pool, map, lf, ranger, laser_param, param);

	if (param->sampling_method == SAMPLING_SYSTEMATIC) {
		systemaitc_selection_init(particle_pool);
	}

	for (i = 0; i < particle_pool->num_particles; i++) {
		draw_particle_with_random(particle_pool, map, param, new_particles + i);

		sum_weight += new_particles[i].weight;
		sum_pose.x += new_particles[i].pose.x;
		sum_pose.y += new_particles[i].pose.y;
		sum_pose.th += new_particles[i].pose.th;
		if (new_particles[i].weight > max_weight) {
			max_weight = new_particles[i].weight;
			max_index = i;
		}
	}
	particle_t* buf = particle_pool->particles;
	particle_pool->particles = new_particles;
	particle_pool->max_weight = max_weight;
	particle_pool->max_index = max_index;
	particle_pool->sum_weight = sum_weight;
	particle_pool->old_particles = buf;
	particle_pool->mean_pose.x = sum_pose.x / particle_pool->num_particles;
	particle_pool->mean_pose.y = sum_pose.y / particle_pool->num_particles;
	particle_pool->mean_pose.th = sum_pose.th / particle_pool->num_particles;
	normalize_particle_weight(particle_pool);
}


void kld_resample_particle(particle_pool_t* particle_pool, sampler_t* sampler, const map_t* map, const likelihood_field_t* lf, const ranger_data_t* ranger, const laser_param_t* laser_param, const sampler_param_t* param) {
	particle_t* new_particles = particle_pool->old_particles;// new particle_t[particle_pool->max_particles];
	real_t max_weight = -1;
	int max_index = -1;

	real_t min_weight = 1000000;
	int min_index = -1;
	
	real_t sum_weight = 0;
	int _M = 0;
	double _Mx = 10000;
	int selected = 0;
	int k = 0;
	pose_t sum_pose;
	sum_pose.x = sum_pose.y = sum_pose.th = 0;
	memset(sampler->kld_bin, 0, sizeof(uint32_t) * sampler->kld_numBins);

	calc_all_paritcles_weight(particle_pool, map, lf, ranger, laser_param, param);

	if (param->sampling_method == SAMPLING_SYSTEMATIC) {
		systemaitc_selection_init(particle_pool);
	}


	while (TRUE) {
		draw_particle_with_random(particle_pool, map, param, new_particles + _M);

		sum_weight += new_particles[_M].weight;
		sum_pose.x += new_particles[_M].pose.x;
		sum_pose.y += new_particles[_M].pose.y;
		sum_pose.th += new_particles[_M].pose.th;

		if (new_particles[_M].weight > max_weight) {
			max_weight = new_particles[_M].weight;
			max_index = _M;
		}
		if (new_particles[_M].weight < min_weight) {
			min_weight = new_particles[_M].weight;
			min_index = _M;
		}
		if (is_fall_into_empty_bin(sampler, map, new_particles + _M, param)) {
			k++;
			if (k > 1) {
				_Mx = calc_Mx(param->kld_epsilon, param->kld_delta, k);
			}
		}
		_M++;
		if (((_M >= _Mx) && (_M > param->kld_min_particles)) || (_M >= param->kld_max_particles)) {
			break;
		}
	}
	particle_t* buf = particle_pool->particles;
	particle_pool->particles = new_particles;

	particle_pool->max_weight = max_weight;
	particle_pool->max_index = max_index;
	particle_pool->min_weight = min_weight;
	particle_pool->min_index = min_index;
	particle_pool->sum_weight = sum_weight;
	particle_pool->old_particles = buf;
	particle_pool->num_particles = _M;
	particle_pool->mean_pose.x = sum_pose.x / _M;
	particle_pool->mean_pose.y = sum_pose.y / _M;
	particle_pool->mean_pose.th = normalize_angle(sum_pose.th / _M);
	normalize_particle_weight(particle_pool);
}


static bool_t is_fall_into_empty_bin(sampler_t* sampler, const map_t* map, const particle_t* particle, const sampler_param_t* param) {
	int x = (int)(particle->pose.x / param->kld_bin_size_xy + (-map->param.topleft_x) / param->kld_bin_size_xy);
	int y = (int)(particle->pose.y / param->kld_bin_size_xy + (+map->param.topleft_y) / param->kld_bin_size_xy);

	//int x = (map->topleft_x + particle->pose.x) / param->kld_bin_size_xy;
	//int y = (map->topleft_y + particle->pose.y) / param->kld_bin_size_xy;
	int phi = (int)(particle->pose.th / param->kld_bin_size_phi);
	int index = (y * sampler->kld_numBinWidth + x) * sampler->kld_numBinDirs + phi;
	sampler->kld_bin[index] ++;
	if (sampler->kld_bin[index] == 1) {
		return TRUE;
	}
	return FALSE;
}

static double calc_Mx(double epsilon, double delta, double k) {
	double d = 2 / (9 * (k - 1));
	double z_delta = 0.3;
	double x = (1 - d + sqrt(d) * z_delta);
	return (k - 1) / (2 * epsilon)*x*x*x;
}