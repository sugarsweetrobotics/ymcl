#pragma once
/*
* resampling.c
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "resampling.h"
#include "laser_likelihood.h"
#include "random.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>


void sample_particle_uniform(const pose_t* initial_pose, const map_t* map, particle_t *particle, const sampling_param_t* param) {
	while (TRUE) {
		particle->pose.x = random_uniform() * (param->initial_sample_x_max - param->initial_sample_x_min) + param->initial_sample_x_min;
		particle->pose.y = random_uniform() * (param->initial_sample_y_max - param->initial_sample_y_min) + param->initial_sample_y_min;
		particle->pose.th = random_uniform() * (param->initial_sample_phi_max - param->initial_sample_phi_min) + param->initial_sample_phi_min;
		int x = llroundf(particle->pose.x / map->resolution) + map->origin_index_x;
		int y = -llroundf(particle->pose.y / map->resolution) + map->origin_index_y;
		if (map_empty(map, x, y)) {
			break;
		}
	}
}

void sample_particle_gaussian(const map_t* map, particle_t *particle, const sampling_param_t* param) {
	while (TRUE) {
		particle->pose.x = random_gaussian((param->initial_sample_x_max - param->initial_sample_x_min) / 2 / 3) + param->initial_sample_pose_x;
		particle->pose.y = random_gaussian((param->initial_sample_y_max - param->initial_sample_y_min) / 2 / 3) + param->initial_sample_pose_y;
		particle->pose.th = random_gaussian((param->initial_sample_phi_max - param->initial_sample_phi_min) / 2 / 3) + param->initial_sample_pose_phi;
		int x = llroundf(particle->pose.x / map->resolution) + map->origin_index_x;
		int y = -llroundf(particle->pose.y / map->resolution) + map->origin_index_y;
		if (map_empty(map, x, y)) {
			break;
		}
	}
}

void sample_initial_particle(particle_pool_t* particle_pool, const map_t* map, const sampling_param_t* param) {
	int i = 0;
	for (; i < param->initial_sample_size; i++) {
		sample_particle_gaussian(map, particle_pool->particles + i, param);
	}
	particle_pool->num_particles = param->initial_sample_size;
}

void normalize_particle_weight(particle_pool_t* particle_pool) {
	int i;
	particle_pool->sum_weight = 0;
	for (i = 0; i < particle_pool->num_particles; i++) {
		particle_pool->particles[i].weight /= particle_pool->max_weight;
		particle_pool->sum_weight += particle_pool->particles[i].weight;
	}
	particle_pool->max_weight = 1.0;
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
	real_t stroke = particle_pool->sum_weight / particle_pool->num_particles;
	particle_pool->sampler_counter = 0;

	for (i = 0; particle_pool->sampler_counter < particle_pool->num_particles; i++) {
		if (i >= particle_pool->num_particles) i = 0;
		sum += particle_pool->particles[i].weight;
		while (sum >= threshold) {
			particle_pool->selected_index_sequence[particle_pool->sampler_counter] = i;
			threshold += stroke;
			particle_pool->sampler_counter++;
		}
	}
	particle_pool->sampler_counter = 0;
}

int systematic_selection(particle_pool_t* particle_pool) {
	int selected_index = particle_pool->selected_index_sequence[particle_pool->sampler_counter];
	particle_pool->sampler_counter++;
	if (particle_pool->sampler_counter >= particle_pool->num_particles) {
		particle_pool->sampler_counter = 0;
	}
	return selected_index;
}

void resample_particle(particle_pool_t* particle_pool, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param) {
	calc_particles_weight_with_likelihood_field(map, ranger, particle_pool, param);
	normalize_particle_weight(particle_pool);

	particle_t* new_particles = particle_pool->old_particles;// new particle_t[particle_pool->max_particles];
	int i;
	particle_pool->max_weight = -1;
	particle_pool->sum_weight = 0;
	for (i = 0; i < particle_pool->num_particles; i++) {
		int selected = roulette_selection(particle_pool);
		new_particles[i] = particle_pool->particles[selected];
		particle_pool->sum_weight += new_particles[i].weight;
		if (new_particles[i].weight > particle_pool->max_weight) {
			particle_pool->max_weight = new_particles[i].weight;
			particle_pool->max_index = i;
		}
	}
	particle_t* buf = particle_pool->particles;
	particle_pool->particles = new_particles;
	particle_pool->old_particles = buf;
}


void KLD_sampler_init(KLD_sampler_t* sampler, const map_t* map, const sampling_param_t* param) {
	sampler->numBinWidth = ceil(map->pixel_width * map->resolution / param->kld_bin_size_xy);
	sampler->numBinHeight = ceil(map->pixel_height * map->resolution / param->kld_bin_size_xy);
	sampler->numBinDirs = ceil(M_PI / param->kld_bin_size_phi);
	sampler->numBins = sampler->numBinWidth * sampler->numBinHeight * sampler->numBinDirs;
//	sampler->bin = new uint32_t[sampler->numBins]();
	sampler->bin = (uint32_t*)malloc(sizeof(uint32_t)*sampler->numBins);
}

static bool_t is_fall_into_empty_bin(KLD_sampler_t* sampler, const map_t* map, const particle_t* particle, const sampling_param_t* param) {
	int x = particle->pose.x / param->kld_bin_size_xy + (-map->topleft_x) / param->kld_bin_size_xy;
	int y = particle->pose.y / param->kld_bin_size_xy + (+map->topleft_y) / param->kld_bin_size_xy;

	//int x = (map->topleft_x + particle->pose.x) / param->kld_bin_size_xy;
	//int y = (map->topleft_y + particle->pose.y) / param->kld_bin_size_xy;
	int phi = particle->pose.th / param->kld_bin_size_phi;
	int index = (y * sampler->numBinWidth + x) * sampler->numBinDirs + phi;
	sampler->bin[index] ++;
	if (sampler->bin[index] == 1) {
		return TRUE;
	}
	return TRUE;
}

static double calc_Mx(double epsilon, double delta, double k) {
	double d = 2 / (9 * (k - 1));
	double z_delta = 3;
	return (k - 1) / (2 * epsilon)*sqrt(1 - d + sqrt(d) * z_delta);
}

void calc_particle_weight_with_likelihood_field(particle_t* particle, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param, const int scan_step);

void kld_resample_particle(particle_pool_t* particle_pool, KLD_sampler_t* sampler, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param) {
	particle_t* new_particles = particle_pool->old_particles;// new particle_t[particle_pool->max_particles];
	real_t max_weight = -1;
	int max_index = -1;
	real_t sum_weight = 0;
//	particle_pool->num_particles = 0;
	int _M = 0;
	double _Mx = 10000;
	int k = 0;
	int scan_step = ranger->num_range / param->scan_num;
	memset(sampler->bin, 0, sizeof(uint32_t) * sampler->numBins);

	systemaitc_selection_init(particle_pool);

	while (TRUE) {
		//int selected = roulette_selection(particle_pool);
		int selected = systematic_selection(particle_pool);
		new_particles[_M] = particle_pool->particles[selected];
		calc_particle_weight_with_likelihood_field(new_particles + _M, map, ranger, param, scan_step);
		sum_weight += new_particles[_M].weight;
		if (new_particles[_M].weight > max_weight) {
			max_weight = new_particles[_M].weight;
			max_index = _M;
		}
		if (is_fall_into_empty_bin(sampler, map, new_particles + _M, param)) {
			k++;
			if (k > 1) {
				_Mx = calc_Mx(param->kld_epsilon, param->kld_delta, k);
			}
		}
		_M++;
		if (((_M < _Mx) && (_M > param->kld_min_particles)) || (_M >= param->kld_max_particles)) {
			break;
		}
	}
	particle_t* buf = particle_pool->particles;
	particle_pool->particles = new_particles;
	particle_pool->max_weight = max_weight;
	particle_pool->max_index = max_index;
	particle_pool->sum_weight = sum_weight;
	particle_pool->old_particles = buf;
	particle_pool->num_particles = _M;
	normalize_particle_weight(particle_pool);
}