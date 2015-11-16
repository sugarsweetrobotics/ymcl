#pragma once
/*
* resampling.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"
#include "map.h"
#include "ranger.h"
#include "particle.h"
#include "sampling_param.h"


	typedef struct KLD_sampler_struct_ {
		uint32_t numBinWidth;
		uint32_t numBinHeight;
		uint32_t numBinDirs;
		uint32_t numBins;
		uint32_t* bin;
	}KLD_sampler_t;


#include "particle.h"

	YMCL_EXPORT void KLD_sampler_init(KLD_sampler_t* sampler, const map_t* map, const sampling_param_t* param);

	YMCL_EXPORT void sample_initial_particle(particle_pool_t* particle_pool, const map_t* map, const sampling_param_t* param);

	YMCL_EXPORT void resample_particle(particle_pool_t* particle_pool, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param);

	YMCL_EXPORT void kld_resample_particle(particle_pool_t* particle_pool, KLD_sampler_t* sampler, const map_t* map, const ranger_data_t* ranger, const sampling_param_t* param);

	YMCL_EXPORT void normalize_particle_weight(particle_pool_t* particle_pool);

#ifdef __cplusplus
}
#endif
