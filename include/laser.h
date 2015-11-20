#pragma once

#include "laser_likelihood.h"
#include "laser_raytracing.h"

#ifdef __cplusplus
extern "C" {
#endif

	enum {
		LASER_LIKELIHOOD,
		LASER_BEAM,
	};


	typedef struct laser_param_struct {
		int32_t model;
		union {
			int32_t dummy;
			laser_likelihood_param_t likelihood;
			laser_beam_param_t beam;
		};
	} laser_param_t;


#ifdef __cplusplus
}
#endif
