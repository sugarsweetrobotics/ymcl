#pragma once

#include "util.h"


#ifdef __cplusplus
extern "C" {
#endif

	enum {
		INITIAL_SAMPLE_UNIFORM,
		INITIAL_SAMPLE_GAUSIAN,
	};

	typedef struct initial_param_struct_ {
		int sample_mode;
		real_t sample_x_min;
		real_t sample_y_min;
		real_t sample_phi_min;
		real_t sample_x_max;
		real_t sample_y_max;
		real_t sample_phi_max;
		int32_t sample_size;
		real_t pose_x;
		real_t pose_y;
		real_t pose_phi;
		real_t std_xy;
		real_t std_phi;
	} initial_param_t;


#ifdef __cplusplus
}
#endif
