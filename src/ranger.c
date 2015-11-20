#pragma once
/*
* ranger.c
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "ranger.h"
#include <stdlib.h>
#include <string.h>

void ranger_data_init(ranger_data_t* ranger, const ranger_param_t *param) {
	ranger->ranges = (real_t*)malloc(sizeof(real_t) * param->num_range);
	ranger->param = *param;
}

void ranger_data_fini(ranger_data_t* ranger) {
	free(ranger->ranges);
}

void ranger_setconfig(ranger_data_t* ranger, const double min_angle, const double max_angle, const double resolution, const double min_distance, const double max_distance)
{
	ranger->param.min_angle = min_angle;
	ranger->param.max_angle = max_angle;
	ranger->param.resolution = resolution;
	ranger->param.min_distance = min_distance;
	ranger->param.max_distance = max_distance;
}


void ranger_setdata(ranger_data_t* ranger, const double* data, const uint32_t size) {
	if (ranger->param.num_range != size) {
		free(ranger->ranges);
		ranger->ranges = (real_t*)malloc(sizeof(real_t) * size);
		ranger->param.num_range = size;
	}
#if REAL_IS_DOUBLE
	memcpy(ranger->ranges, data, sizeof(real_t)*size);
#else
	for (i = 0; i < size; i++) {
		ranger->ranges[i] = data[i];
	}
#endif
}

void ranger_setoffset(ranger_data_t* ranger, const double x, const double y, const double z, const double yaw, const double roll, const double pitch)
{
	ranger->param.offset.x = x;
	ranger->param.offset.y = y;
	ranger->param.offset.z = y;
	ranger->param.offset.yaw = yaw;
	ranger->param.offset.roll = roll;
	ranger->param.offset.pitch = pitch;
}

