#pragma once
/*
* ranger.cpp
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "ranger.h"
#include <stdlib.h>
#include <string.h>

void ranger_data_init(ranger_data_t* ranger, const uint32_t num_range, const double min_distance, const double max_distance) {
	ranger->ranges = (real_t*)malloc(sizeof(real_t) * num_range);
	//ranger->ranges = new real_t[num_range];
	ranger->num_range = num_range;
	ranger->max_distance = max_distance;
	ranger->min_distance = min_distance;
}

void ranger_data_fini(ranger_data_t* ranger) {
	//free((void*)ranger->ranges);
//	delete ranger->ranges;
	free(ranger->ranges);
}

void ranger_setconfig(ranger_data_t* ranger, const double min_angle, const double max_angle, const double resolution, const double min_distance, const double max_distance)
{
	ranger->min_angle = min_angle;
	ranger->max_angle = max_angle;
	ranger->resolution = resolution;
	ranger->min_distance = min_distance;
	ranger->max_distance = max_distance;
}


void ranger_setdata(ranger_data_t* ranger, const double* data, const uint32_t size) {
	int i;
	if (ranger->num_range != size) {
//		delete ranger->ranges;
		//		ranger->ranges = new real_t[size];

		free(ranger->ranges);
		ranger->ranges = (real_t*)malloc(sizeof(real_t) * size);

		//ranger->ranges = (real_t*)realloc(ranger->ranges, size);
		ranger->num_range = size;
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
	ranger->offset.x = x;
	ranger->offset.y = y;
	ranger->offset.z = y;
	ranger->offset.yaw = yaw;
	ranger->offset.roll = roll;
	ranger->offset.pitch = pitch;
}

