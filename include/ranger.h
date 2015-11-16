#pragma once
/*
* ranger.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "util.h"
#include "pose.h"

typedef struct ranger_data_struct_ {
	real_t* ranges;
	uint32_t num_range;
	pose3d_t offset;
	real_t min_angle;
	real_t max_angle;
	real_t resolution;
	real_t min_distance;
	real_t max_distance;
} ranger_data_t;


YMCL_EXPORT void ranger_data_init(ranger_data_t* ranger, const uint32_t num_range, const double min_distance, const double max_distance);
YMCL_EXPORT void ranger_data_fini(ranger_data_t* ranger);

YMCL_EXPORT void ranger_setconfig(ranger_data_t* ranger, const double min_angle, const double max_rangle, const double resolution, const double min_distance, const double max_distance);
YMCL_EXPORT void ranger_setdata(ranger_data_t* ranger, const double* data, const uint32_t size);
YMCL_EXPORT void ranger_setoffset(ranger_data_t* ranger, const double x, const double y, const double z, const double yaw, const double roll, const double pitch);
