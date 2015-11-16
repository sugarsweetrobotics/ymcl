#pragma once
/*
* pose.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "util.h"
typedef struct pose_struct_ {
	real_t x;
	real_t y;
	real_t th;
} pose_t;

typedef struct pose3d_struct_ {
	real_t x;
	real_t y;
	real_t z;
	real_t yaw;
	real_t roll;
	real_t pitch;
} pose3d_t;
