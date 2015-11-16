#pragma once
/*
* map.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"

typedef struct map_struct_ {
	uint32_t pixel_width;
	uint32_t pixel_height;
	real_t resolution;
	uint8_t* cell;
	real_t topleft_x;
	real_t topleft_y;
	uint32_t origin_index_x;
	uint32_t origin_index_y;
	real_t* distance_cell;
	real_t max_distance;
} map_t;


#include "pose.h"

#include <math.h>


YMCL_EXPORT void pose_to_cell(const pose_t* pose, const map_t* map, int32_t *x, int32_t* y);


static int map_occupied(const map_t* map, uint32_t x, uint32_t y) {
	return map->cell[y * map->pixel_width + x] < 100 ? 1 : 0;
}

static int map_empty(const map_t* map, uint32_t x, uint32_t y) {
	return map->cell[y * map->pixel_width + x] > 200 ? 1 : 0;
}

static int map_unknown(const map_t* map, uint32_t x, uint32_t y) {
	return map->cell[y * map->pixel_width + x] <= 200 && map->cell[y * map->pixel_width + x] >= 100 ? 1 : 0;
}

static int map_validate_index(const map_t* map, const int x, const int y) {
	return (x >= 0 && y >= 0 && (uint32_t)x < map->pixel_width && (uint32_t)y < map->pixel_height);
}


YMCL_EXPORT void map_init(const uint32_t pixel_width, const uint32_t pixel_height, const real_t resolution, const real_t topleft_x, const real_t topleft_y, map_t* map);
YMCL_EXPORT void map_fini(map_t* map);

