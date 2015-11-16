/*
* map.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "map.h"
#include <math.h>
#include <stdlib.h>

void map_init(const uint32_t pixel_width, const uint32_t pixel_height, const real_t resolution, const real_t topleft_x, const real_t topleft_y, map_t* map) {
	//map->cell = new uint8_t[pixel_width*pixel_height];
	map->cell = (uint8_t*)malloc(sizeof(uint8_t) * pixel_width * pixel_height);
	map->pixel_width = pixel_width;
	map->pixel_height = pixel_height;
	map->topleft_x = topleft_x;
	map->topleft_y = topleft_y;
	map->resolution = resolution;

	map->origin_index_x = (-map->topleft_x) / map->resolution;
	map->origin_index_y = (+map->topleft_y) / map->resolution;

	map->distance_cell = NULL;
	map->max_distance = 0.00001;
}


void map_fini(map_t* map)
{
	free(map->distance_cell);
	free(map->cell);
//	delete map->distance_cell;
//	delete map->cell;
}

void pose_to_cell(const pose_t* pose, const map_t* map, int32_t *x, int32_t* y) {
	*x = llroundf(pose->x / map->resolution) + map->origin_index_x;
	*y = -llroundf(pose->y / map->resolution) + map->origin_index_y;
}