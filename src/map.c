/*
* map.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "map.h"
#include <math.h>
#include <stdlib.h>

void map_init(map_t* map, const map_param_t *param) {
	map->cell = (map_cell_t*)malloc(sizeof(map_cell_t) * param->pixel_width * param->pixel_height);
	map->param = *param;

	map->origin_index_x = (uint32_t)((-map->param.topleft_x) / map->param.resolution);
	map->origin_index_y = (uint32_t)((+map->param.topleft_y) / map->param.resolution);

}


void map_fini(map_t* map)
{
	free(map->cell);
}

void pose_to_cell(const pose_t* pose, const map_t* map, int32_t *x, int32_t* y) {
	*x = lround(pose->x / map->param.resolution) + map->origin_index_x;
	*y = -lround(pose->y / map->param.resolution) + map->origin_index_y;
}