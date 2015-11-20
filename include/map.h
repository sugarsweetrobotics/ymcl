#pragma once
/*
* map.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/
#include "util.h"
#ifdef __cplusplus
extern "C" {
#endif

	typedef enum map_cell_t_ {
		MAP_OCCUPIED = 0,
		MAP_EMPTY = 255,
		MAP_UNKNOWN = 127,
	} map_cell_t;

	typedef struct _map_param_struct {
		uint32_t pixel_width;
		uint32_t pixel_height;
		real_t resolution;
		real_t topleft_x;
		real_t topleft_y;
	} map_param_t;


	typedef struct map_struct_ {
		map_cell_t* cell;

		map_param_t param;

		uint32_t origin_index_x;
		uint32_t origin_index_y;
		
	} map_t;


#include "pose.h"

	YMCL_EXPORT void pose_to_cell(const pose_t* pose, const map_t* map, int32_t *x, int32_t* y);

	static bool_t map_occupied(const map_t* map, uint32_t x, uint32_t y) {
		return map->cell[y * map->param.pixel_width + x] < 100 ? TRUE : FALSE;
	}

	static bool_t map_empty(const map_t* map, uint32_t x, uint32_t y) {
		return map->cell[y * map->param.pixel_width + x] > 200 ? TRUE : FALSE;
	}

	static bool_t map_unknown(const map_t* map, uint32_t x, uint32_t y) {
		return map->cell[y * map->param.pixel_width + x] <= 200 && map->cell[y * map->param.pixel_width + x] >= 100 ? TRUE : FALSE;
	}

	static bool_t map_validate_index(const map_t* map, const int x, const int y) {
		return (x >= 0 && y >= 0 && (uint32_t)x < map->param.pixel_width && (uint32_t)y < map->param.pixel_height);
	}

	YMCL_EXPORT void map_init(map_t* map, const map_param_t *param);
	YMCL_EXPORT void map_fini(map_t* map);


#ifdef __cplusplus
}
#endif