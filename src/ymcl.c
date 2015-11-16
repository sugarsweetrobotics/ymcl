
/*
* pf.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/


#include "ymcl.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <string.h> // for memcpy

real_t random_gaussian(real_t sigma);
double prob_normal_distribution(const double a, const double sigma);

void show_range(const pose_t* pose, const map_t* map, const ranger_data_t* ranger);

