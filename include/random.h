#pragma once
/*
* random.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "util.h"

YMCL_EXPORT void random_init();
YMCL_EXPORT real_t random_uniform();
YMCL_EXPORT real_t random_gaussian(real_t sigma);
YMCL_EXPORT double prob_normal_distribution(const double a, const double sigma);
YMCL_EXPORT double prob_triangular_distribution(const double a, const double sigma);
