/*
* random.cpp
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include "random.h"
#include "random_generator.h"

#include <math.h>

static double __q__;

YMCL_EXPORT void random_init() {
	unsigned long init[4] = { 0x123, 0x234, 0x345, 0x456 }, length = 4;

	__q__ = sqrt(6.0);
	init_by_array(init, length);
}

YMCL_EXPORT real_t random_uniform()
{
	return genrand_real3();
}


YMCL_EXPORT double prob_normal_distribution(const double a, const double sigma) {
	return /*1 / (sigma * sqrt(2 * M_PI*sigma)) * */ exp(-(a*a / (2 * (sigma * sigma))));
}


YMCL_EXPORT double prob_triangular_distribution(const double a, const double sigma) {
	double d = 1 / (__q__ * sigma) - fabs(a) / (6 * sigma);
	return d > 0 ? d : 0;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
real_t random_gaussian(real_t sigma)
{
	real_t x1, x2, w;

	do
	{
		x1 = 2.0 * random_uniform() - 1.0;
		x2 = 2.0 * random_uniform() - 1.0;
		w = x1*x1 + x2*x2;
	} while (w > 1.0 || w == 0.0);

	return(sigma * x2 * sqrt(-2.0*log(w) / w));
}
