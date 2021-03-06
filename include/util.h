#pragma once
/*
* util.h
* author: Yuki Suga
* copyright: Yuki Suga, 2015
* license: GPLv3
*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



#define REAL_IS_DOUBLE 1
	typedef double real_t;

	typedef int32_t bool_t;


#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

#if WIN32

#ifdef ymcl_EXPORTS
#define YMCL_EXPORT __declspec(dllexport) // WIN32 && _WINDLL

#else
#ifdef _YMCL_USE_STATIC
#define YMCL_EXPORT // WIN32 && !_WINDLL && STATIC

#else
#define YMCL_EXPORT __declspec(dllimport) // WIN32 && !_WINDLL
#endif

#endif //_WINDLL

#else // WIN32

#define YMCL_EXPORT // !WIN32


#endif

#ifdef __cplusplus
}
#endif