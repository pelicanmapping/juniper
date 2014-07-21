#ifndef QSPLAT_UTIL_H
#define QSPLAT_UTIL_H
/*
Szymon Rusinkiewicz

qsplat_util.h
Various little utility functions...

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


// Standard includes we'll want anyway
#include <math.h>
#include <algorithm>
#include <utility>
using std::swap;

#ifdef WIN32
# include <limits.h>
# include <windows.h>
# define strncasecmp strnicmp
# define usleep(x) Sleep((x)/1000)
#else
# include <sys/time.h>
# include <unistd.h>
  using std::min;
  using std::max;
#endif


// Endianness stuff
#if defined(i386) || defined(WIN32)
#define WE_ARE_LITTLE_ENDIAN
#endif

#define SWAP_SHORT(x) ( (((x) & 0xff) << 8) | ((unsigned short)(x) >> 8) )
#define SWAP_LONG(x) ( ((x) << 24) | \
                       (((x) << 8) & 0x00ff0000) | \
		       (((x) >> 8) & 0x0000ff00) | \
		       ((x) >> 24) )

#ifdef WE_ARE_LITTLE_ENDIAN
# define FIX_SHORT(x) (*(unsigned short *)&(x) = \
			SWAP_SHORT(*(unsigned short *)&(x)))
# define FIX_LONG(x) (*(unsigned *)&(x) = \
			SWAP_LONG(*(unsigned *)&(x)))
# define FIX_FLOAT(x) FIX_LONG(x)
#else
# define FIX_SHORT(x) do {} while (0)
# define FIX_LONG(x) do {} while (0)
# define FIX_FLOAT(x) do {} while (0)
#endif


// Unaligned accesses
#ifdef mips
#define NEED_ALIGNED_ACCESSES
#endif

#ifdef NEED_ALIGNED_ACCESSES
// XXXXXX - assumes p has 2-byte alignment, and assumes big endian
#define UNALIGNED_DEREFERENCE_INT(p) (int( \
	(*((unsigned short *)(p)) << 16) | *(((unsigned short *)(p))+1)))
#else
#define UNALIGNED_DEREFERENCE_INT(p) ( * (int *)(p) )
#endif


// Time...
#ifdef WIN32

typedef LARGE_INTEGER timestamp;

static inline float LI2f(const LARGE_INTEGER &li)
{
	// Workaround for compiler bug.  Sigh.
	float f = unsigned(li.u.HighPart) >> 16;  f *= 65536.0f;
	f += unsigned(li.u.HighPart) & 0xffff;    f *= 65536.0f;
	f += unsigned(li.u.LowPart) >> 16;        f *= 65536.0f;
	f += unsigned(li.u.LowPart) & 0xffff;
	return f;
}

static inline float operator - (const timestamp &t1, const timestamp &t2)
{
	static LARGE_INTEGER PerformanceFrequency;
	static int status = QueryPerformanceFrequency(&PerformanceFrequency);
	if (status == 0) return 1.0f;

	return (LI2f(t1) - LI2f(t2)) / LI2f(PerformanceFrequency);
}

static inline void get_timestamp(timestamp &now)
{
	QueryPerformanceCounter(&now);
}

#else

typedef struct timeval timestamp;

static inline float operator - (const timestamp &t1, const timestamp &t2)
{
	return (float)(t1.tv_sec  - t2.tv_sec) +
	       1.0e-6f*(t1.tv_usec - t2.tv_usec);
}

static inline void get_timestamp(timestamp &now)
{
	gettimeofday(&now, 0);
}

#endif


// Many platforms don't have sqrtf
#if !defined(sgi) && !defined(linux)
#define sqrtf(x) sqrt(x)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// A few gratuitous typedefs
typedef unsigned char color[3];
typedef float point[3];
typedef float vec[3];
typedef int face[3];
typedef float quat[4];


// Misc little functions
template <class T>
static inline T sqr(const T &x)
{
	return x*x;  
}

template <class T>
static inline T ABS(const T &x)
{
	if (x < T(0))
		return -x;
	else
		return x;
}


static inline float Len(const float *x)
{
	return sqrtf(sqr(x[0]) + sqr(x[1]) + sqr(x[2]));
}

static inline float Dist(const float *x1, const float *x2)
{
	float dx = x2[0] - x1[0];
	float dy = x2[1] - x1[1];
	float dz = x2[2] - x1[2];

	return sqrtf(sqr(dx) + sqr(dy) + sqr(dz));
}

static inline void Normalize(float *v)
{
	float x = Len(v);
	if (x == 0.0f) {
		v[0] = v[1] = 0.0f;
		v[2] = 1.0f;
		return;
	}

	x = 1.0f / x;
	v[0] *= x; v[1] *= x; v[2] *= x;
}

static inline void CrossProd(const float *x, const float *y, float *z)
{
	z[0] = x[1]*y[2] - x[2]*y[1];
	z[1] = x[2]*y[0] - x[0]*y[2];
	z[2] = x[0]*y[1] - x[1]*y[0];
}

static inline float Dot(const float *x, const float *y)
{
	return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}

static inline void MMult(const float *M1, const float *M2, float *Mout)
{
	Mout[ 0] = M1[ 0]*M2[ 0]+M1[ 4]*M2[ 1]+M1[ 8]*M2[ 2]+M1[12]*M2[ 3];
	Mout[ 1] = M1[ 1]*M2[ 0]+M1[ 5]*M2[ 1]+M1[ 9]*M2[ 2]+M1[13]*M2[ 3];
	Mout[ 2] = M1[ 2]*M2[ 0]+M1[ 6]*M2[ 1]+M1[10]*M2[ 2]+M1[14]*M2[ 3];
	Mout[ 3] = M1[ 3]*M2[ 0]+M1[ 7]*M2[ 1]+M1[11]*M2[ 2]+M1[15]*M2[ 3];
	Mout[ 4] = M1[ 0]*M2[ 4]+M1[ 4]*M2[ 5]+M1[ 8]*M2[ 6]+M1[12]*M2[ 7];
	Mout[ 5] = M1[ 1]*M2[ 4]+M1[ 5]*M2[ 5]+M1[ 9]*M2[ 6]+M1[13]*M2[ 7];
	Mout[ 6] = M1[ 2]*M2[ 4]+M1[ 6]*M2[ 5]+M1[10]*M2[ 6]+M1[14]*M2[ 7];
	Mout[ 7] = M1[ 3]*M2[ 4]+M1[ 7]*M2[ 5]+M1[11]*M2[ 6]+M1[15]*M2[ 7];
	Mout[ 8] = M1[ 0]*M2[ 8]+M1[ 4]*M2[ 9]+M1[ 8]*M2[10]+M1[12]*M2[11];
	Mout[ 9] = M1[ 1]*M2[ 8]+M1[ 5]*M2[ 9]+M1[ 9]*M2[10]+M1[13]*M2[11];
	Mout[10] = M1[ 2]*M2[ 8]+M1[ 6]*M2[ 9]+M1[10]*M2[10]+M1[14]*M2[11];
	Mout[11] = M1[ 3]*M2[ 8]+M1[ 7]*M2[ 9]+M1[11]*M2[10]+M1[15]*M2[11];
	Mout[12] = M1[ 0]*M2[12]+M1[ 4]*M2[13]+M1[ 8]*M2[14]+M1[12]*M2[15];
	Mout[13] = M1[ 1]*M2[12]+M1[ 5]*M2[13]+M1[ 9]*M2[14]+M1[13]*M2[15];
	Mout[14] = M1[ 2]*M2[12]+M1[ 6]*M2[13]+M1[10]*M2[14]+M1[14]*M2[15];
	Mout[15] = M1[ 3]*M2[12]+M1[ 7]*M2[13]+M1[11]*M2[14]+M1[15]*M2[15];
}

static inline void FastProjectPrecompute(const float *P,
					 const float *M,
					 const float *V,
					 float *F,
					 float zoffset=0)
{
	float tmp[16];
	memcpy(tmp, M, 16*sizeof(float));
	if (zoffset) {
		tmp[ 2] += zoffset * tmp[ 3];
		tmp[ 6] += zoffset * tmp[ 7];
		tmp[10] += zoffset * tmp[11];
		tmp[14] += zoffset * tmp[15];
	}

	MMult(P, tmp, F);

	F[ 0] = 0.5f*V[2] * (F[ 0]+F[ 3]) + V[0] * F[ 3];
	F[ 4] = 0.5f*V[2] * (F[ 4]+F[ 7]) + V[0] * F[ 7];
	F[ 8] = 0.5f*V[2] * (F[ 8]+F[11]) + V[0] * F[11];
	F[12] = 0.5f*V[2] * (F[12]+F[15]) + V[0] * F[15];
	F[ 1] = 0.5f*V[3] * (F[ 1]+F[ 3]) + V[1] * F[ 3];
	F[ 5] = 0.5f*V[3] * (F[ 5]+F[ 7]) + V[1] * F[ 7];
	F[ 9] = 0.5f*V[3] * (F[ 9]+F[11]) + V[1] * F[11];
	F[13] = 0.5f*V[3] * (F[13]+F[15]) + V[1] * F[15];
	F[ 2] = 0.5f      * (F[ 2]+F[ 3]);
	F[ 6] = 0.5f      * (F[ 6]+F[ 7]);
	F[10] = 0.5f      * (F[10]+F[11]);
	F[14] = 0.5f      * (F[14]+F[15]);
}

static inline void FastProject(const float *F,
			       float x, float y, float z,
			       float &xout, float &yout, float &zout)
{
	float w_rec = 1.0f  / (F[ 3]*x + F[ 7]*y + F[11]*z + F[15]);
	xout        = w_rec * (F[ 0]*x + F[ 4]*y + F[ 8]*z + F[12]);
	yout        = w_rec * (F[ 1]*x + F[ 5]*y + F[ 9]*z + F[13]);
	zout        = w_rec * (F[ 2]*x + F[ 6]*y + F[10]*z + F[14]);
}


extern const char *QSPLAT_VERSION;

#endif
