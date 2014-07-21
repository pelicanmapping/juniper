/*
Szymon Rusinkiewicz

qsplat_normquant.cpp
Quantize a normal.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


#include "qsplat_normquant.h"
#include "qsplat_util.h"


// Because we quantize normals so severely, there are often banding artifacts
// in regions of very low curvature.  To try to minimize the effect of this,
// we dither the normals by adding a bit of random noise before we quantize
#define DITHER_NORMALS


#define N 52


// Defines warping so that quantized normals cover sphere more evenly
#define WARP(x) (min(max(float(atan(x)*4.0f/M_PI), -1.0f), 1.0f))
#define UNWARP(x) (min(max(float(tan((x)*M_PI*0.25f)), -1.0f), 1.0f))


// Class static variables
float *QSplat_NormQuant::normquant_table = NULL;


// Set up the quantization tables
void QSplat_NormQuant::Init()
{
	if (normquant_table)
		return;

	normquant_table = new float[3*16384];
	static const float twoNths = 2.0f / (float)N;

	int index = 0;
	for (unsigned y = 0; y < N; y++) {
	  for (unsigned z = 0; z < N; z++) {
		float ny = UNWARP((((float)y + 0.5f) * twoNths) - 1.0f);
		float nz = UNWARP((((float)z + 0.5f) * twoNths) - 1.0f);
		float nx = sqrtf(1.0f-sqr(ny)-sqr(nz));
		normquant_table[index++] = nx;
		normquant_table[index++] = ny;
		normquant_table[index++] = nz;
	  }
	}
	int i;
	for (i=0; i < N*N; i++) {
		normquant_table[index++] = -normquant_table[3*i  ];
		normquant_table[index++] =  normquant_table[3*i+1];
		normquant_table[index++] =  normquant_table[3*i+2];
	}

	for (unsigned z = 0; z < N; z++) {
	  for (unsigned x = 0; x < N; x++) {
		float nz = UNWARP((((float)z + 0.5f) * twoNths) - 1.0f);
		float nx = UNWARP((((float)x + 0.5f) * twoNths) - 1.0f);
		float ny = sqrtf(1.0f-sqr(nz)-sqr(nx));
		normquant_table[index++] = nx;
		normquant_table[index++] = ny;
		normquant_table[index++] = nz;
	  }
	}
	for (i=N*N*2; i < N*N*3; i++) {
		normquant_table[index++] =  normquant_table[3*i  ];
		normquant_table[index++] = -normquant_table[3*i+1];
		normquant_table[index++] =  normquant_table[3*i+2];
	}

	for (unsigned x = 0; x < N; x++) {
	  for (unsigned y = 0; y < N; y++) {
		float nx = UNWARP((((float)x + 0.5f) * twoNths) - 1.0f);
		float ny = UNWARP((((float)y + 0.5f) * twoNths) - 1.0f);
		float nz = sqrtf(1.0f-sqr(nx)-sqr(ny));
		normquant_table[index++] = nx;
		normquant_table[index++] = ny;
		normquant_table[index++] = nz;
	  }
	}
	for (i=N*N*4; i < N*N*5; i++) {
		normquant_table[index++] =  normquant_table[3*i  ];
		normquant_table[index++] =  normquant_table[3*i+1];
		normquant_table[index++] = -normquant_table[3*i+2];
	}
	
	for (i=N*N*6; i < 16384; i++) {
		normquant_table[index++] = 0.0f;
		normquant_table[index++] = 0.0f;
		normquant_table[index++] = 1.0f;
	}
}


// Quantize a normal
void QSplat_NormQuant::quantize(const float *norm, unsigned char *q)
{
	static const float twoNths = 2.0f / (float)N;

	float x = norm[0];
	float y = norm[1];
	float z = norm[2];

#ifdef DITHER_NORMALS
	x += ((float)rand() / float(RAND_MAX) - 0.5f) * twoNths;
	y += ((float)rand() / float(RAND_MAX) - 0.5f) * twoNths;
	z += ((float)rand() / float(RAND_MAX) - 0.5f) * twoNths;

	float l = 1.0f / sqrtf(sqr(x) + sqr(y) + sqr(z));
	x *= l;
	y *= l;
	z *= l;
#endif

	float ax = ABS(x);
	float ay = ABS(y);
	float az = ABS(z);

	unsigned A;
	float u;
	float v;
	if (ax > ay) {
		if (ax > az) {
			if (x > 0) {
				A = 0; u = y; v = z;
			} else {
				A = 1; u = y; v = z;
			}
		} else {
			if (z > 0) {
				A = 4; u = x; v = y;
			} else {
				A = 5; u = x; v = y;
			}
		}
	} else {
		if (ay > az) {
			if (y > 0) {
				A = 2; u = z; v = x;
			} else {
				A = 3; u = z; v = x;
			}
		} else {
			if (z > 0) {
				A = 4; u = x; v = y;
			} else {
				A = 5; u = x; v = y;
			}
		}

	}

	u = WARP(u);
	v = WARP(v);
	unsigned U = min(unsigned((u + 1.0f) * 0.5f * N), unsigned(N-1));
	unsigned V = min(unsigned((v + 1.0f) * 0.5f * N), unsigned(N-1));

	unsigned Q = ((A * N) + U) * N + V;

	(* q   ) = (Q >> 6);
	(*(q+1)) = (Q << 2) & 0xff;
}

