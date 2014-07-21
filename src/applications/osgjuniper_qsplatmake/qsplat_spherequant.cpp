/*
Szymon Rusinkiewicz

qsplat_spherequant.cpp
Encode the difference between a sphere and its parent in 13 bits.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include "qsplat_util.h"
#include "qsplat_spherequant.h"

#define N 13
#define INVALID 0xffff
#undef PARANOIA


// Class static variables
float *QSplat_SphereQuant::spherequant_table = NULL;
unsigned short *QSplat_SphereQuant::spherequant_enc_table = NULL;


// Set up the quantization tables
void QSplat_SphereQuant::Init()
{
	if (spherequant_table)
		return;

	static const float oneNth = 1.0f / float(N);
	static const float sqrt3 = sqrtf(3.0f);
	int i;

#ifdef WIN32
	spherequant_table = (float *) GlobalAlloc(GMEM_FIXED,
					sizeof(float) * 4 * (1 << 13));
	spherequant_enc_table = (unsigned short *) GlobalAlloc(GMEM_FIXED,
					sizeof(unsigned short) * N * N * N * N);
#else
	spherequant_table = new float[4 * (1 << 13)];
	spherequant_enc_table = new unsigned short[N * N * N * N];
#endif

	float *tbl = spherequant_table;
	for (i = 0; i < N*N*N*N; i++)
		spherequant_enc_table[i] = INVALID;

	i = 0;
	for (int size = 1; size <= N; size++) {
		for (int x = 0; x < N+1-size; x++)
		  for (int y = 0; y < N+1-size; y++)
		    for (int z = 0; z < N+1-size; z++) {
			float X = (size+2*x)*oneNth - 1.0f;
			float Y = (size+2*y)*oneNth - 1.0f;
			float Z = (size+2*z)*oneNth - 1.0f;
			float R = size*oneNth;
			if (X*X + Y*Y + Z*Z > sqr(1.0f+sqrt3*R))
				continue;
			*tbl++ = X;
			*tbl++ = Y;
			*tbl++ = Z;
			*tbl++ = R;
			spherequant_enc_table[size-1+N*(x+N*(y+N*z))] = i++;
		    }
	}
}


// Quantize a sphere's location and size relative to its parent sphere.
void QSplat_SphereQuant::quantize(float pcx, float pcy, float pcz, float pr,
				  float cx, float cy, float cz, float r,
				  unsigned char *q)
{
	float dx = cx - pcx;
	float dy = cy - pcy;
	float dz = cz - pcz;

	float rdx = 0.5f * (dx / pr + 1.0f);
	float rdy = 0.5f * (dy / pr + 1.0f);
	float rdz = 0.5f * (dz / pr + 1.0f);
	float rr = r / pr;

	int X1 = min(max(int(floor((rdx - 0.5f*rr)*N)), 0), N-1);
	int Y1 = min(max(int(floor((rdy - 0.5f*rr)*N)), 0), N-1);
	int Z1 = min(max(int(floor((rdz - 0.5f*rr)*N)), 0), N-1);
	int X2 = min(max(int(ceil((rdx + 0.5f*rr)*N)), 1), N);
	int Y2 = min(max(int(ceil((rdy + 0.5f*rr)*N)), 1), N);
	int Z2 = min(max(int(ceil((rdz + 0.5f*rr)*N)), 1), N);

	int maxdiff = max(max(X2 - X1, Y2 - Y1), Z2 - Z1);
	if (X2 - X1 < maxdiff) {
		// XXX - FIXME
		if (X1) X1--;
		else X2++;
	}
	if (Y2 - Y1 < maxdiff) {
		if (Y1) Y1--;
		else Y2++;
	}
	if (Z2 - Z1 < maxdiff) {
		if (Z1) Z1--;
		else Z2++;
	}

	unsigned short R = spherequant_enc_table[maxdiff-1+N*(X1+N*(Y1+N*Z1))];
#ifdef PARANOIA
	if (R == INVALID) {
		printf("Badness in quantize(): r = %d, pos = %d %d %d\n",
			maxdiff, X1, Y1, Z1);
		R = 0;
	}
#endif

	(* q   ) = R >> 5;
	(*(q+1)) = (R << 3) & 0xf8;
}

