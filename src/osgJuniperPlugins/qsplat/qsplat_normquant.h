#ifndef QSPLAT_NORMQUANT_H
#define QSPLAT_NORMQUANT_H
/*
Szymon Rusinkiewicz

qsplat_normquant.h
Quantize a normal to 14 bits.

Layout:
	 7 6 5 4 3 2 1 0 7 6 5 4 3 2 1 0
	+---------------+---------------+
	|              N            | C |
	+---------------+---------------+
	     Byte 0     |    Byte 1

N is split up into A, U, V as follows:

	A = N / 2704
	U = (N / 52) % 52
	V = N % 52

A determines the axis to which the normal is closest:
	A = 0	Positive X	U = Y axis	V = Z axis
	    1	Negative X
	    2	Positive Y	U = Z axis	V = X axis
	    3	Negative Y
	    4	Positive Z	U = X axis	V = Y axis
	    5	Negative Z

The components of the normal along the other 2 axes are stored (nonlinearly)
quantized to 52 values each (U and V).

C is one of 4 values for the half-width of the cone enclosing all the normals
of the children of this node.


Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


#include "qsplat_util.h"

class QSplat_NormQuant {
private:
	static float *normquant_table;

public:
	static void Init();

	static void quantize(const float *norm, unsigned char *q);

	static inline const float *lookup(const unsigned char *q)
	{
		unsigned short index = * (const unsigned short *) q;
		FIX_SHORT(index);
		return normquant_table + 3 * (index >> 2);
	}

	static void quantize_cone(const float normcone, unsigned char *q)
	{
		float s = (normcone > 1.5f) ? 0.999f : sin(normcone);
		int x = int(4.0f*sqrtf(s));
		q[1] |= x;
	}

	static inline float lookup_cone(const unsigned char *q)
	{
		return -0.0625f * (1 + (q[1] & 3)) * (1 + (q[1] & 3));
	}
};

#endif
