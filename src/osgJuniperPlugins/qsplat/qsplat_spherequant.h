#ifndef QSPLAT_SPHEREQUANT_H
#define QSPLAT_SPHEREQUANT_H
/*
Szymon Rusinkiewicz

qsplat_spherequant.h
Encode the difference between a sphere and its parent in 13 bits.

Layout:
	 7 6 5 4 3 2 1 0 7 6 5 4 3 2 1 0
	+---------------+---------------+
	|            R            |  X  |
	+---------------+---------------+
	     Byte 0     |    Byte 1

R is an index into a lookup table that describes the position of the center and
the radius of a sphere, relative to its parent sphere.
X in the above diagram is ignored, and is used for other purposes.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_util.h"


class QSplat_SphereQuant {
private:
	static float *spherequant_table;
	static unsigned short *spherequant_enc_table;

public:
	static void Init();

	static void quantize(float pcx, float pcy, float pcz, float pr,
			     float cx, float cy, float cz, float r,
			     unsigned char *q);

	static inline void lookup(const unsigned char *q,
				  float pcx, float pcy, float pcz, float pr,
				  float &mycx, float &mycy, float &mycz, float &myr)
	{
		unsigned short index = * (const unsigned short *)q;
		FIX_SHORT(index);
		float *r = (float *) (
				(const unsigned char *)spherequant_table +
				((unsigned(index) << 1) & 0x1fff0u));
		mycx = pcx + pr * (*r++);
		mycy = pcy + pr * (*r++);
		mycz = pcz + pr * (*r++);
		myr  =       pr * (*r);
	}
};

#endif
