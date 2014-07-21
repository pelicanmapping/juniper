#ifndef QSPLAT_COLORQUANT_H
#define QSPLAT_COLORQUANT_H
/*
Szymon Rusinkiewicz

qsplat_colorquant.h
Quantize a color to 16 bits, 565.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


class QSplat_ColorQuant {
private:
	static float *colorquant_table;

public:
	static void Init();

	static void quantize(const unsigned char *rgb, unsigned char *q);

	static inline const float *lookup(const unsigned char *q)
	{
		unsigned index = (unsigned(*q) << 8) | unsigned(*(q+1));
		return colorquant_table + 3*index;
	}
};

#endif
