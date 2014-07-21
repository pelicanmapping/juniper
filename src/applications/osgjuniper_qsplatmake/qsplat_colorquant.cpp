/*
Szymon Rusinkiewicz

qsplat_colorquant.cpp
Quantize a color to 16 bits, 565.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


#include <stdio.h>
#include "qsplat_colorquant.h"


// Class static variables
float *QSplat_ColorQuant::colorquant_table = NULL;


// Set up the quantization tables
void QSplat_ColorQuant::Init()
{
	if (colorquant_table)
		return;

	colorquant_table = new float[3*65536];

	int index = 0;
	for (unsigned r = 0; r < 32; r++) {
	  for (unsigned g = 0; g < 64; g++) {
	    for (unsigned b = 0; b < 32; b++) {
		colorquant_table[index++] = float((r << 3) | (r >> 2)) / 255.0f;
		colorquant_table[index++] = float((g << 2) | (g >> 4)) / 255.0f;
		colorquant_table[index++] = float((b << 3) | (b >> 2)) / 255.0f;
	    }
	  }
	}
}


// Quantize a color
void QSplat_ColorQuant::quantize(const unsigned char *rgb, unsigned char *q)
{
        unsigned R = (unsigned) (0.125f * (* rgb   ));
        unsigned G = (unsigned) (0.25f  * (*(rgb+1)));
        unsigned B = (unsigned) (0.125f * (*(rgb+2)));
        
        (* q   ) = ((R << 3) | (G >> 3)) & 0xff;
        (*(q+1)) = ((G << 5) | (B     )) & 0xff;
}

