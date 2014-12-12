/*
 * colorcorr.c
 *
 *  Created on: 12 dec. 2014
 *      Author: Frans-Willem
 */


#include "colorcorr.h"
#include "config.h"
#include <math.h>

uint16_t colorcorr_table[1 + COLORCORR_GAMMA_COUNT][256];
unsigned int colorcorr_current;

float colorcorr_lum2duty(double lum) {
	if (lum>0.07999591993063804) {
		return pow(((lum+0.16)/1.16),3.0);
	} else {
		return lum/9.033;
	}
}

void colorcorr_init_table_direct(uint16_t *table) {
	unsigned int i;
	for (i=0; i<256; i++)
		table[i]=(i*((1<<FRAMEBUFFER_MAXBITDEPTH)-1))/255;
}

void colorcorr_init_table_lumgamma(uint16_t *table, double gamma) {
	unsigned int i,oi;
	for (i=0; i<256; i++) {
		double di = colorcorr_lum2duty(((double)i)/255.0);
		di = pow(di, gamma);
		oi = (unsigned int)(di * (double)((1<<FRAMEBUFFER_MAXBITDEPTH)-1));
		if (oi > (1<<FRAMEBUFFER_MAXBITDEPTH)-1)
			oi = (1<<FRAMEBUFFER_MAXBITDEPTH)-1;
		table[i] = oi;
	}
}

void colorcorr_init() {
	colorcorr_init_table_direct(colorcorr_table[0]);
	unsigned int i=0;
	for (i=0; i<=COLORCORR_GAMMA_COUNT; i++) {
		colorcorr_init_table_lumgamma(colorcorr_table[i+1],COLORCORR_GAMMA_MIN + (COLORCORR_GAMMA_STEP*i));
	}
#ifdef COLORCORR_GAMMA_DEFAULT
	colorcorr_current = 1+COLORCORR_GAMMA_DEFAULT;
#else
	colorcorr_current = 0;
#endif
}

uint16_t colorcorr_lookup(uint8_t v) {
	return colorcorr_table[colorcorr_current][v];
}
