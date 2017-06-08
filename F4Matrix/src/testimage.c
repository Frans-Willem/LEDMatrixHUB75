/*
 * testimage.c
 *
 *  Created on: 12 dec. 2014
 *      Author: Frans-Willem
 */

#include "framebuffer.h"
#include "colorcorr.h"

void testimage_set(unsigned int x, unsigned int y, uint8_t r, uint8_t g, uint8_t b) {
	unsigned int offset=(x + (y*MATRIX_WIDTH))*3;
	framebuffer_write(offset,colorcorr_lookup(r));
	framebuffer_write(offset+1,colorcorr_lookup(g));
	framebuffer_write(offset+2,colorcorr_lookup(b));
}

void testimage_setb(unsigned int x, unsigned int y, uint8_t* rgb) {
	unsigned int offset=(x + (y*MATRIX_WIDTH))*3;
	framebuffer_write(offset,colorcorr_lookup(rgb[0]));
	framebuffer_write(offset+1,colorcorr_lookup(rgb[1]));
	framebuffer_write(offset+2,colorcorr_lookup(rgb[2]));
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void testimage_init() {
	uint8_t topcolors[]={
		255,255,255,
		255,255,0,
		0,255,255,
		0,255,0,
		255,0,255,
		255,0,0,
		0,0,255
	};
	uint8_t barcolors[]={
		0,0,255,
		0,0,0,
		255,0,255,
		0,0,0,
		0,255,255,
		0,0,0,
		255,255,255
	};
	unsigned int x,y;
	for (x=0; x<MATRIX_WIDTH; x++) {
		for (y=0; y<MATRIX_HEIGHT; y++) {
			unsigned int seg = (y * 13)/MATRIX_HEIGHT;
			if (seg < 8) {
				unsigned int c=(x * ((sizeof(topcolors)/sizeof(topcolors[0]))/3))/MATRIX_WIDTH;
				testimage_setb(x,y,&topcolors[c*3]);
			} else if (seg == 8) {
				unsigned int c=(x * ((sizeof(barcolors)/sizeof(barcolors[0]))/3))/MATRIX_WIDTH;
				testimage_setb(x,y,&barcolors[c*3]);
			} else {
				unsigned int v = (x * 255)/MATRIX_WIDTH;
				unsigned int c = seg-9;
				testimage_set(x,y,(c == 0 || c == 3)?v:0,(c == 1 || c == 3)?v:0,(c == 2 || c == 3)?v:0);
			}
		}
	}
	framebuffer_swap();
}
