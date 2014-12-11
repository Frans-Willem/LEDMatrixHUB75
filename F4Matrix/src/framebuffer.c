/*
 * framebuffer.c
 *
 *  Created on: 11 dec. 2014
 *      Author: Frans-Willem
 */


#include "framebuffer.h"

//Number of elements per row
//Number of elements per bit (e.g. a row for each scanrow)
#define FRAMEBUFFER_BITLEN	(FRAMEBUFFER_ROWLEN * MATRIX_PANEL_SCANROWS)
//Length of one framebuffer
#define FRAMEBUFFER_LEN		(FRAMEBUFFER_BITLEN * FRAMEBUFFER_MAXBITDEPTH)

FRAMEBUFFER_TYPE framebuffers[FRAMEBUFFER_LEN*FRAMEBUFFER_BUFFERS]={0};
volatile unsigned int framebuffer_writebuffer=0;
volatile unsigned int framebuffer_displaybuffer=0;

void framebuffer_init() {
	unsigned int i;
	for (i=0; i<FRAMEBUFFER_LEN*FRAMEBUFFER_BUFFERS; i++) {
		framebuffers[i]=(i&1)?FRAMEBUFFER_CLOCK:0;
	}
	framebuffer_displaybuffer=0;
	framebuffer_writebuffer=(framebuffer_displaybuffer+1)%FRAMEBUFFER_BUFFERS;
}

void framebuffer_write(unsigned int offset, uint16_t value) {
	unsigned int channel = offset % MATRIX_PANEL_CHANNELS; offset/=MATRIX_PANEL_CHANNELS;
	unsigned int x = offset % MATRIX_WIDTH; offset/=MATRIX_WIDTH;
	unsigned int y = offset;
	if (y >= MATRIX_HEIGHT)
		return;
	unsigned int segment = y / MATRIX_PANEL_HEIGHT;
	unsigned int scanrow = y % MATRIX_PANEL_SCANROWS;
	unsigned int bus = (y / MATRIX_PANEL_SCANROWS) % MATRIX_PANEL_BUSES;
	//Every odd segment (line of panels) is inverted (snake-like configuration)
	if (segment & 1) {
		x = (MATRIX_WIDTH-1)-x;
		scanrow = (MATRIX_PANEL_SCANROWS-1)-scanrow;
		bus = (MATRIX_PANEL_BUSES-1)-bus;
	}
	offset = (scanrow * FRAMEBUFFER_ROWLEN) + ((MATRIX_PANELSH-1-segment) * MATRIX_PANELSW * MATRIX_PANEL_WIDTH * 2) + (x * 2);
	FRAMEBUFFER_TYPE output = 1<<((bus*MATRIX_PANEL_CHANNELS) + channel);
	FRAMEBUFFER_TYPE *ptr = &framebuffers[(framebuffer_writebuffer * FRAMEBUFFER_LEN)+offset];
	unsigned int bit;
	for (bit = (1 << (FRAMEBUFFER_MAXBITDEPTH-1)); bit; bit>>=1) {
		if (value & bit) {
			ptr[0]|=output;
			ptr[1]|=output;
		} else {
			ptr[0]&=~output;
			ptr[1]&=~output;
		}
		ptr = &ptr[FRAMEBUFFER_BITLEN];
	}
}
FRAMEBUFFER_TYPE *framebuffer_get() {
	return &framebuffers[framebuffer_displaybuffer * FRAMEBUFFER_LEN];
}

void framebuffer_swap() {
	framebuffer_displaybuffer=framebuffer_writebuffer;
	framebuffer_writebuffer=(framebuffer_displaybuffer+1)%FRAMEBUFFER_BUFFERS;
}

void framebuffer_sync() {
	//TODO: Implement waiting for DMA to actually display framebuffer
}
