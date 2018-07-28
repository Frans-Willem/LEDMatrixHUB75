/*
 * framebuffer.c
 *
 *  Created on: 11 dec. 2014
 *      Author: Frans-Willem
 */


#include "framebuffer.h"

//Current setup:
//For each bit
//	for each row
//		data to be clocked out
//New setup:
//For each row
//	for each bit
//		data to be clocked out

//Number of elements per row
//Number of elements per bit (e.g. a row for each scanrow)
#define FRAMEBUFFER_BITLEN	FRAMEBUFFER_SHIFTLEN
#define FRAMEBUFFER_ROWLEN	(FRAMEBUFFER_BITLEN * FRAMEBUFFER_MAXBITDEPTH)
//Length of one framebuffer
#define FRAMEBUFFER_LEN		(FRAMEBUFFER_ROWLEN * MATRIX_PANEL_SCANROWS)

FRAMEBUFFER_TYPE framebuffers[FRAMEBUFFER_LEN*FRAMEBUFFER_BUFFERS]={0};
volatile unsigned int framebuffer_writebuffer=0;
volatile unsigned int framebuffer_displaybuffer=0;

void framebuffer_init() {
	unsigned int i;
	for (i=0; i<FRAMEBUFFER_LEN*FRAMEBUFFER_BUFFERS; i++) {
		framebuffers[i]=((i&1)^FRAMEBUFFER_CLOCK_POLARITY)?1:0;
	}
	framebuffer_displaybuffer=0;
	framebuffer_writebuffer=(framebuffer_displaybuffer+1)%FRAMEBUFFER_BUFFERS;
}

void framebuffer_write(unsigned int offset, uint16_t value) {
	unsigned int channel = offset % MATRIX_PANEL_CHANNELS; offset/=MATRIX_PANEL_CHANNELS;
	// Desired X and Y pixel position
	unsigned int x = offset % MATRIX_WIDTH; offset/=MATRIX_WIDTH;
	unsigned int y = offset;
	if (y >= MATRIX_HEIGHT)
		return;
	// Panel position
	unsigned int y_panel = y / MATRIX_PANEL_HEIGHT;
	unsigned int x_panel = x / MATRIX_PANEL_WIDTH;
	// Convert X and Y to position within panel
	x %= MATRIX_PANEL_WIDTH;
	y %= MATRIX_PANEL_HEIGHT;
	// Scanrow and bus calculation
	unsigned int scanrow = y % MATRIX_PANEL_SCANROWS;
	unsigned int stack = (y / MATRIX_PANEL_SCANROWS) % MATRIX_PANEL_STACKED;
	unsigned int bus = (y / MATRIX_PANEL_SCANROWS / MATRIX_PANEL_STACKED) % MATRIX_PANEL_BUSES;
	unsigned int segment = (x / MATRIX_PANEL_SEGMENT_W);
	unsigned int panel_offset =
		((MATRIX_PANEL_STACKED - 1 - stack) * MATRIX_PANEL_SEGMENT_W) +
		(segment * MATRIX_PANEL_SEGMENT_W * MATRIX_PANEL_STACKED) +
		(x % MATRIX_PANEL_SEGMENT_W);
	unsigned int panel_index = x_panel + (y_panel * MATRIX_PANELSW);
	framebuffer_raw_write(panel_index, bus, scanrow, panel_offset, channel, value);
}

void framebuffer_raw_write(unsigned int panel, unsigned int bus, unsigned int scanrow, unsigned int offset, unsigned int channel, uint16_t value) {
	offset += panel * MATRIX_PANEL_WIDTH * MATRIX_PANEL_STACKED;
	offset *= 2;
	offset += (scanrow * FRAMEBUFFER_ROWLEN);
	FRAMEBUFFER_TYPE output = 1<<((bus*MATRIX_PANEL_CHANNELS) + channel + 1);
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

