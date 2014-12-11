/*
 * framebuffer.h
 *
 *  Created on: 11 dec. 2014
 *      Author: Frans-Willem
 */

#ifndef FRAMEBUFFER_H_
#define FRAMEBUFFER_H_
#include <stdint.h>
#include "config.h"

//Number of bits to clock out each time
#define FRAMEBUFFER_BITSPERROW	(MATRIX_PANEL_WIDTH*MATRIX_PANELSW*MATRIX_PANELSH)
#define FRAMEBUFFER_ROWLEN	(FRAMEBUFFER_BITSPERROW * 2)
#define FRAMEBUFFER_CLOCK	(1<<(MATRIX_PANEL_CHANNELS*MATRIX_PANEL_BUSES))

#if (FRAMEBUFFER_CLOCK > 0xFFFF)
	#error "Unable to fit enough channels, buses, and clock into one GPIO"
#elif (FRAMEBUFFER_CLOCK > 0xFF)
	#define FRAMEBUFFER_TYPE	uint16_t
#else
	#define FRAMEBUFFER_TYPE	uint8_t
#endif

void framebuffer_init();
void framebuffer_write(unsigned int offset, uint16_t value);
FRAMEBUFFER_TYPE *framebuffer_get();
void framebuffer_swap();
void framebuffer_sync();
#endif /* FRAMEBUFFER_H_ */
