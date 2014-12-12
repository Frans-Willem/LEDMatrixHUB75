/*
 * colorcorr.h
 *
 *  Created on: 12 dec. 2014
 *      Author: Frans-Willem
 */

#ifndef COLORCORR_H_
#define COLORCORR_H_
#include <stdint.h>

void colorcorr_init();
uint16_t colorcorr_lookup(uint8_t v);

#endif /* COLORCORR_H_ */
