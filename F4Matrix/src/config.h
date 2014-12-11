/*
 * config.h
 *
 *  Created on: 11 dec. 2014
 *      Author: Frans-Willem
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//Information about individual panels
//How many pixels in width?
#define MATRIX_PANEL_WIDTH			32
//Number of scanrows. Should be a power of two.
//The panels I use (16 pixels high) are 1/8th scan.
#define MATRIX_PANEL_SCANROWS		8
//How many buses and channels there are on the panel. For example on mine there's R1 G1 B1, R2 G2 B2, which means 3 channels (RGB), 2 buses (1,2)
#define MATRIX_PANEL_CHANNELS		3
#define MATRIX_PANEL_BUSES			2

//How many panels stuck together?
#define MATRIX_PANELSW	3
#define MATRIX_PANELSH	3

//Output stuff
#define FRAMEBUFFER_MAXBITDEPTH			10
#define FRAMEBUFFER_BUFFERS				2

//Calculated
#define MATRIX_PANEL_HEIGHT (MATRIX_PANEL_SCANROWS * MATRIX_PANEL_BUSES)
#define MATRIX_WIDTH		(MATRIX_PANEL_WIDTH*MATRIX_PANELSW)
#define MATRIX_HEIGHT		(MATRIX_PANEL_HEIGHT*MATRIX_PANELSH)



#endif /* CONFIG_H_ */
