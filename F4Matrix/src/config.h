/*
 * config.h
 *
 *  Created on: 11 dec. 2014
 *      Author: Frans-Willem
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//Information about individual panels
// How many pixels in width?
#define MATRIX_PANEL_WIDTH			32
#define MATRIX_PANEL_HEIGHT			32	
// How many segments ? when in doubt, set to 1.
#define MATRIX_PANEL_SEGMENTS		4
//Number of scanrows. Should be a power of two.
//The panels I use (16 pixels high) are 1/8th scan.
#define MATRIX_PANEL_SCANROWS		4
//How many buses and channels there are on the panel. For example on mine there's R1 G1 B1, R2 G2 B2, which means 3 channels (RGB), 2 buses (1,2)
#define MATRIX_PANEL_CHANNELS		3
#define MATRIX_PANEL_BUSES			4

//How many panels stuck together?
#define MATRIX_PANELSW	3
#define MATRIX_PANELSH	2

//Output stuff
#define FRAMEBUFFER_MAXBITDEPTH			11
#define FRAMEBUFFER_BUFFERS				1
#define MATRIX_MINIMUM_DISPLAY_TIME		50
// Clock polarity:
// 0 means data set when high-to-low, read when low-to-high
// 1 means data set when low-to-high, read when high-to-low
#define FRAMEBUFFER_CLOCK_POLARITY		1

//Gamma & color correction
#define COLORCORR_GAMMA_MIN				1.0
#define COLORCORR_GAMMA_STEP			0.05
#define COLORCORR_GAMMA_COUNT			40
#define COLORCORR_GAMMA_DEFAULT			10

//Output
//GPIO for data and control
#define GPIO_DATA			GPIOD
#define GPIO_DATA_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, x)

#define GPIO_CONTROL		GPIOE
#define GPIO_CONTROL_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, x)
//Pins to use on control GPIO
#define GPIO_CONTROL_RowShift	2 // 2,3,4,5 for 16 scanrows, 2,3,4 for 8
#define GPIO_Pin_STB			GPIO_Pin_6

//Calculated, don't change this!
#define MATRIX_PANEL_STACKED	(MATRIX_PANEL_HEIGHT / (MATRIX_PANEL_BUSES * MATRIX_PANEL_SCANROWS))
#define MATRIX_PANEL_SEGMENT_W	(MATRIX_PANEL_WIDTH / MATRIX_PANEL_SEGMENTS)
#define MATRIX_WIDTH		(MATRIX_PANEL_WIDTH*MATRIX_PANELSW)
#define MATRIX_HEIGHT		(MATRIX_PANEL_HEIGHT*MATRIX_PANELSH)

#define CONTROL_UART_SPEED		115200
#endif /* CONFIG_H_ */
