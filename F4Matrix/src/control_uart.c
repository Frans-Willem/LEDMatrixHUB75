/*
 * uart_control.c
 *
 *  Created on: 12 dec. 2014
 *      Author: Frans-Willem
 */

#include "control_uart.h"
#include "colorcorr.h"
#include "framebuffer.h"
#include "config.h"
#include "matrix.h"
#include "stm32f4xx.h"

#define UART_BUFFER_SIZE	500
volatile uint16_t volatile control_uart_buffer[UART_BUFFER_SIZE];
unsigned int control_uart_index=0;


void control_uart_init() {
	static GPIO_InitTypeDef GPIO_InitStructure;
	static USART_InitTypeDef USART_InitStructure;
	static DMA_InitTypeDef DMA_InitStructure;

	unsigned int i;
	for (i=0; i<UART_BUFFER_SIZE; i++)
		control_uart_buffer[i]=-1;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);//TXD
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);//RXD

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 3000000;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = 8;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

	//Set GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//Clear flags
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);

	/* DMA2 Stream0 disable */
	DMA_Cmd(DMA2_Stream2, DISABLE);

	/* DMA2 Stream3  or Stream6 Config */
	DMA_DeInit(DMA2_Stream2);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4; //Stream2, channel4 is USART1_RX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)control_uart_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	DMA_FlowControllerConfig(DMA2_Stream2, DMA_FlowCtrl_Memory);
	DMA_Cmd(DMA2_Stream2, ENABLE);
}

uint16_t control_uart_read() {
	while (DMA2_Stream2->NDTR == UART_BUFFER_SIZE - control_uart_index || control_uart_buffer[control_uart_index]&0x8000);
	uint16_t cur = control_uart_buffer[control_uart_index];
	control_uart_buffer[control_uart_index]=-1;
	if (control_uart_index == UART_BUFFER_SIZE-1) control_uart_index=0;
	else control_uart_index++;
	return cur;
}

void control_uart_write(uint8_t ch)
{
  while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = ch;
}

#define CMD_SWAP		0
#define CMD_GAMMA		1
#define CMD_BRIGHTNESS	2

void control_uart_command(uint8_t cmd, uint8_t *data, unsigned int len) {
	switch (cmd) {
	case CMD_SWAP:
		framebuffer_swap();
		break;
	case CMD_GAMMA:
		colorcorr_select((len > 0)?data[0]:0);
		break;
	case CMD_BRIGHTNESS:
		matrix_setbrightness((len > 0)?data[0]:0);
		break;
	}
}

#define STATE_IDLE		0 //Nothing received yet
#define STATE_F			1 //F received
#define STATE_FW		2 //FW received
#define STATE_FWP		3 //FWP received
#define STATE_FWPL		4 //FWP_ received
#define STATE_FWPLL		5 //FWP__ received
#define STATE_FWPLLO	6 //FWP___ received
#define STATE_FWPLLOO	7 //FWP____ received
#define STATE_FWC	 	8 //FWC received
#define STATE_FWCX		9 //FWCX received

void control_uart_loop() {
	uint8_t state=STATE_IDLE;
	uint16_t len=0;
	uint16_t off=0;
	uint8_t cmd;
	uint8_t cmddata[32];

	while (1) {
		uint8_t c=control_uart_read();
		switch (state) {
		case STATE_IDLE:
			if (c == 'F')
				state = STATE_F;
			break;
		case STATE_F:
			if (c == 'W')
				state = STATE_FW;
			else if (c == 'F')
				state = STATE_F;
			else
				state = STATE_IDLE;
			break;
		case STATE_FW:
			if (c == 'P')
				state = STATE_FWP;
			else if (c == 'C')
				state = STATE_FWC;
			else if (c == 'F')
				state = STATE_F;
			else
				state = STATE_IDLE;
			break;
		case STATE_FWP:
			len = ((uint16_t)c) << 8;
			state = STATE_FWPL;
			break;
		case STATE_FWPL:
			len |= c;
			state = STATE_FWPLL;
			break;
		case STATE_FWPLL:
			off = ((uint16_t)c) << 8;
			state = STATE_FWPLLO;
			break;
		case STATE_FWPLLO:
			off |= c;
			if (len == 0)
				state = STATE_IDLE;
			else
				state = STATE_FWPLLOO;
			break;
		case STATE_FWPLLOO:
			framebuffer_write(off,colorcorr_lookup(c));
			len--;
			off++;
			if (len == 0) {
				state=STATE_IDLE;
				control_uart_write('O');
				control_uart_write('K');
			}
			break;
		case STATE_FWC:
			len = c & 0x1F;
			off = 0;
			cmd = c >> 5;
			if (len == 0) {
				state = STATE_IDLE;
				control_uart_command(cmd, cmddata, len);
        control_uart_write('O');
        control_uart_write('K');
			} else {
				state = STATE_FWCX;
			}
			break;
		case STATE_FWCX:
			cmddata[off] = c;
			off++;
			if (off == len) {
				state = STATE_IDLE;
				control_uart_command(cmd, cmddata, len);
        control_uart_write('O');
        control_uart_write('K');
			}
			break;
		}
	}
}
