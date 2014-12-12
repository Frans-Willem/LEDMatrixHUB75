/*
 * matrix.c
 *
 *  Created on: 12 dec. 2014
 *      Author: Frans-Willem
 */


#include "matrix.h"
#include "framebuffer.h"
#include "config.h"
#include "stm32f4xx.h"

#define MIN_TIMER_COUNT	150

void matrix_init_timer() {
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	static TIM_OCInitTypeDef TIM_OCStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//Output compare

	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCStructure);
	TIM_OCStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStructure.TIM_Pulse = MIN_TIMER_COUNT/64;
	TIM_OC1Init(TIM3, &TIM_OCStructure);
	TIM_OCStructure.TIM_Pulse = MIN_TIMER_COUNT;
	TIM_OC2Init(TIM3, &TIM_OCStructure);

	//Set and enable interrupt
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

unsigned int matrix_row;

void matrix_next() {
	//Actually show current data
	//Strobe up
	GPIO_CONTROL->BSRRL = GPIO_Pin_STB;
	//Update curdata pointer
	DMA2_Stream5->M0AR += FRAMEBUFFER_ROWLEN * sizeof(FRAMEBUFFER_TYPE);
	//Set row, keep strobe up and output disabled
	GPIO_CONTROL->ODR = (matrix_row << GPIO_CONTROL_RowShift) | GPIO_Pin_STB;
	//Load the timer prescaler
	TIM3->EGR = TIM_PSCReloadMode_Immediate;

	//Prepare data for next row.
	matrix_row++;
	if (matrix_row == MATRIX_PANEL_SCANROWS) {
		matrix_row = 0;
		if (TIM3->PSC == 0) {
			//Reset completely.
			DMA2_Stream5->M0AR = (uint32_t)framebuffer_get();
			//Adjust the prescaler to it's maximum value, but don't actually reload it yet.
			TIM3->PSC = (1 << FRAMEBUFFER_MAXBITDEPTH)-1;
		}
		TIM3->PSC >>= 1;
	}
	GPIOD->ODR = matrix_row << 12;

	//Strobe down (deliberately quite far away from strobe up, give panels some time to respond)
	GPIO_CONTROL->BSRRH = GPIO_Pin_STB;
	//Actually display what is already in the buffer
	//Output is enabled over this loop, seeing as this will always take the same amount of time.
	TIM3->CNT = 0; //Set counter to 0
	//Clear any flags
	DMA2->LIFCR = 0b111101;
	//Enable DMA and Timer
	TIM3->CR1 |= TIM_CR1_CEN;
	DMA2_Stream5->CR |= DMA_SxCR_EN;
}

void matrix_start() {
	//Set up variables as if last data was just clocked in, matrix_next will actually fix things up for us.
	matrix_row = MATRIX_PANEL_SCANROWS - 1;
	TIM3->PSC = 0;
	matrix_next();
}

/* Private functions ---------------------------------------------------------*/

void matrix_init_data_dma() {
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	static DMA_InitTypeDef DMA_InitStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static GPIO_InitTypeDef GPIO_InitStructure;

	//Set GPIO
	GPIO_DATA_CLOCKCMD(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIO_DATA, &GPIO_InitStructure);
	GPIOD->ODR=0;

	//Clear flags
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TCIF5);

	/* DMA2 Stream0 disable */
	DMA_Cmd(DMA2_Stream5, DISABLE);

	/* DMA2 Stream3  or Stream6 Config */
	DMA_DeInit(DMA2_Stream5);

	DMA_InitStructure.DMA_Channel = DMA_Channel_6; //Stream5, channel6 is TIM1_TRIG
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIO_DATA->ODR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = FRAMEBUFFER_ROWLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	if (sizeof(FRAMEBUFFER_TYPE) == 1) {
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	} else if (sizeof(FRAMEBUFFER_TYPE) == 2) {
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	} else if (sizeof(FRAMEBUFFER_TYPE) == 4) {
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
	}
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	DMA_FlowControllerConfig(DMA2_Stream5, DMA_FlowCtrl_Memory);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM1, ENABLE);

	//Set and enable interrupt
	TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);

	//Set and enable interrupt
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_IRQHandler() {
	//Disable timer
	TIM3->CR1 &= ~TIM_CR1_CEN;
	//Clear interrupt flags (maybe delay this?)
	TIM3->SR &= ~TIM_IT_CC2;
	if (!(DMA2_Stream5->CR & DMA_SxCR_EN))
		matrix_next();
	return;
}

void DMA2_Stream5_IRQHandler() {
	//Disable DMA (is this even needed?) (apparently not)
	//DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	//Clear interrupt flags (maybe delay this?)
	DMA2->HIFCR = DMA_HIFCR_CTCIF5;
	//If the timer has already stopped running, queue up the next row.
	if (!(TIM3->CR1 & TIM_CR1_CEN))
		matrix_next();
	return;
}

//Initializes all peripherals to drive the LED Matrix
void matrix_init() {
	static GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_CONTROL_CLOCKCMD(ENABLE);

	//Pinmode
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	//For control, only row and control pins
	GPIO_InitStructure.GPIO_Pin = ((MATRIX_PANEL_SCANROWS-1)<<GPIO_CONTROL_RowShift)|GPIO_Pin_STB;
	GPIO_Init(GPIO_CONTROL, &GPIO_InitStructure);

	matrix_init_timer();
	matrix_init_data_dma();
	matrix_start();
}
