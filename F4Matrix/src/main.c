/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "config.h"
#include "framebuffer.h"
#include "colorcorr.h"
#include "control_uart.h"

/*
 * General options
 */
/*
 * Output options
 */
//Control GPIO, should be different from data GPIOs
#define GPIO_DATA			GPIOD
#define GPIO_DATA_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, x)

#define GPIO_CONTROL		GPIOE
#define GPIO_CONTROL_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, x)
//Pins to use on control GPIO
#define GPIO_CONTROL_RowShift	2 // 0-3 for 16 scanrows, 0-2 for 8
#define GPIO_Pin_STB			GPIO_Pin_6

/*
 * End of options
 */


//The time for one inner column loop to complete.
#define MIN_TIMER_COUNT 150

//TODO: Optimize this!
void setpixel(unsigned int x, unsigned int y, uint16_t r, uint16_t g, uint16_t b) {
	unsigned int offset=(x + (y*MATRIX_WIDTH))*3;
	framebuffer_write(offset,colorcorr_lookup(r));
	framebuffer_write(offset+1,colorcorr_lookup(g));
	framebuffer_write(offset+2,colorcorr_lookup(b));
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void create_image() {
	unsigned int i, row, col;
	//Draw lines
	for (row=0; row<MATRIX_HEIGHT; row++) {
		for (col=0; col<MATRIX_WIDTH/2; col++) {
			setpixel((MATRIX_WIDTH/2)-1-col,row,(row*256)/(MATRIX_WIDTH/2),(col*256)/MATRIX_HEIGHT,0);
			setpixel((MATRIX_WIDTH/2)+col,row,(row*256)/(MATRIX_WIDTH/2),0,(col*256)/MATRIX_HEIGHT);
			//setpixel(col,row,255,255,255);
		}
	}
	for (i=0; i<MIN(MATRIX_HEIGHT,MATRIX_WIDTH); i++) {
		setpixel(i,i,0,0,255);
		setpixel((MATRIX_WIDTH-1)-i,i,0,255,0);
		setpixel((MATRIX_WIDTH-1)-i,(MATRIX_HEIGHT-1)-i,255,0,0);
		setpixel(i,(MATRIX_HEIGHT-1)-i,255,0,0);
		//setpixel((MATRIX_COLS/2)-1,i,255,0,0);
		//setpixel(MATRIX_COLS/2,i,0,255,0);
	}
	//setpixel(0,0,255,0,0);
}

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

void init_timer() {
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
	TIM_OCStructure.TIM_Pulse = MIN_TIMER_COUNT;
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

void matrix_next();

void TIM3_IRQHandler() {
	//Disable timer
	TIM3->CR1 &= ~TIM_CR1_CEN;
	//Clear interrupt flags (maybe delay this?)
	TIM3->SR &= ~TIM_IT_CC2;
	if (!(DMA2_Stream5->CR & DMA_SxCR_EN))
		matrix_next();
	return;
}

volatile uint8_t matrix_ready;
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

//Initializes all peripherals to drive the LED Matrix
void init_matrix() {
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
}

void matrix_start() {
	//Set up variables as if last data was just clocked in, matrix_next will actually fix things up for us.
	matrix_row = MATRIX_PANEL_SCANROWS - 1;
	TIM3->PSC = 0;
	matrix_next();
}

/* Private functions ---------------------------------------------------------*/

void init_dma() {
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


#define STATE_IDLE		0
#define STATE_TYPE		1
#define STATE_LENH		2
#define STATE_LENL		3
#define STATE_DATA		4
#define STATE_DONE		5

char temp[256];
int main(void)
{
	framebuffer_init();
	colorcorr_init();
	control_uart_init();
	init_dma();
	init_timer();
	init_matrix();

	create_image();
	framebuffer_swap();

	matrix_start();

	control_uart_loop();
	return 0;
}

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--) __asm("nop");
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
