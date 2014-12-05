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

/*
 * General options
 */
//Number of screens, 2 are supported with one GPIO output port, 5 with two.
#define MATRIX_CHANNELS	1
//Number of columns
#define MATRIX_COLS		(32*9)
//Number of scan rows in the matrix, 8 for 32x16 PH-10 matrices, 16 for adafruit's 32x32
#define MATRIX_SCANROWS	8
//Bit depth, decrease this to reduce flicker, increase this for more colors.
#define MATRIX_BITDEPTH	10
//Number of buffers, not actually used
#define BUFFERS			2
//Gamma correction, will be removed later.
#define GAMMACORRECTION
/*
 * Output options
 */
//Control GPIO, should be different from data GPIOs
#define GPIO_DATA			GPIOD
#define GPIO_DATA_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, x)
#define GPIO_Pin_Clock	GPIO_Pin_6
#define GPIO_CONTROL		GPIOC
#define GPIO_CONTROL_CLOCKCMD(x) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, x)
//Pins to use on control GPIO
#define GPIO_CONTROL_RowShift	0 // 0-3
#define GPIO_Pin_STB			GPIO_Pin_7

/*
 * End of options
 */


//The time for one inner column loop to complete.
#define MIN_TIMER_COUNT 150


#define MATRIX_ROWS (MATRIX_SCANROWS * MATRIX_CHANNELS)

#define IMGTYPE	uint8_t

/*
 * 32 columns (each containing 12 RGB values)
 * 8 rows
 * 8 bits
 */
#define IMAGEROWLEN	MATRIX_COLS * 2
#define IMAGEBITLEN	IMAGEROWLEN * MATRIX_SCANROWS
#define IMAGELEN	IMAGEBITLEN * MATRIX_BITDEPTH
IMGTYPE imageBuffers[(IMAGELEN*BUFFERS)]={0};
volatile unsigned int nDisplayingBuffer=0;
volatile unsigned int nDisplayBuffer=0; //Put this in a shared register so the main thread won't have to read memory all the time.
volatile unsigned int nWriteBuffer=1;
#ifdef GAMMACORRECTION
uint16_t gammatable[256];

float lum2duty(double lum) {
	if (lum>0.07999591993063804) {
		return pow(((lum+0.16)/1.16),3.0);
	} else {
		return lum/9.033;
	}
}


void init_gamma_table() {
	unsigned int i;
	for (i=0; i<256; i++) {
		float fin = i / 255.0f;
		float fout = lum2duty(fin);
		uint16_t out = fout * ((float)((1<<MATRIX_BITDEPTH)-1));
		gammatable[i] = out;
	}
}
#endif

#define IMAGE_WIDTH	(32*3)
#define IMAGE_HEIGHT (16*3)

//TODO: Optimize this!
void setpixel(unsigned int x, unsigned int y, uint16_t r, uint16_t g, uint16_t b) {
	uint16_t wr,wg,wb;
#ifdef GAMMACORRECTION
	wr=gammatable[r&255];
	wg=gammatable[g&255];
	wb=gammatable[b&255];
#else
	wr=r;
	wg=g;
	wb=b;
#endif
	if (y >= 16 && y<32) {
		x = (IMAGE_WIDTH - 1)-x;
		y = (IMAGE_HEIGHT - 1)-y;
	}

	unsigned int invy = (IMAGE_HEIGHT - 1)-y;
	unsigned int col = x + ((invy / (MATRIX_SCANROWS * 2)) * IMAGE_WIDTH);
	unsigned int channel = (y / MATRIX_SCANROWS) & 1;
	unsigned int row = y % MATRIX_SCANROWS;

	unsigned int rowshift = channel * 3;
	unsigned int rowclear = ~(7 << rowshift);
	unsigned int bit,bitmask=1<<MATRIX_BITDEPTH;
	for (bit=0; bit<MATRIX_BITDEPTH; bit++) {
		bitmask>>=1;
		IMGTYPE *curdata = &imageBuffers[(col*2) + (row * IMAGEROWLEN) + (bit  * IMAGEBITLEN)+(nWriteBuffer*IMAGELEN)];
		IMGTYPE rgb=0;
		//Set blue bit
		//rgb<<=1;
		if (wr & bitmask) rgb |= 1;
		if (wb & bitmask) rgb |= 2;
		if (wg & bitmask) rgb |= 4;
		rgb<<=rowshift;
		curdata[0] = (curdata[0] & rowclear) | rgb;
		curdata[1] = curdata[0] | GPIO_Pin_Clock;
	}
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void create_image() {
	unsigned int i, row, col;
	//Reset image
	for (i=0; i<IMAGELEN*BUFFERS; i+=2) {
		imageBuffers[i]=0;
		imageBuffers[i+1]=GPIO_Pin_Clock;
	}
	//Draw lines
	for (row=0; row<IMAGE_HEIGHT; row++) {
		for (col=0; col<IMAGE_WIDTH/2; col++) {
			setpixel((IMAGE_WIDTH/2)-1-col,row,(row*256)/(IMAGE_WIDTH/2),(col*256)/IMAGE_HEIGHT,0);
			setpixel((IMAGE_WIDTH/2)+col,row,(row*256)/(IMAGE_WIDTH/2),0,(col*256)/IMAGE_HEIGHT);
			//setpixel(col,row,255,255,255);
		}
	}
	for (i=0; i<MIN(IMAGE_HEIGHT,IMAGE_WIDTH); i++) {
		setpixel(i,i,0,0,255);
		setpixel((IMAGE_WIDTH-1)-i,i,0,255,0);
		setpixel((IMAGE_WIDTH-1)-i,(IMAGE_HEIGHT-1)-i,255,0,0);
		setpixel(i,(IMAGE_HEIGHT-1)-i,255,0,0);
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

	//Set and enable interrupt
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

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
	TIM3->SR &= ~TIM_IT_CC1;
	if (!(DMA2_Stream5->CR & DMA_SxCR_EN))
		matrix_next();
	return;
}

volatile uint8_t matrix_ready;
unsigned int matrix_row;

void matrix_next() {
	//Actually show current data
	//Strobe up
	//GPIOE->BSRRH = GPIO_Pin_15;
	GPIO_CONTROL->BSRRL = GPIO_Pin_STB;
	//Update curdata pointer
	DMA2_Stream5->M0AR += IMAGEROWLEN * sizeof(IMGTYPE);
	//Set row, keep strobe up and output disabled
	GPIO_CONTROL->ODR = (matrix_row << GPIO_CONTROL_RowShift) | GPIO_Pin_STB;
	//Load the timer prescaler
	TIM3->EGR = TIM_PSCReloadMode_Immediate;

	//Prepare data for next row.
	matrix_row++;
	if (matrix_row == MATRIX_SCANROWS) {
		matrix_row = 0;
		if (TIM3->PSC == 0) {
			//Reset completely.
			DMA2_Stream5->M0AR = &imageBuffers[IMAGELEN*nDisplayBuffer];
			//Adjust the prescaler to it's maximum value, but don't actually reload it yet.
			TIM3->PSC = (1 << MATRIX_BITDEPTH)-1;
			nDisplayingBuffer = nDisplayBuffer;
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
	GPIO_InitStructure.GPIO_Pin = ((MATRIX_SCANROWS-1)<<GPIO_CONTROL_RowShift)|GPIO_Pin_STB;
	GPIO_Init(GPIO_CONTROL, &GPIO_InitStructure);
}

void matrix_start() {
	//Set up variables as if last data was just clocked in, matrix_next will actually fix things up for us.
	matrix_row = MATRIX_SCANROWS - 1;
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
	DMA_InitStructure.DMA_BufferSize = IMAGEROWLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
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

#define UART_BUFFER_SIZE	((96*48*3)+10)
volatile uint16_t volatile uartbuffer[UART_BUFFER_SIZE];
unsigned int uartindex=0;

void init_uart() {
	static GPIO_InitTypeDef GPIO_InitStructure;
	static USART_InitTypeDef USART_InitStructure;
	static DMA_InitTypeDef DMA_InitStructure;

	unsigned int i;
	for (i=0; i<UART_BUFFER_SIZE; i++)
		uartbuffer[i]=-1;

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

	USART_InitStructure.USART_BaudRate = 5000000;
	//230400 -> 126702
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
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartbuffer;
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

uint16_t read_uart() {
	while (DMA2_Stream2->NDTR == UART_BUFFER_SIZE - uartindex || uartbuffer[uartindex]&0x8000);
	uint16_t cur = uartbuffer[uartindex];
	uartbuffer[uartindex]=-1;
	if (uartindex == UART_BUFFER_SIZE-1) uartindex=0;
	else uartindex++;
	return cur;
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
	nDisplayBuffer=0;
	nWriteBuffer=nDisplayBuffer;
	init_dma();
	init_timer();
	init_matrix();
	init_uart();

	unsigned int i=0;
	for (i=0; i<IMAGELEN*BUFFERS; i+=2) {
		imageBuffers[i]=0;
		imageBuffers[i]=GPIO_Pin_Clock;
	}

#ifdef GAMMACORRECTION
	init_gamma_table();
#endif
	create_image();
	nWriteBuffer=1;
	//0 = R1
	//1 = G1
	//2 = B1 -> G1
	//3 = R2 OK
	//4 = G2 -> B2
	//5 = B2 -> G2
	//for (i=0; i<IMAGELEN; i++)

	matrix_start();
	uint8_t type=0;
	uint16_t len=0;

	while (1) {
		//Wait for start byte?
		while (read_uart() != 0xC9);
		type = read_uart();
		len = read_uart();
		len <<= 8;
		len |= read_uart();

		if (type == 0xDA && len == IMAGE_WIDTH*IMAGE_HEIGHT*3) {
			//Wait for write buffer to be unused
			while (nDisplayingBuffer == nWriteBuffer);
			unsigned int x=0;
			unsigned int y=0;
			uint8_t r,g,b;
			while (len>=3) {
				r=read_uart();
				g=read_uart();
				b=read_uart();
				len-=3;
				if (y<IMAGE_HEIGHT) setpixel(x,y,r,g,b);
				x++;
				if (x >= IMAGE_WIDTH) {
					x=0;
					y++;
				}
			}
			nDisplayBuffer=nWriteBuffer;
			nWriteBuffer=(nWriteBuffer + 1)%BUFFERS;
			while (len--) read_uart();
		}
		//Skip until end byte
		while (read_uart() != 0x36);
	}
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
