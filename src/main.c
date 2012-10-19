/**
 * Link: difur.ru
 * Author: vasyaod (vasyaod@mail.ru)
 */
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include "AsciiLib.h"

#define Bank1_SRAM1_ADDR    ((uint32_t)0x60000000)

void writeBuffer();

uint8_t resolutionX = 80;
uint8_t resolutionY = 30;

#define bufferSize 81
uint8_t *lineBuffer1;
uint8_t *lineBuffer2;

uint8_t *screenBuffer;

uint32_t currentBuffer;
uint32_t tempBuffer;

int16_t screenLineCount = 0;
int16_t lineBySymbolCount = 0;
int16_t symbolLineCount = 0;
uint32_t writeBufferFlag = 0;

uint8_t screenBufferPositionX= 0;
uint8_t screenBufferPositionY= 0;

int SysTickDelay;

caddr_t _sbrk_r ( struct _reent *ptr, int incr )
{
  extern int _end;
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;
  /* check removed to show basic approach */
  heap += incr;

  return (caddr_t) prev_heap;
}


//------------------------------------------------------------------------
void Delay( unsigned int Val)
{
   SysTickDelay = Val;
   while (SysTickDelay != 0) {};
}

//------------------------------------------------------------------------
void SysTick_Handler(void)
{
	if (SysTickDelay != 0)
	{
		SysTickDelay--;
	}
}

void initPWM(void)
{
	int i = 2;
	// Включает тактирование порта для вывода синхро сигналов.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

	// Инициализируем порт для вывода синхро сигналов.
	GPIO_InitTypeDef PORTC;
	PORTC.GPIO_Pin = (GPIO_Pin_0);
	PORTC.GPIO_Mode = GPIO_Mode_Out_PP;
	PORTC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOC , &PORTC);

	// Включает тактирование порта для вывода синхро сигналов.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

	// Инициализируем порт для вывода синхро сигналов.
	GPIO_InitTypeDef PORTA;
	PORTA.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 |	// Выходы таймера TIM2... Каналы 1 и 2
	                  GPIO_Pin_2 | GPIO_Pin_3);  // Выходы таймера TIM5... Каналы 3 и 4
	PORTA.GPIO_Mode = GPIO_Mode_AF_PP;
	PORTA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA , &PORTA);

	// Включаем тактирование таймера TIM2. Отвечает за VSYNC.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// Включаем тактирование таймера TIM3. Отвечает за HSYNC.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//////
	// Инициализиция TIM2 и ШИМ на первом канале.
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
//    base_timer.TIM_Prescaler = SystemCoreClock / 1000000 - 1;

	int zh = 60;
	int vRes = 525;
	int hRes = 800;

	double  factopr = SystemCoreClock/(double)(zh*vRes*hRes);

	base_timer.TIM_Period = (hRes * factopr);
//	base_timer.TIM_Period = SystemCoreClock/4*800;
	TIM_TimeBaseInit(TIM5, &base_timer);

	// Конфигурирование каналов.
	TIM_OCInitTypeDef timer_oc;
	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = 96 * factopr;
	timer_oc.TIM_OCMode = TIM_OCMode_PWM1;
	timer_oc.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM5, &timer_oc);

	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = 135 * factopr;
	timer_oc.TIM_OCMode = TIM_OCMode_Active;
	TIM_OC4Init(TIM5, &timer_oc);

	TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
	// Выбираем вход триггера от TIM2 (ITR0)
	TIM_SelectInputTrigger(TIM5, TIM_TS_ITR0);

	/* Включаем прерывание переполнения счётчика */
	TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC4, ENABLE);
	NVIC_SetPriority(TIM5_IRQn, 15);
	NVIC_EnableIRQ(TIM5_IRQn);

//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	//////
	// Инициализиция TIM5 и ШИМ на первом канале.
	TIM_TimeBaseStructInit(&base_timer);
	base_timer.TIM_Prescaler = (uint16_t)(SystemCoreClock/(double)(zh*vRes*10))-1;
	base_timer.TIM_Period = 10*vRes;
	TIM_TimeBaseInit(TIM2, &base_timer);

	// Конфигурирование каналов.
	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = 10*2 ;
	timer_oc.TIM_OCMode = TIM_OCMode_PWM1;
	timer_oc.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM2, &timer_oc);

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM2, ENABLE);


}
/*
void initFSMC() {

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                               GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                              GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// PD.00(D2), PD.01(D3), PD.04(RD), PD.5(WR), PD.7(CS), PD.8(D13), PD.9(D14),
	//   PD.10(D15), PD.11(RS) PD.14(D0) PD.15(D1)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 |
	                               GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
	                               GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Включим тактирование FSMC
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMReadWriteTimingInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMWriteTimingStructure;

	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMReadWriteTimingInitStructure;
//	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMWriteTimingStructure;

//	FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);

	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AddressSetupTime = 0;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AddressHoldTime = 0;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_DataSetupTime = 1;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMReadWriteTimingInitStructure;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	// Enable FSMC Bank1_SRAM Bank
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);

	// Включим тактирование DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;
//	DMA_StructInit(&DMA_InitStructure);
	DMA_DeInit(DMA1_Channel1);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SRC_Const_Buffer;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(GPIOC->ODR);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)Bank1_SRAM1_ADDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Bank1_SRAM1_ADDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	// Enable DMA2 Channel5 Transfer Complete interrupt
//	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_TE, ENABLE);
//	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA_Cmd(DMA1_Channel1, ENABLE);
//	while(!DMA_GetFlagStatus(DMA1_FLAG_TC1)) {
//		uint16_t t = DMA_GetCurrDataCounter(DMA1_Channel1);
//		GPIOC->ODR ^= GPIO_Pin_0;
//	};
}
//  */

void initSPI()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel3);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)lineBuffer1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = bufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

//	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
//	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
//	DMA_Cmd(DMA1_Channel3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
}

void initUART()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Enable USARTy Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

//	USART_ClockInitTypeDef USART_ClockInitStructure;
//	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
//	USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
//	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
//	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
//	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// Configure the USARTy
	USART_Init(USART1, &USART_InitStructure);

	// Configure interrupted the USARTy
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);

	// Enable the USARTy
	USART_Cmd(USART1, ENABLE);

}
void USART1_IRQHandler() {
	GPIOB->ODR ^= GPIO_Pin_1;
	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {

		// Читаем принятые данные и сбрасываем флаг.
		uint8_t data = USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_RXNE);

		if (data >= 32 &&  data <= 94+32)
		{
			screenBuffer[screenBufferPositionX+screenBufferPositionY*resolutionX] = data-32;
			screenBufferPositionX++;
		}

		// Что то вики, как то припездывает....
		// LF (ASCII 0x0A) - перевод строки (xUNIX);
		// CR (0x0D) - возврат каретки;
		// CR+LF (ASCII 0x0D 0x0A) - windows, dos ...
		else if(data == 0x0A)
		{
		//	screenBufferPositionY++;
		//	screenBufferPositionX = 0;		// Вернем корретку
		}
		else if(data == 0x0D)
		{
			screenBufferPositionY++;
			screenBufferPositionX = 0;		// Вернем корретку
			// При возврате корретки ничего не делаем.

			// Что то вики, как то припездывает....
		}

		if (screenBufferPositionX >= 80)
			screenBufferPositionX = 79;
		if (screenBufferPositionY >= 30)
			screenBufferPositionY = 29;
	}
}

void DMA1_Channel3_IRQHandler() {
	if (DMA_GetFlagStatus(DMA1_FLAG_TC3) != RESET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC3);
		//		writeBufferFlag = 1;
	//	DMA_Cmd(DMA1_Channel3, DISABLE);
	//	GPIOC->ODR ^= GPIO_Pin_0;
	}

	if (DMA_GetFlagStatus(DMA1_FLAG_TE3) != RESET)
	{
		DMA_ClearFlag(DMA1_FLAG_TE3);
		//DMA_Cmd(DMA1_Channel3, DISABLE);
	//	GPIOC->ODR ^= GPIO_Pin_0;
	}
}

void TIM5_IRQHandler()
{
	//	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
	if ((TIM5->SR & TIM_IT_CC4) != 0)
	{
//		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
		TIM5->SR = (uint16_t)~TIM_IT_CC4;
//		DMA_Cmd(DMA1_Channel3, DISABLE);
//		DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);
//		DMA_SetCurrDataCounter(DMA1_Channel3, 81);
//		DMA1_Channel3->CNDTR = 81;
//		DMA_Cmd(DMA1_Channel3, ENABLE);
		DMA1_Channel3->CCR |= DMA_CCR1_EN;
	}

//	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	else if ((TIM5->SR & TIM_IT_Update) != 0)
	{
		// Даём знать, что обработали прерывание
//		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		TIM5->SR = (uint16_t)~TIM_IT_Update;


		if (currentBuffer == (uint32_t)lineBuffer1)
		{
			currentBuffer = (uint32_t)lineBuffer2;
			tempBuffer = (uint32_t)lineBuffer1;
		}
		else
		{
			currentBuffer = (uint32_t)lineBuffer1;
			tempBuffer = (uint32_t)lineBuffer2;
		}

		screenLineCount++;
		if (screenLineCount > 35 && screenLineCount <= 480+35) {

			if (lineBySymbolCount == 16) {
				symbolLineCount ++;
				lineBySymbolCount = 0;
			}
			lineBySymbolCount ++;

	//		DMA_Cmd(DMA1_Channel3, DISABLE);
			DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);     // Выключаем DMA.
	//		DMA_SetCurrDataCounter(DMA1_Channel3, 21);
			DMA1_Channel3->CNDTR = bufferSize;                  // Устанавливаем новое значение счетчика данных DMA.
			DMA1_Channel3->CMAR = (uint32_t)currentBuffer;      // Передаем адрес нового буффера.

			writeBufferFlag = 1;
		}

	}
}

void TIM2_IRQHandler()
{
//	if ((TIM2->SR & TIM_IT_Update) != 0)
//	{
		TIM2->SR = (uint16_t)~TIM_IT_Update;
		screenLineCount = 0;
		lineBySymbolCount = 0;
		symbolLineCount = 0;
//	}
}

void writeBuffer()
{
	int i;
	uint16_t tmpLineCount = lineBySymbolCount-1;

	for (i = 0; i< resolutionX; i++) {
//		((unsigned char *)tempBuffer)[i] = AsciiLib[i][tmpLineCount];
		uint8_t c = screenBuffer[i+symbolLineCount*resolutionX];
		((unsigned char *)tempBuffer)[i] = AsciiLib[c][tmpLineCount];
	}
}

/**
 * Процедура выделяет память под буфферы и инициализирует их.
 * Обратите внимание, что под строковые буфферы выделяется resolutionX+1 байт
 * памяти и последний байт установлен в 0x00. Это сделано для того, что бы
 * "заглушить" передачу сигнала по SPI.
 */
void initBuffers()
{
	// Выделяем память под строчные буффера.
	lineBuffer1 = malloc(resolutionX+1);
	lineBuffer2 = malloc(resolutionX+1);
	// Последный байт устанавливаем в 0x00.
	lineBuffer1[resolutionX] = 0x00;
	lineBuffer2[resolutionX] = 0x00;
	currentBuffer = (uint32_t)lineBuffer1;
	tempBuffer = (uint32_t)lineBuffer2;

	// Выделяем память под экранный буффер.
	screenBuffer = (uint8_t*)malloc(resolutionX*resolutionY);
	int i;
	// Заполним буффер начальными данными.
	for (i = 0; i< resolutionX*resolutionY; i++) {
		screenBuffer[i] = 1;
	}
}

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin = (GPIO_Pin_0| GPIO_Pin_1);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB , &PORT);

	SysTick_Config(SystemCoreClock /1000);//1ms
	initBuffers();

	initUART();
	initSPI();
	initPWM();

	GPIOB->ODR ^= GPIO_Pin_0;	// Посветим индикатором в знак того, что все успешно инициалазировалось.

	while(1)
	{
		if (writeBufferFlag == 1)
		{
			writeBuffer();
			writeBufferFlag = 0;
		}
//		Delay(100);
//		GPIOB->ODR ^= GPIO_Pin_0;
//		if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET) {
//			GPIOB->ODR ^= GPIO_Pin_1;
//			USART_SendData(USART1, 'v');
//		}
		__NOP();
	}
}
