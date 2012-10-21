/*
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */

#include <stdlib.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_spi.h>

#include "AsciiLib.h"
#include "VGA.h"

uint8_t *lineBuffer1;
uint8_t *lineBuffer2;

uint32_t currentBuffer;
uint32_t tempBuffer;

int16_t screenLineCount = 0;
int16_t lineBySymbolCount = 0;
int16_t symbolLineCount = 0;
uint32_t writeBufferFlag = 0;

VGAParamsType VGAParams = {
	800,    // Graphical resolution of X axis
	525,    // Graphical resolution of Y axis
	60,     // Frequency
	96,     // hSync pulse
	2,      // vSync pulse
	135,    // Back porch
	AsciiLib,
	80,     // Text resolution of X axis
	30      // Text resolution of Y axis
};
uint8_t *VGAScreenBuffer;

/**
 * Функция инициализирует таймеры на которых реализован ШИМ для построчного и
 * покадрового синхросгнала.
 */
void initPWM(void)
{
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

	int zh = VGAParams.frequency;
	int vRes = VGAParams.resolutionY;
	int hRes = VGAParams.resolutionX;

	double factopr = SystemCoreClock/(double)(zh*vRes*hRes);

	base_timer.TIM_Period = (hRes * factopr);
//	base_timer.TIM_Period = SystemCoreClock/4*800;
	TIM_TimeBaseInit(TIM5, &base_timer);

	// Конфигурирование каналов.
	TIM_OCInitTypeDef timer_oc;
	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = VGAParams.hSyncPulse * factopr;
	timer_oc.TIM_OCMode = TIM_OCMode_PWM1;
	timer_oc.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM5, &timer_oc);

	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = VGAParams.hBackPorch * factopr;
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
	timer_oc.TIM_Pulse = 10*VGAParams.vSyncPulse ;
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

/**
 * Функция инициализирует SPI для вывода изображения на экран.
 */
void initSPI()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel3);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)lineBuffer1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = VGAParams.textResolutionX+1;
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
			DMA1_Channel3->CNDTR = VGAParams.textResolutionX+1;               // Устанавливаем новое значение счетчика данных DMA.
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

/**
 * Процедура выделяет память под буфферы и инициализирует их.
 * Обратите внимание, что под строковые буфферы выделяется VGAParams.textResolutionX+1 байт
 * памяти и последний байт установлен в 0x00. Это сделано для того, что бы
 * "заглушить" передачу сигнала по SPI.
 */
void initBuffers()
{
	// Выделяем память под строчные буффера.
	lineBuffer1 = malloc(VGAParams.textResolutionX+1);
	lineBuffer2 = malloc(VGAParams.textResolutionX+1);
	// Последный байт устанавливаем в 0x00.
	lineBuffer1[VGAParams.textResolutionX] = 0x00;
	lineBuffer2[VGAParams.textResolutionX] = 0x00;
	currentBuffer = (uint32_t)lineBuffer1;
	tempBuffer = (uint32_t)lineBuffer2;

	// Выделяем память под экранный буффер.
	VGAScreenBuffer = (uint8_t*)malloc(VGAParams.textResolutionX*VGAParams.textResolutionY);
	int i;
	// Заполним буффер начальными данными.
	for (i = 0; i< VGAParams.textResolutionX*VGAParams.textResolutionY; i++) {
		VGAScreenBuffer[i] = 1;
	}
}

void VGARender()
{
	if (writeBufferFlag == 1)
	{
		int i;
		uint16_t tmpLineCount = lineBySymbolCount-1;

		for (i = 0; i< VGAParams.textResolutionX; i++) {
	//		((unsigned char *)tempBuffer)[i] = AsciiLib[i][tmpLineCount];
			uint8_t c = VGAScreenBuffer[i+symbolLineCount*VGAParams.textResolutionX];
	//		((unsigned char *)tempBuffer)[i] = (VGAParams.symbolTable)[c*16+tmpLineCount];
			((unsigned char *)tempBuffer)[i] = ((uint8_t*)AsciiLib)[c*16+tmpLineCount];
		}
		writeBufferFlag = 0;
	}
}

void VGAInit()
{
	// Выделяем под буфферы память и инициализируем их.
	initBuffers();

	initSPI();
	initPWM();
}

uint8_t *VGAGetScreenBuffer()
{
	return VGAScreenBuffer;
}
