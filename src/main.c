/**
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include <stdlib.h>

#include "AsciiLib.h"
#include "VGA.h"

#define Bank1_SRAM1_ADDR    ((uint32_t)0x60000000)

uint8_t screenBufferPositionX = 0;
uint8_t screenBufferPositionY = 0;

uint8_t resolutionX;
uint8_t resolutionY;

int SysTickDelay;

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
	USART_InitStructure.USART_BaudRate = 115200;
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
			VGAScreenBuffer[screenBufferPositionX+screenBufferPositionY*resolutionX] = data-32;
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


int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin = (GPIO_Pin_0| GPIO_Pin_1);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB , &PORT);

	SysTick_Config(SystemCoreClock /1000);//1ms

	initUART();
	VGAInit();
	resolutionX = VGAParams.textResolutionX;
	resolutionY = VGAParams.textResolutionY;

	GPIOB->ODR ^= GPIO_Pin_0;	// Посветим индикатором в знак того, что все успешно инициалазировалось.

	while(1)
	{
		VGARender();
		__NOP();

		//		Delay(100);
//		GPIOB->ODR ^= GPIO_Pin_0;
//		if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET) {
//			GPIOB->ODR ^= GPIO_Pin_1;
//			USART_SendData(USART1, 'v');
//		}
	}
}
