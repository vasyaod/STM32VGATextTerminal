/**
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <misc.h>

#include <stdlib.h>
#include <stdio.h>

#include "AsciiLib.h"
#include "VGA.h"
#include "Terminal.h"
#include "UART.h"

extern uint8_t resolutionX;
extern uint8_t resolutionY;

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin = (GPIO_Pin_0| GPIO_Pin_1);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB , &PORT);

	resolutionX = VGAParams.textResolutionX;
	resolutionY = VGAParams.textResolutionY;

	VGAInit();
	TerminalPrintf("VGA output inited.\n");

	TerminalInit();
	TerminalPrintf("Terminal emulation inited.\n");

	initUART();
	TerminalPrintf("UART inited.\n");

	GPIOB->ODR ^= GPIO_Pin_0;	// Посветим индикатором в знак того, что все успешно инициалазировалось.
	TerminalPrintf("Welcome to STR32 Text termital!\n");

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
