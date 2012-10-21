/*
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */
#include <stdlib.h>

#include "Terminal.h"
#include "VGA.h"

uint8_t screenBufferPositionX = 0;
uint8_t screenBufferPositionY = 0;

uint8_t resolutionX;
uint8_t resolutionY;

void TerminalPutchar(uint8_t data)
{
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
		screenBufferPositionY++;
		screenBufferPositionX = 0;		// Вернем корретку
	}
	else if(data == 0x0D)
	{
		screenBufferPositionY++;
		screenBufferPositionX = 0;		// Вернем корретку
		// При возврате корретки ничего не делаем.

		// Что то вики, как то припездывает....
	}

	if (screenBufferPositionX >= resolutionX)
		screenBufferPositionX = resolutionX-1;
	if (screenBufferPositionY >= resolutionY)
		screenBufferPositionY = resolutionY-1;
}

void TerminalPrintf(char *str)
{
	char ch;
	do {
		ch = *(str++);
		TerminalPutchar(ch);
	} while (ch != 0x00);
}
