/*
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */

#ifndef __VGA_H
#define __VGA_H

// Величины описанные в этой структуре можно поглядеть по ссылке
// http://tinyvga.com/vga-timing/640x480@60Hz
// или
//
typedef struct {
	uint16_t resolutionX;      // Разрешение экрана в пикселях по оси X,
	                           // по умолчанию равно 800, так как исеются еще и поля.

	uint16_t resolutionY;      // Разрешение экрана в пикселях по оси Y,
	                           // по умолчанию равно 525, так как исеются еще и поля.

	uint16_t frequency;        // Частота кадровой развертки.

	uint16_t hSyncPulse;       // Время импульса, синхросигнала строчной развертки,
	                           // время задается в пикселях, от начала синхросигнала.

	uint16_t vSyncPulse;       // Время импульса, синхросигнала кадровой развертки,
	                           // время задается в пикселях, от начала синхросигнала.

	uint16_t hBackPorch;       // Время вывода видимой области от начала синхросигнала строчной развертки
	                           // время задается в пикселях.

	uint8_t *symbolTable;     // Ссылка на талицу символов.

	uint8_t textResolutionX;   // Резмер текстового буфера по горизонтали.
	uint8_t textResolutionY;   // Резмер текстового буфера по вертикали.
} VGAParamsType;

extern VGAParamsType VGAParams;
extern uint8_t *VGAScreenBuffer;

void VGAInit();
void VGARender();

#endif
