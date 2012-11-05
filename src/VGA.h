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

/**
 * Инициализирует микроконтроллер для вывода изображения на экран.
 */
void VGAInit();
void VGARender();

/**
 * Функция выводит символ из таблицы символов (VGAParams.symbolTable) в указанную
 * позицию экрана.
 *
 * Нумерация координат начинается в 0.
 */
void VGAPrintChar(uint8_t x, uint8_t y, uint8_t ch);

/**
 * Заполняет весь экран указаннам символом из таблицы (VGAParams.symbolTable).
 */
void VGAFillScreen(uint8_t ch);

/**
 * Заполняет указанную строку символом из таблицы (VGAParams.symbolTable).
 */
void VGAFillRow(uint8_t y, uint8_t ch);

/**
 * Копирует область экрана из одних координат в другие.
 *
 * ВНИМАНИЕ!!! Функция реализована не правельно, это еще предстоит сделать (исправить).
 */
void VGAMoveRange(uint8_t width, uint8_t height,
                    uint8_t sourceX, uint8_t sourceY,
                    uint8_t destinationX, uint8_t destinationY);


#endif
