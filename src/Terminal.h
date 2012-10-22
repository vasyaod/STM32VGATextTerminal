/*
 * STM32VGATextTerminal
 *
 * Created on: 21.10.2012
 *     Author: vasyaod (vasyaod@mail.ru)
 *       Link: https://github.com/vasyaod/STM32VGATextTerminal
 */
#ifndef __TERMINAL_H
#define __TERMINAL_H

#include <stdint.h>

void TerminalPutchar(uint8_t data);
void TerminalPrintf(char *str);
void TerminalInit();

#endif   // TERMINAL_H
