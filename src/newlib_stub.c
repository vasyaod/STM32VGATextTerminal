/**
 * STM32VGATextTerminal
 * 
 * Link: https://github.com/vasyaod/STM32VGATextTerminal
 * Author: vasyaod (vasyaod@mail.ru)
 */

#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

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
